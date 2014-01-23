/****************************************************************************
 *   drivers/eeprom.c
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************** */

#include <stdint.h>

#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "lib/string.h"
#include "drivers/i2c.h"
#include "drivers/gpio.h"


/***************************************************************************** */
/*       EEPROM Chip select for the GPIO Demo module                           */
/*******************************************************************************/
/* These are place-holders to set the SPI chip select low if the SPI driver is not used.
 * These dummy functions will be over-ridden by SPI ones if the SPI driver is used.
 */
#define I2C_CS_PIN 15
int I2C_CS_Default_Set(void)
{
    struct lpc_gpio* gpio0 = LPC_GPIO_0;
    config_gpio(0, I2C_CS_PIN, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
    /* Configure SPI_CS as output and set it low. */
    gpio0->data_dir |= (1 << I2C_CS_PIN);
    gpio0->clear = (1 << I2C_CS_PIN);
	return 0;
}
void I2C_CS_Default_Release(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;
    gpio0->set = (1 << I2C_CS_PIN);
}

int spi_device_cs_pull_low(void) __attribute__ ((weak, alias ("I2C_CS_Default_Set")));
void spi_device_cs_release(void) __attribute__ ((weak, alias ("I2C_CS_Default_Release")));


/***************************************************************************** */
/*          Read and Write for module eeprom                                   */
/***************************************************************************** */
/* Config */
/* Small eeprom : up to 2k bytes. These use a segment address on the lower three bits
 *   of the address byte, and thus reply on 8 consecutive addresses */
#define EEPROM_ID_SMALL_ADDR  0xA0
#define EEPROM_ID_SMALL_I2C_SIZE  1024
#define EEPROM_ID_SMALL_PAGE_SIZE 16
/* Big eeprom : from 4k bytes and above : These use two address bytes, and the three
 *   physical address pins are used to set the chip address. On DTPlug modules they should
 *   have address 0xA8. */
#define EEPROM_ID_BIG_ADDR  0xA8
#define EEPROM_ID_BIG_I2C_SIZE  16*1024
#define EEPROM_ID_BIG_PAGE_SIZE 64



/* Detect the eeprom size */
int eeprom_detect(void)
{
	int ret = 0;
	char cmd_buf[1] = { EEPROM_ID_SMALL_ADDR, };

	/* Look for small eeproms first, only these would answer on EEPROM_ID_SMALL_ADDR */
	ret = i2c_read(cmd_buf, 1, NULL, NULL, 0);
	if (ret == 0) {
		return EEPROM_TYPE_SMALL;
	}
	/* No small eeprom ... look for big ones */
	cmd_buf[0] = EEPROM_ID_BIG_ADDR;
	ret = i2c_read(cmd_buf, 1, NULL, NULL, 0);
	if (ret == 0) {
		return EEPROM_TYPE_BIG;
	}

	if (ret > 0) {
		return -EIO;
	} else if (ret == -EREMOTEIO) {
		return EEPROM_TYPE_NONE; /* No module */
	}
	return ret; /* Error or module size */
}

int get_eeprom_type(void)
{
	static int eeprom_type = -1;

	if (eeprom_type >= 0) {
		return eeprom_type; /* No need to check again */
	}

	eeprom_type = eeprom_detect();
	if (eeprom_type <= 0) {
		return -1;
	}
	return eeprom_type;
}


/* EEPROM Read
 * Performs a non-blocking read on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EFAULT : address above eeprom size
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
#define CMD_BUF_SIZE 4
int eeprom_read(uint32_t offset, void *buf, size_t count)
{
	int ret = 0;
	char cmd_buf[CMD_BUF_SIZE] = { EEPROM_ID_BIG_ADDR, 0, 0, (EEPROM_ID_BIG_ADDR | 0x01), };
	char ctrl_buf[CMD_BUF_SIZE] = { I2C_CONT, I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	int eeprom_type = 0;

	if (spi_device_cs_pull_low() != 0) {
		return -EBUSY;
	}
	eeprom_type = get_eeprom_type();

	/* Read the requested data */
	switch (eeprom_type) {
		case EEPROM_TYPE_SMALL:
			cmd_buf[0] = EEPROM_ID_SMALL_ADDR | ((offset & 0x700) >> 7);
			cmd_buf[1] = offset & 0xFF;
			cmd_buf[2] = EEPROM_ID_SMALL_ADDR | 0x01;
			ret = i2c_read(cmd_buf, CMD_BUF_SIZE - 1, ctrl_buf + 1, buf, count);
			break;
		case EEPROM_TYPE_BIG:
			cmd_buf[1] = ((offset & 0xFF00) >> 8);
			cmd_buf[2] = offset & 0xFF;
			ret = i2c_read(cmd_buf, CMD_BUF_SIZE, ctrl_buf, buf, count);
			break;
		default:
			ret = -1;
			break;
	}
	spi_device_cs_release();

	return ret;
}


/* EEPROM Write
 * Performs a non-blocking write on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes written. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EFAULT : address above eeprom size
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
#define CMD_SIZE_SMALL 2
#define CMD_SIZE_BIG 3
#define MAX_CMD_SIZE CMD_SIZE_BIG
#define EEPROM_ID_MAX_PAGE_SIZE EEPROM_ID_BIG_PAGE_SIZE
int eeprom_write(uint32_t offset, const void *buf, size_t count)
{
	int ret = 0;
	uint8_t cmd_size = CMD_SIZE_BIG, page_size = EEPROM_ID_BIG_PAGE_SIZE;
	int write_count = 0, size = 0;
	char cmd[MAX_CMD_SIZE] = { EEPROM_ID_BIG_ADDR, 0, 0 };
	char full_buff[(EEPROM_ID_MAX_PAGE_SIZE + MAX_CMD_SIZE)];
	int eeprom_type = 0;

	if (spi_device_cs_pull_low() != 0) {
		return -EBUSY;
	}
	eeprom_type = get_eeprom_type();

	switch (eeprom_type) {
		case EEPROM_TYPE_SMALL:
			cmd_size = CMD_SIZE_SMALL;
			page_size = EEPROM_ID_SMALL_PAGE_SIZE;
			break;
		case EEPROM_TYPE_BIG:
			/* already configured */
			/* cmd_size = CMD_SIZE_BIG; */
			/* page_size = EEPROM_ID_BIG_PAGE_SIZE; */
			break;
		default:
			ret = -1;
			write_count = count + 1; /* skip the while loop, but return error */
			break;
	}
	while (write_count < count) {
		switch (eeprom_type) {
			case EEPROM_TYPE_SMALL:
				cmd[0] = EEPROM_ID_SMALL_ADDR | ((offset & 0x700) >> 7);
				cmd[1] = offset & 0xFF;
				break;
			case EEPROM_TYPE_BIG:
				cmd[1] = ((offset & 0xFF00) >> 8);
				cmd[2] = offset & 0xFF;
				break;
		}
		/* make partial first write to allign to page boundaries */
		if (offset & (page_size - 1)) {
			size = (page_size - (offset & (page_size - 1)));
		} else {
			size = page_size;
		}
		if (size > (count - write_count))
			size = (count - write_count);
		offset += size;
		memcpy(full_buff, cmd, cmd_size);
		memcpy(full_buff + cmd_size, buf + write_count, size);
		ret = i2c_write(full_buff, (cmd_size + size), NULL);

		if (ret != (cmd_size + size)) {
			break;
		}
		/* Wait for page write completion : The device does not acknoledge anything during
		 * page write, perform page writes with no data, until it returns 1 */
		do {
			ret = i2c_write(full_buff, 1, NULL);
		} while (ret != 1);

		write_count += size;
	}
	spi_device_cs_release();

	if (write_count != count)
		return ret;
	return write_count;
}


