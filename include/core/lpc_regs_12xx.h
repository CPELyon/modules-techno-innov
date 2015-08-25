/****************************************************************************
 *   core/lpc12xx_regs.h
 *
 * Cortex-M0 Core Registers definitions
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
#ifndef LPC_REGS_H
#define LPC_REGS_H

/* Get size_t, and NULL from <stddef.h>.  */
#undef __need_malloc_and_calloc
#define __need_size_t
#define __need_NULL
#include <stddef.h>
#include <stdint.h>



/***************************************************************************** */
/*                          Memory Map                                         */
/***************************************************************************** */
/* Base addresses */
#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_RAM_BASE          (0x10000000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_APB1_BASE         (0x40080000UL) /* unused in LPC12xx */
#define LPC_AHB_BASE          (0x50000000UL)

/* Memory mapping of Cortex-M0 Hardware */
#define LPC_SCS_BASE        (0xE000E000UL)         /* System Control Space Base Address */
#define LPC_COREDEBUG_BASE  (0xE000EDF0UL)         /* Core Debug Base Address */
#define LPC_SYSTICK_BASE    (LPC_SCS_BASE + 0x0010UL)  /* SysTick Base Address */
#define LPC_NVIC_BASE       (LPC_SCS_BASE + 0x0100UL)  /* NVIC Base Address */
#define LPC_SCB_BASE        (LPC_SCS_BASE + 0x0D00UL)  /* System Control Block Base Address */

/* APB0 peripherals */
#define LPC_I2C0_BASE          (LPC_APB0_BASE + 0x00000)
#define LPC_WDT_BASE           (LPC_APB0_BASE + 0x04000)
#define LPC_UART0_BASE         (LPC_APB0_BASE + 0x08000)
#define LPC_UART1_BASE         (LPC_APB0_BASE + 0x0C000)
#define LPC_TIMER0_BASE        (LPC_APB0_BASE + 0x10000)
#define LPC_TIMER1_BASE        (LPC_APB0_BASE + 0x14000)
#define LPC_TIMER2_BASE        (LPC_APB0_BASE + 0x18000)
#define LPC_TIMER3_BASE        (LPC_APB0_BASE + 0x1C000)
#define LPC_ADC_BASE           (LPC_APB0_BASE + 0x20000)
#define LPC_PMU_BASE           (LPC_APB0_BASE + 0x38000)
#define LPC_SSP0_BASE          (LPC_APB0_BASE + 0x40000)
#define LPC_IOCON_BASE         (LPC_APB0_BASE + 0x44000)
#define LPC_SYSCON_BASE        (LPC_APB0_BASE + 0x48000)
#define LPC_DMA_BASE           (LPC_APB0_BASE + 0x4C000)
#define LPC_RTC_BASE           (LPC_APB0_BASE + 0x50000)
#define LPC_COMPARATOR_BASE    (LPC_APB0_BASE + 0x54000)

/* AHB peripherals */
#define LPC_GPIO_0_BASE        (LPC_AHB_BASE + 0x00000)
#define LPC_GPIO_1_BASE        (LPC_AHB_BASE + 0x10000)
#define LPC_GPIO_2_BASE        (LPC_AHB_BASE + 0x20000)
#define LPC_FLASH_CONFIG_BASE  (LPC_AHB_BASE + 0x60000)
#define LPC_CRC_BASE           (LPC_AHB_BASE + 0x70000)



/***************************************************************************** */
/*                     System Control                                          */
/***************************************************************************** */
/* System Control (SYSCON) */
struct lpc_sys_start_logic_ctrl
{
	volatile uint32_t edge_ctrl;  /* 0x00 : edge control Register 0 (R/W) */
	volatile uint32_t signal_en;  /* 0x04 : signal enable Register 0 (R/W) */
	volatile uint32_t reset;      /* 0x08 : reset Register 0  (-/W) */
	volatile uint32_t status;     /* 0x0C : status Register 0 (R/-) */
};
struct lpc_sys_control
{
	volatile uint32_t sys_mem_remap;   /* 0x000 System memory remap (R/W) */
	volatile uint32_t peripheral_reset_ctrl; /* 0x004 Peripheral reset control (R/W) */
	volatile uint32_t sys_pll_ctrl;    /* 0x008 System PLL control (R/W) */
	volatile uint32_t sys_pll_status;  /* 0x00C System PLL status (R/ ) */
	uint32_t reserved_0[4];

	volatile uint32_t sys_osc_ctrl;    /* 0x020 : System oscillator control (R/W) */
	volatile uint32_t WDT_osc_ctrl;    /* 0x024 : Watchdog oscillator control (R/W) */
	volatile uint32_t IRC_ctrl;        /* 0x028 : IRC control (R/W) */
	uint32_t reserved_1[1];
	volatile uint32_t sys_reset_status;    /* 0x030 : System reset status Register (R/ ) */
	uint32_t reserved_2[3];
	volatile uint32_t sys_pll_clk_sel;     /* 0x040 : System PLL clock source select (R/W) */	
	volatile uint32_t sys_pll_clk_upd_en;  /* 0x044 : System PLL clock source update enable (R/W) */
	uint32_t reserved_3[10];

	volatile uint32_t main_clk_sel;     /* 0x070 : Main clock source select (R/W) */
	volatile uint32_t main_clk_upd_en;  /* 0x074 : Main clock source update enable (R/W) */
	volatile uint32_t sys_AHB_clk_div;  /* 0x078 : System AHB clock divider (R/W) */
	uint32_t reserved_4[1];

	volatile uint32_t sys_AHB_clk_ctrl; /* 0x080 : System AHB clock control (R/W) */
	uint32_t reserved_5[4];

	volatile uint32_t ssp0_clk_div;   /* 0x094 : SSP0 clock divider (R/W) */
	volatile uint32_t uart_clk_div[2];  /* 0x098 - 0x09C : UART0 and UART1 clock divider (R/W) */
	volatile uint32_t rtc_clk_div;    /* 0x0A0 : RTC clock divider (R/W) */
	uint32_t reserved_6[15];

	volatile uint32_t clk_out_src_sel; /* 0x0E0 : CLKOUT clock source select (R/W) */
	volatile uint32_t clk_out_upd_en;  /* 0x0E4 : CLKOUT clock source update enable (R/W) */
	volatile uint32_t clk_out_div;     /* 0x0E8 : CLKOUT clock divider (R/W) */
	uint32_t reserved_7[5];

	volatile uint32_t por_captured_io_status_0;  /* 0x100 : POR captured PIO status 0 (R/ ) */
	volatile uint32_t por_captured_io_status_1;  /* 0x104 : POR captured PIO status 1 (R/ ) */
	uint32_t reserved_8[11];

	volatile uint32_t IO_config_clk_div[7]; /* 0x134 - 0x14C : Peripheral clocks 6 to 0 for glitch filter */

	volatile uint32_t BOD_ctrl;      /* 0x150 : BOD control (R/W) */
	volatile uint32_t sys_tick_cal;  /* 0x154 : System tick counter calibration (R/W) */
	volatile uint32_t ahb_prio_set;  /* 0x158 : AHB priority setting (-/-) */
	uint32_t reserved_9[5];
	volatile uint32_t irq_latency;   /* 0x170 : IRQ delay, alloxs trade-off bw latency and determinism */
	volatile uint32_t int_nmi_cfg;   /* 0x174 : NMI interrupt source configuration control */
	uint32_t reserved_10[34];

	struct lpc_sys_start_logic_ctrl start_log_strl[2]; /* 0x200 to 0x20C and 0x210 to 0x21C :
												 Start logic 0 and Start logic 1/peripheral interrupts */
	uint32_t reserved_11[4];

	volatile uint32_t powerdown_sleep_cfg;  /* 0x230 : Power-down states in Deep-sleep mode (R/W) */
	volatile uint32_t powerdown_awake_cfg;  /* 0x234 : Power-down states after wake-up (R/W) */
	volatile uint32_t powerdown_run_cfg;        /* 0x238 : Power-down configuration Register (R/W) */
	uint32_t reserved_12[110];
	volatile const uint32_t device_id;  /* 0x3F4 : Device ID (R/ ) */
};

#define LPC_SYS_CONTROL ((struct lpc_sys_control *) LPC_SYSCON_BASE)

/* AHB control bits
 *   0 (System (cortexM0, syscon, PMU, ...)) is a read only bit (system cannot be disabled)
 */
#define LPC_SYS_ABH_CLK_CTRL_SYSTEM     (1 <<  0) /* Read only */
#define LPC_SYS_ABH_CLK_CTRL_ROM        (1 <<  1)
#define LPC_SYS_ABH_CLK_CTRL_RAM        (1 <<  2)
#define LPC_SYS_ABH_CLK_CTRL_FLASH_REG  (1 <<  3)
#define LPC_SYS_ABH_CLK_CTRL_FLASH      (1 <<  4)
#define LPC_SYS_ABH_CLK_CTRL_I2C        (1 <<  5)
#define LPC_SYS_ABH_CLK_CTRL_CRC        (1 <<  6)
#define LPC_SYS_ABH_CLK_CTRL_CT16B0     (1 <<  7)
#define LPC_SYS_ABH_CLK_CTRL_CT16B1     (1 <<  8)
#define LPC_SYS_ABH_CLK_CTRL_CT32B0     (1 <<  9)
#define LPC_SYS_ABH_CLK_CTRL_CT32B1     (1 << 10)
#define LPC_SYS_ABH_CLK_CTRL_SSP0       (1 << 11)
#define LPC_SYS_ABH_CLK_CTRL_UART0      (1 << 12)
#define LPC_SYS_ABH_CLK_CTRL_UART1      (1 << 13)
#define LPC_SYS_ABH_CLK_CTRL_ADC        (1 << 14)
#define LPC_SYS_ABH_CLK_CTRL_Watchdog   (1 << 15)
#define LPC_SYS_ABH_CLK_CTRL_IO_CONFIG  (1 << 16)
#define LPC_SYS_ABH_CLK_CTRL_DMA        (1 << 17)
#define LPC_SYS_ABH_CLK_CTRL_RTC        (1 << 19)
#define LPC_SYS_ABH_CLK_CTRL_CMP        (1 << 20)
#define LPC_SYS_ABH_CLK_CTRL_GPIO2      (1 << 29)
#define LPC_SYS_ABH_CLK_CTRL_GPIO1      (1 << 30)
#define LPC_SYS_ABH_CLK_CTRL_GPIO0      (1 << 31)
/* Helper */
#define LPC_SYS_ABH_CLK_CTRL_MEM_ALL    0x0000001F
/* Flash override in Peripheral reset control register */
#define LPC_FLASH_OVERRIDE  (1 << 15 )

#define LPC_SSP_RESET_N        (1 << 0)
#define LPC_I2C_RESET_N        (1 << 1)
#define LPC_UART0_RESET_N      (1 << 2)
#define LPC_UART1_RESET_N      (1 << 3)

#define LPC_POWER_DOWN_IRC_OUT      (1 << 0)
#define LPC_POWER_DOWN_IRC          (1 << 1)
#define LPC_POWER_DOWN_FLASH        (1 << 2)
#define LPC_POWER_DOWN_BOD          (1 << 3)
#define LPC_POWER_DOWN_ADC          (1 << 4)
#define LPC_POWER_DOWN_SYS_OSC      (1 << 5)
#define LPC_POWER_DOWN_WDT_OSC      (1 << 6)
#define LPC_POWER_DOWN_SYSPLL       (1 << 7)
#define LPC_POWER_DOWN_COPARATOR    (1 << 15)

#define LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_ON  0x0000FFF7
#define LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_OFF 0x0000FFFF

#define LPC_CLKOUT_SRC_IRC_OSC       0x00
#define LPC_CLKOUT_SRC_XTAL_OSC      0x01
#define LPC_CLKOUT_SRC_WATCHDOG_OSC  0x02
#define LPC_CLKOUT_SRC_MAIN_CLK      0x03


/***************************************************************************** */
/*                  Flash Control                                              */
/***************************************************************************** */
/* Flash configuration */
struct lpc_flash_control
{
	uint32_t reserved_0[10];
	volatile uint32_t flash_cfg; /* 0x028 Flash configuration (R/W) */
};
#define LPC_FLASH_CONTROL ((struct lpc_flash_control *) LPC_FLASH_CONFIG_BASE)
#define LPC_FLASH_CFG_MASK   0x03
#define LPC_FLASH_CFG_SHIFT  0



/***************************************************************************** */
/*                     Cortex-M0 NVIC                                          */
/***************************************************************************** */
/* Cortex-M0 NVIC Registers */
struct nvic_regs {
	volatile uint32_t int_set_enable;  /* 0x000 : Interrupt Set Enable Register (R/W) */
	uint32_t reserved_0[31];
	volatile uint32_t int_clear_enable;  /* 0x080 : Interrupt Clear Enable Register (R/W) */
	uint32_t reserved_1[31];
	volatile uint32_t int_set_pending;  /* 0x100 : Interrupt Set Pending Register (R/W) */
	uint32_t reserved_2[31];
	volatile uint32_t int_clear_pending;  /* 0x180 : Interrupt Clear Pending Register (R/W) */
	uint32_t reserved_3[31];
	uint32_t reserved_4[64];
	volatile uint32_t int_priority[8]; /* 0x3EC : Interrupt Priority Register (R/W) */
};
#define LPC_NVIC      ((struct nvic_regs *) LPC_NVIC_BASE)        /* NVIC configuration struct */


/***************************************************************************** */
/*                    Cortex-M0 System Control Block                           */
/***************************************************************************** */
/* Cortex-M0 System Control Block Registers */
struct syst_ctrl_block_regs {
	volatile const uint32_t cpuid; /* 0x000 : CPU ID Base Register (R/ ) */
	volatile uint32_t icsr;        /* 0x004 : Interrupt Control State Register (R/W) */
	uint32_t reserved_0;
	volatile uint32_t aircr;       /* 0x00C : Application Interrupt / Reset Control Register (R/W) */
	volatile uint32_t scr;         /* 0x010 : System Control Register (R/W) */
	volatile uint32_t ccr;         /* 0x014 : Configuration Control Register (R/W) */
	uint32_t reserved_1;
	volatile uint32_t shp[2];      /* 0x01C : System Handlers Priority Registers. [0] is reserved_ (R/W) */
};
#define LPC_SCB       ((struct syst_ctrl_block_regs *) LPC_SCB_BASE) /* SCB configuration struct */

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER      (0xFFUL << 24)     /* SCB CPUID: IMPLEMENTER Mask */
#define SCB_CPUID_VARIANT          (0xFUL << 20)      /* SCB CPUID: VARIANT Mask */
#define SCB_CPUID_ARCHITECTURE     (0xFUL << 16)      /* SCB CPUID: ARCHITECTURE Mask */
#define SCB_CPUID_PARTNO           (0xFFFUL << 4)     /* SCB CPUID: PARTNO Mask */
#define SCB_CPUID_REVISION         (0xFUL << 0)       /* SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET        (1UL << 31)        /* SCB ICSR: NMIPENDSET Mask */
#define SCB_ICSR_PENDSVSET         (1UL << 28)        /* SCB ICSR: PENDSVSET Mask */
#define SCB_ICSR_PENDSVCLR         (1UL << 27)        /* SCB ICSR: PENDSVCLR Mask */
#define SCB_ICSR_PENDSTSET         (1UL << 26)        /* SCB ICSR: PENDSTSET Mask */
#define SCB_ICSR_PENDSTCLR         (1UL << 25)        /* SCB ICSR: PENDSTCLR Mask */
#define SCB_ICSR_ISRPREEMPT        (1UL << 23)        /* SCB ICSR: ISRPREEMPT Mask */
#define SCB_ICSR_ISRPENDING        (1UL << 22)        /* SCB ICSR: ISRPENDING Mask */
#define SCB_ICSR_VECTPENDING       (0x1FFUL << 12)    /* SCB ICSR: VECTPENDING Mask */
#define SCB_ICSR_VECTACTIVE        (0x1FFUL << 0)     /* SCB ICSR: VECTACTIVE Mask */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_OFFSET   16
#define SCB_AIRCR_VECTKEY          (0xFFFFUL << 16)   /* SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_VECTKEYSTAT      (0xFFFFUL << 16)   /* SCB AIRCR: VECTKEYSTAT Mask */
#define SCB_AIRCR_ENDIANESS        (1UL << 15)        /* SCB AIRCR: ENDIANESS Mask */
#define SCB_AIRCR_SYSRESETREQ      (1UL << 2)         /* SCB AIRCR: SYSRESETREQ Mask */
#define SCB_AIRCR_VECTCLRACTIVE    (1UL << 1)         /* SCB AIRCR: VECTCLRACTIVE Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND          (1UL << 4)         /* SCB SCR: SEVONPEND Mask */
#define SCB_SCR_SLEEPDEEP          (1UL << 2)         /* SCB SCR: SLEEPDEEP Mask */
#define SCB_SCR_SLEEPONEXIT        (1UL << 1)         /* SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN           (1UL << 9)         /* SCB CCR: STKALIGN Mask */
#define SCB_CCR_UNALIGN_TRP        (1UL << 3)         /* SCB CCR: UNALIGN_TRP Mask */



/***************************************************************************** */
/*                    Cortex-M0 System Timer                                   */
/***************************************************************************** */
/* Cortex-M0 System Timer Registers */
struct lpc_system_tick {
	volatile uint32_t control;     /* 0x000 : SysTick Control and Status Register (R/W) */
	volatile uint32_t reload_val;  /* 0x004 : SysTick Reload Value Register (R/W) */
	volatile uint32_t value;       /* 0x008 : SysTick Current Value Register (R/W) */
	volatile const uint32_t calibration;  /* 0x00C : SysTick Calibration Register (R/ ) */
};
#define LPC_SYSTICK  ((struct lpc_system_tick*) LPC_SYSTICK_BASE) /* SysTick configuration struct */

/* SysTick Control / Status Register Definitions */
#define LPC_SYSTICK_CTRL_COUNTFLAG  (1UL << 16)   /* SysTick CTRL: COUNTFLAG Mask */
#define LPC_SYSTICK_CTRL_CLKSOURCE  (1UL << 2)    /* SysTick CTRL: CLKSOURCE Mask */
#define LPC_SYSTICK_CTRL_TICKINT    (1UL << 1)    /* SysTick CTRL: TICKINT Mask */
#define LPC_SYSTICK_CTRL_ENABLE     (1UL << 0)    /* SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define LPC_SYSTICK_LOAD_RELOAD     (0xFFFFFFUL)  /* SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define LPC_SYSTICK_VAL_CURRENT     (0xFFFFFFUL)  /* SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define LPC_SYSTICK_CALIB_NOREF     (1UL << 31)   /* SysTick CALIB: NOREF Mask */
#define LPC_SYSTICK_CALIB_SKEW      (1UL << 30)   /* SysTick CALIB: SKEW Mask */
#define LPC_SYSTICK_CALIB_TENMS     (0xFFFFFFUL)  /* SysTick CALIB: TENMS Mask */



/***************************************************************************** */
/*                    Cortex-M0 RTC (Real-Time Clock)                          */
/***************************************************************************** */
/* Cortex-M0 RTC Registers */
struct lpc_rtc {
    volatile uint32_t data;     /* 0x000 : Data register (R/-) */
    volatile uint32_t match;    /* 0x004 : Match register (R/W) */
    volatile uint32_t load;     /* 0x008 : Load register (R/W) */
    volatile uint32_t control;  /* 0x00C : Control register (R/W) */
    volatile uint32_t intr_control;        /* 0x010 : Interrupt control set/clear register (R/W) */
    volatile uint32_t raw_intr_status;     /* 0x014 : Raw interrupt status register (R/-) */
    volatile uint32_t masked_intr_status;  /* 0x018 : Masked interrupt status register (R/-) */
    volatile uint32_t intr_clear;          /* 0x01C : Interrupt clear register (-/W) */
};
#define LPC_RTC  ((struct lpc_rtc*) LPC_RTC_BASE) /* SysTick configuration struct */

/* RTC Clock source selection */
#define LPC_RTC_CLK_1HZ          (0)
#define LPC_RTC_CLK_1HZ_DELAYED  (0x01)
#define LPC_RTC_CLK_1KHZ         (0x0A)
#define LPC_RTC_CLK_PCLK         (0x04)  /* Main clock divided by RTC clock divider value */

/* RTC control register */
#define LPC_RTC_START  (1UL << 0)
#define LPC_RTC_DISABLE  (0)

/* RTC interrupt control register */
#define LPC_RTC_INT_ENABLE  (1UL << 0)
#define LPC_RTC_INT_DISABLE  (0)

/* RTC interrupt clear register */
#define LPC_RTC_CLEAR_INTR  (1UL << 0)



/***************************************************************************** */
/*                     Power Management Unit                                   */
/***************************************************************************** */
/* Power Management Unit (PMU) */
struct lpc_pm_unit
{
	volatile uint32_t power_ctrl;  /* 0x000 : Power control Register (R/W) */
	volatile uint32_t gp_data[4];  /* 0x004 to 0x010 : General purpose Register 0 to 3 (R/W) */
	volatile uint32_t system_config; /* 0x014 : System configuration register (R/W) */
							/* (RTC clock control and hysteresis of the WAKEUP pin) */
};
#define LPC_PMU         ((struct lpc_pm_unit *) LPC_PMU_BASE)

/* System config register */
#define LPC_WAKEUP_PIN_HYST_MASK    (0x01 << 10)
#define LPC_RTC_CLK_SRC_SHIFT       11
#define LPC_RTC_CLK_SRC_MASK        (0x0F << LPC_RTC_CLK_SRC_SHIFT)
/* See RTC section above for RTC Clock source selection bits */


/***************************************************************************** */
/*                     IO Control                                              */
/***************************************************************************** */
/* Pin Connect Block (IOCON) */
struct lpc_io_control
{
	uint32_t reserved_0[2];
	volatile uint32_t pio0_19;   /* 0x008 : I/O configuration for pin pio0_19 (R/W) */
	volatile uint32_t pio0_20;   /* 0x00C : I/O configuration for pin pio0_20 (R/W) */
	volatile uint32_t pio0_21;   /* 0x010 : I/O configuration for pin pio0_21 (R/W) */
	volatile uint32_t pio0_22;   /* 0x014 : I/O configuration for pin pio0_22 (R/W) */
	volatile uint32_t pio0_23;   /* 0x018 : I/O configuration for pin pio0_23 (R/W) */
	volatile uint32_t pio0_24;   /* 0x01C : I/O configuration for pin pio0_24 (R/W) */
	volatile uint32_t pio0_25;   /* 0x020 : I/O configuration for pin pio0_25 (R/W) */
	volatile uint32_t pio0_26;   /* 0x024 : I/O configuration for pin pio0_26 (R/W) */
	volatile uint32_t pio0_27;   /* 0x028 : I/O configuration for pin pio0_27 (R/W) */

	volatile uint32_t pio2_12;   /* 0x02C : I/O configuration for pin pio2_12 (R/W) */
	volatile uint32_t pio2_13;   /* 0x030 : I/O configuration for pin pio2_13 (R/W) */
	volatile uint32_t pio2_14;   /* 0x034 : I/O configuration for pin pio2_14 (R/W) */
	volatile uint32_t pio2_15;   /* 0x038 : I/O configuration for pin pio2_15 (R/W) */

	volatile uint32_t pio0_28;   /* 0x03C : I/O configuration for pin pio0_28 (R/W) */
	volatile uint32_t pio0_29;   /* 0x040 : I/O configuration for pin pio0_29 (R/W) */

	volatile uint32_t pio0_0;    /* 0x044 : I/O configuration for pin pio0_0 (R/W) */
	volatile uint32_t pio0_1;    /* 0x048 : I/O configuration for pin pio0_1 (R/W) */
	volatile uint32_t pio0_2;    /* 0x04C : I/O configuration for pin pio0_2 (R/W) */
	uint32_t reserved_1[1];
	volatile uint32_t pio0_3;    /* 0x054 : I/O configuration for pin pio0_3 (R/W) */
	volatile uint32_t pio0_4;    /* 0x058 : I/O configuration for pin pio0_4 (R/W) */
	volatile uint32_t pio0_5;    /* 0x05C : I/O configuration for pin pio0_5 (R/W) */
	volatile uint32_t pio0_6;    /* 0x060 : I/O configuration for pin pio0_6 (R/W) */
	volatile uint32_t pio0_7;    /* 0x064 : I/O configuration for pin pio0_7 (R/W) */
	volatile uint32_t pio0_8;    /* 0x068 : I/O configuration for pin pio0_8 (R/W) */
	volatile uint32_t pio0_9;    /* 0x06C : I/O configuration for pin pio0_9 (R/W) */

	volatile uint32_t pio2_0;    /* 0x070 : I/O configuration for pin pio2_0 (R/W) */
	volatile uint32_t pio2_1;    /* 0x074 : I/O configuration for pin TDI2_1 (R/W) */
	volatile uint32_t pio2_2;    /* 0x078 : I/O configuration for pin TMS2_2 (R/W) */
	volatile uint32_t pio2_3;    /* 0x07C : I/O configuration for pin TDO2_3 (R/W) */
	volatile uint32_t pio2_4;    /* 0x080 : I/O configuration for pin nTR2_4 (R/W) */
	volatile uint32_t pio2_5;    /* 0x084 : I/O configuration for pin pio2_5 (R/W) */
	volatile uint32_t pio2_6;    /* 0x088 : I/O configuration for pin pio2_6 (R/W) */
	volatile uint32_t pio2_7;    /* 0x08C : I/O configuration for pin pio2_7 (R/W) */

	volatile uint32_t pio0_10;   /* 0x090 : I/O configuration for pin SWD0_10 (R/W) */
	volatile uint32_t pio0_11;   /* 0x094 : I/O configuration for pin pio0_11 (R/W) */
	volatile uint32_t pio0_12;   /* 0x098 : I/O configuration for pin pio0_12 (R/W) */
	volatile uint32_t pio0_13;   /* 0x09C : I/O configuration for pin pio0_13 (R/W) */
	volatile uint32_t pio0_14;   /* 0x0A0 : I/O configuration for pin pio0_14 (R/W) */
	volatile uint32_t pio0_15;   /* 0x0A4 : I/O configuration for pin pio0_15 (R/W) */
	volatile uint32_t pio0_16;   /* 0x0A8 : I/O configuration for pin pio0_16 (R/W) */
	volatile uint32_t pio0_17;   /* 0x0AC : I/O configuration for pin pio0_17 (R/W) */
	volatile uint32_t pio0_18;   /* 0x0B0 : I/O configuration for pin pio0_18 (R/W) */

	volatile uint32_t pio0_30;   /* 0x0B4 : I/O configuration for pin pio0_30 (R/W) */
	volatile uint32_t pio0_31;   /* 0x0B8 : I/O configuration for pin pio0_31 (R/W) */

	volatile uint32_t pio1_0;    /* 0x0BC : I/O configuration for pin pio1_0 (R/W) */
	volatile uint32_t pio1_1;    /* 0x0C0 : I/O configuration for pin pio1_1 (R/W) */
	volatile uint32_t pio1_2;    /* 0x0C4 : I/O configuration for pin pio1_2 (R/W) */
	volatile uint32_t pio1_3;    /* 0x0C8 : I/O configuration for pin pio1_3 (R/W) */
	volatile uint32_t pio1_4;    /* 0x0CC : I/O configuration for pin pio1_4 (R/W) */
	volatile uint32_t pio1_5;    /* 0x0D0 : I/O configuration for pin pio1_5 (R/W) */
	volatile uint32_t pio1_6;    /* 0x0D4 : I/O configuration for pin pio1_6 (R/W) */
	uint32_t reserved_2[2];

	volatile uint32_t pio2_8;    /* 0x0E0 : I/O configuration for pin pio2_8 (R/W) */
	volatile uint32_t pio2_9;    /* 0x0E4 : I/O configuration for pin pio2_9 (R/W) */
	volatile uint32_t pio2_10;   /* 0x0E8 : I/O configuration for pin pio2_10 (R/W) */
	volatile uint32_t pio2_11;   /* 0x0EC : I/O configuration for pin pio2_11 (R/W) */

};
#define LPC_IO_CONTROL  ((struct lpc_io_control *) LPC_IOCON_BASE)

/* FIXME : to be completed */
#define LPC_IO_FUNC_ALT(x) ((x & 0x07) << 0)

#define LPC_IO_MODE_INACTIVE  (0x00 << 4)
#define LPC_IO_MODE_PULL_UP   (0x01 << 4)

#define LPC_IO_INVERTED  (0x01 << 6)

#define LPC_IO_ANALOG    (0x00 << 7)
#define LPC_IO_DIGITAL   (0x01 << 7)

#define LPC_IO_DRIVE_LOWCURENT  (0x00 << 9)
#define LPC_IO_DRIVE_HIGHCURENT (0x01 << 9)

#define LPC_IO_OPEN_DRAIN_ENABLE (0x01 << 10)

#define LPC_IO_SAMPLE_MODE_BYP    (0x00 << 11)
#define LPC_FILTER_ONE_CLK    1
#define LPC_FILTER_TWO_CLK    2
#define LPC_FILTER_THREE_CLK  3
#define LPC_IO_SAMPLE_MODE(x)     ((x & 0x03) << 11)
#define LPC_IO_SAMPLE_CLK_DIV(x)  ((x & 0x07) << 13)
 

/***************************************************************************** */
/*                     General Purpose Input/Output                            */
/***************************************************************************** */
/* General Purpose Input/Output (GPIO) */
struct lpc_gpio
{
	volatile uint32_t mask;       /* 0x00 : Pin mask, affects data, out, set, clear and invert */
	volatile uint32_t in;         /* 0x04 : Port data Register (R/-) */
	volatile uint32_t out;        /* 0x08 : Port output Register (R/W) */
	volatile uint32_t set;        /* 0x0C : Port output set Register (-/W) */
	volatile uint32_t clear;      /* 0x10 : Port output clear Register (-/W) */
	volatile uint32_t toggle;     /* 0x14 : Port output invert Register (-/W) */
	uint32_t reserved_1[2];
	volatile uint32_t data_dir;   /* 0x20 : Data direction Register (R/W) */
	volatile uint32_t int_sense;  /* 0x24 : Interrupt sense Register (R/W) */
	volatile uint32_t int_both_edges; /* 0x28 : Interrupt both edges Register (R/W) */
	volatile uint32_t int_event;  /* 0x2C : Interrupt event Register  (R/W) */
	volatile uint32_t int_enable; /* 0x30 : Interrupt mask Register (R/W) */
	volatile uint32_t raw_int_status;    /* 0x34 : Raw interrupt status Register (R/-) */
	volatile uint32_t masked_int_status; /* 0x38 : Masked interrupt status Register (R/-) */
	volatile uint32_t int_clear;  /* 0x3C : Interrupt clear Register (R/W) */
};
#define LPC_GPIO_0      ((struct lpc_gpio *) LPC_GPIO_0_BASE)
#define LPC_GPIO_1      ((struct lpc_gpio *) LPC_GPIO_1_BASE)
#define LPC_GPIO_2      ((struct lpc_gpio *) LPC_GPIO_2_BASE)

#define LPC_GPIO_REGS(x)  ((struct lpc_gpio *) (LPC_AHB_BASE + (0x10000 * (x))))

#define GPIO_DIR_IN 0
#define GPIO_DIR_OUT 1



/***************************************************************************** */
/*                     Universal Asynchronous Receiver Transmitter             */
/***************************************************************************** */
/* Universal Asynchronous Receiver Transmitter (UART) */
struct lpc_uart_func {
	volatile uint32_t buffer; /* 0x000 : Transmit / Receiver Buffer Register (R/W) */
	volatile uint32_t intr_enable; /* 0x004 : Interrupt Enable Register (R/W) */
	volatile uint32_t intr_pending; /* 0x008 : Interrupt ID Register (R/-) */
};
struct lpc_uart_ctrl {
	volatile uint32_t divisor_latch_lsb;  /* 0x000 : Divisor Latch LSB (R/W) */
	volatile uint32_t divisor_latch_msb;  /* 0x004 : Divisor Latch MSB (R/W) */
	volatile uint32_t fifo_ctrl;  /* 0x008 : Fifo Control Register (-/W) */
};
struct lpc_uart
{
	union {
		struct lpc_uart_func func;
		struct lpc_uart_ctrl ctrl;
	};
	volatile uint32_t line_ctrl;   /* 0x00C : Line Control Register (R/W) */
	volatile uint32_t modem_ctrl;  /* 0x010 : Modem control Register (R/W) */
	volatile const uint32_t line_status;   /* 0x014 : Line Status Register (R/ ) */
	volatile const uint32_t modem_status;  /* 0x018 : Modem status Register (R/ ) */
	volatile uint32_t scratch_pad;  /* 0x01C : Scratch Pad Register (R/W) */
	volatile uint32_t auto_baud_ctrl;  /* 0x020 : Auto-baud Control Register (R/W) */
	volatile uint32_t irda_ctrl;       /* 0x024 : UART IrDA Control Register (R/W) */
	volatile uint32_t fractional_div;  /* 0x028 : Fractional Divider Register (R/W) */
	uint32_t reserved_1;
	volatile uint32_t transmit_enable;  /* 0x030 : Transmit Enable Register (R/W) */
	uint32_t reserved_2[6];
	volatile uint32_t RS485_ctrl;       /* 0x04C : RS-485/EIA-485 Control Register (R/W) */
	volatile uint32_t RS485_addr_match; /* 0x050 : RS-485/EIA-485 address match Register (R/W) */
	volatile uint32_t RS485_dir_ctrl_delay;  /* 0x054 : RS-485/EIA-485 direction control delay Register (R/W) */
	volatile uint32_t fifo_level;  /* 0x058 : Fifo Level Register (R/-) */
};
#define LPC_UART_0        ((struct lpc_uart *) LPC_UART0_BASE)
#define LPC_UART_1        ((struct lpc_uart *) LPC_UART1_BASE)

/* Line Control Register */
#define LPC_UART_5BIT          (0x00 << 0)
#define LPC_UART_6BIT          (0x01 << 0)
#define LPC_UART_7BIT          (0x02 << 0)
#define LPC_UART_8BIT          (0x03 << 0)
#define LPC_UART_1STOP         (0x00 << 2)
#define LPC_UART_2STOP         (0x01 << 2)
#define LPC_UART_NO_PAR        (0x00 << 3)
#define LPC_UART_ODD_PAR      ((0x01 << 3) | (0x00 << 4))
#define LPC_UART_EVEN_PAR      ((0x01 << 3) | (0x01 << 4))
#define LPC_UART_ENABLE_DLAB   (0x01 << 7)
/* FIFO Control Register */
#define LPC_UART_FIFO_EN       (0x01 << 0)
#define LPC_UART_RX_CLR        (0x01 << 1)
#define LPC_UART_TX_CLR        (0x01 << 2)
#define LPC_UART_DMA_MODE_EN   (0x01 << 3)
#define LPC_UART_FIFO_TRIG(x)  ((x & 0x03) << 6) /* 1 / 4 / 8 / 14 chars */
/* Interrupt Enable Register */
#define LPC_UART_RX_INT_EN     (0x01 << 0)
#define LPC_UART_TX_INT_EN     (0x01 << 1)
#define LPC_UART_RX_STATUS_INT_EN   (0x01 << 2)
/* Interrupt status */
#define LPC_UART_INT_MASK      (0x7 << 1)
#define LPC_UART_INT_MODEM     (0x0 << 1)
#define LPC_UART_INT_TX        (0x1 << 1)
#define LPC_UART_INT_RX        (0x2 << 1)
#define LPC_UART_INT_RX_STATUS (0x3 << 1)
#define LPC_UART_INT_TIMEOUT   (0x6 << 1)
/* RS485 Control */
#define LPC_RS485_ENABLE       (0x1 << 0)
#define LPC_RS485_RX_DIS       (0x1 << 1)
#define LPC_RS485_AUTO_ADDR_EN (0x1 << 2)
#define LPC_RS485_DIR_PIN_RTS  (0x0 << 3)
#define LPC_RS485_DIR_PIN_DTR  (0x1 << 3)
#define LPC_RS485_AUTO_DIR_EN  (0x1 << 4)
#define LPC_RS485_DIR_CTRL_INV (0x1 << 5)
/* RS485 */
#define LPC_RS485_ADDR(x)  ((x) & 0xFF)
#define LPC_RS485_DIR_DELAY(x)  ((x) & 0xFF)
/* IrDA */
#define LPC_IRDA_PULSEDIV(x)  (((x) & 0x07) << 3)


/***************************************************************************** */
/*                     Inter-Integrated Circuit                                */
/***************************************************************************** */
/* Inter-Integrated Circuit (I2C) */
struct lpc_i2c
{
	volatile uint32_t ctrl_set;      /* 0x000 : I2C Control Set Register (R/W) */
	volatile const uint32_t status;  /* 0x004 : I2C Status Register (R/-) */
	volatile uint32_t data;          /* 0x008 : I2C Data Register (R/W) */
	volatile uint32_t slave_addr_0;  /* 0x00C : I2C Slave Address Register 0 (R/W) */
	volatile uint32_t clk_duty_high; /* 0x010 : SCL Duty Cycle Register High Half Word (R/W) */
	volatile uint32_t clk_duty_low;  /* 0x014 : SCL Duty Cycle Register Low Half Word (R/W) */
	volatile  uint32_t ctrl_clear;   /* 0x018 : I2C Control Clear Register (-/W) */
	volatile uint32_t monitor_mode_ctrl;  /* 0x01C : Monitor mode control register (R/W) */
	volatile uint32_t slave_addr_1;  /* 0x020 : I2C Slave Address Register 1 (R/W) */
	volatile uint32_t slave_addr_2;  /* 0x024 : I2C Slave Address Register 2 (R/W) */
	volatile uint32_t slave_addr_3;  /* 0x028 : I2C Slave Address Register 3 (R/W) */
	volatile const uint32_t data_buffer;  /* 0x02C : Data buffer register (-/W) */
	volatile uint32_t slave_addr_mask[4]; /* 0x030 to 0x03C : I2C Slave address mask register 0 to 3 (R/W) */
};
#define LPC_I2C0         ((struct lpc_i2c *) LPC_I2C0_BASE)

#define I2C_ASSERT_ACK   (0x01 << 2)
#define I2C_INTR_FLAG    (0x01 << 3)
#define I2C_STOP_FLAG    (0x01 << 4)
#define I2C_START_FLAG   (0x01 << 5)
#define I2C_ENABLE_FLAG  (0x01 << 6)


/***************************************************************************** */
/*                     Synchronous Serial Communication                        */
/***************************************************************************** */
/* Synchronous Serial Communication (SSP) */
struct lpc_ssp
{
	volatile uint32_t ctrl_0;        /* 0x000 : Control Register 0 (R/W) */
	volatile uint32_t ctrl_1;        /* 0x004 : Control Register 1 (R/W) */
	volatile uint32_t data;          /* 0x008 : Data Register (R/W) */
	volatile const uint32_t status;  /* 0x00C : Status Registe (R/-) */
	volatile uint32_t clk_prescale;  /* 0x010 : Clock Prescale Register (R/W) */
	volatile uint32_t int_mask;      /* 0x014 : Interrupt Mask Set and Clear Register (R/W) */
	volatile uint32_t raw_int_status;     /* 0x018 : Raw Interrupt Status Register (R/-) */
	volatile uint32_t masked_int_status;  /* 0x01C : Masked Interrupt Status Register (R/-) */
	volatile uint32_t int_clear;     /* 0x020 : SSPICR Interrupt Clear Register (-/W) */
	volatile uint32_t dma_ctrl;      /* 0x024 : DMA Control Register (R/W) */
};
#define LPC_SSP0        ((struct lpc_ssp *) LPC_SSP0_BASE)

/* SSP Control 0 register */
#define LPC_SSP_DATA_WIDTH(x)    (((x) - 1) & 0x0F) /* Use 4 for 4 bits, 5 for 5 bits, .... */
#define LPC_SSP_FRAME_SPI        (0x00 << 4)
#define LPC_SSP_FRAME_TI         (0x01 << 4)
#define LPC_SSP_FRAME_MICROWIRE  (0x02 << 4)
#define LPC_SSP_SPI_CLK_LOW      (0x00 << 6)
#define LPC_SSP_SPI_CLK_HIGH     (0x01 << 6)
#define LPC_SSP_SPI_CLK_FIRST    (0x00 << 7)
#define LPC_SSP_SPI_CLK_LAST     (0x01 << 7)
#define LPC_SSP_SPI_CLK_RATE_DIV(x) (((x) & 0xFF) << 8)
/* SSP Control 1 register */
#define LPC_SSP_LOOPBACK_MODE      (0x01 << 0)
#define LPC_SSP_ENABLE             (0x01 << 1)
#define LPC_SSP_MASTER_MODE        (0x00 << 2)
#define LPC_SSP_SLAVE_MODE         (0x01 << 2)
#define LPC_SSP_SLAVE_OUT_DISABLE  (0x01 << 3)

/* SSP Status register */
#define LPC_SSP_ST_TX_EMPTY      (0x01 << 0)
#define LPC_SSP_ST_TX_NOT_FULL   (0x01 << 1)
#define LPC_SSP_ST_RX_NOT_EMPTY  (0x01 << 2)
#define LPC_SSP_ST_RX_FULL       (0x01 << 3)
#define LPC_SSP_ST_BUSY          (0x01 << 4)

/* SSP Interrupt Mask, Raw, Status, Clear */
#define LPC_SSP_INTR_RX_OVERRUN      (0x01 << 0)
#define LPC_SSP_INTR_RX_TIMEOUT      (0x01 << 1)
#define LPC_SSP_INTR_RX_HALF_FULL    (0x01 << 2)
#define LPC_SSP_INTR_TX_HALF_EMPTY   (0x01 << 3)

/* SSP DMA Control */
#define LPC_SSP_RX_DMA_EN   (0x01 << 0)
#define LPC_SSP_TX_DMA_EN   (0x01 << 1)



/***************************************************************************** */
/*                     Timer                                                   */
/***************************************************************************** */
/* Timer (TMR) */
struct lpc_timer
{
	volatile uint32_t int_reg;        /* 0x000 : Interrupt Register (R/W) */
	volatile uint32_t timer_ctrl;     /* 0x004 : Timer Control Register (R/W) */
	volatile uint32_t timer_counter;  /* 0x008 : Timer Counter Register (R/W) */
	volatile uint32_t prescale;       /* 0x00C : Prescale Register (R/W) */
	volatile uint32_t prescale_counter;  /* 0x010 : Prescale Counter Register (R/W) */
	volatile uint32_t match_ctrl;     /* 0x014 : Match Control Register (R/W) */
	volatile uint32_t match_reg[4];    /* 0x018 : Match Register 0 to 3 (R/W) */
	volatile uint32_t capture_ctrl;   /* 0x028 : Capture Control Register (R/W) */
	volatile const uint32_t capture_reg[4]; /* 0x02C : Capture Register 0 to 3 (R/ ) */
	volatile uint32_t external_match; /* 0x03C : External Match Register (R/W) */
	uint32_t reserved_2[12];
	volatile uint32_t count_ctrl;     /* 0x070 : Count Control Register (R/W) */
	volatile uint32_t pwm_ctrl;       /* 0x074 : PWM Control Register (R/W) */
};
#define LPC_TMR16B0     ((struct lpc_timer *) LPC_TIMER0_BASE)
#define LPC_TMR16B1     ((struct lpc_timer *) LPC_TIMER1_BASE)
#define LPC_TMR32B0     ((struct lpc_timer *) LPC_TIMER2_BASE)
#define LPC_TMR32B1     ((struct lpc_timer *) LPC_TIMER3_BASE)
#define LPC_TIMER_REGS(x)  ((struct lpc_timer *) (LPC_TIMER0_BASE + ((x) * 0x4000)))

#define LPC_TIMER_COUNTER_ENABLE (1 << 0) /* CEN */
#define LPC_TIMER_COUNTER_RESET  (1 << 1) /* CRST */

/* Match internal configuration */
#define LPC_TIMER_INTERRUPT_ON_MATCH   0x01
#define LPC_TIMER_RESET_ON_MATCH       0x02
#define LPC_TIMER_STOP_ON_MATCH        0x04
#define LPC_TIMER_MATCH_ERASE(x)       (0x07 << ((x) * 3))
#define LPC_TIMER_MATCH_SHIFT(x)       ((x) * 3)
/* Capture internal configuration */
#define LPC_TIMER_CAP_ON_RISING_EDGE   0x01
#define LPC_TIMER_CAP_ON_FALLING_EDGE  0x02
#define LPC_TIMER_INTERRUPT_ON_CAPTURE 0x04
#define LPC_TIMER_CAPTURE_ERASE(x)     (0x07 << ((x) * 3))
#define LPC_TIMER_CAPTURE_SHIFT(x)     ((x) * 3)
/* Match external configuration */
#define LPC_TIMER_NOTHING_ON_MATCH     0x00
#define LPC_TIMER_CLEAR_ON_MATCH       0x01
#define LPC_TIMER_SET_ON_MATCH         0x02
#define LPC_TIMER_TOGGLE_ON_MATCH      0x03
#define LPC_TIMER_EXT_MATCH0_SHIFT     4
#define LPC_TIMER_EXT_MATCH1_SHIFT     6
#define LPC_TIMER_EXT_MATCH2_SHIFT     8
#define LPC_TIMER_EXT_MATCH3_SHIFT     10
/* Counter */
#define LPC_COUNTER_IS_TIMER           0x00
#define LPC_COUNTER_INC_ON_RISING      0x01
#define LPC_COUNTER_INC_ON_FALLING     0x02
#define LPC_COUNTER_INC_ON_BOTH        0x03
#define LPC_COUNTER_INC_INPUT_SHIFT    2
#define LPC_COUNTER_INC_INPUT(x)       (((x) & 0x03) << LPC_COUNTER_INC_INPUT_SHIFT)
#define LPC_COUNTER_CLEAR_ON_EVENT_EN  (0x01 << 4)
#define LPC_COUNTER_CLEAR_ON_EVENT_SHIFT  5
#define LPC_COUNTER_CLEAR_ON_CHAN0_RISE   0x00
#define LPC_COUNTER_CLEAR_ON_CHAN0_FALL   0x01
#define LPC_COUNTER_CLEAR_ON_CHAN1_RISE   0x02
#define LPC_COUNTER_CLEAR_ON_CHAN1_FALL   0x03
#define LPC_COUNTER_CLEAR_ON_CHAN2_RISE   0x04
#define LPC_COUNTER_CLEAR_ON_CHAN2_FALL   0x05
#define LPC_COUNTER_CLEAR_ON_CHAN3_RISE   0x06
#define LPC_COUNTER_CLEAR_ON_CHAN3_FALL   0x07
/* PWM */
#define LPC_PWM_CHANNEL_ENABLE(x)    (0x01 << (x))



/***************************************************************************** */
/*                     Watchdog Timer                                          */
/***************************************************************************** */
/* Watchdog Timer (WDT) */
struct lpc_watchdog
{
	volatile uint32_t mode;          /* 0x000 : Watchdog mode register (R/W) */
	volatile uint32_t timer_const;   /* 0x004 : Watchdog timer constant register (R/W) */
	volatile uint32_t feed_seqence;  /* 0x008 : Watchdog feed sequence register ( /W) */
	volatile const uint32_t timer_value;  /* 0x00C : Watchdog timer value register (R/ ) */
	volatile uint32_t clk_src_sel;   /* 0x010 : Wathdog Clock Source Selection Register (R/W) */
	volatile uint32_t warning_int_compare; /* 0x014 : Watchdog Warning Interrupt compare value. */
	volatile uint32_t window_compare;      /* 0x018 : Watchdog Window compare value. */
};
#define LPC_WDT         ((struct lpc_watchdog *) LPC_WDT_BASE)


/***************************************************************************** */
/*                     Analog-to-Digital Converter                             */
/***************************************************************************** */
/* Analog-to-Digital Converter (ADC) */
struct lpc_adc
{
	volatile uint32_t ctrl;         /* 0x000 : A/D Control Register (R/W) */
	volatile uint32_t global_data;  /* 0x004 : A/D Global Data Register (R/W) */
	uint32_t reserved_0;
	volatile uint32_t int_en;  /* 0x00C : A/D Interrupt Enable Register (R/W) */
	volatile uint32_t data[8]; /* Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W) */
	volatile const uint32_t status; /* 0x030 : A/D Status Register (R/ ) */
	volatile uint32_t trim;    /* 0x034 : A/D Trim Register (R/W) */
};
#define LPC_ADC         ((struct lpc_adc *) LPC_ADC_BASE)

/* ADC Control register bits */
#define LPC_ADC_CTRL_MASK  0x0F01FFFF
/* LPC_ADC_CHANNEL_* are also used for interrupt register */
#define LPC_ADC_CHANNEL_MASK (0xFF << 0)
#define LPC_ADC_CHANNEL(x)  (0x01 << ((x) & 0x07))
#define LPC_ADC_BURST     (0x01 << 16)

#define LPC_ADC_START_CONV_NOW  (0x01 << 24)
enum lpc_adc_start_conv_events {
	LPC_ADC_START_CONV_EDGE_CT16B0_CAP0 = 2,
	LPC_ADC_START_CONV_EDGE_CT32B0_CAP0,
	LPC_ADC_START_CONV_EDGE_CT32B0_MAT0,
	LPC_ADC_START_CONV_EDGE_CT32B0_MAT1,
	LPC_ADC_START_CONV_EDGE_CT16B0_MAT0,
	LPC_ADC_START_CONV_EDGE_CT16B0_MAT1,
};
#define LPC_ADC_START_CONV_EVENT(x) (((x) & 0x7) << 24)
#define LPC_ADC_START_EDGE_FALLING  (0x1 << 27)
#define LPC_ADC_START_EDGE_RISING   (0x0 << 27)
#define LPC_ADC_START_CONV_MASK (0x07 << 24)

/* ADC Data register bits */
#define LPC_ADC_RESULT_SHIFT  6
#define LPC_ADC_RESULT_MASK   0x3FF
#define LPC_ADC_OVERRUN    (0x01 << 30)
#define LPC_ADC_CONV_DONE  (0x01 << 31)

/* For more readability when selecting a channel number */
#define LPC_ADC_NUM(x)    (x)

#endif  /* LPC_REGS_H */
