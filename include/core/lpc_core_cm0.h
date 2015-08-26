/****************************************************************************
 *   core/lpc_core_cm0.h
 *
 * Helper functions to access some registers and Cortex M0 core functionalities.
 *
 *
 * Most of the code from here comes from CMSIS. Should this be rewritten ?
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

#ifndef LPC_CORE_CM0_H
#define LPC_CORE_CM0_H

#include <stdint.h>   /* standard types definitions */
#include "core/lpc_regs_12xx.h"


/*******************************************************************************/
/*                Core Instructions                                            */
/*******************************************************************************/
/* NOP */
#define nop() __asm volatile ("nop" : : : "memory")
/* SEV : Send Event */
#define sev() __asm volatile ("sev" : : : "memory")
/* WFE : Wait For Event */
#define wfe() __asm volatile ("wfe" : : : "memory")
/* WFI : Wait For Interrupt */
#define wfi() __asm volatile ("wfi" : : : "memory")


/* Synchronization */
/* Instruction Synchronization Barrier : Flushes pipeline */
#define isb() __asm volatile ("isb" : : : "memory")
/* Data Synchronization Barrier : All explicit memory access completed */
#define dsb() __asm volatile ("dsb" : : : "memory")
/* Data Memory Barrier : Ensures apparent order but not completion */
#define dmb() __asm volatile ("dmb" : : : "memory")



/* Access to the Application Program Status Register (APSR). */
static inline uint32_t get_APSR(void)
{
	uint32_t r; __asm volatile ("mrs %0, apsr" : "=r" (r)); return (r);
}
#define APSR_SATURATION  (get_APSR() & (0x1 << 27))  /* bit 27  Saturation condition flag */
#define APSR_OVERFLOW    (get_APSR() & (0x1 << 28))  /* bit 28  Overflow condition code flag */
#define APSR_CARRY       (get_APSR() & (0x1 << 29))  /* bit 29  Carry condition code flag */
#define APSR_ZERO        (get_APSR() & (0x1 << 30))  /* bit 30  Zero condition code flag */
#define APSR_NEGATIVE    (get_APSR() & (0x1 << 31))  /* bit 31  Negative condition code flag */


/* Access the Interrupt Program Status Register (IPSR). */
static inline uint32_t get_IPSR(void)
{
	uint32_t r; __asm volatile ("mrs %0, ipsr" : "=r" (r)); return (r);
}
#define IPSR      (get_IPSR() & 0x1FF)  /* bit:  0..8  Exception number */
#define IPSR_IRQ0 16
#define IRQ_NUM   (IPSR - IPSR_IRQ0)


/* Control Register */
static inline uint32_t get_CONTROL(void)
{
	uint32_t r; __asm volatile ("mrs %0, control" : "=r" (r)); return (r);
}
static inline void set_CONTROL(uint32_t control)
{
	__asm volatile ("msr control, %0" : : "r" (control));
}


/* Process Stack Pointer */
static inline uint32_t get_process_stack_pointer(void)
{
	register uint32_t r; __asm volatile ("mrs %0, psp" : "=r" (r)); return (r);
}
static inline void set_process_stack_pointer(uint32_t top_of_stack)
{
	__asm volatile ("msr psp, %0" : : "r" (top_of_stack));
}


/* Main Stack Pointer */
static inline uint32_t get_main_stack_pointer(void)
{
	register uint32_t r; __asm volatile ("mrs %0, msp" : "=r" (r)); return (r);
}
static inline void set_main_stack_pointer(uint32_t top_of_stack)
{
	__asm volatile ("msr msp, %0" : : "r" (top_of_stack));
}


/* Priority Mask */
static inline uint32_t get_priority_mask(void)
{
	uint32_t r; __asm volatile ("mrs %0, primask" : "=r" (r)); return (r);
}
static inline void set_priority_mask(uint32_t mask)
{
	__asm volatile ("msr primask, %0" : : "r" (mask));
}


/* Base Priority */
static inline uint32_t get_base_priority(void)
{
	uint32_t r; __asm volatile ("mrs %0, basepri_max" : "=r" (r)); return (r);
}
static inline void set_base_priority(uint32_t prio)
{
	__asm volatile ("msr basepri, %0" : : "r" (prio));
}


/* Fault Mask */
static inline uint32_t get_fault_mask(void)
{
	uint32_t r; __asm volatile ("mrs %0, faultmask" : "=r" (r)); return (r);
}
static inline void set_fault_mask(uint32_t mask)
{
	__asm volatile ("msr faultmask, %0" : : "r" (mask));
}



/*******************************************************************************/
/*            Byte swap instructions                                           */
/*******************************************************************************/
/* Swap bytes of each 16-bit halfword in a 32-bit word, keeping halfword order */
static inline uint32_t double_byte_swap_16(volatile uint32_t value)
{
	uint32_t result = 0;
	__asm volatile ("rev16 %0, %1" : "=l" (result) : "l" (value));
	return result;
}
/* Change endianness of a 16-bit halfword */
static inline uint32_t byte_swap_16(volatile uint16_t value)
{
	uint32_t result = 0;
	__asm volatile ("rev16 %0, %1" : "=l" (result) : "l" (value));
	return (result & 0xFFFF);
}
/* Change endianness of a 32-bit word */
static inline uint32_t byte_swap_32(volatile uint32_t value)
{
	uint32_t result = 0;
	__asm volatile ("rev %0, %1" : "=l" (result) : "l" (value));
	return result;
}





/*******************************************************************************/
/*            Interrupts                                                       */
/*******************************************************************************/

/* Cortex-M0 Processor Exceptions Numbers */
/* Note : entry 0 is "end stack pointer" */
#define RESET_IRQ       ( -15 ) /*  1 - Reset ... not an interrupt ... */
#define NMI_IRQ         ( -14 ) /*  2 - Non Maskable Interrupt */
#define HARD_FAULT_IRQ  ( -13 ) /*  3 - Cortex-M0 Hard Fault Interrupt */
/* Note : entry 7 is the 2’s complement of the check-sum of the previous 7 entries */
#define SV_CALL_IRQ     (  -5 ) /* 11 - Cortex-M0 Supervisor Call Interrupt */
#define PEND_SV_IRQ     (  -2 ) /* 14 - Cortex-M0 Pend SV Interrupt */
#define SYSTICK_IRQ     (  -1 ) /* 15 - Cortex-M0 System Tick Interrupt */
/* LPC12xx Specific Interrupt Numbers */
#define WAKEUP0_IRQ       0   /* I/O pins can be used as wakeup source. */
#define WAKEUP1_IRQ       1
#define WAKEUP2_IRQ       2
#define WAKEUP3_IRQ       3
#define WAKEUP4_IRQ       4
#define WAKEUP5_IRQ       5
#define WAKEUP6_IRQ       6
#define WAKEUP7_IRQ       7
#define WAKEUP8_IRQ       8
#define WAKEUP9_IRQ       9
#define WAKEUP10_IRQ      10
#define WAKEUP11_IRQ      11
#define I2C0_IRQ          12  /* 12 - I2C 0 */
#define TIMER0_IRQ        13  /* 13 - Timer 0 */
#define TIMER1_IRQ        14  /* 14 - Timer 1 */
#define TIMER2_IRQ        15  /* 15 - Timer 2 */
#define TIMER3_IRQ        16  /* 16 - Timer 3 */
#define SSP0_IRQ          17  /* 17 - SSP 0 */
#define UART0_IRQ         18  /* 18 - UART 0 */
#define UART1_IRQ         19  /* 19 - UART 1 */
#define COMPARATOR_IRQ    20  /* 20 - Comparator */
#define ADC_IRQ           21  /* 21 - A/D Converter */
#define WDT_IRQ           22  /* 22 - Watchdog */
#define BOD_IRQ           23  /* 23 - Brown Out Detect(BOD) */
/* No IRQ 24 */                    
#define PIO_0_IRQ         25  /* 25 - GPIO int for port 0 */
#define PIO_1_IRQ         26  /* 26 - GPIO int for port 1 */
#define PIO_2_IRQ         27  /* 27 - GPIO int for port 2 */
/* No IRQ 28 */                    
#define DMA_IRQ           29  /* 29 - General Purpose DMA */
#define RTC_IRQ           30  /* 30 - RTC */


/* Enable IRQ Interrupts
  Enables IRQ interrupts by clearing the I-bit in the CPSR.
  Can only be executed in Privileged modes.
 */
static inline void lpc_enable_irq(void)
{
	__asm volatile ("cpsie i");
}

/* Disable IRQ Interrupts
  Disables IRQ interrupts by setting the I-bit in the CPSR.
  Can only be executed in Privileged modes.
 */
static inline void lpc_disable_irq(void)
{
	__asm volatile ("cpsid i");
}


/*******************************************************************************/
/*                Hardware Abstraction Layer : NVIC Functions                  */
/*******************************************************************************/
/*  Enable External Interrupt
  This function enables a device specific interrupt in the NVIC interrupt controller.
  The interrupt number cannot be a negative value.
  IRQ : Number of the external interrupt to enable
 */
static inline void NVIC_EnableIRQ(uint32_t IRQ)
{
	struct nvic_regs* nvic = LPC_NVIC;
	nvic->int_set_enable = (1 << (IRQ & 0x1F));
}

/*  Disable External Interrupt
  This function disables a device specific interupt in the NVIC interrupt controller.
  The interrupt number cannot be a negative value.
  IRQ : Number of the external interrupt to disable
 */
static inline void NVIC_DisableIRQ(uint32_t IRQ)
{
	struct nvic_regs* nvic = LPC_NVIC;
	nvic->int_clear_enable = (1 << (IRQ & 0x1F));
}

/*  Get Pending Interrupt
  This function reads the pending register in the NVIC and returns the pending bit
    for the specified interrupt.
  IRQ : Number of the interrupt for get pending
  return :
     0  Interrupt status is not pending
     1  Interrupt status is pending
 */
static inline uint32_t NVIC_GetPendingIRQ(uint32_t IRQ)
{
	struct nvic_regs* nvic = LPC_NVIC;
	return (uint32_t)((nvic->int_set_pending & (1 << (IRQ & 0x1F)))?1:0);
}

/*  Set Pending Interrupt
  This function sets the pending bit for the specified interrupt.
  The interrupt number cannot be a negative value.
  IRQ : Number of the interrupt for set pending
 */
static inline void NVIC_SetPendingIRQ(uint32_t IRQ)
{
	struct nvic_regs* nvic = LPC_NVIC;
	nvic->int_set_pending = (1 << (IRQ & 0x1F));
}

/*  Clear Pending Interrupt
  This function clears the pending bit for the specified interrupt.
  The interrupt number cannot be a negative value.
  IRQ : Number of the interrupt for clear pending
 */
static inline void NVIC_ClearPendingIRQ(uint32_t IRQ)
{
	struct nvic_regs* nvic = LPC_NVIC;
	nvic->int_clear_pending = (1 << (IRQ & 0x1F));
}


/* The following MACROS handle generation of the register offset and byte masks */
#define LPC_NVIC_PRIO_BITS  5  /* Number of Bits used for Priority Levels */
#define LPC_IRQ_BIT_SHIFT(IRQ) (((uint32_t)(IRQ) & 0x03) * 8)
#define LPC_IRQ_SHP_IDX(IRQ)   ((((int32_t)(IRQ) + 16) >> 2) - 1)
#define LPC_IRQ_IP_IDX(IRQ)    ((uint32_t)(IRQ) >> 2)

/*  Set Interrupt Priority
  This function sets the priority for the specified interrupt. The interrupt
    number can be positive to specify an external (device specific)
    interrupt, or negative to specify an internal (core) interrupt.
  Note: The priority cannot be set for every core interrupt.
  IRQ : Number of the interrupt for set priority
  priority : Priority to set
 */
static inline void NVIC_SetPriority(uint32_t IRQ, uint32_t priority)
{
	if ((int32_t)IRQ < 0) {
		if ((int32_t)IRQ > (-12)) {
			struct syst_ctrl_block_regs* scb = LPC_SCB;
			scb->shp[LPC_IRQ_SHP_IDX(IRQ)] = (scb->shp[LPC_IRQ_SHP_IDX(IRQ)] & ~(0xFF << LPC_IRQ_BIT_SHIFT(IRQ))) |
				(((priority << (8 - LPC_NVIC_PRIO_BITS)) & 0xFF) << LPC_IRQ_BIT_SHIFT(IRQ));
		}
	} else if (LPC_IRQ_IP_IDX(IRQ) < 8) {
		struct nvic_regs* nvic = LPC_NVIC;
		nvic->int_priority[LPC_IRQ_IP_IDX(IRQ)] = (nvic->int_priority[LPC_IRQ_IP_IDX(IRQ)] & ~(0xFF << LPC_IRQ_BIT_SHIFT(IRQ))) |
			(((priority << (8 - LPC_NVIC_PRIO_BITS)) & 0xFF) << LPC_IRQ_BIT_SHIFT(IRQ));
	}
}


/*  Get Interrupt Priority
  This function reads the priority for the specified interrupt. The interrupt
    number can be positive to specify an external (device specific)
    interrupt, or negative to specify an internal (core) interrupt.
  The returned priority value is automatically aligned to the implemented
    priority bits of the microcontroller.
  IRQ : Number of the interrupt for get priority
  return : Interrupt Priority
 */
static inline uint32_t NVIC_GetPriority(uint32_t IRQ)
{
	if ((int32_t)IRQ < 0) {
		if ((int32_t)IRQ > (-12)) {
			struct syst_ctrl_block_regs* scb = LPC_SCB;
			/* Get priority for Cortex-M0 system interrupts */
			return ((uint32_t)((scb->shp[LPC_IRQ_SHP_IDX(IRQ)] >> LPC_IRQ_BIT_SHIFT(IRQ) ) >> (8 - LPC_NVIC_PRIO_BITS)));
		}
	} else if (LPC_IRQ_IP_IDX(IRQ) < 8) {
		struct nvic_regs* nvic = LPC_NVIC;
		/* Get priority for device specific interrupts */
		return ((uint32_t)((nvic->int_priority[LPC_IRQ_IP_IDX(IRQ)] >> LPC_IRQ_BIT_SHIFT(IRQ) ) >> (8 - LPC_NVIC_PRIO_BITS)));
	}
}


/*  System Reset
  This function initiate a system reset request to reset the MCU.
 */
static inline void NVIC_SystemReset(void)
{
	struct syst_ctrl_block_regs* scb = LPC_SCB;
	dsb(); /* Ensure all outstanding memory accesses included buffered write are completed before reset */
	scb->aircr = ((0x5FA << SCB_AIRCR_VECTKEY_OFFSET) | SCB_AIRCR_SYSRESETREQ);
	dsb();  /* Ensure completion of memory access */
	while (1); /* wait until reset */
}



/*******************************************************************************/
/*            Sync lock                                                        */
/*******************************************************************************/
/* NOTE : There is no syncro instructions on Cortex-M0 */

/* IRQ released version : IRQ are enabled upon return
 * Returns the old value after the new value has been set.
 */
static inline uint32_t sync_lock_test_and_set(volatile uint32_t *addr, uint32_t value)
{
	uint32_t oldval;
	lpc_disable_irq();
    dsb();
	oldval = *addr;
	*addr = value;
    dsb();
	lpc_enable_irq();
	return oldval;
}
/* Remove the lock */
static inline void sync_lock_release(volatile uint32_t *addr)
{
    *addr = 0;
    dsb();
}

/* IRQ disabled version : If lock is acquired (old value is 0) IRQ are
 *     disabled when the call returns
 * Returns the old value after the new value has been set.
 */
static inline uint32_t irq_sync_lock_test_and_set(volatile uint32_t *addr, uint32_t value)
{
	uint32_t oldval;
	lpc_disable_irq();
    dsb();
	oldval = *addr;
	*addr = value;
    dsb();
	if (oldval) {
		lpc_enable_irq();
	}
	return oldval;
}
/* Remove the lock */
static inline void irq_sync_lock_release(volatile uint32_t *addr)
{
    *addr = 0;
    dsb();
	lpc_enable_irq();
}


#endif /* LPC_CORE_CM0_H */

