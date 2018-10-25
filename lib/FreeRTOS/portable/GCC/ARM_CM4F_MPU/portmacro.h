/*
    FreeRTOS V7.5.3 - Copyright (C) 2013 Real Time Engineers Ltd. 
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long


typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE inline __attribute__(( always_inline))
#endif

portFORCE_INLINE static void vPortResetPrivilege( BaseType_t xRunningPrivileged )
{
	if( xRunningPrivileged != pdTRUE )
	{
		__asm volatile ( " mrs r0, control 	\n" \
						 " orr r0, #1 		\n" \
						 " msr control, r0	\n"	\
						 :::"r0", "memory" );
	}
}


#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Constants required to access and manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL					( ( volatile uint32_t * ) 0xe000e010 )
#define portNVIC_SYSTICK_LOAD					( ( volatile uint32_t * ) 0xe000e014 )
#define portNVIC_SYSPRI2						( ( volatile uint32_t * ) 0xe000ed20 )
#define portNVIC_SYSPRI1						( ( volatile uint32_t * ) 0xe000ed1c )
#define portNVIC_SYS_CTRL_STATE					( ( volatile uint32_t * ) 0xe000ed24 )
#define portNVIC_MEM_FAULT_ENABLE				( 1UL << 16UL )

/* Constants required to access and manipulate the MPU. */
#define portMPU_TYPE							( ( volatile uint32_t * ) 0xe000ed90 )
#define portMPU_REGION_BASE_ADDRESS				( ( volatile uint32_t * ) 0xe000ed9C )
#define portMPU_REGION_ATTRIBUTE				( ( volatile uint32_t * ) 0xe000edA0 )
#define portMPU_CTRL							( ( volatile uint32_t * ) 0xe000ed94 )
#define portEXPECTED_MPU_TYPE_VALUE				( 8UL << 8UL ) /* 8 regions, unified. */
#define portMPU_ENABLE							( 0x01UL )
#define portMPU_BACKGROUND_ENABLE				( 1UL << 2UL )
#define portPRIVILEGED_EXECUTION_START_ADDRESS	( 0UL )
#define portMPU_REGION_VALID					( 0x10UL )
#define portMPU_REGION_ENABLE					( 0x01UL )
#define portPERIPHERALS_START_ADDRESS			0x40000000UL
#define portPERIPHERALS_END_ADDRESS				0x5FFFFFFFUL








/* MPU specific constants. */
#define portUSING_MPU_WRAPPERS		1
#define portPRIVILEGE_BIT			( 0x80000000UL )

// Access permission defines
#define portMPU_AP_REGION_P_NA_U_NA				( 0x00UL << 24UL )
#define portMPU_AP_REGION_P_RW_U_NA				( 0x01UL << 24UL )
#define portMPU_AP_REGION_P_RW_U_RO				( 0x02UL << 24UL )
#define portMPU_AP_REGION_P_RW_U_RW				( 0x03UL << 24UL )
#define portMPU_AP_REGION_RESERVED				( 0x03UL << 24UL )
#define portMPU_AP_REGION_P_RO_U_NA				( 0x05UL << 24UL )
#define portMPU_AP_REGION_P_RO_U_RO				( 0x06UL << 24UL )
//#define portMPU_AP_REGION_P_RO_U_RO				( 0x07UL << 24UL ) Same as the one above in the cortex-M MCU

//Attributes and size define
#define portMPU_AS_REGION_CACHEABLE				( 0x01UL << 17UL)
#define portMPU_AS_REGION_BUFFERABLE			( 0x01UL << 16UL)
#define portMPU_AS_REGION_SHAREABLE				( 0x01UL << 18UL)
#define portMPU_AS_REGION_EXECUTE_NEVER			( 0x01UL << 28UL )

#define portMPU_AS_REGION_ZONE_DISABLE_0		( 0x01UL << 8UL)
#define portMPU_AS_REGION_ZONE_DISABLE_1		( 0x01UL << 9UL)
#define portMPU_AS_REGION_ZONE_DISABLE_2		( 0x01UL << 10UL)
#define portMPU_AS_REGION_ZONE_DISABLE_3		( 0x01UL << 11UL)
#define portMPU_AS_REGION_ZONE_DISABLE_4		( 0x01UL << 12UL)
#define portMPU_AS_REGION_ZONE_DISABLE_5		( 0x01UL << 13UL)
#define portMPU_AS_REGION_ZONE_DISABLE_6		( 0x01UL << 14UL)
#define portMPU_AS_REGION_ZONE_DISABLE_7		( 0x01UL << 15UL)

#define portMPU_REGION_0	0
#define portMPU_REGION_1	1
#define portMPU_REGION_2	2
#define portMPU_REGION_3	3
#define portMPU_REGION_4	4
#define portMPU_REGION_5	5
#define portMPU_REGION_6	6
#define portMPU_REGION_7	7


	//#define portMPU_SUBREGION_DISABLE				( 0xE9UL << 8UL ) //Disable zone 0, 3 and 5-7

//#define portUNPRIVILEGED_FLASH_REGION		( 0UL )
#define portUNPRIVILEGED_FLASH_CCM_SRAM_REGION ( 0UL )
//#define portPRIVILEGED_FLASH_REGION			( 1UL )
//#define portPRIVILEGED_RAM_REGION			( 2UL )
//#define portGENERAL_PERIPHERALS_REGION		( 1UL )
#define portSTACK_REGION					( 1UL )
#define portFIRST_CONFIGURABLE_REGION	    ( 2UL )
#define portLAST_CONFIGURABLE_REGION		( 7UL )
#define portNUM_CONFIGURABLE_REGIONS		2/*( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )*/
#define portTOTAL_NUM_REGIONS				( portNUM_CONFIGURABLE_REGIONS + 1 ) /* Plus one to make space for the stack region. */

#define portSWITCH_TO_USER_MODE() __asm volatile ( " mrs r0, control \n orr r0, #1 \n msr control, r0 " :::"r0" )

typedef struct MPU_REGION_REGISTERS
{
	unsigned portLONG ulRegionBaseAddress;
	unsigned portLONG ulRegionAttribute;
} xMPU_REGION_REGISTERS;

/* Plus 1 to create space for the stack region. */

//6 regions can be configured by the system.

typedef struct MPU_SETTINGS
{
	xMPU_REGION_REGISTERS xRegion[ portNUM_CONFIGURABLE_REGIONS ];
} xMPU_SETTINGS;

#define FPU_STRINGIFY(s) FPU_XSTR(s)
#define FPU_XSTR(s)	#s
#define FPU_VALUE 4 + sizeof(xMPU_SETTINGS)

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			8
/*-----------------------------------------------------------*/

/* SVC numbers for various services. */
#define portSVC_START_SCHEDULER				0
#define portSVC_YIELD						1
#define portSVC_RAISE_PRIVILEGE				2

/* Scheduler utilities. */

#define portYIELD()				__asm volatile ( "	SVC	%0	\n" :: "i" (portSVC_YIELD) )
#define portYIELD_WITHIN_API()	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET
#define portRAISE_PRIVILEGE()	__asm volatile ( "	SVC	%0	\n" :: "i" (portSVC_RAISE_PRIVILEGE) )
#define portNVIC_INT_CTRL			( ( volatile unsigned portLONG *) 0xe000ed04 )
#define portNVIC_PENDSVSET			0x10000000
#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired ) *(portNVIC_INT_CTRL) = portNVIC_PENDSVSET
#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/


/* Critical section management. */

/*
 * Set basepri to portMAX_SYSCALL_INTERRUPT_PRIORITY without effecting other
 * registers.  r0 is clobbered.
 */
#define portSET_INTERRUPT_MASK()						\
	__asm volatile										\
	(													\
		"	mov r0, %0								\n"	\
		"	msr basepri, r0							\n" \
		::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY):"r0"	\
	)

/*
 * Set basepri back to 0 without effective other registers.
 * r0 is clobbered.  FAQ:  Setting BASEPRI to 0 is not a bug.  Please see
 * http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html before disagreeing.
 */
#define portCLEAR_INTERRUPT_MASK()			\
	__asm volatile							\
	(										\
		"	mov r0, #0					\n"	\
		"	msr basepri, r0				\n"	\
		:::"r0"								\
	)



/* FAQ:  Setting BASEPRI to 0 is not a bug.  Please see
http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html before disagreeing. */
#define portSET_INTERRUPT_MASK_FROM_ISR()		0;portSET_INTERRUPT_MASK()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	portCLEAR_INTERRUPT_MASK();(void)x


extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );
extern UBaseType_t vPortIsCritical ( void );

#define portDISABLE_INTERRUPTS()	portSET_INTERRUPT_MASK()
#define portENABLE_INTERRUPTS()		portCLEAR_INTERRUPT_MASK()
#define portENTER_CRITICAL()		vPortEnterCritical()
#define portEXIT_CRITICAL()			vPortExitCritical()
#define portIS_CRITICAL()			vPortIsCritical()
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#define portNOP()



//define standard region here

//#define MPU_IO_REGION_USER_RW  (void *)0x40000000UL, 0x5FFFFFFFUL - 0x40000000UL, portMPU_AP_REGION_P_RW_U_RW
//#define MPU_BBRAM_REGION_USER_RW  (void *)0x40024000UL, 4096, portMPU_AP_REGION_P_RW_U_RW

#define BBRAM_REGION_USER_RW

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

