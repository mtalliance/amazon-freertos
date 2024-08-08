/**
 * @file    port.c
 * @brief   FreeRTOS Cortex M4F core
 * @author  Dr. Klaus Schaefer schaefer@eit.h-da.de
 * @mainpage
 * <b>MPU plus FPU port for the ARM Cortex M4F Microcontroller</b>
 *
 * - MPU and FPU used
 * - FPU context saved and restored on demand when switching to and from tasks using the FPU.
 * - FPU context NOT saved on entry of ISR. No overhead on entry of interrupt service routines.
 *
 * @attention Tasks that run in privileged mode need to set <b>portPRIVILEGE_BIT</b> in the tasks priority field.
 *
*/

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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM3 port.
 *----------------------------------------------------------*/

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE


//#undef prvRaisePrivilege
//#undef portRESET_PRIVILEGE


/* Set the privilege level to user mode if xRunningPrivileged is false. */
//#define portRESET_PRIVILEGE( xRunningPrivileged ) if( xRunningPrivileged != pdTRUE ) __asm volatile ( " mrs r0, control \n orr r0, #1 \n msr control, r0" :::"r0" )





/* Constants required to access and manipulate the FPU. */
#define portSCB_CPACR							( ( volatile unsigned long * ) 0xe000ed88 )
#define portFPU_FPCCR							( ( volatile unsigned long * ) 0xe000ef34 )

/* Constants required to access and manipulate the SysTick. */
#define portNVIC_SYSTICK_CLK					( 0x00000004UL )
#define portNVIC_SYSTICK_INT					( 0x00000002UL )
#define portNVIC_SYSTICK_ENABLE					( 0x00000001UL )
#define portNVIC_PENDSV_PRI						( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI					( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )
//#define portNVIC_SVC_PRI						( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )
 #define portNVIC_SVC_PRI						( ( ( uint32_t ) configMAX_SYSCALL_INTERRUPT_PRIORITY - 0x10 ) << 24UL )


/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR						( 0x01000000 )
#define portINITIAL_CONTROL_IF_UNPRIVILEGED		( 0x03 )
#define portINITIAL_CONTROL_IF_PRIVILEGED		( 0x02 )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR					( 0x01000000 )
#define portINITIAL_EXEC_RETURN				( 0xfffffffd )

/* Offsets in the stack to the parameters when inside the SVC handler. */
#define portOFFSET_TO_PC						( 6 )


/* Each task maintains its own interrupt status in the critical nesting
variable.  Note this is not saved as part of the task context as context
switches can only occur when uxCriticalNesting is zero. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * Setup the timer to generate the tick interrupts.
 */
static void prvSetupTimerInterrupt( void ) PRIVILEGED_FUNCTION;

/*
 * Configure the FPU: FPU enable and automatic FPU context pushing disabled.
 */
static void prvSetupFPU( void ) PRIVILEGED_FUNCTION;

/*
 * Configure a number of standard MPU regions that are used by all tasks.
 */
static void prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/*
 * Return the smallest MPU region size that a given number of bytes will fit
 * into.  The region size is returned as the value that should be programmed
 * into the region attribute register for that region.
 */
static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes ) PRIVILEGED_FUNCTION;


/* Constants required to manipulate the VFP. */
#define portFPCCR							( ( volatile uint32_t * ) 0xe000ef34 ) /* Floating point context control register. */
#define portASPEN_AND_LSPEN_BITS			( 0x3UL << 30UL )

/*
 * Standard FreeRTOS exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked )) PRIVILEGED_FUNCTION;
void xPortSysTickHandler( void )  __attribute__ ((optimize("3"))) PRIVILEGED_FUNCTION;
void vPortSVCHandler( void ) __attribute__ (( naked )) PRIVILEGED_FUNCTION;

/*
 * Starts the scheduler by restoring the context of the first task to run.
 */
static void prvRestoreContextOfFirstTask( void ) __attribute__(( naked )) PRIVILEGED_FUNCTION;

/*
 * C portion of the SVC handler.  The SVC handler is split between an asm entry
 * and a C wrapper for simplicity of coding and maintenance.
 */
static void prvSVCHandler( uint32_t *pulRegisters ) __attribute__(( noinline )) PRIVILEGED_FUNCTION;

/**
 * @brief Checks whether or not the processor is privileged.
 *
 * @return 1 if the processor is already privileged, 0 otherwise.
 */
BaseType_t xIsPrivileged( void ) __attribute__ (( naked ));

/**
 * @brief Lowers the privilege level by setting the bit 0 of the CONTROL
 * register.
 *
 * Bit 0 of the CONTROL register defines the privilege level of Thread Mode.
 *  Bit[0] = 0 --> The processor is running privileged
 *  Bit[0] = 1 --> The processor is running unprivileged.
 */
void vResetPrivilege( void ) __attribute__ (( naked ));

/**
 * @brief Calls the port specific code to raise the privilege.
 *
 * @return pdFALSE if privilege was raised, pdTRUE otherwise.
 */
extern BaseType_t xPortRaisePrivilege( void );

/**
 * @brief If xRunningPrivileged is not pdTRUE, calls the port specific
 * code to reset the privilege, otherwise does nothing.
 */
extern void vPortResetPrivilege( BaseType_t xRunningPrivileged );

/*
 * Prototypes for all the MPU wrappers.
 */
#if 0
BaseType_t MPU_xTaskGenericCreate( TaskFunction_t pvTaskCode, const char * const pcName, uint16_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask, StackType_t *puxStackBuffer, const MemoryRegion_t * const xRegions );
void MPU_vTaskAllocateMPURegions( TaskHandle_t xTask, const MemoryRegion_t * const xRegions );
void MPU_vTaskDelete( TaskHandle_t pxTaskToDelete );
void MPU_vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, TickType_t xTimeIncrement );
void MPU_vTaskDelay( TickType_t xTicksToDelay );
UBaseType_t MPU_uxTaskPriorityGet( TaskHandle_t pxTask );
void MPU_vTaskPrioritySet( TaskHandle_t pxTask, UBaseType_t uxNewPriority );
eTaskState MPU_eTaskGetState( TaskHandle_t pxTask );
void MPU_vTaskSuspend( TaskHandle_t pxTaskToSuspend );
void MPU_vTaskResume( TaskHandle_t pxTaskToResume );
void MPU_vTaskSuspendAll( void );
BaseType_t MPU_xTaskResumeAll( void );
TickType_t MPU_xTaskGetTickCount( void );
UBaseType_t MPU_uxTaskGetNumberOfTasks( void );
void MPU_vTaskList( char *pcWriteBuffer );
void MPU_vTaskGetRunTimeStats( char *pcWriteBuffer );
void MPU_vTaskSetApplicationTaskTag( TaskHandle_t xTask, TaskHookFunction_t pxTagValue );
TaskHookFunction_t MPU_xTaskGetApplicationTaskTag( TaskHandle_t xTask );
BaseType_t MPU_xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter );
UBaseType_t MPU_uxTaskGetStackHighWaterMark( TaskHandle_t xTask );
TaskHandle_t MPU_xTaskGetCurrentTaskHandle( void );
BaseType_t MPU_xTaskGetSchedulerState( void );
TaskHandle_t MPU_xTaskGetIdleTaskHandle( void );
UBaseType_t MPU_uxTaskGetSystemState( TaskStatus_t *pxTaskStatusArray, UBaseType_t uxArraySize, uint32_t *pulTotalRunTime );
QueueHandle_t MPU_xQueueGenericCreate( UBaseType_t uxQueueLength, UBaseType_t uxItemSize, uint8_t ucQueueType );
BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, BaseType_t xCopyPosition );
BaseType_t MPU_xQueueGenericReset( QueueHandle_t pxQueue, BaseType_t xNewQueue );
UBaseType_t MPU_uxQueueMessagesWaiting( const QueueHandle_t pxQueue );
BaseType_t MPU_xQueueGenericReceive( QueueHandle_t pxQueue, void * const pvBuffer, TickType_t xTicksToWait, BaseType_t xJustPeeking );
QueueHandle_t MPU_xQueueCreateMutex( void );
QueueHandle_t MPU_xQueueCreateCountingSemaphore( UBaseType_t uxCountValue, UBaseType_t uxInitialCount );
BaseType_t MPU_xQueueTakeMutexRecursive( QueueHandle_t xMutex, TickType_t xBlockTime );
BaseType_t MPU_xQueueGiveMutexRecursive( QueueHandle_t xMutex );
BaseType_t MPU_xQueueAltGenericSend( QueueHandle_t pxQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, BaseType_t xCopyPosition );
BaseType_t MPU_xQueueAltGenericReceive( QueueHandle_t pxQueue, void * const pvBuffer, TickType_t xTicksToWait, BaseType_t xJustPeeking );
void MPU_vQueueAddToRegistry( QueueHandle_t xQueue, char *pcName );
void MPU_vQueueDelete( QueueHandle_t xQueue );
void *MPU_pvPortMalloc( size_t xSize );
void *MPU_pvPortMemalign( size_t bondary,  size_t xSize );
void MPU_vPortFree( void *pv );
void MPU_vPortInitialiseBlocks( void );
size_t MPU_xPortGetFreeHeapSize( void );
QueueSetHandle_t MPU_xQueueCreateSet( UBaseType_t uxEventQueueLength );
QueueSetMemberHandle_t MPU_xQueueSelectFromSet( QueueSetHandle_t xQueueSet, TickType_t xBlockTimeTicks );
BaseType_t MPU_xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet );
BaseType_t MPU_xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet );
BaseType_t MPU_xQueuePeekFromISR( QueueHandle_t xQueue, void * const pvBuffer );
void* MPU_xQueueGetMutexHolder( QueueHandle_t xSemaphore );
//Event
EventGroupHandle_t MPU_xEventGroupCreate( void );
EventBits_t MPU_xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait );
EventBits_t MPU_xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear );
EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet );
EventBits_t MPU_xEventGroupSync( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, const EventBits_t uxBitsToWaitFor, TickType_t xTicksToWait );
void MPU_vEventGroupDelete( EventGroupHandle_t xEventGroup );
#endif
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged ) PRIVILEGED_FUNCTION;

StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pxCode;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = 0xDEEDBEEF;	/* LR */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	*pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */

	/* A save method is being used that requires each task to maintain its
	own exec return value. */
	pxTopOfStack--;
	*pxTopOfStack = portINITIAL_EXEC_RETURN;

	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5, R4 */

	pxTopOfStack--; // privileged data
	if( xRunPrivileged == pdTRUE )
	{
		*pxTopOfStack = portINITIAL_CONTROL_IF_PRIVILEGED;
	}
	else
	{
		*pxTopOfStack = portINITIAL_CONTROL_IF_UNPRIVILEGED;
	}

	pxTopOfStack++;
	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
	/* Assumes psp was in use. */
	__asm volatile
	(
		#ifndef USE_PROCESS_STACK	/* Code should not be required if a main() is using the process stack. */
			"	tst lr, #4						\n"
			"	ite eq							\n"
			"	mrseq r0, msp					\n"
			"	mrsne r0, psp					\n"
		#else
			"	mrs r0, psp						\n"
		#endif
			"	b %0							\n"
			::"i"(prvSVCHandler):"r0"
	);
}
/*-----------------------------------------------------------*/

static void prvSVCHandler(	uint32_t *pulParam )
{
uint8_t ucSVCNumber;

	/* The stack contains: r0, r1, r2, r3, r12, r14, the return address and
	xPSR.  The first argument (r0) is pulParam[ 0 ]. */
	ucSVCNumber = ( ( uint8_t * ) pulParam[ portOFFSET_TO_PC ] )[ -2 ];
	switch( ucSVCNumber )
	{
		case portSVC_START_SCHEDULER	:	*(portNVIC_SYSPRI1) |= portNVIC_SVC_PRI;
											//traceTASK_FIRST_SWITCHED_IN();
											prvRestoreContextOfFirstTask();
											break;

		case portSVC_YIELD				:	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
											/* Barriers are normally not required
											but do ensure the code is completely
											within the specified behaviour for the
											architecture. */
											__asm volatile( "dsb" );
											__asm volatile( "isb" );

											break;

		case portSVC_RAISE_PRIVILEGE	:	__asm volatile
											(
												"	mrs r1, control		\n" /* Obtain current control value. */
												"	bic r1, #1			\n" /* Set privilege bit. */
												"	msr control, r1		\n" /* Write back new control value. */
												:::"r1"
											);
											break;

		default							:	/* Unknown SVC call. */
											break;
	}
}
/*-----------------------------------------------------------*/

static void prvRestoreContextOfFirstTask( void )
{
	__asm volatile
	(
		"	ldr r0, =0xE000ED08				\n" /* Use the NVIC offset register to locate the stack. */
		"	ldr r0, [r0]					\n"
		"	ldr r0, [r0]					\n"
		"	msr msp, r0						\n" /* Set the msp back to the start of the stack. */

		"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
		"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
		"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */


		"   ldr r3 , [r0, #-4]            	\n"	/* restore the privilege mode */
		"	msr control, r3					\n" /* set the privilege mode depending of what was initialised */

		"	ldmia r0!, {r4-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */


			/* MPU configuration */
		"	ldr	r3, pxCurrentTCBConst2		\n" /* Get the pxCurrentTCP pointer. */
		"	ldr r1, [r3]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
		"	add r1, r1, #68					\n" /* Move onto the second item in the TCB... */
		"	ldr r2, =0xe000ed9c				\n" /* Region Base Address register. */
		"	ldmia r1!, {r4-r7}				\n" /* Read 2 sets of MPU registers. */
		"	stmia r2!, {r4-r7}				\n" /* Write 2 sets of MPU registers. */
			/* END of MPU configuration */
		"	msr psp, r0						\n" /* Restore the task stack pointer. */
		"	isb								\n"
		"	mov r0, #0 						\n"
		"	msr	basepri, r0					\n"
		"	bx r14							\n"
		"									\n"
		"	.align 2						\n"
		"pxCurrentTCBConst2: .word pxCurrentTCB				\n"
	);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.  See
	http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) );

	/* Make PendSV and SysTick the same priority as the kernel. */
	*(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	*(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;

	/* Configure the FPU. */
	prvSetupFPU();

	/* Lazy save always. */
	*( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;


	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;
	/* Start the first task. */
	/* Configure the regions in the MPU that are common to all tasks. */
	prvSetupMPU();
	__asm volatile( "	svc %0			\n"
					:: "i" (portSVC_START_SCHEDULER) );

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
BaseType_t xRunningPrivileged = xPortRaisePrivilege();

	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	vPortResetPrivilege( xRunningPrivileged );
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
BaseType_t xRunningPrivileged = xPortRaisePrivilege();

	configASSERT( uxCriticalNesting );
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
	vPortResetPrivilege( xRunningPrivileged );
}
/*-----------------------------------------------------------*/
UBaseType_t vPortIsCritical ( void )
{
	return uxCriticalNesting;
}
void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	mrs r0, psp							\n"
	"	isb									\n"
	"										\n"
	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
	"	ldr	r2, [r3]						\n"

	"	tst lr, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */
	"	it eq								\n"
	"	vstmdbeq r0!, {s16-s31}				\n"

	"	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */

	"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
	"										\n"
	"	mrs r1, control						\n" /* get the currect execution mode (privilege or user) */
	"	stmdb r0!, {r1}						\n" /* push the privilege mode to the stack frame*/

	"										\n"
	"	stmdb sp!, {r3}						\n"
	"	mov r0, %0 							\n"
	"	msr basepri, r0						\n"
	"	dsb									\n"
	"   isb									\n"
	"	bl vTaskSwitchContext				\n"
	"	mov r0, #0							\n"
	"	msr basepri, r0						\n"

	"	ldmia sp!, {r3}						\n"
/* MPU configuration */
	"	ldr	r3, pxCurrentTCBConst		\n" /* Get the pxCurrentTCP pointer. */
	"	ldr r1, [r3]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
	"	add r1, r1, #68					\n" /* Move onto the second item in the TCB... */
	"	ldr r2, =0xe000ed9c				\n" /* Region Base Address register. */
	"	ldmia r1!, {r4-r7}				\n" /* Read 2 sets of MPU registers. */
	"	stmia r2!, {r4-r7}				\n" /* Write 2 sets of MPU registers. */
/* END of MPU configuration */
	"	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
	"	ldr r0, [r1]						\n"
	"										\n"
	"   ldr r3 , [r0, #-4]            		\n"
	"	msr control, r3						\n"
	"										\n"
	"	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */

	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
	"	it eq								\n"
	"	vldmiaeq r0!, {s16-s31}				\n"
	"										\n"
	"	msr psp, r0							\n"
	"	isb									\n"
	"										\n"
	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata workaround. */
		#if WORKAROUND_PMU_CM001 == 1
	"			push { r14 }				\n"
	"			pop { pc }					\n"
		#endif
	#endif
	"										\n"
	"	bx r14								\n"
	"										\n"
	"	.align 2							\n"
	"pxCurrentTCBConst: .word pxCurrentTCB	\n"
	::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}

/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
uint32_t ulDummy;

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* Pend a context switch. */
			*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
static void prvSetupTimerInterrupt( void )
{

	/* Configure SysTick to interrupt at the requested rate. */
	*(portNVIC_SYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	*(portNVIC_SYSTICK_CTRL) = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;


}
/*-----------------------------------------------------------*/

static void prvSetupFPU( void )
{
	/* set CP10 and CP11 Full Access */
	*portSCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2));

	/* switch off FPU auto context save mechanism */
	*portFPU_FPCCR &= ~(0xc0000000);
}
static void prvSetupMPU( void )
{
	/* Check the expected MPU is present. */
	if( *portMPU_TYPE == portEXPECTED_MPU_TYPE_VALUE )
	{

		// First setup the entire flash, ccm and sram for unprivileged read write access.
		// The sub region 0, 3, 5, 6 and 7 are disabled. This reenable the privilege mode for reserved region and null pointer.

        *portMPU_REGION_BASE_ADDRESS =	( (unsigned long )0x00000000 ) | /* Base address. */
										( portMPU_REGION_VALID ) |
										( portMPU_REGION_0 );

		*portMPU_REGION_ATTRIBUTE =		( portMPU_AP_REGION_P_RW_U_RW ) |
										( portMPU_AS_REGION_BUFFERABLE |  portMPU_AS_REGION_CACHEABLE | portMPU_AS_REGION_SHAREABLE) |
										( prvGetMPURegionSizeSetting( 0x3FFFFFFF ) )|
										( portMPU_AS_REGION_ZONE_DISABLE_0 |  portMPU_AS_REGION_ZONE_DISABLE_5 | portMPU_AS_REGION_ZONE_DISABLE_6 | portMPU_AS_REGION_ZONE_DISABLE_7 ) |
										( portMPU_REGION_ENABLE );

		// Enable the IO memory in privileged and unprivileged read write.
        *portMPU_REGION_BASE_ADDRESS =	( (unsigned long )0x40000000 ) | /* Base address. */
										( portMPU_REGION_VALID ) |
										( portMPU_REGION_1 );

		*portMPU_REGION_ATTRIBUTE =		( portMPU_AP_REGION_P_RW_U_RW ) |
										( portMPU_AS_REGION_BUFFERABLE |  portMPU_AS_REGION_CACHEABLE | portMPU_AS_REGION_SHAREABLE) |
										( prvGetMPURegionSizeSetting( 0x1FFFFFFF ) )|
										( portMPU_REGION_ENABLE );


		// Enable the BBRAM memory in privileged read write and unprivileged to read only.
        *portMPU_REGION_BASE_ADDRESS =	( (unsigned long )0x40024000 ) | /* Base address. */
										( portMPU_REGION_VALID ) |
										( portMPU_REGION_2 );

		*portMPU_REGION_ATTRIBUTE =		( portMPU_AP_REGION_P_RW_U_RO ) |
										( portMPU_AS_REGION_BUFFERABLE |  portMPU_AS_REGION_CACHEABLE | portMPU_AS_REGION_SHAREABLE) |
										( prvGetMPURegionSizeSetting( 4096 ) )|
										( portMPU_REGION_ENABLE );

		/* Enable the memory fault exception. */
		*portNVIC_SYS_CTRL_STATE |= portNVIC_MEM_FAULT_ENABLE;

		/* Enable the MPU with the background region configured. */
		*portMPU_CTRL |= ( portMPU_ENABLE | portMPU_BACKGROUND_ENABLE );
	}
}
/*-----------------------------------------------------------*/

static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes )
{
uint32_t ulRegionSize, ulReturnValue = 4;

	/* 32 is the smallest region size, 31 is the largest valid value for
	ulReturnValue. */
	for( ulRegionSize = 32UL; ulReturnValue < 31UL; ( ulRegionSize <<= 1UL ) )
	{
		if( ulActualSizeInBytes <= ulRegionSize )
		{
			break;
		}
		else
		{
			ulReturnValue++;
		}
	}

	/* Shift the code by one before returning so it can be written directly
	into the the correct bit position of the attribute register. */
	return ( ulReturnValue << 1UL );
}
/*-----------------------------------------------------------*/

BaseType_t xIsPrivileged( void ) /* __attribute__ (( naked )) */
{
	__asm volatile
	(
	"	mrs r0, control							\n" /* r0 = CONTROL. */
	"	tst r0, #1								\n" /* Perform r0 & 1 (bitwise AND) and update the conditions flag. */
	"	ite ne									\n"
	"	movne r0, #0							\n" /* CONTROL[0]!=0. Return false to indicate that the processor is not privileged. */
	"	moveq r0, #1							\n" /* CONTROL[0]==0. Return true to indicate that the processor is privileged. */
	"	bx lr									\n" /* Return. */
	"											\n"
	"	.align 4								\n"
	::: "r0", "memory"
	);
}
/*-----------------------------------------------------------*/

void vResetPrivilege( void ) /* __attribute__ (( naked )) */
{
	__asm volatile
	(
	"	mrs r0, control							\n" /* r0 = CONTROL. */
	"	orr r0, #1								\n" /* r0 = r0 | 1. */
	"	msr control, r0							\n" /* CONTROL = r0. */
	"	bx lr									\n" /* Return to the caller. */
	:::"r0", "memory"
	);
}
/*-----------------------------------------------------------*/

void setTaskMPU(xMPU_SETTINGS *xMPUSettings, StackType_t *pxBottomOfStack, uint32_t usStackDepth);

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t usStackDepth )
{
	(void )xRegions;
	setTaskMPU(xMPUSettings, pxBottomOfStack, usStackDepth); //-1 here return the TCB (task handle). This is a hack.
}
/*-----------------------------------------------------------*/
#if 0
BaseType_t MPU_xTaskGenericCreate( TaskFunction_t pvTaskCode, const char * const pcName, uint16_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask, StackType_t *puxStackBuffer, const MemoryRegion_t * const xRegions )
{
BaseType_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	//disable the MPU because the task create will write 0xA5 in the stack of the task.
	__asm volatile ("dmb");
	uint32_t currentMPUConfiguration = *portMPU_CTRL;
	//disable the MPU if enabled
	*portMPU_CTRL &= ~0x01;

	xReturn = xTaskGenericCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, puxStackBuffer, xRegions );

	*portMPU_CTRL = currentMPUConfiguration;
	__asm volatile ("dsb");
	__asm volatile ("isb");
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

void MPU_vTaskAllocateMPURegions( TaskHandle_t xTask, const MemoryRegion_t * const xRegions )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vTaskAllocateMPURegions( xTask, xRegions );
	portYIELD_WITHIN_API();
	portRESET_PRIVILEGE( xRunningPrivileged );

}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )
	void MPU_vTaskDelete( TaskHandle_t pxTaskToDelete )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskDelete( pxTaskToDelete );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelayUntil == 1 )
	void MPU_vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, TickType_t xTimeIncrement )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskDelayUntil( pxPreviousWakeTime, xTimeIncrement );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )
	void MPU_vTaskDelay( TickType_t xTicksToDelay )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskDelay( xTicksToDelay );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )
	UBaseType_t MPU_uxTaskPriorityGet( TaskHandle_t pxTask )
	{
	UBaseType_t uxReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		uxReturn = uxTaskPriorityGet( pxTask );
        portRESET_PRIVILEGE( xRunningPrivileged );
		return uxReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )
	void MPU_vTaskPrioritySet( TaskHandle_t pxTask, UBaseType_t uxNewPriority )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskPrioritySet( pxTask, uxNewPriority );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_eTaskGetState == 1 )
	eTaskState MPU_eTaskGetState( TaskHandle_t pxTask )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();
	eTaskState eReturn;

		eReturn = eTaskGetState( pxTask );
        portRESET_PRIVILEGE( xRunningPrivileged );
		return eReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )
	TaskHandle_t MPU_xTaskGetIdleTaskHandle( void )
	{
	TaskHandle_t xReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xTaskGetIdleTaskHandle();
        portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )
	void MPU_vTaskSuspend( TaskHandle_t pxTaskToSuspend )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskSuspend( pxTaskToSuspend );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )
	void MPU_vTaskResume( TaskHandle_t pxTaskToResume )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskResume( pxTaskToResume );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

void MPU_vTaskSuspendAll( void )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vTaskSuspendAll();
    portRESET_PRIVILEGE( xRunningPrivileged );
}
/*-----------------------------------------------------------*/

BaseType_t MPU_xTaskResumeAll( void )
{
BaseType_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xTaskResumeAll();
    portRESET_PRIVILEGE( xRunningPrivileged );
    return xReturn;
}
/*-----------------------------------------------------------*/

TickType_t MPU_xTaskGetTickCount( void )
{
TickType_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xTaskGetTickCount();
    portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t MPU_uxTaskGetNumberOfTasks( void )
{
UBaseType_t uxReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	uxReturn = uxTaskGetNumberOfTasks();
    portRESET_PRIVILEGE( xRunningPrivileged );
	return uxReturn;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
	void MPU_vTaskList( char *pcWriteBuffer )
	{
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskList( pcWriteBuffer );
		portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	void MPU_vTaskGetRunTimeStats( char *pcWriteBuffer )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskGetRunTimeStats( pcWriteBuffer );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	void MPU_vTaskSetApplicationTaskTag( TaskHandle_t xTask, TaskHookFunction_t pxTagValue )
	{
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vTaskSetApplicationTaskTag( xTask, pxTagValue );
        portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	TaskHookFunction_t MPU_xTaskGetApplicationTaskTag( TaskHandle_t xTask )
	{
	TaskHookFunction_t xReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xTaskGetApplicationTaskTag( xTask );
        portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	BaseType_t MPU_xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter )
	{
	BaseType_t xReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xTaskCallApplicationTaskHook( xTask, pvParameter );
        portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
	UBaseType_t MPU_uxTaskGetSystemState( TaskStatus_t *pxTaskStatusArray, UBaseType_t uxArraySize, uint32_t *pulTotalRunTime )
	{
	UBaseType_t uxReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		uxReturn = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, pulTotalRunTime );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return uxReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )
	UBaseType_t MPU_uxTaskGetStackHighWaterMark( TaskHandle_t xTask )
	{
	UBaseType_t uxReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		uxReturn = uxTaskGetStackHighWaterMark( xTask );
        portRESET_PRIVILEGE( xRunningPrivileged );
		return uxReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetCurrentTaskHandle == 1 )
	TaskHandle_t MPU_xTaskGetCurrentTaskHandle( void )
	{
	TaskHandle_t xReturn;
    BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xTaskGetCurrentTaskHandle();
        portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetSchedulerState == 1 )
BaseType_t MPU_xTaskGetSchedulerState( void )
{
BaseType_t xReturn;
//BaseType_t xRunningPrivileged = prvRaisePrivilege(); //no Need to be in privilege mode to read the internal var.

	xReturn = xTaskGetSchedulerState();
//	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
#endif
/*-----------------------------------------------------------*/

QueueHandle_t MPU_xQueueGenericCreate( UBaseType_t uxQueueLength, UBaseType_t uxItemSize, uint8_t ucQueueType )
{
QueueHandle_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xQueueGenericCreate( uxQueueLength, uxItemSize, ucQueueType );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueGenericReset( QueueHandle_t pxQueue, BaseType_t xNewQueue )
{
BaseType_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xQueueGenericReset( pxQueue, xNewQueue );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, BaseType_t xCopyPosition )
{
BaseType_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xQueueGenericSend( xQueue, pvItemToQueue, xTicksToWait, xCopyPosition );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t MPU_uxQueueMessagesWaiting( const QueueHandle_t pxQueue )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();
UBaseType_t uxReturn;

	uxReturn = uxQueueMessagesWaiting( pxQueue );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return uxReturn;
}
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueGenericReceive( QueueHandle_t pxQueue, void * const pvBuffer, TickType_t xTicksToWait, BaseType_t xJustPeeking )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();
BaseType_t xReturn;

	xReturn = xQueueGenericReceive( pxQueue, pvBuffer, xTicksToWait, xJustPeeking );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueuePeekFromISR( QueueHandle_t pxQueue, void * const pvBuffer )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();
BaseType_t xReturn;

	xReturn = xQueuePeekFromISR( pxQueue, pvBuffer );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

void* MPU_xQueueGetMutexHolder( QueueHandle_t xSemaphore )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();
void * xReturn;

	xReturn = ( void * ) xQueueGetMutexHolder( xSemaphore );
	portRESET_PRIVILEGE( xRunningPrivileged );
	return xReturn;
}
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )
	QueueHandle_t MPU_xQueueCreateMutex( void )
	{
    QueueHandle_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueCreateMutex( queueQUEUE_TYPE_MUTEX );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if configUSE_COUNTING_SEMAPHORES == 1
	QueueHandle_t MPU_xQueueCreateCountingSemaphore( UBaseType_t uxCountValue, UBaseType_t uxInitialCount )
	{
    QueueHandle_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueCreateCountingSemaphore( uxCountValue, uxInitialCount );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )
	BaseType_t MPU_xQueueTakeMutexRecursive( QueueHandle_t xMutex, TickType_t xBlockTime )
	{
	BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueTakeMutexRecursive( xMutex, xBlockTime );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )
	BaseType_t MPU_xQueueGiveMutexRecursive( QueueHandle_t xMutex )
	{
	BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueGiveMutexRecursive( xMutex );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )
	QueueSetHandle_t MPU_xQueueCreateSet( UBaseType_t uxEventQueueLength )
	{
	QueueSetHandle_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueCreateSet( uxEventQueueLength );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )
	QueueSetMemberHandle_t MPU_xQueueSelectFromSet( QueueSetHandle_t xQueueSet, TickType_t xBlockTimeTicks )
	{
	QueueSetMemberHandle_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueSelectFromSet( xQueueSet, xBlockTimeTicks );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )
	BaseType_t MPU_xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet )
	{
	BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueAddToSet( xQueueOrSemaphore, xQueueSet );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )
	BaseType_t MPU_xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet )
	{
	BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueRemoveFromSet( xQueueOrSemaphore, xQueueSet );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if configUSE_ALTERNATIVE_API == 1
	BaseType_t MPU_xQueueAltGenericSend( QueueHandle_t pxQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, BaseType_t xCopyPosition )
	{
   	BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = 	BaseType_t xQueueAltGenericSend( pxQueue, pvItemToQueue, xTicksToWait, xCopyPosition );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if configUSE_ALTERNATIVE_API == 1
	BaseType_t MPU_xQueueAltGenericReceive( QueueHandle_t pxQueue, void * const pvBuffer, TickType_t xTicksToWait, BaseType_t xJustPeeking )
	{
    BaseType_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		xReturn = xQueueAltGenericReceive( pxQueue, pvBuffer, xTicksToWait, xJustPeeking );
		portRESET_PRIVILEGE( xRunningPrivileged );
		return xReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if configQUEUE_REGISTRY_SIZE > 0
	void MPU_vQueueAddToRegistry( QueueHandle_t xQueue, char *pcName )
	{
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

		vQueueAddToRegistry( xQueue, pcName );

		portRESET_PRIVILEGE( xRunningPrivileged );
	}
#endif
/*-----------------------------------------------------------*/

void MPU_vQueueDelete( QueueHandle_t xQueue )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vQueueDelete( xQueue );

	portRESET_PRIVILEGE( xRunningPrivileged );
}
/*-----------------------------------------------------------*/

void *MPU_pvPortMalloc( size_t xSize )
{
void *pvReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	pvReturn = pvPortMalloc( xSize );

	portRESET_PRIVILEGE( xRunningPrivileged );

	return pvReturn;
}
/*-----------------------------------------------------------*/

void *MPU_pvPortMemalign( size_t bondary,  size_t xSize )
{
void *pvReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	pvReturn = pvPortMemalign(bondary, xSize );

	portRESET_PRIVILEGE( xRunningPrivileged );

	return pvReturn;
}
/*-----------------------------------------------------------*/

void MPU_vPortFree( void *pv )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vPortFree( pv );

	portRESET_PRIVILEGE( xRunningPrivileged );
}
/*-----------------------------------------------------------*/

void *MPU_pvPortRealloc( void *p, size_t xWantedSize )
{
	void *pvReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	pvReturn = pvPortRealloc( p, xWantedSize );

	portRESET_PRIVILEGE( xRunningPrivileged );

	return pvReturn;
}
/*-----------------------------------------------------------*/

void MPU_vPortInitialiseBlocks( void )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vPortInitialiseBlocks();

	portRESET_PRIVILEGE( xRunningPrivileged );
}
/*-----------------------------------------------------------*/

size_t MPU_xPortGetFreeHeapSize( void )
{
size_t xReturn;
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xPortGetFreeHeapSize();

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}

EventGroupHandle_t MPU_xEventGroupCreate( void )
{
	EventGroupHandle_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xEventGroupCreate();

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}
EventBits_t MPU_xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait )
{
	EventBits_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xEventGroupWaitBits(xEventGroup,uxBitsToWaitFor,xClearOnExit,xWaitForAllBits,xTicksToWait);

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}
EventBits_t MPU_xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear )
{
	EventBits_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xEventGroupClearBits(xEventGroup,uxBitsToClear);

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}
EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet )
{
	EventBits_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xEventGroupSetBits(xEventGroup,uxBitsToSet);

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}
EventBits_t MPU_xEventGroupSync( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, const EventBits_t uxBitsToWaitFor, TickType_t xTicksToWait )
{
	EventBits_t xReturn;
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	xReturn = xEventGroupSync(xEventGroup,uxBitsToSet, uxBitsToWaitFor, xTicksToWait);

	portRESET_PRIVILEGE( xRunningPrivileged );

	return xReturn;
}
void MPU_vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
	BaseType_t xRunningPrivileged = prvRaisePrivilege();

	vEventGroupDelete(xEventGroup);

	portRESET_PRIVILEGE( xRunningPrivileged );
}




/* Functions that the application writer wants to execute in privileged mode
can be defined in application_defined_privileged_functions.h.  The functions
must take the same format as those above whereby the privilege state on exit
equals the privilege state on entry.  For example:

void MPU_FunctionName( [parameters ] )
{
BaseType_t xRunningPrivileged = prvRaisePrivilege();

	FunctionName( [parameters ] );

	portRESET_PRIVILEGE( xRunningPrivileged );
}
*/

#if configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS == 1
	#include "application_defined_privileged_functions.h"
#endif

#endif
