// badprog.com
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// ---------------------------------------------------------------------------
// FreeRTOSConfig.h : kernel configuration for STM32F3Discovery (Cortex-M4)
//
// This file is the single place where the FreeRTOS kernel is tuned for the
// target hardware and application requirements.
//
// Every macro here is read by the FreeRTOS kernel source files at compile
// time. Changing a value here changes the behaviour of the entire kernel.
//
// Reference: https://www.freertos.org/a00110.html
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SCHEDULER BEHAVIOUR
// ---------------------------------------------------------------------------

// configUSE_PREEMPTION
//   1 = preemptive scheduler (default and recommended)
//       The kernel can switch tasks at any tick interrupt, even if the
//       running task has not explicitly yielded. Higher-priority tasks
//       always get the CPU immediately when they become ready.
//   0 = cooperative scheduler
//       Tasks run until they call taskYIELD() or block.
#define configUSE_PREEMPTION 1

// configUSE_TIME_SLICING
//   1 = tasks of equal priority share CPU time in round-robin fashion.
//       Each task gets one tick period before the scheduler moves on.
//   0 = a task runs until it blocks or yields even if peers are ready.
#define configUSE_TIME_SLICING 1

// configUSE_PORT_OPTIMISED_TASK_SELECTION
//   1 = use the hardware CLZ instruction to find the highest-priority ready
//       task in O(1). Available on Cortex-M4. Limits configMAX_PRIORITIES
//       to 32.
//   0 = generic C implementation, any number of priorities.
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

// configUSE_TICKLESS_IDLE
//   0 = tick runs continuously (simpler, no power saving).
//       We keep it off for learning purposes.
#define configUSE_TICKLESS_IDLE 0

// ---------------------------------------------------------------------------
// CLOCK AND TICK
// ---------------------------------------------------------------------------

// configCPU_CLOCK_HZ : frequency of the CPU clock in Hz.
// The STM32F3Discovery runs on the default HSI oscillator at 8 MHz.
// Used to calculate the SysTick reload value.
#define configCPU_CLOCK_HZ (8000000UL)

// configTICK_RATE_HZ : how many times per second the tick interrupt fires.
// 1000 Hz = 1 ms tick period. Standard choice for most applications.
// Gives 1 ms resolution for vTaskDelay() and timeouts.
#define configTICK_RATE_HZ (1000)

// configTICK_TYPE_WIDTH_IN_BITS : width of the tick counter.
// TICK_TYPE_WIDTH_32_BITS is correct for Cortex-M4.
// Use this instead of the deprecated configUSE_16_BIT_TICKS (removed in V11).
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

// ---------------------------------------------------------------------------
// TASK PRIORITIES AND STACK
// ---------------------------------------------------------------------------

// configMAX_PRIORITIES : number of distinct priority levels (0 = lowest).
// With configUSE_PORT_OPTIMISED_TASK_SELECTION=1 this must be <= 32.
// 5 levels for this project:
//   0 = idle (used internally by FreeRTOS)
//   1 = low   (led_task)
//   2 = normal (display_task)
//   3 = high  (sensor_task)
//   4 = reserved for future ISR-deferred tasks
#define configMAX_PRIORITIES (5)

// configMINIMAL_STACK_SIZE : stack size in words for the idle task.
// 128 words = 512 bytes. Also used as a baseline for other tasks.
#define configMINIMAL_STACK_SIZE (128)

// configTOTAL_HEAP_SIZE : total FreeRTOS heap size in bytes.
// The STM32F303VCT6 has 40 KB of SRAM. We reserve 10 KB for FreeRTOS.
#define configTOTAL_HEAP_SIZE (10 * 1024)

// configMAX_TASK_NAME_LEN : max characters in a task name (with null
// terminator).
#define configMAX_TASK_NAME_LEN (16)

// ---------------------------------------------------------------------------
// HOOKS
// ---------------------------------------------------------------------------

// No idle hook and no tick hook for this project.
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0

// ---------------------------------------------------------------------------
// MEMORY ALLOCATION
// ---------------------------------------------------------------------------

// Dynamic allocation enabled (heap_4.c). Static allocation not needed here.
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0

// ---------------------------------------------------------------------------
// STACK OVERFLOW DETECTION
// ---------------------------------------------------------------------------

// configCHECK_FOR_STACK_OVERFLOW = 2 : fill stack with a known pattern at
// creation time and verify the last bytes at every context switch.
// The application must define vApplicationStackOverflowHook().
#define configCHECK_FOR_STACK_OVERFLOW 2

// ---------------------------------------------------------------------------
// MALLOC FAILURE HOOK
// ---------------------------------------------------------------------------

// Call vApplicationMallocFailedHook() when pvPortMalloc() returns NULL.
#define configUSE_MALLOC_FAILED_HOOK 1

// ---------------------------------------------------------------------------
// RUNTIME STATS AND DEBUGGING
// ---------------------------------------------------------------------------

#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
#define configGENERATE_RUN_TIME_STATS 0

// ---------------------------------------------------------------------------
// KERNEL FEATURES : enable only what we use
// ---------------------------------------------------------------------------

#define configUSE_QUEUE_SETS 0
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_TIMERS 0
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1

// ---------------------------------------------------------------------------
// CORTEX-M4 INTERRUPT PRIORITY CONFIGURATION
//
// The Cortex-M4 uses a priority numbering scheme where LOWER numbers mean
// HIGHER priority.
//
// FreeRTOS manages interrupts with priorities >=
// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. ISRs with numerically lower
// priority (higher urgency) must NOT call any FreeRTOS API functions.
//
// STM32F3 uses 4 bits for priority (16 levels: 0..15).
// ---------------------------------------------------------------------------

// Lowest interrupt priority (highest number = lowest urgency).
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

// Highest priority from which FreeRTOS API functions may be called safely.
// ISRs using xQueueSendFromISR() etc. must have priority >= 5.
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

// Shift values into the correct bit position for the Cortex-M NVIC.
// The 4-bit priority field lives in the top 4 bits of the 8-bit register.
#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - 4))

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - 4))

// ---------------------------------------------------------------------------
// MAP FREERTOS HANDLERS TO CORTEX-M VECTOR TABLE NAMES
//
// The FreeRTOS port uses SysTick, PendSV, and SVC internally.
// These defines create aliases so the linker finds the right functions
// under the names expected by the startup file and vector table.
// ---------------------------------------------------------------------------
#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler
#define xPortSysTickHandler SysTick_Handler

// ---------------------------------------------------------------------------
// OPTIONAL API FUNCTIONS : set to 1 to include in the build
// ---------------------------------------------------------------------------
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

#endif  // FREERTOS_CONFIG_H