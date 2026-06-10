// badprog.com
/**
 * @file    main.c
 * @brief   FreeRTOS tasks and queues on STM32F3Discovery
 *
 * Bare metal C with FreeRTOS. No HAL, no CubeMX.
 *
 * We are going to run three concurrent tasks that communicate via a queue:
 *
 *   Task 1 : sensor_task  (priority 3, HIGH)
 *            Reads X, Y, Z acceleration from the LSM303DLHC via I2C every
 *            100 ms and sends the data into a queue.
 *
 *   Task 2 : display_task (priority 2, NORMAL)
 *            Waits on the queue, formats the values as text, and sends them
 *            over UART on PA9 at 115200 baud.
 *
 *   Task 3 : led_task     (priority 1, LOW)
 *            Runs independently. Lights one of the 8 compass LEDs on
 *            PE8..PE15 in rotation every 250 ms to show the scheduler
 *            is running even when the sensor is idle.
 *
 * The queue decouples production (sensor read) from consumption (UART print).
 * If display_task is slow, sensor_task does not block: it either fills the
 * queue or drops the oldest item (configurable). This is the fundamental
 * pattern for safe inter-task communication in FreeRTOS.
 *
 * Hardware:
 *   I2C1  : PB6 (SCL), PB7 (SDA) -- LSM303DLHC accelerometer
 *   USART1: PA9 (TX)              -- UART output at 115200 baud
 *   GPIOE : PE8..PE15             -- compass LEDs
 *
 * CPU clock: 8 MHz (default HSI, no PLL)
 *
 * -------------------------------------------------------------------------
 * FREERTOS CONCEPTS USED IN THIS FILE
 * -------------------------------------------------------------------------
 *
 * Task:
 *   A function that runs as an independent thread of execution. Each task
 *   has its own stack, its own priority, and its own state (running, ready,
 *   blocked, suspended). The FreeRTOS scheduler decides which task runs.
 *   Created with xTaskCreate(). Never returns (must loop forever or call
 *   vTaskDelete(NULL) to delete itself).
 *
 * Scheduler:
 *   The kernel component that decides which task runs at any given moment.
 *   With preemption enabled (configUSE_PREEMPTION=1), it runs at every tick
 *   interrupt and always picks the highest-priority ready task.
 *   Started with vTaskStartScheduler() -- this call never returns.
 *
 * Queue:
 *   A FIFO buffer that allows tasks to exchange data safely. The sender
 *   copies data IN, the receiver copies data OUT. The queue owns the data
 *   while it is in transit -- no pointers, no shared memory, no race
 *   conditions. Created with xQueueCreate(). Send with xQueueSend().
 *   Receive with xQueueReceive().
 *
 * Blocking:
 *   A task that calls xQueueReceive() with a non-zero timeout will block
 *   (give up the CPU) if the queue is empty. The scheduler immediately runs
 *   the next ready task. The blocked task is woken up when data arrives.
 *   This is more efficient than busy-waiting (polling).
 *
 * vTaskDelay():
 *   Puts the calling task to sleep for a given number of ticks. The task
 *   is removed from the ready list and the CPU is given to other tasks.
 *   pdMS_TO_TICKS(ms) converts milliseconds to ticks using configTICK_RATE_HZ.
 *
 * Task priority:
 *   Higher number = higher priority. The scheduler always runs the highest-
 *   priority ready task. If sensor_task (priority 3) becomes ready while
 *   led_task (priority 1) is running, the scheduler immediately preempts
 *   led_task and switches to sensor_task.
 *
 * -------------------------------------------------------------------------
 * PERIPHERAL GLOSSARY (same as previous exos, repeated for completeness)
 * -------------------------------------------------------------------------
 *
 * RCC  : Reset and Clock Control. Must enable clock for each peripheral
 *        before using it.
 * GPIO : General Purpose Input/Output. Configured as Output, Input, or AF.
 * USART: Universal Asynchronous Receiver Transmitter. Serial communication.
 * I2C  : Inter-Integrated Circuit. Two-wire bus (SCL + SDA).
 * -------------------------------------------------------------------------
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f303xc.h"

// ---------------------------------------------------------------------------
// Queue configuration
// ---------------------------------------------------------------------------

// Number of items the queue can hold at once.
// If sensor_task produces faster than display_task consumes, the queue
// acts as a buffer. When full, xQueueSend() either blocks or returns
// errQUEUE_FULL depending on the timeout given.
#define QUEUE_LENGTH    5

// The queue transports AccelData structs (6 bytes each).
// The kernel copies the entire struct into the queue storage -- the sender
// does not need to keep the original alive after xQueueSend() returns.
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData;

// Queue handle: a pointer to the internal queue structure managed by FreeRTOS.
// Declared globally so all tasks can access it.
// In a larger project, pass it as a task parameter instead.
static QueueHandle_t xAccelQueue = NULL;

// ---------------------------------------------------------------------------
// Task stack sizes (in words, not bytes -- 1 word = 4 bytes on Cortex-M4)
// ---------------------------------------------------------------------------

// sensor_task: runs I2C transactions and a small local buffer.
// 256 words = 1024 bytes. Generous to account for I2C helper call depth.
#define SENSOR_TASK_STACK_SIZE  256

// display_task: runs snprintf and uart_print. snprintf needs extra stack
// for its internal formatting buffers.
// 256 words = 1024 bytes.
#define DISPLAY_TASK_STACK_SIZE 256

// led_task: very simple loop, minimal stack needed.
// 128 words = 512 bytes = configMINIMAL_STACK_SIZE.
#define LED_TASK_STACK_SIZE     128

// ---------------------------------------------------------------------------
// UART -- USART1 on PA9, 115200 baud
// ---------------------------------------------------------------------------

static void uart_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 : AF7 (USART1 TX)
    GPIOA->MODER  &= ~(0x3UL << 18);
    GPIOA->MODER  |=  (0x2UL << 18);
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |=  (0x7UL << 4);

    // BRR = fPCLK / baudrate = 8000000 / 115200 = 69
    USART1->BRR = 69;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart_print(const char *str)
{
    while (*str) {
        while (!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = (uint32_t)(*str++);
    }
}

// ---------------------------------------------------------------------------
// LEDs -- PE8..PE15 as push-pull outputs
// ---------------------------------------------------------------------------

static void leds_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_IOPEEN;
    GPIOE->MODER &= ~(0xFFFFUL << 16);
    GPIOE->MODER |=  (0x5555UL << 16);
    GPIOE->BSRR   = LED_ALL_PINS_OFF;
}

// ---------------------------------------------------------------------------
// I2C1 -- PB6 (SCL), PB7 (SDA), 100 kHz
// ---------------------------------------------------------------------------

static void i2c1_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6 and PB7 : AF4 (I2C1), open-drain, pull-up, high speed
    GPIOB->MODER   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->MODER   |=  (0x2UL << 12) | (0x2UL << 14);
    GPIOB->OTYPER  |=  (1UL << 6) | (1UL << 7);
    GPIOB->OSPEEDR |=  (0x3UL << 12) | (0x3UL << 14);
    GPIOB->PUPDR   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->PUPDR   |=  (0x1UL << 12) | (0x1UL << 14);
    GPIOB->AFR[0]  &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB->AFR[0]  |=  (0x4UL << 24) | (0x4UL << 28);

    // TIMINGR for 100 kHz Standard Mode with HSI at 8 MHz (ST AN4235)
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x10420F13U;
    I2C1->CR1 |= I2C_CR1_PE;
}

// I2C low-level helpers (same as exo06)
static void i2c_wait_busy(void)  { while (I2C1->ISR & I2C_ISR_BUSY); }
static void i2c_wait_tc(void)    { while (!(I2C1->ISR & I2C_ISR_TC)); }
static void i2c_write_byte(uint8_t b) { while (!(I2C1->ISR & I2C_ISR_TXIS)); I2C1->TXDR = b; }
static uint8_t i2c_read_byte(void)   { while (!(I2C1->ISR & I2C_ISR_RXNE)); return (uint8_t)I2C1->RXDR; }

static void i2c_start_write(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

static void i2c_start_read(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | I2C_CR2_RD_WRN
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;
}

static void i2c_stop(void) { I2C1->CR2 |= I2C_CR2_STOP; while (I2C1->ISR & I2C_ISR_BUSY); }

// ---------------------------------------------------------------------------
// LSM303DLHC -- accelerometer at I2C address 0x19
// ---------------------------------------------------------------------------

#define LSM303_ADDR         0x19U
#define LSM303_CTRL_REG1_A  0x20U
#define LSM303_CTRL_REG4_A  0x23U
#define LSM303_OUT_X_L_A    0x28U

static void lsm303_write_reg(uint8_t reg, uint8_t val)
{
    i2c_wait_busy();
    i2c_start_write(LSM303_ADDR, 2);
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_wait_tc();
    i2c_stop();
}

static void lsm303_read_xyz(AccelData *data)
{
    uint8_t buf[6];

    // Write phase: send register address with auto-increment bit set
    i2c_wait_busy();
    i2c_start_write(LSM303_ADDR, 1);
    i2c_write_byte(LSM303_OUT_X_L_A | 0x80U);
    i2c_wait_tc();

    // Read phase: read 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    i2c_start_read(LSM303_ADDR, 6);
    for (uint8_t i = 0; i < 6; i++) {
        buf[i] = i2c_read_byte();
    }

    // 12-bit values are left-aligned in a 16-bit word: shift right by 4
    data->x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4;
    data->y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]) >> 4;
    data->z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]) >> 4;
}

static void lsm303_init(void)
{
    // CTRL_REG1_A : ODR=100Hz, all axes enabled
    lsm303_write_reg(LSM303_CTRL_REG1_A, 0x57U);
    // CTRL_REG4_A : +/-2g full scale, high resolution
    lsm303_write_reg(LSM303_CTRL_REG4_A, 0x08U);
}

// ---------------------------------------------------------------------------
// Task 1: sensor_task -- reads LSM303 and sends to queue (priority 3)
//
// This is the highest-priority task. It wakes up every 100 ms, reads the
// accelerometer via I2C, and sends the AccelData struct into xAccelQueue.
//
// xQueueSend() with timeout 0: if the queue is full, the item is dropped
// rather than blocking sensor_task. Dropping is acceptable here because
// display_task will catch the next sample 100 ms later. If we blocked
// instead, sensor_task would miss its timing deadline.
// ---------------------------------------------------------------------------

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;

    AccelData data;

    while (1) {
        // Read accelerometer
        lsm303_read_xyz(&data);

        // Send to queue -- drop if full (timeout = 0 means do not block)
        xQueueSend(xAccelQueue, &data, 0);

        // Sleep for 100 ms. vTaskDelay() blocks this task and lets other
        // tasks run. pdMS_TO_TICKS converts ms to scheduler ticks.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------
// Task 2: display_task -- receives from queue and prints via UART (priority 2)
//
// This task blocks indefinitely on xQueueReceive() waiting for sensor data.
// When sensor_task sends an AccelData, display_task is unblocked by the
// scheduler and formats the values as a string for UART output.
//
// portMAX_DELAY as timeout means: block forever until data arrives.
// This is safe here because sensor_task always produces data eventually.
// ---------------------------------------------------------------------------

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    AccelData data;
    char buf[48];

    uart_print("FreeRTOS tasks and queues\r\n");
    uart_print("---\r\n");
    uart_print("     X        Y        Z\r\n");

    while (1) {
        // Block until an AccelData item is available in the queue.
        // When sensor_task calls xQueueSend(), the scheduler wakes us up.
        if (xQueueReceive(xAccelQueue, &data, portMAX_DELAY) == pdTRUE) {

            // Format X, Y, Z into a fixed-width string
            // We avoid printf to keep binary size small (no float support
            // pulled in from newlib).
            int n = 0;
            int16_t vals[3] = {data.x, data.y, data.z};

            for (int axis = 0; axis < 3; axis++) {
                int16_t v = vals[axis];
                buf[n++] = (v < 0) ? '-' : ' ';
                if (v < 0) v = -v;
                uint8_t started = 0;
                for (int16_t div = 1000; div >= 1; div /= 10) {
                    int16_t digit = v / div;
                    v %= div;
                    if (digit || started || div == 1) {
                        buf[n++] = '0' + (char)digit;
                        started = 1;
                    } else {
                        buf[n++] = ' ';
                    }
                }
                buf[n++] = ' '; buf[n++] = ' '; buf[n++] = ' ';
            }
            buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0';
            uart_print(buf);
        }
    }
}

// ---------------------------------------------------------------------------
// Task 3: led_task -- rotates one LED around PE8..PE15 (priority 1)
//
// This is the lowest-priority task. It runs only when sensor_task and
// display_task are both blocked (sleeping or waiting on the queue).
// The LED rotation is a visual proof that the scheduler is running and
// that low-priority tasks do get CPU time.
//
// Because this task has lower priority than sensor_task (3) and
// display_task (2), it will be preempted immediately when either of
// those tasks becomes ready. You will see the LED rotation pause briefly
// every 100 ms when sensor_task wakes up.
// ---------------------------------------------------------------------------

static void led_task(void *pvParameters)
{
    (void)pvParameters;

    uint8_t led = 8;   // current LED pin (PE8..PE15)

    while (1) {
        // Turn all LEDs off
        GPIOE->BSRR = LED_ALL_PINS_OFF;

        // Turn on current LED
        GPIOE->BSRR = (1UL << led);

        // Advance to next LED (wrap from PE15 back to PE8)
        led++;
        if (led > 15) led = 8;

        // Sleep 250 ms
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void)
{
    // Initialise peripherals before starting the scheduler.
    // After vTaskStartScheduler() is called, the scheduler takes over and
    // main() never returns. All peripheral access from that point happens
    // inside tasks.
    uart_init();
    leds_init();
    i2c1_init();
    lsm303_init();

    // Create the queue.
    // xQueueCreate(length, item_size):
    //   length    : max number of items in the queue at once
    //   item_size : size in bytes of one item (the kernel copies this many
    //               bytes on each send/receive)
    // Returns NULL if the heap is exhausted -- check for this.
    xAccelQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AccelData));

    if (xAccelQueue == NULL) {
        // Queue creation failed : not enough heap.
        // Increase configTOTAL_HEAP_SIZE in FreeRTOSConfig.h.
        uart_print("ERROR: queue creation failed\r\n");
        while (1);
    }

    // Create tasks.
    // xTaskCreate(function, name, stack_depth, param, priority, handle):
    //   function    : the task function (must loop forever)
    //   name        : string for debugging (vTaskList, stack overflow hook)
    //   stack_depth : stack size in words (1 word = 4 bytes on Cortex-M4)
    //   param       : passed as pvParameters to the task function (NULL here)
    //   priority    : 0 = lowest (idle), configMAX_PRIORITIES-1 = highest
    //   handle      : pointer to store the task handle (NULL if not needed)
    xTaskCreate(sensor_task,  "sensor",  SENSOR_TASK_STACK_SIZE,  NULL, 3, NULL);
    xTaskCreate(display_task, "display", DISPLAY_TASK_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(led_task,     "leds",    LED_TASK_STACK_SIZE,     NULL, 1, NULL);

    // Start the FreeRTOS scheduler.
    // This call initialises SysTick, enables interrupts, and starts running
    // the highest-priority ready task. It never returns under normal operation.
    // If it does return, it means there was not enough heap for the idle task.
    vTaskStartScheduler();

    // Should never reach here
    uart_print("ERROR: scheduler returned\r\n");
    while (1);

    return 0;
}
