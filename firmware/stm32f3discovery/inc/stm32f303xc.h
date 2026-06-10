// badprog.com
#ifndef STM32F303XC_H
#define STM32F303XC_H

#include <stdint.h>

typedef volatile uint32_t reg32_t;

// ---------------------------------------------------------------------------
// Bus base addresses
// ---------------------------------------------------------------------------
#define PERIPH_BASE         (0x40000000UL)
#define APB1PERIPH_BASE     (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x08000000UL)

// ---------------------------------------------------------------------------
// RCC - Reset and Clock Control
// ---------------------------------------------------------------------------
#define RCC_BASE            (AHB1PERIPH_BASE + 0x00001000UL)

typedef struct {
    reg32_t CR;
    reg32_t CFGR;
    reg32_t CIR;
    reg32_t APB2RSTR;
    reg32_t APB1RSTR;
    reg32_t AHBENR;
    reg32_t APB2ENR;
    reg32_t APB1ENR;
    reg32_t BDCR;
    reg32_t CSR;
    reg32_t AHBRSTR;
    reg32_t CFGR2;
    reg32_t CFGR3;
} RCC_TypeDef;

#define RCC                     ((RCC_TypeDef *) RCC_BASE)

// AHB peripheral clock enable bits
#define RCC_AHBENR_IOPAEN       (1UL << 17)   // GPIOA clock
#define RCC_AHBENR_IOPBEN       (1UL << 18)   // GPIOB clock (I2C1 pins PB6/PB7)
#define RCC_AHBENR_IOPEEN       (1UL << 21)   // GPIOE clock (LEDs PE8..PE15)

// APB2 peripheral clock enable bits
#define RCC_APB2ENR_USART1EN    (1UL << 14)   // USART1 clock (TX on PA9)

// APB1 peripheral clock enable bits
#define RCC_APB1ENR_I2C1EN      (1UL << 21)   // I2C1 clock (PB6=SCL, PB7=SDA)

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
#define GPIOA_BASE          (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE          (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOE_BASE          (AHB2PERIPH_BASE + 0x00001000UL)

typedef struct {
    reg32_t MODER;      // Mode register          - 2 bits per pin: 00=in 01=out 10=AF 11=analog
    reg32_t OTYPER;     // Output type register   - 1 bit per pin: 0=push-pull 1=open-drain
    reg32_t OSPEEDR;    // Output speed register  - 2 bits per pin: 00=low 01=med 10=high 11=vhigh
    reg32_t PUPDR;      // Pull-up/down register  - 2 bits per pin: 00=none 01=pull-up 10=pull-down
    reg32_t IDR;        // Input data register    - read pin state
    reg32_t ODR;        // Output data register   - set pin state
    reg32_t BSRR;       // Bit set/reset register - bits[15:0]=set, bits[31:16]=reset (atomic)
    reg32_t LCKR;       // Lock register
    reg32_t AFR[2];     // Alternate function registers - AFR[0]=pins 0..7, AFR[1]=pins 8..15
    reg32_t BRR;        // Bit reset register
} GPIO_TypeDef;

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)

// MODER field values (2 bits per pin)
#define GPIO_MODER_INPUT    (0x0UL)
#define GPIO_MODER_OUTPUT   (0x1UL)
#define GPIO_MODER_AF       (0x2UL)
#define GPIO_MODER_ANALOG   (0x3UL)

// OTYPER field values (1 bit per pin)
#define GPIO_OTYPER_PP      (0x0UL)   // Push-pull
#define GPIO_OTYPER_OD      (0x1UL)   // Open-drain (required for I2C)

// OSPEEDR field values (2 bits per pin)
// Values match RM0316 Table 36: 00=Low 01=Medium 10=High 11=Very High
#define GPIO_OSPEEDR_LOW    (0x0UL)
#define GPIO_OSPEEDR_MED    (0x1UL)
#define GPIO_OSPEEDR_HIGH   (0x2UL)
#define GPIO_OSPEEDR_VHIGH  (0x3UL)   // Very High - used for I2C pins

// PUPDR field values (2 bits per pin)
#define GPIO_PUPDR_NONE     (0x0UL)
#define GPIO_PUPDR_UP       (0x1UL)   // Pull-up
#define GPIO_PUPDR_DOWN     (0x2UL)   // Pull-down

// LEDs on GPIOE PE8..PE15
// LED_ALL_PINS_OFF: BSRR reset half (bits[31:16]) for PE8..PE15
// 0xFF << 8 = pin mask for PE8..PE15, then << 16 = BSRR reset half
#define LED_ALL_PINS_OFF    (0xFFUL << 24)

// ---------------------------------------------------------------------------
// USART1 - Universal Synchronous/Asynchronous Receiver Transmitter
// PA9 = TX (AF7), used for printf() output to logic analyzer / FTDI
// ---------------------------------------------------------------------------
#define USART1_BASE         (APB2PERIPH_BASE + 0x00003800UL)

typedef struct {
    reg32_t CR1;    // Control register 1 - UE (enable), TE (tx enable), RE (rx enable)
    reg32_t CR2;    // Control register 2 - stop bits
    reg32_t CR3;    // Control register 3 - DMA, flow control
    reg32_t BRR;    // Baud rate register - BRR = fPCLK / baudrate
    reg32_t GTPR;   // Guard time and prescaler
    reg32_t RTOR;   // Receiver timeout
    reg32_t RQR;    // Request register
    reg32_t ISR;    // Interrupt and status register
    reg32_t ICR;    // Interrupt flag clear register
    reg32_t RDR;    // Receive data register
    reg32_t TDR;    // Transmit data register
} USART_TypeDef;

#define USART1              ((USART_TypeDef *) USART1_BASE)

// CR1 bits
#define USART_CR1_UE        (1UL << 0)    // USART enable
#define USART_CR1_TE        (1UL << 3)    // Transmitter enable
#define USART_CR1_RE        (1UL << 2)    // Receiver enable

// ISR bits
#define USART_ISR_TXE       (1UL << 7)    // Transmit data register empty - safe to write next byte

// ---------------------------------------------------------------------------
// I2C1 - Inter-Integrated Circuit
// PB6 = SCL (AF4), PB7 = SDA (AF4)
// Used to communicate with the LSM303DLHC accelerometer (on-board)
// ---------------------------------------------------------------------------
#define I2C1_BASE           (APB1PERIPH_BASE + 0x00005400UL)

typedef struct {
    reg32_t CR1;        // Control register 1    - PE (enable), filters, interrupts
    reg32_t CR2;        // Control register 2    - SADD (slave addr), NBYTES, START, STOP, RD_WRN
    reg32_t OAR1;       // Own address register 1
    reg32_t OAR2;       // Own address register 2
    reg32_t TIMINGR;    // Timing register       - PRESC, SCLDEL, SDADEL, SCLH, SCLL
    reg32_t TIMEOUTR;   // Timeout register
    reg32_t ISR;        // Interrupt and status  - TXE, TXIS, RXNE, TC, BUSY, NACKF...
    reg32_t ICR;        // Interrupt clear register
    reg32_t PECR;       // PEC register
    reg32_t RXDR;       // Receive data register
    reg32_t TXDR;       // Transmit data register
} I2C_TypeDef;

#define I2C1                ((I2C_TypeDef *) I2C1_BASE)

// CR1 bits
#define I2C_CR1_PE          (1UL << 0)    // Peripheral enable

// CR2 bits
#define I2C_CR2_RD_WRN      (1UL << 10)   // Transfer direction: 0=write, 1=read
#define I2C_CR2_START       (1UL << 13)   // Start generation
#define I2C_CR2_STOP        (1UL << 14)   // Stop generation
#define I2C_CR2_AUTOEND     (1UL << 25)   // Automatic end: STOP after NBYTES transferred
#define I2C_CR2_NBYTES_Pos  16            // Bit position of NBYTES field in CR2

// ISR bits
#define I2C_ISR_TXE         (1UL << 0)    // Transmit data register empty
#define I2C_ISR_TXIS        (1UL << 1)    // Transmit interrupt status - ready to write TXDR
#define I2C_ISR_RXNE        (1UL << 2)    // Receive data register not empty - data ready in RXDR
#define I2C_ISR_TC          (1UL << 6)    // Transfer complete (when AUTOEND=0)
#define I2C_ISR_BUSY        (1UL << 15)   // Bus busy

// ---------------------------------------------------------------------------
// LSM303DLHC - Accelerometer registers and constants
// On-board sensor, connected to I2C1
// ---------------------------------------------------------------------------

// 7-bit I2C address of the accelerometer part
// Full write address = 0x32 (0x19 << 1 | 0), read = 0x33 (0x19 << 1 | 1)
#define LSM303_ACCEL_ADDR       0x19U

// CTRL_REG1_A (0x20): power mode, output data rate, axis enable
//   0x57 = 0b01010111
//   bits [7:4] = ODR  = 0101 -> 100 Hz output data rate
//   bit  3     = LPen = 0    -> normal mode (not low power)
//   bit  2     = Zen  = 1    -> Z axis enabled
//   bit  1     = Yen  = 1    -> Y axis enabled
//   bit  0     = Xen  = 1    -> X axis enabled
#define LSM303_CTRL_REG1_A      0x20U
#define LSM303_CTRL_REG1_A_VAL  0x57U

// CTRL_REG4_A (0x23): full scale, high resolution
//   0x08 = 0b00001000
//   bits [5:4] = FS  = 00 -> +/-2g full scale
//   bit  3     = HR  = 1  -> high resolution mode (12-bit output)
//   bit  7     = BDU = 0  -> continuous update
#define LSM303_CTRL_REG4_A      0x23U
#define LSM303_CTRL_REG4_A_VAL  0x08U

// Output data registers (little-endian, 2 bytes per axis)
// OUT_X_L_A is the starting address for a 6-byte burst read
#define LSM303_OUT_X_L_A        0x28U

// ---------------------------------------------------------------------------
// Cortex-M4 intrinsics
// ---------------------------------------------------------------------------
static inline void __NOP(void) { __asm volatile ("nop"); }

#endif // STM32F303XC_H