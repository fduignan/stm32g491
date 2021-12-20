/****************************************************************************************************//**
 *
 *  CMSIS-SVD SVD Consistency Checker / Header File Generator V3.2.17
 *  Copyright (C) 2010 - 2016 ARM Ltd and ARM Germany GmbH. All rights reserved.
 *
 * @brief    CMSIS HeaderFile
 *
 * @date     23. November 2021
 *
 * @note     Generated with SVDConv V3.2.17 on Tuesday, 23.11.2021 21:14:10
 *
 *           from CMSIS SVD File 'STM32G491xx.svd',
 *           created on Tuesday, 23.11.2021 20:13:35, last modified on Tuesday, 23.11.2021 20:13:35
 *
 *******************************************************************************************************/

/*
  
 */



/** @addtogroup 
  * @{
  */


/** @addtogroup STM32G491xx
  * @{
  */


#ifndef STM32G491XX_H
#define STM32G491XX_H

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup Configuration_of_CMSIS
  * @{
  */



/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum {
/* =======================================  ARM Cortex-M7 Specific Interrupt Numbers  ======================================== */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  MemoryManagement_IRQn     = -12,              /*!< -12  Memory Management, MPU mismatch, including Access Violation
                                                      and No Match                                                             */
  BusFault_IRQn             = -11,              /*!< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                      related Fault                                                            */
  UsageFault_IRQn           = -10,              /*!< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  DebugMonitor_IRQn         =  -4,              /*!< -4 Debug Monitor                                                          */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */
/* ========================================  STM32G491xx Specific Interrupt Numbers  ========================================= */
  WWDG_IRQn                 =   0,              /*!< 0  Window Watchdog interrupt                                              */
  PVD_PVM_IRQn              =   1,              /*!< 1  PVD through EXTI line detection                                        */
  RTC_TAMP_CSS_LSE_IRQn     =   2,              /*!< 2  RTC_TAMP_CSS_LSE                                                       */
  RTC_WKUP_IRQn             =   3,              /*!< 3  RTC Wakeup timer                                                       */
  FLASH_IRQn                =   4,              /*!< 4  FLASH                                                                  */
  RCC_IRQn                  =   5,              /*!< 5  RCC                                                                    */
  EXTI0_IRQn                =   6,              /*!< 6  EXTI Line0 interrupt                                                   */
  EXTI1_IRQn                =   7,              /*!< 7  EXTI Line1 interrupt                                                   */
  EXTI2_IRQn                =   8,              /*!< 8  EXTI Line2 interrupt                                                   */
  EXTI3_IRQn                =   9,              /*!< 9  EXTI Line3 interrupt                                                   */
  EXTI4_IRQn                =  10,              /*!< 10 EXTI Line4 interrupt                                                   */
  DMA1_CH1_IRQn             =  11,              /*!< 11 DMA1 channel 1 interrupt                                               */
  DMA1_CH2_IRQn             =  12,              /*!< 12 DMA1 channel 2 interrupt                                               */
  DMA1_CH3_IRQn             =  13,              /*!< 13 DMA1 channel 3 interrupt                                               */
  DMA1_CH4_IRQn             =  14,              /*!< 14 DMA1 channel 4 interrupt                                               */
  DMA1_CH5_IRQn             =  15,              /*!< 15 DMA1 channel 5 interrupt                                               */
  DMA1_CH6_IRQn             =  16,              /*!< 16 DMA1 channel 6 interrupt                                               */
  ADC1_2_IRQn               =  18,              /*!< 18 ADC1 and ADC2 global interrupt                                         */
  USB_HP_IRQn               =  19,              /*!< 19 USB_HP                                                                 */
  USB_LP_IRQn               =  20,              /*!< 20 USB_LP                                                                 */
  fdcan1_intr1_it_IRQn      =  21,              /*!< 21 fdcan1_intr1_it                                                        */
  fdcan1_intr0_it_IRQn      =  22,              /*!< 22 fdcan1_intr0_it                                                        */
  EXTI9_5_IRQn              =  23,              /*!< 23 EXTI9_5                                                                */
  TIM1_BRK_TIM15_IRQn       =  24,              /*!< 24 TIM1_BRK_TIM15                                                         */
  TIM1_UP_TIM16_IRQn        =  25,              /*!< 25 TIM1_UP_TIM16                                                          */
  TIM1_TRG_COM_IRQn         =  26,              /*!< 26 TIM1_TRG_COM/                                                          */
  TIM1_CC_IRQn              =  27,              /*!< 27 TIM1 capture compare interrupt                                         */
  TIM2_IRQn                 =  28,              /*!< 28 TIM2                                                                   */
  TIM3_IRQn                 =  29,              /*!< 29 TIM3                                                                   */
  TIM4_IRQn                 =  30,              /*!< 30 TIM4                                                                   */
  I2C1_EV_IRQn              =  31,              /*!< 31 I2C1_EV                                                                */
  I2C1_ER_IRQn              =  32,              /*!< 32 I2C1_ER                                                                */
  I2C2_EV_IRQn              =  33,              /*!< 33 I2C2_EV                                                                */
  I2C2_ER_IRQn              =  34,              /*!< 34 I2C2_ER                                                                */
  SPI1_IRQn                 =  35,              /*!< 35 SPI1                                                                   */
  SPI2_IRQn                 =  36,              /*!< 36 SPI2                                                                   */
  USART1_IRQn               =  37,              /*!< 37 USART1                                                                 */
  USART2_IRQn               =  38,              /*!< 38 USART2                                                                 */
  USART3_IRQn               =  39,              /*!< 39 USART3                                                                 */
  EXTI15_10_IRQn            =  40,              /*!< 40 EXTI15_10                                                              */
  RTC_ALARM_IRQn            =  41,              /*!< 41 RTC_ALARM                                                              */
  USBWakeUP_IRQn            =  42,              /*!< 42 USBWakeUP                                                              */
  TIM8_BRK_IRQn             =  43,              /*!< 43 TIM8_BRK                                                               */
  TIM8_UP_IRQn              =  44,              /*!< 44 TIM8_UP                                                                */
  TIM8_TRG_COM_IRQn         =  45,              /*!< 45 TIM8_TRG_COM                                                           */
  TIM8_CC_IRQn              =  46,              /*!< 46 TIM8_CC                                                                */
  ADC3_IRQn                 =  47,              /*!< 47 ADC3                                                                   */
  LPTIM1_IRQn               =  49,              /*!< 49 LPTIM1                                                                 */
  SPI3_IRQn                 =  51,              /*!< 51 SPI3                                                                   */
  UART4_IRQn                =  52,              /*!< 52 UART4                                                                  */
  UART5_IRQn                =  53,              /*!< 53 UART5                                                                  */
  TIM6_DACUNDER_IRQn        =  54,              /*!< 54 TIM6_DACUNDER                                                          */
  TIM7_IRQn                 =  55,              /*!< 55 TIM7                                                                   */
  DMA2_CH1_IRQn             =  56,              /*!< 56 DMA2_CH1                                                               */
  DMA2_CH2_IRQn             =  57,              /*!< 57 DMA2_CH2                                                               */
  DMA2_CH3_IRQn             =  58,              /*!< 58 DMA2_CH3                                                               */
  DMA2_CH4_IRQn             =  59,              /*!< 59 DMA2_CH4                                                               */
  DMA2_CH5_IRQn             =  60,              /*!< 60 DMA2_CH5                                                               */
  UCPD1_IRQn                =  63,              /*!< 63 UCPD1                                                                  */
  COMP1_2_3_IRQn            =  64,              /*!< 64 COMP1_2_3                                                              */
  COMP4_5_6_IRQn            =  65,              /*!< 65 COMP4_5_6                                                              */
  COMP7_IRQn                =  66,              /*!< 66 COMP7                                                                  */
  CRS_IRQn                  =  75,              /*!< 75 CRS                                                                    */
  SAI_IRQn                  =  76,              /*!< 76 SAI                                                                    */
  FPU_IRQn                  =  81,              /*!< 81 Floating point unit interrupt                                          */
  RNG_IRQn                  =  90,              /*!< 90 RNG                                                                    */
  LPUART_IRQn               =  91,              /*!< 91 LPUART                                                                 */
  I2C3_EV_IRQn              =  92,              /*!< 92 I2C3_EV                                                                */
  I2C3_ER_IRQn              =  93,              /*!< 93 I2C3_ER                                                                */
  DMAMUX_OVR_IRQn           =  94,              /*!< 94 DMAMUX_OVR                                                             */
  DMA2_CH6_IRQn             =  97,              /*!< 97 DMA2_CH6                                                               */
  Cordic_IRQn               = 100,              /*!< 100  Cordic                                                               */
  FMAC_IRQn                 = 101               /*!< 101  FMAC                                                                 */
} IRQn_Type;



/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* ===========================  Configuration of the ARM Cortex-M7 Processor and Core Peripherals  =========================== */
#define __CM4_REV                 0x0001        /*!< CM4 Core Revision                                                         */
#define __MPU_PRESENT                  1        /*!< MPU present or not                                                        */
#define __NVIC_PRIO_BITS               4        /*!< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0        /*!< Set to 1 if different SysTick Config is used                              */
#define __FPU_PRESENT                  1        /*!< FPU present or not                                                        */


/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_CM4.h"                           /*!< ARM Cortex-M7 processor and core peripherals                              */
#include "system_STM32G491xx.h"                 /*!< STM32G491xx System                                                        */


/* ========================================  Start of section using anonymous unions  ======================================== */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                            Device Specific Peripheral Section                             ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripherals
  * @{
  */



/* =========================================================================================================================== */
/* ================                                            CRC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Cyclic redundancy check calculation unit (CRC)
  */

typedef struct {                                /*!< (@ 0x40023000) CRC Structure                                              */
  __IO uint32_t   DR;                           /*!< (@ 0x00000000) Data register                                              */
  __IO uint32_t   IDR;                          /*!< (@ 0x00000004) Independent data register                                  */
  __IO uint32_t   CR;                           /*!< (@ 0x00000008) Control register                                           */
  __I  uint32_t   RESERVED;
  __IO uint32_t   INIT;                         /*!< (@ 0x00000010) Initial CRC value                                          */
  __IO uint32_t   POL;                          /*!< (@ 0x00000014) polynomial                                                 */
} CRC_Type;                                     /*!< Size = 24 (0x18)                                                          */



/* =========================================================================================================================== */
/* ================                                           IWDG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief WinWATCHDOG (IWDG)
  */

typedef struct {                                /*!< (@ 0x40003000) IWDG Structure                                             */
  __O  uint32_t   KR;                           /*!< (@ 0x00000000) Key register                                               */
  __IO uint32_t   PR;                           /*!< (@ 0x00000004) Prescaler register                                         */
  __IO uint32_t   RLR;                          /*!< (@ 0x00000008) Reload register                                            */
  __I  uint32_t   SR;                           /*!< (@ 0x0000000C) Status register                                            */
  __IO uint32_t   WINR;                         /*!< (@ 0x00000010) Window register                                            */
} IWDG_Type;                                    /*!< Size = 20 (0x14)                                                          */



/* =========================================================================================================================== */
/* ================                                           WWDG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief System window watchdog (WWDG)
  */

typedef struct {                                /*!< (@ 0x40002C00) WWDG Structure                                             */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) Control register                                           */
  __IO uint32_t   CFR;                          /*!< (@ 0x00000004) Configuration register                                     */
  __IO uint32_t   SR;                           /*!< (@ 0x00000008) Status register                                            */
} WWDG_Type;                                    /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                           I2C1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Inter-integrated circuit (I2C1)
  */

typedef struct {                                /*!< (@ 0x40005400) I2C1 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Control register 2                                         */
  __IO uint32_t   OAR1;                         /*!< (@ 0x00000008) Own address register 1                                     */
  __IO uint32_t   OAR2;                         /*!< (@ 0x0000000C) Own address register 2                                     */
  __IO uint32_t   TIMINGR;                      /*!< (@ 0x00000010) Timing register                                            */
  __IO uint32_t   TIMEOUTR;                     /*!< (@ 0x00000014) Status register 1                                          */
  __IO uint32_t   ISR;                          /*!< (@ 0x00000018) Interrupt and Status register                              */
  __O  uint32_t   ICR;                          /*!< (@ 0x0000001C) Interrupt clear register                                   */
  __I  uint32_t   PECR;                         /*!< (@ 0x00000020) PEC register                                               */
  __I  uint32_t   RXDR;                         /*!< (@ 0x00000024) Receive data register                                      */
  __IO uint32_t   TXDR;                         /*!< (@ 0x00000028) Transmit data register                                     */
} I2C1_Type;                                    /*!< Size = 44 (0x2c)                                                          */



/* =========================================================================================================================== */
/* ================                                           FLASH                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Flash (FLASH)
  */

typedef struct {                                /*!< (@ 0x40022000) FLASH Structure                                            */
  __IO uint32_t   ACR;                          /*!< (@ 0x00000000) Access control register                                    */
  __O  uint32_t   PDKEYR;                       /*!< (@ 0x00000004) Power down key register                                    */
  __O  uint32_t   KEYR;                         /*!< (@ 0x00000008) Flash key register                                         */
  __O  uint32_t   OPTKEYR;                      /*!< (@ 0x0000000C) Option byte key register                                   */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) Status register                                            */
  __IO uint32_t   CR;                           /*!< (@ 0x00000014) Flash control register                                     */
  __IO uint32_t   ECCR;                         /*!< (@ 0x00000018) Flash ECC register                                         */
  __I  uint32_t   RESERVED;
  __IO uint32_t   OPTR;                         /*!< (@ 0x00000020) Flash option register                                      */
  __IO uint32_t   PCROP1SR;                     /*!< (@ 0x00000024) Flash Bank 1 PCROP Start address register                  */
  __IO uint32_t   PCROP1ER;                     /*!< (@ 0x00000028) Flash Bank 1 PCROP End address register                    */
  __IO uint32_t   WRP1AR;                       /*!< (@ 0x0000002C) Flash Bank 1 WRP area A address register                   */
  __IO uint32_t   WRP1BR;                       /*!< (@ 0x00000030) Flash Bank 1 WRP area B address register                   */
  __I  uint32_t   RESERVED1[15];
  __IO uint32_t   SEC1R;                        /*!< (@ 0x00000070) securable area bank1 register                              */
} FLASH_Type;                                   /*!< Size = 116 (0x74)                                                         */



/* =========================================================================================================================== */
/* ================                                          DBGMCU                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Debug support (DBGMCU)
  */

typedef struct {                                /*!< (@ 0xE0042000) DBGMCU Structure                                           */
  __I  uint32_t   IDCODE;                       /*!< (@ 0x00000000) MCU Device ID Code Register                                */
  __IO uint32_t   CR;                           /*!< (@ 0x00000004) Debug MCU Configuration Register                           */
  __IO uint32_t   APB1L_FZ;                     /*!< (@ 0x00000008) APB Low Freeze Register 1                                  */
  __IO uint32_t   APB1H_FZ;                     /*!< (@ 0x0000000C) APB Low Freeze Register 2                                  */
  __IO uint32_t   APB2_FZ;                      /*!< (@ 0x00000010) APB High Freeze Register                                   */
} DBGMCU_Type;                                  /*!< Size = 20 (0x14)                                                          */



/* =========================================================================================================================== */
/* ================                                            RCC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Reset and clock control (RCC)
  */

typedef struct {                                /*!< (@ 0x40021000) RCC Structure                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) Clock control register                                     */
  __IO uint32_t   ICSCR;                        /*!< (@ 0x00000004) Internal clock sources calibration register                */
  __IO uint32_t   CFGR;                         /*!< (@ 0x00000008) Clock configuration register                               */
  __IO uint32_t   PLLSYSCFGR;                   /*!< (@ 0x0000000C) PLL configuration register                                 */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   CIER;                         /*!< (@ 0x00000018) Clock interrupt enable register                            */
  __I  uint32_t   CIFR;                         /*!< (@ 0x0000001C) Clock interrupt flag register                              */
  __O  uint32_t   CICR;                         /*!< (@ 0x00000020) Clock interrupt clear register                             */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   AHB1RSTR;                     /*!< (@ 0x00000028) AHB1 peripheral reset register                             */
  __IO uint32_t   AHB2RSTR;                     /*!< (@ 0x0000002C) AHB2 peripheral reset register                             */
  __IO uint32_t   AHB3RSTR;                     /*!< (@ 0x00000030) AHB3 peripheral reset register                             */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   APB1RSTR1;                    /*!< (@ 0x00000038) APB1 peripheral reset register 1                           */
  __IO uint32_t   APB1RSTR2;                    /*!< (@ 0x0000003C) APB1 peripheral reset register 2                           */
  __IO uint32_t   APB2RSTR;                     /*!< (@ 0x00000040) APB2 peripheral reset register                             */
  __I  uint32_t   RESERVED3;
  __IO uint32_t   AHB1ENR;                      /*!< (@ 0x00000048) AHB1 peripheral clock enable register                      */
  __IO uint32_t   AHB2ENR;                      /*!< (@ 0x0000004C) AHB2 peripheral clock enable register                      */
  __IO uint32_t   AHB3ENR;                      /*!< (@ 0x00000050) AHB3 peripheral clock enable register                      */
  __I  uint32_t   RESERVED4;
  __IO uint32_t   APB1ENR1;                     /*!< (@ 0x00000058) APB1ENR1                                                   */
  __IO uint32_t   APB1ENR2;                     /*!< (@ 0x0000005C) APB1 peripheral clock enable register 2                    */
  __IO uint32_t   APB2ENR;                      /*!< (@ 0x00000060) APB2ENR                                                    */
  __I  uint32_t   RESERVED5;
  __IO uint32_t   AHB1SMENR;                    /*!< (@ 0x00000068) AHB1 peripheral clocks enable in Sleep and Stop
                                                                    modes register                                             */
  __IO uint32_t   AHB2SMENR;                    /*!< (@ 0x0000006C) AHB2 peripheral clocks enable in Sleep and Stop
                                                                    modes register                                             */
  __IO uint32_t   AHB3SMENR;                    /*!< (@ 0x00000070) AHB3 peripheral clocks enable in Sleep and Stop
                                                                    modes register                                             */
  __I  uint32_t   RESERVED6;
  __IO uint32_t   APB1SMENR1;                   /*!< (@ 0x00000078) APB1SMENR1                                                 */
  __IO uint32_t   APB1SMENR2;                   /*!< (@ 0x0000007C) APB1 peripheral clocks enable in Sleep and Stop
                                                                    modes register 2                                           */
  __IO uint32_t   APB2SMENR;                    /*!< (@ 0x00000080) APB2SMENR                                                  */
  __I  uint32_t   RESERVED7;
  __IO uint32_t   CCIPR1;                       /*!< (@ 0x00000088) CCIPR                                                      */
  __I  uint32_t   RESERVED8;
  __IO uint32_t   BDCR;                         /*!< (@ 0x00000090) BDCR                                                       */
  __IO uint32_t   CSR;                          /*!< (@ 0x00000094) CSR                                                        */
  __IO uint32_t   CRRCR;                        /*!< (@ 0x00000098) Clock recovery RC register                                 */
  __IO uint32_t   CCIPR2;                       /*!< (@ 0x0000009C) Peripherals independent clock configuration register       */
} RCC_Type;                                     /*!< Size = 160 (0xa0)                                                         */



/* =========================================================================================================================== */
/* ================                                            PWR                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Power control (PWR)
  */

typedef struct {                                /*!< (@ 0x40007000) PWR Structure                                              */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Power control register 1                                   */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Power control register 2                                   */
  __IO uint32_t   CR3;                          /*!< (@ 0x00000008) Power control register 3                                   */
  __IO uint32_t   CR4;                          /*!< (@ 0x0000000C) Power control register 4                                   */
  __I  uint32_t   SR1;                          /*!< (@ 0x00000010) Power status register 1                                    */
  __I  uint32_t   SR2;                          /*!< (@ 0x00000014) Power status register 2                                    */
  __O  uint32_t   SCR;                          /*!< (@ 0x00000018) Power status clear register                                */
  __I  uint32_t   RESERVED;
  __IO uint32_t   PUCRA;                        /*!< (@ 0x00000020) Power Port A pull-up control register                      */
  __IO uint32_t   PDCRA;                        /*!< (@ 0x00000024) Power Port A pull-down control register                    */
  __IO uint32_t   PUCRB;                        /*!< (@ 0x00000028) Power Port B pull-up control register                      */
  __IO uint32_t   PDCRB;                        /*!< (@ 0x0000002C) Power Port B pull-down control register                    */
  __IO uint32_t   PUCRC;                        /*!< (@ 0x00000030) Power Port C pull-up control register                      */
  __IO uint32_t   PDCRC;                        /*!< (@ 0x00000034) Power Port C pull-down control register                    */
  __IO uint32_t   PUCRD;                        /*!< (@ 0x00000038) Power Port D pull-up control register                      */
  __IO uint32_t   PDCRD;                        /*!< (@ 0x0000003C) Power Port D pull-down control register                    */
  __IO uint32_t   PUCRE;                        /*!< (@ 0x00000040) Power Port E pull-up control register                      */
  __IO uint32_t   PDCRE;                        /*!< (@ 0x00000044) Power Port E pull-down control register                    */
  __IO uint32_t   PUCRF;                        /*!< (@ 0x00000048) Power Port F pull-up control register                      */
  __IO uint32_t   PDCRF;                        /*!< (@ 0x0000004C) Power Port F pull-down control register                    */
  __IO uint32_t   PUCRG;                        /*!< (@ 0x00000050) Power Port G pull-up control register                      */
  __IO uint32_t   PDCRG;                        /*!< (@ 0x00000054) Power Port G pull-down control register                    */
  __I  uint32_t   RESERVED1[10];
  __IO uint32_t   CR5;                          /*!< (@ 0x00000080) Power control register 5                                   */
} PWR_Type;                                     /*!< Size = 132 (0x84)                                                         */



/* =========================================================================================================================== */
/* ================                                            RNG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Random number generator (RNG)
  */

typedef struct {                                /*!< (@ 0x50060800) RNG Structure                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) control register                                           */
  __IO uint32_t   SR;                           /*!< (@ 0x00000004) status register                                            */
  __I  uint32_t   DR;                           /*!< (@ 0x00000008) data register                                              */
} RNG_Type;                                     /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                           GPIOA                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOA)
  */

typedef struct {                                /*!< (@ 0x48000000) GPIOA Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
  __O  uint32_t   BRR;                          /*!< (@ 0x00000028) GPIO port bit reset register                               */
} GPIOA_Type;                                   /*!< Size = 44 (0x2c)                                                          */



/* =========================================================================================================================== */
/* ================                                           GPIOB                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOB)
  */

typedef struct {                                /*!< (@ 0x48000400) GPIOB Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
  __O  uint32_t   BRR;                          /*!< (@ 0x00000028) GPIO port bit reset register                               */
} GPIOB_Type;                                   /*!< Size = 44 (0x2c)                                                          */



/* =========================================================================================================================== */
/* ================                                           GPIOC                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOC)
  */

typedef struct {                                /*!< (@ 0x48000800) GPIOC Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
  __O  uint32_t   BRR;                          /*!< (@ 0x00000028) GPIO port bit reset register                               */
} GPIOC_Type;                                   /*!< Size = 44 (0x2c)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM15                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timers (TIM15)
  */

typedef struct {                                /*!< (@ 0x40014000) TIM15 Structure                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register (output mode)                */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __IO uint32_t   RCR;                          /*!< (@ 0x00000030) repetition counter register                                */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __I  uint32_t   RESERVED1[2];
  __IO uint32_t   BDTR;                         /*!< (@ 0x00000044) break and dead-time register                               */
  __I  uint32_t   RESERVED2[3];
  __IO uint32_t   DTR2;                         /*!< (@ 0x00000054) timer Deadtime Register 2                                  */
  __I  uint32_t   RESERVED3;
  __IO uint32_t   TISEL;                        /*!< (@ 0x0000005C) TIM timer input selection register                         */
  __IO uint32_t   AF1;                          /*!< (@ 0x00000060) TIM alternate function option register 1                   */
  __IO uint32_t   AF2;                          /*!< (@ 0x00000064) TIM alternate function option register 2                   */
  __I  uint32_t   RESERVED4[221];
  __IO uint32_t   DCR;                          /*!< (@ 0x000003DC) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x000003E0) DMA address for full transfer                              */
} TIM15_Type;                                   /*!< Size = 996 (0x3e4)                                                        */



/* =========================================================================================================================== */
/* ================                                           TIM16                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timers (TIM16)
  */

typedef struct {                                /*!< (@ 0x40014400) TIM16 Structure                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __I  uint32_t   RESERVED;
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register (output mode)                */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  __I  uint32_t   RESERVED1;
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __IO uint32_t   RCR;                          /*!< (@ 0x00000030) repetition counter register                                */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __I  uint32_t   RESERVED2[3];
  __IO uint32_t   BDTR;                         /*!< (@ 0x00000044) break and dead-time register                               */
  __I  uint32_t   RESERVED3[3];
  __IO uint32_t   DTR2;                         /*!< (@ 0x00000054) timer Deadtime Register 2                                  */
  __I  uint32_t   RESERVED4;
  __IO uint32_t   TISEL;                        /*!< (@ 0x0000005C) TIM timer input selection register                         */
  __IO uint32_t   AF1;                          /*!< (@ 0x00000060) TIM alternate function option register 1                   */
  __IO uint32_t   AF2;                          /*!< (@ 0x00000064) TIM alternate function option register 2                   */
  __IO uint32_t   OR1;                          /*!< (@ 0x00000068) TIM option register 1                                      */
  __I  uint32_t   RESERVED5[220];
  __IO uint32_t   DCR;                          /*!< (@ 0x000003DC) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x000003E0) DMA address for full transfer                              */
} TIM16_Type;                                   /*!< Size = 996 (0x3e4)                                                        */



/* =========================================================================================================================== */
/* ================                                           TIM1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Advanced-timers (TIM1)
  */

typedef struct {                                /*!< (@ 0x40012C00) TIM1 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __IO uint32_t   RCR;                          /*!< (@ 0x00000030) repetition counter register                                */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __IO uint32_t   BDTR;                         /*!< (@ 0x00000044) break and dead-time register                               */
  __IO uint32_t   CCR5;                         /*!< (@ 0x00000048) capture/compare register 4                                 */
  __IO uint32_t   CCR6;                         /*!< (@ 0x0000004C) capture/compare register 4                                 */
  __IO uint32_t   CCMR3_Output;                 /*!< (@ 0x00000050) capture/compare mode register 2 (output mode)              */
  __IO uint32_t   DTR2;                         /*!< (@ 0x00000054) timer Deadtime Register 2                                  */
  __IO uint32_t   ECR;                          /*!< (@ 0x00000058) DMA control register                                       */
  __IO uint32_t   TISEL;                        /*!< (@ 0x0000005C) TIM timer input selection register                         */
  __IO uint32_t   AF1;                          /*!< (@ 0x00000060) TIM alternate function option register 1                   */
  __IO uint32_t   AF2;                          /*!< (@ 0x00000064) TIM alternate function option register 2                   */
  __I  uint32_t   RESERVED[221];
  __IO uint32_t   DCR;                          /*!< (@ 0x000003DC) control register                                           */
  __IO uint32_t   DMAR;                         /*!< (@ 0x000003E0) DMA address for full transfer                              */
} TIM1_Type;                                    /*!< Size = 996 (0x3e4)                                                        */



/* =========================================================================================================================== */
/* ================                                           TIM2                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Advanced-timers (TIM2)
  */

typedef struct {                                /*!< (@ 0x40000000) TIM2 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __IO uint32_t   RCR;                          /*!< (@ 0x00000030) repetition counter register                                */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __IO uint32_t   BDTR;                         /*!< (@ 0x00000044) break and dead-time register                               */
  __IO uint32_t   CCR5;                         /*!< (@ 0x00000048) capture/compare register 4                                 */
  __IO uint32_t   CCR6;                         /*!< (@ 0x0000004C) capture/compare register 4                                 */
  __IO uint32_t   CCMR3_Output;                 /*!< (@ 0x00000050) capture/compare mode register 2 (output mode)              */
  __IO uint32_t   DTR2;                         /*!< (@ 0x00000054) timer Deadtime Register 2                                  */
  __IO uint32_t   ECR;                          /*!< (@ 0x00000058) DMA control register                                       */
  __IO uint32_t   TISEL;                        /*!< (@ 0x0000005C) TIM timer input selection register                         */
  __IO uint32_t   AF1;                          /*!< (@ 0x00000060) TIM alternate function option register 1                   */
  __IO uint32_t   AF2;                          /*!< (@ 0x00000064) TIM alternate function option register 2                   */
  __I  uint32_t   RESERVED[221];
  __IO uint32_t   DCR;                          /*!< (@ 0x000003DC) control register                                           */
  __IO uint32_t   DMAR;                         /*!< (@ 0x000003E0) DMA address for full transfer                              */
} TIM2_Type;                                    /*!< Size = 996 (0x3e4)                                                        */



/* =========================================================================================================================== */
/* ================                                           TIM6                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Basic-timers (TIM6)
  */

typedef struct {                                /*!< (@ 0x40001000) TIM6 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __I  uint32_t   RESERVED;
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  __I  uint32_t   RESERVED1[3];
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
} TIM6_Type;                                    /*!< Size = 48 (0x30)                                                          */



/* =========================================================================================================================== */
/* ================                                         LPTIMER1                                          ================ */
/* =========================================================================================================================== */


/**
  * @brief Low power timer (LPTIMER1)
  */

typedef struct {                                /*!< (@ 0x40007C00) LPTIMER1 Structure                                         */
  __I  uint32_t   ISR;                          /*!< (@ 0x00000000) Interrupt and Status Register                              */
  __O  uint32_t   ICR;                          /*!< (@ 0x00000004) Interrupt Clear Register                                   */
  __IO uint32_t   IER;                          /*!< (@ 0x00000008) Interrupt Enable Register                                  */
  __IO uint32_t   CFGR;                         /*!< (@ 0x0000000C) Configuration Register                                     */
  __IO uint32_t   CR;                           /*!< (@ 0x00000010) Control Register                                           */
  __IO uint32_t   CMP;                          /*!< (@ 0x00000014) Compare Register                                           */
  __IO uint32_t   ARR;                          /*!< (@ 0x00000018) Autoreload Register                                        */
  __I  uint32_t   CNT;                          /*!< (@ 0x0000001C) Counter Register                                           */
  __IO uint32_t   OR;                           /*!< (@ 0x00000020) option register                                            */
} LPTIMER1_Type;                                /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                          USART1                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (USART1)
  */

typedef struct {                                /*!< (@ 0x40013800) USART1 Structure                                           */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Control register 2                                         */
  __IO uint32_t   CR3;                          /*!< (@ 0x00000008) Control register 3                                         */
  __IO uint32_t   BRR;                          /*!< (@ 0x0000000C) Baud rate register                                         */
  __IO uint32_t   GTPR;                         /*!< (@ 0x00000010) Guard time and prescaler register                          */
  __IO uint32_t   RTOR;                         /*!< (@ 0x00000014) Receiver timeout register                                  */
  __O  uint32_t   RQR;                          /*!< (@ 0x00000018) Request register                                           */
  __I  uint32_t   ISR;                          /*!< (@ 0x0000001C) Interrupt & status register                                */
  __O  uint32_t   ICR;                          /*!< (@ 0x00000020) Interrupt flag clear register                              */
  __I  uint32_t   RDR;                          /*!< (@ 0x00000024) Receive data register                                      */
  __IO uint32_t   TDR;                          /*!< (@ 0x00000028) Transmit data register                                     */
  __IO uint32_t   PRESC;                        /*!< (@ 0x0000002C) USART prescaler register                                   */
} USART1_Type;                                  /*!< Size = 48 (0x30)                                                          */



/* =========================================================================================================================== */
/* ================                                           UART4                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (UART4)
  */

typedef struct {                                /*!< (@ 0x40004C00) UART4 Structure                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Control register 2                                         */
  __IO uint32_t   CR3;                          /*!< (@ 0x00000008) Control register 3                                         */
  __IO uint32_t   BRR;                          /*!< (@ 0x0000000C) Baud rate register                                         */
  __IO uint32_t   GTPR;                         /*!< (@ 0x00000010) Guard time and prescaler register                          */
  __IO uint32_t   RTOR;                         /*!< (@ 0x00000014) Receiver timeout register                                  */
  __O  uint32_t   RQR;                          /*!< (@ 0x00000018) Request register                                           */
  __I  uint32_t   ISR;                          /*!< (@ 0x0000001C) Interrupt & status register                                */
  __O  uint32_t   ICR;                          /*!< (@ 0x00000020) Interrupt flag clear register                              */
  __I  uint32_t   RDR;                          /*!< (@ 0x00000024) Receive data register                                      */
  __IO uint32_t   TDR;                          /*!< (@ 0x00000028) Transmit data register                                     */
  __IO uint32_t   PRESC;                        /*!< (@ 0x0000002C) USART prescaler register                                   */
} UART4_Type;                                   /*!< Size = 48 (0x30)                                                          */



/* =========================================================================================================================== */
/* ================                                          LPUART1                                          ================ */
/* =========================================================================================================================== */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (LPUART1)
  */

typedef struct {                                /*!< (@ 0x40008000) LPUART1 Structure                                          */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Control register 2                                         */
  __IO uint32_t   CR3;                          /*!< (@ 0x00000008) Control register 3                                         */
  __IO uint32_t   BRR;                          /*!< (@ 0x0000000C) Baud rate register                                         */
  __I  uint32_t   RESERVED[2];
  __O  uint32_t   RQR;                          /*!< (@ 0x00000018) Request register                                           */
  __I  uint32_t   ISR;                          /*!< (@ 0x0000001C) Interrupt & status register                                */
  __O  uint32_t   ICR;                          /*!< (@ 0x00000020) Interrupt flag clear register                              */
  __I  uint32_t   RDR;                          /*!< (@ 0x00000024) Receive data register                                      */
  __IO uint32_t   TDR;                          /*!< (@ 0x00000028) Transmit data register                                     */
  __IO uint32_t   PRESC;                        /*!< (@ 0x0000002C) Prescaler register                                         */
} LPUART1_Type;                                 /*!< Size = 48 (0x30)                                                          */



/* =========================================================================================================================== */
/* ================                                           SPI1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Serial peripheral interface/Inter-IC sound (SPI1)
  */

typedef struct {                                /*!< (@ 0x40013000) SPI1 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SR;                           /*!< (@ 0x00000008) status register                                            */
  __IO uint32_t   DR;                           /*!< (@ 0x0000000C) data register                                              */
  __IO uint32_t   CRCPR;                        /*!< (@ 0x00000010) CRC polynomial register                                    */
  __I  uint32_t   RXCRCR;                       /*!< (@ 0x00000014) RX CRC register                                            */
  __I  uint32_t   TXCRCR;                       /*!< (@ 0x00000018) TX CRC register                                            */
  __IO uint32_t   I2SCFGR;                      /*!< (@ 0x0000001C) configuration register                                     */
  __IO uint32_t   I2SPR;                        /*!< (@ 0x00000020) prescaler register                                         */
} SPI1_Type;                                    /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                           EXTI                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief External interrupt/event controller (EXTI)
  */

typedef struct {                                /*!< (@ 0x40010400) EXTI Structure                                             */
  __IO uint32_t   IMR1;                         /*!< (@ 0x00000000) Interrupt mask register                                    */
  __IO uint32_t   EMR1;                         /*!< (@ 0x00000004) Event mask register                                        */
  __IO uint32_t   RTSR1;                        /*!< (@ 0x00000008) Rising Trigger selection register                          */
  __IO uint32_t   FTSR1;                        /*!< (@ 0x0000000C) Falling Trigger selection register                         */
  __IO uint32_t   SWIER1;                       /*!< (@ 0x00000010) Software interrupt event register                          */
  __IO uint32_t   PR1;                          /*!< (@ 0x00000014) Pending register                                           */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   IMR2;                         /*!< (@ 0x00000020) Interrupt mask register                                    */
  __IO uint32_t   EMR2;                         /*!< (@ 0x00000024) Event mask register                                        */
  __IO uint32_t   RTSR2;                        /*!< (@ 0x00000028) Rising Trigger selection register                          */
  __IO uint32_t   FTSR2;                        /*!< (@ 0x0000002C) Falling Trigger selection register                         */
  __IO uint32_t   SWIER2;                       /*!< (@ 0x00000030) Software interrupt event register                          */
  __IO uint32_t   PR2;                          /*!< (@ 0x00000034) Pending register                                           */
} EXTI_Type;                                    /*!< Size = 56 (0x38)                                                          */



/* =========================================================================================================================== */
/* ================                                            RTC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Real-time clock (RTC)
  */

typedef struct {                                /*!< (@ 0x40002800) RTC Structure                                              */
  __IO uint32_t   TR;                           /*!< (@ 0x00000000) time register                                              */
  __IO uint32_t   DR;                           /*!< (@ 0x00000004) date register                                              */
  __I  uint32_t   SSR;                          /*!< (@ 0x00000008) sub second register                                        */
  __IO uint32_t   ICSR;                         /*!< (@ 0x0000000C) initialization and status register                         */
  __IO uint32_t   PRER;                         /*!< (@ 0x00000010) prescaler register                                         */
  __IO uint32_t   WUTR;                         /*!< (@ 0x00000014) wakeup timer register                                      */
  __IO uint32_t   CR;                           /*!< (@ 0x00000018) control register                                           */
  __I  uint32_t   RESERVED[2];
  __O  uint32_t   WPR;                          /*!< (@ 0x00000024) write protection register                                  */
  __IO uint32_t   CALR;                         /*!< (@ 0x00000028) calibration register                                       */
  __O  uint32_t   SHIFTR;                       /*!< (@ 0x0000002C) shift control register                                     */
  __I  uint32_t   TSTR;                         /*!< (@ 0x00000030) time stamp time register                                   */
  __I  uint32_t   TSDR;                         /*!< (@ 0x00000034) time stamp date register                                   */
  __I  uint32_t   TSSSR;                        /*!< (@ 0x00000038) timestamp sub second register                              */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   ALRMAR;                       /*!< (@ 0x00000040) alarm A register                                           */
  __IO uint32_t   ALRMASSR;                     /*!< (@ 0x00000044) alarm A sub second register                                */
  __IO uint32_t   ALRMBR;                       /*!< (@ 0x00000048) alarm B register                                           */
  __IO uint32_t   ALRMBSSR;                     /*!< (@ 0x0000004C) alarm B sub second register                                */
  __I  uint32_t   SR;                           /*!< (@ 0x00000050) status register                                            */
  __I  uint32_t   MISR;                         /*!< (@ 0x00000054) status register                                            */
  __I  uint32_t   RESERVED2;
  __O  uint32_t   SCR;                          /*!< (@ 0x0000005C) status register                                            */
} RTC_Type;                                     /*!< Size = 96 (0x60)                                                          */



/* =========================================================================================================================== */
/* ================                                           DMA1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief DMA controller (DMA1)
  */

typedef struct {                                /*!< (@ 0x40020000) DMA1 Structure                                             */
  __I  uint32_t   ISR;                          /*!< (@ 0x00000000) interrupt status register                                  */
  __O  uint32_t   IFCR;                         /*!< (@ 0x00000004) DMA interrupt flag clear register                          */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000008) DMA channel 1 configuration register                       */
  __IO uint32_t   CNDTR1;                       /*!< (@ 0x0000000C) channel x number of data to transfer register              */
  __IO uint32_t   CPAR1;                        /*!< (@ 0x00000010) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR1;                        /*!< (@ 0x00000014) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCR2;                         /*!< (@ 0x0000001C) DMA channel 2 configuration register                       */
  __IO uint32_t   CNDTR2;                       /*!< (@ 0x00000020) channel x number of data to transfer register              */
  __IO uint32_t   CPAR2;                        /*!< (@ 0x00000024) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR2;                        /*!< (@ 0x00000028) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   CCR3;                         /*!< (@ 0x00000030) DMA channel 3 configuration register                       */
  __IO uint32_t   CNDTR3;                       /*!< (@ 0x00000034) channel x number of data to transfer register              */
  __IO uint32_t   CPAR3;                        /*!< (@ 0x00000038) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR3;                        /*!< (@ 0x0000003C) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000044) DMA channel 3 configuration register                       */
  __IO uint32_t   CNDTR4;                       /*!< (@ 0x00000048) channel x number of data to transfer register              */
  __IO uint32_t   CPAR4;                        /*!< (@ 0x0000004C) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR4;                        /*!< (@ 0x00000050) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED3;
  __IO uint32_t   CCR5;                         /*!< (@ 0x00000058) DMA channel 4 configuration register                       */
  __IO uint32_t   CNDTR5;                       /*!< (@ 0x0000005C) channel x number of data to transfer register              */
  __IO uint32_t   CPAR5;                        /*!< (@ 0x00000060) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR5;                        /*!< (@ 0x00000064) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED4;
  __IO uint32_t   CCR6;                         /*!< (@ 0x0000006C) DMA channel 5 configuration register                       */
  __IO uint32_t   CNDTR6;                       /*!< (@ 0x00000070) channel x number of data to transfer register              */
  __IO uint32_t   CPAR6;                        /*!< (@ 0x00000074) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR6;                        /*!< (@ 0x00000078) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED5;
  __IO uint32_t   CCR7;                         /*!< (@ 0x00000080) DMA channel 6 configuration register                       */
  __IO uint32_t   CNDTR7;                       /*!< (@ 0x00000084) channel x number of data to transfer register              */
  __IO uint32_t   CPAR7;                        /*!< (@ 0x00000088) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR7;                        /*!< (@ 0x0000008C) DMA channel x memory address register                      */
  __I  uint32_t   RESERVED6;
  __IO uint32_t   CCR8;                         /*!< (@ 0x00000094) DMA channel 7 configuration register                       */
  __IO uint32_t   CNDTR8;                       /*!< (@ 0x00000098) channel x number of data to transfer register              */
  __IO uint32_t   CPAR8;                        /*!< (@ 0x0000009C) DMA channel x peripheral address register                  */
  __IO uint32_t   CMAR8;                        /*!< (@ 0x000000A0) DMA channel x memory address register                      */
} DMA1_Type;                                    /*!< Size = 164 (0xa4)                                                         */



/* =========================================================================================================================== */
/* ================                                          DMAMUX                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief DMAMUX (DMAMUX)
  */

typedef struct {                                /*!< (@ 0x40020800) DMAMUX Structure                                           */
  __IO uint32_t   C0CR;                         /*!< (@ 0x00000000) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C1CR;                         /*!< (@ 0x00000004) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C2CR;                         /*!< (@ 0x00000008) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C3CR;                         /*!< (@ 0x0000000C) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C4CR;                         /*!< (@ 0x00000010) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C5CR;                         /*!< (@ 0x00000014) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C6CR;                         /*!< (@ 0x00000018) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C7CR;                         /*!< (@ 0x0000001C) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C8CR;                         /*!< (@ 0x00000020) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C9CR;                         /*!< (@ 0x00000024) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C10CR;                        /*!< (@ 0x00000028) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C11CR;                        /*!< (@ 0x0000002C) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C12CR;                        /*!< (@ 0x00000030) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C13CR;                        /*!< (@ 0x00000034) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C14CR;                        /*!< (@ 0x00000038) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __IO uint32_t   C15CR;                        /*!< (@ 0x0000003C) DMAMux - DMA request line multiplexer channel
                                                                    x control register                                         */
  __I  uint32_t   RESERVED[16];
  __I  uint32_t   CSR;                          /*!< (@ 0x00000080) DMAMUX request line multiplexer interrupt channel
                                                                    status register                                            */
  __O  uint32_t   CFR;                          /*!< (@ 0x00000084) DMAMUX request line multiplexer interrupt clear
                                                                    flag register                                              */
  __I  uint32_t   RESERVED1[30];
  __IO uint32_t   RG0CR;                        /*!< (@ 0x00000100) DMAMux - DMA request generator channel x control
                                                                    register                                                   */
  __IO uint32_t   RG1CR;                        /*!< (@ 0x00000104) DMAMux - DMA request generator channel x control
                                                                    register                                                   */
  __IO uint32_t   RG2CR;                        /*!< (@ 0x00000108) DMAMux - DMA request generator channel x control
                                                                    register                                                   */
  __IO uint32_t   RG3CR;                        /*!< (@ 0x0000010C) DMAMux - DMA request generator channel x control
                                                                    register                                                   */
  __I  uint32_t   RESERVED2[12];
  __I  uint32_t   RGSR;                         /*!< (@ 0x00000140) DMAMux - DMA request generator status register             */
  __O  uint32_t   RGCFR;                        /*!< (@ 0x00000144) DMAMux - DMA request generator clear flag register         */
} DMAMUX_Type;                                  /*!< Size = 328 (0x148)                                                        */



/* =========================================================================================================================== */
/* ================                                          SYSCFG                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief System configuration controller (SYSCFG)
  */

typedef struct {                                /*!< (@ 0x40010000) SYSCFG Structure                                           */
  __IO uint32_t   MEMRMP;                       /*!< (@ 0x00000000) Remap Memory register                                      */
  __IO uint32_t   CFGR1;                        /*!< (@ 0x00000004) peripheral mode configuration register                     */
  __IO uint32_t   EXTICR1;                      /*!< (@ 0x00000008) external interrupt configuration register 1                */
  __IO uint32_t   EXTICR2;                      /*!< (@ 0x0000000C) external interrupt configuration register 2                */
  __IO uint32_t   EXTICR3;                      /*!< (@ 0x00000010) external interrupt configuration register 3                */
  __IO uint32_t   EXTICR4;                      /*!< (@ 0x00000014) external interrupt configuration register 4                */
  __IO uint32_t   SCSR;                         /*!< (@ 0x00000018) CCM SRAM control and status register                       */
  __IO uint32_t   CFGR2;                        /*!< (@ 0x0000001C) configuration register 2                                   */
  __IO uint32_t   SWPR;                         /*!< (@ 0x00000020) SRAM Write protection register 1                           */
  __O  uint32_t   SKR;                          /*!< (@ 0x00000024) SRAM2 Key Register                                         */
} SYSCFG_Type;                                  /*!< Size = 40 (0x28)                                                          */



/* =========================================================================================================================== */
/* ================                                          VREFBUF                                          ================ */
/* =========================================================================================================================== */


/**
  * @brief Voltage reference buffer (VREFBUF)
  */

typedef struct {                                /*!< (@ 0x40010030) VREFBUF Structure                                          */
  __IO uint32_t   VREFBUF_CSR;                  /*!< (@ 0x00000000) VREF_BUF Control and Status Register                       */
  __IO uint32_t   VREFBUF_CCR;                  /*!< (@ 0x00000004) VREF_BUF Calibration Control Register                      */
} VREFBUF_Type;                                 /*!< Size = 8 (0x8)                                                            */



/* =========================================================================================================================== */
/* ================                                           COMP                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Comparator control and status register (COMP)
  */

typedef struct {                                /*!< (@ 0x40010200) COMP Structure                                             */
  __IO uint32_t   COMP_C1CSR;                   /*!< (@ 0x00000000) Comparator control/status register                         */
  __IO uint32_t   COMP_C2CSR;                   /*!< (@ 0x00000004) Comparator control/status register                         */
  __IO uint32_t   COMP_C3CSR;                   /*!< (@ 0x00000008) Comparator control/status register                         */
  __IO uint32_t   COMP_C4CSR;                   /*!< (@ 0x0000000C) Comparator control/status register                         */
} COMP_Type;                                    /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                           OPAMP                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Operational amplifiers (OPAMP)
  */

typedef struct {                                /*!< (@ 0x40010300) OPAMP Structure                                            */
  __IO uint32_t   OPAMP1_CSR;                   /*!< (@ 0x00000000) OPAMP1 control/status register                             */
  __IO uint32_t   OPAMP2_CSR;                   /*!< (@ 0x00000004) OPAMP2 control/status register                             */
  __IO uint32_t   OPAMP3_CSR;                   /*!< (@ 0x00000008) OPAMP3 control/status register                             */
  __I  uint32_t   RESERVED[3];
  __IO uint32_t   OPAMP1_TCMR;                  /*!< (@ 0x00000018) OPAMP1 control/status register                             */
  __IO uint32_t   OPAMP2_TCMR;                  /*!< (@ 0x0000001C) OPAMP2 control/status register                             */
  __IO uint32_t   OPAMP3_TCMR;                  /*!< (@ 0x00000020) OPAMP3 control/status register                             */
} OPAMP_Type;                                   /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                           DAC1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Digital-to-analog converter (DAC1)
  */

typedef struct {                                /*!< (@ 0x50000800) DAC1 Structure                                             */
  __IO uint32_t   DAC_CR;                       /*!< (@ 0x00000000) DAC control register                                       */
  __O  uint32_t   DAC_SWTRGR;                   /*!< (@ 0x00000004) DAC software trigger register                              */
  __IO uint32_t   DAC_DHR12R1;                  /*!< (@ 0x00000008) DAC channel1 12-bit right-aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR12L1;                  /*!< (@ 0x0000000C) DAC channel1 12-bit left aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR8R1;                   /*!< (@ 0x00000010) DAC channel1 8-bit right aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR12R2;                  /*!< (@ 0x00000014) DAC channel2 12-bit right aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR12L2;                  /*!< (@ 0x00000018) DAC channel2 12-bit left aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR8R2;                   /*!< (@ 0x0000001C) DAC channel2 8-bit right-aligned data holding
                                                                    register                                                   */
  __IO uint32_t   DAC_DHR12RD;                  /*!< (@ 0x00000020) Dual DAC 12-bit right-aligned data holding register        */
  __IO uint32_t   DAC_DHR12LD;                  /*!< (@ 0x00000024) DUAL DAC 12-bit left aligned data holding register         */
  __IO uint32_t   DAC_DHR8RD;                   /*!< (@ 0x00000028) DUAL DAC 8-bit right aligned data holding register         */
  __I  uint32_t   DAC_DOR1;                     /*!< (@ 0x0000002C) DAC channel1 data output register                          */
  __I  uint32_t   DAC_DOR2;                     /*!< (@ 0x00000030) DAC channel2 data output register                          */
  __IO uint32_t   DAC_SR;                       /*!< (@ 0x00000034) DAC status register                                        */
  __IO uint32_t   DAC_CCR;                      /*!< (@ 0x00000038) DAC calibration control register                           */
  __IO uint32_t   DAC_MCR;                      /*!< (@ 0x0000003C) DAC mode control register                                  */
  __IO uint32_t   DAC_SHSR1;                    /*!< (@ 0x00000040) DAC Sample and Hold sample time register 1                 */
  __IO uint32_t   DAC_SHSR2;                    /*!< (@ 0x00000044) DAC Sample and Hold sample time register 2                 */
  __IO uint32_t   DAC_SHHR;                     /*!< (@ 0x00000048) DAC Sample and Hold hold time register                     */
  __IO uint32_t   DAC_SHRR;                     /*!< (@ 0x0000004C) DAC Sample and Hold refresh time register                  */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   DAC_STR1;                     /*!< (@ 0x00000058) Sawtooth register                                          */
  __IO uint32_t   DAC_STR2;                     /*!< (@ 0x0000005C) Sawtooth register                                          */
  __IO uint32_t   DAC_STMODR;                   /*!< (@ 0x00000060) Sawtooth Mode register                                     */
} DAC1_Type;                                    /*!< Size = 100 (0x64)                                                         */



/* =========================================================================================================================== */
/* ================                                           ADC1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Analog-to-Digital Converter (ADC1)
  */

typedef struct {                                /*!< (@ 0x50000000) ADC1 Structure                                             */
  __IO uint32_t   ISR;                          /*!< (@ 0x00000000) interrupt and status register                              */
  __IO uint32_t   IER;                          /*!< (@ 0x00000004) interrupt enable register                                  */
  __IO uint32_t   CR;                           /*!< (@ 0x00000008) control register                                           */
  __IO uint32_t   CFGR;                         /*!< (@ 0x0000000C) configuration register                                     */
  __IO uint32_t   CFGR2;                        /*!< (@ 0x00000010) configuration register                                     */
  __IO uint32_t   SMPR1;                        /*!< (@ 0x00000014) sample time register 1                                     */
  __IO uint32_t   SMPR2;                        /*!< (@ 0x00000018) sample time register 2                                     */
  __I  uint32_t   RESERVED;
  __IO uint32_t   TR1;                          /*!< (@ 0x00000020) watchdog threshold register 1                              */
  __IO uint32_t   TR2;                          /*!< (@ 0x00000024) watchdog threshold register                                */
  __IO uint32_t   TR3;                          /*!< (@ 0x00000028) watchdog threshold register 3                              */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   SQR1;                         /*!< (@ 0x00000030) regular sequence register 1                                */
  __IO uint32_t   SQR2;                         /*!< (@ 0x00000034) regular sequence register 2                                */
  __IO uint32_t   SQR3;                         /*!< (@ 0x00000038) regular sequence register 3                                */
  __IO uint32_t   SQR4;                         /*!< (@ 0x0000003C) regular sequence register 4                                */
  __I  uint32_t   DR;                           /*!< (@ 0x00000040) regular Data Register                                      */
  __I  uint32_t   RESERVED2[2];
  __IO uint32_t   JSQR;                         /*!< (@ 0x0000004C) injected sequence register                                 */
  __I  uint32_t   RESERVED3[4];
  __IO uint32_t   OFR1;                         /*!< (@ 0x00000060) offset register 1                                          */
  __IO uint32_t   OFR2;                         /*!< (@ 0x00000064) offset register 2                                          */
  __IO uint32_t   OFR3;                         /*!< (@ 0x00000068) offset register 3                                          */
  __IO uint32_t   OFR4;                         /*!< (@ 0x0000006C) offset register 4                                          */
  __I  uint32_t   RESERVED4[4];
  __I  uint32_t   JDR1;                         /*!< (@ 0x00000080) injected data register 1                                   */
  __I  uint32_t   JDR2;                         /*!< (@ 0x00000084) injected data register 2                                   */
  __I  uint32_t   JDR3;                         /*!< (@ 0x00000088) injected data register 3                                   */
  __I  uint32_t   JDR4;                         /*!< (@ 0x0000008C) injected data register 4                                   */
  __I  uint32_t   RESERVED5[4];
  __IO uint32_t   AWD2CR;                       /*!< (@ 0x000000A0) Analog Watchdog 2 Configuration Register                   */
  __IO uint32_t   AWD3CR;                       /*!< (@ 0x000000A4) Analog Watchdog 3 Configuration Register                   */
  __I  uint32_t   RESERVED6[2];
  __IO uint32_t   DIFSEL;                       /*!< (@ 0x000000B0) Differential Mode Selection Register 2                     */
  __IO uint32_t   CALFACT;                      /*!< (@ 0x000000B4) Calibration Factors                                        */
  __I  uint32_t   RESERVED7[2];
  __IO uint32_t   GCOMP;                        /*!< (@ 0x000000C0) Gain compensation Register                                 */
} ADC1_Type;                                    /*!< Size = 196 (0xc4)                                                         */



/* =========================================================================================================================== */
/* ================                                           ADC3                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Analog-to-Digital Converter (ADC3)
  */

typedef struct {                                /*!< (@ 0x50000400) ADC3 Structure                                             */
  __IO uint32_t   ISR;                          /*!< (@ 0x00000000) interrupt and status register                              */
  __IO uint32_t   IER;                          /*!< (@ 0x00000004) interrupt enable register                                  */
  __IO uint32_t   CR;                           /*!< (@ 0x00000008) control register                                           */
  __IO uint32_t   CFGR;                         /*!< (@ 0x0000000C) configuration register                                     */
  __IO uint32_t   CFGR2;                        /*!< (@ 0x00000010) configuration register                                     */
  __IO uint32_t   SMPR1;                        /*!< (@ 0x00000014) sample time register 1                                     */
  __IO uint32_t   SMPR2;                        /*!< (@ 0x00000018) sample time register 2                                     */
  __I  uint32_t   RESERVED;
  __IO uint32_t   TR1;                          /*!< (@ 0x00000020) watchdog threshold register 1                              */
  __IO uint32_t   TR2;                          /*!< (@ 0x00000024) watchdog threshold register                                */
  __IO uint32_t   TR3;                          /*!< (@ 0x00000028) watchdog threshold register 3                              */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   SQR1;                         /*!< (@ 0x00000030) regular sequence register 1                                */
  __IO uint32_t   SQR2;                         /*!< (@ 0x00000034) regular sequence register 2                                */
  __IO uint32_t   SQR3;                         /*!< (@ 0x00000038) regular sequence register 3                                */
  __IO uint32_t   SQR4;                         /*!< (@ 0x0000003C) regular sequence register 4                                */
  __I  uint32_t   DR;                           /*!< (@ 0x00000040) regular Data Register                                      */
  __I  uint32_t   RESERVED2[2];
  __IO uint32_t   JSQR;                         /*!< (@ 0x0000004C) injected sequence register                                 */
  __I  uint32_t   RESERVED3[4];
  __IO uint32_t   OFR1;                         /*!< (@ 0x00000060) offset register 1                                          */
  __IO uint32_t   OFR2;                         /*!< (@ 0x00000064) offset register 2                                          */
  __IO uint32_t   OFR3;                         /*!< (@ 0x00000068) offset register 3                                          */
  __IO uint32_t   OFR4;                         /*!< (@ 0x0000006C) offset register 4                                          */
  __I  uint32_t   RESERVED4[4];
  __I  uint32_t   JDR1;                         /*!< (@ 0x00000080) injected data register 1                                   */
  __I  uint32_t   JDR2;                         /*!< (@ 0x00000084) injected data register 2                                   */
  __I  uint32_t   JDR3;                         /*!< (@ 0x00000088) injected data register 3                                   */
  __I  uint32_t   JDR4;                         /*!< (@ 0x0000008C) injected data register 4                                   */
  __I  uint32_t   RESERVED5[4];
  __IO uint32_t   AWD2CR;                       /*!< (@ 0x000000A0) Analog Watchdog 2 Configuration Register                   */
  __IO uint32_t   AWD3CR;                       /*!< (@ 0x000000A4) Analog Watchdog 3 Configuration Register                   */
  __I  uint32_t   RESERVED6[2];
  __IO uint32_t   DIFSEL;                       /*!< (@ 0x000000B0) Differential Mode Selection Register 2                     */
  __IO uint32_t   CALFACT;                      /*!< (@ 0x000000B4) Calibration Factors                                        */
  __I  uint32_t   RESERVED7[2];
  __IO uint32_t   GCOMP;                        /*!< (@ 0x000000C0) Gain compensation Register                                 */
} ADC3_Type;                                    /*!< Size = 196 (0xc4)                                                         */



/* =========================================================================================================================== */
/* ================                                       ADC12_Common                                        ================ */
/* =========================================================================================================================== */


/**
  * @brief Analog-to-Digital Converter (ADC12_Common)
  */

typedef struct {                                /*!< (@ 0x50000300) ADC12_Common Structure                                     */
  __I  uint32_t   CSR;                          /*!< (@ 0x00000000) ADC Common status register                                 */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCR;                          /*!< (@ 0x00000008) ADC common control register                                */
  __I  uint32_t   CDR;                          /*!< (@ 0x0000000C) ADC common regular data register for dual and
                                                                    triple modes                                               */
} ADC12_Common_Type;                            /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                           FMAC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Filter Math Accelerator (FMAC)
  */

typedef struct {                                /*!< (@ 0x40021400) FMAC Structure                                             */
  __IO uint32_t   X1BUFCFG;                     /*!< (@ 0x00000000) FMAC X1 Buffer Configuration register                      */
  __IO uint32_t   X2BUFCFG;                     /*!< (@ 0x00000004) FMAC X2 Buffer Configuration register                      */
  __IO uint32_t   YBUFCFG;                      /*!< (@ 0x00000008) FMAC Y Buffer Configuration register                       */
  __IO uint32_t   PARAM;                        /*!< (@ 0x0000000C) FMAC Parameter register                                    */
  __IO uint32_t   CR;                           /*!< (@ 0x00000010) FMAC Control register                                      */
  __I  uint32_t   SR;                           /*!< (@ 0x00000014) FMAC Status register                                       */
  __O  uint32_t   WDATA;                        /*!< (@ 0x00000018) FMAC Write Data register                                   */
  __I  uint32_t   RDATA;                        /*!< (@ 0x0000001C) FMAC Read Data register                                    */
} FMAC_Type;                                    /*!< Size = 32 (0x20)                                                          */



/* =========================================================================================================================== */
/* ================                                          CORDIC                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief CORDIC Co-processor (CORDIC)
  */

typedef struct {                                /*!< (@ 0x40020C00) CORDIC Structure                                           */
  __IO uint32_t   CSR;                          /*!< (@ 0x00000000) CORDIC Control Status register                             */
  __IO uint32_t   WDATA;                        /*!< (@ 0x00000004) FMAC Write Data register                                   */
  __I  uint32_t   RDATA;                        /*!< (@ 0x00000008) FMAC Read Data register                                    */
} CORDIC_Type;                                  /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                            SAI                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Serial audio interface (SAI)
  */

typedef struct {                                /*!< (@ 0x40015400) SAI Structure                                              */
  __I  uint32_t   RESERVED;
  __IO uint32_t   ACR1;                         /*!< (@ 0x00000004) AConfiguration register 1                                  */
  __IO uint32_t   ACR2;                         /*!< (@ 0x00000008) AConfiguration register 2                                  */
  __IO uint32_t   AFRCR;                        /*!< (@ 0x0000000C) AFRCR                                                      */
  __IO uint32_t   ASLOTR;                       /*!< (@ 0x00000010) ASlot register                                             */
  __IO uint32_t   AIM;                          /*!< (@ 0x00000014) AInterrupt mask register2                                  */
  __IO uint32_t   ASR;                          /*!< (@ 0x00000018) AStatus register                                           */
  __IO uint32_t   ACLRFR;                       /*!< (@ 0x0000001C) AClear flag register                                       */
  __IO uint32_t   ADR;                          /*!< (@ 0x00000020) AData register                                             */
  __IO uint32_t   BCR1;                         /*!< (@ 0x00000024) BConfiguration register 1                                  */
  __IO uint32_t   BCR2;                         /*!< (@ 0x00000028) BConfiguration register 2                                  */
  __IO uint32_t   BFRCR;                        /*!< (@ 0x0000002C) BFRCR                                                      */
  __IO uint32_t   BSLOTR;                       /*!< (@ 0x00000030) BSlot register                                             */
  __IO uint32_t   BIM;                          /*!< (@ 0x00000034) BInterrupt mask register2                                  */
  __I  uint32_t   BSR;                          /*!< (@ 0x00000038) BStatus register                                           */
  __O  uint32_t   BCLRFR;                       /*!< (@ 0x0000003C) BClear flag register                                       */
  __IO uint32_t   BDR;                          /*!< (@ 0x00000040) BData register                                             */
  __IO uint32_t   PDMCR;                        /*!< (@ 0x00000044) PDM control register                                       */
  __IO uint32_t   PDMDLY;                       /*!< (@ 0x00000048) PDM delay register                                         */
} SAI_Type;                                     /*!< Size = 76 (0x4c)                                                          */



/* =========================================================================================================================== */
/* ================                                           TAMP                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Tamper and backup registers (TAMP)
  */

typedef struct {                                /*!< (@ 0x40002400) TAMP Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __I  uint32_t   RESERVED;
  __IO uint32_t   FLTCR;                        /*!< (@ 0x0000000C) TAMP filter control register                               */
  __I  uint32_t   RESERVED1[7];
  __IO uint32_t   IER;                          /*!< (@ 0x0000002C) TAMP interrupt enable register                             */
  __I  uint32_t   SR;                           /*!< (@ 0x00000030) TAMP status register                                       */
  __I  uint32_t   MISR;                         /*!< (@ 0x00000034) TAMP masked interrupt status register                      */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   SCR;                          /*!< (@ 0x0000003C) TAMP status clear register                                 */
  __I  uint32_t   RESERVED3[48];
  __IO uint32_t   BKP0R;                        /*!< (@ 0x00000100) TAMP backup register                                       */
  __IO uint32_t   BKP1R;                        /*!< (@ 0x00000104) TAMP backup register                                       */
  __IO uint32_t   BKP2R;                        /*!< (@ 0x00000108) TAMP backup register                                       */
  __IO uint32_t   BKP3R;                        /*!< (@ 0x0000010C) TAMP backup register                                       */
  __IO uint32_t   BKP4R;                        /*!< (@ 0x00000110) TAMP backup register                                       */
  __IO uint32_t   BKP5R;                        /*!< (@ 0x00000114) TAMP backup register                                       */
  __IO uint32_t   BKP6R;                        /*!< (@ 0x00000118) TAMP backup register                                       */
  __IO uint32_t   BKP7R;                        /*!< (@ 0x0000011C) TAMP backup register                                       */
  __IO uint32_t   BKP8R;                        /*!< (@ 0x00000120) TAMP backup register                                       */
  __IO uint32_t   BKP9R;                        /*!< (@ 0x00000124) TAMP backup register                                       */
  __IO uint32_t   BKP10R;                       /*!< (@ 0x00000128) TAMP backup register                                       */
  __IO uint32_t   BKP11R;                       /*!< (@ 0x0000012C) TAMP backup register                                       */
  __IO uint32_t   BKP12R;                       /*!< (@ 0x00000130) TAMP backup register                                       */
  __IO uint32_t   BKP13R;                       /*!< (@ 0x00000134) TAMP backup register                                       */
  __IO uint32_t   BKP14R;                       /*!< (@ 0x00000138) TAMP backup register                                       */
  __IO uint32_t   BKP15R;                       /*!< (@ 0x0000013C) TAMP backup register                                       */
  __IO uint32_t   BKP16R;                       /*!< (@ 0x00000140) TAMP backup register                                       */
  __IO uint32_t   BKP17R;                       /*!< (@ 0x00000144) TAMP backup register                                       */
  __IO uint32_t   BKP18R;                       /*!< (@ 0x00000148) TAMP backup register                                       */
  __IO uint32_t   BKP19R;                       /*!< (@ 0x0000014C) TAMP backup register                                       */
  __IO uint32_t   BKP20R;                       /*!< (@ 0x00000150) TAMP backup register                                       */
  __IO uint32_t   BKP21R;                       /*!< (@ 0x00000154) TAMP backup register                                       */
  __IO uint32_t   BKP22R;                       /*!< (@ 0x00000158) TAMP backup register                                       */
  __IO uint32_t   BKP23R;                       /*!< (@ 0x0000015C) TAMP backup register                                       */
  __IO uint32_t   BKP24R;                       /*!< (@ 0x00000160) TAMP backup register                                       */
  __IO uint32_t   BKP25R;                       /*!< (@ 0x00000164) TAMP backup register                                       */
  __IO uint32_t   BKP26R;                       /*!< (@ 0x00000168) TAMP backup register                                       */
  __IO uint32_t   BKP27R;                       /*!< (@ 0x0000016C) TAMP backup register                                       */
  __IO uint32_t   BKP28R;                       /*!< (@ 0x00000170) TAMP backup register                                       */
  __IO uint32_t   BKP29R;                       /*!< (@ 0x00000174) TAMP backup register                                       */
  __IO uint32_t   BKP30R;                       /*!< (@ 0x00000178) TAMP backup register                                       */
  __IO uint32_t   BKP31R;                       /*!< (@ 0x0000017C) TAMP backup register                                       */
} TAMP_Type;                                    /*!< Size = 384 (0x180)                                                        */



/* =========================================================================================================================== */
/* ================                                            FPU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Floting point unit (FPU)
  */

typedef struct {                                /*!< (@ 0xE000EF34) FPU Structure                                              */
  __IO uint32_t   FPCCR;                        /*!< (@ 0x00000000) Floating-point context control register                    */
  __IO uint32_t   FPCAR;                        /*!< (@ 0x00000004) Floating-point context address register                    */
  __IO uint32_t   FPSCR;                        /*!< (@ 0x00000008) Floating-point status control register                     */
} FPU_Type;                                     /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                            MPU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Memory protection unit (MPU)
  */

typedef struct {                                /*!< (@ 0xE000E084) MPU Structure                                              */
  __I  uint32_t   TYPER;                        /*!< (@ 0x00000000) MPU type register                                          */
  __IO uint32_t   CTRL;                         /*!< (@ 0x00000004) MPU control register                                       */
  __IO uint32_t   RNR;                          /*!< (@ 0x00000008) MPU region number register                                 */
  __IO uint32_t   RBAR;                         /*!< (@ 0x0000000C) MPU region base address register                           */
  __IO uint32_t   RASR;                         /*!< (@ 0x00000010) MPU region attribute and size register                     */
} MPU_Type;                                     /*!< Size = 20 (0x14)                                                          */



/* =========================================================================================================================== */
/* ================                                            STK                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief SysTick timer (STK)
  */

typedef struct {                                /*!< (@ 0xE000E010) STK Structure                                              */
  __IO uint32_t   CTRL;                         /*!< (@ 0x00000000) SysTick control and status register                        */
  __IO uint32_t   LOAD;                         /*!< (@ 0x00000004) SysTick reload value register                              */
  __IO uint32_t   VAL;                          /*!< (@ 0x00000008) SysTick current value register                             */
  __IO uint32_t   CALIB;                        /*!< (@ 0x0000000C) SysTick calibration value register                         */
} STK_Type;                                     /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                            SCB                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief System control block (SCB)
  */

typedef struct {                                /*!< (@ 0xE000ED00) SCB Structure                                              */
  __I  uint32_t   CPUID;                        /*!< (@ 0x00000000) CPUID base register                                        */
  __IO uint32_t   ICSR;                         /*!< (@ 0x00000004) Interrupt control and state register                       */
  __IO uint32_t   VTOR;                         /*!< (@ 0x00000008) Vector table offset register                               */
  __IO uint32_t   AIRCR;                        /*!< (@ 0x0000000C) Application interrupt and reset control register           */
  __IO uint32_t   SCR;                          /*!< (@ 0x00000010) System control register                                    */
  __IO uint32_t   CCR;                          /*!< (@ 0x00000014) Configuration and control register                         */
  __IO uint32_t   SHPR1;                        /*!< (@ 0x00000018) System handler priority registers                          */
  __IO uint32_t   SHPR2;                        /*!< (@ 0x0000001C) System handler priority registers                          */
  __IO uint32_t   SHPR3;                        /*!< (@ 0x00000020) System handler priority registers                          */
  __IO uint32_t   SHCSR;                        /*!< (@ 0x00000024) System handler control and state register                  */
  __IO uint32_t   CFSR_UFSR_BFSR_MMFSR;         /*!< (@ 0x00000028) Configurable fault status register                         */
  __IO uint32_t   HFSR;                         /*!< (@ 0x0000002C) Hard fault status register                                 */
  __I  uint32_t   RESERVED;
  __IO uint32_t   MMFAR;                        /*!< (@ 0x00000034) Memory management fault address register                   */
  __IO uint32_t   BFAR;                         /*!< (@ 0x00000038) Bus fault address register                                 */
  __IO uint32_t   AFSR;                         /*!< (@ 0x0000003C) Auxiliary fault status register                            */
} SCB_Type;                                     /*!< Size = 64 (0x40)                                                          */



/* =========================================================================================================================== */
/* ================                                           NVIC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Nested Vectored Interrupt Controller (NVIC)
  */

typedef struct {                                /*!< (@ 0xE000E100) NVIC Structure                                             */
  __IO uint32_t   ISER0;                        /*!< (@ 0x00000000) Interrupt Set-Enable Register                              */
  __IO uint32_t   ISER1;                        /*!< (@ 0x00000004) Interrupt Set-Enable Register                              */
  __IO uint32_t   ISER2;                        /*!< (@ 0x00000008) Interrupt Set-Enable Register                              */
  __IO uint32_t   ISER3;                        /*!< (@ 0x0000000C) Interrupt Set-Enable Register                              */
  __I  uint32_t   RESERVED[28];
  __IO uint32_t   ICER0;                        /*!< (@ 0x00000080) Interrupt Clear-Enable Register                            */
  __IO uint32_t   ICER1;                        /*!< (@ 0x00000084) Interrupt Clear-Enable Register                            */
  __IO uint32_t   ICER2;                        /*!< (@ 0x00000088) Interrupt Clear-Enable Register                            */
  __IO uint32_t   ICER3;                        /*!< (@ 0x0000008C) Interrupt Clear-Enable Register                            */
  __I  uint32_t   RESERVED1[28];
  __IO uint32_t   ISPR0;                        /*!< (@ 0x00000100) Interrupt Set-Pending Register                             */
  __IO uint32_t   ISPR1;                        /*!< (@ 0x00000104) Interrupt Set-Pending Register                             */
  __IO uint32_t   ISPR2;                        /*!< (@ 0x00000108) Interrupt Set-Pending Register                             */
  __IO uint32_t   ISPR3;                        /*!< (@ 0x0000010C) Interrupt Set-Pending Register                             */
  __I  uint32_t   RESERVED2[28];
  __IO uint32_t   ICPR0;                        /*!< (@ 0x00000180) Interrupt Clear-Pending Register                           */
  __IO uint32_t   ICPR1;                        /*!< (@ 0x00000184) Interrupt Clear-Pending Register                           */
  __IO uint32_t   ICPR2;                        /*!< (@ 0x00000188) Interrupt Clear-Pending Register                           */
  __IO uint32_t   ICPR3;                        /*!< (@ 0x0000018C) Interrupt Clear-Pending Register                           */
  __I  uint32_t   RESERVED3[28];
  __I  uint32_t   IABR0;                        /*!< (@ 0x00000200) Interrupt Active Bit Register                              */
  __I  uint32_t   IABR1;                        /*!< (@ 0x00000204) Interrupt Active Bit Register                              */
  __I  uint32_t   IABR2;                        /*!< (@ 0x00000208) Interrupt Active Bit Register                              */
  __I  uint32_t   IABR3;                        /*!< (@ 0x0000020C) Interrupt Active Bit Register                              */
  __I  uint32_t   RESERVED4[60];
  __IO uint32_t   IPR0;                         /*!< (@ 0x00000300) Interrupt Priority Register                                */
  __IO uint32_t   IPR1;                         /*!< (@ 0x00000304) Interrupt Priority Register                                */
  __IO uint32_t   IPR2;                         /*!< (@ 0x00000308) Interrupt Priority Register                                */
  __IO uint32_t   IPR3;                         /*!< (@ 0x0000030C) Interrupt Priority Register                                */
  __IO uint32_t   IPR4;                         /*!< (@ 0x00000310) Interrupt Priority Register                                */
  __IO uint32_t   IPR5;                         /*!< (@ 0x00000314) Interrupt Priority Register                                */
  __IO uint32_t   IPR6;                         /*!< (@ 0x00000318) Interrupt Priority Register                                */
  __IO uint32_t   IPR7;                         /*!< (@ 0x0000031C) Interrupt Priority Register                                */
  __IO uint32_t   IPR8;                         /*!< (@ 0x00000320) Interrupt Priority Register                                */
  __IO uint32_t   IPR9;                         /*!< (@ 0x00000324) Interrupt Priority Register                                */
  __IO uint32_t   IPR10;                        /*!< (@ 0x00000328) Interrupt Priority Register                                */
  __IO uint32_t   IPR11;                        /*!< (@ 0x0000032C) Interrupt Priority Register                                */
  __IO uint32_t   IPR12;                        /*!< (@ 0x00000330) Interrupt Priority Register                                */
  __IO uint32_t   IPR13;                        /*!< (@ 0x00000334) Interrupt Priority Register                                */
  __IO uint32_t   IPR14;                        /*!< (@ 0x00000338) Interrupt Priority Register                                */
  __IO uint32_t   IPR15;                        /*!< (@ 0x0000033C) Interrupt Priority Register                                */
  __IO uint32_t   IPR16;                        /*!< (@ 0x00000340) Interrupt Priority Register                                */
  __IO uint32_t   IPR17;                        /*!< (@ 0x00000344) Interrupt Priority Register                                */
  __IO uint32_t   IPR18;                        /*!< (@ 0x00000348) Interrupt Priority Register                                */
  __IO uint32_t   IPR19;                        /*!< (@ 0x0000034C) Interrupt Priority Register                                */
  __IO uint32_t   IPR20;                        /*!< (@ 0x00000350) Interrupt Priority Register                                */
  __IO uint32_t   IPR21;                        /*!< (@ 0x00000354) Interrupt Priority Register                                */
  __IO uint32_t   IPR22;                        /*!< (@ 0x00000358) Interrupt Priority Register                                */
  __IO uint32_t   IPR23;                        /*!< (@ 0x0000035C) Interrupt Priority Register                                */
  __IO uint32_t   IPR24;                        /*!< (@ 0x00000360) Interrupt Priority Register                                */
  __IO uint32_t   IPR25;                        /*!< (@ 0x00000364) Interrupt Priority Register                                */
} NVIC_Type;                                    /*!< Size = 872 (0x368)                                                        */



/* =========================================================================================================================== */
/* ================                                         NVIC_STIR                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief Nested vectored interrupt  controller (NVIC_STIR)
  */

typedef struct {                                /*!< (@ 0xE000EF00) NVIC_STIR Structure                                        */
  __IO uint32_t   STIR;                         /*!< (@ 0x00000000) Software trigger interrupt register                        */
} NVIC_STIR_Type;                               /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                         FPU_CPACR                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief Floating point unit CPACR (FPU_CPACR)
  */

typedef struct {                                /*!< (@ 0xE000ED88) FPU_CPACR Structure                                        */
  __IO uint32_t   CPACR;                        /*!< (@ 0x00000000) Coprocessor access control register                        */
} FPU_CPACR_Type;                               /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                         SCB_ACTLR                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief System control block ACTLR (SCB_ACTLR)
  */

typedef struct {                                /*!< (@ 0xE000E008) SCB_ACTLR Structure                                        */
  __IO uint32_t   ACTRL;                        /*!< (@ 0x00000000) Auxiliary control register                                 */
} SCB_ACTLR_Type;                               /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                           FDCAN                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief FDCAN (FDCAN)
  */

typedef struct {                                /*!< (@ 0x4000A400) FDCAN Structure                                            */
  __I  uint32_t   CREL;                         /*!< (@ 0x00000000) FDCAN Core Release Register                                */
  __I  uint32_t   ENDN;                         /*!< (@ 0x00000004) FDCAN Core Release Register                                */
  __I  uint32_t   RESERVED;
  __IO uint32_t   DBTP;                         /*!< (@ 0x0000000C) This register is only writable if bits CCCR.CCE
                                                                    and CCCR.INIT are set. The CAN bit time
                                                                    may be programed in the range of 4 to 25
                                                                    time quanta. The CAN time quantum may be
                                                                    programmed in the range of 1 to 1024 FDCAN
                                                                    clock periods. tq = (DBRP + 1) FDCAN clock
                                                                    period. DTSEG1 is the sum of Prop_Seg and
                                                                    Phase_Seg1. DTSEG2 is Phase_Seg2. Therefore
                                                                    the length of the bit time is (programmed
                                                                    values) [DTSEG1 + DTSEG2 + 3] tq or (functional
                                                                    values) [Sync_Seg + Prop_Seg + Phase_Seg1
                                                                    + Phase_Seg2] tq. The                                      */
  __IO uint32_t   TEST;                         /*!< (@ 0x00000010) Write access to the Test Register has to be enabled
                                                                    by setting bit CCCR[TEST] to 1 . All Test
                                                                    Register functions are set to their reset
                                                                    values when bit CCCR[TEST] is reset. Loop
                                                                    Back mode and software control of Tx pin
                                                                    FDCANx_TX are hardware test modes. Programming
                                                                    TX differently from 00 may disturb the
                                                                    message transfer on the CAN bus.                           */
  __IO uint32_t   RWD;                          /*!< (@ 0x00000014) The RAM Watchdog monitors the READY output of
                                                                    the Message RAM. A Message RAM access starts
                                                                    the Message RAM Watchdog Counter with the
                                                                    value configured by the RWD[WDC] bits.
                                                                    The counter is reloaded with RWD[WDC] bits
                                                                    when the Message RAM signals successful
                                                                    completion by activating its READY output.
                                                                    In case there is no response from the Message
                                                                    RAM until the counter has counted down
                                                                    to 0, the counter stops and interrupt flag
                                                                    IR[WDI] bit is set. The RAM Watchdog Counter
                                                                    is clocked by the fdcan_p                                  */
  __IO uint32_t   CCCR;                         /*!< (@ 0x00000018) For details about setting and resetting of single
                                                                    bits see Software initialization.                          */
  __IO uint32_t   NBTP;                         /*!< (@ 0x0000001C) FDCAN_NBTP                                                 */
  __IO uint32_t   TSCC;                         /*!< (@ 0x00000020) FDCAN Timestamp Counter Configuration Register             */
  __I  uint32_t   TSCV;                         /*!< (@ 0x00000024) FDCAN Timestamp Counter Value Register                     */
  __IO uint32_t   TOCC;                         /*!< (@ 0x00000028) FDCAN Timeout Counter Configuration Register               */
  __I  uint32_t   TOCV;                         /*!< (@ 0x0000002C) FDCAN Timeout Counter Value Register                       */
  __I  uint32_t   RESERVED1[4];
  __I  uint32_t   ECR;                          /*!< (@ 0x00000040) FDCAN Error Counter Register                               */
  __IO uint32_t   PSR;                          /*!< (@ 0x00000044) FDCAN Protocol Status Register                             */
  __IO uint32_t   TDCR;                         /*!< (@ 0x00000048) FDCAN Transmitter Delay Compensation Register              */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   IR;                           /*!< (@ 0x00000050) The flags are set when one of the listed conditions
                                                                    is detected (edge-sensitive). The flags
                                                                    remain set until the Host clears them.
                                                                    A flag is cleared by writing a 1 to the
                                                                    corresponding bit position. Writing a 0
                                                                    has no effect. A hard reset will clear
                                                                    the register. The configuration of IE controls
                                                                    whether an interrupt is generated. The
                                                                    configuration of ILS controls on which
                                                                    interrupt line an interrupt is signaled.                   */
  __IO uint32_t   IE;                           /*!< (@ 0x00000054) The settings in the Interrupt Enable register
                                                                    determine which status changes in the Interrupt
                                                                    Register will be signaled on an interrupt
                                                                    line.                                                      */
  __IO uint32_t   ILS;                          /*!< (@ 0x00000058) The Interrupt Line Select register assigns an
                                                                    interrupt generated by a specific interrupt
                                                                    flag from the Interrupt Register to one
                                                                    of the two module interrupt lines. For
                                                                    interrupt generation the respective interrupt
                                                                    line has to be enabled via ILE[EINT0] and
                                                                    ILE[EINT1].                                                */
  __IO uint32_t   ILE;                          /*!< (@ 0x0000005C) Each of the two interrupt lines to the CPU can
                                                                    be enabled/disabled separately by programming
                                                                    bits EINT0 and EINT1.                                      */
  __I  uint32_t   RESERVED3[8];
  __IO uint32_t   RXGFC;                        /*!< (@ 0x00000080) Global settings for Message ID filtering. The
                                                                    Global Filter Configuration controls the
                                                                    filter path for standard and extended messages
                                                                    as described in Figure706: Standard Message
                                                                    ID filter path and Figure707: Extended
                                                                    Message ID filter path.                                    */
  __IO uint32_t   XIDAM;                        /*!< (@ 0x00000084) FDCAN Extended ID and Mask Register                        */
  __I  uint32_t   HPMS;                         /*!< (@ 0x00000088) This register is updated every time a Message
                                                                    ID filter element configured to generate
                                                                    a priority event match. This can be used
                                                                    to monitor the status of incoming high
                                                                    priority messages and to enable fast access
                                                                    to these messages.                                         */
  __I  uint32_t   RESERVED4;
  __I  uint32_t   RXF0S;                        /*!< (@ 0x00000090) FDCAN Rx FIFO 0 Status Register                            */
  __IO uint32_t   RXF0A;                        /*!< (@ 0x00000094) CAN Rx FIFO 0 Acknowledge Register                         */
  __I  uint32_t   RXF1S;                        /*!< (@ 0x00000098) FDCAN Rx FIFO 1 Status Register                            */
  __IO uint32_t   RXF1A;                        /*!< (@ 0x0000009C) FDCAN Rx FIFO 1 Acknowledge Register                       */
  __I  uint32_t   RESERVED5[8];
  __IO uint32_t   TXBC;                         /*!< (@ 0x000000C0) FDCAN Tx Buffer Configuration Register                     */
  __I  uint32_t   TXFQS;                        /*!< (@ 0x000000C4) The Tx FIFO/Queue status is related to the pending
                                                                    Tx requests listed in register TXBRP. Therefore
                                                                    the effect of Add/Cancellation requests
                                                                    may be delayed due to a running Tx scan
                                                                    (TXBRP not yet updated).                                   */
  __I  uint32_t   TXBRP;                        /*!< (@ 0x000000C8) FDCAN Tx Buffer Request Pending Register                   */
  __IO uint32_t   TXBAR;                        /*!< (@ 0x000000CC) FDCAN Tx Buffer Add Request Register                       */
  __IO uint32_t   TXBCR;                        /*!< (@ 0x000000D0) FDCAN Tx Buffer Cancellation Request Register              */
  __I  uint32_t   TXBTO;                        /*!< (@ 0x000000D4) FDCAN Tx Buffer Transmission Occurred Register             */
  __I  uint32_t   TXBCF;                        /*!< (@ 0x000000D8) FDCAN Tx Buffer Cancellation Finished Register             */
  __IO uint32_t   TXBTIE;                       /*!< (@ 0x000000DC) FDCAN Tx Buffer Transmission Interrupt Enable
                                                                    Register                                                   */
  __IO uint32_t   TXBCIE;                       /*!< (@ 0x000000E0) FDCAN Tx Buffer Cancellation Finished Interrupt
                                                                    Enable Register                                            */
  __I  uint32_t   TXEFS;                        /*!< (@ 0x000000E4) FDCAN Tx Event FIFO Status Register                        */
  __IO uint32_t   TXEFA;                        /*!< (@ 0x000000E8) FDCAN Tx Event FIFO Acknowledge Register                   */
  __I  uint32_t   RESERVED6[5];
  __IO uint32_t   CKDIV;                        /*!< (@ 0x00000100) FDCAN CFG clock divider register                           */
} FDCAN_Type;                                   /*!< Size = 260 (0x104)                                                        */



/* =========================================================================================================================== */
/* ================                                           UCPD1                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief UCPD1 (UCPD1)
  */

typedef struct {                                /*!< (@ 0x4000A000) UCPD1 Structure                                            */
  __IO uint32_t   CFG1;                         /*!< (@ 0x00000000) UCPD configuration register 1                              */
  __IO uint32_t   CFG2;                         /*!< (@ 0x00000004) UCPD configuration register 2                              */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CR;                           /*!< (@ 0x0000000C) UCPD configuration register 2                              */
  __IO uint32_t   IMR;                          /*!< (@ 0x00000010) UCPD Interrupt Mask Register                               */
  __IO uint32_t   SR;                           /*!< (@ 0x00000014) UCPD Status Register                                       */
  __IO uint32_t   ICR;                          /*!< (@ 0x00000018) UCPD Interrupt Clear Register                              */
  __IO uint32_t   TX_ORDSET;                    /*!< (@ 0x0000001C) UCPD Tx Ordered Set Type Register                          */
  __IO uint32_t   TX_PAYSZ;                     /*!< (@ 0x00000020) UCPD Tx Paysize Register                                   */
  __IO uint32_t   TXDR;                         /*!< (@ 0x00000024) UCPD Tx Data Register                                      */
  __I  uint32_t   RX_ORDSET;                    /*!< (@ 0x00000028) UCPD Rx Ordered Set Register                               */
  __I  uint32_t   RX_PAYSZ;                     /*!< (@ 0x0000002C) UCPD Rx Paysize Register                                   */
  __I  uint32_t   RXDR;                         /*!< (@ 0x00000030) UCPD Rx Data Register                                      */
  __IO uint32_t   RX_ORDEXT1;                   /*!< (@ 0x00000034) UCPD Rx Ordered Set Extension Register 1                   */
  __IO uint32_t   RX_ORDEXT2;                   /*!< (@ 0x00000038) UCPD Rx Ordered Set Extension Register 2                   */
} UCPD1_Type;                                   /*!< Size = 60 (0x3c)                                                          */



/* =========================================================================================================================== */
/* ================                                       USB_FS_device                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief USB_FS_device (USB_FS_device)
  */

typedef struct {                                /*!< (@ 0x40005C00) USB_FS_device Structure                                    */
  __IO uint32_t   EP0R;                         /*!< (@ 0x00000000) USB endpoint n register                                    */
  __IO uint32_t   EP1R;                         /*!< (@ 0x00000004) USB endpoint n register                                    */
  __IO uint32_t   EP2R;                         /*!< (@ 0x00000008) USB endpoint n register                                    */
  __IO uint32_t   EP3R;                         /*!< (@ 0x0000000C) USB endpoint n register                                    */
  __IO uint32_t   EP4R;                         /*!< (@ 0x00000010) USB endpoint n register                                    */
  __IO uint32_t   EP5R;                         /*!< (@ 0x00000014) USB endpoint n register                                    */
  __IO uint32_t   EP6R;                         /*!< (@ 0x00000018) USB endpoint n register                                    */
  __IO uint32_t   EP7R;                         /*!< (@ 0x0000001C) USB endpoint n register                                    */
  __I  uint32_t   RESERVED[8];
  __IO uint32_t   CNTR;                         /*!< (@ 0x00000040) USB control register                                       */
  __IO uint32_t   ISTR;                         /*!< (@ 0x00000044) USB interrupt status register                              */
  __I  uint32_t   FNR;                          /*!< (@ 0x00000048) USB frame number register                                  */
  __IO uint32_t   DADDR;                        /*!< (@ 0x0000004C) USB device address                                         */
  __IO uint32_t   BTABLE;                       /*!< (@ 0x00000050) Buffer table address                                       */
} USB_FS_device_Type;                           /*!< Size = 84 (0x54)                                                          */



/* =========================================================================================================================== */
/* ================                                            CRS                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief CRS (CRS)
  */

typedef struct {                                /*!< (@ 0x40002000) CRS Structure                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) CRS control register                                       */
  __IO uint32_t   CFGR;                         /*!< (@ 0x00000004) This register can be written only when the frequency
                                                                    error counter is disabled (CEN bit is cleared
                                                                    in CRS_CR). When the counter is enabled,
                                                                    this register is write-protected.                          */
  __I  uint32_t   ISR;                          /*!< (@ 0x00000008) CRS interrupt and status register                          */
  __IO uint32_t   ICR;                          /*!< (@ 0x0000000C) CRS interrupt flag clear register                          */
} CRS_Type;                                     /*!< Size = 16 (0x10)                                                          */


/** @} */ /* End of group Device_Peripheral_peripherals */


/* =========================================  End of section using anonymous unions  ========================================= */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

#define CRC_BASE                    0x40023000UL
#define IWDG_BASE                   0x40003000UL
#define WWDG_BASE                   0x40002C00UL
#define I2C1_BASE                   0x40005400UL
#define I2C2_BASE                   0x40005800UL
#define I2C3_BASE                   0x40007800UL
#define FLASH_BASE                  0x40022000UL
#define DBGMCU_BASE                 0xE0042000UL
#define RCC_BASE                    0x40021000UL
#define PWR_BASE                    0x40007000UL
#define RNG_BASE                    0x50060800UL
#define GPIOA_BASE                  0x48000000UL
#define GPIOB_BASE                  0x48000400UL
#define GPIOC_BASE                  0x48000800UL
#define GPIOD_BASE                  0x48000C00UL
#define GPIOE_BASE                  0x48001000UL
#define GPIOF_BASE                  0x48001400UL
#define GPIOG_BASE                  0x48001800UL
#define TIM15_BASE                  0x40014000UL
#define TIM16_BASE                  0x40014400UL
#define TIM17_BASE                  0x40014800UL
#define TIM1_BASE                   0x40012C00UL
#define TIM8_BASE                   0x40013400UL
#define TIM2_BASE                   0x40000000UL
#define TIM3_BASE                   0x40000400UL
#define TIM4_BASE                   0x40000800UL
#define TIM6_BASE                   0x40001000UL
#define TIM7_BASE                   0x40001400UL
#define LPTIMER1_BASE               0x40007C00UL
#define USART1_BASE                 0x40013800UL
#define USART2_BASE                 0x40004400UL
#define USART3_BASE                 0x40004800UL
#define UART4_BASE                  0x40004C00UL
#define UART5_BASE                  0x40005000UL
#define LPUART1_BASE                0x40008000UL
#define SPI1_BASE                   0x40013000UL
#define SPI3_BASE                   0x40003C00UL
#define SPI2_BASE                   0x40003800UL
#define EXTI_BASE                   0x40010400UL
#define RTC_BASE                    0x40002800UL
#define DMA1_BASE                   0x40020000UL
#define DMA2_BASE                   0x40020400UL
#define DMAMUX_BASE                 0x40020800UL
#define SYSCFG_BASE                 0x40010000UL
#define VREFBUF_BASE                0x40010030UL
#define COMP_BASE                   0x40010200UL
#define OPAMP_BASE                  0x40010300UL
#define DAC1_BASE                   0x50000800UL
#define DAC2_BASE                   0x50000C00UL
#define DAC3_BASE                   0x50001000UL
#define DAC4_BASE                   0x50001400UL
#define ADC1_BASE                   0x50000000UL
#define ADC2_BASE                   0x50000100UL
#define ADC3_BASE                   0x50000400UL
#define ADC12_Common_BASE           0x50000300UL
#define ADC345_Common_BASE          0x50000700UL
#define FMAC_BASE                   0x40021400UL
#define CORDIC_BASE                 0x40020C00UL
#define SAI_BASE                    0x40015400UL
#define TAMP_BASE                   0x40002400UL
#define FPU_BASE                    0xE000EF34UL
#define MPU_BASE                    0xE000E084UL
#define STK_BASE                    0xE000E010UL
#define SCB_BASE                    0xE000ED00UL
#define NVIC_BASE                   0xE000E100UL
#define NVIC_STIR_BASE              0xE000EF00UL
#define FPU_CPACR_BASE              0xE000ED88UL
#define SCB_ACTLR_BASE              0xE000E008UL
#define FDCAN_BASE                  0x4000A400UL
#define FDCAN1_BASE                 0x40006400UL
#define UCPD1_BASE                  0x4000A000UL
#define USB_FS_device_BASE          0x40005C00UL
#define CRS_BASE                    0x40002000UL

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define CRC                         ((CRC_Type*)               CRC_BASE)
#define IWDG                        ((IWDG_Type*)              IWDG_BASE)
#define WWDG                        ((WWDG_Type*)              WWDG_BASE)
#define I2C1                        ((I2C1_Type*)              I2C1_BASE)
#define I2C2                        ((I2C1_Type*)              I2C2_BASE)
#define I2C3                        ((I2C1_Type*)              I2C3_BASE)
#define FLASH                       ((FLASH_Type*)             FLASH_BASE)
#define DBGMCU                      ((DBGMCU_Type*)            DBGMCU_BASE)
#define RCC                         ((RCC_Type*)               RCC_BASE)
#define PWR                         ((PWR_Type*)               PWR_BASE)
#define RNG                         ((RNG_Type*)               RNG_BASE)
#define GPIOA                       ((GPIOA_Type*)             GPIOA_BASE)
#define GPIOB                       ((GPIOB_Type*)             GPIOB_BASE)
#define GPIOC                       ((GPIOC_Type*)             GPIOC_BASE)
#define GPIOD                       ((GPIOC_Type*)             GPIOD_BASE)
#define GPIOE                       ((GPIOC_Type*)             GPIOE_BASE)
#define GPIOF                       ((GPIOC_Type*)             GPIOF_BASE)
#define GPIOG                       ((GPIOC_Type*)             GPIOG_BASE)
#define TIM15                       ((TIM15_Type*)             TIM15_BASE)
#define TIM16                       ((TIM16_Type*)             TIM16_BASE)
#define TIM17                       ((TIM16_Type*)             TIM17_BASE)
#define TIM1                        ((TIM1_Type*)              TIM1_BASE)
#define TIM8                        ((TIM1_Type*)              TIM8_BASE)
#define TIM2                        ((TIM2_Type*)              TIM2_BASE)
#define TIM3                        ((TIM2_Type*)              TIM3_BASE)
#define TIM4                        ((TIM2_Type*)              TIM4_BASE)
#define TIM6                        ((TIM6_Type*)              TIM6_BASE)
#define TIM7                        ((TIM6_Type*)              TIM7_BASE)
#define LPTIMER1                    ((LPTIMER1_Type*)          LPTIMER1_BASE)
#define USART1                      ((USART1_Type*)            USART1_BASE)
#define USART2                      ((USART1_Type*)            USART2_BASE)
#define USART3                      ((USART1_Type*)            USART3_BASE)
#define UART4                       ((UART4_Type*)             UART4_BASE)
#define UART5                       ((UART4_Type*)             UART5_BASE)
#define LPUART1                     ((LPUART1_Type*)           LPUART1_BASE)
#define SPI1                        ((SPI1_Type*)              SPI1_BASE)
#define SPI3                        ((SPI1_Type*)              SPI3_BASE)
#define SPI2                        ((SPI1_Type*)              SPI2_BASE)
#define EXTI                        ((EXTI_Type*)              EXTI_BASE)
#define RTC                         ((RTC_Type*)               RTC_BASE)
#define DMA1                        ((DMA1_Type*)              DMA1_BASE)
#define DMA2                        ((DMA1_Type*)              DMA2_BASE)
#define DMAMUX                      ((DMAMUX_Type*)            DMAMUX_BASE)
#define SYSCFG                      ((SYSCFG_Type*)            SYSCFG_BASE)
#define VREFBUF                     ((VREFBUF_Type*)           VREFBUF_BASE)
#define COMP                        ((COMP_Type*)              COMP_BASE)
#define OPAMP                       ((OPAMP_Type*)             OPAMP_BASE)
#define DAC1                        ((DAC1_Type*)              DAC1_BASE)
#define DAC2                        ((DAC1_Type*)              DAC2_BASE)
#define DAC3                        ((DAC1_Type*)              DAC3_BASE)
#define DAC4                        ((DAC1_Type*)              DAC4_BASE)
#define ADC1                        ((ADC1_Type*)              ADC1_BASE)
#define ADC2                        ((ADC1_Type*)              ADC2_BASE)
#define ADC3                        ((ADC3_Type*)              ADC3_BASE)
#define ADC12_Common                ((ADC12_Common_Type*)      ADC12_Common_BASE)
#define ADC345_Common               ((ADC12_Common_Type*)      ADC345_Common_BASE)
#define FMAC                        ((FMAC_Type*)              FMAC_BASE)
#define CORDIC                      ((CORDIC_Type*)            CORDIC_BASE)
#define SAI                         ((SAI_Type*)               SAI_BASE)
#define TAMP                        ((TAMP_Type*)              TAMP_BASE)
#define FPU                         ((FPU_Type*)               FPU_BASE)
#define MPU                         ((MPU_Type*)               MPU_BASE)
#define STK                         ((STK_Type*)               STK_BASE)
#define SCB                         ((SCB_Type*)               SCB_BASE)
#define NVIC                        ((NVIC_Type*)              NVIC_BASE)
#define NVIC_STIR                   ((NVIC_STIR_Type*)         NVIC_STIR_BASE)
#define FPU_CPACR                   ((FPU_CPACR_Type*)         FPU_CPACR_BASE)
#define SCB_ACTLR                   ((SCB_ACTLR_Type*)         SCB_ACTLR_BASE)
#define FDCAN                       ((FDCAN_Type*)             FDCAN_BASE)
#define FDCAN1                      ((FDCAN_Type*)             FDCAN1_BASE)
#define UCPD1                       ((UCPD1_Type*)             UCPD1_BASE)
#define USB_FS_device               ((USB_FS_device_Type*)     USB_FS_device_BASE)
#define CRS                         ((CRS_Type*)               CRS_BASE)

/** @} */ /* End of group Device_Peripheral_declaration */


#ifdef __cplusplus
}
#endif

#endif /* STM32G491XX_H */


/** @} */ /* End of group STM32G491xx */

/** @} */ /* End of group  */
