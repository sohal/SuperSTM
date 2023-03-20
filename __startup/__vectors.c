#include <stdint.h>
#include "cmsis_compiler.h"

/******************************************************************************
 * @file     startup_<Device>.c
 * @brief    CMSIS-Core(M) Device Startup File for
 *           Device <Device>
 * @version  V1.0.0
 * @date     20. January 2021
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stm32h743xx.h"

/*---------------------------------------------------------------------------
  External References
 *---------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/*---------------------------------------------------------------------------
  Internal References
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler  (void);
__NO_RETURN void Default_Handler(void);

/* ToDo: Add Cortex exception handler according the used Cortex-Core */
/*---------------------------------------------------------------------------
  Exception / Interrupt Handler
 *---------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler                        (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler                  (void) __attribute__ ((weak));
void MemManage_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler                        (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler                    (void) __attribute__ ((weak, alias("Default_Handler")));





/* Add your device specific interrupt handler */
/*---------------------------------------------------------------------------
  ISR
 *---------------------------------------------------------------------------*/
void WWDG_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_AVD_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT0_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT0_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT1_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT1_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_043                            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void FMC_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void SDMMC1_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN_CAL_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_065                            (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_066                            (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_067                            (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_068                            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void USART6_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void DCMI_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_070                            (void) __attribute__ ((weak, alias("Default_Handler")));
void RNG_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART8_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI5_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI6_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI1_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void LTDC_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void LTDC_ER_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2D_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI2_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void CEC_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_EV_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPDIF_RX_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_EP1_OUT_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_EP1_IN_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void DMAMUX1_OVR_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_Master_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMA_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMB_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMC_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMD_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIME_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_FLT_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT0_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT2_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT3_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI3_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void SWPMI1_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM15_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM16_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM17_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void MDIOS_WKUP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void MDIOS_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void JPEG_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void MDMA_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_124                            (void) __attribute__ ((weak, alias("Default_Handler")));
void SDMMC2_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void HSEM1_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_127                            (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void DMAMUX2_OVR_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel0_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel1_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel2_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel3_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel4_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel5_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel6_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel7_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP1_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM2_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM3_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM4_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM5_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void LPUART1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_204                            (void) __attribute__ ((weak, alias("Default_Handler")));
void CRS_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void ECC_IRQHandler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI4_IRQHandler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_208                            (void) __attribute__ ((weak, alias("Default_Handler")));
void ISR_209                            (void) __attribute__ ((weak, alias("Default_Handler")));
void WAKEUP_PIN_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

/* ToDo: Add Cortex exception vectors according the used Cortex-Core */
extern const uint32_t VC[];
       const uint32_t VC[] __VECTOR_TABLE_ATTRIBUTE = {
  (uint32_t)(&__INITIAL_SP),  /*     Initial Stack Pointer */
  (uint32_t)&Reset_Handler,                       /*     Reset Handler */
  (uint32_t)&NMI_Handler,                         /* -14 NMI Handler */
  (uint32_t)&HardFault_Handler,                   /* -13 Hard Fault Handler */
  (uint32_t)&MemManage_Handler,                   /* -12 MPU Fault Handler */
  (uint32_t)&BusFault_Handler,                    /* -11 Bus Fault Handler */
  (uint32_t)&UsageFault_Handler,                  /* -10 Usage Fault Handler */
  (uint32_t)&SecureFault_Handler,                 /*  -9 Secure Fault Handler */
  (uint32_t)0,                                   /*     Reserved */
  (uint32_t)0,                                   /*     Reserved */
  (uint32_t)0,                                   /*     Reserved */
  (uint32_t)&SVC_Handler,                         /*  -5 SVCall Handler */
  (uint32_t)&DebugMon_Handler,                    /*  -4 Debug Monitor Handler */
  (uint32_t)0,                                   /*     Reserved */
  (uint32_t)&PendSV_Handler,                      /*  -2 PendSV Handler */
  (uint32_t)&SysTick_Handler,                     /*  -1 SysTick Handler */

/* ToDo: Add your device specific interrupt vectors */
 (uint32_t)&WWDG_IRQHandler                    ,
 (uint32_t)&PVD_AVD_IRQHandler                 ,
 (uint32_t)&TAMP_STAMP_IRQHandler              ,
 (uint32_t)&RTC_WKUP_IRQHandler                ,
 (uint32_t)&FLASH_IRQHandler                   ,
 (uint32_t)&RCC_IRQHandler                     ,
 (uint32_t)&EXTI0_IRQHandler                   ,
 (uint32_t)&EXTI1_IRQHandler                   ,
 (uint32_t)&EXTI2_IRQHandler                   ,
 (uint32_t)&EXTI3_IRQHandler                   ,
 (uint32_t)&EXTI4_IRQHandler                   ,
 (uint32_t)&DMA1_Stream0_IRQHandler            ,
 (uint32_t)&DMA1_Stream1_IRQHandler            ,
 (uint32_t)&DMA1_Stream2_IRQHandler            ,
 (uint32_t)&DMA1_Stream3_IRQHandler            ,
 (uint32_t)&DMA1_Stream4_IRQHandler            ,
 (uint32_t)&DMA1_Stream5_IRQHandler            ,
 (uint32_t)&DMA1_Stream6_IRQHandler            ,
 (uint32_t)&ADC_IRQHandler                     ,
 (uint32_t)&FDCAN1_IT0_IRQHandler              ,
 (uint32_t)&FDCAN2_IT0_IRQHandler              ,
 (uint32_t)&FDCAN1_IT1_IRQHandler              ,
 (uint32_t)&FDCAN2_IT1_IRQHandler              ,
 (uint32_t)&EXTI9_5_IRQHandler                 ,
 (uint32_t)&TIM1_BRK_IRQHandler                ,
 (uint32_t)&TIM1_UP_IRQHandler                 ,
 (uint32_t)&TIM1_TRG_COM_IRQHandler            ,
 (uint32_t)&TIM1_CC_IRQHandler                 ,
 (uint32_t)&TIM2_IRQHandler                    ,
 (uint32_t)&TIM3_IRQHandler                    ,
 (uint32_t)&TIM4_IRQHandler                    ,
 (uint32_t)&I2C1_EV_IRQHandler                 ,
 (uint32_t)&I2C1_ER_IRQHandler                 ,
 (uint32_t)&I2C2_EV_IRQHandler                 ,
 (uint32_t)&I2C2_ER_IRQHandler                 ,
 (uint32_t)&SPI1_IRQHandler                    ,
 (uint32_t)&SPI2_IRQHandler                    ,
 (uint32_t)&USART1_IRQHandler                  ,
 (uint32_t)&USART2_IRQHandler                  ,
 (uint32_t)&USART3_IRQHandler                  ,
 (uint32_t)&EXTI15_10_IRQHandler               ,
 (uint32_t)&RTC_Alarm_IRQHandler               ,
 (uint32_t)&ISR_043                            ,
 (uint32_t)&TIM8_BRK_TIM12_IRQHandler          ,
 (uint32_t)&TIM8_UP_TIM13_IRQHandler           ,
 (uint32_t)&TIM8_TRG_COM_TIM14_IRQHandler      ,
 (uint32_t)&TIM8_CC_IRQHandler                 ,
 (uint32_t)&DMA1_Stream7_IRQHandler            ,
 (uint32_t)&FMC_IRQHandler                     ,
 (uint32_t)&SDMMC1_IRQHandler                  ,
 (uint32_t)&TIM5_IRQHandler                    ,
 (uint32_t)&SPI3_IRQHandler                    ,
 (uint32_t)&UART4_IRQHandler                   ,
 (uint32_t)&UART5_IRQHandler                   ,
 (uint32_t)&TIM6_DAC_IRQHandler                ,
 (uint32_t)&TIM7_IRQHandler                    ,
 (uint32_t)&DMA2_Stream0_IRQHandler            ,
 (uint32_t)&DMA2_Stream1_IRQHandler            ,
 (uint32_t)&DMA2_Stream2_IRQHandler            ,
 (uint32_t)&DMA2_Stream3_IRQHandler            ,
 (uint32_t)&DMA2_Stream4_IRQHandler            ,
 (uint32_t)&ETH_IRQHandler                     ,
 (uint32_t)&ETH_WKUP_IRQHandler                ,
 (uint32_t)&FDCAN_CAL_IRQHandler               ,
 (uint32_t)&ISR_065                            ,
 (uint32_t)&ISR_066                            ,
 (uint32_t)&ISR_067                            ,
 (uint32_t)&ISR_068                            ,
 (uint32_t)&DMA2_Stream5_IRQHandler            ,
 (uint32_t)&DMA2_Stream6_IRQHandler            ,
 (uint32_t)&DMA2_Stream7_IRQHandler            ,
 (uint32_t)&USART6_IRQHandler                  ,
 (uint32_t)&I2C3_EV_IRQHandler                 ,
 (uint32_t)&I2C3_ER_IRQHandler                 ,
 (uint32_t)&OTG_HS_EP1_OUT_IRQHandler          ,
 (uint32_t)&OTG_HS_EP1_IN_IRQHandler           ,
 (uint32_t)&OTG_HS_WKUP_IRQHandler             ,
 (uint32_t)&OTG_HS_IRQHandler                  ,
 (uint32_t)&DCMI_IRQHandler                    ,
 (uint32_t)&ISR_070                            ,
 (uint32_t)&RNG_IRQHandler                     ,
 (uint32_t)&FPU_IRQHandler                     ,
 (uint32_t)&UART7_IRQHandler                   ,
 (uint32_t)&UART8_IRQHandler                   ,
 (uint32_t)&SPI4_IRQHandler                    ,
 (uint32_t)&SPI5_IRQHandler                    ,
 (uint32_t)&SPI6_IRQHandler                    ,
 (uint32_t)&SAI1_IRQHandler                    ,
 (uint32_t)&LTDC_IRQHandler                    ,
 (uint32_t)&LTDC_ER_IRQHandler                 ,
 (uint32_t)&DMA2D_IRQHandler                   ,
 (uint32_t)&SAI2_IRQHandler                    ,
 (uint32_t)&QUADSPI_IRQHandler                 ,
 (uint32_t)&LPTIM1_IRQHandler                  ,
 (uint32_t)&CEC_IRQHandler                     ,
 (uint32_t)&I2C4_EV_IRQHandler                 ,
 (uint32_t)&I2C4_ER_IRQHandler                 ,
 (uint32_t)&SPDIF_RX_IRQHandler                ,
 (uint32_t)&OTG_FS_EP1_OUT_IRQHandler          ,
 (uint32_t)&OTG_FS_EP1_IN_IRQHandler           ,
 (uint32_t)&OTG_FS_WKUP_IRQHandler             ,
 (uint32_t)&OTG_FS_IRQHandler                  ,
 (uint32_t)&DMAMUX1_OVR_IRQHandler             ,
 (uint32_t)&HRTIM1_Master_IRQHandler           ,
 (uint32_t)&HRTIM1_TIMA_IRQHandler             ,
 (uint32_t)&HRTIM1_TIMB_IRQHandler             ,
 (uint32_t)&HRTIM1_TIMC_IRQHandler             ,
 (uint32_t)&HRTIM1_TIMD_IRQHandler             ,
 (uint32_t)&HRTIM1_TIME_IRQHandler             ,
 (uint32_t)&HRTIM1_FLT_IRQHandler              ,
 (uint32_t)&DFSDM1_FLT0_IRQHandler             ,
 (uint32_t)&DFSDM1_FLT1_IRQHandler             ,
 (uint32_t)&DFSDM1_FLT2_IRQHandler             ,
 (uint32_t)&DFSDM1_FLT3_IRQHandler             ,
 (uint32_t)&SAI3_IRQHandler                    ,
 (uint32_t)&SWPMI1_IRQHandler                  ,
 (uint32_t)&TIM15_IRQHandler                   ,
 (uint32_t)&TIM16_IRQHandler                   ,
 (uint32_t)&TIM17_IRQHandler                   ,
 (uint32_t)&MDIOS_WKUP_IRQHandler              ,
 (uint32_t)&MDIOS_IRQHandler                   ,
 (uint32_t)&JPEG_IRQHandler                    ,
 (uint32_t)&MDMA_IRQHandler                    ,
 (uint32_t)&ISR_124                            ,
 (uint32_t)&SDMMC2_IRQHandler                  ,
 (uint32_t)&HSEM1_IRQHandler                   ,
 (uint32_t)&ISR_127                            ,
 (uint32_t)&ADC3_IRQHandler                    ,
 (uint32_t)&DMAMUX2_OVR_IRQHandler             ,
 (uint32_t)&BDMA_Channel0_IRQHandler           ,
 (uint32_t)&BDMA_Channel1_IRQHandler           ,
 (uint32_t)&BDMA_Channel2_IRQHandler           ,
 (uint32_t)&BDMA_Channel3_IRQHandler           ,
 (uint32_t)&BDMA_Channel4_IRQHandler           ,
 (uint32_t)&BDMA_Channel5_IRQHandler           ,
 (uint32_t)&BDMA_Channel6_IRQHandler           ,
 (uint32_t)&BDMA_Channel7_IRQHandler           ,
 (uint32_t)&COMP1_IRQHandler                   ,
 (uint32_t)&LPTIM2_IRQHandler                  ,
 (uint32_t)&LPTIM3_IRQHandler                  ,
 (uint32_t)&LPTIM4_IRQHandler                  ,
 (uint32_t)&LPTIM5_IRQHandler                  ,
 (uint32_t)&LPUART1_IRQHandler                 ,
 (uint32_t)&ISR_204                            ,
 (uint32_t)&CRS_IRQHandler                     ,
 (uint32_t)&ECC_IRQHandler                     ,
 (uint32_t)&SAI4_IRQHandler                    ,
 (uint32_t)&ISR_208                            ,
 (uint32_t)&ISR_209                            ,
 (uint32_t)&WAKEUP_PIN_IRQHandler
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/*---------------------------------------------------------------------------
  Reset Handler called on controller reset
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
  __set_PSP((uint32_t)(&__INITIAL_SP));

/* ToDo: Initialize stack limit register for Armv8-M Main Extension based processors*/
  __set_MSP((uint32_t)(&__STACK_LIMIT));
  __set_PSP((uint32_t)(&__STACK_LIMIT));

/* ToDo: Add stack sealing for Armv8-M based processors */
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  __TZ_set_STACKSEAL_S((uint32_t *)(&__STACK_SEAL));
#endif

  SystemInit();                    /* CMSIS System Initialization */
  __PROGRAM_START();               /* Enter PreMain (C library entry point) */
}


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
  while(1);
}

/*---------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *---------------------------------------------------------------------------*/
void Default_Handler(void)
{
  while(1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#endif
