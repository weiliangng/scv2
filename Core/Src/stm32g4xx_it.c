/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"
#include "app_constants.h"
#include "shared_state.h"
#include "scap_io_owner.h"
#include "referee_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint16_t clamp_u12(int32_t v)
{
  if (v < 0)
  {
    return 0;
  }
  if (v > 4095)
  {
    return 4095;
  }
  return (uint16_t)v;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  const uint32_t irq_start_cycles = DWT->CYCCNT;
  if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
  {
    LL_DMA_ClearFlag_GI1(DMA1);

    //LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_4);

    // ADC1 (see `shared_state.h`): [0]=Vcap, [1]=Vbus
    const uint16_t n_adc_vbus = g_adc1_dma_buf[1] & 0x0FFFU;

    // ADC2 (see `shared_state.h`): [0]=ILOAD differential (offset-binary)
    const uint16_t n_adc_iload = g_adc2_dma_buf[0] & 0x0FFFU;

    const float v_bus = (A_VBUS * (float)n_adc_vbus) + B_VBUS;
    const float i_load = (A_ILOAD * (float)n_adc_iload) + B_ILOAD;

    const float p_set = g_latest.p_set;
    float denom = (float)n_adc_vbus + N_OFFSET;
    if (denom < 1.0f) { denom = 1.0f; }
    const float inv_v_bus = A_VBUS_INV / denom;
    const float i_conv = (p_set * inv_v_bus) - i_load;



    const uint16_t n_dac_p = clamp_u12((int32_t)(A_INP + (i_conv * B_INP)));
    const uint16_t n_dac_n = clamp_u12((int32_t)(A_INN + (i_conv * B_INN)));

    g_latest.v_bus = v_bus;
    g_latest.i_load = i_load;
    g_latest.i_conv = i_conv;

    const ctrl_src_t ctrl_src = g_ctrl_src;
    if ((ctrl_src == SRC_ALGO) || (ctrl_src == SRC_CAN))
    {
      // Optional: mirror ILOAD ADC counts on DAC3_CH1 for scope/debug.
      LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, n_adc_iload);
      LL_DAC_ConvertDualData12RightAligned(DAC1, n_dac_p, n_dac_n);
    }

    // Only let ISR drive DIR when in ALGO ownership.
    if (ctrl_src == SRC_ALGO)
    {
      // Fast: one write to BSRR (set or reset PB1) based on i_conv sign.
      // (BS1 sets PB1; BR1 resets PB1).
      GPIOB->BSRR = (i_conv > 0.0f) ? GPIO_BSRR_BS1 : GPIO_BSRR_BR1;
    }
  }
  else
  {
    LL_DMA_ClearFlag_GI1(DMA1);
  }

  const uint32_t irq_cycles = (uint32_t)(DWT->CYCCNT - irq_start_cycles);
  g_dma1_ch1_irq_cycles_last = irq_cycles;
  if (irq_cycles > g_dma1_ch1_irq_cycles_max)
  {
    g_dma1_ch1_irq_cycles_max = irq_cycles;
  }
  return;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  LL_DMA_ClearFlag_GI2(DMA1);
  return;
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  RefereeUart_UsartIdleIsr(&huart3);

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  FDCAN_RxHeaderTypeDef rxh;
  uint8_t d[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, d) != HAL_OK)
    {
      break;
    }

    if (rxh.IdType != FDCAN_STANDARD_ID)
    {
      continue;
    }

    if (rxh.DataLength < FDCAN_DLC_BYTES_4)
    {
      continue;
    }

    g_can_rx.last_can_tick = HAL_GetTick();
    g_can_rx.can_rx_count++;

    if (rxh.Identifier == METER_ID)
    {
      const uint16_t raw_v = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
      const int16_t raw_i = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));

      meter_v = (float)raw_v / 100.0f;
      meter_i = (float)raw_i / 100.0f;
      continue;
    }

    if (rxh.Identifier == SCAP_CMD_ID)
    {
      const uint8_t settings = d[0];

      g_can_rx.settings_raw = settings;
      g_can_rx.en = (settings & (1u << 0)) != 0u;
      g_can_rx.mode = (settings & (1u << 1)) != 0u;
      g_can_rx.dir = (settings & (1u << 2)) != 0u;
      g_can_rx.override_power = (settings & (1u << 3)) != 0u;
      g_can_rx.siphon_buffer = (settings & (1u << 4)) != 0u;

      g_can_rx.last_cmd_tick = g_can_rx.last_can_tick;
      g_can_rx.can_power = (uint16_t)d[1] | ((uint16_t)d[2] << 8);//can target power
      g_can_rx.can_buf = d[3];//target buffer

      g_latest.p_set = (float)g_can_rx.can_power;//to be changed later
      ScapIo_CanRxUpdateIsr(g_can_rx.en, g_can_rx.dir, g_can_rx.mode);
      continue;
    }
  }
}

/* USER CODE END 1 */
