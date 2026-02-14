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
#include "eeprom_emul.h"
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

static inline bool ScapSafety_IsSafe(float v_bus, float v_cap)
{
  return (v_bus > 10.0f) && (v_bus < 30.0f) && (v_cap < 30.0f);
}

static inline void gpio_write_masked_bsrr(GPIO_TypeDef *port, uint16_t affect_mask, uint16_t desired)
{
  const uint16_t set_mask = (uint16_t)(desired & affect_mask);
  const uint16_t reset_mask = (uint16_t)((~desired) & affect_mask);
  port->BSRR = ((uint32_t)reset_mask << 16) | (uint32_t)set_mask;
}

static uint8_t g_swen_last_applied = 0xFFu; /* force first apply */

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
/* During the cleanup phase in EE_Init, AddressRead is the address being read */
extern __IO uint32_t AddressRead;
/* Flag equal to 1 when the cleanup phase is in progress, 0 if not */
extern __IO uint8_t CleanupPhase;

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
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
  {
    if (CleanupPhase == 1U)
    {
      if ((AddressRead >= START_PAGE_ADDRESS) && (AddressRead <= END_EEPROM_ADDRESS))
      {
        if (EE_DeleteCorruptedFlashAddress((uint32_t)AddressRead) == EE_OK)
        {
          return;
        }

        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR) && __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) &&
            __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR))
        {
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PROGERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
          return;
        }
      }
    }
    else
    {
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
      return;
    }
  }

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
    g_adc_seq_count++;

    //LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_4);

    // ADC1 (see `shared_state.h`): [0]=Vcap, [1]=Vbus
    const uint16_t n_adc_vbus = g_adc1_dma_buf[1] & 0x0FFFU;
    const uint16_t n_adc_vcap = g_adc1_dma_buf[0] & 0x0FFFU;

    // ADC2 (see `shared_state.h`): [0]=ILOAD differential (offset-binary)
    const uint16_t n_adc_iload = g_adc2_dma_buf[0] & 0x0FFFU;

    const float v_bus = (A_VBUS * (float)n_adc_vbus) + B_VBUS;
    const float v_cap = (A_VCAP * (float)n_adc_vcap) + B_VCAP;
    const float i_load = (A_ILOAD * (float)n_adc_iload) + B_ILOAD;

    float curr_buf;
    if (g_uart_connected) curr_buf = g_uart_rx.buf_e_j;
    else if (g_can_cmd_connected) curr_buf = (float)g_can_rx.can_buf;
    else curr_buf = -1.0f;
    g_curr_buf_e_j = curr_buf;

    const float p_set = g_latest.p_set;
    float denom = (float)n_adc_vbus + N_OFFSET;
    if (denom < 1.0f) { denom = 1.0f; }
    const float inv_v_bus = A_VBUS_INV / denom;
    float i_conv = (p_set * inv_v_bus) - i_load;
    if (i_conv > I_CONV_CLAMP_ABS_A)
    {
      i_conv = I_CONV_CLAMP_ABS_A;
    }
    else if (i_conv < -I_CONV_CLAMP_ABS_A)
    {
      i_conv = -I_CONV_CLAMP_ABS_A;
    }

    uint16_t n_dac_p = clamp_u12((int32_t)(A_INP + (i_conv * B_INP)));
    uint16_t n_dac_n = clamp_u12((int32_t)(A_INN + (i_conv * B_INN)));

    g_latest.v_bus = v_bus;
    g_latest.v_cap = v_cap;
    g_latest.i_load = i_load;
    g_latest.i_conv = i_conv;

    const ctrl_src_t ctrl_src = g_ctrl_src;
    if (ctrl_src == SRC_ALGO)
    {
      if (i_conv > 0.0f) {
        GPIOB->BSRR = GPIO_BSRR_BS1;
        n_dac_n = n_dac_p;
      } else {
        GPIOB->BSRR = GPIO_BSRR_BR1;
        n_dac_p = n_dac_n;
      }
      LL_DAC_ConvertDualData12RightAligned(DAC1, n_dac_n, n_dac_p);
    }

    uint8_t swen_auto = g_swen_auto_req;
    
    uint8_t desired = 0u;

    if ((ScapSafety_IsSafe(v_bus, v_cap)) && (g_swen_force_low_slow == 0u))
    {
      const ctrl_src_t src = g_ctrl_src;
      desired = (src == SRC_MANUAL) ? (((g_pb_manual & GPIO_SWEN_Pin) != 0u) ? 1u : 0u)
                                    : ((swen_auto != 0u) ? 1u : 0u);
    }

    if (desired != g_swen_last_applied)
    {
      gpio_write_masked_bsrr(GPIOB, GPIO_SWEN_Pin, desired ? GPIO_SWEN_Pin : 0u);
      gpio_write_masked_bsrr(GPIO_LED_GPIO_Port, GPIO_LED_Pin, desired ? GPIO_LED_Pin : 0u);
      g_swen_last_applied = desired;
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

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
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
      g_can_rx.en = (settings & (1u << 0)) != 0u;//bit0: enable or disable SWEN (other bits reserved)
      ScapIo_AutoSetSwenFromCanIsr(g_can_rx.en);

      g_can_rx.last_cmd_tick = g_can_rx.last_can_tick;
      g_can_rx.can_power = (uint16_t)d[1] | ((uint16_t)d[2] << 8);//alternate source of power limit if UART breaks down
      g_can_rx.can_buf = d[3];//alternate source of current buffer energy if UART breaks down

      continue;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13)
  {
    return;
  }

  /*
   * Debounce EXTI13 pushbutton: ignore additional edges within a short window.
   * PC13 is configured as GPIO_MODE_IT_RISING with pulldown in MX_GPIO_Init().
   */
  enum
  {
    EXTI13_DEBOUNCE_MS = 500u,
  };
  static uint32_t s_last_btn_tick_ms;
  static uint8_t s_has_last_btn_tick;
  const uint32_t now_ms = HAL_GetTick();
  if ((s_has_last_btn_tick != 0u) && ((uint32_t)(now_ms - s_last_btn_tick_ms) < (uint32_t)EXTI13_DEBOUNCE_MS))
  {
    return;
  }
  s_has_last_btn_tick = 1u;
  s_last_btn_tick_ms = now_ms;

  ScapIo_ButtonToggleSwenIsr();
}

/* USER CODE END 1 */
