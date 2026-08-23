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
#include "capacitor_monitor.h"
#include "can_protocol.h"
#include "command_inputs.h"
#include "shared_state.h"
#include "referee_uart.h"
#include "eeprom_emul.h"
#include "scap_io_owner.h"
#include "app_watchdog.h"
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

static inline void gpio_write_masked_bsrr(GPIO_TypeDef *port, uint16_t affect_mask, uint16_t desired)
{
  const uint16_t set_mask = (uint16_t)(desired & affect_mask);
  const uint16_t reset_mask = (uint16_t)((~desired) & affect_mask);
  port->BSRR = ((uint32_t)reset_mask << 16) | (uint32_t)set_mask;
}

static uint8_t g_swen_last_applied = 0xFFu; /* force first apply */
static uint32_t dir_counter = 0;

static uint8_t ScapSafety_FaultBits(float v_bus, float v_cap)
{
  uint8_t faults = 0u;
  if (v_bus >= SCAP_VBUS_OVP_V)
  {
    faults |= CONTROL_FAULT_VBUS_OVP;
  }
  if (v_cap >= SCAP_VCAP_OVP_V)
  {
    faults |= CONTROL_FAULT_VCAP_OVP;
  }
  return faults;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
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
    const uint16_t n_adc_imonop = g_adc2_dma_buf[1] & 0x0FFFU;
    const uint16_t n_adc_imonon = g_adc2_dma_buf[2] & 0x0FFFU;
    const float i_out_p = (A_OP * (float)n_adc_imonop) + B_OP;
    const float i_out_n = (A_ON * (float)n_adc_imonon) + B_ON;
    const float i_out = (i_out_p > -i_out_n) ? i_out_p : i_out_n;
    g_latest.i_out_p = i_out_p;
    g_latest.i_out_n = i_out_n;

    if (LL_GPIO_IsOutputPinSet(GPIOB, GPIO_SWEN_Pin) == 0u) g_latest.i_out = 0.0f;//imonOP/N only valid when ON
    else g_latest.i_out = i_out;



    const float v_bus = (A_VBUS * (float)n_adc_vbus) + B_VBUS;
    const float v_cap = (A_VCAP * (float)n_adc_vcap) + B_VCAP;

    const uint8_t fault_bits = ScapSafety_FaultBits(v_bus, v_cap);
    g_is_safe = (fault_bits == 0u);
    const uint8_t safety_state = ScapIo_FastUpdateSafety(fault_bits, v_bus); //vbus for UVLO
    const bool fault_latched = (safety_state & SCAP_FAST_SAFETY_FAULT_LATCHED) != 0u;
    const bool uvlo_lockout = (safety_state & SCAP_FAST_SAFETY_UVLO_LOCKOUT) != 0u;

    CapacitorMonitor_AccumulateFromIsr(v_cap, g_latest.i_out);

    // ADC2 (see `shared_state.h`): [0]=ILOAD differential (offset-binary)
    const uint16_t n_adc_iload = g_adc2_dma_buf[0] & 0x0FFFU;
    const float i_load = (A_ILOAD * (float)n_adc_iload) + B_ILOAD;

    control_fast_command_t command;
    ScapIo_ReadFastCommand(&command);
    const float p_set = command.p_set_w;
    float denom = (float)n_adc_vbus + N_OFFSET;
    if (denom < 1.0f) { denom = 1.0f; }
    const float inv_v_bus = A_VBUS_INV / denom;
    float i_conv = (p_set * inv_v_bus) - i_load;

    //guard against high cap/low bus voltage done by UVLO
    //only guards against low cap voltage/high bus voltage
    float cap_limit = I_CAP_CLAMP_ABS_A;
    if ((v_bus > 0.0f) && (v_cap > 0.0f)) cap_limit = (v_cap * I_CAP_CLAMP_ABS_A) / v_bus;
    float i_limit = (cap_limit < I_CONV_CLAMP_ABS_A) ? cap_limit : I_CONV_CLAMP_ABS_A;
    if (i_conv > i_limit) i_conv = i_limit;
    else if (i_conv < -i_limit) i_conv = -i_limit;


    //15 cycle constant DIR pin gate
    //dir == high, then if current DIR = high, business as usual
    //dir == high, else current DIR = low, transition required
    // if counter == 10, transition then business as usual, reset counter to 0
    // else block transition, set iconv close to 0, maintain DIR = low
    if (dir_counter < SCAP_DIR_MIN_TRANSITION_ISR_CYCLES) dir_counter++;
    const bool dir_current = (GPIO_DIR_GPIO_Port->ODR & GPIO_DIR_Pin);
    bool dir_new = (i_conv > 0.0f);

    if (dir_new == 1)//new dir is high
    {
      if (dir_current == 0)//current dir is low
      {
        if (dir_counter == SCAP_DIR_MIN_TRANSITION_ISR_CYCLES) dir_counter = 0;
        else
        {
          dir_new = dir_current;
          i_conv = 0.0f;
        }
      }
    }
    else //new dir == low/0
    {
      if (dir_current == 1)
      {
        if (dir_counter == SCAP_DIR_MIN_TRANSITION_ISR_CYCLES) dir_counter = 0;
        else
        {
          dir_new = dir_current;
          i_conv = 0.0f;
        }
      }
    }

    uint16_t n_dac_p = clamp_u12((int32_t)(A_INP + (i_conv * B_INP)));
    uint16_t n_dac_n = clamp_u12((int32_t)(A_INN + (i_conv * B_INN)));



    if ((g_app_watchdog_failed != 0u) || fault_latched)
    {
      gpio_write_masked_bsrr(GPIOB, GPIO_SWEN_Pin, 0u);
      const uint32_t fault_led_phase = (g_adc_seq_count / 5000u) & 1u; /* 5 Hz */
      gpio_write_masked_bsrr(GPIO_LED_GPIO_Port, GPIO_LED_Pin,
                             fault_led_phase != 0u ? GPIO_LED_Pin : 0u);
      g_swen_last_applied = 0u;
    }
    else if (uvlo_lockout)
    {
      if (dir_new)
      {
        gpio_write_masked_bsrr(GPIOB, GPIO_DIR_Pin, GPIO_DIR_Pin);
        n_dac_n = n_dac_p;
      }
      else
      {
        gpio_write_masked_bsrr(GPIOB, GPIO_DIR_Pin, 0u);
        n_dac_p = n_dac_n;
      }
      LL_DAC_ConvertDualData12RightAligned(DAC1, n_dac_n, n_dac_p);
      gpio_write_masked_bsrr(GPIOB, GPIO_SWEN_Pin, 0u);
      const uint32_t idle_led_phase = (g_adc_seq_count / 25000u) & 1u; /* 1 Hz */
      gpio_write_masked_bsrr(GPIO_LED_GPIO_Port, GPIO_LED_Pin,
                             idle_led_phase != 0u ? GPIO_LED_Pin : 0u);
      g_swen_last_applied = 0u;
    }
    else if (command.decision == CONTROL_DECISION_DIRECT_GPIO)
    {
      g_swen_last_applied = 0xFFu;
    }
    else
    {
      uint8_t desired_swen = command.swen_output_request;
      uint16_t led_desired = 0u;
      const bool drives_algo = (command.decision == CONTROL_DECISION_MANUAL_SET_ALGO) ||
                               (command.decision == CONTROL_DECISION_CAN_ALGO) ||
                               (command.decision == CONTROL_DECISION_UART_ALGO) ||
                               (command.decision == CONTROL_DECISION_IDLE) ||
                               (command.decision == CONTROL_DECISION_NO_SOURCE);

      if (drives_algo)
      {
        if (dir_new)
        {
          gpio_write_masked_bsrr(GPIOB, GPIO_DIR_Pin, GPIO_DIR_Pin);
          n_dac_n = n_dac_p;
        }
        else
        {
          gpio_write_masked_bsrr(GPIOB, GPIO_DIR_Pin, 0u);
          n_dac_p = n_dac_n;
        }
        LL_DAC_ConvertDualData12RightAligned(DAC1, n_dac_n, n_dac_p);

        if ((command.decision == CONTROL_DECISION_IDLE) ||
            (command.decision == CONTROL_DECISION_NO_SOURCE))
        {
          const uint32_t idle_led_phase = (g_adc_seq_count / 25000u) & 1u; /* 1 Hz */
          led_desired = idle_led_phase != 0u ? GPIO_LED_Pin : 0u;
        }
        else led_desired = command.swen_output_request != 0u ? GPIO_LED_Pin : 0u;
      }
      if (desired_swen != g_swen_last_applied)
      {
        gpio_write_masked_bsrr(GPIOB, GPIO_SWEN_Pin,
                               desired_swen != 0u ? GPIO_SWEN_Pin : 0u);
        g_swen_last_applied = desired_swen;
      }
      gpio_write_masked_bsrr(GPIO_LED_GPIO_Port, GPIO_LED_Pin, led_desired);
    }

    g_latest.v_bus = v_bus;
    g_latest.v_cap = v_cap;
    g_latest.i_load = i_load;
    g_latest.i_conv = i_conv;
    g_latest.dir = (GPIO_DIR_GPIO_Port->ODR & GPIO_DIR_Pin);
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
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
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
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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

    CanProtocol_NoteBusActivity(HAL_GetTick());

    if ((rxh.IdType != FDCAN_STANDARD_ID) ||
        (rxh.RxFrameType != FDCAN_DATA_FRAME) ||
        (rxh.Identifier != SUPERCAP_NODE_ID))
    {
      continue;
    }

    if (rxh.DataLength != FDCAN_DLC_BYTES_5)
    {
      continue;
    }

    (void)CanProtocol_TryAcceptCommand(d, sizeof(incoming_msg_packet), HAL_GetTick());
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
  {
    /* FDCAN requires software to leave INIT after bus-off. */
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
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

  ScapIo_HandlePushbuttonFromIsr(now_ms);
}

/* USER CODE END 1 */
