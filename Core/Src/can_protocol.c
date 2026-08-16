#include "can_protocol.h"

#include "main.h"

volatile can_command_state_t g_can_command;
static volatile uint32_t s_can_bus_activity_timestamp;
static volatile bool s_can_bus_activity_seen;

bool CanProtocol_TryAcceptCommand(const uint8_t *data, size_t data_len, uint32_t timestamp_ms)
{
  if ((data == NULL) || (data_len != sizeof(incoming_msg_packet)) || (timestamp_ms == 0u))
  {
    return false;
  }

  const uint8_t enable_module = data[0];
  const uint8_t power_w = data[2];
  const uint16_t energy_j = (uint16_t)data[3] | ((uint16_t)data[4] << 8);

  if ((enable_module > 1u) ||
      (power_w < CAN_POWER_MIN_W) || (power_w > CAN_POWER_MAX_W) ||
      (energy_j > CAN_ENERGY_MAX_J))
  {
    return false;
  }

  g_can_command.can_power = power_w;
  g_can_command.can_energy = energy_j;
  g_can_command.can_swen = (enable_module != 0u);
  g_can_command.can_cmd_timestamp = timestamp_ms;
  return true;
}

void CanProtocol_NoteBusActivity(uint32_t timestamp_ms)
{
  s_can_bus_activity_timestamp = timestamp_ms;
  s_can_bus_activity_seen = true;
}

void CanProtocol_PollBusActivity(void)
{
  const uint32_t fill_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1);
  if (fill_level == 0u)
  {
    return;
  }

  CanProtocol_NoteBusActivity(HAL_GetTick());

  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];
  for (uint32_t i = 0u; i < fill_level; i++)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &rx_header, data) != HAL_OK)
    {
      break;
    }
  }
}

bool CanProtocol_IsBusActive(uint32_t now_ms)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint32_t activity_timestamp = s_can_bus_activity_timestamp;
  const bool activity_seen = s_can_bus_activity_seen;
  if (primask == 0u)
  {
    __enable_irq();
  }

  return activity_seen &&
         ((uint32_t)(now_ms - activity_timestamp) <= CAN_BUS_ACTIVITY_TIMEOUT_MS);
}

void CanProtocol_ReadCommandSnapshot(can_command_state_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot->can_cmd_timestamp = g_can_command.can_cmd_timestamp;
  snapshot->can_power = g_can_command.can_power;
  snapshot->can_energy = g_can_command.can_energy;
  snapshot->can_swen = g_can_command.can_swen;
  if (primask == 0u)
  {
    __enable_irq();
  }
}
