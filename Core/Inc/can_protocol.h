#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "command_inputs.h"

#define SUPERCAP_NODE_ID 0x067u
#define DEVC_NODE_ID 0x077u

#define CAN_POWER_MIN_W COMMAND_POWER_MIN_W
#define CAN_POWER_MAX_W COMMAND_POWER_MAX_W
#define CAN_ENERGY_MIN_J COMMAND_ENERGY_MIN_J
#define CAN_ENERGY_MAX_J COMMAND_ENERGY_MAX_J
#define CAN_ENERGY_DISABLED_MAGIC_J 777u

/* Diagnostic-only CAN bus activity monitor. */
#define CAN_BUS_ACTIVITY_POLL_MS 100u
#define CAN_BUS_ACTIVITY_TIMEOUT_MS 200u

/*
 * Telemetry sent from the supercap board on DEVC_NODE_ID (little-endian).
 * See CAN_TELEMETRY.md in the repository root for a downstream decoder:
 *   bytes 0..1: load power in 0.1 W, unsigned
 *   bytes 2..3: capacitor voltage in 0.1 V, unsigned
 *   bytes 4..5: converter current in 0.1 A, signed two's complement
 *   byte 6:     reserved, zero
 *   byte 7:     CAN_TELEMETRY_STATUS_* bitmask
 */
enum
{
  CAN_TELEMETRY_STATUS_VBUS_OVP = 1u << 0,
  CAN_TELEMETRY_STATUS_VCAP_OVP = 1u << 1,
};

/* Command sent from DEVC to the supercap board. energy_buffer is little-endian. */
typedef struct __attribute__((packed))
{
  uint8_t enable_module;
  uint8_t reset;
  uint8_t pow_limit;
  uint16_t energy_buffer;
} incoming_msg_packet;

_Static_assert(sizeof(incoming_msg_packet) == 5u, "incoming_msg_packet must be exactly 5 bytes");

typedef struct
{
  uint32_t can_cmd_timestamp;
  uint8_t can_power;
  uint16_t can_energy;
  bool can_energy_disabled;
  bool can_swen;
} can_command_state_t;

/* Timestamp zero means that no valid command has been accepted. */
extern volatile can_command_state_t g_can_command;

/*
 * Validate and publish one wire command. Invalid packets leave the mailbox
 * unchanged. timestamp_ms must be nonzero to preserve the empty-mailbox sentinel.
 */
bool CanProtocol_TryAcceptCommand(const uint8_t *data, size_t data_len, uint32_t timestamp_ms);

/* Record activity from an accepted RX FIFO frame. Safe from ISR or task context. */
void CanProtocol_NoteBusActivity(uint32_t timestamp_ms);

/* Poll and discard the bounded FIFO1 activity sample. Call from task context. */
void CanProtocol_PollBusActivity(void);

/* True when accepted CAN traffic has been observed within the activity timeout. */
bool CanProtocol_IsBusActive(uint32_t now_ms);

/* Take a coherent task-context snapshot of the ISR-written mailbox. */
void CanProtocol_ReadCommandSnapshot(can_command_state_t *snapshot);

#endif /* CAN_PROTOCOL_H */
