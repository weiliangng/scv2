---
title: CAN telemetry
description: Decode SCV2's 8-byte Classic CAN fast telemetry and fault-status frame.
sidebar:
  order: 2
---

SCV2 transmits one Classic CAN frame containing fast electrical telemetry and fault status.

## Frame contract

| Property | Value |
|---|---|
| Format | Classic CAN data frame; not CAN FD or a remote frame |
| Identifier | Standard 11-bit ID `0x077` |
| DLC | 8 bytes |
| Bus rate | 1 Mbit/s |
| Nominal update rate | 1 kHz, task-scheduled and best effort |
| Multi-byte order | Little-endian; least-significant byte first |

A receiver must tolerate jitter and missing frames rather than assuming an exact one-millisecond arrival interval.

## Payload

| Bytes | Field | Raw type | Scale | Conversion |
|---|---|---|---|---|
| 0–1 | Load power | `uint16_t` | 0.1 W/count | `p_load_W = raw / 10.0` |
| 2–3 | Capacitor voltage | `uint16_t` | 0.1 V/count | `v_cap_V = raw / 10.0` |
| 4–5 | Converter-current command | Signed 16-bit two's complement | 0.1 A/count | `i_conv_A = signed_raw / 10.0` |
| 6 | Reserved | `uint8_t` | — | Ignore; currently zero |
| 7 | Status | `uint8_t` bitmask | — | Decode using the masks below |

Unsigned fields saturate to the range 0–6553.5 W/V. Converter current uses the signed 16-bit wire range, although the current control source clamps its command to ±10 A.

## Status byte

| Bit | Mask | Meaning when set |
|---:|---:|---|
| 0 | `0x01` | Vbus overvoltage fault latched |
| 1 | `0x02` | Vcap overvoltage fault latched |
| 2–7 | — | Reserved; ignore unknown bits |

Test individual masks instead of comparing the entire status byte with a fixed value. This keeps receivers compatible with future status bits.

## Portable C decoder

```c
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
  SCV2_TELEMETRY_CAN_ID = 0x077u,
  SCV2_STATUS_VBUS_OVP = 1u << 0,
  SCV2_STATUS_VCAP_OVP = 1u << 1,
};

typedef struct {
  uint16_t p_load_dW;
  uint16_t v_cap_dV;
  int32_t i_conv_dA;
  uint8_t status;
  bool vbus_ovp;
  bool vcap_ovp;
} scv2_telemetry_t;

static uint16_t read_u16_le(const uint8_t *bytes)
{
  return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static int32_t read_i16_le(const uint8_t *bytes)
{
  const uint16_t raw = read_u16_le(bytes);
  return ((raw & 0x8000u) != 0u) ? ((int32_t)raw - 65536L) : (int32_t)raw;
}

bool Scv2Telemetry_Decode(uint32_t identifier,
                          bool is_extended_id,
                          bool is_remote_frame,
                          uint8_t dlc_bytes,
                          const uint8_t *data,
                          scv2_telemetry_t *telemetry)
{
  if ((identifier != SCV2_TELEMETRY_CAN_ID) || is_extended_id ||
      is_remote_frame || (dlc_bytes != 8u) ||
      (data == NULL) || (telemetry == NULL)) {
    return false;
  }

  telemetry->p_load_dW = read_u16_le(&data[0]);
  telemetry->v_cap_dV = read_u16_le(&data[2]);
  telemetry->i_conv_dA = read_i16_le(&data[4]);
  telemetry->status = data[7];
  telemetry->vbus_ovp = (data[7] & SCV2_STATUS_VBUS_OVP) != 0u;
  telemetry->vcap_ovp = (data[7] & SCV2_STATUS_VCAP_OVP) != 0u;
  return true;
}
```

Do not cast the payload directly to a packed C struct. Explicit decoding avoids alignment dependencies and makes wire endianness unambiguous.

## Worked frame

```text
88 13 2C 01 9C FF 00 03
```

- Load power: `0x1388 = 5000` → `500.0 W`
- Capacitor voltage: `0x012C = 300` → `30.0 V`
- Converter current: `0xFF9C = -100` → `-10.0 A`
- Reserved byte: `0x00`
- Status: `0x03` → both overvoltage flags are set

## Receiver checklist

1. Accept standard identifier `0x077` with DLC 8 on a 1 Mbit/s Classic CAN bus.
2. Reject extended, remote, and wrong-DLC frames.
3. Copy or decode the complete frame and record its arrival timestamp.
4. Publish it from the receive callback using the robot firmware's normal synchronization mechanism.
5. Keep fixed-point values until display or control boundaries when practical.
6. Ignore byte 6 and unknown status bits.
7. Detect stale telemetry using a timeout selected for the robot's safety requirements.

The canonical firmware-side receiver guide remains available in [`CAN_TELEMETRY.md`](https://github.com/weiliangng/scv2/blob/main/CAN_TELEMETRY.md).
