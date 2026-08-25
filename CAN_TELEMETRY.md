# SCV2 CAN telemetry receiver guide

The SCV2 controller sends one Classic CAN data frame containing its fast electrical telemetry and fault status. The frame has eight payload bytes, but five logical items: three 16-bit measurements, one reserved byte, and one status byte.

## Frame contract

- Format: Classic CAN data frame (not CAN FD and not a remote frame)
- Identifier: standard 11-bit ID `0x077`
- DLC: 8 bytes
- Nominal bus rate: 1 Mbit/s
- Nominal update rate: 1 kHz
- Multi-byte order: little-endian (least-significant byte first)

The update rate is task-scheduled and best effort. A receiver must tolerate jitter and missing frames rather than assuming that a frame arrives at every exact millisecond.

| Byte(s) | Logical field | Raw type | Scale | Conversion |
|---|---|---|---|---|
| 0–1 | Load power | `uint16_t` | 0.1 W/count | `p_load_W = raw / 10.0` |
| 2–3 | Capacitor voltage | `uint16_t` | 0.1 V/count | `v_cap_V = raw / 10.0` |
| 4–5 | Converter-current command | signed 16-bit two's complement | 0.1 A/count | `i_conv_A = signed_raw / 10.0` |
| 6 | Reserved | `uint8_t` | — | Ignore; currently transmitted as zero |
| 7 | Status | `uint8_t` bitmask | — | Decode the status bits below |

The unsigned fields represent `0` through `6553.5` W/V. Negative load power or capacitor voltage is transmitted as zero, and larger values saturate at `6553.5`. Converter current represents `-3276.8` through `3276.7` A, although the current SCV2 control source clamps its command to ±10 A.

### Status byte

| Bit | Mask | Meaning when set |
|---:|---:|---|
| 0 | `0x01` | Vbus over-voltage fault latched |
| 1 | `0x02` | Vcap over-voltage fault latched |
| 2–7 | — | Reserved; ignore unknown bits |

The fault flags stay latched during the configured recovery interval. Downstream code should test masks rather than compare the complete status byte to a fixed value, so future status bits remain compatible.

## Portable C decoder

The example keeps the measurements in fixed-point deci-units. This avoids floating-point work in an interrupt and preserves the exact transmitted values. Convert to `float` later if the application needs engineering units.

```c
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum
{
  SCV2_TELEMETRY_CAN_ID = 0x077u,
  SCV2_STATUS_VBUS_OVP = 1u << 0,
  SCV2_STATUS_VCAP_OVP = 1u << 1,
};

typedef struct
{
  uint16_t p_load_dW;  /* 0.1 W/count */
  uint16_t v_cap_dV;   /* 0.1 V/count */
  int32_t i_conv_dA;   /* 0.1 A/count; range is signed 16-bit */
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
      (data == NULL) || (telemetry == NULL))
  {
    return false;
  }

  telemetry->p_load_dW = read_u16_le(&data[0]);
  telemetry->v_cap_dV = read_u16_le(&data[2]);
  telemetry->i_conv_dA = read_i16_le(&data[4]);
  /* data[6] is reserved and deliberately ignored. */
  telemetry->status = data[7];
  telemetry->vbus_ovp = (data[7] & SCV2_STATUS_VBUS_OVP) != 0u;
  telemetry->vcap_ovp = (data[7] & SCV2_STATUS_VCAP_OVP) != 0u;
  return true;
}
```

After decoding, engineering-unit access is straightforward:

```c
const float p_load_w = (float)telemetry.p_load_dW * 0.1f;
const float v_cap_v = (float)telemetry.v_cap_dV * 0.1f;
const float i_conv_a = (float)telemetry.i_conv_dA * 0.1f;
```

Do not cast the eight-byte payload directly to a packed C struct. Explicit byte decoding works regardless of the downstream MCU's alignment rules and makes the wire endianness unambiguous.

## Worked frame

Given this payload:

```text
88 13 2C 01 9C FF 00 03
```

- Load power raw: `0x1388 = 5000`, therefore `500.0 W`
- Capacitor voltage raw: `0x012C = 300`, therefore `30.0 V`
- Converter current raw: `0xFF9C = -100`, therefore `-10.0 A`
- Reserved byte: `0x00`
- Status: `0x03`, so both Vbus and Vcap over-voltage flags are set

## Receiver integration guidance

1. Configure the downstream CAN peripheral for 1 Mbit/s Classic CAN and accept standard identifier `0x077` with DLC 8.
2. Reject extended, remote, and wrong-DLC frames before decoding.
3. In the receive ISR/callback, copy or decode the complete frame and record its arrival timestamp. Publish the result to task code using the downstream firmware's normal mailbox, critical-section, or double-buffer mechanism.
4. Keep the fixed-point values when possible; divide by ten only at display or control boundaries.
5. Ignore byte 6 and unrecognized status bits for forward compatibility.
6. Detect stale telemetry using the recorded arrival timestamp and a timeout selected for the downstream application's safety requirements. The SCV2 wire protocol does not impose a universal receiver timeout.

