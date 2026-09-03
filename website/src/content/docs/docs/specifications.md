---
title: Technical specifications
description: Electrical limits, control modes, performance, and protection behavior.
sidebar:
  order: 1
---

## Electrical limits

| Port | Parameter | Range or limit |
|---|---|---|
| **IN — chassis bus** | Voltage | 12–28 V |
| | Current limit | −10 to +10 A; programmable in debug mode |
| | Power setting | −240 to +240 W in 1 W steps |
| | Reverse-flow voltage limit | 26.2 V if the battery disconnects or shuts down while reverse power is set |
| **OUT — supercapacitor** | Voltage and setting | 5–30 V; digitally programmable |
| | Current limit | −15 to +15 A; fixed limits |
| **Response** | Chassis load-step settling | <1 ms; 150 μs typical |
| **Performance** | Peak efficiency | Up to 97% |
| **Energy** | Supported bank | Up to 2,000 J stored |
| | Expected usable energy | Approximately 1,200–1,400 J |

## Sign convention

- **Positive power** flows from IN to OUT and charges the supercapacitor bank.
- **Negative power** flows from OUT to IN and supports the chassis bus.

The current and power limits apply to magnitude and direction. Read negative telemetry values using this convention rather than treating them as faults.

## Regulation

SCV2 supports:

- Constant input-power regulation
- Constant-current / constant-voltage charging
- Digitally programmable capacitor voltage
- External control using referee UART or CAN
- Manual and measurement modes for commissioning
- Guarded direct mode for expert bench work

## Protection and fault handling

The hardware provides:

- Short-circuit current limiting
- Overvoltage detection and shutdown on both ports
- IN-port undervoltage lockout
- Latched overvoltage status reporting
- Capacitor degradation detection
- Charge-voltage foldback when added energy does not produce the expected voltage rise
- Prevention of unintended backfeeding or “zombie power”

:::caution
The hardware is designed to tolerate hot-plugging the source and a charged or discharged capacitor bank, but these are not recommended operating procedures.
:::

## Related reference

- [Operating envelope](../operating-envelope/)
- [Interfaces overview](../../api/overview/)
- [CAN telemetry status bits](../../api/can-telemetry/#status-byte)
