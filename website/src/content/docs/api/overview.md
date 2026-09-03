---
title: Interfaces overview
description: Choose and configure the referee UART, CAN, telemetry UART, and USB interfaces.
sidebar:
  order: 1
---

SCV2 separates robot control, fast telemetry, and bench diagnostics across four interfaces.

## Connections

| Interface | Configuration | Direction | Primary use |
|---|---|---|---|
| Referee UART | USART3, 115200, 8-N-1 | Receive | Referee power limit and buffer energy |
| CAN | Classic CAN, 1 Mbit/s | Bidirectional | Robot commands and fast telemetry |
| Telemetry UART | USART1 TX on PA9, 921600, 8-N-1 | Transmit | `T1` CSV record approximately every 10 ms |
| USB | CDC virtual COM port | Bidirectional | Interactive CLI and optional `T1` telemetry mirror |

## External control-source selection

External mode resolves the active command source in this order:

1. A fresh referee UART command
2. A fresh CAN command
3. No-source state when neither input is fresh

Command acceptance does not override protection. Faults, undervoltage lockout, capacitor-voltage lockout, direction rules, and switching timers may keep the power stage disabled.

## Choose an interface

### Robot integration

Use **CAN** for robot commands and the 1 kHz fast telemetry frame. Use **referee UART** when SCV2 should receive referee power and energy directly.

### Continuous diagnostics

Use **USART1 telemetry** for an always-on 69-field `T1` stream without sharing the USB port. Connect a 3.3 V receiver to PA9 at 921600 baud.

### Bench and pit work

Use **USB CDC** for the interactive CLI. USB can also mirror `T1` telemetry, but only one host application can normally own the COM port at a time.

## Next steps

- Decode the fast frame: [CAN telemetry](../can-telemetry/)
- Operate the diagnostic console: [USB CLI](../usb-cli/)
- Connect the dashboard: [Telemetry transports](../telemetry/)
