---
title: Telemetry transports
description: Connect the SCV2 T1 telemetry stream over USB, external UART, or a UDP bridge.
sidebar:
  order: 4
---

SCV2 emits the current 69-field `T1` CSV telemetry record through USB CDC or
its dedicated telemetry UART. Desktop display and UDP bridge behavior are
owned by their separate repositories.

## Transport matrix

| Transport | Settings | Behavior |
|---|---|---|
| Board USB CDC | COM port; displayed baud does not affect USB | Host controls the optional mirror with the `telemetry` CLI command |
| External UART | USART1 TX on PA9, 921600 8-N-1, 3.3 V logic | Always-on transmit stream; does not accept CLI commands |
| UDP bridge | External project configuration | Bridge behavior is not defined by SCV2 firmware |

## USB CDC

Connect the board using a USB data cable. The firmware's telemetry mirror is
controlled with `telemetry on`, `telemetry off`, and `telemetry toggle`; see
[USB CLI](../usb-cli/) for command behavior. Application-specific connection
steps belong to the
[dashboard setup guide](https://github.com/weiliangng/scv2-dashboard/blob/main/docs/SETUP.md).

## External USART1 receiver

1. Use a 3.3 V USB-to-UART adapter.
2. Connect adapter RX to PA9 and connect grounds.
3. Do not drive PA9 and do not apply 5 V logic.
4. Select baud rate `921600`.
5. Clear **Enable USB telemetry while connected** because USART1 streams continuously.

## UDP bridge

An external bridge may forward the USART1 stream over UDP. Its network and
forwarding behavior are not controlled by this firmware repository; consult
the bridge and dashboard repositories.

:::note[One owner per COM port]
The dashboard, PuTTY, CLion's serial monitor, and other serial tools cannot normally share the same COM port. Disconnect one before opening another.
:::

For desktop setup, executable packaging, and application troubleshooting, see
the canonical [dashboard setup guide](https://github.com/weiliangng/scv2-dashboard/blob/main/docs/SETUP.md).
