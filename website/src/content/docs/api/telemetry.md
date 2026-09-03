---
title: Telemetry transports
description: Connect the SCV2 T1 telemetry stream over USB, external UART, or a UDP bridge.
sidebar:
  order: 4
---

The dashboard displays the current 69-field `T1` CSV telemetry record. SCV2 can deliver that stream through USB CDC or its dedicated telemetry UART; an external bridge can forward the UART stream over UDP.

## Transport matrix

| Transport | Settings | Behavior |
|---|---|---|
| Board USB CDC | COM port; displayed baud does not affect USB | Dashboard enables and disables the USB telemetry mirror |
| External UART | USART1 TX on PA9, 921600 8-N-1, 3.3 V logic | Always-on transmit stream; does not accept CLI commands |
| UDP bridge | UDP port 14551 by convention | External bridge forwards raw UART bytes; dashboard only listens |

## USB CDC

1. Connect the board using a USB data cable.
2. Select **USB serial** in the dashboard.
3. Select the board's COM port.
4. Keep **Enable USB telemetry while connected** checked.
5. Select **Connect**.

The dashboard sends `telemetry on` when it connects and `telemetry off` when it disconnects.

## External USART1 receiver

1. Use a 3.3 V USB-to-UART adapter.
2. Connect adapter RX to PA9 and connect grounds.
3. Do not drive PA9 and do not apply 5 V logic.
4. Select baud rate `921600`.
5. Clear **Enable USB telemetry while connected** because USART1 streams continuously.

## UDP bridge

Configure the bridge to forward raw UART telemetry to UDP port `14551` on the dashboard PC. The dashboard does not discover the bridge, send replies, or require datagram boundaries to align with `T1` records.

:::note[One owner per COM port]
The dashboard, PuTTY, CLion's serial monitor, and other serial tools cannot normally share the same COM port. Disconnect one before opening another.
:::

For development setup, executable packaging, and transport troubleshooting, see the canonical [dashboard guide](https://github.com/weiliangng/scv2/blob/main/DASHBOARD_SETUP.md).
