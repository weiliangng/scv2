---
title: Debugging and testing
description: Diagnose SCV2 using status, live telemetry, the dashboard, and controlled capacity tests.
sidebar:
  order: 1
---

Start diagnosis with observation. Do not change control or calibration until the reported measurements, mode, and protection state are understood.

## First response

1. Stop other programs that may own the SCV2 COM port.
2. Open the [USB CLI](../../api/usb-cli/).
3. Run:

   ```text
   telemetry off
   status
   ```

4. Record the complete output before changing state.
5. Check faults, undervoltage lockout, stale inputs, capacitor-voltage lockout, direction, and switching timers.

## Live dashboard

The dashboard accepts the 69-field `T1` telemetry record over:

- Board USB CDC
- External USART1 at 921600 baud
- UDP from an external UART bridge

Use [Telemetry transports](../../api/telemetry/) for connection details. The downloadable Windows executable bundles Python, Qt, and pyserial and does not require a development environment on the target PC.

## Switch enable remains off

An accepted command does not override protection. In `status`, confirm:

- The selected mode consumes the command source you are using
- `Safety flag (is_safe)` reports a safe measurement state
- No fault is latched
- Undervoltage lockout is clear
- The capacitor voltage permits the requested direction
- The physical pushbutton has not published a newer off request
- Minimum on/off timing is not delaying the transition

The physical `Switch enable output (SWEN) pin` line is the relevant confirmation.

## Bank-capacity investigation

During a controlled test, inspect:

- `cap_energy_mJ`
- `cap_dE_mJ_min`
- `cap_dV_mV_min`
- Capacitor derate fields

:::caution
A complete capacity-test procedure and pass criteria are not yet published. Do not infer a safe pass/fail threshold from a single telemetry field.
:::

## Further reference

- [Dashboard setup and operation](https://github.com/weiliangng/scv2/blob/main/DASHBOARD_SETUP.md)
- [Complete USB CLI guide](https://github.com/weiliangng/scv2/blob/main/CLI_GUIDE.md)
- [Advanced firmware and telemetry reference](https://github.com/weiliangng/scv2/blob/main/agent.md)
