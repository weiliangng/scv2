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

## Graphical telemetry

SCV2 provides the `T1` stream over board USB CDC and external USART1. Use
[Telemetry transports](../../api/telemetry/) for the firmware-owned interface
details. The separate
[dashboard repository](https://github.com/weiliangng/scv2-dashboard) owns the
desktop application, packaging, connection workflow, and troubleshooting.

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

- [Dashboard setup and operation](https://github.com/weiliangng/scv2-dashboard/blob/main/docs/SETUP.md)
- [Complete USB CLI guide](https://github.com/weiliangng/scv2/blob/main/CLI_GUIDE.md)
- [Advanced firmware and telemetry reference](https://github.com/weiliangng/scv2/blob/main/agent.md)
