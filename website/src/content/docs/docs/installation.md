---
title: Installation
description: Safely mount, wire, and commission an SCV2 controller and capacitor bank.
sidebar:
  order: 2
---

:::caution[Verify released hardware documents]
Confirm mounting dimensions, connector pinouts, fuse selection, polarity, and wiring against the released schematic and connector documents before applying power. Those hardware-specific details are not yet published in this repository.
:::

## Before you begin

Prepare:

- An SCV2 controller with current firmware
- A compatible 5–30 V supercapacitor bank
- A 12–28 V chassis bus or current-limited commissioning supply
- Correctly rated wiring, connectors, fuse, and shutdown arrangement
- CAN and/or referee UART wiring required by the robot
- A USB data cable for initial diagnostics

## Mechanical installation

1. Secure the controller and capacitor bank so neither can move under chassis acceleration or impact.
2. Prevent cable strain at every connector.
3. Keep exposed conductors away from the chassis and other conductive objects.
4. Route signal wiring away from high-current switching paths where practical.
5. Make the normal shutdown method accessible before testing.

## Electrical checklist

1. Confirm the chassis bus is within the 12–28 V input range.
2. Confirm the capacitor bank is within the 5–30 V output range.
3. Check the polarity of both power ports before inserting connectors.
4. Verify the fuse, isolation, and shutdown arrangement.
5. Connect CAN at 1 Mbit/s and referee UART at 115200 8-N-1 as required.
6. Connect USB only with a known data-capable cable.

See [Technical specifications](../specifications/) before selecting supplies, loads, wiring, or protection components.

## First power-up

1. Start with a current-limited supply and a known-safe load.
2. Keep the power-stage switch-enable request off.
3. Connect the USB CLI and enter:

   ```text
   telemetry off
   help
   status
   ```

4. Verify measured bus voltage, capacitor voltage, safety state, faults, and physical switch-enable output.
5. Confirm the selected control source is the one expected by the robot.
6. Enable the power stage only after the reported state and physical wiring agree.

:::danger[Command acceptance is not output confirmation]
An `ok` response only means firmware accepted a command. Fault handling, undervoltage lockout, controller policy, and output timing can still prevent a requested physical output. Verify with `status` and the actual hardware.
:::

## Final competition check

- Controller and bank are secure
- Cables cannot pull against connectors
- Power polarity and connector seating are correct
- Fuse and shutdown path are verified
- CAN and referee UART connections are correct
- Controller reports no unexpected faults
- Robot software detects stale or missing telemetry safely
