---
title: Operating envelope
description: Understand the input-current, output-current, and power limits across the voltage range.
sidebar:
  order: 2
---

SCV2 transfer capability is bounded by chassis-bus current, supercapacitor current, and the 240 W power setting limit. The active constraint changes with port voltage.

## At a 24 V chassis bus

![SCV2 power transfer envelope at a 24 V chassis bus. Transfer is limited by 15 A supercapacitor current below 16 V and by 240 W above 16 V.](/scv2/assets/power-envelope-24v.png)

Below approximately 16 V at the supercapacitor port, the ±15 A OUT-port current limit bounds transferable power. At higher capacitor voltage, the ±240 W power limit becomes dominant.

## Across the specified voltage range

![Maximum SCV2 power transfer by chassis-bus voltage and supercapacitor voltage, showing input-current, output-current, and 240 W limits.](/scv2/assets/maximum-power-transfer.png)

Use the lower magnitude imposed by:

1. The ±10 A chassis-bus current limit
2. The ±15 A supercapacitor current limit
3. The ±240 W programmed power limit

:::note
These diagrams describe maximum transfer magnitude. Actual operation can be reduced by control policy, voltage limits, protection state, thermal conditions, or capacitor degradation foldback.
:::
