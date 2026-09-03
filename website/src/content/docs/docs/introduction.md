---
title: Introduction
description: Understand what SCV2 does and choose the right integration path.
sidebar:
  order: 1
---

SCV2 is a bidirectional switching converter that manages a physical supercapacitor energy buffer for power-limited robot chassis motors.

## The energy problem

The referee system limits chassis motor power from **50 W to 120 W** across robot levels 1 to 10. Its 60 J virtual buffer permits short periods above that limit, but excess power drains it quickly:

```text
60 W excess × 1 second = 60 J
```

When chassis demand falls below the limit, unused allowance refills the virtual buffer. A depleted buffer can trigger a five-second chassis shutdown.

## What SCV2 adds

SCV2 measures chassis voltage and current. When chassis power is below the referee limit, it uses the surplus to charge a supercapacitor bank. When demand rises above the limit, the bank supplies the difference.

A bank may store up to 2,000 J. After conversion losses and operating limits, expected usable energy is approximately **1,200–1,400 J**—enough to support transient loads such as acceleration, ramps, and slopes without depleting the referee buffer as quickly.

## Choose an integration path

| Goal | Start here |
|---|---|
| Mount and commission the hardware | [Installation](../installation/) |
| Confirm electrical limits | [Technical specifications](../specifications/) |
| Integrate robot control and telemetry | [Interfaces overview](../../api/overview/) |
| Receive fast CAN telemetry | [CAN telemetry](../../api/can-telemetry/) |
| Diagnose a board in the pit | [Debugging and testing](../../operations/debugging/) |

:::note[Port names]
**IN** is the chassis-bus side and **OUT** is the supercapacitor side. Because power flow is bidirectional, either port can source or sink power.
:::
