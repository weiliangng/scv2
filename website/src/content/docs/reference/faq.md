---
title: Frequently asked questions
description: Answers to common SCV2 installation, interface, and diagnostic questions.
sidebar:
  order: 1
---

## Why is switch enable off after a command was accepted?

Check faults, undervoltage lockout, stale inputs, capacitor-voltage lockout, power direction, and switching timers. An `ok` response confirms command acceptance only; it does not override protection or prove that the physical output changed.

Use `status` and inspect the physical `Switch enable output (SWEN) pin` line.

## Can the dashboard and CLI share one USB port?

Not as separate applications. Only one program can normally own a COM port. Close or disconnect the dashboard before opening PuTTY or another terminal.

The separate dashboard application provides an integrated CLI transaction for
this use case; see its
[setup and operation guide](https://github.com/weiliangng/scv2-dashboard/blob/main/docs/SETUP.md).

## Where is the connector pinout?

It is not yet published in this repository. Use the released schematic and connector document before wiring the controller.

## Does USB baud rate affect transfer speed?

No. The CLI uses USB CDC, so the terminal's baud selection does not change physical USB transfer speed. Use 115200 as the project convention. The separate USART1 telemetry output physically operates at 921600 baud.

## Why does external mode report no source?

External mode requires a fresh referee UART or CAN command. It prefers fresh referee UART, then fresh CAN, and enters no-source state if neither source is fresh.

## Can hot-plugging be used as the normal power switch?

No. The hardware is designed to tolerate hot-plug events, but they are not recommended operating procedures. Use the approved fuse, isolation, and shutdown arrangement.

## Does SCV2 need calibration after every installation?

Normal installation does not require individual board calibration after the board has received its initial approved flashing, soldering, and calibration process.
