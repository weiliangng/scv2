---
title: USB CLI
description: Connect to SCV2 over USB CDC and use its diagnostic and control commands safely.
sidebar:
  order: 3
---

The USB CDC command-line interface exposes measurements, controller state, mode selection, manual control, and expert calibration functions.

:::danger[Power hardware]
Commands such as `ctrl`, `pset`, and `swen` change controller behavior. `gpio`, `dac`, and `cal` can directly affect hardware outputs or calibration. Use them only with a known circuit, a current-limited supply, a safe load, and an accessible shutdown method.
:::

## Connect

SCV2 appears on Windows 10 or 11 as `STM32 Virtual ComPort (COMx)` or `USB Serial Device (COMx)`. Use a USB data cable and configure the terminal as:

| Setting | Value |
|---|---|
| Connection | Serial |
| Port | Assigned SCV2 COM port |
| Speed | 115200 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | None |

USB CDC does not physically depend on the selected baud rate; 115200 is the project convention.

## Safe first session

Enter:

```text
telemetry off
help
status
```

`status` is read-only and takes approximately one second because it averages 1,000 measurement samples. It reports mode, physical output pins, faults, lockouts, voltages, currents, energy, power setpoint, input-source state, and peripheral diagnostics.

## Command summary

| Command | Purpose | Risk level |
|---|---|---|
| `help` | Show available commands | Read-only |
| `status` | Print averaged controller and diagnostic state | Read-only |
| `telemetry on\|off\|toggle` | Control the USB `T1` telemetry mirror | Diagnostic |
| `ctrl external\|manual\|measure\|direct` | Select requested controller mode | Control |
| `pset <watts>` | Set manual power from −240 to +240 W | Control |
| `swen 0\|1` | Publish a manual switch-enable request | Control |
| `gpio write <PIN> <0\|1>` | Directly write an MCU GPIO | Expert |
| `gpio toggle <PIN>` | Toggle an MCU GPIO | Expert |
| `dac set <1\|3> <1\|2> <0..4095>` | Write a raw DAC value | Expert |
| `cal set <NAME> <VALUE>` | Change active calibration in RAM | Calibration |
| `cal load` | Replace active values with saved calibration | Calibration |
| `cal save` | Persist the active calibration set | Calibration |

## Controller modes

| Mode | Behavior |
|---|---|
| `external` | Fresh referee UART first, then fresh CAN, then no source. Default after boot. |
| `manual` | Persistent `pset` and `swen` values together with the physical pushbutton. |
| `measure` | Measurements continue while switch enable is forced off. |
| `direct` | Guarded power-stage GPIOs and DAC1 become available for expert bench control. |

Changing mode resets the pushbutton switch-enable request to off. Fault and undervoltage protection override all modes, including direct mode.

## Manual-operation pattern

Use only on an approved current-limited setup:

```text
swen 0
pset 80
ctrl manual
status
swen 1
status
swen 0
ctrl external
status
```

Always request `swen 0` before returning to external control, then verify the physical switch-enable output using `status`.

## Input behavior

- Commands and arguments are case-sensitive and normally lowercase.
- Calibration names are uppercase.
- GPIO names are case-insensitive; `PB4` and `b4` are equivalent.
- Up and Down browse history; normal cursor editing keys are supported.
- `Ctrl+C` cancels the current input line.
- Lines are limited to 119 characters and 15 tokens.

For the complete calibration-name list, GPIO guards, DAC behavior, workflows, and troubleshooting, use the canonical [`CLI_GUIDE.md`](https://github.com/weiliangng/scv2/blob/main/CLI_GUIDE.md).
