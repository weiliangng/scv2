# SCV2 USB CLI setup and usage guide

This guide explains how to connect to the SCV2 controller's command-line interface (CLI) from a Windows 10 or Windows 11 PC. It assumes that the current SCV2 firmware is already installed on the board. No programming experience is required.

The CLI can display measurements and controller state, select a control source, and change power-stage or calibration settings. Some commands can enable or directly control power hardware, so read the safety section before using anything beyond `help`, `status`, and `telemetry`.

## What you need

- A Windows 10 or Windows 11 PC
- The SCV2 board with current firmware installed
- A USB **data** cable that fits the board; a charge-only cable will not work
- [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html), used here as the serial terminal
- A known-safe, current-limited bench setup before using control, GPIO, DAC, or calibration commands

Only one program can normally use a COM port at a time. Close the SCV2 dashboard, another PuTTY window, CLion's serial monitor, and any other serial-terminal program before connecting.

## Safety first

The following commands are read-only or only change whether diagnostic telemetry is printed over USB:

```text
help
status
telemetry on
telemetry off
telemetry toggle
```

Commands such as `ctrl`, `pset`, and `swen` can change controller behavior. The `gpio`, `dac`, and `cal` commands can directly change hardware outputs or calibration. Use them only with an understood circuit, a current-limited supply, a safe load, and a shutdown method within reach.

An `ok` response means that the firmware accepted the command. It does not prove that a requested physical output changed. Fault handling, undervoltage lockout, controller policy, and output timing can override a request. Run `status` and check the real hardware when verification matters.

## 1. Install PuTTY

1. Open the [official PuTTY download page](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).
2. Under **MSI (Windows Installer)**, download the 64-bit x86 installer for a normal Intel or AMD Windows PC. Use the Arm installer only if the PC runs Windows on Arm.
3. Open the downloaded `.msi` file.
4. Accept the default installation options and finish the installer.
5. Open the Start menu and search for **PuTTY** to confirm it was installed.

Do not download PuTTY from an unofficial software-download site.

## 2. Install or verify the virtual COM-port driver

The SCV2 firmware identifies itself as a USB Communications Device Class (CDC) device named `STM32 Virtual ComPort`. Windows 10 and Windows 11 contain the correct Microsoft `usbser.sys` driver and should install it automatically when the board is connected. A separate ST driver download is not normally needed.

ST's older `STSW-STM32102` package is intended for Windows 8.x and earlier. ST specifically recommends the native Windows driver starting with Windows 10. See the official [Microsoft USB device-class driver documentation](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/supported-usb-classes) and [ST virtual COM-port driver notice](https://www.st.com/en/development-tools/stsw-stm32102.html).

To connect and find the assigned port:

1. Power the board as required by the hardware setup.
2. Connect the board's USB device port directly to the PC with a USB data cable.
3. Wait a few seconds while Windows configures the device.
4. Right-click the Windows Start button and open **Device Manager**.
5. Expand **Ports (COM & LPT)**.
6. Look for `STM32 Virtual ComPort (COMx)` or `USB Serial Device (COMx)`. Record the complete port name, such as `COM8`.

The COM number is assigned by Windows and may differ between PCs or USB sockets. Never assume that an example such as `COM8` is the correct port.

If no new port appears, see [Troubleshooting](#troubleshooting) before installing any third-party driver.

## 3. Open the CLI in PuTTY

1. Start PuTTY.
2. In the **Session** screen, select **Serial** as the connection type.
3. Set **Serial line** to the port found in Device Manager, for example `COM8`.
4. Set **Speed** to `115200`.
5. In the left navigation tree, open **Connection > Serial** and verify:
   - **Speed (baud):** `115200`
   - **Data bits:** `8`
   - **Stop bits:** `1`
   - **Parity:** `None`
   - **Flow control:** `None`
6. Return to **Session** and select **Open**.

The CLI uses USB CDC rather than a physical UART, so the selected baud rate does not change the actual USB transfer speed. `115200` is the project convention and is safe to use. The separate USART1 telemetry output on PA9 uses `921600` baud and does not accept CLI commands.

After opening the port, the terminal may show:

```text
CLI ready. Type 'help'.
scv2>
```

If the board finished booting before PuTTY connected, the startup message may already have been discarded. Press Enter once to display a fresh `scv2>` prompt.

## 4. First connection

Enter these commands one at a time, pressing Enter after each line:

```text
telemetry off
help
status
```

`telemetry off` keeps the interactive terminal readable. The USB telemetry mirror is off after a normal reboot, but explicitly disabling it avoids CSV records becoming mixed with prompts and command output.

`status` is read-only. It averages 1,000 measurement samples with a 1 ms delay, so its response takes about one second. It reports:

- The selected controller mode and physical output-pin states
- Safety faults and undervoltage lockout
- Bus voltage, capacitor voltage, currents, energy, and power setpoint
- CAN, UART, manual-command, and pushbutton state
- USB, USART1, CAN, ADC, DAC, and interrupt diagnostics

The `Current mode` line shows the controller's resolved decision, which can differ from the mode requested with `ctrl`. For example, external mode with no fresh command source reports `no-source`.

## Entering and editing commands

- Command names and their arguments are case-sensitive. Use the lowercase spellings shown in this guide.
- Calibration names are uppercase, for example `A_VBUS`.
- GPIO pin names are case-insensitive and may include or omit the leading `P`; `PB4` and `b4` are equivalent to the parser.
- The Up and Down arrow keys browse command history.
- Left, Right, Home, End, Backspace, and Delete support normal line editing.
- `Ctrl+C` cancels the current input line.
- A command line can contain at most 119 characters and at most 15 command/argument tokens.
- An invalid command name produces `unknown command: <name>`. Invalid arguments normally produce a `usage:` or `err:` message.

## Command reference

### `help`

```text
help
```

Prints the main command list. Two internally dispatched names, `mode` and `dir`, are intentionally absent from that list because their former direct controls are unavailable.

### `status`

```text
status
```

Prints the read-only controller and diagnostic snapshot described above. Allow about one second for the averaging operation to finish.

### `telemetry`

```text
telemetry on
telemetry off
telemetry toggle
```

Controls the `T1` telemetry mirror on the same USB connection as the CLI:

- `on` starts a telemetry record approximately every 10 ms.
- `off` stops the USB mirror and is recommended for interactive CLI use.
- `toggle` changes the current setting.

This command never stops the separate, always-on USART1 telemetry output. When USB telemetry is enabled, `T1,...` CSV lines are expected to appear between the prompt and command responses.

For a graphical telemetry view, use the separate
[SCV2 dashboard repository](https://github.com/weiliangng/scv2-dashboard).

### `ctrl`

```text
ctrl external
ctrl manual
ctrl measure
ctrl direct
```

Selects the controller's requested operating mode:

| Argument | Purpose |
|---|---|
| `external` | Uses fresh referee UART power and energy first, otherwise fresh CAN, otherwise no source. This is the default mode after boot. |
| `manual` | Uses the persistent values set by `pset` and `swen`, together with the physical pushbutton. |
| `measure` | Keeps measurements running while forcing switch enable off. |
| `direct` | Leaves guarded power-stage GPIOs and DAC1 available for direct bench control. This is an expert-only mode. |

Changing the requested mode resets the pushbutton switch-enable request to off. Fault and undervoltage handling still override every mode, including direct mode.

### `pset`

```text
pset <watts>
```

Publishes a manual power setpoint from `-240` through `240` watts. Examples:

```text
pset 80
pset -50
pset 0
```

The value only controls power while manual mode is selected. It does not select manual mode and does not enable the power stage. If no value has been set since boot, manual mode uses a 50 W default. The mailbox value remains present until the board resets or another `pset` command replaces it.

### `swen`

```text
swen 0
swen 1
```

Publishes a persistent manual switch-enable request:

- `0` requests off.
- `1` requests on.

It only affects output selection in manual mode. The most recent manual `swen` command or physical pushbutton event wins. Safety, undervoltage lockout, and the firmware's output timing can still keep the physical SWEN output off.

### `gpio` — expert use only

```text
gpio write <PIN> <0|1>
gpio toggle <PIN>
```

Examples of valid syntax:

```text
gpio write PB4 0
gpio toggle PA10
```

The parser accepts MCU ports A through G and pin numbers 0 through 15, but this is **not** a list of safe pins. A syntactically valid pin may be an input, may belong to a peripheral, or may be critical to the board.

The following guarded power-stage pins require `ctrl direct` before they can be written or toggled:

| Pin | Firmware name |
|---|---|
| `PB1` | DIR |
| `PB4` | SWEN |
| `PB5` | MODEMSB |
| `PB6` | MODELSB |
| `PA10` | LED |

Other pins do not receive this direct-mode protection. Do not experiment with arbitrary GPIO numbers.

### `dac` — expert use only

```text
dac set <1|3> <1|2> <0..4095>
```

Writes a raw 12-bit value to DAC1 or DAC3, channel 1 or 2. For example:

```text
dac set 3 1 920
```

DAC1 writes require `ctrl direct`. DAC3 writes do not. A raw value of 0 corresponds approximately to 0 V and 4095 approximately to 3.3 V, but the electrical effect depends on where the DAC output is connected in the circuit.

A `dac set` value is normally volatile. On reboot, DAC3 uses the saved `DAC3_CH1_BOOT_U12` and `DAC3_CH2_BOOT_U12` calibration values; DAC1 returns to firmware-controlled startup behavior.

### `cal` — calibration use only

```text
cal set <NAME> <VALUE>
cal load
cal save
```

`cal set` changes one active value in RAM. The accepted names are:

```text
A_VBUS B_VBUS A_ILOAD B_ILOAD MIDPOINT
A_INP B_INP A_INN B_INN
A_VCAP B_VCAP
A_OP B_OP A_ON B_ON
DAC3_CH1_BOOT_U12 DAC3_CH2_BOOT_U12
```

Floating-point values must be finite. `A_VBUS` must also be nonzero. The two DAC3 boot values must be integers from 0 through 4095 and are applied to the physical DAC immediately.

`cal save` writes the complete active calibration set to emulated EEPROM in flash. `cal load` replaces active RAM values with the saved set and applies both saved DAC3 boot outputs. If no valid saved set exists, `cal load` prints `err: no valid saved cal`.

There is no read-only command that prints the active calibration set without first loading it. A successful `cal load` prints the values it loaded, but it also discards any unsaved RAM changes. Do not use it as an inspection command when unsaved work must be preserved.

### Unavailable legacy commands

Entering either of these names:

```text
mode
dir
```

returns:

```text
unavailable: main mode control is staged off
```

They do not change the controller. The guarded `gpio` interface in direct mode is the current low-level mechanism.

## Safe example workflows

### Observe the board without changing control

```text
telemetry off
status
```

This is the recommended first workflow and is safe for routine diagnosis.

### Manual operation with an explicit shutdown

Only perform this workflow on an approved, current-limited bench setup:

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

Set the manual SWEN request to off before selecting manual mode. Confirm the `Switch enable output (SWEN) pin` line before and after enabling. Always issue `swen 0` before returning to external control.

### Direct GPIO or DAC1 work

Direct mode does not automatically clear every stage output when entered. Plan every pin/value change and the shutdown sequence before starting:

```text
ctrl direct
gpio write PB4 0
# perform only the previously planned GPIO or DAC operations
gpio write PB4 0
ctrl external
status
```

The line beginning with `#` is an explanation, not a CLI command; do not paste it into the terminal. Put every altered output into its intended safe state before leaving direct mode.

### Calibration session

1. Record a `status` snapshot and the existing approved calibration values before changing anything.
2. Use `cal set` for one value at a time.
3. Validate the resulting measurements with suitable bench equipment.
4. Repeat until the complete active set is known-good.
5. Run `cal save` only after all active values have been verified.
6. Reboot or use `cal load`, then verify that the saved values behave as expected.

## Ending a session

Before closing PuTTY after any manual or direct-control work:

1. Request `swen 0` when using manual mode, or explicitly clear PB4 in direct mode.
2. Return to the required normal mode, usually `ctrl external`.
3. Run `status` and confirm the physical SWEN output is off or otherwise in the intended state.
4. Close the PuTTY window.

Closing PuTTY only releases the PC's COM port. It does not reset the board, clear manual mailbox values, change the selected controller mode, or automatically disable hardware.

## Troubleshooting

### No COM port appears in Device Manager

- Confirm the board is powered.
- Try a cable known to carry data; many inexpensive USB cables only provide power.
- Try a different USB socket on the PC and avoid an unpowered hub.
- Disconnect the board, note the existing COM ports, reconnect it, and watch which entry appears.
- Check **Other devices** and **Universal Serial Bus controllers** for an entry with a warning symbol.
- Right-click the problem device, choose **Update driver**, then **Search automatically for drivers**. Also allow Windows Update to finish pending driver updates.
- If available, try the board and cable on another Windows 10/11 PC to separate a board/cable fault from a PC configuration problem.

Do not install the legacy ST VCP package as the first fix on Windows 10 or Windows 11.

### PuTTY reports that it cannot open the serial port

- Recheck the COM number after reconnecting the board; Windows may assign a different number.
- Close the dashboard, other PuTTY sessions, CLion monitors, and any application using the port.
- Disconnect and reconnect the board, then reopen PuTTY.
- If the board reset or disappeared, close the old PuTTY session before trying again.

### PuTTY opens but the window is blank

- Press Enter once. The board may have printed its startup message before PuTTY connected.
- Confirm that PuTTY is using **Serial**, not SSH or Telnet.
- Confirm that the selected COM port belongs to the SCV2 board.
- Reconnect the USB cable and wait for the port to reappear.

### Typed characters do not appear or commands do not run

- Set PuTTY flow control to **None**.
- Confirm that the correct COM port is open.
- Press `Ctrl+C`, then Enter, to cancel a partially entered line and recover a prompt.
- Disconnect other software that may be writing commands to the same device.

### `T1,...` lines continuously cover the prompt

USB telemetry is enabled. Type:

```text
telemetry off
```

The command may appear mixed into a telemetry line while you type, but the firmware still receives it. If necessary, type it carefully and press Enter once.

### A command prints `usage:`

The spelling, number of arguments, or numeric range is invalid. Command words are case-sensitive. Run `help` and compare the command with the examples in this guide.

### A GPIO or DAC1 command says that direct mode is required

The firmware is protecting a power-stage output. Do not bypass the check until the direct operation and a safe shutdown sequence have been planned. On an approved bench setup, select `ctrl direct`, perform only the planned changes, restore safe outputs, and leave direct mode.

### `swen 1` returns `ok` but the output remains off

Check all of the following in `status`:

- The controller must be in manual mode for the manual SWEN mailbox to be selected.
- `Safety flag (is_safe)` must indicate a safe measurement state.
- No fault may be latched.
- Undervoltage lockout must be clear.
- The physical pushbutton may have published a newer off request.
- Minimum on/off timing may delay a transition briefly.

The physical `Switch enable output (SWEN) pin` line is the relevant confirmation; `ok` only confirms command acceptance.

## Related documentation

- [SCV2 dashboard](https://github.com/weiliangng/scv2-dashboard)
- [Downstream CAN telemetry receiver guide](CAN_TELEMETRY.md)
- [`agent.md`](agent.md), the advanced firmware, telemetry, and bench-test reference
