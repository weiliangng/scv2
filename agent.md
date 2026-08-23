# SCV2 USB CLI, control, and telemetry guide

This document is the operating reference for agents and bench users working with the SCV2 firmware. The command implementation in `Core/Src/usb_cli.c`, the resolver in `Core/Src/scap_io_owner.c`, and constants in `Core/Inc/app_constants.h` are the source of truth. Check those files before extending this guide or relying on a remembered behavior.

## Agent operating rules

- Start with read-only observation: connect the USB CDC port, run `help`, disable the USB telemetry mirror if it obscures the prompt, and run `status`.
- Infer command effects by tracing the CLI handler through its mailbox or HAL call and then through `ScapIo_Resolve1kHz()` and the ADC ISR. An `ok` response only means the command was accepted; verify the requested state with `status` or `T1`, and verify the physical result with `swen_out`, GPIO measurement, or appropriate bench equipment.
- Do not guess the COM port, safe supply/load conditions, calibration values, GPIO target, DAC value, or acceptable power setpoint. If the requested hardware operation or its safe value is not explicit and cannot be established from the repository and fixture description, ask the user before issuing a mutating command.
- Treat `ctrl direct`, `gpio`, `dac`, `cal set`, `cal load`, and `cal save` as hardware-mutating operations. Use a current-limited, known-safe bench setup and arrange a shutdown path before using them.
- When this document and firmware disagree, document the observed mismatch and follow the current source. Do not silently preserve an obsolete test expectation.

## USB CLI quick start

The CLI is exposed as the board's USB CDC virtual COM port. Repository tooling conventionally opens it at 115200 baud, 8-N-1; USB CDC does not use that setting as a physical UART baud rate. The separate USART1 telemetry output is 921600 baud, 8-N-1 and is not the CLI.

1. Connect the board's USB device port and identify its virtual COM port. Do not assume the example `COM8` is correct for the current host.
2. Open the port in a serial terminal. Either CR or LF submits a command.
3. Wait for `CLI ready. Type 'help'.` and the `scv2> ` prompt. Press Enter once if the board was already running before the terminal connected.
4. Run `telemetry off` for a clean interactive prompt, then run `help` and `status`.

`status` deliberately averages 1000 samples with a 1 ms delay, so expect it to take about one second. It does not write control state, but command freshness and physical conditions can change while it is sampling. `telemetry on` interleaves `T1` records with ordinary CLI output on the same USB stream; the USART1 `T1` stream remains on regardless of this setting.

## CLI command reference

| Command | What it does | Important constraints |
|---|---|---|
| `help` | Prints the supported command syntax. | `dir` and `mode` are dispatched but intentionally absent because they are unavailable. |
| `status` | Reports control decision, physical pins, safety state, averaged electrical values, command mailboxes, telemetry/link statistics, DAC values, and ISR timing. | Read-only, but takes about one second. The displayed "Current mode" is the resolver decision, not merely the requested `ctrl` mode. |
| `telemetry on\|off\|toggle` | Controls the lossy USB mirror of `T1`. | Does not change the always-on USART1 stream or control behavior. |
| `ctrl external` | Selects external control. | Fresh UART power **and** energy win over fresh CAN; otherwise fresh CAN is used; otherwise the decision is `no-source`. |
| `ctrl manual` | Selects manual algorithm control. | Uses the last `pset`, or 50 W if none has been published. Manual SWEN is selected by timestamp against the pushbutton mailbox. |
| `ctrl measure` | Selects measurement mode. | Forces the requested SWEN output off without waiting for the minimum-on timer. |
| `ctrl direct` | Relinquishes normal MODE, DIR, SWEN, LED, and DAC1 algorithm updates so they can be driven directly. | Fault/watchdog and UVLO handling still take priority. Enter only on a safe bench. |
| `pset <-240..240>` | Publishes a persistent manual power mailbox in watts. | It affects power only in manual mode; it does not switch modes or enable SWEN. |
| `swen <0\|1>` | Publishes a persistent manual SWEN mailbox. | It affects output selection only in manual mode. The newer pushbutton/manual mailbox wins, and safety still overrides it. |
| `gpio write <PIN> <0\|1>` | Writes a GPIO output. | The parser accepts optional `P`, ports compiled for the MCU, and pins `0..15`, case-insensitively; it is not an allowlist of safe outputs. |
| `gpio toggle <PIN>` | Toggles a GPIO output. | Same parser and hazards as `gpio write`. |
| `dac set <1\|3> <1\|2> <0..4095>` | Writes a raw 12-bit DAC value. | DAC1 requires `ctrl direct`; DAC3 does not. Values are volatile unless represented by saved boot calibration. |
| `cal set <NAME> <VALUE>` | Changes an active calibration constant in RAM. | Does not persist until `cal save`; the two DAC3 boot values are also applied to hardware immediately. |
| `cal load` | Loads the saved calibration set from emulated EEPROM and applies the saved DAC3 boot values. | Returns an error if no valid saved set exists and replaces unsaved RAM changes. |
| `cal save` | Saves the complete active calibration set to emulated EEPROM. | Persistent hardware mutation; verify values before saving. |
| `dir ...` / `mode ...` | Returns `unavailable: main mode control is staged off`. | Use guarded GPIO commands in direct mode instead. |

Power-stage GPIO writes and toggles for PB1/DIR, PB4/SWEN, PB5/MODEMSB, PB6/MODELSB, and PA10/LED are rejected unless the requested mode is direct. Other syntactically valid pins are not protected by this check; writing a peripheral-owned, input-only, or otherwise unsafe pin can still disrupt the board.

Calibration names are `A_VBUS`, `B_VBUS`, `A_ILOAD`, `B_ILOAD`, `MIDPOINT`, `A_INP`, `B_INP`, `A_INN`, `B_INN`, `A_VCAP`, `B_VCAP`, `A_OP`, `B_OP`, `A_ON`, `B_ON`, `DAC3_CH1_BOOT_U12`, and `DAC3_CH2_BOOT_U12`. Float inputs must be finite; `A_VBUS` must also be nonzero, and DAC boot values must be `0..4095`.

## Safe command workflows

### Observe without changing control

```text
telemetry off
status
```

Use `telemetry on` only when a consumer is ready to separate complete `T1` lines from the prompt and command responses.

### Manual power and SWEN

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

Set `swen 0` and confirm the physical SWEN pin before enabling the power stage. The `pset` and manual `swen` mailboxes persist until reset or replacement. A mode or resolver-decision transition publishes pushbutton SWEN off; after entering manual mode, the newest manual `swen` or physical pushbutton event determines the request.

### Direct GPIO or DAC1 work

```text
ctrl direct
gpio write PB4 0
# perform only the explicitly planned GPIO/DAC operations
gpio write PB4 0
ctrl external
status
```

Direct mode leaves the current stage outputs in their applied state; entering it does not automatically clear every pin. Before leaving, explicitly put altered outputs into the intended safe state. Fault/watchdog handling can still force SWEN off, and UVLO can still update DIR/DAC1 while holding SWEN off.

### Calibration

At the start of a calibration session, before making unsaved changes, record `status` and the values printed by a successful `cal load`. Use `cal set` for one value at a time, validate the measurements, and run `cal save` only after the complete active set is known-good. There is no read-only CLI command that prints the active calibration set: `cal load` replaces RAM values and applies the saved DAC3 boot outputs, so never use it merely to inspect values that must not be changed.

## Control behavior inferred from `scap_io_owner.c`

The 1 kHz resolver publishes a double-buffered fast command, and the ADC ISR applies the physical DIR, DAC1, SWEN, and LED behavior. The priority and source flow are:

1. A watchdog failure or latched Vbus/Vcap over-voltage fault forces SWEN off immediately. A fault clears only after 500 consecutive safe 1 kHz resolver cycles and a subsequent fault-free fast update.
2. Vbus UVLO enters at `<=10 V`, exits at `>=11 V`, and forces SWEN off.
3. Direct mode leaves normal power-stage output ownership to direct writes; measure mode forces SWEN off; manual mode uses manual inputs; external mode selects fresh UART power+energy before fresh CAN.
4. CAN and UART fields are fresh for 300 ms. No fresh external source produces `no-source`, zero power, and a false SWEN policy request.
5. Outside direct, measure, fault, and UVLO handling, SWEN observes a 50 ms minimum-on time and a 1 ms minimum-off time.

For enabled external commands with valid energy, SWEN permission is based on converter direction and Vcap lockout, not on crossing an energy threshold. The initial runtime maximum is 26.3 V. Charging is allowed only while `i_conv > 0` and Vcap is below that maximum; it resumes at 0.1 V below the current maximum. The discharge lockout is 20% of the current maximum and resumes 0.1 V above that level. Initially these thresholds are 26.3/26.2 V for charge and 5.26/5.36 V for discharge. A zero converter-current command allows neither direction.

Energy changes the resolved power setpoint while a direction is allowed:

| Direction | Energy condition | Resolved power adjustment |
|---|---|---:|
| Charge | `energy > 50 J` | `command power + 10 W` |
| Charge | `energy <= 50 J` | `command power - 20 W` |
| Discharge | `energy < 20 J` | `command power - 10 W` |
| Discharge | `energy >= 20 J` | `command power + 20 W` |

CAN energy `777` is the disabled sentinel: it bypasses both the direction/Vcap energy-policy permission and the power adjustment, but it does not bypass command freshness, the CAN SWEN bit, fault/watchdog handling, UVLO, or minimum-on/off timing.

Outside direct mode the resolver writes MODE[1:0] to `01`, the normal algorithm mode. Use `swen_req` to see the selected source's request, `decision` to see which control branch won, `pset_W` to see the adjusted power, and `swen_out` to verify what the hardware actually received.

## Telemetry `T1` decoder

USART1 emits one CSV record every 10 ms from boot at 921600 baud, 8-N-1. The USB CLI command `telemetry on|off|toggle` controls an independent USB CDC mirror of the same records; it does not affect USART1 or control. Every record starts with `T1`; remaining columns are positional and must be decoded in the order below. It is a best-effort snapshot, so DMA, register, and shared-state values can be up to one control cycle apart.

### Conventions

- **Binary** is `0` or `1`.
- **Continuous** is a decimal integer in the stated unit.
- **N-ary** is a decimal enum defined below.
- Raw ADC and DAC values are unsigned 12-bit counts (`0..4095`).
- `valid` means a mailbox has published an accepted value. `fresh` means it is within the shared 300 ms command timeout for CAN and UART.

### Columns

| # | Field | Type / range | Meaning and source |
|---:|---|---|---|
| 1 | `seq` | continuous, uint32 | T1 telemetry record sequence shared by USART1 and USB. |
| 2 | `adc_hz` | continuous, Hz | Measured ADC1 DMA sequence rate. |
| 3 | `usb_drop` | continuous, uint32 | Cumulative records rejected by the lossy USB telemetry queue. |
| 4 | `dma_last` | continuous, cycles | Last ADC1 DMA ISR execution time. |
| 5 | `dma_max` | continuous, cycles | Maximum ADC1 DMA ISR execution time since boot. |
| 6 | `adc_vcap` | continuous, 0..4095 | ADC1 rank 1 raw Vcap input: PA0 / ADC1_IN1. |
| 7 | `adc_vbus` | continuous, 0..4095 | ADC1 rank 2 raw Vbus input: PA1 / ADC1_IN2. |
| 8 | `adc_iload` | continuous, 0..4095 | ADC2 rank 1 differential Iload sample: PA6/PA7. |
| 9 | `adc_iop` | continuous, 0..4095 | ADC2 rank 2 IMONOP sample via OPAMP2: PB14. |
| 10 | `adc_ion` | continuous, 0..4095 | ADC2 rank 3 IMONON sample via OPAMP3: PB13. |
| 11 | `vc_mV` | continuous, signed mV | `adc_vcap` after active Vcap calibration. |
| 12 | `vb_mV` | continuous, signed mV | `adc_vbus` after active Vbus calibration. |
| 13 | `il_mA` | continuous, signed mA | `adc_iload` after active Iload calibration. |
| 14 | `iop_mA` | continuous, signed mA | `adc_iop` after active IMONOP calibration. |
| 15 | `ion_mA` | continuous, signed mA | `adc_ion` after active IMONON calibration. |
| 16 | `io_mA` | continuous, signed mA | Selected output current: larger of IMONOP and negative IMONON. |
| 17 | `ic_mA` | continuous, signed mA | Calculated converter-current command, clamped to ±10 A. |
| 18 | `pset_W` | continuous, signed W | Power setpoint selected by the control resolver. |
| 19 | `btn_in` | binary | Physical PC13 pushbutton level; control uses its debounced rising edge. |
| 20 | `dir_out` | binary | Physical PB1 direction output; `1` is positive converter-current direction. |
| 21 | `swen_out` | binary | Physical PB4 switch-enable after safety, direction/Vcap policy, and min-on/off gating. |
| 22 | `mode_out` | n-ary, 0..3 | Physical PB5:PB6 mode bits (`2*MODEMSB + MODELSB`); normal algorithm mode is `1`. |
| 23 | `rvsoff_out` | binary | Physical PB3 reverse-switch-off output. |
| 24 | `nsil_out` | binary | Physical PB7 NSIL output. |
| 25 | `led_out` | binary | Physical PA10 status LED output. |
| 26 | `dac1_ch1` | continuous, 0..4095 | Live DAC1 channel 1, IMONINP comparator/reference output. |
| 27 | `dac1_ch2` | continuous, 0..4095 | Live DAC1 channel 2, IMONINN comparator/reference output. |
| 28 | `dac3_ch1` | continuous, 0..4095 | Live DAC3 channel 1, OPAMP1 feedback/reference output. |
| 29 | `dac3_ch2` | continuous, 0..4095 | Live DAC3 channel 2, COMP4 reference output. |
| 30 | `mode_req` | n-ary, 0..3 | Requested mode: `0=external`, `1=manual`, `2=measure`, `3=direct GPIO`. |
| 31 | `decision` | n-ary, 0..7 | Resolver result: `0=fault disable`, `1=idle/UVLO`, `2=no source`, `3=manual algorithm`, `4=CAN algorithm`, `5=UART algorithm`, `6=measure`, `7=direct GPIO`. |
| 32 | `swen_req` | binary | Resolver request before final output gating. |
| 33 | `safe` | binary | Current ADC safety check has no Vbus/Vcap over-voltage fault. |
| 34 | `uvlo` | binary | Undervoltage lockout; enters at Vbus ≤10 V, exits at Vbus ≥11 V. |
| 35 | `fault_latched` | binary | Over-voltage fault is latched and forces SWEN off until recovery. |
| 36 | `fault_bits` | bitmask, 0..3 | `bit0=Vbus ≥30 V`; `bit1=Vcap ≥30 V`. |
| 37 | `fault_healthy_ms` | continuous, 0..500 | Consecutive safe time while clearing a latched fault. |
| 38 | `can_bus` | binary | Any CAN FIFO1 traffic observed in the last 200 ms. |
| 39 | `can_p` | continuous, W 50..120 when valid | CAN command power. |
| 40 | `can_p_valid` | binary | Valid CAN command packet accepted since boot. |
| 41 | `can_p_fresh` | binary | CAN command is no more than 300 ms old. |
| 42 | `can_e` | continuous, J 0..60 or 777 | CAN energy used for power adjustment; `777` disables the energy/Vcap policy. |
| 43 | `can_e_valid` | binary | Accepted CAN energy is usable: packet valid and `can_e != 777`. |
| 44 | `can_e_fresh` | binary | CAN energy packet freshness, shared with CAN power. |
| 45 | `can_e_disabled` | binary | CAN energy is the `777` disabled sentinel. |
| 46 | `can_swen` | binary | SWEN request carried by the CAN packet. |
| 47 | `can_swen_valid` | binary | Valid CAN command packet accepted since boot. |
| 48 | `can_swen_fresh` | binary | CAN SWEN packet freshness, shared with CAN power. |
| 49 | `uart_p` | continuous, W 50..120 when valid | Referee UART chassis power limit. |
| 50 | `uart_p_valid` | binary | CRC-validated referee power frame parsed since boot. |
| 51 | `uart_p_fresh` | binary | UART power is no more than 300 ms old. |
| 52 | `uart_e` | continuous, J 0..60 when valid | Referee UART buffer energy. |
| 53 | `uart_e_valid` | binary | CRC-validated referee energy frame parsed since boot. |
| 54 | `uart_e_fresh` | binary | UART energy is no more than 300 ms old. |
| 55 | `uart_swen_req` | binary | Derived request: `1` only while UART power and energy are both fresh. |
| 56 | `man_p` | continuous, signed W -240..240 when valid | Manual CLI power mailbox; it does not expire. |
| 57 | `man_p_valid` | binary | Manual power has been set by the CLI since boot. |
| 58 | `man_swen` | binary | Manual CLI SWEN mailbox request; it does not expire. |
| 59 | `man_swen_valid` | binary | Manual SWEN has been set by the CLI since boot. |
| 60 | `btn_swen` | binary | Debounced pushbutton SWEN mailbox request; it does not expire. |
| 61 | `btn_swen_valid` | binary | Pushbutton SWEN mailbox set since boot or reset by a mode transition. |
| 62 | `cap_energy_mJ` | continuous, signed mJ | Integrated capacitor energy; charging adds and discharging subtracts. |
| 63 | `vcap_max_mV` | continuous, mV | Current runtime Vcap maximum used by charge lockout, discharge lockout, and CAN capacity scaling. |
| 64 | `cap_unhealthy` | binary | Latched once three consecutive bad one-minute capacitor-health windows cause a derate. |
| 65 | `cap_bad_windows` | continuous, 0..3 | Current consecutive bad-window count; reset after a derate or a good window. |
| 66 | `cap_derates` | continuous, uint8 | Number of 0.1 V runtime derates since boot. |
| 67 | `cap_dE_mJ_min` | continuous, signed mJ/min | Integrated energy change measured over the last completed one-minute health window. |
| 68 | `cap_dV_mV_min` | continuous, signed mV/min | Vcap change measured over the last completed one-minute health window. |

`cap_energy_mJ` starts at zero. The 50 kHz ADC ISR integrates signed `Vcap * Iout` energy into 50-sample (1 ms) blocks; the 1 kHz task transfers completed blocks into the signed run-long counter while retaining fractional mJ. `Iout` is forced to zero while SWEN is off. On each upward crossing of 23.0 V the counter is rebased to 1,322,500 mJ, the ideal stored energy of a 5 F capacitor at 23.0 V. The anchor re-arms after Vcap falls to 22.9 V or below.

The capacitor-health check compares energy and voltage across non-overlapping one-minute windows. A window is bad when energy rises by at least 500,000 mJ while Vcap rises by less than 100 mV. Three consecutive bad windows latch `cap_unhealthy`, reduce the runtime Vcap maximum by 0.1 V, and restart the count. Repeated groups of three bad windows continue derating down to a 23.0 V floor. The maximum, latch, and counters reset on reboot; the fixed 30 V hardware-safety threshold is unchanged.

### Control precedence

In external mode, fresh UART power plus energy wins; otherwise fresh CAN is used; otherwise the controller has no source. In manual mode, the newer pushbutton or manual-SWEN mailbox wins. Faults and UVLO override normal operation. Valid external energy adjusts `pset_W`; converter direction and Vcap lockout determine whether that source may enable SWEN. Use `swen_out` to verify what hardware actually received.

### Local dashboard

`tools/scv2_dashboard.py` is the read-only local viewer for this exact `T1` schema. It consumes only complete records with all 68 columns and ignores ordinary CLI output. Its field order is intentionally tied to this document.

- Continuous fields are shown as live current-value cards; the dashboard does not retain or graph historical samples. Invalid CAN, UART, and manual command values are greyed out rather than shown as zero.
- Validity and freshness fields are displayed as `VALID`/`INVALID` and `FRESH`/`STALE`; invalid or stale values are grey.
- Binary fields are semantic state cards such as `ON`/`OFF`, `SAFE`/`UNSAFE`, and `UP`/`DOWN`.
- N-ary resolver values are decoded: for example, `decision=5` is shown as `UART` and `decision=1` as `IDLE / UVLO`.

Start it by double-clicking `Start SCV2 Dashboard.cmd`, or run:

```powershell
.venv\Scripts\python.exe tools\scv2_dashboard.py --port COM8
```

For USB CDC, the dashboard enables `telemetry on` when it connects and sends `telemetry off` before disconnecting. For an external USB-UART receiver connected to PA9, select 921600 baud and uncheck "Enable USB telemetry while connected"; USART1 is already streaming.

For the ESP32 UART bridge, select **UDP listener**, leave **UDP port** at `14551` for UART1, and connect. The dashboard binds that local port and only receives raw datagrams; it does not send discovery traffic or expect a reply from `192.168.4.1`. Datagram boundaries are not treated as telemetry-message boundaries: received bytes are buffered until a newline completes a record. Use `--transport udp --udp-port 14551` to open in that mode. Use `--demo` to inspect the display without a board.

## CAN five-byte command test guide

This is an automation-ready bench-test matrix for commands sent by the DEVC/peer device to this board. It covers protocol and control-behaviour equivalence classes and validation boundaries; it is intentionally not a brute-force test of all `2^40` possible payloads.

### Test-rig correlation and safe profiles

- The sender STM32 writes `CASE <id>` to its USB CDC port immediately before it transmits the listed CAN frame. The host records that event and SCV2's `T1` stream with one monotonic PC clock; it never tries to correlate by the ignored `reset` byte.
- Enable `telemetry on` on SCV2. A normal target observation is the first complete `T1` record after a case event, within 30 ms. FIFO1-only traffic is polled at 100 ms, so its `can_bus=1` observation has a 130 ms deadline. `T1` itself is sampled every 10 ms.
- Profile `S` means `mode_req=0`, both UART freshness fields are `0`, `safe=1`, `uvlo=0`, `fault_latched=0`, and no earlier energy/Vcap lockout is retained. Use a current-limited supply and a safe load; start with `enable_module=0` cases.
- Baseline `B0` is the accepted frame `[00 00 55 0A 00]`: power 85 W, energy 10 J, SWEN off. For a rejection test, send B0, wait at least 310 ms, then send the candidate. Immediately before the candidate, the expected retained state is `can_p=85`, `can_p_valid=1`, `can_p_fresh=0`, `can_e=10`, `can_e_valid=1`, `can_e_fresh=0`, `can_e_disabled=0`, `can_swen=0`, `can_swen_valid=1`, `can_swen_fresh=0`, `can_bus=0`, `decision=2`, `pset_W=0`, `swen_req=0`, and `swen_out=0`.
- `valid` means a command was accepted since boot; it does **not** expire. All three CAN `fresh` fields use the same accepted-command timestamp and become `0` only after more than 300 ms without another accepted command.
- Do not assert exact `seq`, ADC/DAC counts, measured voltages/currents, `dir_out`, or `led_out` unless the fixture controls their inputs. `swen_out` is exact only when the stated analogue/safety profile and its 1 ms/50 ms timing history make it deterministic.

### Wire contract

Send a Classic CAN **standard data frame** with 11-bit ID `0x067` and DLC `5`.

| Byte | Name | Encoding | Accepted values | Effect |
|---:|---|---|---|---|
| 0 | `enable_module` | unsigned byte | `0` or `1` | Publishes CAN SWEN request: `0=off`, `1=on`. |
| 1 | `reset` | unsigned byte | any `0..255` | Currently ignored; it does not reset faults, timers, or the mailbox. |
| 2 | `pow_limit` | unsigned byte, W | `50..120` | Sets the CAN power setpoint. |
| 3 | `energy_buffer` LSB | unsigned byte | see below | Low byte of the little-endian energy value. |
| 4 | `energy_buffer` MSB | unsigned byte | see below | High byte of the little-endian energy value. |

`energy_buffer = byte[3] + 256 * byte[4]`. It is valid when `0..60 J`, or when it is exactly `777` (`0x0309`, bytes `09 03`). `777` is the disabled-energy sentinel: it bypasses direction/Vcap permission and power adjustment but does not bypass freshness, the CAN SWEN bit, UVLO, fault handling, or the SWEN timing limiter.

For sender notation below, payloads are hexadecimal bytes in the order on the wire: `[enable reset power energy_LSB energy_MSB]`.

For every accepted packet with values `(P,E,SW)`, the common `T1` assertion within 30 ms is: `can_bus=1`, `can_p=P`, `can_p_valid=1`, `can_p_fresh=1`, `can_e=E`, `can_e_fresh=1`, `can_e_disabled=(E==777)`, `can_e_valid=(E!=777)`, `can_swen=SW`, `can_swen_valid=1`, and `can_swen_fresh=1`. Under profile `S`, it additionally gives `mode_req=0`, `decision=4`, and `swen_req=SW`. `pset_W=P` when `SW=0`, when `E=777`, or while no direction is allowed; an enabled valid-energy command uses the direction-dependent adjustment table above. The per-case cells below state the remaining exact or conditional output assertions.

### Transport and frame-shape tests

| Case | Frame sent and precondition | Expected observed `T1` values |
|---|---|---|
| T1 baseline | `S`; standard data ID `0x067`, DLC 5, `[00 00 50 00 00]` | Common accepted assertion `(50,0,0)`; also `swen_out=0`. |
| T2 wrong standard ID | `B0`; standard data ID `0x066`, DLC 5, `[00 00 50 00 00]` | Within 130 ms, `can_bus=1`. All B0 mailbox values remain retained and stale: `can_p=85`, `can_e=10`, `can_swen=0`, all CAN `valid=1`, all CAN `fresh=0`; in `S`, `decision=2`, `pset_W=0`, `swen_req=0`, `swen_out=0`. |
| T3 extended ID | `B0`; extended data ID `0x67`, DLC 5, `[00 00 50 00 00]` | Same T1 assertion as T2. The global non-matching extended filter sends this to FIFO1, not the command callback. |
| T4 remote frame | `B0`; standard remote ID `0x067`, DLC 5 | Through 210 ms, `can_bus=0` and every B0 field remains unchanged/stale. Remote frames are rejected by the global filter, so they do not enter FIFO0/FIFO1. |
| T5 wrong DLC | `B0`; standard data ID `0x067`, DLC `0..4` or `6..8` | Within 30 ms, `can_bus=1`; every B0 mailbox field remains unchanged/stale. This proves activity is noted before the DLC-5 check, but timestamp publication is not performed. |
| T6 FIFO drain/order | `S`; enqueue in order valid `[00 00 50 00 00]`, invalid `[02 00 50 00 00]`, valid `[01 FF 78 09 03]` | Within 30 ms of the queue becoming non-empty, common accepted assertion `(120,777,1)`, `decision=4`, `pset_W=120`, `swen_req=1`. With the switch known off for at least 1 ms, `swen_out=1`; `reset=FF` causes no separate effect. |

### Payload validation matrix

Every rejected payload uses stale B0. Therefore `can_p=85`, `can_e=10`, `can_swen=0`, all CAN `valid=1`, all CAN `fresh=0`, `decision=2`, `pset_W=0`, `swen_req=0`, and `swen_out=0` must remain true; because the ID/type/DLC are correct, `can_bus=1` must appear within 30 ms. This both proves mailbox preservation and proves that the rejected frame did not refresh B0's timestamp.

| Case | Payload and precondition | Expected observed `T1` values |
|---|---|---|
| P1 enable off | `S`; `[00 00 50 00 00]` | Common accepted assertion `(80,0,0)`; exact `swen_out=0`. |
| P2 enable on | `S`; `[01 00 50 00 00]` | Common accepted assertion `(80,0,1)` and `swen_req=1`. With an allowed charge direction, `pset_W=60` and SWEN may turn on; with an allowed discharge direction, `pset_W=70` and SWEN may turn on. With zero current direction or the relevant Vcap lockout, `pset_W=80` and `swen_out=0`. |
| P3 invalid enable | stale `B0`; `[02 00 50 00 00]`, then `[FF 00 50 00 00]` | Rejected assertion above for both frames. Values `2..255` form one invalid class. |
| P4 ignored reset | `S`; `[00 00 50 00 00]`, `[00 01 50 00 00]`, `[00 FF 50 00 00]` | After each frame, common accepted assertion `(80,0,0)` and `swen_out=0`. No T1 field distinguishes the three reset-byte values; no fault, UVLO, timing, or mailbox reset occurs. |
| P5 power below range | stale `B0`; `[00 00 00 00 00]`, `[00 00 31 00 00]` | Rejected assertion above for `0 W` and `49 W`. |
| P6 power lower boundary | `S`; `[00 00 32 00 00]` | Common accepted assertion `(50,0,0)`; `decision=4`, `pset_W=50`, `swen_out=0`. |
| P7 representative power | `S`; `[00 00 55 00 00]` | Common accepted assertion `(85,0,0)`; `decision=4`, `pset_W=85`, `swen_out=0`. |
| P8 power upper boundary | `S`; `[00 00 78 00 00]` | Common accepted assertion `(120,0,0)`; `decision=4`, `pset_W=120`, `swen_out=0`. |
| P9 power above range | stale `B0`; `[00 00 79 00 00]`, `[00 00 FF 00 00]` | Rejected assertion above for `121 W` and `255 W`. |
| P10 energy lower boundary | `S`; `[00 00 50 00 00]` | Common accepted assertion `(80,0,0)`; `swen_out=0`. |
| P11 energy boundaries | `S`; `[00 00 50 14 00]`, `[00 00 50 37 00]`, `[00 00 50 3C 00]` | Respectively common accepted assertions `(80,20,0)`, `(80,55,0)`, `(80,60,0)`; each has `can_e_valid=1`, `can_e_disabled=0`, `swen_req=0`, `swen_out=0`. |
| P12 energy just above range | stale `B0`; `[00 00 50 3D 00]` | Rejected assertion above (`61 J`). |
| P13 energy-disabled sentinel | `S`; `[01 00 50 09 03]` | Exact accepted values: `can_p=80`, `can_e=777`, `can_e_valid=0`, `can_e_disabled=1`, `can_swen=1`, all CAN valid/fresh fields as defined above, `decision=4`, `pset_W=80`, `swen_req=1`. If SWEN has been off for at least 1 ms, `swen_out=1`. |
| P14 other high energy | stale `B0`; `[00 00 50 0A 03]`, `[00 00 50 FF FF]` | Rejected assertion above for `778 J` and `65535 J`. |
| P15 byte order | stale `B0`; `[00 00 50 03 09]` | Rejected assertion above: bytes decode little-endian as `0x0903=2307 J`, not 777 J. |

### End-to-end controller-effect tests

`decision=4` is only expected in external mode while both UART freshness fields are zero. `ScapIo_Resolve1kHz()` selects the source and applies the direction/Vcap permission, energy power adjustment, and SWEN timing, then the fast ADC ISR applies the published output request plus immediate safety overrides; allow two `T1` samples before treating an output as settled.

| Case | Preconditions and command | Expected observed `T1` values |
|---|---|---|
| C1 CAN selected | `S`; send valid `[00 00 55 0A 00]` every 50 ms | On every sample after the first: common accepted assertion `(85,10,0)`, `decision=4`, `pset_W=85`, `swen_req=0`, `swen_out=0`. There must be no sample with CAN freshness zero between refreshes. |
| C2 CAN timeout | `S`; send one `[01 00 50 09 03]`, then stop | Up to 300 ms: P13 assertion. After 310 ms: `can_p=80`, `can_e=777`, `can_swen=1`, CAN `valid=1`, all CAN `fresh=0`, `can_e_valid=0`, `can_e_disabled=1`, `decision=2`, `pset_W=0`, `swen_req=0`; `swen_out=0` after any outstanding 50 ms minimum-on interval. |
| C3 periodic refresh | `S`; send P13 at 50 ms intervals for 500 ms | Every post-settle T1 record has P13's exact mailbox fields, all CAN freshness fields `1`, `decision=4`, `pset_W=80`, `swen_req=1`, and (after the initial minimum-off interval) `swen_out=1`. |
| C4 UART precedence | Supply fresh valid UART power and energy; send P13 | CAN fields show P13 exactly, while `uart_p_fresh=1`, `uart_e_fresh=1`, `uart_swen_req=1`, and `decision=5`. `pset_W` uses UART power plus the applicable direction/energy adjustment, and `swen_req`/`swen_out` follow UART policy rather than CAN. After either UART freshness field reaches 0 while CAN remains fresh: `decision=4`, `pset_W=80`, `swen_req=1`. |
| C5 enable off | `S`; send `[00 00 50 09 03]` | Exact: `can_p=80`, `can_e=777`, `can_e_valid=0`, `can_e_disabled=1`, `can_swen=0`, all CAN valid/fresh fields `1`, `decision=4`, `pset_W=80`, `swen_req=0`, `swen_out=0`. |
| C6 sentinel enable on | `S`, switch off for at least 1 ms; send P13 | P13 exact assertion, including `swen_req=1`; `swen_out=1` in the next settled T1 record. |
| C7 charge adjustment | `S`; force `i_conv>0`, Vcap below the current runtime maximum, and no charge lockout. Send `[01 00 50 32 00]`, then `[01 00 50 33 00]` | At 50 J: common accepted `(80,50,1)`, `pset_W=60`, `swen_req=1`, and `swen_out=1` after the 1 ms off limit. At 51 J: common accepted `(80,51,1)`, `pset_W=90`, and SWEN remains requested/on. The energy boundary changes power, not enable permission. |
| C8 discharge adjustment | `S`; force `i_conv<0`, Vcap `>5.26 V`, and no discharge lockout. Send `[01 00 50 13 00]`, then `[01 00 50 14 00]` | At 19 J: common accepted `(80,19,1)`, `pset_W=70`, `swen_req=1`, and `swen_out=1` after minimum-off. At 20 J: common accepted `(80,20,1)`, `pset_W=100`, and SWEN remains requested/on. The energy boundary changes power, not enable permission. |
| C9 Vcap hysteresis | Charge profile: use 51 J, enter below the current runtime maximum, raise Vcap to that maximum, then lower it by 0.1 V. Discharge profile: use 19 J, enter above 20% of the current maximum, lower Vcap to that level, then raise it by 0.1 V. Initially these pairs are 26.3/26.2 V and 5.26/5.36 V. | In charge lockout, mailbox fields stay accepted/fresh, `decision=4`, `pset_W=80`, `swen_req=1`, and `swen_out=0`; after resume, `pset_W=90` and SWEN may turn on. In discharge lockout the corresponding values are `pset_W=80`/off, then `pset_W=70`/on after resume and timing. |
| C10 UVLO | `S`; keep P13 fresh, reduce Vbus to `<=10 V`, then restore to `>=11 V` | In UVLO: P13 mailbox fields remain fresh, `safe=1`, `uvlo=1`, `decision=1`, `pset_W=0`, `swen_req=0`, `swen_out=0`. After recovery and a fresh P13: `uvlo=0`, `decision=4`, `pset_W=80`, `swen_req=1`, `swen_out=1` after timing. |
| C11 over-voltage fault | `S`; keep P13 fresh, make Vbus or Vcap `>=30 V`, then restore both below 30 V for 500 ms | At fault: P13 mailbox fields may remain fresh, `safe=0`, `fault_latched=1`, `fault_bits=1` (Vbus), `2` (Vcap), or `3`, `decision=0`, `pset_W=0`, `swen_req=0`, `swen_out=0`. Reset-byte values do not change this. After safe time: `safe=1`, eventually `fault_latched=0`, `fault_bits=0`; with P13 still fresh, CAN resolution resumes. |
| C12 SWEN limits | `S`; keep P13 fresh. From a known off state send enable `1`, then send enable `0`, then `1` again | `swen_req` follows each accepted command within 30 ms. `swen_out` stays on for at least 50 ms after the on transition despite `swen_req=0`; after it goes off, it cannot turn on for at least 1 ms. T1's 10 ms period can show the 50 ms rule but cannot prove a 1 ms edge: use a GPIO logic analyser for that threshold. |

### Acceptance criteria

- Every accepted case has the published mailbox values stated in its row; a rejected case retains stale B0 and never refreshes its CAN freshness fields.
- `can_bus` is a separate 200 ms diagnostic timer. FIFO0 traffic sets it in the receive callback; non-matching data frames reach FIFO1 and set it only when the 100 ms task poll drains FIFO1; remote frames are hardware-rejected.
- The resolver and output chain is intentional: CAN packet validation publishes the mailbox, the 1 kHz resolver selects UART before CAN and then applies freshness, energy/Vcap policy, and SWEN timing; the ADC ISR applies that final output request and immediate safety overrides. A valid packet alone is never sufficient to require `swen_out=1`.
