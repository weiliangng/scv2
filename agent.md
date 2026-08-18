# USB telemetry `T1` decoder

`telemetry on` emits one CSV record every 10 ms. It does not alter control. Every record starts with `T1`; remaining columns are positional and must be decoded in the order below. It is a best-effort snapshot, so DMA, register, and shared-state values can be up to one control cycle apart.

## Conventions

- **Binary** is `0` or `1`.
- **Continuous** is a decimal integer in the stated unit.
- **N-ary** is a decimal enum defined below.
- Raw ADC and DAC values are unsigned 12-bit counts (`0..4095`).
- `valid` means a mailbox has published an accepted value. `fresh` means it is within its control timeout: CAN 100 ms, UART 300 ms.

## Columns

| # | Field | Type / range | Meaning and source |
|---:|---|---|---|
| 1 | `seq` | continuous, uint32 | USB telemetry record sequence. |
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
| 21 | `swen_out` | binary | Physical PB4 switch-enable after safety, energy, and min-on/off gating. |
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
| 41 | `can_p_fresh` | binary | CAN command is no more than 100 ms old. |
| 42 | `can_e` | continuous, J 0..60 or 777 | CAN energy; `777` explicitly disables energy gating. |
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

## Control precedence

In external mode, fresh UART power plus energy wins; otherwise fresh CAN is used; otherwise the controller has no source. In manual mode, the newer pushbutton or manual-SWEN mailbox wins. Faults and UVLO override normal operation. Use `swen_out` to verify what hardware actually received.

## Local dashboard

`tools/scv2_dashboard.py` is the read-only local viewer for this exact `T1` schema. It consumes only complete records with all 61 columns and ignores ordinary CLI output. Its field order is intentionally tied to this document.

- Continuous fields are shown as live current-value cards; the dashboard does not retain or graph historical samples. Invalid CAN, UART, and manual command values are greyed out rather than shown as zero.
- Validity and freshness fields are displayed as `VALID`/`INVALID` and `FRESH`/`STALE`; invalid or stale values are grey.
- Binary fields are semantic state cards such as `ON`/`OFF`, `SAFE`/`UNSAFE`, and `UP`/`DOWN`.
- N-ary resolver values are decoded: for example, `decision=5` is shown as `UART` and `decision=1` as `IDLE / UVLO`.

Start it by double-clicking `Start SCV2 Dashboard.cmd`, or run:

```powershell
.venv\Scripts\python.exe tools\scv2_dashboard.py --port COM8
```

The dashboard enables `telemetry on` when it connects and sends `telemetry off` before disconnecting. Use `--demo` to inspect the display without a board.
