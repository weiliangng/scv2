# Legacy SCV2 website content

This file preserves the content of the original hand-written SCV2 website before the Astro and Starlight migration. It is a knowledge-base snapshot, not a published page. The source was the original documentation site introduced in commit `1dc9715`, including the later 97% peak-efficiency specification from commit `180e360`.

## Home

# SCV2 supercapacitor controller

SCV2 manages a physical energy buffer for power-limited robot chassis motors.

## Why an energy buffer is needed

The referee system limits chassis motor power from 50 W to 120 W across robot levels 1 to 10. It also provides a 60 J virtual power buffer for short periods above that limit.

When chassis power exceeds the limit, the virtual buffer drains by the excess power. For example, 60 W above the limit consumes 60 J in one second. When chassis power is below the limit, the unused allowance refills the buffer.

If a level 1 robot stops moving, the full 50 W allowance is available. The virtual buffer will refill in:

`60 J / 50 W = 1.2 s`

A depleted buffer can trigger a competition penalty which used to be HP loss, but now results in a temporary chassis shutdown for 5 seconds.

Figure: Chassis power fluctuates above and below the referee limit during a match (`chassis.png`).

## The physical buffer

SCV2 is a bidirectional switching converter that works with a supercapacitor bank of up to 2000 J. It is intended to behave like a larger version of the referee system's virtual buffer.

Chassis power is measured from voltage and current. When chassis power is below the referee limit, SCV2 uses the surplus to charge the capacitor bank. When chassis power exceeds the limit, the bank supplies the difference.

The converter supports constant input-power regulation and CC/CV charging. After conversion losses and operating limits, the expected usable energy is about 1200 to 1400 J. This supports short loads such as acceleration, ramps, and slopes without depleting the referee buffer as quickly.

## Specifications

**Port convention:** IN is the chassis-bus side and OUT is the supercapacitor side. Since power flow is bidirectional, each port can source or sink power.

| Port | Parameter | Range or limit |
|---|---|---|
| IN — chassis bus | Voltage range | 12 to 28 V |
| IN — chassis bus | Current limit | -10 to +10 A; programmable in debug mode |
| IN — chassis bus | Power setting | -240 to +240 W; programmable in 1 W steps |
| IN — chassis bus | Reverse-flow voltage limit | 26.2 V if the battery is disconnected or shuts down while reverse power is set |
| OUT — supercapacitor | Voltage range and setting | 5 to 30 V; digitally programmable |
| OUT — supercapacitor | Current limit | -15 to +15 A; fixed limits |
| Response | Chassis load-step settling time | <1 ms; 150 µs typical |
| Performance | Peak efficiency | Up to 97% |

**Power sign:** positive power flows from IN to OUT and charges the bank. Negative power flows from OUT to IN and supports the chassis bus.

Figures:

- Power transfer envelope at a 24 V chassis bus (`power-envelope-24v.png`).
- Maximum transfer magnitude across the specified voltage range (`maximum-power-transfer.png`).

## Protection and fault handling

> The hardware is designed to tolerate these events, but they are not recommended operating procedures.

- Hot-plugging the power source
- Hot-plugging a charged or discharged supercapacitor bank
- Short-circuit current limiting
- Overvoltage detection and shutdown on both ports
- IN-port undervoltage lockout
- Capacitor degradation detection with charge-voltage foldback when added energy does not produce the expected voltage rise

## Features

1. Robot-independent UART and CAN interfaces
2. USB CLI for field debugging and manual bank-capacity testing
3. Telemetry dashboard over wired USB serial or Wi-Fi UDP; Wi-Fi requires an additional transmitter
4. Normal installation does not require individual board calibration after initial flashing and soldering
5. No unintended backfeeding or "zombie power"; no isolation relay required

## Documentation links

- Physical installation
- Robot interface
- Debugging and testing
- FAQ

---

## Physical installation

> Confirm mounting, connector, fuse, and wiring details against the released hardware documents before applying power.

### Competition checklist

1. Secure the controller and capacitor bank.
2. Prevent cable strain and contact with conductive objects.
3. Check power polarity and connector seating.
4. Check the fuse, isolation, and shutdown arrangement.
5. Connect CAN and referee UART as required.
6. Commission with an approved current-limited setup.
7. Check status before enabling the power stage.

### Hardware details

Mounting dimensions, connector pinouts, and photographs are not yet included in the repository.

---

## Robot interface

External mode uses a fresh referee UART command first, then a fresh CAN command. With no fresh source, the controller selects no-source state.

### Connections

| Interface | Configuration | Use |
|---|---|---|
| Referee UART | USART3, 115200, 8-N-1, receive | Power limit and buffer energy |
| CAN | Classic CAN, 1 Mbit/s | Commands and telemetry |
| Telemetry UART | USART1, 921600, 8-N-1, transmit | `T1` CSV stream every 10 ms |
| USB | CDC virtual COM port | CLI and optional telemetry |

### CAN telemetry

SCV2 sends an 8-byte frame using standard identifier `0x077`.

The original page linked to `CAN_TELEMETRY.md` for the receiver guide.

---

## Debugging

### Dashboard

The dashboard displays the 69-field `T1` telemetry record. It supports USB serial, external UART, and UDP input.

The original page linked to a dashboard setup guide that has since moved to
the separate [`scv2-dashboard` repository](https://github.com/weiliangng/scv2-dashboard).

### Bank capacity

Use `cap_energy_mJ`, `cap_dE_mJ_min`, `cap_dV_mV_min`, and the derate fields during a controlled test.

A capacity test procedure and pass criteria are not yet documented.

The original page linked to the telemetry field reference in `agent.md`.

### USB CLI

Start with read-only commands:

```text
telemetry off
help
status
```

The original page linked to `CLI_GUIDE.md` for the complete guide.

---

## FAQ

### Why is switch enable off after a command was accepted?

Check for faults, undervoltage lockout, stale inputs, capacitor-voltage lockout, direction, and switching timers. Command acceptance does not override protection.

### Can the dashboard and CLI share one USB port?

No. Close one program before connecting the other.

### Where is the connector pinout?

It is not yet published. Use the released schematic and connector document.
