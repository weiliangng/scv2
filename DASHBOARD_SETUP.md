# SCV2 dashboard setup and operation

For normal use, copy `SCV2 Dashboard.exe` to any Windows 10 or 11 PC and run it directly. It bundles the dashboard, Python, Qt, and pyserial; Python, Qt, CMake, and an internet connection are not required on the target PC. Windows may show a SmartScreen warning for an unsigned in-house executable; use the normal organisation-approved review path before choosing **More info** then **Run anyway**.

The remaining sections describe the Python development setup and how to rebuild the executable after changing the dashboard.

## Development requirements

- Windows 10 or 11
- 64-bit Python 3.10 through 3.14; Python 3.14 is recommended
- An internet connection for the initial package download
- A current SCV2 firmware build that emits the 69-field `T1` telemetry record

The dashboard depends on PySide6 and pyserial. PySide6's pip packages include the required Qt runtime, so a separate Qt SDK, C compiler, and CMake installation are not required. See the official [Python on Windows](https://docs.python.org/3/using/windows.html) and [Qt for Python getting-started](https://doc.qt.io/qtforpython-6/gettingstarted.html) documentation for the underlying installation mechanisms.

## 1. Install Python on a fresh PC

Install the Python Install Manager from the Microsoft Store or [python.org](https://www.python.org/downloads/). Close and reopen PowerShell after installation, then install and verify Python 3.14:

```powershell
py install 3.14
py -V:3.14 --version
```

If `py` is not recognized but `python` works, use this instead:

```powershell
python --version
```

The reported version must be Python 3.10 through 3.14. Do not use the embeddable Python ZIP: it does not include the normal `pip` and virtual-environment setup needed here.

## 2. Open PowerShell in the repository root

The repository root is the folder containing `Start SCV2 Dashboard.cmd`, `tools`, and this guide. In File Explorer, open that folder, click the address bar, type `powershell`, and press Enter. Alternatively:

```powershell
cd "C:\path\to\scv2"
```

Run all remaining setup commands from this folder.

## 3. Create the project virtual environment

With the Python Install Manager:

```powershell
py -V:3.14 -m venv .venv
```

If the PC uses a conventional Python installation without the `py` command:

```powershell
python -m venv .venv
```

There is no need to activate the environment. Every command below invokes its Python executable directly, which also avoids PowerShell execution-policy problems with `Activate.ps1`.

## 4. Install the dashboard libraries

```powershell
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r .\tools\requirements-dashboard.txt
```

The PySide6 download is relatively large and may take a few minutes. Always use `.venv\Scripts\python.exe -m pip`; running a plain `pip install` may install packages into a different Python interpreter.

## 5. Verify the installation

Check that both libraries import and then run the dashboard's built-in parser test:

```powershell
.\.venv\Scripts\python.exe -c "import PySide6, serial; print('PySide6', PySide6.__version__, 'pyserial', serial.VERSION)"
.\.venv\Scripts\python.exe .\tools\scv2_dashboard.py --self-test
```

The self-test should end with:

```text
T1 parser, UDP stream buffering, and state-display self-test passed.
```

To inspect the UI without hardware:

```powershell
.\.venv\Scripts\python.exe .\tools\scv2_dashboard.py --demo
```

Close the demo window before connecting hardware.

## 6. Start the dashboard from source

After the one-time setup, either double-click `Start SCV2 Dashboard.cmd` or run:

```powershell
& ".\Start SCV2 Dashboard.cmd"
```

The direct Python form is also available:

```powershell
.\.venv\Scripts\python.exe .\tools\scv2_dashboard.py
```

Command-line options preselect settings but do not press the UI's **Connect** button automatically:

```powershell
# Preselect a serial port
.\.venv\Scripts\python.exe .\tools\scv2_dashboard.py --port COM8 --baud 115200

# Preselect the UDP listener
.\.venv\Scripts\python.exe .\tools\scv2_dashboard.py --transport udp --udp-port 14551
```

## 7. Connect a telemetry source

### Board USB CDC

1. Connect the SCV2 board with a USB data cable and wait for its COM port to appear.
2. Leave **Source** set to **USB serial** and click **Refresh ports** if necessary.
3. Select the board's COM port. The displayed baud rate does not affect USB CDC; the default `115200` is fine.
4. Leave **Enable USB telemetry while connected** checked.
5. Click **Connect**. The dashboard sends `telemetry on` and sends `telemetry off` when it disconnects.

### External USB-to-UART receiver

The firmware continuously transmits the same T1 stream from USART1 TX on PA9 at 921600 baud.

1. Use a 3.3 V logic-level USB-to-UART adapter. Connect adapter RX to PA9 and connect the grounds. Do not drive PA9 and do not apply 5 V logic.
2. Select **USB serial**, the adapter's COM port, and baud `921600`.
3. Uncheck **Enable USB telemetry while connected**. USART1 already streams continuously and does not accept the USB CLI command.
4. Click **Connect**.

### ESP32 UDP bridge

1. Put the PC and bridge on the required network and configure the bridge to forward raw UART1 telemetry to UDP port `14551` on the PC.
2. Select **UDP listener**, leave **UDP port** at `14551`, and click **Connect**.
3. Allow Python through Windows Firewall for the appropriate network profile if prompted.

The dashboard only binds and listens. It does not discover the bridge, send a reply, or require UDP datagrams to align with T1 record boundaries.

### USB CLI commands from the dashboard

The **USB CLI** tab is available only while connected to the board's USB CDC port with **Enable USB telemetry while connected** selected. Enter one command, such as `status`, and click **Send command**. The dashboard pauses its USB telemetry mirror with `telemetry off`, waits for the CLI prompt, sends the command, waits for the command to finish at the next prompt, then restores telemetry with `telemetry on`. The response is displayed in the tab.

It is intentionally unavailable for the UDP listener and for the PA9 external USB-UART receiver. Treat dashboard commands exactly like commands typed in a serial terminal: `ctrl direct`, `gpio`, `dac`, and calibration commands can mutate hardware, so use a safe, current-limited bench setup and follow the constraints in `agent.md`.

## 8. Build or update the portable executable

After changing `tools/scv2_dashboard.py`, rebuild the executable before committing or distributing the change. From the repository root, run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build-dashboard-exe.ps1
```

The script installs PyInstaller in `.venv` if it is missing, packages the dashboard as a single `SCV2 Dashboard.exe` in the repository root, and runs its built-in `--self-test`. Subsequent rebuilds do not need an internet connection. The executable is the only file needed by the target PC. Build it on 64-bit Windows for 64-bit Windows targets; PyInstaller does not cross-compile Windows executables.

The script deliberately replaces the root executable only after PyInstaller succeeds. Its intermediate directories under `tools` are ignored by Git. Review and commit the changed Python source, documentation, and `SCV2 Dashboard.exe` together so the distributed binary matches its source.

## Troubleshooting

### `python` or `py` is not recognized

- Close and reopen PowerShell after installing the Python Install Manager.
- Try `pymanager` if another program has claimed the `py` command.
- Reopen the official Python installer and repair its command aliases if neither command works.

### `.venv\Scripts\python.exe` does not exist

PowerShell is either in the wrong folder or step 3 did not complete. Confirm that the current folder contains `Start SCV2 Dashboard.cmd`, then recreate `.venv` using the command in step 3.

### `No module named PySide6` or `No module named serial`

Install the requirements with the exact `.venv\Scripts\python.exe -m pip` command from step 4. Do not substitute a global `pip` command.

### `No matching distribution found for PySide6`

Check the interpreter version:

```powershell
.\.venv\Scripts\python.exe --version
```

Use a normal 64-bit Python release from 3.10 through 3.14. Delete or rename the incomplete `.venv`, create it with Python 3.14, and reinstall the requirements.

### The COM port is missing or access is denied

- Confirm the cable supports data, not charging only.
- Check Windows Device Manager for the board or USB-to-UART adapter.
- Close serial terminals, CLion monitors, and other programs that may already own the COM port.
- Disconnect and reconnect the device, then click **Refresh ports**.

### Packets arrive but no live T1 values appear

- Flash firmware using the current 69-field T1 schema; the dashboard deliberately rejects older 68-field records.
- For USB CDC, keep **Enable USB telemetry while connected** checked.
- For the PA9 external UART, use `921600` baud and uncheck that box.
- Use the dashboard connection message to inspect malformed-record errors.

### UDP receives nothing

- Confirm the sender targets this PC's IP address and UDP port `14551`.
- Allow Python through Windows Firewall.
- Ensure no other application is already bound to port `14551`.
- Confirm the PC is on the network expected by the bridge.

### Installation must be performed offline

On an internet-connected Windows PC using the same Python version and CPU architecture, download the wheels:

```powershell
py -V:3.14 -m pip download -r .\tools\requirements-dashboard.txt -d .\dashboard-wheels
```

Copy `dashboard-wheels` with the repository to the offline PC. After creating `.venv`, install only from that folder:

```powershell
.\.venv\Scripts\python.exe -m pip install --no-index --find-links .\dashboard-wheels -r .\tools\requirements-dashboard.txt
```
