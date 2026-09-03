@echo off
setlocal

set "DASHBOARD_EXE=%~dp0SCV2 Dashboard.exe"
set "DASHBOARD_PY=%~dp0.venv\Scripts\python.exe"
set "DASHBOARD_GUIDE=%~dp0DASHBOARD_SETUP.md"

if exist "%DASHBOARD_EXE%" (
  "%DASHBOARD_EXE%" %*
  exit /b %ERRORLEVEL%
)

if not exist "%DASHBOARD_PY%" (
  echo SCV2 Dashboard.exe and the dashboard Python environment were not found.
  echo.
  echo Follow the setup guide:
  echo   "%DASHBOARD_GUIDE%"
  echo.
  pause
  exit /b 1
)

"%DASHBOARD_PY%" "%~dp0tools\scv2_dashboard.py" %*
set "DASHBOARD_EXIT=%ERRORLEVEL%"

if not "%DASHBOARD_EXIT%"=="0" (
  echo.
  echo The dashboard exited with error code %DASHBOARD_EXIT%.
  echo Check the setup and troubleshooting guide:
  echo   "%DASHBOARD_GUIDE%"
  echo.
  pause
)

exit /b %DASHBOARD_EXIT%
