@echo off
setlocal
rem Resolve the editor from this script's directory so it works from any folder.
rem Node is preferred because PlatformIO's Python virtual environment can outlive
rem its base interpreter and fail even though python.exe still exists.
where node >nul 2>&1
if not errorlevel 1 (
  node -e "process.exit(Number(process.versions.node.split('.')[0]) >= 16 ? 0 : 1)" >nul 2>&1
  if not errorlevel 1 goto run_node
)

rem Keep the Python server available when Node is not installed.
where py >nul 2>&1
if not errorlevel 1 (
  py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
  if not errorlevel 1 goto run_py
)
where python >nul 2>&1
if not errorlevel 1 (
  python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
  if not errorlevel 1 goto run_python
)
set "EDITOR_PYTHON=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
if exist "%EDITOR_PYTHON%" (
  "%EDITOR_PYTHON%" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
  if not errorlevel 1 goto run_platformio_python
)

rem Every supported Windows installation includes PowerShell and .NET.
where powershell.exe >nul 2>&1
if not errorlevel 1 goto run_powershell

echo No se encontro Node.js, Python 3.10+ ni Windows PowerShell.
goto done

:run_node
node "%~dp0tools\recipe-editor\serve.cjs" %*
goto done

:run_py
py -3 "%~dp0tools\recipe-editor\serve.py" %*
goto done

:run_python
python "%~dp0tools\recipe-editor\serve.py" %*
goto done

:run_platformio_python
"%EDITOR_PYTHON%" "%~dp0tools\recipe-editor\serve.py" %*
goto done

:run_powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\recipe-editor\serve.ps1" %*

:done
pause
