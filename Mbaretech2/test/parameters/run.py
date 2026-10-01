"""Compila y ejecuta pruebas host del registro/decoder sin ESP32."""
from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--compile-only', action='store_true')
options = parser.parse_args()

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
out = root / '.pio' / 'parameter-host-tests'
out.mkdir(parents=True, exist_ok=True)
vswhere = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe')
visual_studio = subprocess.check_output(
    [str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'],
    text=True).strip()
vcvars = Path(visual_studio) / 'VC/Auxiliary/Build/vcvars64.bat'
sources = [tests / 'test.cpp', root / 'src/control/runtimeParameters.cpp',
           root / 'src/communication/parameterCommand.cpp',
           root / 'src/control/recipeDrive.cpp', root / 'src/control/evaluateCondition.cpp',
           root / 'src/control/recipeState.cpp', root / 'src/control/recipeStateMachine.cpp',
           root / 'src/control/recipeValidation.cpp']
source_args = ' '.join(f'"{source}"' for source in sources)
script = out / 'run.cmd'
script.write_text(
    f'@echo off\ncall "{vcvars}" >nul\n'
    f'cl /nologo /std:c++17 /EHsc /W4 /utf-8 /I"{root / "test/recipes/mocks"}" '
    f'/I"{root / "include"}" /DENABLE_RECIPE_FSM=1 /DENABLE_SENSOR_TASK=1 '
    f'/DENABLE_SERIAL=1 {source_args} /Fe:parameters.exe\n'
    'if errorlevel 1 exit /b %errorlevel%\n'
    + ('' if options.compile_only else 'parameters.exe\nif errorlevel 1 exit /b %errorlevel%\n'))
subprocess.run(['cmd.exe', '/d', '/c', 'run.cmd'], cwd=out, check=True)
