"""Run the real nonblocking controller on synthetic snapshots (MSVC on Windows)."""
from pathlib import Path
import argparse
import re
import shutil
import subprocess
parser = argparse.ArgumentParser()
parser.add_argument('--compile-only', action='store_true')
parser.add_argument('--suite', choices=['all', 'fsm', 'sensors'], default='all')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
build = root / '.pio/control-host-tests'
build.mkdir(parents=True, exist_ok=True)
for path in ['include/firmwareConfig.h', 'include/states.h', 'include/combatFsm.h', 'include/sensorTasks.h', 'include/sensorSnapshot.h', 'src/control/combatFsm.cpp', 'src/sensors/sensorTasks.cpp']:
    shutil.copy(root / path, build / Path(path).name)
for name in ['test.cpp', 'sensorMocks.h', 'sensors.cpp']:
    shutil.copy(tests / name, build / name)
globals_text = (root / 'include/globals.h').read_text(encoding='utf-8-sig')
constants = globals_text[globals_text.index('#ifdef MBARETECH_2'):globals_text.index('extern TickType_t')]
enums = '\n'.join(re.findall(r'enum (?:Sensor|State)\s*\{.*?\};', globals_text, re.S))
(build / 'globals.h').write_text('#pragma once\n#include <cstdint>\nusing TickType_t = uint32_t;\n#include "states.h"\n' + constants + enums + '\n#ifdef SENSOR_HOST_TEST\nconstexpr int ADC1_CHANNEL_2=2, ADC1_CHANNEL_7=7;\n#include "sensorMocks.h"\n#endif\n')
vswhere = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe')
vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'], text=True).strip()
vcvars = Path(vs) / 'VC/Auxiliary/Build/vcvars64.bat'
commands = [f'@echo off\ncall "{vcvars}" >nul']
enabled = '/DENABLE_LINE_SENSORS=1 /DENABLE_IR_SENSORS=1 /DENABLE_DIP_SWITCHES=1'
for board in ['MBARETECH_1', 'MBARETECH_2']:
    if args.suite in ['all', 'fsm']:
        commands += [f'cl /nologo /std:c++17 /EHsc /utf-8 /DCOMBAT_FSM_HOST_TEST /D{board} {enabled} /DENABLE_TURN_CANCEL=1 test.cpp combatFsm.cpp /Fe:{board}-tests.exe',
                     'if errorlevel 1 exit /b %errorlevel%', f'{board}-tests.exe', 'if errorlevel 1 exit /b %errorlevel%']
    if args.suite in ['all', 'sensors']:
        for name, flags in [('all', enabled), ('none', ''), ('line', '/DENABLE_LINE_SENSORS=1'),
                            ('ir', '/DENABLE_IR_SENSORS=1'), ('dip', '/DENABLE_DIP_SWITCHES=1'),
                            ('start', '/DENABLE_RECIPE_FSM=1 /DENABLE_SERIAL=1')]:
            exe = f'{board}-sensors-{name}.exe'
            commands += [f'cl /nologo /std:c++17 /EHsc /utf-8 /DSENSOR_HOST_TEST /DENABLE_SENSOR_TASK=1 /D{board} {flags} sensors.cpp /Fe:{exe}',
                         'if errorlevel 1 exit /b %errorlevel%', exe, 'if errorlevel 1 exit /b %errorlevel%']
if args.compile_only:
    commands = [line for line in commands if not re.fullmatch(r'MBARETECH_[12]-(?:tests|sensors-\w+)\.exe', line)]
(build / 'run.cmd').write_text('\n'.join(commands) + '\n')
subprocess.run(['cmd.exe', '/d', '/c', 'run.cmd'], cwd=build, check=True)
