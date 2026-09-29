"""Run host behavior checks against the actual logging implementation (MSVC on Windows)."""
from pathlib import Path
import argparse
import re
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--disabled', action='store_true')
parser.add_argument('--state-only', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
build = root / ".pio" / "logging-host-tests"
build.mkdir(parents=True, exist_ok=True)
shutil.copy(root / "include/firmwareConfig.h", build / "firmwareConfig.h")
shutil.copy(root / "src/communication/dataLogging.cpp", build / "dataLogging.cpp")
shutil.copy(tests / "mocks.h", build / "mocks.h")
shutil.copy(tests / ("state_only.cpp" if args.state_only else "disabled.cpp" if args.disabled else "test.cpp"), build / "test.cpp")
for path in ['include/states.h', 'include/dataLogging.h', 'src/core/states.cpp']:
    shutil.copy(root / path, build / Path(path).name)
globals_text = (root / "include/globals.h").read_text(encoding="utf-8-sig")
enums = "\n".join(re.findall(r"enum Sensor\s*\{.*?\};", globals_text, re.S))
(build / "globals.h").write_text('#pragma once\n#include "mocks.h"\n#include "states.h"\n' + enums + '\nextern volatile bool irSensor[7];\n')
(build / "bluetoothComm.h").write_text('#include "mocks.h"\nvoid sendData(const String&);\n')
(build / "IMU.h").write_text('#include "mocks.h"\n')
vswhere = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe")
vs = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-property", "installationPath"], text=True).strip()
vcvars = Path(vs) / "VC/Auxiliary/Build/vcvars64.bat"
flags = '' if (args.disabled or args.state_only) else '/DENABLE_LINE_SENSORS=1 /DENABLE_IR_SENSORS=1 /DENABLE_GYRO=1'
service_flags = '' if args.state_only else '/DENABLE_LOGGING=1 /DENABLE_SERIAL=1'
command = f'call "{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /utf-8 /DMBARETECH_2 {service_flags} {flags} test.cpp /Fe:logging-tests.exe && logging-tests.exe'
(build / "run-tests.cmd").write_text("@echo off\n" + command + "\n")
subprocess.run(["cmd.exe", "/d", "/c", "run-tests.cmd"], cwd=build, check=True)

