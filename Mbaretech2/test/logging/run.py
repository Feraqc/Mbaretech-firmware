"""Run host behavior checks against the actual logging implementation (MSVC on Windows)."""
from pathlib import Path
import re
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
build = root / ".pio" / "logging-host-tests"
build.mkdir(parents=True, exist_ok=True)
shutil.copy(root / "src/dataLogging.cpp", build / "dataLogging.cpp")
shutil.copy(tests / "mocks.h", build / "mocks.h")
shutil.copy(tests / "test.cpp", build / "test.cpp")
globals_text = (root / "include/globals.h").read_text(encoding="utf-8-sig")
enums = "\n".join(re.findall(r"enum (?:Sensor|State)\s*\{.*?\};", globals_text, re.S))
header = (root / "include/dataLogging.h").read_text(encoding="utf-8-sig")
header = header.replace('#include "globals.h"', '#include "mocks.h"\n' + enums +
                        '\nextern volatile State currentState;\nextern volatile bool irSensor[7];\nvoid changeState(State);')
(build / "dataLogging.h").write_text(header, encoding="utf-8")
(build / "bluetoothComm.h").write_text('#include "mocks.h"\nvoid sendData(const String&);\n')
(build / "IMU.h").write_text('#include "mocks.h"\n')
vswhere = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe")
vs = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-property", "installationPath"], text=True).strip()
vcvars = Path(vs) / "VC/Auxiliary/Build/vcvars64.bat"
command = f'call "{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /utf-8 /DMBARETECH_2 test.cpp /Fe:logging-tests.exe && logging-tests.exe'
(build / "run-tests.cmd").write_text("@echo off\n" + command + "\n")
subprocess.run(["cmd.exe", "/d", "/c", "run-tests.cmd"], cwd=build, check=True)

