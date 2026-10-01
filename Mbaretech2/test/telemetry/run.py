"""Compile and run the portable canonical telemetry serializer with MSVC."""
from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--compile-only', action='store_true')
args = parser.parse_args()

root = Path(__file__).resolve().parents[2]
build = root / '.pio' / 'telemetry-host-tests'
build.mkdir(parents=True, exist_ok=True)
vswhere = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe')
vs = subprocess.check_output(
    [str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'],
    text=True).strip()
vcvars = Path(vs) / 'VC/Auxiliary/Build/vcvars64.bat'
test = Path(__file__).with_name('protocol.cpp')
protocol = root / 'src/communication/telemetryProtocol.cpp'
parameters = root / 'src/control/runtimeParameters.cpp'
commands = [
    '@echo off',
    f'call "{vcvars}" >nul',
    'if errorlevel 1 exit /b %errorlevel%',
    f'cl /nologo /std:c++17 /EHsc /W4 /utf-8 /I"{root / "include"}" '
    f'"{test}" "{protocol}" "{parameters}" /Fe:telemetry-protocol.exe',
    'if errorlevel 1 exit /b %errorlevel%',
]
if not args.compile_only:
    commands += ['telemetry-protocol.exe', 'if errorlevel 1 exit /b %errorlevel%']
(build / 'run.cmd').write_text('\n'.join(commands) + '\n')
subprocess.run(['cmd.exe', '/d', '/c', 'run.cmd'], cwd=build, check=True)
