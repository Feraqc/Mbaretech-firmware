"""Test recipe progression and production Drive/Motor hardware gating (MSVC)."""
from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--compile-only', action='store_true')
parser.add_argument('--recipe', choices=['all', 'motor-test', 'turn-calibration'], default='all')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
build = root / '.pio/recipe-host-tests'
build.mkdir(parents=True, exist_ok=True)
vswhere = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe')
vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'], text=True).strip()
vcvars = Path(vs) / 'VC/Auxiliary/Build/vcvars64.bat'
sources = [tests / 'test.cpp'] + [root / 'src/control' / name for name in [
    'recipeDrive.cpp', 'evaluateCondition.cpp', 'recipeState.cpp',
    'recipeStateMachine.cpp', 'recipeValidation.cpp', 'recipeLogFormat.cpp',
    'runtimeParameters.cpp']]
source_args = ' '.join(f'"{source}"' for source in sources)
flags = '/DENABLE_RECIPE_FSM=1 /DENABLE_SENSOR_TASK=1 /DENABLE_SERIAL=1'
commands = ['@echo off', f'call "{vcvars}" >nul', 'if errorlevel 1 exit /b %errorlevel%']
recipes = ['MOTOR_TEST', 'TURN_CALIBRATION'] if args.recipe == 'all' else [args.recipe.upper().replace('-', '_')]
recipes = ['TEST' if recipe == 'MOTOR_TEST' else recipe for recipe in recipes]
for force_start in (0, 1):
    for compact in (0, 1):
        for recipe in recipes:
            executable = f'recipe-{recipe}-start{force_start}-compact{compact}.exe'
            commands += [
                f'cl /nologo /std:c++17 /EHsc /W4 /utf-8 /I"{tests / "mocks"}" /I"{root / "include"}" '
                f'{flags} /DFSM_CONSOLE_COMPACT={compact} /DFORCE_START_ACTIVE={force_start} /DFSM_ACTIVE_RECIPE_{recipe} {source_args} /Fe:{executable}',
                'if errorlevel 1 exit /b %errorlevel%']
            if not args.compile_only:
                commands += [executable, 'if errorlevel 1 exit /b %errorlevel%']
    for motors in (0, 1):
        # Compile the real Motor header with only Arduino/LEDC boundary stubs.
        executable = f'drive-hardware-motors{motors}-start{force_start}.exe'
        commands += [
            f'cl /nologo /std:c++20 /EHsc /W4 /utf-8 /I"{tests / "hardware"}" /I"{root / "include"}" '
            f'{flags} /DENABLE_MOTORS={motors} /DFORCE_START_ACTIVE={force_start} '
            f'"{tests / "driveHardware.cpp"}" "{root / "src/control/recipeDrive.cpp"}" /Fe:{executable}',
            'if errorlevel 1 exit /b %errorlevel%']
        if not args.compile_only:
            commands += [executable, 'if errorlevel 1 exit /b %errorlevel%']
(build / 'run.cmd').write_text('\n'.join(commands) + '\n')
subprocess.run(['cmd.exe', '/d', '/c', 'run.cmd'], cwd=build, check=True)
