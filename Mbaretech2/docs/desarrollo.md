# Desarrollo, pruebas y diagnóstico

## Selección del programa

Editar `include/buildConfig.h` para seleccionar programa, placa y switches `ENABLE_*=0/1`. `firmwareConfig.h` valida dependencias. `platformio.ini` tiene un único entorno técnico; no se elige programa con `-e` ni se agregan entornos. La configuración guardada es gyro aislado por Serial.

Desde la raíz usar siempre `pio run -d Mbaretech2`; dentro del proyecto, `pio run`. La tabla de combinaciones de combate, sensores y recetas está en [firmware.md](firmware.md). Los flags anteriores `RUN_*`, `DEBUG`, `OLD`, `FORWARDON`, `CANCEL_TURNS` y `ESTADOS_ORDEN` siguen rechazándose.

Los modos de diagnóstico que definen un programa exclusivo no se combinan con combate. La tarea de sensores sí se combina con la FSM; no se combina con los diagnósticos antiguos que leen directamente ADC/GPIO.

## Diagnósticos adicionales

En `include/buildConfig.h`, dejar otros programas exclusivos en 0 y habilitar los switches siguientes:

- Motor: `ENABLE_MOTOR_TEST=1`, `ENABLE_MOTORS=1`, `ENABLE_SERIAL=1`. `setup()` inicializa los motores y el diagnóstico alterna atrás/freno/adelante.
- Línea: `ENABLE_LINE_TEST=1`, `ENABLE_LINE_SENSORS=1`, `ENABLE_SERIAL=1`. Lee canales delanteros y traseros. La configuración ADC está habilitada por el mismo flag; verificar físicamente los canales disponibles.
- Movimientos: `ENABLE_MOVEMENT_TEST=1`, `ENABLE_LINE_SENSORS=1`, `ENABLE_IR_SENSORS=1`, `ENABLE_DIP_SWITCHES=1`, `ENABLE_SERIAL=1`. Para producir salida física agregar `ENABLE_MOTORS=1`; para la implementación antigua agregar `ENABLE_LEGACY_MOVEMENTS=1`. Sus maniobras todavía contienen esperas.
- Verbosidad adicional: `ENABLE_DEBUG=1` con `ENABLE_SERIAL=1`.
- Registro opcional en los modos compatibles: `ENABLE_LOGGING=1` y al menos `ENABLE_SERIAL=1` o `ENABLE_BLE=1`.

Para registro de gyro habilitar `ENABLE_GYRO=1`, seleccionar opción 4 del menú y enviar 5. Mantener el robot inmóvil durante la calibración. El diagnóstico aislado `ENABLE_GYRO_TEST=1` no admite la tarea de registro, porque ambos serían propietarios de la misma IMU.

## Estados y ajustes

`include/states.h` define una sola lista de estados, sus IDs y nombres. `changeState()` en `src/core/states.cpp` publica los cambios y llama al registro opcional. Combate y ambas tareas de movimientos comparten esa interfaz. Sus algoritmos de maniobra siguen separados.

`include/globals.h` contiene umbrales, velocidades y tiempos por placa. `src/control/combatFsm.cpp` contiene fases no bloqueantes y constantes específicas; sus duraciones son milisegundos. El temporizador antiguo `elapsedTime()` recibe ticks y solo se utiliza en las pruebas antiguas. Los giros no tienen corrección por encoder o IMU.

Los comandos `ÍNDICE VALOR` de Serial/BLE modifican `parametros[0..18]`, utilizados principalmente por movimientos de prueba. No sustituyen las constantes del controlador de combate.

## Verificación

- Compilar cada configuración afectada con el mismo comando; una compilación no verifica temporización ni hardware.
- `python Mbaretech2/test/control/run.py --suite fsm`: decisiones, aperturas, interrupciones, fases y temporización simulada.
- `python Mbaretech2/test/control/run.py --suite sensors`: adquisición y grupos deshabilitados con hardware simulado.
- `python Mbaretech2/test/control/run.py --compile-only`: compila ambas suites sin ejecutar binarios.
- `python Mbaretech2/test/logging/run.py`: menú, transiciones y registro.
- `python Mbaretech2/test/logging/run.py --disabled`: canales deshabilitados.
- `python Mbaretech2/test/logging/run.py --state-only`: catálogo compartido y cambios de estado sin logger.
- `python Mbaretech2/test/hardware/run.py --motors-disabled`: ausencia de escrituras GPIO/PWM al deshabilitar motores.

Las pruebas host requieren Python y MSVC Build Tools. Device Guard puede bloquear ejecución aunque la compilación termine. En placa, comprobar polaridad, dirección de ruedas, detección de borde y parada antes de probar maniobras completas. `ENABLE_TASK_TIMING=1` permite observar duración, separación de ciclos y overruns; el procedimiento se describe en [control.md](control.md).

## Problemas frecuentes

- Sin Serial: verificar `ENABLE_SERIAL=1`, 115200 baudios y el programa seleccionado. Para el menú se requiere `ENABLE_LOGGING=1`; DEBUG no sustituye ninguno.
- Sin BLE: habilitar `ENABLE_BLE=1` y `ENABLE_LOGGING=1`.
- FSM sin movimiento: verificar `ENABLE_MOTORS=1`, START, datos válidos/recientes y apertura DIP o `FSM_DEFAULT_OPENING`.
- Bordes detectados demasiado pronto/tarde: verificar ADC y umbral; siete lecturas consecutivas introducen una latencia dependiente del período de adquisición.
- Cambio BLE sin efecto en combate: verificar si la maniobra usa `parametros` o constantes.
