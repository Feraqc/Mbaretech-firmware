# Mbaretech2

Firmware de un robot de sumo autónomo para ESP32-S3. Dos motores impulsan el robot, siete sensores IR localizan al rival y dos sensores delanteros detectan el borde de la pista. El programa usa Arduino, FreeRTOS y PlatformIO.

La configuración se edita en `include/buildConfig.h`. Actualmente selecciona `MBARETECH_2` y el diagnóstico aislado de gyro por Serial, sin motores. `platformio.ini` contiene un único build; no se seleccionan entornos. Para combate, recetas y sensores, consultar [firmware.md](docs/firmware.md).

## Primeros pasos

1. Instalar PlatformIO y la plataforma ESP32.
2. Abrir una terminal en `Mbaretech2`.
3. Revisar `include/buildConfig.h` y compilar con `pio run`.
4. Conectar el ESP32-S3 y cargar con `pio run -t upload`.
5. Para leer mensajes serie, habilitar `ENABLE_SERIAL=1` en `include/buildConfig.h`, volver a compilar y usar `pio device monitor -b 115200`.

El repositorio no fija el puerto de carga. PlatformIO debe detectarlo o se debe indicar según el equipo. Se verificó la compilación del build unificado; no se realizó una carga física.

## Documentación

- [Contexto y flujo de control](CONTEXT.md): recorrido desde el arranque hasta los estados de combate.
- [Firmware, clases y funciones](docs/firmware.md): responsabilidades, interfaces y datos compartidos.
- [Hardware](docs/hardware.md): señales, GPIO y periféricos del robot 2.
- [Desarrollo y pruebas](docs/desarrollo.md): configuración, diagnósticos, límites y resolución de problemas.

Para orientarse en el código: `src/main.cpp` prepara el hardware; `src/control/tasks.cpp` toma decisiones; `include/motor.h` acciona los motores; `src/sensors/lineSensor.cpp` lee el borde; `include/globals.h` define pines, estados y constantes.
