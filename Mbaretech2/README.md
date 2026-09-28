# Mbaretech2

Firmware de un robot de sumo autónomo para ESP32-S3. Dos motores impulsan el robot, siete sensores IR localizan al rival y dos sensores delanteros detectan el borde de la pista. El programa usa Arduino, FreeRTOS y PlatformIO.

La configuración actual de `platformio.ini` selecciona `MBARETECH_2` y la lógica de combate de `src/tasks.cpp`. Las pruebas de movimientos y de componentes existen como rutas alternativas; no son parte de esa compilación.

## Primeros pasos

1. Instalar PlatformIO y la plataforma ESP32.
2. Abrir una terminal en `Mbaretech2`.
3. Compilar con `pio run`.
4. Conectar el ESP32-S3 y cargar con `pio run -t upload`.
5. Para leer mensajes serie, activar `DEBUG` en `platformio.ini`, volver a compilar y usar `pio device monitor -b 115200`.

El repositorio no fija el puerto de carga. PlatformIO debe detectarlo o se debe indicar según el equipo. No se ha verificado aquí una compilación ni una carga física.

## Documentación

- [Contexto y flujo de control](CONTEXT.md): recorrido desde el arranque hasta los estados de combate.
- [Firmware, clases y funciones](docs/firmware.md): responsabilidades, interfaces y datos compartidos.
- [Hardware](docs/hardware.md): señales, GPIO y periféricos del robot 2.
- [Desarrollo y pruebas](docs/desarrollo.md): configuración, diagnósticos, límites y resolución de problemas.

Para orientarse en el código: `src/main.cpp` prepara el hardware; `src/tasks.cpp` toma decisiones; `include/motor.h` acciona los motores; `src/lineSensor.cpp` lee el borde; `include/globals.h` define pines, estados y constantes.
