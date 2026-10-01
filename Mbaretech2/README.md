# Mbaretech2

Firmware de un robot de sumo autónomo para ESP32-S3. Dos motores impulsan el robot, siete sensores IR localizan al rival y dos sensores delanteros detectan el borde de la pista. El programa usa Arduino, FreeRTOS y PlatformIO.

La configuración se edita en `include/buildConfig.h`. Actualmente selecciona `MBARETECH_2`, la receta `FSM_ACTIVE_RECIPE_STATE_TEST` (máquina `state_test`) y telemetría por Serial/WiFi, sin salida física de motores. `platformio.ini` contiene un único build; no se seleccionan entornos. Para combate, recetas, sensores y el protocolo canónico de telemetría, consultar [firmware.md](docs/firmware.md).

## Primeros pasos

1. Instalar PlatformIO y la plataforma ESP32.
2. Abrir una terminal en `Mbaretech2`.
3. Revisar `include/buildConfig.h` y compilar con `pio run`.
4. Conectar el ESP32-S3 y cargar con `pio run -t upload`.
5. Para leer mensajes serie, habilitar `ENABLE_SERIAL=1` en `include/buildConfig.h`, volver a compilar y usar `pio device monitor -b 115200`.

El repositorio no fija el puerto de carga. PlatformIO debe detectarlo o se debe indicar según el equipo. Se verificó la compilación del build unificado; no se realizó una carga física.

Para abrir el editor de recetas y su consola de telemetría, ejecutar `iniciar_editor.cmd` en Windows o `sh start_editor.sh` en macOS/Linux. El lanzador Windows funciona con PowerShell incluido en el sistema aunque no estén instalados Node.js, Python ni PlatformIO. En macOS/Linux se requiere Node.js 16+ o Python 3.10+. Si el navegador no se abre solo, copiar la URL `http://127.0.0.1:8765/fsm_context_editor_v31.html` que muestra la terminal; `--port 0` solicita un puerto libre e imprime su URL real. **Open Telemetry Console** abre una segunda ventana para WiFi, Web Serial, Web Bluetooth o la fuente **Mock** sin robot. La consola ofrece paneles de señales, FSM, rendimiento, eventos y captura; el editor permanece utilizable offline y recibe solo eventos relevantes para el grafo. Los detalles están en [firmware.md](docs/firmware.md).

## Documentación

- [Contexto y flujo de control](CONTEXT.md): recorrido desde el arranque hasta los estados de combate.
- [Firmware, clases y funciones](docs/firmware.md): responsabilidades, interfaces y datos compartidos.
- [Hardware](docs/hardware.md): señales, GPIO y periféricos del robot 2.
- [Desarrollo y pruebas](docs/desarrollo.md): configuración, diagnósticos, límites y resolución de problemas.

Para orientarse en el código: `src/main.cpp` prepara el hardware; `src/control/tasks.cpp` toma decisiones; `include/motor.h` acciona los motores; `src/sensors/lineSensor.cpp` lee el borde; `include/globals.h` define pines, estados y constantes.
