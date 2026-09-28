# Desarrollo, pruebas y diagnóstico

## Configuración que se compila hoy

`platformio.ini` selecciona `esp32-s3-devkitc-1` y el modo de sensores sin FSM:

```ini
build_flags =
    -DMBARETECH_2
    -DRUN_LINE_SENSOR
    -DRUN_SENSORS_TEST
```

El bucle lee los siete IR y los dos sensores de línea y cede la CPU durante 10 ms entre vueltas. Bluetooth publica los datos con su propio intervalo (100 ms por defecto). Enviar `menu`, seleccionar canales y enviar 5. Yaw se inicializa bajo demanda; mantener el robot inmóvil durante la calibración. Este modo no crea la tarea de combate ni ordena movimientos, y no produce transiciones ESTADO.

Mantener un solo modo que defina `loop()`: no combinar sensores con `RUN_TASK_TEST`, `RUN_MOVEMENTS_TEST` ni otros diagnósticos. Para volver a combate, restaurar el bloque FIGHT comentado y desactivar el bloque de sensores.

## Ajustes de comportamiento

`include/globals.h` contiene `THRESHOLD`, velocidades y tiempos para `MBARETECH_2`. `src/tasks.cpp` también contiene valores literales de maniobras. Antes de ajustar un giro o el umbral, identificar todas las ramas que lo usan y comprobar el resultado con el robot en la pista.

Las duraciones que recibe `elapsedTime()` son ticks FreeRTOS. Confirmar la frecuencia de tick de la plataforma antes de interpretarlas como milisegundos. Los movimientos no usan encoder ni IMU para corregir su ángulo.

BLE acepta mensajes de texto con forma `ÍNDICE VALOR`, por ejemplo `2 80`. El arreglo `parametros` tiene índices 0–18 y se usa sobre todo en la ruta `RUN_MOVEMENTS_TEST`. La interfaz valida índices 0–18; no valida el rango del valor. Cambiar un valor por BLE no cambia necesariamente el combate compilado en `src/tasks.cpp`.

## Cómo comprobar una modificación

1. Ejecutar `pio run` en esta carpeta para compilar la ruta activa.
2. Si se cambió un pin o sensor, verificar sus lecturas y polaridad en el robot.
3. Si se cambió un motor o giro, comprobar primero el sentido de ambas ruedas con el robot inmovilizado.
4. Si se cambió el borde o el arranque, comprobar las transiciones `IDLE`, `FORWARD`, `BRAKE` y `LINE_RETREAT` antes de una prueba completa en la pista.

La carpeta `test/logging/` contiene pruebas host del registro: ejecutar `python test/logging/run.py` con MSVC Build Tools instalado. Compilar confirma una ruta de macros, pero no el comportamiento eléctrico ni la detección física.

## Programas de diagnóstico existentes

- **Motor (`RUN_DRIVER_TEST`, `src/tests/driverTest.cpp`):** alterna marcha atrás, freno y avance. `setup()` no llama a `Motor::begin()` para este modo; requiere corregir la inicialización antes de ejecutarlo.
- **Sensores (`RUN_SENSORS_TEST`, `src/tests/sensorsTest.cpp`):** lee siete IR, DIP y los dos ADC de línea en MBARETECH_2; usa los filtros izquierdo/derecho existentes y cede la CPU durante 10 ms. Los datos se consultan por el menú BLE; no ejecuta la FSM ni ordena movimientos.
- **Línea (`RUN_LS_SENSOR_TEST`, `src/tests/LSsensorTest.cpp`):** intenta leer sensores delanteros y traseros. Incluye `lineSensor.h`, ausente de `Mbaretech2/include`, y los canales traseros no están inicializados. No está listo para usar.
- **Giroscopio (`RUN_GYRO_TEST`, `src/tests/gyroTest.cpp`):** diagnóstico aislado por Serial a 115200: usa begin()/getData() y los métodos de estado existentes en IMU.h para inicializar el DMP y mostrar yaw válido. Activar únicamente RUN_GYRO_TEST; no ejecuta BLE ni FSM. Mantener inmóvil durante la calibración.
- **Movimientos (`RUN_MOVEMENTS_TEST`, `src/movements.cpp`):** máquina alternativa que emplea `parametros` para ensayar maniobras. Debe seleccionarse sin `RUN_TASK_TEST`; revisar su combinación de macros antes de utilizarla.

La interfaz de las clases y funciones usadas por estas rutas está en [firmware.md](firmware.md).

## Problemas frecuentes

**No sale información por serie.** Comprobar que `DEBUG` esté activo, recompilar y abrir el monitor a 115200 baudios. Gran parte de las impresiones están dentro de bloques `#ifdef DEBUG`.

**No comienza a moverse.** Revisar el nivel de `START_PIN` y la combinación DIP leída por `IDLE`. Comprobar también que se haya compilado `RUN_TASK_TEST` y que la tarea haya iniciado los motores.

**Un motor gira en sentido contrario.** Verificar las salidas A0/A1 del motor afectado, el cableado y la macro `MBARETECH_2`.

**Detecta el borde demasiado pronto o tarde.** Observar las lecturas ADC de ambos sensores delanteros y ajustar `THRESHOLD` según la pista. La función exige varias lecturas consecutivas.

**Un cambio BLE no altera el combate.** Buscar si la rama de `src/tasks.cpp` usa `parametros` o una constante de `globals.h`. La mayoría de las maniobras activas emplean constantes.

## Límites que deben tenerse presentes

- `BRAKE` puede escribir `IDLE` cuando cae `startSignal` y luego sobrescribir ese estado según los IR. Las esperas activas tampoco comprueban todas la señal del mismo modo. El paro no está verificado como mecanismo de seguridad.
- `elapsedTime()` comparte un único temporizador. Una salida temprana puede alterar la duración de otra maniobra.
- `checkLineSensora()` y `checkLineSensorb()` usan contadores de 8 bits que pueden desbordarse tras lecturas continuas por debajo del umbral.
- Hay declaraciones y modos experimentales incompletos. Revisar las condiciones de compilación antes de asumir que una ruta de prueba funciona.

Actualizar estos documentos cuando cambien interfaces, pines, estados, flags o procedimientos de prueba. Para cambios internos sin efecto observable, no hace falta ampliar la documentación.



