# Contexto del firmware

Este archivo resume **cómo se ejecuta la configuración de combate actual**. Las interfaces de clases y funciones están en [firmware.md](docs/firmware.md); los pines están en [hardware.md](docs/hardware.md).

## Recorrido de ejecución

```text
setup()
  ├─ inicia BLE y configura entradas IR, línea, DIP y arranque
  ├─ conecta la interrupción de START_PIN con startSignal
  └─ crea la tarea FreeRTOS stateMachineTask()
       ├─ inicia ambos motores y entra en IDLE
       └─ lee sensores → evalúa currentState → ordena motores → repite
```

La función Arduino `loop()` está vacía en esta compilación. La tarea intenta ceder 1 ms al final de cada iteración con `vTaskDelay(pdMS_TO_TICKS(1))`, pero muchas maniobras contienen esperas activas. Por eso, ese retardo no garantiza un ciclo de control de 1 ms.

## Estados que conviene entender primero

- **`IDLE`** frena las ruedas mientras espera `startSignal`. Al recibirlo, los DIP `DIPE`, `DIPA`, `DIPB` y `DIPC` seleccionan una de 16 aperturas: avance, giros, movimientos laterales o estrategias `snake` y `turkish`. `DIPD` está configurado, pero no participa en esa selección.
- **`BRAKE`** es el punto principal de decisión. Lee los IR y da prioridad al sensor central, luego a los cortos, los superiores izquierdo/derecho y los laterales. Si no ve al rival, vuelve a avanzar o realiza avances intermitentes cuando `turkish` está activo. El nombre del estado no implica que siempre ejecute `Motor::brake()`.
- **`FORWARD`** avanza hacia el rival. Los IR cortos ajustan la velocidad relativa de las ruedas; `snake` alterna impulsos asimétricos. Puede pasar a `BRAKE` al perder el objetivo o a `LINE_RETREAT` al detectar el borde bajo las condiciones de esa rama.
- **Giros y correcciones** usan duraciones fijas. Los estados de 45°, 90°, 180°, movimientos cortos y giros en U combinan avance y giro; algunos giros terminan antes si `ENABLE_TURN_CANCEL=1` está activo y el IR central detecta al rival.
- **`LINE_RETREAT`** ordena marcha atrás por un intervalo breve. Después elige entre volver a evaluar al rival y girar 180°.

La lógica de borde no tiene prioridad absoluta en todas las ramas: algunas condiciones permiten continuar hacia un rival detectado. La caída de `startSignal` tampoco se aplica uniformemente a todos los estados. No se debe considerar esta señal un paro de emergencia verificado.

## Qué código se ejecuta

`ENABLE_FSM=1` habilita la máquina de estados de `src/control/tasks.cpp`, aunque su nombre sugiera una prueba. `ENABLE_MOVEMENT_TEST=1` habilita la implementación alternativa de `src/diagnostics/movements.cpp`; las dos no deben compilarse juntas porque definen la misma tarea y `loop()`. `src/diagnostics/movements_old.cpp` requiere además `ENABLE_LEGACY_MOVEMENTS=1` y conserva una versión anterior. El MPU6050 y los pines de encoder no intervienen en el control de combate activo.

Las velocidades, umbrales y duraciones del combate proceden principalmente de `include/globals.h`, con algunos valores literales en `src/control/tasks.cpp`. El arreglo `parametros` sirve sobre todo al modo de pruebas de movimientos, aunque BLE se inicializa también en combate.

## Cambios delicados

Al modificar un estado, revisar su condición de entrada, los dos comandos de motor, la detección de línea, la señal de arranque y la transición de salida. `elapsedTime()` comparte un temporizador estático entre llamadas y compara ticks FreeRTOS; una salida anticipada puede afectar la siguiente maniobra. Ver [desarrollo.md](docs/desarrollo.md) para los límites conocidos y las pruebas disponibles.
