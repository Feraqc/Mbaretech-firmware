# Hardware de Mbaretech2

El usuario confirmó que los GPIO definidos para `MBARETECH_2` corresponden al cableado actual. Las funciones de cada señal proceden de `include/globals.h`, `include/motor.h` y `src/main.cpp`. El modelo del controlador de motores sigue pendiente.

## Placa y motores

La placa seleccionada es `esp32-s3-devkitc-1`. Los dos motores se controlan mediante una señal PWM y dos señales de dirección por rueda:

- **Izquierdo:** PWM en GPIO 35, A0 en GPIO 36 y A1 en GPIO 37; usa `LEDC_CHANNEL_1`.
- **Derecho:** PWM en GPIO 48, A0 en GPIO 47 y A1 en GPIO 20; usa `LEDC_CHANNEL_0`.

La clase `Motor` configura PWM a 20 kHz y 10 bits. Con A0 bajo y A1 alto ordena marcha adelante; con los niveles invertidos, marcha atrás. `brake()` pone ambas señales en bajo y conserva el duty PWM anterior. **Pendiente:** identificar el controlador de motores y verificar en su hoja de datos qué hace eléctricamente con ambas entradas de dirección en bajo.

## Sensores de rival

Los siete sensores IR son entradas digitales. En la máquina activa, un nivel bajo se interpreta como detección:

- Lateral izquierdo `IR1`: GPIO 39.
- Corto izquierdo `IR2`: GPIO 40.
- Superior izquierdo `IR3`: GPIO 38.
- Superior central `IR4`: GPIO 4.
- Superior derecho `IR5`: GPIO 5.
- Corto derecho `IR6`: GPIO 18.
- Lateral derecho `IR7`: GPIO 17.

El orden de decisión para estos sensores se explica en [CONTEXT.md](../CONTEXT.md). El código no configura resistencias internas de pull-up o pull-down para estas entradas.

## Borde de la pista

El sensor delantero izquierdo está en `ADC1_CHANNEL_2` y el delantero derecho en `ADC1_CHANNEL_7`. `lineSensorsInit()` configura ambos canales con resolución de 12 bits y atenuación `ADC_ATTEN_DB_12`. Cada lado declara una detección después de siete lecturas consecutivas menores o iguales al umbral `THRESHOLD`, que vale 145 para `MBARETECH_2`.

También están declarados `ADC2_CHANNEL_8` y `ADC2_CHANNEL_9` para sensores traseros. Su configuración está comentada y la máquina de combate no los usa. Medir los valores reales en la pista antes de cambiar el umbral.

## Arranque e interruptores

- `START_PIN`: GPIO 41. Una interrupción en ambos flancos copia su nivel a `startSignal`.
- `DIPA`: GPIO 42; `DIPB`: GPIO 2; `DIPC`: GPIO 1; `DIPE`: GPIO 19. Esas cuatro entradas seleccionan la maniobra inicial.
- `DIPD`: GPIO 44. Se configura como entrada y aparece en mensajes de depuración, pero no elige maniobras en la lógica activa.

La rama `IDLE` de `src/tasks.cpp` contiene las 16 combinaciones posibles de `DIPE-DIPA-DIPB-DIPC`. El código lee los niveles directamente con `digitalRead()`; no hay una capa de inversión común para DIP.

## Otras interfaces

BLE anuncia el nombre `MBARETECH`; los UUID RX y TX están en `include/bluetoothComm.h`. La ruta opcional del MPU6050 declara I²C con SDA en GPIO 15 y SCL en GPIO 16. Hay definiciones para encoder izquierdo en GPIO 14 y derecho en GPIO 12, sin uso en combate.

El repositorio no muestra una salida para arma, lectura de batería ni señal de fallo del controlador. No se consideran funciones implementadas en este firmware.
