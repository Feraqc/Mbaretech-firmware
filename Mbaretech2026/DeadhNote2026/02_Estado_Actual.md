# Estado Actual del Proyecto (Mbaretech 2)

## 📌 Tabla Completa de Pines (build activo `MBARETECH_2`)

> **Excepción de orden autorizada por el usuario (2026-09-17):** `01_Reglas_IA.md` exige que la sección "Objetivos Importantes Cumplidos" sea siempre lo primero en este documento. Por pedido explícito del usuario, esta tabla de pines y las de DIP switches y `parametros[]` BLE (justo debajo) se ubican como encabezado, por delante de esa sección, dada su importancia crítica como referencia rápida de hardware/estrategia/calibración. No es un incumplimiento accidental de la regla — es una excepción puntual autorizada.

Verificado línea por línea contra `include/globals.h`, `include/motor.h` y `src/main.cpp`:

| Pin (GPIO) | Macro | Señal / función |
|---|---|---|
| 39 | `IR1` | Entrada digital IR — `SIDE_LEFT` (lateral izquierdo) |
| 40 | `IR2` | Entrada digital IR — `SHORT_LEFT` (corto izquierdo) |
| 38 | `IR3` | Entrada digital IR — `TOP_LEFT` (frontal izquierdo) |
| 4  | `IR4` | Entrada digital IR — `TOP_MID` (frontal centro) |
| 5  | `IR5` | Entrada digital IR — `TOP_RIGHT` (frontal derecho) |
| 18 | `IR6` | Entrada digital IR — `SHORT_RIGHT` (corto derecho) |
| 17 | `IR7` | Entrada digital IR — `SIDE_RIGHT` (lateral derecho) |
| 42 | `DIPA` | Entrada digital — bit de estrategia (ver tabla de DIP switches abajo) |
| 2  | `DIPB` | Entrada digital — bit de estrategia (ver tabla de DIP switches abajo) |
| 1  | `DIPC` | Entrada digital — bit de estrategia (ver tabla de DIP switches abajo) |
| 44 | `DIPD` | Entrada digital — **leído pero sin efecto** en la lógica de decisión actual |
| 19 | `DIPE` | Entrada digital — bit de estrategia más significativo (ver tabla de DIP switches abajo) |
| 41 | `START_PIN` | Entrada + interrupción `CHANGE` — señal de arranque/kill-switch (`startSignal`) |
| 15 | `SDA_PIN` | I2C — dato (IMU MPU6050, no usado por la máquina de estados) |
| 16 | `SCL_PIN` | I2C — reloj (IMU MPU6050, no usado por la máquina de estados) |
| 14 | `ENCODER_LEFT` (repurpuesto 2026-09-29) | Ahora `LINE_BACK_RIGHT` (ADC2_CH3) — sensor de línea trasero derecho, "LS4". Los encoders nunca se leyeron en `src/`, pin libre reutilizado |
| 12 | `ENCODER_RIGHT` (repurpuesto 2026-09-29) | Ahora `LINE_BACK_LEFT` (ADC2_CH1) — sensor de línea trasero izquierdo, "LS3". Ídem |
| ADC1_CH2 | `LINE_FRONT_LEFT` | Entrada analógica — sensor de línea frontal izquierdo (usado en combate), "LS1" |
| ADC1_CH7 | `LINE_FRONT_RIGHT` | Entrada analógica — sensor de línea frontal derecho (usado en combate), "LS2" |
| ADC2_CH1 | `LINE_BACK_LEFT` | = GPIO12 (antes `ENCODER_RIGHT`). Sensor de línea trasero izquierdo ("LS3"). Instalado y activo desde 2026-09-29 (`LINE_BACK_INSTALLED=1`) |
| ADC2_CH3 | `LINE_BACK_RIGHT` | = GPIO14 (antes `ENCODER_LEFT`). Sensor de línea trasero derecho ("LS4"). Instalado y activo desde 2026-09-29 |
| 35 | `PWM_A` | Salida PWM (LEDC) — motor izquierdo |
| 36 | `PIN_A0` | Salida digital — dirección motor izquierdo |
| 37 | `PIN_A1` | Salida digital — dirección motor izquierdo |
| 48 | `PWM_B` | Salida PWM (LEDC) — motor derecho |
| 47 | `PIN_B0` | Salida digital — dirección motor derecho |
| 20 | `PIN_B1` | Salida digital — dirección motor derecho. Ya no hay conflicto con `LINE_BACK_RIGHT` (se movió a GPIO14). |

*(La comunicación BLE no usa pines GPIO — corre sobre el radio interno del ESP32-S3.)*

## 🎛️ Estrategias Pre-Combate (DIP Switches)

**Corregido tras verificar contra `tasks.cpp` actual** — la descripción anterior (heredada de `CLAUDE.md`) de "un switch, un modo" es **inexacta**. Físicamente hay **5 pines DIP** definidos (`DIPA`, `DIPB`, `DIPC`, `DIPD`, `DIPE`), pero solo **4 se leen realmente** en la decisión. En el `case IDLE`, justo cuando `startSignal` pasa a `true`, el código lee `DIPE·DIPA·DIPB·DIPC` como una palabra binaria (en ese orden de peso) y evalúa las 16 combinaciones posibles con un `if/else if` en cascada; `snake` y `turkish` son flags que cada combinación fija de forma independiente, no el efecto de un solo switch. Tabla completa (1 = `digitalRead` devuelve `HIGH`; no verificado a qué posición física del switch corresponde eso — depende del cableado de pull-up/pull-down de la placa):

| DIPE DIPA DIPB DIPC | Estado inicial | snake | turkish | Qué hace |
|---|---|---|---|---|
| 0 0 0 0 | `TEST_FORWARD` ⚠️ | no | no | **Repurposado (2026-09-17) como modo de prueba de banco**, ya no es combate: ambos motores adelante a `parametros[2]`%, **ignora todos los sensores** (7 IR + línea blanca), ajustable en vivo por BLE (`"2 <valor>"`), solo obedece al `startSignal`. Antes era "avance recto simple" (`FORWARD` de combate) — para restaurarlo, volver a poner `currentState = FORWARD;` en ese branch de `case IDLE` en `tasks.cpp`. |
| 0 0 0 1 | `FORWARD` | sí | no | Avance en zigzag ("snake": alterna 95%/85% entre ruedas) |
| 0 0 1 0 | `BRAKE` | no | sí | Modo "turco": no avanza de una, hace saltitos cortos cada 2s esperando que algo entre en rango |
| 0 0 1 1 | `BRAKE` | sí | sí | Snake + turco combinados |
| 0 1 0 0 | `TURN_LEFT_90` | no | no | Apertura: gira 90° a la izquierda antes de cazar |
| 0 1 0 1 | `TURN_RIGHT_90` | no | no | Apertura: gira 90° a la derecha |
| 0 1 1 0 | `TURN_180` | no | no | Apertura: giro completo de 180° |
| 0 1 1 1 | `L_MOVEMENT_45` | no | sí | Movimiento compuesto: gira 45° izq, avanza un poco, corrige 90° a la derecha |
| 1 0 0 0 | `R_MOVEMENT_45` | no | sí | Espejo del anterior: gira 45° der, avanza, corrige 90° a la izquierda |
| 1 0 0 1 | `GIRO_U_L` | no | sí | Arco amplio hacia la izquierda (ruedas a 90%/60%, no es un giro en el lugar) por 1s |
| 1 0 1 0 | `GIRO_U_R` | no | sí | Espejo: arco amplio hacia la derecha, 1s |
| 1 0 1 1 | `GIRO_U_L_LONG` | no | sí | Igual que `GIRO_U_L` pero 2s (arco más largo) |
| 1 1 0 0 | `GIRO_U_R_LONG` | no | sí | Igual que `GIRO_U_R` pero 2s |
| 1 1 0 1 | `SHORT_LEFT_MOVE` | sí | sí | Corrección corta hacia la izquierda (90%/42%, 80ms) |
| 1 1 1 0 | `SHORT_RIGHT_MOVE` | sí | sí | Corrección corta hacia la derecha |
| 1 1 1 1 | `GIRO_U_L` ⚠️ | no | no | **Libre / redundante:** mismo estado que la fila 1001 (`GIRO_U_L`), y el `case GIRO_U_L` no lee `snake` ni `turkish` en ningún lado, así que ambas filas son 100% idénticas en comportamiento. Combinación candidata si en el futuro se necesita un modo más para reutilizar. |

* **`DIPD` está declarado y en `pinMode(INPUT)` (`main.cpp`), pero `tasks.cpp` nunca lo lee en la lógica de decisión** (solo aparece en un `Serial.print` bajo `DEBUG`). Es un pin muerto para la estrategia actual, a pesar de estar documentado como parte de la "coreografía de apertura" en versiones anteriores de esta nota y en `CLAUDE.md`.
* `DIPE` (pin 19) sí es un bit significativo activo en la tabla de decisión y **no estaba documentado en ninguna nota anterior**.
* **`turkish` también cambia el comportamiento dentro de `FORWARD`**: si el robot pierde de vista al enemigo (se apaga el sensor central) y `turkish` está activo, frena en seco antes de pasar a `BRAKE`; si no, pasa a `BRAKE` sin frenar explícitamente.
* **Hallazgo:** en el `case BRAKE` actual, las llamadas `leftMotor.brake()`/`rightMotor.brake()` están **comentadas** — el estado se llama "BRAKE" pero no frena motores, solo lee sensores y decide el próximo estado. Si se entra desde un giro, los motores ya estaban frenados por ese giro; si se entra desde `FORWARD` sin `turkish`, el robot puede seguir avanzando con la última velocidad seteada hasta que se resuelve el próximo estado.
* **Código muerto adicional:** hay una variable `bandera` (declarada `bool bandera = true;`) que varias combinaciones de DIP ponen en `true`, pero **ningún otro lugar del archivo la lee** — no tiene efecto, igual que `DIPD` y el `delta` de `FORWARD` (ver "Máquina de Estados" más abajo).

## 🔧 Tabla de Parámetros BLE (`parametros[]`)

Array de 20 enteros (`ARRAY_PARAMETROS_SIZE = 20`, era 19 hasta el 2026-10-04) definido y con sus valores por defecto en `src/globals.cpp`, escribible en caliente por Bluetooth mandando `"INDICE VALOR"` (`include/bluetoothComm.h` hace `parametros[INDICE] = VALOR;`). Cada índice nació pensado para un uso específico (comentarios originales del array), pero **no todos están conectados hoy en `tasks.cpp`** — verificado línea por línea contra el código actual:

| Índice | Constante de origen | Valor por defecto | Para qué es | ¿Se usa en `tasks.cpp` (build activo) hoy? |
|---|---|---|---|---|
| 0 | — | 0 | Selector de modo en `movements.cpp` (0=nada, 1=disparar estado puntual, 2=debug de sensores) | No — solo se escribe una vez (`parametros[0]=0`) en `TURN_RIGHT_45`, nadie lo lee |
| 1 | — | 0 | En `movements.cpp`, elige QUÉ estado disparar cuando `parametros[0]==1` (3=`TURN_LEFT_45`, 4=`TURN_LEFT_90`, etc.) | No |
| 2 | `FORWARD_X` | 94 | Velocidad de avance (pensada para `forward`/`backward`/`line retreat`) | **Sí** — usado por el nuevo `TEST_FORWARD` (ver más abajo) |
| 3 | `TURN_LEFT_SPEED` | 94 | Velocidad de giro izquierdo | **Sí** — `TURN_LEFT_45`, `TURN_LEFT_90`, `TURN_180`, tramo de `L_MOVEMENT_45` |
| 4 | `TURN_LEFT_45_DELAY` | 55 | Duración del giro de 45° izquierdo | **Sí** |
| 5 | `TURN_LEFT_90_DELAY` | 80 | Duración del giro de 90° izquierdo | **Sí** |
| 6 | `TURN_RIGHT_SPEED` | 94 | Velocidad de giro derecho | **Sí** — `TURN_RIGHT_45`, `TURN_RIGHT_90`, tramo de `R_MOVEMENT_45`, corrección dentro de `L_MOVEMENT_45` |
| 7 | `TURN_RIGHT_45_DELAY` | 45 | Duración del giro de 45° derecho | **Sí** |
| 8 | `TURN_RIGHT_90_DELAY` | 70 | Duración del giro de 90° derecho | **Sí** |
| 9 | `TURN_LEFT_180_DELAY` | 150 (**calibrado a ojo 2026-10-05: 120**, todavía no pasado a `globals.h`) | Duración del giro de 180° a la izquierda | **Sí** — `TURN_180` |
| 10 | `SHORT_LEFT_DELAY` | 15 | Duración de la corrección corta hacia la izquierda | No — `tasks.cpp` sigue usando el `#define` fijo (en `FORWARD`, ramas de snake y corrección) |
| 11 | `SHORT_RIGHT_DELAY` | 15 | Duración de la corrección corta hacia la derecha | No — ídem, `#define` fijo |
| 12 | `THRESHOLD` | 145 | Umbral del sensor de línea (blanco vs. dohyo) | No — `lineSensor.cpp` usa el `#define` fijo directamente |
| 13 | `TURKISH_TIME` | 2000 | Cada cuánto (ms) el modo "turco" hace un saltito | No — `#define` fijo en `BRAKE` |
| 14 | `TURKISH_DELAY` | 100 | Duración de cada saltito "turco" | No — `#define` fijo |
| 15 | `TURKISH_SPEED` | 70 | Velocidad del saltito "turco" | No — y además **no se usa ni el `#define`**: el saltito real usa `FORWARD_60` hardcodeado, `TURKISH_SPEED` está muerta en todo `tasks.cpp` |
| 16 | `GIRO_U_DELAY` | 500 | Pensado como duración de un arco en U | No — `GIRO_U_L`/`GIRO_U_R` usan un `1000` hardcodeado, no esta constante ni este índice |
| 17 | `GIRO_U_L_DELAY` | 1000 | Pensado como duración de un arco en U largo | No — `GIRO_U_L_LONG`/`GIRO_U_R_LONG` usan un `2000` hardcodeado, no esta constante ni este índice |
| 18 | `CORRECT_SPEED` | 4 | Sesgo de compensación en la rueda "interna" de cada giro | **Sí** — en todos los giros y movimientos compuestos que usan corrección |
| 19 | `TURN_RIGHT_180_DELAY` | 150 (**calibrado a ojo 2026-10-05: 125**, todavía no pasado a `globals.h`) | Duración del giro de 180° a la derecha (nuevo 2026-10-04) | No — combate solo tiene 180° izquierdo; lo usa `calibracion.cpp` `0110` |

**Resumen:** de los 20 (`[19]` agregado 2026-10-04, solo para calibración), hoy **9 están realmente conectados a `tasks.cpp`** (`[2]` a `[9]` y `[18]`, todos ligados a giros + el nuevo modo de prueba). El resto (`[0]`,`[1]`,`[10]`-`[17]`) solo tiene efecto en `movements.cpp` (el estado-máquina viejo), o directamente no tiene efecto en ningún lado (`[15]`,`[16]`,`[17]`, por las constantes muertas ya documentadas).

**Confirmación por BLE agregada (2026-09-22):** durante las pruebas de campo, el usuario no tenía forma de saber si un comando BLE realmente llegaba/se parseaba (`onWrite()` no devolvía nada). Se corrigieron dos cosas en `include/bluetoothComm.h`:
* La característica RX solo aceptaba `PROPERTY_WRITE` (con respuesta) — muchas apps terminal UART (ej. "BLE Serial nRF") mandan por defecto **"Write Without Response"**, que el servidor rechazaba en silencio. Ahora acepta ambas (`PROPERTY_WRITE | PROPERTY_WRITE_NR`).
* `onWrite()` ahora valida que el índice esté dentro de `0`-`ARRAY_PARAMETROS_SIZE-1` (antes escribía fuera de los límites del array con cualquier índice `>= 0`, un riesgo real de corrupción de memoria) y **siempre responde por notificación BLE**: `"OK parametros[N]=V"` si escribió bien, o un mensaje `"ERROR ..."` si el formato o el índice están mal. También imprime el string recibido por Serial si `DEBUG` está activo. Antes no había ninguna confirmación, ni por BLE ni por Serial.

---

## Objetivos Importantes Cumplidos
* Creación y estructuración de la DeathNote para trazabilidad del proyecto `[2026-05-12]`.
* Auditoría completa del código base y detección del bug físico de sensores `[2026-05-12]`.
* Definición de estrategias para mitigación de banderas enemigas y uso de IMU `[2026-05-12]`.
* Prueba de frenado con sensores de línea (adelante LS1/LS2 y atrás LS3/LS4) validada en el dohyo `[2026-10-01]`.

---

Este documento refleja cómo está funcionando el robot actualmente a nivel de hardware y software para evitar retrabajos y confusiones.

## Actualmente

### 1. Hardware y Bugs Conocidos
* **Placa Activa:** `MBARETECH_2` (Definido en `platformio.ini`).
* **Sensores Infrarrojos (IR):** 7 sensores digitales.
  * *CRÍTICO - Cruce de Pines:* Verificado contra los comentarios actuales de `src/tests/sensorsTest.cpp` (no se verificó físicamente en esta sesión, solo contra el código). El cruce documentado originalmente es real, pero el archivo de test documenta **más swaps de los que esta sección registraba**:
    * El sensor que apunta al centro (`TOP_MID` físico) está conectado al pin de `SHORT_RIGHT` (`IR6`). ✅ Confirmado.
    * El sensor que apunta a la derecha corta (`SHORT_RIGHT` físico) está conectado al pin de `TOP_MID` (`IR4`). ✅ Confirmado.
    * **Nuevo hallazgo:** `IR1` (`SIDE_LEFT` lógico) físicamente sería el corto-izquierdo; `IR2` (`SHORT_LEFT` lógico) directamente **"no lee"** según el comentario del test (posible sensor muerto o mal conectado); `IR5` (`TOP_RIGHT` lógico) físicamente sería el lateral derecho; `IR7` (`SIDE_RIGHT` lógico) físicamente sería el frontal-derecho.
    * *Nota:* Esto causó un bug donde el robot giraba hacia el enemigo pero no atacaba porque leía el pin equivocado. Dado el alcance real del cruce (probablemente todo el lado derecho + `SIDE_LEFT`/`SHORT_LEFT`), se recomienda una re-verificación física completa del arnés de sensores antes de confiar en el mapeo lógico del enum `Sensor`.
* **Sensores de Línea:** 2 frontales analógicos (ADC1_CH2/ADC1_CH7) integrados y usados por la máquina de estados. Existen además 2 sensores traseros (`LINE_BACK_LEFT/RIGHT`, ADC2) definidos en `globals.h`, pero **no están conectados a la lógica de combate** — solo se leen en el sketch de calibración aislado `LSsensorTest.cpp` (requiere su propio build flag, no es parte del build activo).
* **Hardware inactivo en combate:** MPU6050 (Acelerómetro/Giroscopio) y Encoders. Están conectados pero la máquina de estados actual no los utiliza para tomar decisiones.
  * **Prueba del IMU (2026-09-28):** `src/tests/pruebaIMU.cpp` (flag `RUN_PRUEBA_IMU`, **build activo actualmente** en `platformio.ini` en lugar de `CALIBRACION`) lee el MPU6050 por registros directos en I2C 15/16, sin librerías. **Probado y funcionando (2026-09-28)**, después de arreglar una mala conexión del VCC del módulo (medía 1.5V). `src/tests/gyroTest.cpp` (`RUN_GYRO_TEST`) y la clase de `include/IMU.h` (DMP) no compilan en el estado actual (faltan declaraciones y librerías); no usarlos como referencia de que "el IMU ya funciona".
  * **Yaw del giroscopio confirmado preciso (2026-09-30):** el usuario giró el robot físicamente contra una referencia y comparó contra el `yaw=` integrado que imprime `pruebaIMU.cpp` por Serial — **coincide bien ("esta perfecto")**. Esto habilita el punto 2 de `99_Razonamientos_y_mejoras.md` (reemplazar los giros a tiempo fijo por giros de lazo cerrado con el giroscopio) como algo viable de implementar, no solo teórico.

### 1.b Modo de Calibración de Sensores (`RUN_CALIBRACION`, agregado 2026-09-22)

Nuevo archivo `src/calibracion.cpp`, **separado del código de competencia** (a pedido del usuario, para no mezclar lógica de prueba con `tasks.cpp`), con **su propio `setup()`**, totalmente desacoplado del `setup()` de combate en `main.cpp`. Es un `loop()` puro sin motores ni lógica de decisión: lee los 7 IR, los 5 DIP (incluyendo `DIPD`, que en combate está muerto) y los 2 sensores de línea cada 300ms, y los manda como texto tanto por Serial (si `DEBUG`) como por BLE (`sendData()`), aprovechando que ya confirmamos que el canal BLE funciona (ver más abajo). Formato: `IR=SLscLTLTMTRscRSR DIP=ABCDE L=<adc>/<bool> R=<adc>/<bool>` (bits en ese orden fijo).

**Por qué tiene `setup()` propio (2026-09-22, segunda vuelta):** el usuario notó que, tal como estaba, el modo de calibración seguía dependiendo del `setup()` de combate en `main.cpp` — que incluye `esp_efuse_write_field_cnt(...)` (quema un eFuse, irreversible) **sin condición, en cada arranque**, ninguno necesario para leer sensores. Se envolvió el `setup()` de `main.cpp` en `#ifndef RUN_CALIBRACION` (no compila en este modo) y se le agregó a `calibracion.cpp` su propio `setup()` mínimo: Serial, BLE, `pinMode` de IR/DIP, `lineSensorsInit()` — nada de eFuse, nada de motores.

**Kill switch sí se mantiene (2026-09-22, tercera vuelta):** el usuario pidió que el comportamiento del `startSignal` esté presente igual que en combate — es la única red de seguridad real del proyecto. `calibracion.cpp` tiene su propio `pinMode(START_PIN, INPUT)` + `attachInterrupt` (con un ISR propio, `CalibKS_ISR`, para no depender del `KS_ISR` de `main.cpp`). La lectura de sensores sigue corriendo siempre (es inofensiva), pero **el reporte por BLE/Serial solo se manda mientras `startSignal` esté activo** — primera activación del switch prende el reporte, la siguiente lo apaga, igual que el kill switch de combate.

**Para usarlo:** en `platformio.ini`, comentar el bloque `build_flags = ;FIGHT` (combate) y descomentar/usar el bloque `build_flags = ;CALIBRACION` — **actualmente el build activo es CALIBRACIÓN, no combate**, hay que volver a cambiarlo antes de competir.

**DIP switches como selector de acción a calibrar (2026-09-22, cuarta vuelta):** el usuario pidió que los DIP también seleccionen qué calibrar en este modo, igual que en combate, pero sin tocar `tasks.cpp`. Se agregó un dispatcher en `calibracion.cpp` que reutiliza el mismo orden de bits (`DIPE·DIPA·DIPB·DIPC`) que la tabla de combate, pero acotado a lo que hoy está conectado a `parametros[]`:

| Combo | Acción | `parametros[]` |
|---|---|---|
| `0000` | Solo reporte de sensores, sin mover motores (default/seguro) | — |
| `0001` | Avance recto (solo DIP, sin sensor) | `[2]` |
| `0010` | Giro 45°, **bilateral** — `TOP_LEFT` → izquierda (`[3]`/`[4]`), `TOP_RIGHT` → derecha (`[6]`/`[7]`), nada detectado → frena | `[3]`, `[4]`, `[6]`, `[7]`, `[18]` |
| `0011` | Giro 90°, **bilateral** — `SIDE_LEFT` → izquierda (`[3]`/`[5]`), `SIDE_RIGHT` → derecha (`[6]`/`[8]`), nada detectado → frena | `[3]`, `[5]`, `[6]`, `[8]`, `[18]` |
| `0100` | Junta `0010`+`0011`: TOP → 45°, SIDE → 90°, nada → frena. **Idéntico a `0111`** (candidato a liberar) | `[3]`–`[8]`, `[18]` |
| `0101` | **Shorts de combate (2026-10-04):** `SHORT_LEFT` (prioridad) / `SHORT_RIGHT` → `SHORT_LEFT_MOVE`/`SHORT_RIGHT_MOVE` exactos de `tasks.cpp` (ambas ruedas adelante 90/42+`[18]`, 80ms). Una vez y después **10s de cooldown único** para los dos lados. Sin probar en hardware | `[18]` |
| `0110` | **Giro 180° bilateral (2026-10-04):** `SIDE_LEFT` → 180° izquierda (`[3]`/`[9]`, igual que `TURN_180`), `SIDE_RIGHT` → 180° derecha (`[6]`/`[19]`), nada → frena. Antes giraba sin parar (solo DIP). **Calibrado a ojo 2026-10-05: `[9]`=120ms, `[19]`=125ms** | `[3]`, `[6]`, `[9]`, `[18]`, `[19]` |
| `0111` | Seguir sin atacar 1: TOP → 45°, SIDE → 90°; SHORT y `TOP_MID` no hacen nada (frena) | `[3]`–`[8]`, `[18]` |
| `1000` | **Seguir sin atacar 2 (2026-10-04):** prioridad 1) línea delantera confirmada con 5 lecturas → reversa `FORWARD_90` 80ms + 180° izquierda (`[3]`/`[9]`); 2) SHORT → shorts de combate de `0101` con **5s de cooldown solo para los shorts**; 3) TOP → 45°; 4) SIDE → 90°. Sin probar en hardware | `[3]`–`[9]`, `[18]` |
| `1001` | **Autocalibración 45° con IMU (2026-10-04, traída de `autoCalGiro.cpp`):** alterna IZQ/DER, mide con el giroscopio y corrige `[4]`/`[7]` hasta 45 ± 2°. Una vez por activación, después repite `FIN 45g ...` cada 2s. No guarda los tiempos solos. Sin probar en hardware | `[3]`, `[4]`, `[6]`, `[7]`, `[18]` |
| `1010` | **Autocalibración 90° con IMU (2026-10-04):** igual que `1001`, con `[5]`/`[8]` y objetivo 90 ± 2° (`FIN 90g ...`). Sin probar en hardware | `[3]`, `[5]`, `[6]`, `[8]`, `[18]` |
| `1011`-`1111` | Sin asignar, no hace nada (frena) | — |

**Dos modelos de disparo distintos, coexistiendo a propósito:** `0001` es "solo DIP" (`0110` también lo era hasta el 2026-10-04, ahora es por sensor) — corren sin condición apenas se selecciona ese combo (se repiten cada ~300ms), sin mirar ningún sensor IR; esto es intencional, para poder ejercitar un movimiento puntual y tunearlo sin necesidad de poner un objeto delante del sensor. `0010`, `0011`, `0111`, `1000`, `1001` son "por sensor" — el DIP elige un *modo*, y ese modo solo mueve motores cuando el sensor correspondiente detecta algo, igual que hace `BRAKE` en combate.

**Unificación bilateral de 45°/90° (2026-09-23, undécima vuelta):** el usuario notó un error de concepto en su cabeza (no en el código): pensaba que `0010`-`0110` ya respondían a sensores como en combate, cuando en realidad eran puramente DIP-only. Tras aclarar la diferencia, pidió que los giros de 45°/90° sí respondan a sensores, pero de forma bilateral: un solo combo por ángulo, que gire hacia el lado que corresponda según cuál sensor detecte. Se modificó `calibracion.cpp`:
- **`0010` (antes "giro 45 izquierda" fijo):** ahora mira `TOP_LEFT`/`TOP_RIGHT` y gira hacia el lado que corresponda (mismos `parametros[3]`/`[4]` para izquierda, `[6]`/`[7]` para derecha); si no detecta nada, frena.
- **`0011` (antes "giro 90 izquierda" fijo):** mismo patrón con `SIDE_LEFT`/`SIDE_RIGHT` (`[3]`/`[5]` izquierda, `[6]`/`[8]` derecha).
- **`0100`/`0101`** (antes "giro 45/90 derecha" fijos) quedaron **libres** — ya no hacen falta, la lógica de "derecha" vive ahora adentro de `0010`/`0011`.
- **`0110` (giro 180°) no se tocó** — no hay un sensor único de "atrás" en este robot para gatillarlo automáticamente, sigue siendo DIP-only.
`pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `Mbaretech2026/CLAUDE.md`.

Cada giro corre completo (bloqueante con `elapsedTime()`, misma cuenta que el estado equivalente de `tasks.cpp`, sin `CANCEL_TURNS` — a propósito, para medir el giro "puro" sin interrupciones) y se repite automáticamente cada ~300ms mientras ese combo de DIP y `startSignal` sigan activos, así se puede ajustar un valor por BLE y verlo de nuevo sin tener que volver a disparar nada. El reporte de sensores (ver más abajo) sigue funcionando igual sin importar qué acción esté corriendo.

**Reporte solo ante cambios, en español (2026-09-22, quinta vuelta):** a pedido del usuario, el mensaje ya no se manda cada 300ms sin importar si cambió algo — se compara contra el último mensaje enviado (`static String ultimoMsg`) y solo se manda/imprime si es distinto. Las etiquetas de los sensores de línea pasaron de `L`/`R` a `IZQ`/`DER`.

**Causa real de que igual imprimiera "sucio" (2026-09-22, sexta vuelta):** el mensaje incluía el valor crudo del ADC de línea (`IZQ=1234/0`), que fluctúa por ruido eléctrico aunque no cambie nada físicamente — eso rompía la comparación de "solo si cambió" porque el número crudo casi nunca coincidía entre una lectura y la siguiente. Se sacó el valor crudo del mensaje; ahora `IZQ=`/`DER=` reportan **solo el booleano** ya debounced (`checkLineSensora`/`checkLineSensorb`, que exige 7 lecturas seguidas bajo el umbral antes de marcar `true`), no el número. Con esto el reporte debería quedar realmente silencioso hasta que un sensor cambie de estado de verdad.

**Formato rehecho: sensores individuales, sin DIP ni MODO (2026-09-22, séptima vuelta):** el usuario pidió ver "todos los sensores, ON/OFF nada más" — no le servía el `IR=0000000` comprimido (no se distingue cuál bit es cuál sensor sin memorizar el orden) ni el `DIP=` (no es un sensor ambiental, es un switch de configuración física). Se rehizo el mensaje para listar los **9 sensores reales** (7 IR + 2 línea) con nombre propio: `SIDE_LEFT=0 SHORT_LEFT=0 TOP_LEFT=0 TOP_MID=0 TOP_RIGHT=0 SHORT_RIGHT=0 SIDE_RIGHT=0 LINEA_IZQ=0 LINEA_DER=0`. Se sacaron `DIP=` y `MODO=` del reporte (el `combo`/dispatcher de acciones se mantiene igual, solo dejó de imprimirse). *Nota:* el mensaje ronda ~115 caracteres — si en la app aparece cortado, es un tema de MTU de BLE negociado bajo, no del parseo; se puede acortar de ser necesario.

**Importante — esto no sobrevive un reflash:** `parametros[]` es solo RAM, se reinicia desde los `#define` de `globals.h` en cada arranque. Para llevar un valor calibrado acá al build de combate hay que (a) volver a mandar el mismo `"INDICE VALOR"` por BLE una vez corriendo `tasks.cpp`, o (b) editar el `#define` correspondiente en `globals.h` para que quede como default permanente.

Este modo permite por fin **verificar empíricamente** el cruce de pines documentado en el punto 1 arriba (hasta ahora solo confirmado por comentarios viejos en `sensorsTest.cpp`, nunca por prueba física): acercar un objeto a cada sensor IR uno por uno y confirmar por BLE cuál bit cambia, sin depender del cable USB.

**Diagnóstico de DIP switches y hallazgo de hardware (2026-09-22/23, octava vuelta):** el usuario reportó que el combo `0001` (avance recto) nunca se activaba, quedando siempre en `0000`. Se comparó línea por línea la lectura de `combo` en `calibracion.cpp` contra la de `tasks.cpp` (mismo orden de bits, mismos pines, misma polaridad) — no hay discrepancia de código entre ambos archivos, así que el problema es de hardware, no de firmware. Se agregó un print de depuración solo por Serial (bajo `DEBUG`, no va por BLE) que muestra los 5 pines crudos y el `combo` calculado, solo cuando cambia.

**Confirmado con el robot a mano (2026-09-23):** el usuario probó el switch de `DIPD` con el monitor serial abierto y confirmó que el string queda en `...00001`, con el último carácter (`D`) sin cambiar sin importar la posición del switch — **`DIPD` está físicamente roto/atascado en `HIGH`**. Esto es inofensivo para la selección de modo (ni `tasks.cpp` ni `calibracion.cpp` leen `DIPD` en el cálculo de `combo`).

**`DIPC` confirmado sano:** probado individualmente, cambia correctamente entre `0` y `1`. Esto resuelve la duda pendiente — no hay ningún switch roto que impida llegar a `0001`. La lectura anterior de `...00001` con `C=0` fijo fue, con esta info, casi seguro simplemente que `DIPC` estaba en `OFF` en ese momento (no se había movido ese switch todavía), no un switch fallado. Queda pendiente la prueba final: poner físicamente `DIPA=OFF, DIPB=OFF, DIPC=ON, DIPE=OFF`, confirmar `combo=1` en el monitor, y confirmar que el avance recto (`parametros[2]`%) efectivamente mueve las ruedas.

**Nuevo combo `0111` — "seguir sin atacar" (2026-09-23, novena vuelta):** el usuario pidió un modo que gire hasta encarar al objetivo sin atacarlo, reutilizando la lógica de decisión de `BRAKE` en `tasks.cpp` mientras el robot no está disponible para más pruebas físicas ("vamos a seguir con los demás estados para tenerlo ready"). Se agregó `case 7` al dispatcher de `calibracion.cpp`, inicialmente cubriendo solo `TOP_LEFT`/`TOP_RIGHT` (giro 45°) y `SIDE_LEFT`/`SIDE_RIGHT` (giro 90°), dejando `TOP_MID`, `SHORT_LEFT` y `SHORT_RIGHT` en freno. El usuario notó correctamente que de los 6 sensores no-centrales, solo 4 quedaban cubiertos ("¿cuál es el que falta?"). Se corrigió inicialmente tratando `SHORT_LEFT`/`SHORT_RIGHT` igual que `TOP_LEFT`/`TOP_RIGHT` (giro puro de 45°) — explicado en detalle el porqué `SHORT_LEFT_MOVE`/`SHORT_RIGHT_MOVE` de `tasks.cpp:808-833` no son giros puros (avanzan ambos motores a velocidad distinta, empuje+curva, no pivote).

**Se abrió en tres variantes en vez de una sola (2026-09-23, décima vuelta):** tras entender la diferencia entre pivote puro y empuje+curva, el usuario pidió separar el tratamiento de `SHORT_LEFT`/`SHORT_RIGHT` en tres combos distintos:
- **`0111` (seguir sin atacar 1):** se revirtió a la versión original — `SHORT_LEFT`/`SHORT_RIGHT` se **omiten por completo** (quedan en freno, igual que "nada detectado"), sin ningún pivote ni empuje. Solo `TOP_LEFT`/`TOP_RIGHT`/`SIDE_LEFT`/`SIDE_RIGHT` giran.
- **`1000` (seguir sin atacar 2):** `SHORT_LEFT`/`SHORT_RIGHT` disparan un **pivote asimétrico** nuevo (no reutiliza `parametros[3]`/`[6]`): una rueda al 90% adelante, la otra al 42% atrás (sin corrección `[18]`), sigue siendo pivote puro (cero avance neto), solo que con velocidades desparejas en vez de simétricas. La duración, en vez de quedar hardcodeada, se hizo ajustable en vivo por BLE reutilizando `parametros[10]`/`[11]` (`SHORT_LEFT_DELAY`/`SHORT_RIGHT_DELAY` — existían en el array desde el origen pero `tasks.cpp` nunca los leía, solo `movements.cpp`).
- **`1001` (seguir sin atacar 3):** `SHORT_LEFT`/`SHORT_RIGHT` reproducen el movimiento **real** de combate tal cual (`90%`/`42%+parametros[18]`, ambas ruedas adelante, 80ms fijos, igual que `SHORT_LEFT_MOVE`/`SHORT_RIGHT_MOVE`) — es el único combo de todo `calibracion.cpp` que empuja de verdad. A pedido explícito del usuario, por seguridad, este movimiento queda limitado a **una repetición cada 10 segundos por lado** — se implementó con un timestamp propio por `millis()` (`static unsigned long ultimoShortLeft/ultimoShortRight`), **sin tocar la función compartida `elapsedTime()`** (que ya es un singleton global usado por todos los giros del archivo — meterle un segundo uso ahí habría interferido con el resto de las esperas bloqueantes). El resto de acciones (los pivotes de 45°/90° en los tres combos) no tienen ninguna restricción de repetición.

Las tres variantes comparten la misma prioridad de decisión que `BRAKE` de combate (`SHORT` antes que 45° antes que 90°) y el mismo comportamiento final: solo `TOP_MID` (encarado, dispararía el ataque real) o "nada detectado" frenan. `1010`-`1111` quedan como los únicos combos libres. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

**Requirió un cambio estructural en `include/bluetoothComm.h`:** antes definía variables y funciones directamente en el header (no solo las declaraba), lo cual solo funcionaba porque un único `.cpp` (`main.cpp`) lo incluía. Para que `calibracion.cpp` también pudiera llamar a `sendData()`, se separó en declaraciones (`.h`) + definiciones nuevas en `src/bluetoothComm.cpp`. Cualquier archivo nuevo que quiera mandar datos por BLE ahora puede incluir `bluetoothComm.h` sin riesgo de error de símbolo duplicado.

**`0011` (giro 90° bilateral) validado en banco (2026-09-23):** el usuario reportó inicialmente que no giraba como esperaba. Se comparó línea por línea contra `0010` (que sí funcionaba) — resultaron estructuralmente idénticos (mismo patrón, solo cambian el sensor y el índice de `parametros[]`), sin diferencia de código real. Se agregó un print de depuración puntual dentro de `case 3` (`case3 combo=... SIDE_LEFT=... SIDE_RIGHT=...`, bajo `DEBUG`, cada vez que ese combo corre) para confirmar en el momento exacto qué ve el `switch`. Tras la prueba, el usuario confirmó que **ya gira bien** — no se identificó la causa raíz exacta del fallo inicial (podría haber sido el DIP no estable en `0011` en ese momento, o la prueba con batería baja de más abajo), pero quedó descartado que sea un bug de código.

**Valores del giro de 90° confirmados con batería llena (2026-09-23):** el usuario reconfirmó con batería llena — `parametros[5]=85` (delay 90° izquierda, ajustado desde el `100` tentativo de la prueba con batería baja) y `parametros[8]=100` (delay 90° derecha, se mantiene el mismo valor que ya estaba probado). Estos quedan como los valores buenos para ese giro.

**Llevado a `#define` permanente en `globals.h` (2026-09-23):** a pedido del usuario, se actualizaron los defaults en el bloque `MBARETECH_2`: `TURN_LEFT_90_DELAY` de `75` a `85`, y `TURN_RIGHT_90_DELAY` de `75` a `100`. En cada línea se dejó un comentario con el valor anterior y la fecha/motivo (`// antes 75 -- calibrado en calibracion.cpp (0011) con bateria llena, 2026-09-23`), sin borrar los comentarios históricos que ya tenían (referencias a pruebas contra "charizard"). Ahora estos valores son el punto de partida de fábrica incluso después de un reflash — ya no dependen de mandarlos por BLE cada vez. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

**Delays de 45°/90° recalibrados con batería cargada (2026-09-25):** el usuario probó `0010`/`0011` con batería cargada y dio como "oficiales" cuatro valores nuevos: `parametros[4]=55` (45° izq), `parametros[7]=45` (45° der), `parametros[5]=80` (90° izq), `parametros[8]=70` (90° der). A pedido explícito ("cargalos tambien en nuestro codigo de prueba y de competencia"), se actualizaron los `#define` en `globals.h` (bloque `MBARETECH_2`): `TURN_LEFT_45_DELAY` `60`→`55`, `TURN_RIGHT_45_DELAY` `60`→`45`, `TURN_LEFT_90_DELAY` `85`→`80`, `TURN_RIGHT_90_DELAY` `100`→`70`. **Importante:** `TURN_LEFT_90_DELAY`/`TURN_RIGHT_90_DELAY` ya habían sido fijados como "confirmados con batería llena" el 2026-09-23 en `85`/`100` — esta recalibración del 25/09 los **supera/reemplaza**; se dejó explícito en el comentario del código para no perder el rastro de que hubo dos rondas de calibración. Como `calibracion.cpp` y `tasks.cpp` inicializan `parametros[]` desde el mismo `globals.cpp`/`globals.h`, un solo cambio en los `#define` alcanza para que ambos códigos (prueba y competencia) usen los valores nuevos — no hay `#define` separados por build. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

**Placeholder para los 2 sensores de línea traseros (2026-09-29):** el usuario quiere agregar los 2 sensores delanteros (ya en el reporte) y los 2 traseros al reporte de `calibracion.cpp`. Antes de tocar el reporte, surgió un hallazgo de hardware: los pines traseros definidos hoy (`LINE_BACK_LEFT`=GPIO19, `LINE_BACK_RIGHT`=GPIO20) **están en conflicto real con otros usos activos** — GPIO19 es el mismo pin que `DIPE`, y GPIO20 es el mismo pin que `PIN_B1` (salida de dirección del motor derecho, `motor.h`), este último más grave por ser una salida activa, no otra entrada. El usuario decidió puentear los traseros a pines distintos (y sacar un divisor resistivo que tenían pensado usar, discusión aparte) pero todavía no tiene los pines nuevos definidos.

Para dejar el código "terminado" sin poder usar los pines reales todavía, se armó con un sentinel: `#define LINE_BACK_INSTALLED 0` en `globals.h`, al lado de `LINE_BACK_LEFT`/`LINE_BACK_RIGHT`. Mientras esté en `0`:
- `lineSensorsInit()` (`src/lineSensor.cpp`) no configura la atenuación de ADC2 en esos pines (evita tocar GPIO19/20 en el arranque).
- `calibracion.cpp` no lee ni agrega los traseros al reporte.

Se agregaron `checkLineSensorc`/`checkLineSensord` en `src/lineSensor.cpp` (mismo filtro de 7 lecturas que `checkLineSensora`/`b`, para los traseros izq/der respectivamente) y sus declaraciones + la de `readLineSensorBack` (ya existía la función pero no estaba declarada en `globals.h`, nadie la usaba) en el header. Se renombraron los campos del reporte de `LINEA_IZQ`/`LINEA_DER` a `LINEA_DEL_IZQ`/`LINEA_DEL_DER` (para no confundir con los traseros una vez agregados) y se agregó `LINEA_TRAS_IZQ`/`LINEA_TRAS_DER` al mensaje, envueltos en `#if LINE_BACK_INSTALLED`.

`pio run` — compila OK, **mismo tamaño exacto que antes** (Flash 28.1%/939317 bytes, RAM 13.9%), confirmando que todo el código nuevo queda inerte mientras el sentinel esté en `0`. **Pendiente:** cuando el usuario confirme los pines nuevos después de puentear, actualizar `LINE_BACK_LEFT`/`LINE_BACK_RIGHT` a esos valores y poner `LINE_BACK_INSTALLED` en `1` — no hace falta tocar nada más del código para que el reporte y la lectura se activen.

**Reporte de línea con texto `BLANCO`/`negro` en vez de `0`/`1` (2026-09-29):** al usuario le gustó mucho el formato de `pruebaLinea.cpp` ("directo y sin drama, blanco y negro") y pidió aplicar ese mismo criterio al reporte de `calibracion.cpp`, **solo para los sensores de línea** (los IR se quedan en `0`/`1`, no tienen un equivalente natural a blanco/negro). Se agregó un helper `lineaTexto(bool)` que devuelve `"BLANCO"`/`"negro"`, usado en los 4 campos de línea del mensaje (`LINEA_DEL_IZQ`/`DER` ya activos, `LINEA_TRAS_IZQ`/`DER` todavía detrás del sentinel `LINE_BACK_INSTALLED`). `pio run` — compila OK (Flash 28.1%, +40 bytes por el texto nuevo, RAM sin cambios). Documentado en `Mbaretech2026/CLAUDE.md`.

**Traseros puenteados y activados (2026-09-29):** el usuario terminó de puentear los sensores de línea traseros a pines libres, evitando los conflictos de `LINE_BACK_LEFT`/`RIGHT` originales (GPIO19/20 = `DIPE`/`PIN_B1`): `LS3` (trasero izquierdo) → **GPIO12** (antes `ENCODER_RIGHT`, nunca usado), `LS4` (trasero derecho) → **GPIO14** (antes `ENCODER_LEFT`, ídem). Confirmó que los encoders no se usan, así que reutilizar esos pines no tiene costo. Nomenclatura confirmada con el usuario: `LS1`=delantero izquierdo, `LS2`=delantero derecho, `LS3`=trasero izquierdo, `LS4`=trasero derecho.

Se actualizó `globals.h`: `LINE_BACK_LEFT` pasó de `ADC2_CHANNEL_8` a `ADC2_CHANNEL_1` (GPIO12), `LINE_BACK_RIGHT` de `ADC2_CHANNEL_9` a `ADC2_CHANNEL_3` (GPIO14), y `LINE_BACK_INSTALLED` de `0` a `1` — con esto solo, sin tocar `calibracion.cpp` ni `lineSensor.cpp` (ya estaban listos detrás del sentinel desde el cambio anterior), se activan la configuración de ADC2, la lectura de los traseros y los campos `LINEA_TRAS_IZQ`/`LINEA_TRAS_DER` en el reporte. Los comentarios de `ENCODER_LEFT`/`ENCODER_RIGHT` se actualizaron para dejar constancia de que esos pines fueron repurpuestos. `pio run` — compila OK (Flash 28.1%, +~1000 bytes respecto a la versión inerte, RAM sin cambios) — confirma que el código de los traseros ya está generando instrucciones reales. **Pendiente:** probar en el robot que los 4 sensores de línea (2 delanteros + 2 traseros) reporten bien, y confirmar que `LS1`/`LS2` (nomenclatura del usuario) efectivamente corresponden a `LINE_FRONT_LEFT`/`LINE_FRONT_RIGHT` como se asumió.

**⚠️ HALLAZGO DE HARDWARE — posible sobretensión en los traseros (2026-09-29):** con el fix del bug de `-1` y el print de valores crudos ya andando, el usuario compartió una tanda de lecturas reales. Delanteros se comportan normal (negro ~3350-3850, blanco ~150-200, sin saturar). **Traseros dan exactamente `4095` en reposo (el máximo absoluto de un ADC de 12 bits) y caen correctamente a ~150-190 al detectar blanco.** Un `4095` fijo es la firma clásica de un ADC saturado — la entrada está por encima de lo que el rango configurado (`ADC_ATTEN_DB_12`, ~0-3.9V) puede representar. Esto coincide con lo que el usuario había contado antes de puentear: esos pines originalmente iban a llevar una señal de **5V DC** con un divisor resistivo para bajarla a 3.3V, pero luego decidieron **sacar las resistencias y puentear directo**. Se le pidió al usuario medir con multímetro antes de seguir.

**Cerrado — no hay sobretensión (2026-09-29):** el usuario midió con multímetro: **3.3V en reposo (negro), 0.3V al detectar blanco.** 3.3V es justo el riel de alimentación del ESP32-S3, no lo excede — sin riesgo de daño al pin. El `4095` saturado se explica por una característica conocida del ADC del ESP32 (las lecturas cerca de VDD se comprimen/saturan con `ADC_ATTEN_DB_12`, incluso sin exceder el voltaje real de alimentación) — no es un cable mal puesto.

**Calibración real de `THRESHOLD` (2026-09-29):** revisando los datos crudos compartidos por el usuario, se encontró el problema de fondo: los valores de "blanco" de los 4 sensores (`DEL_IZQ`=145,151 · `DEL_DER`=155,169,199 · `TRAS_IZQ`=153,159,179 · `TRAS_DER`=153,186) caían casi todos **por encima** del `THRESHOLD=145` vigente — solo un `145` exacto tocaba el límite. Como el filtro de combate exige 7 lecturas seguidas `<= THRESHOLD` para confirmar "blanco", con la mayoría de las muestras reales en 150-199 el filtro casi nunca se activaba, aunque el sensor sí estuviera viendo la línea. El negro nunca baja de ~2793 (delantero) / ~3721 (trasero), así que hay margen de sobra para subir el umbral. A pedido del usuario, se actualizó `#define THRESHOLD` en `globals.h` (bloque `MBARETECH_2`) de `145` a `250` — deja margen cómodo sobre el blanco más alto visto (199) sin acercarse al negro más bajo (2793). **Importante:** esta constante también la usa `tasks.cpp` para el `LINE_RETREAT` de combate real, así que el cambio no es solo cosmético para calibración — afecta la detección del borde del dohyo en combate. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

**Filtro de línea hecho simétrico, `THRESHOLD` vuelto a `250` (2026-09-29):** se probó `THRESHOLD=500` para ver si el retraso al detectar blanco mejoraba (teoría: ruido cruzando el umbral a mitad de las 7 lecturas seguidas que pedía el filtro, reseteando el contador). En vez de seguir por ese camino, el usuario pidió eliminar directamente la asimetría del filtro: que blanco y negro se confirmen con una sola lectura por igual, como ya pasaba con negro (una sola lectura por encima del umbral resetea el contador a 0 de inmediato, contra las 7 lecturas seguidas que pedía para confirmar blanco). Se revirtió `THRESHOLD` a `250` y se reescribieron `checkLineSensora`/`b`/`c`/`d` (`lineSensor.cpp`): se sacó el contador estático de 7 lecturas, ahora son una comparación directa de una sola muestra (`return measurement <= THRESHOLD;`). `pio run` — compila OK (Flash 28.1%, RAM 13.9%, levemente menos flash sin la lógica del contador). **Importante — impacto en combate real:** esta es la misma función que usa `tasks.cpp` para `LINE_RETREAT`; sin el filtro de 7 lecturas, un solo pico de ruido puede disparar (o dejar de disparar) el retroceso de línea en combate, en cualquiera de los dos sentidos. Antes el filtro protegía contra falsos positivos puntuales; ahora la detección es más rápida pero más sensible a ruido. A tener en cuenta si aparecen falsos positivos de línea en un match real.

**`0100` nuevo — junta `0010`+`0011` (2026-09-25):** con los primeros cuatro combos ya validados en banco (`0000` reporte, `0001` avance recto, `0010` giro 45° bilateral, `0011` giro 90° bilateral), el usuario pidió combinar `0010` y `0011` en un combo nuevo. Se agregó `case 4` a `calibracion.cpp`: revisa los 4 sensores de giro en un único combo (`TOP_LEFT`/`TOP_RIGHT` → 45°, `SIDE_LEFT`/`SIDE_RIGHT` → 90°, nada detectado → frena), reutilizando exactamente los mismos `parametros[]` que ya usan los combos individuales — no se agregó ningún valor nuevo. **Nota para el futuro:** este combo queda, por ahora, funcionalmente idéntico a `0111` ("seguir sin atacar 1"), ya que ese combo todavía no le suma nada por encima (ninguno de los dos toca `SHORT_LEFT`/`SHORT_RIGHT`) — si en algún momento se quiere diferenciarlos, revisar si conviene consolidar en uno solo. `0101` queda como el único combo libre de ese par ahora. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

### 2. Máquina de Estados (`tasks.cpp`)
El cerebro del robot es una tarea de FreeRTOS que opera bajo las siguientes prioridades:
1. **Supervivencia (`LINE_RETREAT`):** Si toca la línea blanca y no tiene al enemigo pegado, retrocede, gira 180° y entra en modo defensivo ("Turkish").
2. **Cacería (`BRAKE`):** Estado de decisión. Lee sensores laterales para girar, o el sensor central para atacar.
3. **Ataque (`FORWARD`):** Empuja al 80% (`FORWARD_80`). Si pasan **200ms** (corregido: el código actual usa 200ms, un comentario inline indica que antes era 500ms) con el sensor central activo, incrementa `delta` en +5 (aunque `delta` actualmente no se suma a `local_speed`, con lo cual el incremento no tiene efecto real sobre la velocidad — ver nota abajo). Si los sensores cortos (`SHORT_LEFT` y `SHORT_RIGHT`) se activan simultáneamente, inyecta el 100% de potencia (`MAX_SPEED`, capado a ~97% real por el driver) asumiendo impacto inminente.
4. **Giros Abiertos:** Los estados como `TURN_LEFT_90` giran ciegamente usando `while(!elapsedTime(delay))`. No leen sensores mientras giran (a menos que `-DCANCEL_TURNS` esté activado en compilación).

**Calibración por BLE portada a `tasks.cpp` (2026-09-17):** `TURN_LEFT_45`, `TURN_LEFT_90`, `TURN_RIGHT_45`, `TURN_RIGHT_90`, `TURN_180`, y los tramos de giro dentro de `L_MOVEMENT_45`/`R_MOVEMENT_45` ya **no leen los `#define` fijos** (`TURN_LEFT_SPEED`, `TURN_RIGHT_SPEED`, `TURN_LEFT_45_DELAY`, `TURN_LEFT_90_DELAY`, `TURN_RIGHT_45_DELAY`, `TURN_RIGHT_90_DELAY`, `TURN_LEFT_180_DELAY`, `CORRECT_SPEED`) — ahora leen `parametros[3]`, `[6]`, `[4]`, `[5]`, `[7]`, `[8]`, `[9]` y `[18]` respectivamente, igual que ya hacía `movements.cpp`. Esto permite tunear velocidad de giro, delays y el sesgo de corrección por BLE **sin recompilar**, en el build activo. Build verificado con `pio run` — compila OK.

* **Los `#define` siguen existiendo en `globals.h`** — ahora solo sirven para inicializar el array `parametros[]` en `globals.cpp` al arrancar (los valores por defecto no cambiaron).
* **El resto de `parametros[]` sigue sin usarse en `tasks.cpp`** (`THRESHOLD`, tiempos de "turkish", delays de movimiento corto, delays de `GIRO_U_*`) — esos índices solo hacen algo en el `movements.cpp` viejo. `parametros[2]` ("vel forward") pasó a estar en uso — ver `TEST_FORWARD` abajo.
* **Falta el disparador manual de estado:** a diferencia de `movements.cpp` (`parametros[0]`/`parametros[1]` fuerzan cualquier estado a pedido), `tasks.cpp` no tiene forma de forzar un giro puntual por BLE — para calibrar hay que llegar a él por el camino normal (DIP switches de apertura, o presentando un objeto al sensor IR correspondiente durante una corrida real con `startSignal` activo).

**Nuevo estado `TEST_FORWARD` (2026-09-17, corregido el mismo día):** se agregó al enum `State` (`globals.h`) y a `tasks.cpp` un estado de prueba de banco **totalmente sordo a sensores**: ambos motores adelante a `parametros[2]`% en cada vuelta del loop (se puede cambiar en caliente por BLE mandando `"2 <valor>"`, sin recompilar ni resetear), y solo obedece al kill switch (`!startSignal` → vuelve a `IDLE` y frena). **Corrección aplicada:** el chequeo de línea blanca que corre antes del `switch` en cada vuelta del loop (independiente del estado) originalmente podía interrumpir igual `TEST_FORWARD` mandándolo a `LINE_RETREAT` — se agregó `currentState != TEST_FORWARD` a esa condición para que este modo ignore por completo tanto los 7 IR como los sensores de línea. **Advertencia:** si se prueba este modo cerca del borde del dohyo, el robot ya NO frena solo al pisar la línea — depende 100% de que se corte el `startSignal` a mano. **El combo de DIP `0000` fue repurposado** para disparar este estado en vez del `FORWARD` de combate — ver la fila `0000` de la tabla de arriba para cómo revertirlo.

## Misiones / Tareas Pendientes
* [ ] **Comprensión Profunda:** Terminar de analizar cada rama lógica de `tasks.cpp` y su relación con `movements.cpp`.
* [ ] **Validación Física:** Realizar la primera prueba con el robot real para verificar que los motores y sensores responden según lo esperado.
* [ ] **Mini-Calibración:** Ejecutar una prueba de movimiento controlada para ajustar los primeros parámetros de velocidad y giro en pista.
