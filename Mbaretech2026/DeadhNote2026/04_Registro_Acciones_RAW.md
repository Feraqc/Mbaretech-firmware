# Registro de Acciones (RAW)

Historial detallado y automático de todos los pequeños cambios, ajustes de parámetros, revisiones y correcciones menores en el código y documentación del Megasumo Mbaretech2.
> Las IAs tienen la directiva de actualizar este archivo automáticamente tras modificar o auditar el proyecto.

## Historial

---
**[2026-05-13 | 00:50 | Madrugada]**

* **Auditoría Inicial de Código:** Se revisaron los archivos `tasks.cpp`, `main.cpp`, `sensorsTest.cpp`, `globals.h` y `platformio.ini` para entender la arquitectura base (FreeRTOS, lazo abierto vs cerrado, uso de pines).
* **Diagnóstico de Hardware:** Se cruzaron los comentarios de los tests antiguos con la máquina de estados, deduciendo el bug de cableado invertido entre `TOP_MID` y `SHORT_RIGHT`.
* **Análisis de Calibración:** Se debatió la táctica de calibración estática (la "prueba de la escoba" anulando el estado `FORWARD`) y su impacto negativo en competición por culpa de variables como batería o fricción.
* **Evaluación de Tácticas Enemigas:** Simulamos paso a paso cómo el código actuaría ante un señuelo de bandera enemigo, descubriendo que el robot atacaría a ciegas.
* **Registro de Soluciones:** Se estructura la documentación oficial (`DeathNote`). Todo el desglose técnico, el razonamiento profundo y las soluciones propuestas para estos problemas (ej. implementar MPU6050, validación geométrica) **han sido referenciados y explicados en detalle en el archivo `99_Razonamientos_y_mejoras.md`**.

---
**[2026-05-13 | 01:10 | Madrugada]**

* **Actualización de Reglas de IA:** Se modificó `01_Reglas_IA.md` para prohibir la edición de entradas pasadas en bitácoras y establecer el flujo de actualización de razonamientos (comentarios entre paréntesis) y estado actual (sección inmutable de objetivos).
* **Reestructuración de Estado Actual:** Se actualizó `02_Estado_Actual.md` para incluir la sección de "Objetivos Importantes Cumplidos" con fechas de referencia.

---
**[2026-05-13 | 01:15 | Madrugada]**

* **Gestión de Tareas:** Se añade la sección "4. Misiones / Tareas Pendientes" en `02_Estado_Actual.md` para trackear los siguientes pasos operativos (Comprensión del código y validación física).
* **Sincronización de Reglas:** Se observa que el usuario limpió la sección 3 de `01_Reglas_IA.md` para simplificar el documento.

---
**[2026-05-13 | 01:25 | Madrugada]**

* **Optimización de Interacción:** Se añade la regla de **Prohibición de Redundancia** en `01_Reglas_IA.md`. Esta obliga a cualquier IA a auditar toda la DeathNote antes de actuar para evitar repeticiones de análisis o errores ya documentados.

---
**[2026-09-17 | 13:13 | Tarde]**

* **Migración de Carpeta:** Esta DeathNote fue copiada desde el proyecto `Mbaretech2` (código anterior) hacia `Mbaretech2025` (código más reciente, tratado como proyecto totalmente independiente, no sincronizado). Se creó `Mbaretech2025/CLAUDE.md` propio y se recortó el `CLAUDE.md` de la raíz del repo para que ya no describa este proyecto.
* **Auditoría de Veracidad:** Se verificó `02_Estado_Actual.md` contra el código actual de `Mbaretech2025` (`tasks.cpp`, `globals.h`, `sensorsTest.cpp`). Correcciones aplicadas: (1) el ramp de potencia en `FORWARD` decía 500ms, el código usa 200ms, y además se descubrió que el `delta` calculado nunca se suma a `local_speed` (código muerto); (2) la sección de DIP switches describía "un switch = un modo" (heredado de `Mbaretech2`), pero el código real usa una tabla de 16 combinaciones con `DIPA/B/C/E`, con `DIPD` muerto en la decisión; (3) el cruce de pines IR documentado (`TOP_MID`↔`SHORT_RIGHT`) se confirmó, y se amplió con más swaps que aparecen en los comentarios de `sensorsTest.cpp` (`SIDE_LEFT`, `SHORT_LEFT`, `TOP_RIGHT`, `SIDE_RIGHT`). No se tocaron `03` ni `04` (inmutables) ni las entradas de `99` (siguen todas vigentes, ninguna resuelta).
* **Nueva Sección:** Se agregó "4. Tabla Completa de Pines" a `02_Estado_Actual.md` — no existía un listado exhaustivo de GPIOs, solo menciones puntuales de IR/DIP al hablar de bugs.

---
**[2026-09-17 | 13:15 | Tarde]**

* **Excepción de Orden Autorizada por el Usuario:** Se movió la Tabla Completa de Pines al encabezado de `02_Estado_Actual.md`, por delante de "Objetivos Importantes Cumplidos". Esto contradice literalmente `01_Reglas_IA.md` (que exige que Objetivos sea siempre lo primero), pero el usuario autorizó explícitamente esta excepción puntual por la importancia crítica de los pines para el trabajo de hardware. Se dejó una nota inline en el propio documento aclarando que es una excepción autorizada, no un incumplimiento accidental.

---
**[2026-09-17 | 13:26 | Tarde]**

* **Tabla Completa de DIP Switches:** Se reemplazó la sección 3 ("Estrategias Pre-Combate") de `02_Estado_Actual.md` por la tabla de las 16 combinaciones (`DIPE·DIPA·DIPB·DIPC`) leída directamente de `tasks.cpp`, `case IDLE`, incluyendo qué estado/flags fija cada una. Se confirmó que solo 4 de los 5 pines DIP físicos (`DIPD` queda fuera) participan en la decisión.
* **Hallazgos adicionales durante la verificación:** (1) `turkish` también cambia el comportamiento de `FORWARD` al perder al enemigo (frena antes de `BRAKE` solo si `turkish` está activo); (2) el `case BRAKE` de `tasks.cpp` tiene las llamadas a `leftMotor.brake()`/`rightMotor.brake()` comentadas — el estado no frena motores pese al nombre; (3) la variable `bandera`, escrita en `true` por varias combinaciones de DIP, no la lee nadie más en el archivo — código muerto, sumado a `DIPD` y al `delta` de `FORWARD` ya documentados.

---
**[2026-09-17 | 13:28 | Tarde]**

* **Segunda Excepción de Orden Autorizada por el Usuario:** Se movió también la tabla de DIP switches al encabezado de `02_Estado_Actual.md`, justo debajo de la Tabla Completa de Pines y por delante de "Objetivos Importantes Cumplidos" — misma excepción a `01_Reglas_IA.md` ya autorizada antes, ahora extendida a esta segunda tabla por pedido del usuario. Se sacó la sección duplicada de "Actualmente" y se actualizó la nota de excepción inline para cubrir ambas tablas.

---
**[2026-09-17 | 13:50 | Tarde]**

* **CAMBIO DE CÓDIGO — Calibración por BLE portada a `tasks.cpp`:** a pedido del usuario, se modificó `src/tasks.cpp` (`replace_all` por identificador, sin tocar `globals.h`/`globals.cpp`) para que `TURN_LEFT_45`, `TURN_LEFT_90`, `TURN_RIGHT_45`, `TURN_RIGHT_90`, `TURN_180` y los tramos de giro de `L_MOVEMENT_45`/`R_MOVEMENT_45` lean `parametros[3]`, `[6]`, `[4]`, `[5]`, `[7]`, `[8]`, `[9]` y `[18]` en vez de los `#define TURN_LEFT_SPEED`/`TURN_RIGHT_SPEED`/`TURN_LEFT_45_DELAY`/`TURN_LEFT_90_DELAY`/`TURN_RIGHT_45_DELAY`/`TURN_RIGHT_90_DELAY`/`TURN_LEFT_180_DELAY`/`CORRECT_SPEED`. Los estados muertos/inalcanzables (`TURN_LEFT_45_IF`, `TURN_RIGHT_45_IF`, `TURN_LEFT_90_IF`, `TURN_RIGHT_90_IF`, `MOVEMENT_45` — verificado que ningún `currentState =` los alcanza) también quedaron con la sustitución por ser `replace_all`, sin efecto real ya que no se ejecutan. Se corrió `pio run` — compila OK (Flash 27.5%, RAM 13.9%). Documentado en `02_Estado_Actual.md` sección "Máquina de Estados" y en `Mbaretech2025/CLAUDE.md` sección "BLE communication". Se dejó pendiente de decisión del usuario si también se quiere portar un disparador manual de estado (equivalente a `parametros[0]`/`[1]` de `movements.cpp`) para poder forzar un giro puntual sin depender de sensores/DIP.

---
**[2026-09-17 | 14:22 | Tarde]**

* **CAMBIO DE CÓDIGO — Nuevo estado `TEST_FORWARD` para prueba de banco:** a pedido del usuario, se agregó `TEST_FORWARD` al enum `State` (`include/globals.h`, al final de la lista) y un nuevo `case TEST_FORWARD` en `src/tasks.cpp` (ubicado justo después de `case FORWARD`, antes de `case BACKWARD`): ambos motores adelante a `parametros[2]`% en cada vuelta del loop (sin bloquear con `while`, así una escritura BLE a `parametros[2]` se aplica casi al instante), respeta `startSignal` (vuelve a `IDLE` si se corta) y queda bajo los flags `ESTADOS_ORDEN`/`FORWARDON` como el resto de los estados.
* **Repurposeo del combo DIP `0000`:** en `case IDLE`, la rama `0000` que antes hacía `currentState = FORWARD;` (avance recto de combate) ahora hace `currentState = TEST_FORWARD;`. Esto es un cambio de comportamiento de combate temporal — documentado en `02_Estado_Actual.md` (tabla de DIP, fila `0000` marcada con ⚠️) y en `Mbaretech2025/CLAUDE.md`, con instrucción de cómo revertirlo antes de un match real.
* Se corrió `pio run` de nuevo tras el cambio — compila OK (Flash 27.5%, RAM 13.9%, sin cambios de tamaño relevantes).

---
**[2026-09-17 | 14:26 | Tarde]**

* **CORRECCIÓN — `TEST_FORWARD` ahora ignora también la línea blanca:** el usuario notó que el chequeo de `LINE_RETREAT` corre antes del `switch` en cada vuelta del loop, sin importar el estado activo, y podía interrumpir `TEST_FORWARD` igual que en combate. Se agregó `currentState != TEST_FORWARD` a esa condición en `src/tasks.cpp` para que el modo de prueba quede sordo a los 7 sensores IR **y** a los sensores de línea, respondiendo únicamente a `startSignal`. Se documentó la advertencia de seguridad correspondiente (no frena solo cerca del borde) en `02_Estado_Actual.md` y `Mbaretech2025/CLAUDE.md`. `pio run` — compila OK.

---
**[2026-09-22 | 17:12 | Tarde]**

* **Sesión de pruebas físicas (retomada tras varios días):** el usuario confirmó que el motor test de banco (`TEST_FORWARD`, DIP `0000`) funciona sin problemas — drivers y motores verificados. El bloqueo de `"waiting for download"` de la sesión anterior no volvió a aparecer; se descartó como causa de hardware (el sketch mínimo "Hola Mundo" arrancó bien) y no se determinó la causa exacta del episodio puntual, pero quedó resuelto. Se revirtió sin problemas el sketch de diagnóstico y se restauró el firmware completo (`RUN_TASK_TEST` activo de nuevo, `setup()` original con el eFuse write incluido).
* **CAMBIO DE CÓDIGO — Confirmación de escritura BLE:** al pasar a probar el canal Bluetooth (app "BLE Serial nRF" en modo UTF8), el usuario no podía verificar si los comandos `"INDICE VALOR"` llegaban — `onWrite()` no daba ninguna respuesta. Se modificó `include/bluetoothComm.h`:
  1. La característica RX pasó de aceptar solo `PROPERTY_WRITE` a aceptar `PROPERTY_WRITE | PROPERTY_WRITE_NR` — varias apps mandan por defecto "Write Without Response" y el servidor lo rechazaba en silencio, posible causa raíz de que "no hubiera cambio".
  2. `onWrite()` ahora valida el índice contra `ARRAY_PARAMETROS_SIZE` (antes escribía fuera de los límites del array con cualquier índice `>= 0`, riesgo de corrupción de memoria) y responde siempre por notificación BLE (`"OK parametros[N]=V"` o `"ERROR ..."`), más un `Serial.print` del string crudo recibido bajo `DEBUG`.
  * `pio run` — compila OK (Flash 28.2%, RAM 14.0%). Documentado en `02_Estado_Actual.md` (sección de la tabla de `parametros[]`) y en `Mbaretech2025/CLAUDE.md` (sección BLE communication). Pendiente: que el usuario vuelva a subir y confirme si ahora sí ve la respuesta `"OK ..."` en la app.

---
**[2026-09-22 | 17:28 | Tarde]**

* **Confirmado — BLE con respuesta funcionando:** el usuario confirmó que ahora sí recibe la confirmación (`"OK parametros[N]=V"`) en la app "BLE Serial nRF". El fix de `PROPERTY_WRITE_NR` y/o la validación de índice resolvieron el problema de "no había cambio".
* **CAMBIO DE CÓDIGO — Modo de calibración de sensores, separado del código de competencia:** a pedido del usuario ("armamos un código dentro del src el cual se llame Calibración, separado del código de competencia"). Se creó `src/calibracion.cpp` (flag nuevo `RUN_CALIBRACION`): lee los 7 IR + 5 DIP (incluye `DIPD`) + 2 sensores de línea cada 300ms y los manda por BLE y Serial en formato compacto de bits. No toca motores ni lógica de decisión — es puramente de lectura/reporte.
* **REFACTOR — `bluetoothComm.h`/`bluetoothComm.cpp` separados:** requisito técnico para que `calibracion.cpp` pudiera usar `sendData()` sin duplicar símbolos con `main.cpp`. Se movieron las definiciones de `pServer`, `pTxCharacteristic`, `deviceConnected`, `sendData()` y `BLE_UART_Init()` a un `src/bluetoothComm.cpp` nuevo; el header ahora solo declara (`extern`/prototipos). Las clases `MyServerCallbacks`/`MyCallbacks` se dejaron en el header (son inline por ser definiciones de clase, sin riesgo de duplicado).
* **`platformio.ini`:** se comentó el bloque `FIGHT` (combate, `RUN_TASK_TEST`) y se activó un bloque nuevo `CALIBRACION` (`MBARETECH_2`, `RUN_LINE_SENSOR`, `RUN_CALIBRACION`, `DEBUG`). **El build activo hoy es CALIBRACIÓN, no combate** — hay que revertirlo antes de competir o de volver a probar `tasks.cpp`.
* **Hallazgo adicional:** el combo de DIP `1111` está confirmado como libre/redundante (idéntico a `1001`, `GIRO_U_L`, ya que ese estado no lee `snake` ni `turkish`) — anotado en la tabla de DIP switches por si se necesita un modo más a futuro.
* `pio run` — compila OK (Flash 27.6%, RAM 13.8%). Documentado en `02_Estado_Actual.md` (nueva sección "1.b", tabla de DIP) y `Mbaretech2025/CLAUDE.md` (nueva sección "Sensor calibration mode", "BLE communication", "Hardware abstraction").

---
**[2026-09-22 | 17:31 | Tarde]**

* **CAMBIO DE CÓDIGO — `setup()` propio para `calibracion.cpp`:** el usuario propuso desacoplar por completo el modo de calibración del `setup()` de combate en `main.cpp`, que corría sin condición en cualquier modo (incluido `esp_efuse_write_field_cnt(...)`, una quema de eFuse irreversible que no tiene ningún sentido durante una simple lectura de sensores). Se envolvió el `setup()` original de `main.cpp` en `#ifndef RUN_CALIBRACION` (ya no compila en este modo) y se agregó un `setup()` nuevo y mínimo dentro de `calibracion.cpp` (Serial, BLE, `pinMode` de IR1-7/DIPA-E, `lineSensorsInit()`) — sin eFuse, sin motores, sin `attachInterrupt` del `startSignal`. `pio run` — compila OK sin `setup()` duplicado (Flash 27.5%, RAM 13.5%, levemente menor que antes). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md` (sección "Sensor calibration mode").

---
**[2026-09-22 | 17:35 | Tarde]**

* **CAMBIO DE CÓDIGO — Kill switch en modo calibración:** el usuario pidió que `calibracion.cpp` respete el `startSignal` igual que combate ("primordial"). Se agregó `pinMode(START_PIN, INPUT)` + `attachInterrupt` con un ISR propio (`CalibKS_ISR`, no reutiliza el `KS_ISR` de `main.cpp` para mantener el archivo autocontenido) en el `setup()` de `calibracion.cpp`. La lectura de sensores sigue corriendo siempre en `loop()`, pero el envío del reporte por BLE/Serial ahora está adentro de `if (startSignal) { ... }` — primera activación del switch prende el reporte, la siguiente lo apaga. `pio run` — compila OK (Flash 27.5%, RAM 13.7%). Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-22 | 17:42 | Tarde]**

* **CAMBIO DE CÓDIGO — DIP switches como selector de acción en modo calibración:** el usuario pidió que, igual que en combate, los DIP determinen qué se está calibrando en cada momento, sin tocar `tasks.cpp`. Se agregó un dispatcher a `calibracion.cpp` (mismo orden de bits `DIPE·DIPA·DIPB·DIPC` que la tabla de combate) acotado a las 6 acciones que ya tienen `parametros[]` conectado: `0000` solo sensores, `0001` avance recto (`[2]`), `0010`/`0011` giros izq 45°/90° (`[3]`/`[4]`/`[18]`, `[3]`/`[5]`/`[18]`), `0100`/`0101` giros der 45°/90° (`[6]`/`[7]`/`[18]`, `[6]`/`[8]`/`[18]`), `0110` giro 180° (`[3]`/`[9]`/`[18]`). Los combos `0111`-`1111` quedan libres/sin asignar (frenan, no hacen nada). Cada giro corre completo (bloqueante, sin `CANCEL_TURNS` a propósito, para medir el giro puro) y se repite cada ~300ms mientras el combo y `startSignal` sigan activos. `rightMotor.begin()`/`leftMotor.begin()` se agregaron al `setup()` de `calibracion.cpp` (antes no instanciaba uso de motores). El mensaje BLE ahora arranca con `MODO=<nombre>`.
* **Advertencia documentada:** los valores tuneados acá no sobreviven un reflash (`parametros[]` es RAM-only) — hay que reenviarlos por BLE en el build de combate o actualizar los `#define` en `globals.h` para que queden permanentes.
* `pio run` — compila OK (Flash 28.0%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md` (sección "Sensor calibration mode").

---
**[2026-09-22 | 17:58 | Tarde]**

* **CAMBIO DE CÓDIGO — Reporte solo ante cambios, etiquetas en español:** a pedido del usuario ("más formal, solo cuando existen cambios que imprima o envíe, y que sea en español"). En `calibracion.cpp` se agregó comparación contra el último mensaje mandado (`static String ultimoMsg`) — ya no se envía/imprime nada si el mensaje es idéntico al anterior, evitando saturar el canal BLE con lecturas repetidas. Las etiquetas de los sensores de línea pasaron de `L=`/`R=` a `IZQ=`/`DER=`. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md` (sección "Sensor calibration mode").

---
**[2026-09-22 | 18:04 | Tarde]**

* **CAMBIO DE CÓDIGO — Se saca el valor crudo del ADC del reporte de línea:** el usuario reportó que la impresión seguía "sucia"/muy rápida a pesar del cambio anterior. Causa: el mensaje incluía `IZQ=<adc>/<bool>` con el valor crudo del ADC, que fluctúa por ruido eléctrico constantemente aunque nada cambie físicamente — eso rompía la comparación `msg != ultimoMsg`. Se sacó el número crudo; ahora el mensaje solo reporta el booleano ya debounced (`IZQ=0`/`IZQ=1`), pedido explícito del usuario ("saber si está on u off nomás"). `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-22 | 18:11 | Tarde]**

* **CAMBIO DE CÓDIGO — Reporte rehecho: 9 sensores individuales, sin DIP ni MODO:** el usuario pidió ver "todos los sensores, ON/OFF nada más", indicando que el `IR=0000000` comprimido no dejaba distinguir sensores y que el `DIP=` no le servía como info (no es un sensor ambiental). Se reescribió el mensaje en `calibracion.cpp` para listar los 7 IR + 2 línea como pares `NOMBRE=0/1` individuales (`SIDE_LEFT=`, `SHORT_LEFT=`, `TOP_LEFT=`, `TOP_MID=`, `TOP_RIGHT=`, `SHORT_RIGHT=`, `SIDE_RIGHT=`, `LINEA_IZQ=`, `LINEA_DER=`), y se sacaron `DIP=`/`MODO=` del reporte (se eliminó también la variable `modo`, ya sin uso — el dispatcher de acciones por DIP sigue funcionando igual, solo dejó de imprimirse el nombre). `pio run` — compila OK (Flash 28.0%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b", séptima vuelta) y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-17 | 14:59 | Tarde]**

* **Nueva Sección — Tabla de Parámetros BLE (`parametros[]`):** a pedido del usuario, se agregó al encabezado de `02_Estado_Actual.md` (tercera tabla, después de Pines y DIP switches, misma excepción de orden ya autorizada) el listado completo de los 19 índices de `parametros[]`: constante de origen, valor por defecto (leídos de `src/globals.cpp` y `include/globals.h`) y si están realmente conectados a `tasks.cpp` hoy o no. Se confirmó que 9 de 19 índices (`[2]`-`[9]` y `[18]`) están en uso real en el build activo; el resto solo aplica a `movements.cpp` o está completamente muerto (`[15]` `TURKISH_SPEED`, `[16]`/`[17]` `GIRO_U_DELAY`/`GIRO_U_L_DELAY`, ninguno referenciado por su nombre en los estados `GIRO_U_*` de `tasks.cpp`, que usan literales `1000`/`2000` hardcodeados). Se actualizó también la nota de excepción de orden para mencionar esta tercera tabla.

---
**[2026-09-22 | 20:20 | Noche]**

* **CAMBIO DE CÓDIGO — Diagnóstico de DIP switches:** el usuario reportó que el combo `0001` (avance recto) no se activaba, quedándose siempre en `0000`. Se verificó línea por línea que la lectura de `combo` en `calibracion.cpp` (orden de bits `DIPE·DIPA·DIPB·DIPC`, mismos pines, misma polaridad activo-alto) es idéntica a la de `tasks.cpp` — no había discrepancia de código entre ambos archivos. Para diagnosticar si el problema era de cableado/switch físico en vez de código, se agregó un print de depuración solo por Serial (bajo `DEBUG`, no se manda por BLE) que muestra los 5 pines crudos y el `combo` calculado, imprimiendo solo cuando cambia (`DIP crudo E,A,B,C,D = ... -> combo=...`). También se corrigió el bloque de comentarios del encabezado del archivo, que seguía describiendo el formato viejo `MODO=.../IR=.../DIP=...` en vez del formato de 9 sensores individuales vigente. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `Mbaretech2025/CLAUDE.md` (sección "Sensor calibration mode").
* **Hallazgo confirmado (2026-09-23):** con el robot a mano y el monitor serial abierto, el usuario confirmó que `DIPD` está físicamente roto/atascado en `HIGH` — el string se mantiene en `...00001` sin importar la posición de ese switch. Se documentó como confirmado (antes era solo una sospecha). Sigue sin explicar el síntoma original ("no entra a `0001`"), ya que `DIPD` no participa del cálculo de `combo` en ningún archivo.
* **`DIPC` confirmado sano:** el usuario probó ese switch específicamente y confirmó que cambia bien entre `0` y `1`. No hay switch roto bloqueando `0001` — la lectura fija anterior (`C=0`) fue casi seguro porque ese switch estaba en `OFF` en ese momento, no una falla de hardware. Pendiente: confirmar `combo=1` con `DIPC=ON` y el resto en `OFF`, y verificar que el avance recto mueva las ruedas.

---
**[2026-09-23 | 12:15 | Tarde]**

* **CAMBIO DE CÓDIGO — Giros de 45°/90° unificados en bilateral, sensor decide el lado:** el usuario detectó un error de concepto (no de código): asumía que `0010`-`0110` ya respondían a sensores IR igual que `BRAKE` en combate, cuando en realidad eran combos DIP-only (se repetían solos cada 300ms sin mirar ningún sensor, a propósito, para poder tunear un giro sin necesitar un objeto delante del sensor). Se aclaró la diferencia entre los dos modelos de disparo del archivo (DIP-only vs. sensor-gated) y, a pedido explícito, se rediseñaron los giros de 45°/90°:
  - `case 2` (`0010`): pasó de "giro 45 izquierda fijo" a bilateral — mira `TOP_LEFT`/`TOP_RIGHT`, gira hacia el lado que detecte (mismos `parametros[3]`/`[4]`/`[6]`/`[7]`/`[18]`), frena si no detecta nada.
  - `case 3` (`0011`): mismo patrón con `SIDE_LEFT`/`SIDE_RIGHT` para el giro de 90° (`[3]`/`[5]`/`[6]`/`[8]`/`[18]`).
  - `case 4`/`case 5` (`0100`/`0101`, antes los giros fijos a la derecha) se **eliminaron** — quedan libres, capturados por el `default`.
  - `0110` (giro 180°) no se tocó, sigue DIP-only — no existe un sensor único de "atrás" en este robot para gatillarlo automáticamente.
  * Se actualizó el comentario de encabezado del archivo y el comentario del `default` para reflejar los combos recién liberados. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b", undécima vuelta) y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-25 | 10:15 | Mañana]**

* **CAMBIO DE CÓDIGO — Combo `0100` nuevo, junta `0010`+`0011`:** el usuario confirmó que los primeros 4 combos (`0000`-`0011`) ya funcionan bien en banco, y pidió combinar los dos giros bilaterales (`0010` 45° y `0011` 90°) en un solo combo. Se agregó `case 4` a `calibracion.cpp`, entre `case 3` y `case 6`: revisa `TOP_LEFT`/`TOP_RIGHT` (45°) y `SIDE_LEFT`/`SIDE_RIGHT` (90°) en la misma decisión, reutilizando los mismos `parametros[3]`-`[8]`/`[18]` que ya usaban los combos individuales, sin agregar ningún valor nuevo. Se actualizó el comentario de encabezado del archivo (`0100` deja de estar en la lista de libres, ahora solo `0101` sigue suelto) y el comentario del `default`. **Nota dejada en el código y en `02_Estado_Actual.md`:** este combo queda, por ahora, funcionalmente idéntico a `0111` ("seguir sin atacar 1"), ya que ninguno de los dos toca `SHORT_LEFT`/`SHORT_RIGHT` todavía — a revisar si conviene consolidarlos más adelante. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-23 | 10:30 | Mañana]**

* **CAMBIO DE CÓDIGO — Nuevo combo `0111` "seguir sin atacar":** a pedido del usuario, se agregó a `calibracion.cpp` un `case 7` que reutiliza la misma prioridad de decisión que `case BRAKE` de `tasks.cpp` (`TOP_LEFT`/`TOP_RIGHT` → giro 45°, `SIDE_LEFT`/`SIDE_RIGHT` → giro 90°, usando los mismos `parametros[]` que los combos `0010`-`0101`), pero **sin nunca empujar**: `TOP_MID`, `SHORT_LEFT`, `SHORT_RIGHT` (los sensores que en combate dispararían el ataque) y "nada detectado" quedan todos en freno en vez de avanzar. Permite verificar en banco que el robot gira correctamente para encarar un objetivo sin arriesgar un empujón real. Se actualizó el comentario de encabezado del archivo y la tabla de combos DIP en `Mbaretech2025/CLAUDE.md`, dejando `1000`-`1111` como los únicos combos libres ahora. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

---
**[2026-09-23 | 11:05 | Mañana]**

* **CORRECCIÓN DE CÓDIGO — `SHORT_LEFT`/`SHORT_RIGHT` faltaban en el combo `0111`:** el usuario notó que de los 6 sensores IR no-centrales, la primera versión de `case 7` solo cubría 4 con giro (`TOP_LEFT`/`TOP_RIGHT`/`SIDE_LEFT`/`SIDE_RIGHT`) y dejaba `SHORT_LEFT`/`SHORT_RIGHT` frenando junto con `TOP_MID`. Motivo original: en `tasks.cpp`, `SHORT_LEFT_MOVE`/`SHORT_RIGHT_MOVE` (líneas 808-833) no son giros puros, avanzan ambos motores a velocidad distinta (empuje + curva), por eso se habían tratado como "ataque". Se corrigió `calibracion.cpp` para que `SHORT_LEFT`/`SHORT_RIGHT` disparen un giro puro de 45° (mismos `parametros[3]`/`[4]`/`[6]`/`[7]`/`[18]` que `TOP_LEFT`/`TOP_RIGHT`, sin componente de avance), respetando la prioridad de `BRAKE` (`SHORT` antes que 45° antes que 90°). Ahora los 6 sensores no-centrales giran; solo `TOP_MID` o "nada detectado" frenan. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-23 | 11:30 | Mañana]**

* **CAMBIO DE CÓDIGO — Se dividió "seguir sin atacar" en tres combos (`0111`/`1000`/`1001`):** a pedido del usuario, tras confirmar que entendió la diferencia entre pivote puro y el empuje+curva real de `SHORT_LEFT_MOVE`/`SHORT_RIGHT_MOVE`, se reorganizó el tratamiento de `SHORT_LEFT`/`SHORT_RIGHT` en `calibracion.cpp`:
  1. **`case 7` (`0111`) revertido:** `SHORT_LEFT`/`SHORT_RIGHT` vuelven a omitirse por completo (frenan, como "nada detectado") — se sacó el giro de 45° que se les había agregado en el cambio anterior. Solo `TOP_LEFT`/`TOP_RIGHT`/`SIDE_LEFT`/`SIDE_RIGHT` giran en este combo.
  2. **`case 8` (`1000`) nuevo:** igual que `0111`, pero `SHORT_LEFT`/`SHORT_RIGHT` disparan un pivote asimétrico nuevo — una rueda al 90% adelante, la otra al 42% atrás (sin `parametros[18]`), duración ajustable en vivo por `parametros[10]`/`[11]` (`SHORT_LEFT_DELAY`/`SHORT_RIGHT_DELAY`, existían en el array pero nunca los leía `tasks.cpp`) en vez de un valor fijo.
  3. **`case 9` (`1001`) nuevo:** igual que los anteriores, pero `SHORT_LEFT`/`SHORT_RIGHT` reproducen el movimiento real de combate tal cual (90%/42%+`parametros[18]`, ambas ruedas adelante, 80ms fijos) — el único combo del archivo que empuja de verdad. A pedido explícito del usuario, por seguridad, se le agregó un cooldown de **10 segundos por lado**, implementado con timestamps propios por `millis()` (`static unsigned long ultimoShortLeft/ultimoShortRight`) — deliberadamente **sin reutilizar** la función compartida `elapsedTime()` (ya es un singleton global con un solo `static startTime`, usado por todos los giros del archivo; sumarle un segundo propósito ahí habría interferido con las esperas bloqueantes de otros combos). Los pivotes de 45°/90° en los tres combos no tienen restricción de repetición, solo el empuje de `1001`.
  * `1010`-`1111` quedan como los únicos combos libres. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b", décima vuelta) y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-23 | 13:40 | Tarde]**

* **CAMBIO DE CÓDIGO — Giro de 90° calibrado, llevado a `#define` permanente:** tras probar `0011` en banco (primero con batería baja, resultado descartado como no representativo), el usuario confirmó con batería llena los valores finales: `parametros[5]=85` (90° izquierda) y `parametros[8]=100` (90° derecha, sin cambios respecto a la prueba anterior). A pedido explícito, se actualizaron los `#define` correspondientes en `include/globals.h` (bloque `MBARETECH_2`) para que dejen de ser solo un valor de arranque en RAM y pasen a ser el default de fábrica: `TURN_LEFT_90_DELAY` `75`→`85`, `TURN_RIGHT_90_DELAY` `75`→`100`. En cada línea se agregó un comentario con el valor anterior y el motivo (`// antes 75 -- calibrado en calibracion.cpp (0011) con bateria llena, 2026-09-23`), sin tocar los comentarios históricos previos de esa línea. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-25 | 10:40 | Mañana]**

* **CAMBIO DE CÓDIGO — Delays de 45°/90° recalibrados con batería cargada, llevados a `#define`:** el usuario dio cuatro valores "oficiales" tras probar `0010`/`0011` con batería cargada: `parametros[4]=55`, `parametros[7]=45`, `parametros[5]=80`, `parametros[8]=70`. A pedido explícito de cargarlos "en el código de prueba y de competencia", se actualizaron los `#define` en `include/globals.h` (bloque `MBARETECH_2`): `TURN_LEFT_45_DELAY` `60`→`55`, `TURN_RIGHT_45_DELAY` `60`→`45`, `TURN_LEFT_90_DELAY` `85`→`80`, `TURN_RIGHT_90_DELAY` `100`→`70` — cada línea con comentario del valor anterior. Los dos últimos **superan** los valores fijados el 2026-09-23 (`85`/`100`), que en su momento también se habían marcado como "confirmados con batería llena"; se dejó explícito en el comentario que esta recalibración del 25/09 los reemplaza. No hizo falta tocar `calibracion.cpp` ni `tasks.cpp` — ambos inicializan `parametros[]` desde el mismo `globals.cpp`/`globals.h`, así que un solo cambio de `#define` alcanza para los dos builds. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-28 | 18:40 | Tarde]**

* **CAMBIO DE CÓDIGO — Nueva prueba del IMU (`src/tests/pruebaIMU.cpp`, flag `RUN_PRUEBA_IMU`):** a pedido del usuario, primer paso para usar el MPU6050 (ver `99`, puntos 2 y 3). Se revisó primero lo existente: pines `SDA_PIN 15` / `SCL_PIN 16` (`globals.h`); `include/IMU.h` usa la librería DMP de Rowberg (`MPU6050_6Axis_MotionApps20.h`), pero `I2Cdev`/`MPU6050` están comentados en `lib_deps` y `globals.h` tiene `//#include "IMU.h"`. **`src/tests/gyroTest.cpp` (`RUN_GYRO_TEST`) está roto y no compila:** usa `imu`, `cmdQueue`, `imuDataQueue` y `desiredAngle` sin declararlos en ningún lado, nadie crea `imuTask` y mete un `bool` y un `float` en la misma cola. No se tocó. La prueba nueva va por registros directos con `Wire`, sin librerías: escaneo I2C → detección en 0x68/0x69 + `WHO_AM_I` → configuración (±2000 dps, ±8g, DLPF 44Hz) → calibración del bias del giro con el robot quieto → impresión cada 100ms de acel (g), `|a|`, pico de `|a|`, giro (dps), yaw integrado desde gz y temperatura. Comandos Serial: `z` (yaw a 0) y `c` (recalibrar). No mueve motores. Tiene `setup()` propio; se cambió la guarda del `setup()` de combate en `main.cpp` a `#if !defined(RUN_CALIBRACION) && !defined(RUN_PRUEBA_IMU)`. En `platformio.ini`, se comentó el bloque `CALIBRACION` y se activó el nuevo bloque `PRUEBA IMU` (`-DMBARETECH_2 -DRUN_PRUEBA_IMU -DDEBUG`). Se agregó `Wire` a `lib_deps`: el LDF no lo encontraba solo (`fatal error: Wire.h`). `pio run` compila OK. **Pendiente: probar en el robot real.**

---
**[2026-09-28 | 19:05 | Tarde]**

* **PRUEBA EN HARDWARE — El IMU no responde, SDA retenido en LOW:** primera carga de `pruebaIMU.cpp`: se colgaba en el escaneo I2C. Se agregó diagnóstico previo a `Wire` (nivel de SDA/SCL con y sin pull-up interna), recuperación de bus (9 pulsos de SCL + STOP manual), `Wire.setTimeOut(20)`, escaneo a 100kHz y la impresión del código de error por dirección. Resultado (cargado por COM23): `sin pull-up: SDA=0 SCL=0 | con pull-up: SDA=0 SCL=1`. La recuperación no cambia nada (SDA sigue en 0 tras 9 pulsos) y todas las direcciones dan error 5 (timeout). **Conclusión:** no es un esclavo trabado. Que ninguna línea lea HIGH sin pull-up interna indica que las pull-up del módulo (en el esquemático `PCB/MBARETECH.kicad_sch` el IMU es un `MPU-9250_breakout`, sin pull-ups propias en la placa) no están alimentadas: módulo sin VCC, ausente o dañado. Además, SDA está atado a GND o a un riel muerto. **Pendiente de revisión física** (alimentación del módulo con batería, continuidad de SDA a GPIO15 y a GND, soldaduras).

---
**[2026-09-28 | 19:20 | Tarde]**

* **DIAGNÓSTICO — El IMU está alimentado a solo 1.5V:** el usuario midió **1.5V en el VCC del módulo IMU** (debería ser 3.3V). Se trazaron las conexiones en `PCB/MBARETECH.kicad_sch` (componente `IMU1`, `MPU-9250_breakout`, header 1x10): VCC → riel **`+3V3`** directo (sin regulador intermedio), GND → GND, SCL/SDA → etiquetas globales `SCL`/`SDA` (GPIO16/15), AD0 sin conectar (dirección esperada 0x68). Hipótesis: si el resto del riel `+3V3` mide 3.3V, la conexión +3V3 → header del IMU está cortada (pista, soldadura, pin), y el 1.5V es alimentación parásita a través de los diodos de protección de SDA/SCL (pull-up interna del ESP), lo que explica que SDA quede en LOW. Si todo el riel `+3V3` está bajo, el problema es la fuente. **Pendiente:** medir `+3V3` en otros puntos (pin 3V3 del ESP32, conectores de IR/línea) para distinguir los dos casos.

---
**[2026-09-28 | 19:40 | Tarde]**

* **PRUEBA EN HARDWARE — Después de restablecer 3V3 en el IMU, SDA sigue en LOW:** el usuario reportó haber llevado el VCC del IMU a 3.3V. Se repitió la prueba (COM23): mismo diagnóstico `sin pull-up: SDA=0 SCL=0 | con pull-up: SDA=0 SCL=1`, la recuperación de bus no cambia nada. En una corrida el escaneo dio "ACK" en casi todas las direcciones desde 0x09 (falso positivo: con SDA pegado a LOW, el slot de ACK siempre lee 0); en otra solo dio timeouts. Se verificó en `PCB/MBARETECH.kicad_sch` que la etiqueta `SDA` va a `GPIO15` y `SCL` a `GPIO16` del `ESP32-S3-DEVKITC-1` → el mapeo del código es correcto. Que SCL lea 0 sin pull-up interna indica que las pull-up del módulo siguen sin alimentarse (o que el módulo no las tiene). **Pendiente:** medir VCC del IMU con el robot corriendo, resistencia/continuidad SDA–GND con todo apagado, y probar con el módulo retirado.

---
**[2026-09-28 | 19:55 | Tarde]**

* **RESUELTO — IMU funcionando:** el usuario confirmó que la falla era un **problema de conexión del VCC** del módulo IMU (medía 1.5V por alimentación parásita desde SDA/SCL). Con la conexión arreglada, `pruebaIMU.cpp` funciona. No se capturaron valores desde acá (el robot ya estaba desconectado de COM23).

---
**[2026-09-28 | 20:10 | Noche]**

* **CAMBIO DE CÓDIGO — Nueva prueba de sensores de línea (`src/tests/pruebaLinea.cpp`, flag `RUN_PRUEBA_LINEA`):** se revisó lo existente. `src/tests/LSsensorTest.cpp` (`RUN_LS_SENSOR_TEST`) **no compila**: incluye `lineSensor.h`, que no existe; `readLineSensorBack()` no está declarada en ningún header; no tiene `setup()` propio. No se tocó. `calibracion.cpp` solo reporta el booleano filtrado de los 2 delanteros, sin el valor crudo. La prueba nueva tiene `setup()` propio y no mueve motores. Cada 10ms lee los 4 sensores (delanteros ADC1 canales 2/7 = GPIO3/GPIO8; traseros ADC2 canales 8/9 = GPIO19/GPIO20) y cada 100ms imprime por Serial: crudo, min/max desde el último reset, medio=(min+max)/2, `*` si crudo ≤ `THRESHOLD`, y el booleano filtrado de combate de los delanteros (`checkLineSensora/b`, 7 lecturas seguidas). Comando `r` = resetear min/max. **Posible conflicto detectado:** `LINE_BACK_LEFT` = ADC2 canal 8 = **GPIO19 = mismo pin que `DIPE`** en `globals.h`; hay que verificar en la placa qué está conectado ahí. En `main.cpp` se agregó `RUN_PRUEBA_LINEA` a la guarda del `setup()` de combate. En `platformio.ini` se comentó el bloque `PRUEBA IMU` y se activó `PRUEBA LINEA` (`-DMBARETECH_2 -DRUN_LINE_SENSOR -DRUN_PRUEBA_LINEA -DDEBUG`). `pio run` compila OK (Flash 8.7%, RAM 5.7%). Pendiente de probar en el robot.

---
**[2026-09-28 | 20:25 | Noche]**

* **CAMBIO DE CÓDIGO — `pruebaLinea.cpp` más legible:** a pedido del usuario, la impresión pasa de una línea cada 100ms a una **tabla cada 1 segundo** (columnas Sensor/Crudo/Min/Max/Medio/Ve, con "BLANCO"/"negro" en vez de `*`). El muestreo sigue cada 10ms para que el filtro de combate (7 lecturas seguidas) y el min/max se comporten igual que antes. Como un cruce rápido de línea puede caer entre dos impresiones, además del filtro actual se muestra si el filtro **se activó en algún momento del último segundo** (`vioLineaIzq/Der`, se resetea en cada impresión). `pio run` compila OK.

---
**[2026-09-28 | 20:35 | Noche]**

* **CAMBIO DE CÓDIGO — `pruebaLinea.cpp` solo con los delanteros:** el usuario confirmó que el robot **solo tiene instalados los 2 sensores de línea delanteros**; los traseros se omiten por ahora. Se sacó de la prueba toda la lectura de ADC2 (`LINE_BACK_LEFT/RIGHT`, `readLineSensorBack`, configuración de atenuación y manejo de `ERR`); la tabla ahora tiene 2 filas (`DEL_IZQ`, `DEL_DER`). El conflicto GPIO19 = `DIPE` = `LINE_BACK_LEFT` queda sin efecto práctico mientras no se instalen los traseros, pero sigue en `globals.h`. `pio run` compila OK. La carga falló porque COM23 estaba ocupado/denegado (probablemente el monitor serie abierto).

---
**[2026-09-28 | 21:10 | Noche]**

* **`platformio.ini` — vuelta a `CALIBRACION`:** el usuario pidió volver a cargar el modo de calibración de sensores para seguir con la tanda de giros pendiente (180°, `0111`/`1000`/`1001`). Se comentó el bloque `PRUEBA LINEA` (`RUN_PRUEBA_LINEA`) y se descomentó el bloque `CALIBRACION` (`-DMBARETECH_2 -DRUN_LINE_SENSOR -DRUN_CALIBRACION -DDEBUG`), sin tocar `src/calibracion.cpp` ni ningún otro archivo. `pio run` — compila OK (Flash 28.1%, RAM 13.9%, mismo tamaño que la última vez que `CALIBRACION` estuvo activo, confirmando que no se perdió nada del trabajo previo en ese archivo). Documentado en `Mbaretech2025/CLAUDE.md` (nota de flag activo actualizada).

---
**[2026-09-29 | 10:05 | Mañana]**

* **`platformio.ini` — vuelta a `CALIBRACION` (otra vez):** entre la sesión anterior y esta, en trabajo aparte, se había activado un bloque nuevo `BORRAR` (`RUN_BORRAR`, `src/borrar.cpp`: smoke test mínimo, imprime "Hola mundo" por Serial cada 1s) — no documentado antes en `04`, visto solo al re-leer `platformio.ini` y `Mbaretech2025/CLAUDE.md` actualizado externamente. El usuario pidió volver a `CALIBRACION` para seguir con la calibración de giros. Se comentó el bloque `BORRAR` y se descomentó `CALIBRACION`, sin tocar `src/calibracion.cpp`. `pio run` — compila OK (Flash 28.1%, RAM 13.9%, mismo tamaño de siempre). Documentado en `Mbaretech2025/CLAUDE.md` (nota de flag activo actualizada).

---
**[2026-09-29 | 11:20 | Mañana]**

* **HALLAZGO DE HARDWARE — Sensores de línea traseros en conflicto de pines:** a pedido del usuario ("añadir en el 0000 los dos sensores de enfrente y los dos de atrás"), se revisó `globals.h` antes de tocar código. `LINE_BACK_LEFT` (ADC2 canal 8) = GPIO19 = mismo pin que `DIPE`; `LINE_BACK_RIGHT` (ADC2 canal 9) = GPIO20 = mismo pin que `PIN_B1` (`motor.h`, salida de dirección del motor derecho) — este segundo caso más grave, es una salida activa, no otra entrada. Se le avisó al usuario antes de escribir nada. El usuario decidió puentear los traseros a pines distintos (y descartar un divisor resistivo que tenían pensado para esos pines, tema aparte) pero todavía no tiene los pines nuevos.
* **CAMBIO DE CÓDIGO — Placeholder para los traseros, sin usar los pines en conflicto:** se agregó `#define LINE_BACK_INSTALLED 0` en `globals.h` junto a `LINE_BACK_LEFT`/`LINE_BACK_RIGHT`, con nota explicando el conflicto. Mientras esté en `0`: `lineSensorsInit()` (`src/lineSensor.cpp`) no configura la atenuación ADC2 en esos pines, y `calibracion.cpp` no lee ni reporta los traseros — todo queda envuelto en `#if LINE_BACK_INSTALLED`. Se agregaron `checkLineSensorc`/`checkLineSensord` a `src/lineSensor.cpp` (mismo filtro de 7 lecturas que `checkLineSensora`/`b`) y se declararon en `globals.h` junto con `readLineSensorBack` (la función ya existía en `lineSensor.cpp` pero nunca se había declarado en el header, así que nadie podía llamarla desde afuera). En `calibracion.cpp`, los campos del reporte `LINEA_IZQ`/`LINEA_DER` se renombraron a `LINEA_DEL_IZQ`/`LINEA_DEL_DER` (para no confundirlos con los traseros una vez agregados) y se sumó `LINEA_TRAS_IZQ`/`LINEA_TRAS_DER` condicionados al mismo sentinel. `pio run` — compila OK, mismo tamaño exacto que antes (Flash 28.1%/939317 bytes, RAM 13.9%) confirmando que el código nuevo no genera nada mientras el sentinel esté apagado. **Pendiente:** cuando el usuario confirme los pines nuevos, actualizar `LINE_BACK_LEFT`/`LINE_BACK_RIGHT` y poner `LINE_BACK_INSTALLED` en `1` — nada más debería hacer falta tocar. Documentado en `02_Estado_Actual.md` (tabla de pines con ⚠️ en las filas afectadas, sección "1.b") y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-29 | 11:35 | Mañana]**

* **CAMBIO DE CÓDIGO — Reporte de línea en texto `BLANCO`/`negro`:** a pedido del usuario, que le gustó el formato de `src/tests/pruebaLinea.cpp` ("directo y sin drama, blanco y negro"), se cambiaron los 4 campos de línea del mensaje de `calibracion.cpp` (`LINEA_DEL_IZQ`/`DER` activos, `LINEA_TRAS_IZQ`/`DER` detrás de `LINE_BACK_INSTALLED`) de `0`/`1` a texto `BLANCO`/`negro`, vía un helper nuevo `lineaTexto(bool)`. **Alcance confirmado con el usuario:** solo los sensores de línea — los 7 IR se quedan en `0`/`1`, ya que "detectado/no detectado" no tiene un equivalente natural a blanco/negro. Se actualizó el comentario de formato en el encabezado del archivo. `pio run` — compila OK (Flash 28.1%, +40 bytes por el texto nuevo, RAM sin cambios). Documentado en `02_Estado_Actual.md` (sección "1.b") y `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-29 | 12:00 | Tarde]**

* **CAMBIO DE CÓDIGO — Prueba mínima del ESP32 (`src/borrar.cpp`, flag `RUN_BORRAR`):** a pedido del usuario, programa temporal que imprime `"Hola mundo"` por Serial (115200) cada 1 segundo, con `setup()`/`loop()` propios, sin motores, BLE ni sensores. En `main.cpp` se agregó `RUN_BORRAR` a la guarda del `setup()` de combate. En `platformio.ini` se comentó el bloque `CALIBRACION` y se activó el bloque `BORRAR` (`-DMBARETECH_2 -DRUN_BORRAR`). `pio run` compila OK. Para volver a calibración: comentar `BORRAR` y descomentar `CALIBRACION`.

---
**[2026-09-29 | 12:10 | Tarde]**

* **CAMBIO DE CÓDIGO — Sensores de línea traseros puenteados y activados:** el usuario terminó de puentear `LS3` (trasero izquierdo) a **GPIO12** y `LS4` (trasero derecho) a **GPIO14** — pines libres (antes `ENCODER_RIGHT`/`ENCODER_LEFT`, confirmado que no se usan encoders), evitando los conflictos que tenían los pines originales (GPIO19=`DIPE`, GPIO20=`PIN_B1`). Nomenclatura confirmada: `LS1`=delantero izq, `LS2`=delantero der, `LS3`=trasero izq, `LS4`=trasero der. Se actualizó `include/globals.h`: `LINE_BACK_LEFT` `ADC2_CHANNEL_8`→`ADC2_CHANNEL_1` (GPIO12), `LINE_BACK_RIGHT` `ADC2_CHANNEL_9`→`ADC2_CHANNEL_3` (GPIO14), `LINE_BACK_INSTALLED` `0`→`1`, y los comentarios de `ENCODER_LEFT`/`RIGHT` para reflejar el repurposeo. No hizo falta tocar `calibracion.cpp` ni `lineSensor.cpp` — ya estaban listos detrás del sentinel desde el cambio anterior (placeholder). `pio run` — compila OK (Flash 28.1%, +~1000 bytes respecto a la versión inerte, confirmando que el código de los traseros ahora genera instrucciones reales; RAM sin cambios). Documentado en `02_Estado_Actual.md` (tabla de pines actualizada sin las advertencias ⚠️, sección "1.b") y pendiente de prueba física.

---
**[2026-09-29 | 12:15 | Tarde]**

* **CORRECCIÓN DE CÓDIGO — Bug de manejo de error en sensores traseros + visibilidad de valores crudos:** el usuario reportó que los sensores de línea no leían correctamente y pidió comparar contra `src/tests/pruebaLinea.cpp` (el que sí funciona). Se encontró un bug real: `readLineSensorBack()` (`lineSensor.cpp`) devuelve `-1` cuando `adc2_get_raw()` falla (el ESP32 puede bloquear ADC2 mientras el radio Wi-Fi/BT está activo, no confirmado si BLE dispara exactamente la misma restricción que Wi-Fi clásico) — pero `-1 <= THRESHOLD` es siempre verdadero, así que un fallo de lectura se malinterpretaba como "ve blanco" y ensuciaba el contador de 7 lecturas de `checkLineSensorc`/`checkLineSensord`. Se corrigió en `calibracion.cpp`: si la lectura da `-1`, se ignora ese ciclo en vez de pasarlo al filtro. Además, como el reporte por BLE no muestra valores crudos (a propósito, para no mandar ruido), se agregó un print solo por Serial (bajo `DEBUG`, cada 1s, no por BLE) con los 4 valores crudos y un `(ERROR lectura)` explícito si `readLineSensorBack()` sigue fallando — mismo espíritu que `pruebaLinea.cpp`. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). **Pendiente:** el usuario debe revisar esta salida para confirmar si el problema es el bloqueo de ADC2 (mostraría `ERROR lectura`) o una calibración de `THRESHOLD` que no le sirve a los traseros.

---
**[2026-09-29 | 12:30 | Tarde]**

* **⚠️ HALLAZGO DE HARDWARE — posible sobretensión en sensores traseros, sin cambio de código:** el usuario pasó una tanda de lecturas del print de debug agregado en el cambio anterior. Delanteros normales (negro ~3350-3850, blanco ~150-200). **Traseros dan `4095` fijo en reposo (saturación de ADC de 12 bits) y caen bien a ~150-190 al detectar blanco.** No aparece ningún `-1`/`ERROR lectura`, así que el fix anterior funcionó y el bloqueo de ADC2 por radio queda descartado como causa — el problema es analógico, no de driver. `4095` fijo es la firma de una entrada por encima del rango que el ADC puede representar (`ADC_ATTEN_DB_12`, ~0-3.9V). Coincide con que esos pines originalmente iban a llevar 5V DC con un divisor resistivo que el usuario después decidió sacar, puenteando directo. Si el reposo real del sensor trasero es ~5V sin dividir, eso supera el máximo absoluto de un GPIO del ESP32-S3 (~3.6V) — riesgo de daño. Se avisó al usuario y se le pidió medir con multímetro el voltaje real en GPIO12/GPIO14 (reposo y con blanco) antes de seguir probando o tocar más código. Sin cambios de código en esta entrada — es una pausa de seguridad, no una corrección de software. Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-29 | 12:45 | Tarde]**

* **Cerrado el hallazgo de sobretensión — sin riesgo:** el usuario midió con multímetro: **3.3V en reposo (negro), 0.3V al detectar blanco** en los sensores traseros. 3.3V no excede el riel de alimentación del ESP32-S3, así que no hay riesgo de daño al pin — el `4095` saturado es una característica conocida del ADC del ESP32 cerca de VDD con `ADC_ATTEN_DB_12`, no un problema de cableado.
* **CAMBIO DE CÓDIGO — `THRESHOLD` recalibrado de `145` a `250`:** revisando los datos crudos que había compartido el usuario, se encontró el problema real detrás de "no me leen bien los sensores": los valores de "blanco" de los 4 sensores (`DEL_IZQ`=145,151 · `DEL_DER`=155,169,199 · `TRAS_IZQ`=153,159,179 · `TRAS_DER`=153,186) caían casi todos por encima del `THRESHOLD=145` vigente, así que el filtro de 7 lecturas seguidas casi nunca confirmaba "blanco" aunque el sensor sí lo viera. El negro nunca baja de ~2793, dejando margen de sobra. A pedido del usuario ("puedes subirlo a 250 para asegurar"), se actualizó `#define THRESHOLD` en `include/globals.h` (bloque `MBARETECH_2`) de `145` a `250`, con comentario explicando el motivo y los datos que lo respaldan. **Importante:** esta constante también la usa `tasks.cpp` para `LINE_RETREAT` en combate real — el cambio afecta la detección del borde del dohyo, no es solo para calibración. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-29 | 13:00 | Tarde]**

* **PRUEBA — `THRESHOLD` subido a `500` para probar teoría del retraso:** el usuario notó que detectar blanco tarda notoriamente, pero volver a negro es casi instantáneo. Se explicó que esto es **asimétrico por diseño** en `checkLineSensora/b/c/d` (`lineSensor.cpp`): confirmar "blanco" exige 7 lecturas seguidas `<= THRESHOLD` (a ~300ms por vuelta de loop con `startSignal` activo, ~2s+ reales), mientras que una sola lectura por encima del umbral resetea el contador a 0 de inmediato — mismo filtro que usa `tasks.cpp` en combate, no es algo introducido en esta sesión. El usuario planteó que con `THRESHOLD=250` (recién calibrado, muy cerca del máximo de blanco observado, ~199) el ruido eléctrico podría cruzar el umbral a mitad de la secuencia de 7 lecturas, reseteando el contador y alargando el retraso mucho más allá del mínimo teórico. Se subió `THRESHOLD` a `500` (bien lejos de cualquier pico de ruido en blanco, marcado explícitamente como prueba en el comentario del código, no como valor definitivo) para descartar/confirmar esa teoría. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). **Pendiente:** el usuario debe probar y confirmar si el retraso mejora; si no mejora, el retraso es inherente al filtro de 7 lecturas en sí, no al `THRESHOLD`.

---
**[2026-09-29 | 13:15 | Tarde]**

* **CAMBIO DE CÓDIGO — Filtro de línea hecho simétrico, `THRESHOLD` vuelto a `250`:** en vez de seguir probando con `500`, el usuario pidió directamente eliminar la asimetría del filtro: que blanco y negro se confirmen con una sola lectura por igual, como ya pasaba con negro. Se revirtió `THRESHOLD` de `500` a `250` (su valor calibrado) en `globals.h`, y se reescribieron `checkLineSensora`/`b`/`c`/`d` en `lineSensor.cpp`: se sacó el contador estático de 7 lecturas consecutivas, ahora cada función es una comparación directa de una sola muestra (`return measurement <= THRESHOLD;`). `pio run` — compila OK (Flash 28.1%, RAM 13.9%, levemente menos flash al sacar la lógica del contador). **Aviso dejado al usuario:** esta misma función la usa `tasks.cpp` para `LINE_RETREAT` en combate real — el cambio también la hace más sensible a ruido puntual ahí (antes el filtro de 7 lecturas protegía contra un solo pico disparando un retroceso falso; ahora una sola lectura alcanza en cualquiera de los dos sentidos). Documentado en `02_Estado_Actual.md` (sección "1.b").

---
**[2026-09-29 | 13:30 | Tarde]**

* **CAMBIO DE CÓDIGO — Nueva prueba mínima de motores (`src/tests/pruebaMotores.cpp`, flag `RUN_PRUEBA_MOTORES`):** a pedido del usuario ("un cpp que mueva las ruedas hacia adelante con el killswitch"), archivo nuevo con `setup()`/`loop()` propios: mientras `startSignal` (killswitch) esté activo, ambos motores van `forward(parametros[2])`; si se corta, `brake()`. ISR propio (`PruebaMotoresKS_ISR`, no reutiliza `KS_ISR` de `main.cpp` ni `CalibKS_ISR` de `calibracion.cpp`, para que el archivo quede autocontenido). Sin sensores ni BLE — el más simple de todos los modos de prueba. Se reutilizó `parametros[2]` (mismo índice que usa `TEST_FORWARD` en `tasks.cpp` y el avance recto de `calibracion.cpp`) en vez de un valor fijo, aunque este archivo no expone BLE para tunearlo en vivo. Se agregó `RUN_PRUEBA_MOTORES` a la guarda del `setup()` de combate en `main.cpp` (ahora excluye `RUN_CALIBRACION`, `RUN_PRUEBA_IMU`, `RUN_PRUEBA_LINEA`, `RUN_BORRAR` y `RUN_PRUEBA_MOTORES`). En `platformio.ini` se comentó el bloque `CALIBRACION` y se activó el nuevo bloque `PRUEBA MOTORES` (`-DMBARETECH_2 -DRUN_PRUEBA_MOTORES -DDEBUG`). `pio run` — compila OK (Flash 9.0%, RAM 6.0% — mucho más liviano que los otros modos, sin BLE ni sensores). Documentado en `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-29 | 13:45 | Tarde]**

* **CONFIRMADO EN HARDWARE — `pruebaMotores.cpp` anda perfecto:** el usuario probó y confirmó que este modo (avance continuo con killswitch, sin DIP ni sensores) gira liso, sin problemas.
* **REPORTE — `0001` de `calibracion.cpp` "tira como patadas":** el mismo usuario reportó que, en cambio, el combo `0001` (avance recto, código no tocado) da un giro discontinuo/a los tirones. Se planteó la hipótesis: la diferencia clave entre ambos archivos es que `calibracion.cpp` recalcula `combo` desde los 5 DIP en cada vuelta del loop, mientras que `pruebaMotores.cpp` no lee ningún DIP. Si algún DIP tiene un contacto flojo (ya sabemos que `DIPD` está roto, podría haber otros marginales), `combo` podría estar saltando de `1` a otro valor y volviendo, y como casi todos los demás combos frenan, eso se traduciría en el motor alternando entre avanzar y frenar — exactamente lo descripto. No se tocó código en este hallazgo, se le indicó al usuario mirar el print de depuración de DIP ya existente (`DIP crudo E,A,B,C,D = ... -> combo=...`) para confirmar si `combo` cambia justo cuando el motor se traba.

---
**[2026-09-29 | 13:50 | Tarde]**

* **CAMBIO DE CÓDIGO — Nueva prueba mínima de DIP switches (`src/tests/pruebaSwitch.cpp`, flag `RUN_PRUEBA_SWITCH`):** a pedido del usuario ("dame un codigo para leer los switches y verificar el estado sin que haga nada... con todos los switches"), para confirmar la hipótesis de arriba de forma más cómoda que mirando el print de `calibracion.cpp` en medio de una prueba de motor. Archivo nuevo con `setup()`/`loop()` propios: lee los 5 DIP cada vuelta, imprime por Serial el estado individual de cada uno (`DIPA=... DIPB=... DIPC=... DIPD=... DIPE=...`) más el `combo` calculado (mismo orden de bits que `tasks.cpp`/`calibracion.cpp`), solo cuando algo cambia. No mueve motores, no usa BLE, no depende de `startSignal` — pensado para mover un switch a la vez y confirmar que cambia el carácter correcto, sin que el ruido de los motores corriendo pueda influir. El usuario mencionó que cree haber arreglado el switch menos significativo que antes no andaba. Se agregó `RUN_PRUEBA_SWITCH` a la guarda del `setup()` de combate en `main.cpp`. En `platformio.ini` se comentó el bloque `PRUEBA MOTORES` y se activó el nuevo bloque `PRUEBA SWITCH` (`-DMBARETECH_2 -DRUN_PRUEBA_SWITCH`). `pio run` — compila OK (Flash 8.5%, RAM 5.7%). Documentado en `Mbaretech2025/CLAUDE.md`. **Pendiente:** el usuario debe mover cada switch uno por uno y confirmar cuál(es) cambian correctamente y cuál no.

---
**[2026-09-29 | 14:05 | Tarde]**

* **RESULTADO — `pruebaSwitch.cpp`: `DIPD` sigue roto, pero A/B/C/E están sanos:** el usuario movió cada switch uno por uno. `DIPA`, `DIPB`, `DIPC` y `DIPE` cambian limpio, cada uno solo afecta su propio carácter y el `combo` correspondiente, sin saltos espontáneos cuando no se toca nada. `DIPD` sigue fijo en `1` sin importar la posición del switch — confirma (no es nuevo) que ese pin sigue roto, pero como no participa del cálculo de `combo`, **descarta la hipótesis de un DIP flojo como causa del tirón del motor en `0001`**. Se le explicó al usuario que, con el DIP descartado, las diferencias reales entre `pruebaMotores.cpp` (liso) y `calibracion.cpp` `case 1` (a los tirones) pasan a ser: lecturas ADC1/ADC2 de línea, armado de un `String` de 9 campos cada vuelta (reserva/libera memoria de heap aunque el mensaje no cambie), y el `sendData()` por BLE cuando sí cambia. Hipótesis principal: pico de corriente del radio BLE al notificar, compartiendo batería/regulador con los motores, causando una caída de tensión momentánea. Se le pidió al usuario probar `0001` con una escena sin cambios (para que `sendData()` nunca se dispare) como prueba aislante sin tocar código. Sin cambios de código en esta entrada.

---
**[2026-09-29 | 14:15 | Tarde]**

* **`platformio.ini` — vuelta a `CALIBRACION`, con desliz corregido:** el usuario pidió volver a cargar el modo de calibración para hacer la prueba de la escena estática. Al descomentar el bloque `CALIBRACION`, se dejó por error la línea `build_flags = ;CALIBRACION...` todavía comentada (con `;` adelante) mientras se descomentaban las líneas de `-D` debajo — como esa línea es la que declara la clave `build_flags`, PlatformIO no la reconoció y absorbió los `-DMBARETECH_2`/`-DRUN_LINE_SENSOR`/`-DRUN_CALIBRACION`/`-DDEBUG` como si fueran parte de `lib_deps` (intentó "instalarlos" como librerías), rompiendo la compilación (`TURN_LEFT_SPEED`/`IR4`/etc. "no declarados" porque `MBARETECH_2` nunca se definió). Se corrigió sacando el `;` de esa línea. `pio run` — compila OK (Flash 28.1%, RAM 13.9%).

---
**[2026-09-30 | 00:10 | Madrugada]**

* **`platformio.ini` — vuelto a `PRUEBA MOTORES` y de nuevo a `CALIBRACION`:** el usuario pidió recargar `PRUEBA MOTORES` "para verificar" (compiló OK, Flash 9.0%/RAM 6.0%, igual que la última vez confirmada lisa en hardware). Después reportó, ya con `calibracion.cpp` probado en el robot: en el combo `0001` **los motores no giran en absoluto** — no es el tirón/discontinuidad reportada antes, ahora es movimiento nulo. Como `pruebaMotores.cpp` (sin DIP, sin BLE, sin sensores) sigue funcionando perfecto con el mismo `parametros[2]`, se descarta un problema de `Motor`/PWM/cableado de motores. Se revisó `case 1` de `calibracion.cpp` línea por línea contra `pruebaMotores.cpp`: llaman al mismo `forward(parametros[2])` con el mismo valor default (`FORWARD_X`=94 vía `globals.cpp`), sin ningún `brake()` interpuesto en el medio del combo 1 — la única forma de que no arranque es que **`combo` nunca llegue a valer `1`**, es decir que el DIP físico que el usuario arma para "0001" no esté generando `dC=1` con `dA=dB=dE=0` como se asume. Esto conecta directo con el pendiente dejado en la entrada de las `13:15`/`135` de este mismo archivo (confirmar `combo=1` con `DIPC=ON` y el resto en `OFF` usando el print de depuración de `calibracion.cpp`, no `pruebaSwitch.cpp` que no toca motores) — ese paso todavía no se había hecho con `calibracion.cpp` corriendo de verdad. Se volvió a activar el bloque `CALIBRACION` en `platformio.ini` (routine de comentar/descomentar, revisada línea por línea para no repetir el desliz del `build_flags` comentado de la entrada anterior). `pio run` — compila OK (Flash 28.1%, RAM 13.9%). **Pendiente:** el usuario debe subir este firmware, abrir el Serial monitor, activar el killswitch, armar lo que cree que es `0001` y copiar la línea exacta `DIP crudo E,A,B,C,D = ... -> combo=...` para confirmar si realmente llega a `combo=1`; si no, el problema es de armado/lectura del DIP, no de lógica de motor.

---
**[2026-09-30 | 00:25 | Madrugada]**

* **CAMBIO DE CÓDIGO — nuevo flag `SKIP_BLE` para aislar BLE como causa del giro intermitente:** el usuario volvió a reportar el síntoma, ahora descrito como "intermitente" otra vez (no ya "no gira nada"), y preguntó directamente si puede ser el bluetooth. Dato clave revisando `src/bluetoothComm.cpp`: `BLE_UART_Init()` deja el radio **advertising de forma continua** (`pServer->getAdvertising()->start()`) desde el `setup()`, independientemente de si hay un celular conectado o no — o sea que el radio BLE genera actividad/consumo periódico todo el tiempo que el combo 0001 está corriendo, no solo cuando `sendData()`/`notify()` se dispara (eso sí está bien gateado por `deviceConnected`). Esa actividad de radio no existe en absoluto en `pruebaMotores.cpp` (nunca llama a `BLE_UART_Init`), así que sigue siendo la diferencia más importante entre ambos archivos. En vez de seguir especulando, se agregó un flag de aislamiento **en el propio `calibracion.cpp`** (no en un archivo de prueba aparte, para no cambiar nada más de la lógica real): `SKIP_BLE` envuelve tanto la llamada a `BLE_UART_Init()` en `setup()` como la llamada a `sendData(msg)` en el loop, dejando el resto (lectura de IR/DIP/línea, cálculo de `combo`, el `switch` completo, los `Serial.print` de depuración) exactamente igual. Se activó `-DSKIP_BLE` en el bloque `CALIBRACION` de `platformio.ini` (con comentario aclarando que es prueba temporal). `pio run` — compila OK (Flash 9.4%, RAM 6.1% — cae casi al nivel de `PRUEBA MOTORES`, confirma que con este flag el radio BLE deja de pesar en tiempo de ejecución). **Pendiente:** el usuario debe subir este firmware (calibración + combo 0001, pero sin BLE) y confirmar si el giro sigue intermitente o queda liso. Si queda liso, BLE (probablemente el advertising continuo, no el notify) queda confirmado como la causa y el siguiente paso es mitigar (bajar potencia de TX, aumentar intervalo de advertising, o directamente no inicializar BLE mientras el combo activo mueve motores). Si sigue intermitente incluso sin BLE, la causa es otra (línea/ADC2, el armado del `String`, o directamente algo eléctrico ajeno al código) y hay que revisar aparte. También se sugirió al usuario, sin necesidad de ningún cambio más de código, revisar si el Serial monitor muestra un banner de reinicio (`rst:0x...` / `ets Jun 8 2016...`) justo en el momento del tirón — eso confirmaría un brownout/reset por caída de tensión en vez de un problema de lógica.

---
**[2026-09-30 | 00:40 | Madrugada]**

* **BLE DESCARTADO — el usuario probó con `SKIP_BLE` y el tirón sigue igual.** Se descarta por completo el radio (advertising incluido) como causa. El usuario pidió analizar a fondo la secuencia del código.
* **Nueva hipótesis líder — ruido del motor acoplado a las líneas del DIP:** con BLE fuera de la ecuación, la diferencia real que queda entre `calibracion.cpp` (a los tirones) y `pruebaMotores.cpp`/`pruebaSwitch.cpp` (ambos lisos, cada uno por separado) es que `calibracion.cpp` es el único archivo que **lee los DIP y mueve los motores al mismo tiempo, en el mismo loop**. `pruebaSwitch.cpp` prueba los DIP con los motores siempre apagados (por eso salieron "sanos" ahí) y `pruebaMotores.cpp` mueve los motores sin leer nunca el DIP. Si el H-bridge (conmutación PWM de 20kHz + transiciones de los pines de dirección) induce ruido eléctrico sobre las líneas del DIP (por cercanía de pistas o retorno de tierra compartido), ese ruido nunca podría haberse visto en ninguna de las dos pruebas anteriores por separado — solo se manifestaría en `calibracion.cpp`, exactamente el síntoma reportado. Si `combo` cambia de `1` a otro valor por una sola vuelta de loop (~300ms) cada vez que el motor conmuta, ese ciclo cae en `default` (frena) y el siguiente vuelve a `case 1` (avanza) — eso es literalmente el tirón/intermitencia descripta desde el principio ("tira como patadas").
* **CAMBIO DE CÓDIGO — el print de depuración de DIP/combo se hizo incondicional y con timestamp:** antes solo imprimía `DIP crudo E,A,B,C,D = ... -> combo=...` cuando `combo` cambiaba respecto a la vuelta anterior; ahora imprime SIEMPRE, una vez por vuelta de loop, con `millis()` al principio de la línea, para poder correlacionar en el tiempo un tirón físico visible con una lectura de `combo` distinta de `1` en el Serial monitor. `pio run` — compila OK (Flash 9.4%, RAM 6.1%, sin cambios relevantes de tamaño). **Pendiente:** el usuario debe subir este firmware (sigue con `SKIP_BLE` activo), activar el combo `0001`, y mientras el motor está a los tirones, mirar el Serial monitor: si aparecen líneas con `combo=0` (o cualquier valor distinto de `1`) intercaladas entre las de `combo=1`, queda confirmado el ruido del DIP inducido por el motor. Si el log muestra `combo=1` de forma continua y estable durante todo el tirón, la hipótesis del DIP queda descartada y hay que mirar en otro lado (posiblemente algo puramente eléctrico en el driver/H-bridge o la batería, no atribuible a ninguna diferencia de código).

---
**[2026-09-30 | 00:55 | Madrugada]**

* **CAMBIO DE CÓDIGO — modelo de 3 etapas del killswitch, el combo ya no se relee del DIP mientras corre:** el usuario propuso, antes de esperar el resultado del print, una arquitectura mas robusta que ya usa `tasks.cpp` en combate real (leer el DIP una sola vez al activar, nunca de nuevo durante la corrida) pero que `calibracion.cpp` nunca implemento -- hasta ahora recalculaba `combo` desde el DIP en CADA vuelta de loop, incluso con el motor corriendo, dejando abierta la ventana a que una lectura mala (ruido del motor, contacto flojo) metiera un frenon de por medio sin importar la causa exacta. Se implementaron 3 etapas explicitas en `calibracion.cpp`: Etapa 1 (`startSignal=false`, "preparar") -- el DIP se lee libre cada vuelta (`comboLive`), sin mover nada, el usuario ve en vivo que combo va a quedar armado. Etapa 2 (flanco de subida de `startSignal`, "ejecutar") -- se congela `combo` a partir del `comboLive` de ESE instante en una variable `static comboCongelado` (con `static bool corriendo` para detectar el flanco), y desde ahi el DIP ya no se vuelve a tocar para nada mientras siga corriendo. Etapa 3 (`startSignal` vuelve a `false`, "detener") -- frena y resetea `corriendo=false`, listo para congelar un combo nuevo la proxima vez. El print de depuración de DIP tambien se actualizo para mostrar ambos valores por separado (`combo_vivo` vs `combo_ejecutando`, este ultimo en `-1` durante la etapa 1) y asi poder ver si el DIP en vivo flickea aunque ya no pueda afectar el movimiento. Esto no depende de confirmar la teoria del ruido del motor sobre el DIP -- la vuelve irrelevante para la ejecucion, sea cierta o no. Se actualizo el comentario de cabecera de `calibracion.cpp` explicando el modelo. `pio run` -- compila OK (Flash 9.4%, RAM 6.1%, sigue con `SKIP_BLE` activo). Documentado en `Mbaretech2025/CLAUDE.md` (seccion "Sensor calibration mode"). **Pendiente:** el usuario debe subir este firmware y confirmar si el combo `0001` ahora corre liso de punta a punta (con el DIP ya inmune a cambios durante la corrida). Si SIGUE a los tirones incluso con el combo congelado, la causa queda acotada a algo puramente electrico/mecanico del driver, el H-bridge o la bateria -- ya no puede ser el DIP bajo ninguna hipotesis.

---
**[2026-09-30 | 01:05 | Madrugada]**

* **FALSA ALARMA — el usuario reportó "NO FUNCIONA" pero el DIP estaba armado en `0100` (combo 4), no en `0001`.** El robot solo se mueve en combo `4` cuando detecta algo en los sensores `TOP_LEFT`/`TOP_RIGHT`/`SIDE_LEFT`/`SIDE_RIGHT` -- en el banco, sin nada al frente, se queda quieto por diseño, lo cual parecía "no funciona" pero era el combo equivocado, no un bug. El usuario se disculpó por el error de armado. Sin cambios de código en esta entrada. **Pendiente (sigue igual que la entrada anterior):** confirmar con el DIP correctamente en `0001` si el avance recto corre liso ahora que el combo está congelado, o si el tirón persiste.

---
**[2026-09-30 | 01:15 | Madrugada]**

* **`platformio.ini` — vuelta a `CALIBRACION` con BLE reactivado, `PRUEBA IMU` desactivada:** el usuario había cambiado a mano el build a `PRUEBA IMU` (probablemente probando el IMU aparte) y preguntó si el bluetooth seguía andando. Se le confirmó que `SKIP_BLE` solo afecta al bloque `CALIBRACION` (no se tocó `bluetoothComm.cpp`) y que `PRUEBA IMU` no usa BLE para nada. El usuario pidió volver a probar con bluetooth, así que se reactivó el bloque `CALIBRACION` y se comentó `-DSKIP_BLE` (con nota actualizada: ya se confirmó que no era la causa del tirón, así que queda comentado en vez de borrado por si hace falta de nuevo) y se comentó el bloque `PRUEBA IMU`. `pio run` — compila OK (Flash 28.1%, RAM 13.9%, igual que el `CALIBRACION` con BLE de siempre). **Pendiente:** sigue sin confirmarse si el combo `0001` corre liso con el modelo de 3 etapas (congelado) ya en su lugar, ahora con BLE de vuelta.

---
**[2026-09-30 | 01:25 | Madrugada]**

* **CAMBIO DE CÓDIGO — el reporte por BLE/Serial ahora incluye `MODO=XXXX`:** a pedido del usuario ("enviemos por bluetooth tambien el modo al inicio"), se agregó un campo nuevo al mensaje de `calibracion.cpp` con el combo que realmente está corriendo (`comboCongelado`, el mismo que ya se congela con el modelo de 3 etapas), como texto binario de 4 bits (mismo orden `DIPE-DIPA-DIPB-DIPC` que el resto del archivo) vía una función nueva `comboTexto(int)`, para que sea directamente comparable con la notación `0001`/`0010`/etc. que se usa en la tabla de combos y en la conversación, en vez de un número decimal. Va primero en el mensaje: `"MODO=0001 SIDE_LEFT=... "`. Sigue siendo el único dato derivado del DIP que se manda por BLE — el DIP crudo en sí sigue sin mandarse, solo se usa localmente y se imprime por Serial. Como el reporte solo se arma y envía mientras `startSignal` está activo (etapa 2), `MODO` siempre refleja el combo ya congelado, nunca `-1`. Se actualizó el comentario de formato del mensaje al principio del archivo. `pio run` — compila OK (Flash 28.2%, RAM 13.9%, sin cambio relevante de tamaño). Documentado en `Mbaretech2025/CLAUDE.md` (sección "Sensor calibration mode").

---
**[2026-09-30 | 01:35 | Madrugada]**

* **`platformio.ini` — cambio a `PRUEBA IMU`, pendiente de calibración `0001` queda pausada:** el usuario pidió pasar a trabajar sobre `src/tests/pruebaIMU.cpp`. Antes de tocar código se releyó el archivo completo y se buscó contexto previo de IMU en la DeathNote (`99_Razonamientos_y_mejoras.md` ya tenía anotada la idea de reemplazar los giros a tiempo fijo por giros de lazo cerrado usando el giroscopio, sección "2. Precisión de los Giros"). Se preguntó al usuario qué quería hacer concretamente; eligió: cargar el test tal cual está y verificar físicamente cuántos grados gira el robot a mano, comparando contra el yaw que imprime el Serial, para ver si el giroscopio integra bien. Se reactivó el bloque `PRUEBA IMU` en `platformio.ini` (comentando `CALIBRACION`). `pio run` — compila OK (Flash 9.4%, RAM 5.9%). **Nota:** el test ya soporta esto sin cambios de código — comando `z` pone yaw en 0, comando `c` recalibra el bias del giro (robot quieto), y el Serial imprime `yaw=...` cada 100ms. **Pendiente sin resolver, queda en pausa:** confirmar si el combo `0001` de `calibracion.cpp` corre liso con el modelo de 3 etapas ya implementado — no se abandonó, solo se pausó por el cambio de foco a IMU.

---
**[2026-09-30 | 01:45 | Madrugada]**

* **RESULTADO — yaw del giroscopio confirmado preciso:** el usuario giró el robot físicamente contra una referencia, comparó contra el `yaw=` que imprime `pruebaIMU.cpp` cada 100ms, y confirmó que coincide bien ("esta perfecto"). Sin cambios de código en esta entrada. Documentado en `02_Estado_Actual.md` (sección "Hardware inactivo en combate"). Esto habilita como viable el punto 2 de `99_Razonamientos_y_mejoras.md` (giros de lazo cerrado con el giroscopio en vez de `TURN_LEFT_90_DELAY`/etc. a tiempo fijo) — todavía no implementado, es el candidato natural de siguiente paso si el usuario quiere avanzar por ahí.

---
**[2026-09-30 | 02:00 | Madrugada]**

* **Nueva idea de estrategia — "Modo Martillo":** el usuario propuso que si el robot queda trabado empujando al sumo (forcejeo parejo, sin ganar terreno), en vez de seguir empujando en el lugar convendría retroceder un poco y volver a embestir. Preguntó qué datos del IMU sirven para esto antes de definir parámetros.
* **Aclaración de física dada al usuario:** la aceleración lineal es ≈0 tanto si el robot está trabado (fuerza neta cancelada) como si se mueve a velocidad constante — un umbral simple sobre `|a|` no distingue los dos casos. Lo que sí sirve es integrar la aceleración hacia adelante durante una ventana corta y acotada justo al arrancar el empuje, para estimar cuánta velocidad ganó en ese lapso.
* **Eje e signo confirmados con datos reales:** el usuario empujó el robot a mano hacia adelante/atrás con `pruebaIMU.cpp` corriendo y pegó el log — confirmó que `ay` es el eje que responde (`ax`/`az` casi no se mueven, `az≈-1.08` en reposo consistente con IMU montado plano). Se le pidió confirmar el signo con un empujón limpio en una sola dirección; respondió que **adelante es `ay` positivo**.
* **CAMBIO DE CÓDIGO — nuevo archivo `src/tests/pruebaMartillo.cpp` (flag `RUN_PRUEBA_MARTILLO`):** mide, no implementa el martillo todavía. Al activar el killswitch, empuja ambos motores a `parametros[2]`% durante una ventana fija (`VENTANA_MS=500`), integrando `ay*dt` en un acumulador de "velocidad estimada" cada vuelta, y al cerrar la ventana frena e imprime el resultado final por Serial — una sola medición por activación (mismo patrón de "congelar en la activación, ignorar hasta soltar" que el modelo de 3 etapas de `calibracion.cpp`, para no seguir chocando en loop). Duplica las funciones de bajo nivel del MPU6050 (`escribirReg`/`leerRegs`/`detectarIMU`/`configurarIMU`) de `pruebaIMU.cpp` en vez de compartir código — mismo criterio de autocontención que el resto de `src/tests/`. Se agregó `RUN_PRUEBA_MARTILLO` a la guarda del `setup()` de combate en `main.cpp`. En `platformio.ini` se comentó el bloque `PRUEBA IMU` y se activó el nuevo bloque `PRUEBA MARTILLO`. `pio run` — compila OK (Flash 9.6%, RAM 6.1%). Documentado en `Mbaretech2025/CLAUDE.md` y en `99_Razonamientos_y_mejoras.md` (nuevo punto 4 "Modo Martillo", con anotación en el punto 3 sobre que los encoders ya no están disponibles por el repurpose de pines a línea trasera). **Pendiente:** el usuario debe correr esta prueba dos veces — una empujando contra algo pesado/trabado, otra libre — y pasar los dos resultados de "velocidad estimada" para definir el umbral real y, recién ahí, escribir la lógica del martillo (retroceso + reintento) en `tasks.cpp`.

---
**[2026-09-30 | 02:10 | Madrugada]**

* **CAMBIO DE CÓDIGO — resultado de `pruebaMartillo.cpp` ahora se repite por Serial en vez de imprimirse una sola vez:** el usuario avisó que no puede hacer la prueba con el cable/Serial monitor conectado porque el empuje es demasiado rápido/violento para sostener el robot y leer la pantalla al mismo tiempo. Se sacó el print en vivo durante el empuje (`ay=... vel_estimada=...` en cada vuelta, que tampoco se podía leer a tiempo) y en su lugar, una vez cerrada la ventana de medición, el resultado final se repite cada 1 segundo por Serial mientras se espera que se suelte el killswitch, en vez de imprimirse una sola vez y quedar mudo. Como el ESP32-S3 sigue corriendo a batería aunque se desconecte el USB (no se resetea solo por desenchufar/reenchufar el cable, mientras la batería lo mantenga alimentado), el flujo pensado es: desconectar el USB, hacer el empuje de prueba suelto, reconectar y abrir el monitor recién después — el resultado va a aparecer solo, dentro del primer segundo, sin apurar el timing. Se agregó un aviso de esto al mensaje de `setup()`. `pio run` — compila OK (Flash 9.6%, RAM 6.1%, sin cambio relevante de tamaño).

---
**[2026-09-30 | 02:20 | Madrugada]**

* **CAMBIO DE CÓDIGO — `pruebaMartillo.cpp` ahora también manda el resultado por BLE:** el usuario aclaró el motivo real de la restricción anterior: durante el empuje de prueba probablemente tenga que sostener el killswitch con la mano para poder cortar rápido si el robot sale disparado del dohyo — no es solo que sea difícil de leer, es que no puede tener las manos ocupadas con un cable. Propuso ver el valor por bluetooth desde el celular, ya que una app de terminal BLE conserva el historial de mensajes recibidos aunque corte con el killswitch. Se agregó `#include "bluetoothComm.h"`, `BLE_UART_Init("MBARETECH")` en el `setup()` (mismo servicio UART que ya usa `calibracion.cpp`) y `sendData(...)` tanto en el aviso de "Empuje iniciado..." como en el resultado que se repite cada 1s — ahora va por Serial y BLE a la vez, así que con el celular emparejado de antemano el usuario puede operar el killswitch con las manos libres y el historial de resultados queda en el log de la app del celular. `pio run` — compila OK (Flash 28.4%, RAM 13.9% — sube porque ahora sí enlaza la librería BLE, como los demás modos que la usan). Documentado en `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-30 | 02:30 | Madrugada]**

* **CORRECCIÓN — `pruebaMartillo.cpp` empujaba a `parametros[2]` en vez de la velocidad real de ataque:** el usuario pidió usar el mismo PWM que usa el código de competencia. Al revisar `tasks.cpp`, el estado `FORWARD` empuja a `local_speed = FORWARD_80` (80%, con salto a `MAX_SPEED`=100% solo si `SHORT_LEFT` y `SHORT_RIGHT` disparan juntos, caso de contacto confirmado que esta prueba no cubre) — **`parametros[2]` no lo lee `tasks.cpp` para nada**, solo lo usan `movements.cpp`/`TEST_FORWARD`, así que medir con `parametros[2]` (94% por defecto) habría dado un umbral que no corresponde a las condiciones reales de combate. Se cambiaron las dos llamadas a `forward()` de `parametros[2]` a `FORWARD_80`, y se actualizó el mensaje de `setup()` y los comentarios de cabecera para dejar explícita la razón. `pio run` — compila OK (Flash 28.4%, RAM 13.9%, sin cambio de tamaño relevante). Documentado en `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-30 | 02:40 | Madrugada]**

* **`platformio.ini` — vuelta a `PRUEBA MOTORES` para verificar algo puntual:** el usuario pidió cargar de nuevo este build sin especificar el motivo todavía. Se comentó `PRUEBA MARTILLO` y se reactivó `PRUEBA MOTORES`. `pio run` — compila OK (Flash 9.0%, RAM 6.0%, igual que la última vez confirmada lisa en hardware).

---
**[2026-09-30 | 02:50 | Madrugada]**

* **`platformio.ini` — vuelta a `PRUEBA MARTILLO`:** el usuario pidió retomar la prueba del martillo. Se comentó `PRUEBA MOTORES` y se reactivó `PRUEBA MARTILLO`. `pio run` — compila OK (Flash 28.4%, RAM 13.9%). **Pendiente sin cambios:** correr la prueba empujando trabado y libre, pasar los dos resultados de "velocidad estimada" para definir el umbral real del martillo.

---
**[2026-09-30 | 02:55 | Madrugada]**

* **CAMBIO DE CÓDIGO — `VENTANA_MS` de 500 a 300:** a pedido del usuario, en `src/tests/pruebaMartillo.cpp`. `pio run` — compila OK (Flash 28.4%, RAM 13.9%, sin cambio relevante de tamaño).

---
**[2026-09-30 | 03:00 | Madrugada]**

* **`platformio.ini` — vuelta a `PRUEBA MOTORES`:** el usuario pidió cargar este build de nuevo, sin motivo especificado. Se comentó `PRUEBA MARTILLO` y se reactivó `PRUEBA MOTORES`. `pio run` — compila OK (Flash 9.0%, RAM 6.0%).

---
**[2026-09-30 | 03:10 | Madrugada]**

* **BUG ENCONTRADO Y CORREGIDO — `pruebaMartillo.cpp` no activaba los motores:** el usuario reportó que al probar el martillo, los motores no respondían al killswitch. Revisando `setup()`, el problema era de orden: `pinMode(START_PIN)`, `attachInterrupt(...)` y `rightMotor.begin()`/`leftMotor.begin()` estaban DESPUÉS de `while (!detectarIMU()) { ...; delay(2000); }` — un loop bloqueante **sin límite de reintentos**. Si el IMU no respondía a la primera (ruido I2C, timing al arrancar, lo que sea), `setup()` se quedaba trabado ahí para siempre y nunca llegaba a inicializar el killswitch ni los motores — el robot no podía responder a nada, sin importar qué hiciera el usuario con el switch. A diferencia de `pruebaIMU.cpp` (donde el IMU es TODO el propósito del archivo y bloquear tiene sentido), acá el IMU es secundario; lo crítico es que el killswitch y los motores funcionen. Se corrigió: `pinMode`/`attachInterrupt`/`rightMotor.begin()`/`leftMotor.begin()` ahora van primero, sin condición. La detección del IMU se volvió no bloqueante: una función `intentarIMU()` que se llama una vez en `setup()` y, si falla, se reintenta sola una vez por segundo desde `loop()` (flag `imuOk`) — un IMU lento o directamente no conectado ya no puede impedir que el test funcione. `pio run` — compila OK (Flash 28.4%, RAM 13.9%, sin cambio relevante de tamaño) con `RUN_PRUEBA_MARTILLO` reactivado en `platformio.ini` para probar el fix de verdad (el build anterior estaba en `PRUEBA MOTORES`, que no compila este archivo). Documentado en `Mbaretech2025/CLAUDE.md`.

---
**[2026-09-30 | 03:20 | Madrugada]**

* **RESULTADO — primeros datos reales del martillo, condición LIBRE:** el fix anterior funcionó — el usuario confirmó por capturas del log BLE del celular que el empuje de prueba corre bien. Hizo 4 corridas en condición **libre** (sin nada trabando, con baterías de repuesto de capacidad similar): **-0.226, -0.238, -0.245, -0.273 (g\*s)** tras 300ms — consistentes entre sí (mismo orden de magnitud, no al azar). El usuario notó que el signo salió negativo en vez de positivo como se había confirmado a mano con `pruebaIMU.cpp`, y planteó que puede estar "al revés" (el lado que el usuario considera "adelante", la pala, podría no coincidir con la dirección física que produce `motor.forward()` — ya pasó algo similar con sensores IR cruzados y motores con pines invertidos en este proyecto). **No hace falta resolver esa ambigüedad de signo para el martillo**: lo que importa es comparar la métrica entre las dos condiciones (libre vs trabado) usando el mismo mecanismo real (`motor.forward()`, el mismo que usa `tasks.cpp`), no la convención de signo en sí. Pendiente: repetir el mismo empuje (3-4 veces) en condición **trabada** (contra algo pesado/fijo), con batería de capacidad similar, para comparar contra este baseline libre y definir el umbral real.

---
**[2026-09-30 | 03:35 | Madrugada]**

* **Discusión de seguridad — cómo armar la condición "trabado" sin dañar el robot:** el usuario señaló que no puede usar ni la pala/cuchilla (filo) ni la parte trasera (impresión 3D, frágil) como punto de contacto para la prueba trabada. Se propuso trabar las RUEDAS en vez del chasis: un tope rígido bajo (tablón de canto, ladrillo acostado, borde de escalón) delante de las ruedas, de una altura que no puedan superar, con el cuerpo del robot pasando por encima sin tocar nada — mismo fenómeno físico (tracción plena contra resistencia, chasis sin traslación neta) sin arriesgar ni la pala ni la parte impresa.
* **Intento fallido — Mbaretech1 como obstáculo:** el usuario probó empujar contra el otro robot del proyecto (Mbaretech1), pero no genera resistencia real ("no le ataja a mbaretech2, basicamente le lleva de una"). Resultado (-0.273, -0.238 g\*s) cayó dentro del mismo rango que la condición libre (-0.226 a -0.273), confirmando que no fue una condición trabada de verdad, solo otro empuje libre contra algo liviano. **Pendiente sin cambios:** conseguir una condición realmente trabada (tope de ruedas fijo, no otro robot) para tener el contraste real contra el baseline libre.

---
**[2026-09-30 | 03:45 | Madrugada]**

* **Problema — empuje contra la pared no es concluyente, y el dohyo no está anclado:** el usuario señaló que empujar contra una pared en realidad movería el dohyo hacia atrás en vez de trabar al robot (el dohyo no está fijo al piso), contaminando la medición igual que pasó con Mbaretech1. Se propusieron dos alternativas sin depender de anclar nada externo: (A) trabar el tope de ruedas contra algo pesado/fijo que no sea el dohyo (pared de verdad, mueble cargado), con peso extra encima si hace falta; (B) sostener el chasis a mano con firmeza por una parte estructural sólida (no pala, no parte impresa, no ruedas) para que las ruedas patinen en el lugar sin que el robot se traslade. El usuario probó igual contra la pared y confirmó que el resultado dio -0.226 (g\*s), **idéntico al rango libre (-0.226 a -0.273)** — no concluyente, como ya sospechaba.
* **CAMBIO DE CÓDIGO — de ventana fija a curva de checkpoints:** en vez de seguir reflasheando con distintos valores de `VENTANA_MS` a ciegas, se rediseñó `pruebaMartillo.cpp` para medir una curva completa en un solo empuje: `CHECKPOINT_MS=100`, `NUM_CHECKPOINTS=6` (duración total 600ms), guardando la velocidad acumulada en cada checkpoint de 100ms en un array, y el resultado final ahora lista TODOS los puntos (`100ms=... 200ms=... ... 600ms=...`) en vez de un solo número — así una sola corrida muestra si/cuándo diverge la condición libre de la trabada, sin necesidad de probar ventana por ventana. `pio run` — compila OK (Flash 28.4%, RAM 13.9%, sin cambio relevante de tamaño). Documentado en `Mbaretech2025/CLAUDE.md`. **Pendiente:** el usuario debe repetir las pruebas (libre y, con el método A o B, trabado) y pasar las curvas completas en vez de un solo número.

---
**[2026-10-01 | 00:15 | Madrugada]**

* **Seguridad — 600ms saca al robot del dohyo:** el usuario avisó que con 300ms el robot ya va del borde a un poco más de la mitad del dohyo (154cm), así que 600ms lo saca afuera. Ese firmware de 600ms **nunca se subió**.
* **Análisis — la integración libre es coherente, la de la pared no:** -0.23 g·s ≈ 2.3 m/s a los 300ms. Acelerando parejo son ~35cm de empuje + ~50cm de envión al frenar (≈0.5g) ≈ 85cm, coincide con lo observado. Eso confirma que la integración funciona y que **`motor.forward()` da `ay` negativo** (la prueba a mano había dado al revés; el "adelante" del usuario no coincide con el sentido del motor). La prueba contra la pared arrancó **con espacio**: el robot aceleró libre y chocó. El frenazo del choque (~30-40g en pocos ms) se perdió porque el IMU estaba a ±8g (saturado), con filtro de 44Hz y una lectura cada ~11ms — por eso dio igual que libre.
* **CAMBIO DE CÓDIGO — `pruebaMartillo.cpp`:** duración total vuelta a **300ms** (`CHECKPOINT_MS` 100→50, 6 puntos); acelerómetro a **±16g** (`REG_ACCEL_CFG` 0x10→0x18, `ACCEL_LSB_POR_G` 4096→2048); filtro DLPF de ~44Hz a **~184Hz** (`REG_CONFIG` 0x03→0x01) para no aplastar el pico del golpe; **sin `delay(10)` mientras empuja** (solo cuando ya midió); se registran `picoFrenada` (ay más positivo = golpe), `picoEmpuje` (ay más negativo = arranque) y `muestras` (lecturas en la ventana), agregados al resultado por Serial/BLE. Se corrigió el comentario del signo del eje. `pio run` — compila OK. **Pendiente:** repetir libre y contra la pared (con espacio, como un choque real) y comparar curva + `picoFrenada`.

---
**[2026-10-01 | 00:25 | Madrugada]**

* **RESULTADO — dos corridas libres con signo opuesto, bug de medición:** corrida 1: `50ms=-0.163 100ms=-0.163 150ms=-0.243 200ms=-0.243 250ms=-0.182 300ms=-0.180 | picoFrenada=1.89g picoEmpuje=-13.83g muestras=86`. Corrida 2: `50ms=0.179 100ms=0.179 150ms=0.187 200ms=0.185 250ms=0.183 300ms=0.182 | picoFrenada=2.00g picoEmpuje=-2.61g muestras=105`. Misma condición, signo opuesto, casi todo acumulado en los primeros 50ms y checkpoints repetidos exactos (vueltas de loop de más de 50ms). Causa: `ultimoMicros` se tomaba ANTES del `Serial.println` + `sendData()` por BLE de "Empuje iniciado", así que el primer `dt` incluía todo el tiempo de envío BLE y se multiplicaba por la primera lectura (el sacudón del arranque, de signo al azar: -13.83g en la corrida 1), dominando la integral. **Las mediciones anteriores (-0.226 a -0.273, y la "coherencia física" de ~2.3 m/s analizada en la entrada anterior) probablemente estaban contaminadas por el mismo bug — no son confiables.**
* **CAMBIO DE CÓDIGO — `pruebaMartillo.cpp`:** los avisos (Serial + BLE) ahora se mandan ANTES de arrancar motores y reloj (`tInicio`/`ultimoMicros` se toman después). Además, una vuelta con `dt > 10ms` ya no se integra (se cuenta en `saltos`), y se reporta `dtMax` (la vuelta más lenta) para detectar si quedan otros cuelgues. `pio run` — compila OK. **Pendiente:** repetir 2-3 corridas libres y ver si ahora dan el mismo signo y una curva que crece de forma gradual.

---
**[2026-10-01 | 00:35 | Madrugada]**

* **RESULTADO — 3 corridas libres con el fix del `dt`:** (1) `0.000 0.000 0.007 0.007 0.012 0.015 | frenada=2.10g empuje=-2.39g muestras=108 saltos=1 dtMax=124.4ms`; (2) `-0.164 -0.169 -0.171 -0.176 -0.168 -0.174 | 2.32g -11.15g 169 saltos=1 dtMax=26.1ms`; (3) `0.000 0.000 0.005 0.008 0.007 0.017 | 2.22g -2.16g 105 saltos=1 dtMax=128.4ms`.
* **Análisis:** (a) después de ~100ms la curva queda plana, así que el robot alcanza su velocidad en los primeros ~100ms y después va a velocidad constante (aceleración ≈ 0), justo la zona donde "trabado" y "andando" son indistinguibles. (b) En las 3 corridas hay exactamente 1 salto: la lectura I2C del IMU se cuelga 26-128ms justo al arrancar los motores (probable ruido eléctrico o caída de tensión sobre el bus), así que la ventana donde está la información es ilegible. En la corrida 2 entró además el sacudón del arranque (-11g) y eso explica el -0.164. **Conclusión: integrar la aceleración para estimar velocidad NO sirve como detector del martillo con este hardware.**
* **Lo que sí parece útil:** `picoFrenada` en vacío es muy estable (2.10 / 2.32 / 2.22g). Propuesta: usar el pico del golpe (contacto) como detector, y decidir "trabado" por tiempo de contacto sostenido (rival todavía enfrente después de X ms), sin medir velocidad.
* **Riesgo anotado:** el cuelgue del I2C al arrancar motores también afectaría a los giros de lazo cerrado con giroscopio (punto 2 de `99`). Recomendado revisar el cableado del IMU (cables cortos, pull-ups 2.2k-4.7k, capacitor en VCC del módulo).
* **Pendiente:** 2-3 empujes contra la pared con espacio, con el mismo firmware, para comparar `picoFrenada` del golpe contra el baseline de ~2.3g en vacío. Sin cambios de código en esta entrada.

---
**[2026-10-01 | 00:45 | Madrugada]**

* **RESULTADO — 3 empujes contra la pared SIN DATOS:** las tres corridas dieron `muestras=0 saltos=0 dtMax=0.0ms`, todo en 0. El IMU no respondió ninguna lectura durante el empuje: `leerAy()` falló al instante en cada vuelta (`dtMax≈0`, típico de NACK en el bus I2C). Antes andaba (105-169 muestras en las corridas en vacío), así que el IMU se desconectó o el bus quedó trabado entre una prueba y otra. Como `imuOk` queda en `true` una vez detectado, el test no reintenta la detección ni avisa. Antecedente: el 2026-09-28 el VCC del IMU medía 1.5V por mala conexión. **Recomendado revisar el cableado del IMU antes de depender de él en combate.**
* **Decisión del usuario:** no seguir probando; usar **8g** como umbral de golpe y definir el martillo con tiempos. Se propuso, para validar antes de tocar `tasks.cpp`: en `FORWARD`, golpe ≥8g (ay positivo) arranca un reloj; si a los 500ms el rival sigue enfrente (`TOP_MID` o ambos `SHORT`), retrocede al 80% durante 120ms y vuelve a embestir al 100%; con umbral y tiempos ajustables por BLE. Se marcaron dos riesgos: el retroceso cerca del borde trasero (los sensores de línea traseros no se usan en `tasks.cpp`) y que el IMU se caiga (el martillo no dispararía nunca); como respaldo, se ofreció disparar también por los dos `SHORT` sostenidos durante X ms. **Pendiente:** confirmación del usuario de los valores y del respaldo. Sin cambios de código en esta entrada.

---
**[2026-10-01 | 00:55 | Madrugada]**

* **Martillo en pausa (standby)** a pedido del usuario, sin confirmar los valores propuestos. Queda pendiente retomarlo: valores (8g / 500ms / 120ms al 80% / 100% al volver), respaldo por sensores `SHORT`, y revisar el cableado del IMU.
* **CAMBIO DE CÓDIGO — nueva prueba de frenado con sensor de línea (`src/tests/pruebaFrenadoLinea.cpp`, flag `RUN_PRUEBA_FRENADO`, necesita también `RUN_LINE_SENSOR`):** el usuario eligió reproducir la reacción de combate y avanzar al 80%. Al activar el killswitch avanza a `FORWARD_80`; apenas un sensor de línea delantero ve blanco (`checkLineSensora/b`, mismo criterio que `tasks.cpp`), hace exactamente `LINE_RETREAT` (reversa `FORWARD_90` durante 80ms) y frena. Corte de seguridad: si en 1500ms no ve la línea, frena igual. Reporta por Serial y BLE (repetido cada 1s hasta soltar el killswitch): ms hasta ver la línea, qué sensor (IZQ/DER/AMBOS), valores crudos de ADC, `THRESHOLD`, y la vuelta de loop más lenta (`lazoMax`, la latencia de reacción). Una prueba por activación. Se agregó `RUN_PRUEBA_FRENADO` a la guarda del `setup()` de combate en `main.cpp`. En `platformio.ini` se comentó `PRUEBA MARTILLO` (marcado EN PAUSA) y se activó `PRUEBA FRENADO`. `pio run` — compila OK (Flash 28.1%, RAM 13.9%). Documentado en `Mbaretech2025/CLAUDE.md`. **Pendiente:** el usuario debe probar en el dohyo y contar si frena a tiempo y cuánto se pasa de la línea.

---
**[2026-10-01 | 01:00 | Madrugada]**

* **`platformio.ini` — vuelta a `PRUEBA MOTORES`** a pedido del usuario (se comentó `PRUEBA FRENADO`). `pio run` — compila OK (Flash 9.0%, RAM 6.0%). Registrado después de la entrada de abajo.
* **CAMBIO DE CÓDIGO — `TIMEOUT_MS` de 1500 a 3000, y después a 5000** (el usuario pidió más margen para el banco), a pedido del usuario: va a probar primero en banco (ruedas sin tracción) acercando algo blanco al sensor a mano, para ver que la reversa se dispare. `pio run` — compila OK. Ojo: antes de probar en el dohyo convendría volver a 1500ms (con 3s, si el sensor falla, el robot sale del dohyo antes de que corte).

---
**[2026-10-01 | 14:20 | Tarde]**

* **CAMBIO DE CÓDIGO — `pruebaFrenadoLinea.cpp`: no arranca si ya ve blanco.** El usuario reportó que en banco el robot no avanzaba. Causa probable: los sensores delanteros leían la base del banco como blanco (≤ `THRESHOLD`) en el mismo instante de activar → "línea a los 0ms", reversa 80ms y freno. Ahora, al activar el killswitch, se leen ambos sensores delanteros antes de mover los motores; si alguno ya ve blanco no avanza y reporta (Serial + BLE, cada 1s) `ARRANCA SOBRE BLANCO` con los crudos y el `THRESHOLD`.
* **`platformio.ini` — vuelta a `PRUEBA FRENADO`** (se comentó `PRUEBA MOTORES`). `pio run` — compila OK (Flash 28.1%, RAM 13.9%). El primer intento falló por disco C: lleno; el usuario liberó espacio.

---
**[2026-10-01 | 14:30 | Tarde]**

* **`platformio.ini` — vuelta a `CALIBRACION`** (se comentó `PRUEBA FRENADO`), a pedido del usuario, para verificar los sensores de línea sobre el dohyo. `SKIP_BLE` sigue comentado (BLE activo). `pio run` — compila OK.

---
**[2026-10-01 | 14:45 | Tarde]**

* **`platformio.ini` — vuelta a `PRUEBA FRENADO`** (se comentó `CALIBRACION`). El usuario verificó en el dohyo que los rangos de los sensores de línea están bien.
* **CAMBIO DE CÓDIGO — `TIMEOUT_MS` de 5000 a 1500** en `pruebaFrenadoLinea.cpp`, porque ahora se prueba en el dohyo (5000 era solo para el banco). `pio run` — compila OK.

---
**[2026-10-01 | 15:00 | Tarde]**

* **CAMBIO DE CÓDIGO — `THRESHOLD` de `250` a `800`** (`include/globals.h`, bloque `MBARETECH_2`), a pedido del usuario: cree que el umbral bajo hace que la prueba de frenado detecte el borde tarde. Con 250 el sensor tiene que estar casi entero sobre el blanco (blanco medido ≤ ~199) para disparar; con 800 dispara antes, en la transición negro→blanco. El negro nunca bajó de ~2793, así que sigue con margen amplio contra falsos blancos. **Afecta también a `tasks.cpp` (`LINE_RETREAT` en combate) y a `calibracion.cpp`.** `pio run` — compila OK.

---
**[2026-10-01 | 15:10 | Tarde]**

* **CAMBIO DE CÓDIGO — `THRESHOLD` revertido de `800` a `250`.** El usuario reporta que en la prueba de frenado el robot se pone en reversa apenas se activa, y pidió ir al revés (bajar, no subir). No se bajó de 250 porque el blanco medido llega a ~199 y quedaría sin margen. Hipótesis abierta: como el chequeo previo `ARRANCA SOBRE BLANCO` se hace con los motores apagados, si dispara justo después de arrancar podría ser ruido eléctrico de los motores al arrancar bajando una lectura suelta del ADC (desde 2026-09-29 una sola lectura alcanza, sin filtro). Pendiente: ver los ms y crudos que reporta la prueba. `pio run` — compila OK.

---
**[2026-10-01 | 15:20 | Tarde]**

* **CAMBIO DE CÓDIGO — `pruebaFrenadoLinea.cpp`: 3 lecturas seguidas para confirmar línea** (`LECTURAS_BLANCO=3`), a pedido del usuario, para filtrar picos de ruido de los motores al arrancar. Contador por sensor: suma con cada lectura `<= THRESHOLD`, vuelve a 0 con una sola en negro (simétrico en la práctica: negro sigue siendo instantáneo). Solo en esta prueba — `lineSensor.cpp` y `tasks.cpp` (combate) siguen con una sola lectura. Costo de tiempo: durante el avance el lazo no tiene `delay()`, solo 2 lecturas de ADC1, así que 3 lecturas son décimas de ms (se puede confirmar con `lazoMax` del reporte). `pio run` — compila OK.

---
**[2026-10-01 | 15:35 | Tarde]**

* **CAMBIO DE CÓDIGO — `pruebaFrenadoLinea.cpp`: variante hacia atrás (`FRENADO_ATRAS`)**, a pedido del usuario. Con el flag, el robot retrocede al 80% (`backward(FORWARD_80)`) leyendo los sensores traseros LS3/LS4 (`readLineSensorBack`, ADC2); al confirmar blanco (3 lecturas seguidas) empuja hacia adelante 90% x 80ms (espejo de `LINE_RETREAT`) y frena. Sin el flag sigue siendo la prueba hacia adelante con LS1/LS2, sin cambios de comportamiento. Las lecturas fallidas de ADC2 (`-1`, el BLE puede bloquearlo) no cuentan como blanco ni resetean el contador; se reportan como `errores=izq/der` en el resultado. **El combate (`tasks.cpp`) no tiene esta reacción trasera:** esta prueba solo la mide.
* **`platformio.ini` — nuevo bloque activo `PRUEBA FRENADO ATRAS`** (`RUN_PRUEBA_FRENADO` + `FRENADO_ATRAS`); el bloque `PRUEBA FRENADO` (adelante) quedó comentado. `pio run` — compila OK.

---
**[2026-10-01 | 15:50 | Tarde]**

* **VALIDACIÓN EN HARDWARE — prueba de frenado con línea, adelante y atrás:** el usuario confirma que funciona correctamente. Sin cambios de código. Detalle en `03_Bitacora_Diaria.md`.

---
**[2026-10-01 | 16:15 | Tarde]**

* **CÓDIGO NUEVO — `src/tests/autoCalGiro.cpp` (`RUN_AUTOCAL_GIRO`): autocalibración del tiempo de giro con el IMU**, a pedido del usuario. Por cada activación del killswitch: espera 1s, calibra el bias del giroscopio (~0.6s quieto) y alterna giros IZQ/DER (el mismo giro que `0010`/`tasks.cpp`: `parametros[3]`/`[6]` + `[18]`), arrancando desde `parametros[4]`/`[7]` (hoy 55ms/45ms — no 0.8s como suponía el usuario). Mide el ángulo integrando `gz` desde que arrancan los motores hasta que el robot queda quieto después de frenar (incluye la inercia). Si queda fuera de 45 ± 5°, corrige el tiempo de forma proporcional (`tiempo*45/medido`, limitado a ±30% y mínimo 1ms) y espera 1s antes del siguiente intento. Máximo 15 intentos por lado. El freno lo da un `esp_timer` (no el lazo), porque en el martillo se vieron trabas de I2C de 26–128ms al arrancar motores; si entre lecturas hay un hueco > 10ms, el intento se descarta y se repite con el mismo tiempo. El IMU se detecta sin bloquear (lección del martillo). Reporta cada intento y un resumen final por Serial y BLE. No guarda los tiempos: hay que pasarlos a mano a `parametros[]`/`globals.h`. `main.cpp`: `RUN_AUTOCAL_GIRO` sumado a la exclusión del `setup()` de combate. `platformio.ini`: nuevo bloque activo `AUTOCAL GIRO`; `PRUEBA FRENADO ATRAS` comentado (marcado VALIDADA). `pio run` — compila OK. **Sin probar en hardware.**

---
**[2026-10-01 | 15:40 | Tarde]**

* **PRIMERA CORRIDA EN HARDWARE — `autoCalGiro.cpp` (captura del celular del usuario, 15:29):**
  * **IZQ:** 55ms→25.3° · 72ms→32.4° · 94ms→45.9° **OK en 3 intentos**. Consistente: ~0.48°/ms, casi lineal.
  * **DER:** no convergió en 15 intentos. 45→15.5 · 58→31.5 · 75→51.6 · 65→51.4 · 57→28.2 · 74→65.8 · 52→23.2 · 68→21.7 · 88→56.7 · 70→54.7, más 5 intentos DESCARTADOS (huecos de I2C de 19–45ms). Muy inconsistente: 65ms→51° pero 68ms→22°. **Todos los descartes fueron en giros DER**, ninguno en IZQ: el giro a la derecha (izq forward 98% + der backward 94%) mete mucho más ruido en el I2C.
  * **Discrepancia grande con la calibración a ojo:** los 55/45ms "de 45°" (calibrados mirando, 2026-09-25) miden ~25°/~16° con el IMU. Falta confirmar si el IMU mide bien (girar a mano un ángulo conocido con `pruebaIMU`) antes de pasar ningún tiempo a `globals.h`.
* **CAMBIO DE CÓDIGO — reporte de `autoCalGiro.cpp`:** cada mensaje BLE termina en `\n` (la app del celular los pegaba en un solo bloque) y cada intento muestra `gap` (peor hueco entre lecturas) y `fallos` (lecturas I2C fallidas), también en los válidos, para ver si los giros DER "válidos" están contaminados por huecos de menos de 10ms. `pio run` — compila OK.

---
**[2026-10-01 | 15:50 | Tarde]**

* **`platformio.ini` — vuelta a `PRUEBA FRENADO` (adelante)** a pedido del usuario, para volver a verificarla. `AUTOCAL GIRO` quedó comentado. Hipótesis del usuario pendiente para la autocalibración: la batería baja explica el desorden del giro DER (respuesta dada: explica que todo gire menos que con la calibración a ojo hecha con batería llena, pero no los saltos sin orden de DER con IZQ coherente; se propuso repetir la autocalibración con batería llena). `pio run` — compila OK.

---
**[2026-10-01 | 16:00 | Tarde]**

* **`platformio.ini` — vuelta a `AUTOCAL GIRO`** a pedido del usuario (se comentó `PRUEBA FRENADO`). Incluye el reporte nuevo (`gap`/`fallos` por intento, salto de línea en BLE). `pio run` — compila OK.

---
**[2026-10-01 | 16:23 | Tarde]**

* **SEGUNDA CORRIDA EN HARDWARE — `autoCalGiro.cpp` (captura del celular, 16:23), sin cambios de código:** convergió en **2 intentos por lado**, todas las mediciones limpias (`gap` 0.6–0.7ms, `fallos 0`, ningún DESCARTADO).
  * IZQ: 55ms→35.7° · **69ms→43.2° OK**
  * DER: 45ms→31.6° · **58ms→42.1° OK**
  * Coherente con la hipótesis del usuario: el desorden de la primera corrida (DER errático, 5 descartes por I2C) coincide con batería baja; esta corrida sale limpia. ~0.63°/ms IZQ, ~0.72°/ms DER.
  * Los dos quedaron en el borde bajo de la tolerancia (43.2/42.1 con ±5°). Proporcionalmente, 45° exactos serían ~72ms/~62ms.
  * **Sigue pendiente** confirmar que el IMU mide el ángulo real (girar a mano 90° con `pruebaIMU`): los 55/45ms calibrados a ojo con batería llena miden ~36°/~32°. No se pasó nada a `globals.h`.

---
**[2026-10-01 | 16:30 | Tarde]**

* **CAMBIO DE CÓDIGO — `TOLERANCIA_GRADOS` de 5 a 2** en `autoCalGiro.cpp`, a pedido del usuario (con ±5 los dos lados quedaban en 42–43°). A ~0.63–0.72°/ms, la ventana de 4° equivale a ~6ms, así que se puede alcanzar con pasos de 1ms. Sigue arrancando desde 55/45ms (`globals.h`). Comentario del bloque en `platformio.ini` actualizado a "45 +-2 grados". `pio run` — compila OK.

---
**[2026-10-01 | 16:35 | Tarde]**

* **`platformio.ini` — vuelta a `PRUEBA MOTORES`** a pedido del usuario, "para verificar algo" (sin detalle). `AUTOCAL GIRO` (ya con tolerancia ±2°, todavía sin correr) quedó comentado. `pio run` — compila OK.

---
**[2026-10-01 | 16:40 | Tarde]**

* **`platformio.ini` — vuelta a `PRUEBA FRENADO` (adelante)** a pedido del usuario (se comentó `PRUEBA MOTORES`). `pio run` — compila OK.

---
**[2026-10-01 | 16:45 | Tarde]**

* **`platformio.ini` — vuelta a `AUTOCAL GIRO`** a pedido del usuario (se comentó `PRUEBA FRENADO`), ahora con tolerancia ±2°. `pio run` — compila OK.

---
**[2026-10-01 | 16:50 | Tarde]**

* **`platformio.ini` — vuelta a `CALIBRACION` (manual, modos DIP)** a pedido del usuario (se comentó `AUTOCAL GIRO`). `SKIP_BLE` sigue comentado (BLE activo). `pio run` — compila OK. Ojo: `calibracion.cpp` arranca con los tiempos de `globals.h` (55/45ms para 45°); los de la autocalibración (69/58ms a ±5°) no se pasaron todavía.

---
**[2026-10-01 | 16:55 | Tarde]**

* **CÓDIGO NUEVO — `src/tests/borrarAtrasAdelante.cpp` (`RUN_BORRAR_ATRAS_ADELANTE`), temporal**, a pedido del usuario: en cada activación del killswitch va 300ms hacia adelante y 300ms hacia atrás a `parametros[2]` (94%), sin pausa en el cambio de sentido, y frena. Una vez por activación; soltar el killswitch frena al instante. Sin sensores ni BLE. Flag sumado a la exclusión del `setup()` de combate en `main.cpp`. `platformio.ini`: nuevo bloque activo `BORRAR ATRAS ADELANTE`; `CALIBRACION` comentado. `pio run` — compila OK.

---
**[2026-10-01 | 17:00 | Tarde]**

* **CAMBIO DE CÓDIGO — `borrarAtrasAdelante.cpp` ahora repite en bucle**, a pedido del usuario: mientras el killswitch esté activo, 300ms adelante → 1000ms frenado → 300ms atrás → 1000ms frenado → repite, con el mismo PWM en los dos sentidos (`parametros[2]`, 94%). Soltar el killswitch frena en cualquier punto del ciclo; al reactivarlo arranca por "adelante". La pausa también elimina el cambio de sentido directo de la versión anterior. `pio run` — compila OK.

---
**[2026-10-01 | 17:10 | Tarde]**

* **CAMBIO DE CÓDIGO — `borrarAtrasAdelante.cpp`: un motor a la vez**, a pedido del usuario. En bucle mientras el killswitch esté activo: derecho adelante 300ms → 1s frenado → derecho atrás 300ms → 1s → izquierdo adelante 300ms → 1s → izquierdo atrás 300ms → 1s → repite. El motor que no se prueba queda frenado. Mismo PWM en todos los pasos (`parametros[2]`, 94%). `pio run` — compila OK.

---
**[2026-10-01 | 17:15 | Tarde]**

* **`platformio.ini` — vuelta a `CALIBRACION` (manual, modos DIP)** a pedido del usuario (se comentó `BORRAR ATRAS ADELANTE`). BLE activo. `pio run` — compila OK.

---
**[2026-10-01 | 17:20 | Tarde]**

* **`platformio.ini` — vuelta a `AUTOCAL GIRO`** (±2°) a pedido del usuario (se comentó `CALIBRACION`). `pio run` — compila OK.

---
**[2026-10-01 | 17:35 | Tarde]**

* **CÓDIGO NUEVO — `src/tests/Girar45Linea.cpp` (`RUN_GIRAR45_LINEA`)**, a pedido del usuario. Mientras el killswitch esté activo, avanza a `FORWARD_80` y rebota en el borde: línea con el sensor IZQ → reversa `FORWARD_90` x 80ms + giro 45° DER (`[6]`/`[7]`+`[18]`, igual que `TURN_RIGHT_45`) y sigue; línea con el DER → reversa + giro 45° IZQ (`[3]`/`[4]`+`[18]`); **AMBOS → reversa + giro a la izquierda de 120ms fijos (`GIRO_180_MS`)**, provisorio para un ~180° todavía sin calibrar, a pedido del usuario. La línea se confirma con 3 lecturas seguidas (como en la prueba de frenado validada); solo sensores delanteros. BLE activo: reporta cada reacción y permite ajustar `[4]`/`[7]` en vivo. Arranca con los tiempos de 45° de `globals.h` (55/45ms), no con los de la autocalibración. Flag sumado a la exclusión del `setup()` de combate en `main.cpp`. `platformio.ini`: nuevo bloque activo `GIRAR 45 LINEA`; `AUTOCAL GIRO` comentado. `pio run` — compila OK. **Sin probar en hardware.**

---
**[2026-10-02 | Madrugada]**

* **GIT — proyecto subido a GitHub**, a pedido del usuario: commit `9138303` en la branch `firmware_FedeAlegre` de `Feraqc/Mbaretech-firmware` (clon local en `Mbaretech2025/Mbaretech-firmware/`), como carpeta nueva `Mbaretech2025/`. Se copió `src/`, `include/`, `lib/`, `test/`, `platformio.ini`, `CLAUDE.md`, `DeadhNote2026/`, `.gitignore`, `.vscode/`. **Quedaron afuera:** `MBARETECH.pdf`, `Mbaretech2_RobochallengeBR2025.rar` (14MB), `.pio/` y el propio `Mbaretech-firmware/`. Es una **copia**: los cambios futuros en esta carpeta hay que volver a copiarlos al clon antes de cada commit. El repo propio de esta carpeta (remoto `pferreiram97/Mbaretech2`) no se tocó: GitHub responde "Repository not found".

---
**[2026-10-02 | Madrugada]**

* **RENOMBRE — el proyecto pasa a llamarse `Mbaretech2026`** (antes `Mbaretech2025`, el nombre quedó del año anterior), a pedido del usuario. En GitHub (`Feraqc/Mbaretech-firmware`, branch `firmware_FedeAlegre`) la carpeta se renombró con `git mv`. Referencias actualizadas en `../CLAUDE.md` y `02_Estado_Actual.md`. Las menciones a `Mbaretech2025` en las entradas anteriores de este registro se dejan como están (historial). La carpeta local la renombra el usuario a mano (VS Code la tiene abierta).
