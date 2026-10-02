# Bitácora Diaria de Decisiones y Logros

Este documento se utiliza para registrar decisiones arquitectónicas importantes, razonamientos estratégicos, análisis profundos y refactorizaciones grandes en el software del Megasumo Mbaretech2. 
A diferencia del archivo RAW, aquí se documenta el "por qué" y el "para qué" a nivel estratégico.

## Entradas

---
### [2026-05-13 | 00:48 | Madrugada] - Primer Vistazo y Debate Estratégico

**1. Auditoría Inicial:**
Dimos un primer vistazo al código fuente del proyecto (`tasks.cpp`, `main.cpp`, `globals.h`). Analizamos en detalle cómo está estructurada la máquina de estados actual (sobre FreeRTOS) y cómo el robot prioriza la supervivencia ante la línea, y luego el ataque frontal o los giros.

**2. Descubrimientos Lógicos y de Hardware:**
* **El Bug de la Escoba:** Analizamos y debatimos por qué el robot giraba hacia los objetivos pero no atacaba en las pruebas. Descubrimos, gracias a los archivos de testeo, que existe un cruce físico de pines en la placa (`TOP_MID` invertido con `SHORT_RIGHT`).
* **Calibración de Giros:** Comprendimos que la calibración se realizaba anulando el avance (`FORWARD`) para medir visualmente si los milisegundos de giro eran exactos.

**3. Debate de Estrategias a Futuro:**
Debatimos fuertemente sobre las vulnerabilidades del código actual frente a trampas comunes en Megasumo (como las banderas enemigas).
* Se acordó que el código actual es muy estricto e ingenuo, atacando a ciegas cualquier reflejo.
* Se estableció la necesidad de implementar lógicas de "geometría" (requerir lectura de múltiples sensores) para ignorar banderas.
* Se discutió la ventaja táctica de utilizar el MPU6050 (Acelerómetro/Giroscopio) que ya está en la placa, tanto para detectar choques falsos como para garantizar giros cerrados precisos independientes de la fricción o batería.

---
### [2026-10-01 | 15:50 | Tarde] - Prueba de frenado con sensores de línea validada (adelante y atrás)

**1. Resultado:** el usuario confirmó que la prueba de frenado con sensor de línea (`src/tests/pruebaFrenadoLinea.cpp`) **funciona correctamente** en el dohyo, en sus dos variantes: hacia adelante (LS1/LS2, reacción `LINE_RETREAT` de combate: reversa 90% x 80ms) y hacia atrás (`FRENADO_ATRAS`, LS3/LS4, empuje adelante 90% x 80ms).

**2. Decisiones que lo hicieron funcionar:**
* Chequeo previo `ARRANCA SOBRE BLANCO`: no moverse si un sensor ya ve blanco al activar (en banco la base se leía como blanco → "línea a los 0ms").
* `THRESHOLD` se mantiene en 250: se probó 800 y empeoró (reversa apenas se activaba).
* Confirmación con **3 lecturas seguidas** en blanco, para filtrar picos de ruido de los motores al arrancar. Cuesta menos de 1ms porque el lazo no tiene `delay()`.

**3. Pendiente:** llevar el filtro de 3 lecturas a `lineSensor.cpp`/`tasks.cpp` (el combate sigue con una sola lectura) y evaluar sumar la reacción de los sensores traseros al combate (hoy `tasks.cpp` no los lee).

---
### [2026-10-01 | 16:25 | Tarde] - Autocalibración de giros con IMU: primera convergencia limpia

**1. Qué se armó:** `src/tests/autoCalGiro.cpp` (`RUN_AUTOCAL_GIRO`). El robot gira 45° con el mismo giro de `0010`/`tasks.cpp`, mide el ángulo real con el giroscopio (incluida la inercia después de frenar) y corrige el tiempo de forma proporcional hasta quedar en 45 ± 5°, alternando IZQ/DER con 1s de pausa entre intentos. Es el primer paso hacia los giros a lazo cerrado con IMU propuestos en `99_Razonamientos_y_mejoras.md`.

**2. Resultados:**
* Primera corrida (batería baja): IZQ convergió (94ms) pero DER fue errático y no convergió, con 5 intentos descartados por trabas de I2C, todos en giros a la derecha.
* Segunda corrida (16:23): convergencia limpia en 2 intentos por lado, sin huecos ni fallos de I2C → **IZQ 69ms (43.2°), DER 58ms (42.1°)**.
* Conclusión: el estado de la batería cambia mucho el ángulo para un mismo tiempo y, con batería baja, también el ruido en el I2C. Refuerza que en combate conviene cortar el giro por grados (IMU) en vez de por tiempo fijo.

**3. Pendiente antes de pasar los tiempos a `globals.h`:** validar que el IMU mide el ángulo real (giro a mano de 90° con `pruebaIMU`), porque los 55/45ms calibrados a ojo con batería llena miden ~36°/~32°. Después, achicar la tolerancia (±2–3°) y seguir con 90°.
