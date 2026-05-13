# Estado Actual del Proyecto (Mbaretech 2)

## Objetivos Importantes Cumplidos
* Creación y estructuración de la DeathNote para trazabilidad del proyecto `[2026-05-12]`.
* Auditoría completa del código base y detección del bug físico de sensores `[2026-05-12]`.
* Definición de estrategias para mitigación de banderas enemigas y uso de IMU `[2026-05-12]`.

---

Este documento refleja cómo está funcionando el robot actualmente a nivel de hardware y software para evitar retrabajos y confusiones.

## Actualmente

### 1. Hardware y Bugs Conocidos
* **Placa Activa:** `MBARETECH_2` (Definido en `platformio.ini`).
* **Sensores Infrarrojos (IR):** 7 sensores digitales.
  * *CRÍTICO - Cruce de Pines:* Según las pruebas (`sensorsTest.cpp`), la conexión física real no coincide con `globals.h`. 
    * El sensor que apunta al centro (`TOP_MID` físico) está conectado al pin de `SHORT_RIGHT` (`IR6`).
    * El sensor que apunta a la derecha corta (`SHORT_RIGHT` físico) está conectado al pin de `TOP_MID` (`IR4`).
    * *Nota:* Esto causó un bug donde el robot giraba hacia la escoba pero no atacaba porque leía el pin equivocado.
* **Sensores de Línea:** 2 frontales analógicos (ADC) funcionando correctamente.
* **Hardware inactivo en combate:** MPU6050 (Acelerómetro/Giroscopio) y Encoders. Están conectados pero la máquina de estados actual no los utiliza para tomar decisiones.

### 2. Máquina de Estados (`tasks.cpp`)
El cerebro del robot es una tarea de FreeRTOS que opera bajo las siguientes prioridades:
1. **Supervivencia (`LINE_RETREAT`):** Si toca la línea blanca y no tiene al enemigo pegado, retrocede, gira 180° y entra en modo defensivo ("Turkish").
2. **Cacería (`BRAKE`):** Estado de decisión. Lee sensores laterales para girar, o el sensor central para atacar.
3. **Ataque (`FORWARD`):** Empuja al 80%. Si pasan 500ms, incrementa gradualmente la potencia. Si los sensores cortos (`SHORT_LEFT` y `SHORT_RIGHT`) se activan simultáneamente, inyecta el 100% de potencia (`MAX_SPEED`) asumiendo impacto inminente.
4. **Giros Abiertos:** Los estados como `TURN_LEFT_90` giran ciegamente usando `while(!elapsedTime(delay))`. No leen sensores mientras giran (a menos que `-DCANCEL_TURNS` esté activado en compilación).

### 3. Estrategias Pre-Combate (DIP Switches)
Los DIP switches se leen justo después de recibir el `startSignal`:
* `DIPA`: Activa modo "Snake" (ataque zigzagueante).
* `DIPB`: Activa modo "Turkish" (estrategia de pequeños saltos y esperas en el borde).
* `DIPC / DIPD`: Definen la coreografía de apertura (ej. ir directo, hacer un movimiento de 45°, o girar 180°).

## Misiones / Tareas Pendientes
* [ ] **Comprensión Profunda:** Terminar de analizar cada rama lógica de `tasks.cpp` y su relación con `movements.cpp`.
* [ ] **Validación Física:** Realizar la primera prueba con el robot real para verificar que los motores y sensores responden según lo esperado.
* [ ] **Mini-Calibración:** Ejecutar una prueba de movimiento controlada para ajustar los primeros parámetros de velocidad y giro en pista.
