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
