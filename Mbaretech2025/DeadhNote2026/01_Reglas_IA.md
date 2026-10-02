# Reglas para Agentes de Inteligencia Artificial (IA)

> **[!IMPORTANT]**
> **ESTA ES UNA LECTURA OBLIGATORIA.** Si eres un agente de IA asignado a este proyecto, debes leer y asimilar estas reglas antes de proponer cualquier cambio en el código o responder al usuario.

## 1. Contexto del Proyecto
* **Categoría:** Megasumo (3kg). **Bajo ningún concepto** te refieras al proyecto como "minisumo". Es una categoría de peso pesado con imanes de neodimio, donde los choques son violentos y la tracción lo es todo.
* **Microcontrolador:** ESP32-S3 programado con el framework de Arduino sobre PlatformIO.
* **Arquitectura de Software:** Basada fuertemente en FreeRTOS. La lógica principal vive en una tarea dedicada llamada `stateMachineTask` dentro de `tasks.cpp`.

## 2. Reglas de Interacción
* **Idioma:** Toda la comunicación, explicaciones y comentarios en el código deben realizarse en **Español**.
* **Simulaciones Mentales:** Antes de programar lógicas complejas, realiza simulaciones paso a paso de lo que hará el robot en el Dohyo para validar la estrategia con el usuario.
* **Prohibición de Redundancia:** Antes de proponer cualquier cambio o realizar una pregunta al usuario, es OBLIGATORIO que revises el Historial RAW (`04`), los Razonamientos (`99`), el Estado Actual (`02`) y la Bitácora (`03`). Esto es para evitar repetir análisis ya realizados, proponer caminos que ya se descartaron o cometer errores técnicos que ya fueron identificados y documentados.

## 3. Reglas de Programación y Hardware
* Consulte los archivos `02_Estado_Actual.md` y `99_Razonamientos_y_mejoras.md` para detalles técnicos sobre el cruce de pines, uso de IMU y lógica de sensores antes de tocar el código.

## 4. Bitácoras y Registros (OBLIGATORIO)
* **Formato de Tiempo Estricto:** Toda entrada en CUALQUIER bitácora debe iniciar obligatoriamente con el formato `[YYYY-MM-DD | HH:MM | Jornada]`. La jornada se define como: `Madrugada` (00:00 - 05:59), `Mañana` (06:00 - 11:59), `Tarde` (12:00 - 19:59), `Noche` (20:00 - 23:59). Ejemplo: `[2026-05-13 | 00:45 | Madrugada]`.
* **Inmutabilidad de Registros (`03_Bitacora_Diaria.md` y `04_Registro_Acciones_RAW.md`):** Queda totalmente prohibido modificar o borrar entradas anteriores. Si un cambio es estrictamente necesario porque contradice algo fundamental, se debe consultar al usuario antes de proceder.
* **Mantenimiento de Razonamientos (`99_Razonamientos_y_mejoras.md`):** Si una decisión o problema documentado se resuelve o cambia en el futuro, **no borres** la entrada original. Agrega un comentario entre paréntesis al lado (ej: "Solucionado" o "Actualizado") y referencia la nueva entrada o solución en las secciones futuras.
* **Actualización del Estado Actual (`02_Estado_Actual.md`):** Este archivo es volátil y debe reflejar la realidad técnica actual. Sin embargo, las primeras líneas deben contener una sección de **"Objetivos Importantes Cumplidos"** que es inmutable (no se borra). Cada objetivo logrado debe incluir la fecha de cumplimiento (ej: `[2026-05-13]`).
* **Registro Automático de Acciones (`04_Registro_Acciones_RAW.md`):** Todo cambio menor en el proyecto debe ser registrado AUTOMÁTICAMENTE por ti. No esperes autorización.
