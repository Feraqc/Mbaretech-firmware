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
