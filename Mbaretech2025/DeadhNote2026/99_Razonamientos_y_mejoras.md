# Razonamientos, Estrategias y Mejoras a Futuro

Este documento recopila las vulnerabilidades del código actual y las estrategias de Megasumo que se deben implementar para solucionarlas. 
> Cada nueva sesión de análisis o estrategia debe quedar enmarcada bajo su fecha y hora correspondiente.

---
### [2026-05-13 | 00:50 | Madrugada] - Vulnerabilidades Base y Tácticas de Solución

## 1. El Problema de las Banderas (Señuelos)
**Situación Actual:** El código es ingenuo. Si cualquier sensor (ej. `SIDE_LEFT`) detecta un reflejo IR, asume que es el bloque de 3kg del enemigo e inicia la secuencia de ataque. Esto es letal contra Megasumos que usan banderas (telas o palos laterales), ya que Mbaretech atacará la bandera regalando su flanco al chasis real del enemigo.

**Soluciones Propuestas:**
1. **Verificación Geométrica (Ancho):** Antes de pasar al estado `FORWARD` a toda velocidad, el robot debe exigir que al menos **dos sensores contiguos** estén activos (ej. `TOP_MID` + `SHORT_LEFT`). Esto confirma que el objeto enfrente tiene volumen/ancho y no es un simple mástil.
2. **Debounce / Filtro de Flameo:** Exigir que el sensor mantenga la lectura en `HIGH` durante una ventana de tiempo (ej. 30ms continuos) para descartar telas que flamean.
3. **Ataque Escalonado:** Entrar a `FORWARD` con un 60% o 70% de potencia para "cebar" al enemigo. Solo desatar el 100% (`MAX_SPEED`) si los sensores cortos confirman contacto o si el MPU6050 detecta choque masivo.

## 2. Precisión de los Giros (Lazo Abierto vs Lazo Cerrado MPU6050)
**Situación Actual:** El robot gira de forma rígida basado en milisegundos (`TURN_LEFT_90_DELAY`). Como se descubrió analizando las calibraciones ("la prueba de la escoba"), estos tiempos se ajustan a ojo. El grave problema en competición es que a medida que la batería se descarga o el dohyo acumula polvo, la tracción cambia y el robot girará menos grados en ese mismo tiempo, perdiendo su ventaja posicional.

**Solución Propuesta (Giroscopio):**
Reemplazar los delays estáticos por lecturas dinámicas integradas del giroscopio (MPU6050). En lugar de `while(!elapsedTime(70))`, utilizar `while(gradosGirados < 90)`. Esto garantiza matemáticamente que la pala quede perfectamente alineada al objetivo en todo momento, anulando los efectos de la caída de voltaje o derrapes.

## 3. Confirmación de Impacto en Combate
**Situación Actual:** El robot empuja a ciegas asumiendo contacto sólido simplemente porque el sensor frontal lee un objeto a centímetros de distancia.
**Solución Propuesta:** Utilizar el acelerómetro y los Encoders. Si se envía orden de ataque pero no se registra un impacto inercial (Acelerómetro) o si las ruedas siguen girando sin fricción (Encoders), la máquina debe deducir que está embistiendo aire o un señuelo endeble, abortando el empuje a fondo.
**Actualización [2026-09-29]:** los pines de los encoders (`ENCODER_LEFT`/`RIGHT`) fueron repurpuestos para los sensores de línea traseros (LS3/LS4) — la vía de "ruedas girando sin fricción" ya no está disponible como encoder real. Queda solo el acelerómetro como sensor inercial disponible para esto.

---
### [2026-09-30 | 02:00 | Madrugada] - Avance: Giro de lazo cerrado confirmado viable, arranca "Modo Martillo"

**Punto 2 (giros de lazo cerrado) — confirmado viable, no implementado todavía:** el usuario verificó a mano con `src/tests/pruebaIMU.cpp` que el `yaw` integrado desde `gz` coincide bien contra la rotación física real ("esta perfecto"). Sigue pendiente escribir la lógica real en `tasks.cpp` que reemplace `while(!elapsedTime(TURN_LEFT_90_DELAY))` por algo basado en `yaw`.

## 4. Modo Martillo (nueva idea, no estaba en este documento)
**Situación propuesta por el usuario:** si el robot está empujando al sumo (`FORWARD`/ataque) y queda trabado en un empuje parejo sin ganar terreno, en vez de seguir empujando en el lugar, conviene retroceder un poco y volver a embestir (un "martillazo" repetido) en vez de gastar la pelea en un forcejeo estático.

**Problema físico identificado:** la aceleración lineal es ≈0 tanto si el robot está trabado (fuerza neta cancelada por el rival) como si se mueve a velocidad constante — un umbral simple sobre `|a|` no alcanza para distinguir los dos casos.

**Solución en desarrollo:** integrar la aceleración hacia adelante (eje `ay` del MPU6050, confirmado 2026-09-30 con pruebas físicas: `ay` positivo = adelante, `ay` negativo = atrás, IMU montado con el eje Y paralelo a la dirección de avance) durante una ventana corta y acotada justo al arrancar el empuje, para estimar la velocidad ganada en ese lapso. Si la velocidad estimada queda por debajo de un umbral, se asume "trabado" y dispara el martillo (retroceder + reintentar). Se creó `src/tests/pruebaMartillo.cpp` (flag `RUN_PRUEBA_MARTILLO`) para medir esta velocidad estimada empujando contra algo trabado y contra algo libre por separado, y sacar de ahí los dos parámetros que faltan: la duración de la ventana de medición y el umbral de "trabado". **Pendiente:** correr esa prueba en las dos condiciones y con esos números implementar el martillo real en `tasks.cpp` (todavía no existe ahí).

**Actualización [2026-10-01 | 00:35 | Madrugada] — la integración de velocidad queda descartada:** con mediciones reales (después de corregir un bug de `dt`), el robot alcanza su velocidad en los primeros ~100ms de empuje y después la aceleración es ≈0. Además, la lectura I2C del IMU se cuelga 26-128ms justo al arrancar los motores. La información útil queda en una ventana que no se puede leer de forma confiable, así que la integración no distingue "trabado" de "libre". **Nueva dirección:** detectar el *contacto* por el pico del golpe (`picoFrenada`; en vacío es estable en ~2.1-2.3g, el choque debería superarlo con margen) y decidir "trabado" por tiempo de contacto sostenido, sin medir velocidad. Pendiente medir el golpe real contra la pared. Ver también el riesgo del bus I2C para el punto 2 (giros con giroscopio).
