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
