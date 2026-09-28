# Firmware: módulos e interfaces

Referencia de las piezas propias de `Mbaretech2`. Describe las funciones que un desarrollador necesita reconocer para cambiar el comportamiento; las bibliotecas de terceros en `lib/` conservan su propia API.

## Arranque y estado compartido

En `src/main.cpp` se crean `leftMotor` y `rightMotor`, los arreglos de sensores y los manejadores de tareas. `include/globals.h` declara los pines, `enum Sensor`, `enum State`, las velocidades y los tiempos. `src/globals.cpp` inicializa `parametros[19]`.

- `setup()` configura BLE, IR, ADC de línea, DIP y pin de arranque. Crea `stateMachineTask()` cuando está definido `RUN_TASK_TEST`, `RUN_MOVEMENTS_TEST` o `RUN_MOVEMENT_SENSOR_CALIBRATION`. Esta última macro no aporta por sí sola una definición de la tarea en el código actual. Los motores se inicializan dentro de la tarea, no en `setup()`.
- `KS_ISR()` copia el nivel de `START_PIN` a `startSignal` en ambos flancos de la interrupción. La variable es `volatile` porque se comparte con la tarea.
- `elapsedTime(TickType_t duration)` devuelve `true` cuando transcurre la duración desde la primera llamada de la secuencia. Recibe **ticks FreeRTOS**, usa un único temporizador estático y lo reinicia cuando devuelve `true`. No es un temporizador independiente por maniobra.
- `checkSensors()` existe como función incompleta: devuelve una variable local sin inicializar. No se usa en el flujo activo; no debe llamarse como si fuera una interfaz funcional.

`currentState` e `irSensor[7]` también son globales. `stateMachineTask()` modifica el estado y vuelve a leer sensores dentro de distintas ramas; por ello, una lectura previa no siempre representa el valor que gobierna la maniobra siguiente. `dipSwitch[4]` está declarado, pero la apertura de combate lee directamente los GPIO DIP.

En la implementación activa, los `case` principales se agrupan en espera y búsqueda (`IDLE`, `BRAKE`, `FORWARD`), evasión del borde (`LINE_RETREAT`), giros (`TURN_LEFT_45`, `TURN_RIGHT_45`, `TURN_LEFT_90`, `TURN_RIGHT_90`, `TURN_180`), correcciones cortas y maniobras compuestas (`SHORT_LEFT_MOVE`, `SHORT_RIGHT_MOVE`, `L_MOVEMENT_45`, `R_MOVEMENT_45`, `GIRO_U_*`). También hay ramas `_IF` y `MOVEMENT_45` de carácter experimental. `SNAKE` y `TURKISH` son nombres del enum, pero la ruta activa los utiliza como indicadores booleanos, no como estados con un `case` propio.

## Clase `Motor`

Definida en `include/motor.h`; hay una instancia por rueda. Encapsula dos pines de dirección y un canal PWM LEDC.

- `Motor(pwmPin, A0pin, A1pin, pwmChannel)` guarda los pines y el canal que utilizará la instancia.
- `begin()` configura los GPIO como salidas y prepara el temporizador y el canal LEDC a 20 kHz y 10 bits. Debe ejecutarse antes de ordenar movimiento.
- `setSpeed(percentage)` convierte un porcentaje a duty LEDC y lo limita al intervalo 1–990. El valor 0 **no** produce duty 0: se limita a 1. No modifica los pines de dirección.
- `forward(speed)` y `backward(speed)` fijan direcciones opuestas y llaman a `setSpeed()`.
- `brake()` pone ambas entradas de dirección en bajo. No pone el duty en cero; el efecto mecánico exacto depende del controlador de motores, cuyo modelo sigue pendiente.

Los campos `currentSpeed` y `ledc_channel` están declarados públicamente, pero el primero no se actualiza y el segundo no es la configuración local creada dentro de `begin()`. No deben tratarse como estado fiable del hardware.

## Sensores de línea

Las funciones están implementadas en `src/lineSensor.cpp` y se compilan con `RUN_LINE_SENSOR` o `RUN_SENSORS_TEST`.

- `lineSensorsInit()` configura resolución y atenuación de los dos canales ADC1 delanteros. La configuración de los canales traseros ADC2 está comentada.
- `readLineSensorFront(channel)` devuelve una lectura ADC1 sin filtrarla.
- `checkLineSensora(measurement)` y `checkLineSensorb(measurement)` mantienen contadores independientes para izquierda y derecha. Devuelven `true` tras siete lecturas consecutivas menores o iguales a `THRESHOLD`; una lectura mayor reinicia su contador. Los contadores son `uint8_t`, por lo que pueden desbordarse si la condición permanece activa durante muchas lecturas.
- `readLineSensorBack(channel)` intenta leer ADC2 y devuelve `-1` si falla. No participa en el combate y los canales traseros no se inicializan actualmente.

El combate usa `lineSensor[0]` y `lineSensor[1]` para los sensores delanteros. El arreglo tiene cuatro posiciones, pero la ruta activa no procesa las dos traseras.

## Comunicación BLE y registro de datos

`include/bluetoothComm.h` declara la interfaz; `src/bluetoothComm.cpp` implementa el UART BLE con los mismos UUID RX/TX y nombre `MBARETECH`. RX recibe **un comando por escritura** (máximo 63 bytes, con CR/LF opcional). Sus callbacks encolan comandos sin esperar; la tarea `bluetoothLog` los procesa y transmite. La cola admite 16 comandos; un exceso se descarta. Suscribirse a TX y enviar cualquier tecla (también Enter) abre el menú; `menu` y `registro` siguen disponibles. `ayuda` recuerda los comandos. El saludo de conexión puede preceder a la suscripción.

El protocolo existente `ÍNDICE VALOR` funciona dentro y fuera del menú, incluso durante la entrada de intervalo. Ahora valida índices 0–18. Los valores conservan su semántica previa. No se implementaron menús de parámetros ni de calibración.

```text
=== REGISTRO DE DATOS ===
Estado: DETENIDO
Intervalo: 100 ms
1. Sensores de línea [OFF]
2. Sensores IR [OFF]
3. Máquina de estados [OFF]
4. Yaw IMU [OFF]
5. Iniciar registro
6. Detener registro
7. Cambiar intervalo
0. Volver
```

1–4 alternan canales independientemente, también durante el registro. 5 inicia y muestra canales e intervalo; 6 detiene sin reiniciar el firmware. 7 solicita un entero entre **20 y 60000 ms**, o 0 para cancelar. La entrada inválida mantiene el intervalo previo. El valor inicial es **100 ms**, todos los canales OFF y registro detenido. Fuera del menú, la primera tecla solo lo abre, sin ejecutar la opción numérica. Dentro del menú, las opciones válidas conservan su acción y cualquier otra tecla vuelve a mostrarlo. La entrada de intervalo y los comandos `ÍNDICE VALOR` conservan su interpretación. 0 sale del menú sin detener el registro; para detenerlo, volver con `menu` y enviar 6. La desconexión detiene el registro, conserva canales/intervalo y reinicia publicidad BLE.

### Formatos y origen de datos

Cada registro termina en LF. TX fragmenta el flujo en notificaciones de hasta 20 bytes para admitir el MTU predeterminado. El cliente debe concatenar bytes antes de separar líneas o decodificar UTF-8. Las respuestas del menú comparten el flujo: filtrar por los prefijos siguientes para análisis.

| Canal | Formato exacto | Origen |
| --- | --- | --- |
| Línea | `LINEA,tiempo_ms,left,right` | Últimos ADC delanteros sin filtrar, publicados por `readLineSensorFront()`; no incluye sensores traseros. -1 indica que aún no hubo lectura. |
| IR | `IR,tiempo_ms,SIDE_LEFT,SHORT_LEFT,TOP_LEFT,TOP_MID,TOP_RIGHT,SHORT_RIGHT,SIDE_RIGHT` | Siete valores 0/1 de `irSensor[]`, en orden de `enum Sensor`; 1 significa detección normalizada por el control. |
| Yaw | `YAW,tiempo_ms,yaw_grados` | `IMU::currentAngle`, calculado por el `getYaw()` existente a partir del cuaternión DMP; resolución entera heredada. |
| Transición | `ESTADO,tiempo_ms,ANTERIOR->NUEVO` | Nombres reales de `enum State`, por ejemplo `IDLE->FORWARD`. |
| Pérdidas de eventos | `PERDIDOS,ESTADO,cantidad` | Transiciones descartadas por cola llena desde el informe anterior. |

El orden IR de MBARETECH_2 corresponde a IR1, IR2, IR3, IR4, IR5, IR6, IR7. Para MBARETECH_1 se omiten los laterales inexistentes: SHORT_LEFT, TOP_LEFT, TOP_MID, TOP_RIGHT, SHORT_RIGHT (IR2–IR6). La compilación verificada en esta iteración es MBARETECH_2.

Línea, IR y yaw son **periódicos**, con tiempo de emisión `millis()` y comparación por resta sin signo que tolera su desbordamiento. Son muestras de los últimos datos disponibles, no una adquisición simultánea: las maniobras bloqueantes existentes pueden dejar lecturas de línea/IR antiguas. No se agregan lecturas ADC/GPIO ni filtros al módulo Bluetooth. No se recuperan períodos atrasados en ráfagas.

Las transiciones son **eventos**: `changeState()` conserva la asignación de estado y, si cambió efectivamente, encola tiempo de transición, estado anterior y nuevo con espera cero. Las asignaciones en `tasks.cpp`, `movements.cpp` y `movements_old.cpp` pasan por ese punto, incluso dentro de maniobras bloqueantes. No se imprime un estado repetido ni un estado inicial artificial al iniciar. La cola tiene 64 posiciones; cada activación utiliza una sesión para descartar eventos antiguos después de detener/reiniciar o alternar el canal. La tarea BLE drena hasta ocho eventos por vuelta. El transporte BLE y su congestión nunca esperan dentro de la tarea de control.

### IMU y arquitectura del registro

`include/dataLogging.h` y `src/dataLogging.cpp` contienen `LoggingConfig`, menú de registro, temporización, caché de línea y cola de transiciones. Solo la tarea BLE modifica configuración/menú; valores compartidos con productores usan atómicos y la cola FreeRTOS. La tarea BLE duerme 5 ms entre vueltas; estas pausas pertenecen a esa tarea, no al control.

La tarea separada `loggingIMU` consulta el DMP cada 10 ms mientras yaw está seleccionado y el registro activo. Inicializa la clase IMU existente en el primer uso, sin cambiar el arranque del combate. **Mantener el robot inmóvil durante esta inicialización**, porque `IMU::begin()` ya incluye calibración. No es un nuevo menú de calibración. Esta operación puede tardar, pero no bloquea comandos BLE ni la FSM. `IMU.h` expone disponibilidad del DMP y del paquete recién leído; no agrega otra fórmula de orientación.

Se emite `YAW,tiempo_ms,NA` mientras inicia, si falla la IMU, si aún no existe una muestra o si la última supera 500 ms de antigüedad. Una inicialización fallida no se repite continuamente: revisar hardware y enviar 5 para reintentar, o desactivar/activar yaw. No habilitar RUN_GYRO_TEST: el registro ya tiene su propia tarea IMU. Al detener, cesa el sondeo de yaw; el DMP permanece habilitado. Notificaciones BLE no tienen entrega garantizada; intervalos pequeños y muchos cambios pueden saturar la salida. El registro es de desarrollo, con tiempos aproximados sujetos al planificador y al enlace.

`parametros` se usa principalmente en `src/movements.cpp`, no como fuente de los ajustes principales de `src/tasks.cpp`. Sus índices se agrupan así: 0–1 para comando y movimiento de prueba; 2 para velocidad de avance; 3–11 para velocidades y duraciones de giros y movimientos cortos; 12 para umbral; 13–17 para estrategia `turkish` y giros en U; 18 para compensación de velocidad. Los valores iniciales están en `src/globals.cpp`.

### Verificación

- Compilar: `pio run -d Mbaretech2` desde la raíz.
- Pruebas de comportamiento en Windows con Python y MSVC Build Tools: `python Mbaretech2/test/logging/run.py`. Compilan el módulo de registro real con sustitutos de hardware/FreeRTOS; cubren menú, selección simultánea, intervalos inválidos, desbordamiento de millis, inicio/parada, desconexión, transiciones breves, cola llena y yaw no disponible/antiguo.
- Validación pendiente en placa: suscribirse a TX, enviar `menu`, activar 1–4 y enviar 5; comprobar ADC izquierdo/derecho, cada IR en orden y yaw girando el robot después de calibrar. Cambiar intervalo con 7, detener con 6, reiniciar y desconectar/reconectar. Verificar `0 0` como comando de parámetros y rechazo de índice 19.
- Confirmar en placa que las notificaciones fragmentadas se reconstruyen, que la cadencia y las pérdidas son aceptables y que motores/FSM conservan su comportamiento. Las pruebas host no simulan radio, DMP ni temporización física.

## Clase `IMU` y rutas alternativas

`include/IMU.h` envuelve un MPU6050 por I²C. El registro de yaw la instancia en su tarea dedicada y la inicia bajo demanda; `src/main.cpp` no la inicia durante el arranque.

- `begin()` inicia I²C en SDA 15 y SCL 16, configura y calibra el MPU6050 y habilita su DMP si la inicialización funciona. Informa del resultado por `Serial`.
- `getData()` lee un paquete disponible del DMP y actualiza orientación y `currentAngle`.
- `transmitData()` imprime yaw, pitch y roll por `Serial`.
- `checkRotation(desiredAngle)` compara `currentAngle` con un ángulo inicial estático y devuelve `true` al alcanzar la diferencia solicitada. Este método no controla ningún motor por sí mismo.
- `getYaw(data, q, gravity)` calcula el ángulo usado por `getData()`. El parámetro `gravity` está en la firma, pero no interviene en el cálculo actual.

`src/movements.cpp` define otra `stateMachineTask()` para ensayar maniobras con `parametros`; se compila únicamente con `RUN_MOVEMENTS_TEST`. `src/movements_old.cpp` guarda una versión anterior y requiere además `OLD`. Los programas de `src/tests/` son rutas de diagnóstico independientes, no pruebas unitarias automáticas. Sus condiciones de uso están en [desarrollo.md](desarrollo.md).

## Funciones declaradas sin ruta activa

`globals.h` declara `lineSensorTask()`, sin definición para combate. `changeState()` ahora se define en `src/dataLogging.cpp` como punto de asignación y registro de transiciones. También declara pines de encoder sin lectura asociada. Los nombres de `State` incluyen opciones que la máquina activa no maneja; consultar los `case` de `src/tasks.cpp` antes de asumir que un estado está implementado.


## Modo de prueba de sensores sin FSM

La configuración activa de esta prueba usa solamente `MBARETECH_2`, `RUN_LINE_SENSOR` y `RUN_SENSORS_TEST`. No combinar con otros modos que definan `loop()`.

`src/tests/sensorsTest.cpp` actualiza los siete IR, DIP y ambos ADC delanteros, aplica `checkLineSensora()`/`checkLineSensorb()` y cede la CPU durante 10 ms por vuelta. Se eliminó el bucle de parámetros que no terminaba y la pausa anterior de dos segundos. La cadencia de adquisición es independiente del intervalo de salida BLE (100 ms por defecto).

Este modo no crea la tarea de combate ni emite comandos de motor. BLE y la tarea de yaw siguen disponibles: enviar `menu`, activar los canales deseados y enviar 5. No habrá registros ESTADO porque no se ejecuta la FSM. La verificación de detección física y yaw requiere la placa.


## Depuración por cable

`Serial` se inicia siempre a **115200 baudios**, antes de BLE, sin requerir `DEBUG` ni esperar a que se abra un terminal. `sendData()` duplica menús, respuestas y registros en Serial y en BLE cuando hay conexión.

El terminal serie también acepta los mismos comandos: enviar texto seguido de Enter (LF, CR o CRLF). Enter solo abre o muestra el menú. La entrada es incremental y no espera una línea completa; admite 63 bytes y rechaza líneas más largas hasta el siguiente terminador. El menú y configuración son compartidos entre ambos transportes.

Para probar sin BLE: abrir el monitor a 115200, pulsar Enter, enviar 1/2/4 según los canales deseados y 5 para iniciar. No es necesario conectar un cliente BLE. Si se desconecta un cliente BLE durante el registro, se conserva la parada automática existente; enviar 5 desde el menú serie para reanudar. Las impresiones propias de IMU o DEBUG también pueden aparecer en Serial; filtrar prefijos para analizar registros. La salida serie se realiza en la tarea de comunicaciones, no en la FSM.

### Diagnóstico de yaw no disponible

Además de YAW,...,NA, se emite una línea IMU cuando cambia el estado: ESPERANDO_INICIO, CALIBRANDO, LISTA, SIN_PAQUETES_RECIENTES_DMP, ERROR_TAREA, ERROR_BUS_I2C, NO_DETECTADA o ERROR_DMP,codigo. Los mensajes llegan por Serial y BLE. NO_DETECTADA corresponde al chequeo del MPU6050 en 0x68 con SDA=15/SCL=16; comprobar conexión, alimentación y dirección AD0. ERROR_DMP conserva el código devuelto por la biblioteca. Tras inicializar el DMP se limpia FIFO. Los diagnósticos permiten distinguir fallo de conexión, calibración y falta de muestras; no implican que el hardware haya sido verificado.

## Prueba aislada IMU por Serial

`src/tests/gyroTest.cpp` usa únicamente métodos existentes de IMU.h: begin(), isReady(), getInitError(), getData() y hasYaw(), junto con currentAngle. Activar solo `-DRUN_GYRO_TEST` en el entorno habitual. No inicia BLE ni la FSM.

Abrir Serial a 115200 y mantener el sensor inmóvil durante la calibración. La prueba consulta el DMP cada 10 ms e imprime muestras válidas hasta cada 100 ms como `YAW,tiempo_ms,grados`. Si falla begin(), informa el error (-2: bus; -3: MPU6050 no detectado; otros negativos: fallo DMP). Corregir y reiniciar. Si no hay paquetes recientes durante un segundo, informa SIN_PAQUETES_RECIENTES_DMP en lugar de imprimir un ángulo antiguo.

Se eliminaron las funciones adicionales de diagnóstico: este modo ya no escanea direcciones ni imprime ejes crudos. Usa la configuración existente de IMU.h (SDA 15, SCL 16, dirección predeterminada 0x68).

Para volver al registro normal, desactivar RUN_GYRO_TEST y restaurar MBARETECH_2, RUN_LINE_SENSOR y RUN_SENSORS_TEST.

### Corrección de getYaw()

getYaw() usa atan2(2(xy-wz), w²+x²-y²-z²), equivalente a la fórmula anterior para un cuaternión unitario e independiente de una escala común no nula. Conserva signo, grados e interfaz entera currentAngle. gyroTest.cpp imprime currentAngle como primera columna para verificar esta ruta; pitch/roll continúan usando ypr. La conversión de cuaterniones y ypr[0] de la biblioteca no se modificaron.
