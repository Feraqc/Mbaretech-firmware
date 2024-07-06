#include "globals.h" 

bool dipSwitchPin[4];

Motor leftMotor(PWM_A,PIN_A0,PIN_A1,CHANNEL_LEFT);
Motor rightMotor(PWM_B,PIN_B0,PIN_B1,CHANNEL_RIGHT);
IMU imu;

//SHARED VARIABLES
int currentAngle;
int desiredAngle;
bool lineSensor[4];

//VOLATILE VARIABLES
volatile State currentState = IDLE;
volatile bool irSensor[7];
volatile bool startSignal = false;
volatile unsigned long encoderLeftCounter = 0;
volatile unsigned long encoderRightCounter = 0;

// ISR DEFINITION
void  IRAM_ATTR IR1_ISR() { irSensor[0] = digitalRead(IR1); }
void  IRAM_ATTR IR2_ISR() { irSensor[1] = digitalRead(IR2); }
void  IRAM_ATTR IR3_ISR() { irSensor[2] = digitalRead(IR3); }
void  IRAM_ATTR IR4_ISR() { irSensor[3] = digitalRead(IR4); }
void  IRAM_ATTR IR5_ISR() { irSensor[4] = digitalRead(IR5); }
void  IRAM_ATTR IR6_ISR() { irSensor[5] = digitalRead(IR6); }
void  IRAM_ATTR IR7_ISR() { irSensor[6] = digitalRead(IR7); }
void  IRAM_ATTR encoderLeftISR(){encoderLeftCounter++;}
void  IRAM_ATTR encoderRightISR(){encoderRightCounter++;}


// Create AsyncWebServer instance on port 80
AsyncWebServer server(80);
// Create a WebSocket instance on port 81
AsyncWebSocket ws("/ws");
// Network credentials
const char* ssid = "Pixel";     // Replace with your WiFi network name
const char* password = "guana123"; // Replace with your WiFi password

void setup() {
    esp_efuse_write_field_cnt(ESP_EFUSE_VDD_SPI_FORCE, 1); 
    // Initialize Serial communication
    Serial.begin(115200);
    imu.begin();
    // Motors
    rightMotor.begin();
    leftMotor.begin();

    // Line sensors 
    lineSensorsInit();

    // IR Sensors
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR6, INPUT);
    pinMode(IR7, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR1), IR1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR2), IR2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR3), IR3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR4), IR4_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR5), IR5_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR6), IR6_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR7), IR7_ISR, CHANGE);

    // DIPS
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    // Start pin
    pinMode(START_PIN, INPUT);

    // Encoders
    pinMode(ENCODER_LEFT, INPUT);
    pinMode(ENCODER_RIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), encoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), encoderRightISR, RISING);

    // Create the queues
    imuDataQueue = xQueueCreate(10, sizeof(float));  // tamanho arbitrario
    cmdQueue = xQueueCreate(10, sizeof(float));

    if (imuDataQueue == NULL) {
        Serial.println("Failed to create sensor data queue");
        while (true);  // Halt the program
    }
    if (cmdQueue == NULL) {
        Serial.println("Failed to create command queue");
        while (true);  // Halt the program
    }

    xTaskCreate(stateMachineTask, "stateMachineTask", 4096, NULL, 1, &stateMachineTaskHandle);
    xTaskCreate(imuTask, "imuTask", 4096, NULL, 1, &imuTaskHandle);
    xTaskCreate(lineSensorTask, "lineSensorTask", 4096, NULL, 1, &lineSensorTaskHandle);

    // Create tasks conditionally
    #ifndef RUN_GYRO_TEST
    #ifndef RUN_IR_SENSOR_TEST
    #ifndef RUN_LS_IR_SENSOR_TEST
    #ifndef RUN_WIFI_SENSORS_TEST
    #ifndef RUN_DRIVER_TEST
    #ifndef RUN_LINE_SENSOR_TEST
    //xTaskCreate(mainTask, "MainTask", 2048, NULL, 1, NULL);
    #endif  // RUN_LINE_SENSOR_TEST
    #endif  // RUN_IR_SENSOR_TEST
    #endif  // RUN_GYRO_TEST
    #endif  // RUN_LS_IR_SENSOR_TEST
    #endif  // RUN_WIFI_SENSORS_TEST
    #endif  // RUN_DRIVER_TEST

    #ifdef RUN_WIFI_SENSORS_TEST
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
        Serial.println(WiFi.status());
    }
    Serial.println("Connected to WiFi");

    // Setup WebSocket server
    ws.onEvent(onEvent);
    server.addHandler(&ws);

    // Start server
    server.begin();
    Serial.println("WebSocket server started");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    xTaskCreate(webSocketTask, "WebSocketTask", 2048, NULL, 1, NULL);
    #endif
}




#ifndef RUN_GYRO_TEST
#ifndef RUN_IR_SENSOR_TEST
#ifndef RUN_LS_IR_SENSOR_TEST
#ifndef RUN_WIFI_SENSORS_TEST
#ifndef RUN_DRIVER_TEST
#ifndef RUN_LINE_SENSOR_TEST

void mainTask(void *pvParameters) {
    // Main loop of the FreeRTOS task
    while (true) {
        handleState();
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100 ms
    }
}

// void handleState() {    
//     switch (currentState) {
//             // TODO:
//             // INITIAL_MOVEMENT
//             // MID_MOVE,
//             // TOP_LEFT_MOVE,
//             // TOP_RIGHT_MOVE,
//             // SIDE_LEFT_MOVE,
//             // SIDE_RIGHT_MOVE,
//             // BOUND_MOVE, ---> Todavia no escribi el caso hay que hablar como
//             // manejar DEFAULT_ACTION_STATE
//         case WAIT_ON_START:
//             Serial.println("State: WAIT_ON_START");
//             if (digitalRead(START_PIN)) {
//                 if (digitalRead(DIPA)) {  // Se puede cambiar a otro dip segun
//                                           // necesitemos para estrategia
//                     changeState(INITIAL_MOVEMENT);
//                 }
//                 else {
//                     // TODO: Leer todos los sensores
//                 }
//             }
//             else {
//                 // TODO: Motor OFF
//             }
//             break;

//         case INITIAL_MOVEMENT:
//             Serial.println("State: INITIAL_MOVEMENT");
//             // Ver como separar los movimientos iniciales
//             // No necesita break

//         case MID_SENSOR_CHECK:
//             Serial.println("State: MID_SENSOR_CHECK");
//             if (sensorReadings[TOP_MID]) {
//                 changeState(MID_MOVE);
//             }
//             else {
//                 changeState(TOP_SENSORS_CHECK);
//             }
//             break;

//         case MID_MOVE:
//             Serial.println("State: MID_MOVE");
//             // TODO: Do movement
//             // IF FRONTLEFT AND FRONTRIGHT --> 100%
//             // else if FRONTLEFT --> shortLeft
//             // else if FRONTRIGHT --> shortRight
//             // else ---> 30% o algo asi
//             changeState(MID_SENSOR_CHECK);
//             break;

//         case TOP_SENSORS_CHECK:
//             Serial.println("State: TOP_SENSOR_CHECK");
//             if (sensorReadings[TOP_LEFT]) {
//                 changeState(TOP_LEFT_MOVE);
//             }
//             else if (sensorReadings[TOP_RIGHT]) {
//                 changeState(TOP_RIGHT_MOVE);
//             }
//             else {
//                 changeState(SIDE_SENSORS_CHECK);
//             }
//             break;

//         case TOP_LEFT_MOVE:
//             Serial.println("State: TOP_LEFT_MOVE");
//             // TODO: LeftTurn45
//             changeState(MID_SENSOR_CHECK);
//             break;

//         case TOP_RIGHT_MOVE:
//             Serial.println("State: TOP_RIGHT_MOVE");
//             // TODO: RightTurn45
//             changeState(MID_SENSOR_CHECK);
//             break;

//         case SIDE_SENSORS_CHECK:
//             Serial.println("State: SIDE_SENSORS_CHECK");
//             if (sensorReadings[SIDE_LEFT]) {
//                 changeState(SIDE_LEFT_MOVE);
//             }
//             else if (sensorReadings[SIDE_RIGHT]) {
//                 changeState(SIDE_RIGHT_MOVE);
//             }
//             else {
//                 changeState(DEFAULT_ACTION_STATE);
//             }
//             break;

//         case SIDE_LEFT_MOVE:
//             Serial.println("State: SIDE_LEFT_MOVE");
//             // TODO: LEFTTURN90
//             changeState(MID_SENSOR_CHECK);
//             break;

//         case SIDE_RIGHT_MOVE:
//             Serial.println("State: SIDE_RIGHT_MOVE");
//             // TODO: RIGHTTURN90
//             changeState(MID_SENSOR_CHECK);
//             break;

//         case DEFAULT_ACTION_STATE:
//             Serial.println("State: DEFAULT_ACTION_STATE");
//             // TODO: Aca es un dependiendo del switch por ejemplo
//             // para saber que queremos que haga por default
//             // puede ser turco, ir para delante, quedarse quieto
//             changeState(
//                 MID_SENSOR_CHECK);  // mientras le pongo que mire sensores nomas

//         default:
//             Serial.println("State: UNKNOWN");
//             break;
//     }
// }

void changeState(State newState) { currentState = newState; }

void loop() {}

#endif  // RUN_LINE_SENSOR_TEST
#endif  // RUN_IR_SENSOR_TEST
#endif  // RUN_GYRO_TEST
#endif  // RUN_LS_IR_SENSOR_TEST
#endif  // RUN_WIFI_SENSORS_TEST
#endif  // RUN_DRIVER_TEST

#ifdef RUN_WIFI_SENSORS_TEST

// Function to handle WebSocket events
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        client->text("Hello from ESP32 Server");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        Serial.printf("WebSocket client #%u sent data: %s\n", client->id(), data);
    }
}

void webSocketTask(void *pvParameters) {
    while (true) {
        ws.cleanupClients(); // Maintain WebSocket clients
        sendSensorData();
        vTaskDelay(pdMS_TO_TICKS(100)); // Send data every 100 milliseconds
    }
}

// Function to handle the sensors endpoint
void sendSensorData() {
    String sensorData = "";
    for (int i = 0; i < 7; i++) {
        sensorData += "IR" + String(i + 1) + ": " + String(sensorReadings[i]) + "\t";
    }
    sensorData += "\t";
    sensorData += "Time: " + String(millis()) + " ms\n";
    ws.textAll(sensorData);
}

void loop() {};

#endif // RUN_WIFI_SENSORS_TEST

