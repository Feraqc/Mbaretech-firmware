#include "globals.h"  // Include the header file

// Define the shared variables
volatile bool irSensor[3];
volatile bool startSignal = false; // Creo que debe ser volatile si le trato con interrupt
bool dipSwitchPin[4];  // de A a D

Motor leftMotor(PWM_B,FORWARD_MAX_LEFT,FORWARD_MIN_LEFT,BACKWARD_MAX_LEFT,BACKWARD_MIN_LEFT);
Motor rightMotor(PWM_A,FORWARD_MAX_RIGHT,FORWARD_MIN_RIGHT,BACKWARD_MAX_RIGHT,BACKWARD_MIN_RIGHT);

//TASK HANDLERS
TaskHandle_t imuTaskHandle;
TaskHandle_t motorTaskHandle;

//TAKS
void motorTask(void *param);
void imuTask(void *param);


//MUTEXS
SemaphoreHandle_t gyroDataMutex;


//INTERRUPS
void IRAM_ATTR IR1_ISR() { irSensor[0] = !digitalRead(IR1); }  // 0 esta sensor izquierdo le llamo short right
void IRAM_ATTR IR2_ISR() { irSensor[1] = !digitalRead(IR2); }  // 1 medio no problem
void IRAM_ATTR IR3_ISR() { irSensor[2] = !digitalRead(IR3); }  // 2 esta sensor derecho le llamo short left

void IRAM_ATTR KS_ISR(){startSignal = digitalRead(START_PIN);};

// Create AsyncWebServer instance on port 80
AsyncWebServer server(80);

// Create a WebSocket instance on port 81
AsyncWebSocket ws("/ws");

// Replace with your network credentials
const char* ssid = "Pixel";     // Replace with your WiFi network name
const char* password = "guana123"; // Replace with your WiFi password

int currentAngle;
IMU imu;

void setup() {
    esp_efuse_write_field_cnt(ESP_EFUSE_VDD_SPI_FORCE, 1); 
    // Initialize Serial communication
    Serial.begin(115200);
    rightMotor.begin();
    leftMotor.begin();

    imu.begin();
    // IR sensors
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), KS_ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(IR1), IR1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR2), IR2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR3), IR3_ISR, CHANGE);

    // Start pins
    pinMode(START_PIN, INPUT);
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    gyroDataMutex = xSemaphoreCreateMutex();

    xTaskCreate(motorTask, "motorTask", 4096, NULL, 1, &motorTaskHandle);
   // xTaskCreate(imuTask, "imuTask", 4096, NULL, 1, &imuTaskHandle);


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


// void handleState() {
//     switch (currentState) {
//         case WAIT_ON_START:
//             Serial.println("State: WAIT_ON_START");
//             if (digitalRead(START_PIN)) {
//                 if (digitalRead(DIPA)) {  // Se puede cambiar a otro dip segun
//                                           // necesitemos para estrategia
//                     changeState(INITIAL_MOVEMENT);
//                 }
//                 else {
//                     sensorReadings[0] = digitalRead(IR1);
//                     sensorReadings[1] = digitalRead(IR2);
//                     sensorReadings[2] = digitalRead(IR3);
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
//                 changeState(TOP_SENSORS_CHECK); // En realidad no hay top pero si no lee el medio se usa como top
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
//             if (sensorReadings[SHORT_LEFT]) {
//                 changeState(TOP_LEFT_MOVE);
//             }
//             else if (sensorReadings[SHORT_RIGHT]) {
//                 changeState(TOP_RIGHT_MOVE);
//             }
//             else {
//                 changeState(SEARCH);
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

//         case SEARCH:
//             Serial.println("State: DEFAULT_ACTION_STATE");
//             // TODO: Aca es un dependiendo del switch por ejemplo
//             // para saber que queremos que haga por default
//             // puede ser turco, ir para delante, quedarse quieto
//             changeState(MID_SENSOR_CHECK);  // mientras le pongo que mire sensores nomas

//         default:
//             Serial.println("State: UNKNOWN");
//             break;
//     }
// }

// void changeState(State newState) { currentState = newState; }

void loop() {} //Empty loop since I am using freertos

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
    for (int i = 0; i < 3; i++) {
        sensorData += "IR" + String(i + 1) + ": " + String(sensorReadings[i]) + "\t";
    }
    sensorData += "\t";
    sensorData += "Time: " + String(millis()) + " ms\n";
    ws.textAll(sensorData);
}

void loop() {};

#endif // RUN_WIFI_SENSORS_TEST









