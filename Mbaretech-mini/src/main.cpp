#include "globals.h"  // Include the header file

// Define the shared variables
volatile bool sensorReadings[7];
volatile bool startSignal = false; // Creo que debe ser volatile si le trato con interrupt
bool dipSwitchPin[4];  // de A a D

volatile State currentState = WAIT_ON_START;

QueueHandle_t imuDataQueue;
QueueHandle_t cmdQueue;

TaskHandle_t imuTaskHandle;

IMU imu;

int desiredAngle;

void IR1_ISR() { sensorReadings[0] = digitalRead(IR1); }
void IR2_ISR() { sensorReadings[1] = digitalRead(IR2); }
void IR3_ISR() { sensorReadings[2] = digitalRead(IR3); }
void IR4_ISR() { sensorReadings[3] = digitalRead(IR4); }
void IR5_ISR() { sensorReadings[4] = digitalRead(IR5); }
void IR6_ISR() { sensorReadings[5] = digitalRead(IR6); }
void IR7_ISR() { sensorReadings[6] = digitalRead(IR7); }

WebServer server(80);  // Initialize the web server
WebSocketsServer webSocket = WebSocketsServer(81); // Initialize the WebSocket server on port 81

// Replace with your network credentials
const char* ssid = "Pixel";     // Replace with your WiFi network name
const char* password = "guana123"; // Replace with your WiFi password

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);

    //imu.begin();
    // IR sensors
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

    // Start pins
    pinMode(START_PIN, INPUT);
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    // Motors
    pinMode(MOTOR_LEFT, OUTPUT);
    pinMode(MOTOR_RIGHT, OUTPUT);

    // Configure LEDC PWM functionalities for both motors
    ledcSetup(leftMotorChannel, freq, resolution);
    ledcSetup(rightMotorChannel, freq, resolution);
    ledcAttachPin(MOTOR_LEFT, leftMotorChannel);
    ledcAttachPin(MOTOR_RIGHT, rightMotorChannel);

    // Start motors at 0
    ledcWrite(leftMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledcWrite(rightMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    // Ojo que tengo entendido que esto calienta el driver igual, osea no dejar prendido al pedo

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

    // Create tasks conditionally
    #ifndef RUN_GYRO_TEST
    #ifndef RUN_IR_SENSOR_TEST
    #ifndef RUN_DRIVER_TEST
    xTaskCreate(imuTask, "imuTask", 4096, NULL, 1, &imuTaskHandle);
    xTaskCreate(mainTask, "MainTask", 2048, NULL, 1, NULL);
    #endif  // RUN_IR_SENSOR_TEST
    #endif  // RUN_GYRO_TEST
    #endif  // RUN_DRIVER_TEST

    #ifdef RUN_WIFI_SENSORS_TEST
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Set up web server routes
    server.on("/", handleRoot);

    // Start the server
    server.begin();
    Serial.println("Web server started");

    // Initialize WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");

    xTaskCreate(webSocketTask, "WebSocketTask", 2048, NULL, 1, NULL);
    #endif
}

#ifndef RUN_GYRO_TEST
#ifndef RUN_IR_SENSOR_TEST
#ifndef RUN_DRIVER_TEST

void mainTask(void *pvParameters) {
    // Main loop of the FreeRTOS task
    while (true) {
        handleState();
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100 ms
    }
}

void handleState() {
    switch (currentState) {
            // TODO:
            // INITIAL_MOVEMENT
            // MID_MOVE,
            // TOP_LEFT_MOVE,
            // TOP_RIGHT_MOVE,
            // SIDE_LEFT_MOVE,
            // SIDE_RIGHT_MOVE,
            // BOUND_MOVE, ---> Todavia no escribi el caso hay que hablar como
            // manejar DEFAULT_ACTION_STATE
        case WAIT_ON_START:
            Serial.println("State: WAIT_ON_START");
            if (digitalRead(START_PIN)) {
                if (digitalRead(DIPA)) {  // Se puede cambiar a otro dip segun
                                          // necesitemos para estrategia
                    changeState(INITIAL_MOVEMENT);
                }
                else {
                    changeState(MID_SENSOR_CHECK);
                }
            }
            else {
                // TODO: Motor OFF
            }
            break;

        case INITIAL_MOVEMENT:
            Serial.println("State: INITIAL_MOVEMENT");
            // Ver como separar los movimientos iniciales
            // No necesita break

        case MID_SENSOR_CHECK:
            Serial.println("State: MID_SENSOR_CHECK");
            if (sensorReadings[TOP_MID]) {
                changeState(MID_MOVE);
            }
            else {
                changeState(TOP_SENSORS_CHECK);
            }
            break;

        case MID_MOVE:
            Serial.println("State: MID_MOVE");
            // TODO: Do movement
            // IF FRONTLEFT AND FRONTRIGHT --> 100%
            // else if FRONTLEFT --> shortLeft
            // else if FRONTRIGHT --> shortRight
            // else ---> 30% o algo asi
            changeState(MID_SENSOR_CHECK);
            break;

        case TOP_SENSORS_CHECK:
            Serial.println("State: TOP_SENSOR_CHECK");
            if (sensorReadings[TOP_LEFT]) {
                changeState(TOP_LEFT_MOVE);
            }
            else if (sensorReadings[TOP_RIGHT]) {
                changeState(TOP_RIGHT_MOVE);
            }
            else {
                changeState(SIDE_SENSORS_CHECK);
            }
            break;

        case TOP_LEFT_MOVE:
            Serial.println("State: TOP_LEFT_MOVE");
            // TODO: LeftTurn45
            changeState(MID_SENSOR_CHECK);
            break;

        case TOP_RIGHT_MOVE:
            Serial.println("State: TOP_RIGHT_MOVE");
            // TODO: RightTurn45
            changeState(MID_SENSOR_CHECK);
            break;

        case SIDE_SENSORS_CHECK:
            Serial.println("State: SIDE_SENSORS_CHECK");
            if (sensorReadings[SIDE_LEFT]) {
                changeState(SIDE_LEFT_MOVE);
            }
            else if (sensorReadings[SIDE_RIGHT]) {
                changeState(SIDE_RIGHT_MOVE);
            }
            else {
                changeState(DEFAULT_ACTION_STATE);
            }
            break;

        case SIDE_LEFT_MOVE:
            Serial.println("State: SIDE_LEFT_MOVE");
            // TODO: LEFTTURN90
            changeState(MID_SENSOR_CHECK);
            break;

        case SIDE_RIGHT_MOVE:
            Serial.println("State: SIDE_RIGHT_MOVE");
            // TODO: RIGHTTURN90
            changeState(MID_SENSOR_CHECK);
            break;

        case DEFAULT_ACTION_STATE:
            Serial.println("State: DEFAULT_ACTION_STATE");
            // TODO: Aca es un dependiendo del switch por ejemplo
            // para saber que queremos que haga por default
            // puede ser turco, ir para delante, quedarse quieto
            changeState(
                MID_SENSOR_CHECK);  // mientras le pongo que mire sensores nomas

        default:
            Serial.println("State: UNKNOWN");
            break;
    }
}

void changeState(State newState) { currentState = newState; }

void imuTask(void *param) {
    float currentAngle;

    while (true) {
        // #ifdef IMU_TEST
        imu.getData();
        currentAngle = imu.currentAngle;

        xQueueSend(imuDataQueue, &currentAngle, portMAX_DELAY);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void loop() {} //Empty loop since I am using freertos

#endif  // RUN_IR_SENSOR_TEST
#endif  // RUN_GYRO_TEST
#endif  // RUN_DRIVER_TEST

#ifdef RUN_WIFI_SENSORS_TEST

void webServerTask(void *pvParameters) {
    while (true) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// Function to handle the root endpoint
void handleRoot() {
    server.send(200, "text/html", "<html><head><script>"
        "var ws = new WebSocket('ws://' + location.hostname + ':81');"
        "ws.onmessage = function(event) {"
        "document.getElementById('sensorData').innerHTML = event.data;"
        "};"
        "</script></head><body>"
        "<h1>Sensor Data</h1>"
        "<div id='sensorData'>Waiting for data...</div>"
        "</body></html>");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("WebSocket %u disconnected\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("WebSocket %u connected from %s\n", num, ip.toString().c_str());
            break;
        }
        case WStype_TEXT:
            Serial.printf("WebSocket %u received text: %s\n", num, payload);
            // You can handle incoming messages here
            break;
        case WStype_BIN:
            Serial.printf("WebSocket %u received binary data\n", num);
            break;
        case WStype_PING:
            Serial.printf("WebSocket %u received ping\n", num);
            break;
        case WStype_PONG:
            Serial.printf("WebSocket %u received pong\n", num);
            break;
    }
}

void webSocketTask(void *pvParameters) {
    while (true) {
        webSocket.loop();
        sendSensorData();
        vTaskDelay(pdMS_TO_TICKS(500)); // Send data every second
    }
}

// Function to handle the sensors endpoint
void sendSensorData() {
    String sensorData = "";
    sensorData += "IR1: " + String(sensorReadings[0]) + "\t";
    sensorData += "IR2: " + String(sensorReadings[1]) + "\t";
    sensorData += "IR3: " + String(sensorReadings[2]) + "\t";
    sensorData += "IR4: " + String(sensorReadings[3]) + "\t";
    sensorData += "IR5: " + String(sensorReadings[4]) + "\t";
    sensorData += "IR6: " + String(sensorReadings[5]) + "\t";
    sensorData += "IR7: " + String(sensorReadings[6]) + "\n";

    webSocket.broadcastTXT(sensorData);
}

#endif // RUN_WIFI_SENSORS_TEST

uint32_t usToDutyCycle(int pulseWidth) {
    uint32_t dutyCycle = ((pulseWidth * pow(2, resolution)) / period);
    return dutyCycle;
}









