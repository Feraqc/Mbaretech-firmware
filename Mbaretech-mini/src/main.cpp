#include "globals.h"  // Include the header file

// Define the shared variables
volatile bool sensorReadings[7];
volatile bool startSignal = false; // Creo que debe ser volatile si le trato con interrupt
bool dipSwitchPin[4];  // de A a D

Motor leftMotor(PWM_A,CHANNEL_LEFT);
Motor rightMotor(PWM_B,CHANNEL_RIGHT);

volatile State currentState = WAIT_ON_START;

QueueHandle_t imuDataQueue;
QueueHandle_t cmdQueue;

TaskHandle_t imuTaskHandle;

// Define task handles for motor control tasks
TaskHandle_t leftTurnTaskHandle = NULL;
TaskHandle_t rightTurnTaskHandle = NULL;
TaskHandle_t forwardTaskHandle = NULL;

// Define a mutex for motor control
SemaphoreHandle_t motorControlMutex;

// Function prototypes
void leftTurnTask(void *pvParameters);
void rightTurnTask(void *pvParameters);
void forwardMovement(uint32_t speed);

// Notification value to stop tasks early
volatile bool stopTask = false;

//IMU imu;

int desiredAngle;

void IR1_ISR() { sensorReadings[0] = digitalRead(IR1); }  // 0 esta sensor izquierdo le llamo short right
void IR2_ISR() { sensorReadings[1] = digitalRead(IR2); }  // 1 medio no problem
void IR3_ISR() { sensorReadings[2] = digitalRead(IR3); }  // 2 esta sensor derecho le llamo short left

#ifdef RUN_WIFI_SENSORS_TEST
// Create AsyncWebServer instance on port 80
AsyncWebServer server(80);

// Create a WebSocket instance on port 81
AsyncWebSocket ws("/ws");

// Replace with your network credentials
const char* ssid = "Pixel";     // Replace with your WiFi network name
const char* password = "guana123"; // Replace with your WiFi password
#endif   // RUN_WIFI_SENSORS_TEST

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);

    //imu.begin();
    // IR sensors
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR1), IR1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR2), IR2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR3), IR3_ISR, CHANGE);

    // Start pins
    pinMode(START_PIN, INPUT);
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    // Motors
    rightMotor.begin();
    leftMotor.begin();

    // Create the motor control mutex
    motorControlMutex = xSemaphoreCreateMutex();

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
    #ifndef RUN_WIFI_SENSORS_TEST
    xTaskCreate(imuTask, "imuTask", 4096, NULL, 1, &imuTaskHandle);
    xTaskCreate(mainTask, "MainTask", 2048, NULL, 1, NULL);

    // Create the motor control tasks (tasks are created but initially suspended)
    xTaskCreate(leftTurnTask, "Left Turn Task", 1024, NULL, 2, &leftTurnTaskHandle);
    xTaskCreate(rightTurnTask, "Right Turn Task", 1024, NULL, 2, &rightTurnTaskHandle);
    #endif // RUN_WIFI_SENSORS_TEST
    #endif // RUN_DRIVER_TEST
    #endif // RUN_IR_SENSOR_TEST
    #endif // RUN_GYRO_TEST

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
#ifndef RUN_DRIVER_TEST
#ifndef RUN_WIFI_SENSORS_TEST

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
                    sensorReadings[0] = digitalRead(IR1);
                    sensorReadings[1] = digitalRead(IR2);
                    sensorReadings[2] = digitalRead(IR3);
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
                changeState(TOP_SENSORS_CHECK); // En realidad no hay top pero si no lee el medio se usa como top
            }
            break;

        case MID_MOVE:
            Serial.println("State: MID_MOVE");
            startForward(20);
            // Hacer el if else if 
            // TODO: Do movement
            // IF FRONTLEFT AND FRONTRIGHT --> 100%
            // else if FRONTLEFT --> shortLeft
            // else if FRONTRIGHT --> shortRight
            // else ---> 30% o algo asi
            changeState(MID_SENSOR_CHECK);
            break;

        case TOP_SENSORS_CHECK:
            Serial.println("State: TOP_SENSOR_CHECK");
            if (sensorReadings[SHORT_LEFT]) {
                changeState(TOP_LEFT_MOVE);
            }
            else if (sensorReadings[SHORT_RIGHT]) {
                changeState(TOP_RIGHT_MOVE);
            }
            else {
                changeState(SEARCH);
            }
            break;

        case TOP_LEFT_MOVE:
            Serial.println("State: TOP_LEFT_MOVE");
            startLeftTurn(TICKS_FOR_45_DEGREES); // Example: 45-degree left turn
            changeState(MID_SENSOR_CHECK);
            break;

        case TOP_RIGHT_MOVE:
            Serial.println("State: TOP_RIGHT_MOVE");
            startRightTurn(TICKS_FOR_45_DEGREES); // Example: 45-degree right turn
            changeState(MID_SENSOR_CHECK);
            break;

        case SEARCH:
            Serial.println("State: DEFAULT_ACTION_STATE");
            // TODO: Aca es un dependiendo del switch por ejemplo
            // para saber que queremos que haga por default
            // puede ser turco, ir para delante, quedarse quieto
            changeState(MID_SENSOR_CHECK);  // mientras le pongo que mire sensores nomas

        default:
            Serial.println("State: UNKNOWN");
            break;
    }
}

void changeState(State newState) { currentState = newState; }

void imuTask(void *param) {
    float currentAngle;

    while (true) {
        /*
        // #ifdef IMU_TEST
        imu.getData();
        currentAngle = imu.currentAngle;

        xQueueSend(imuDataQueue, &currentAngle, portMAX_DELAY);

        vTaskDelay(10 / portTICK_PERIOD_MS);
        */
    }
}

void loop() {} //Empty loop since I am using freertos

#endif // RUN_WIFI_SENSORS_TEST
#endif // RUN_DRIVER_TEST
#endif // RUN_IR_SENSOR_TEST
#endif // RUN_GYRO_TEST

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


// Left turn task
void leftTurnTask(void *pvParameters) {
    uint32_t ticks = *((uint32_t *)pvParameters); // Extract the ticks parameter

    while (true) {
        // Suspend this task until it's triggered
        vTaskSuspend(NULL);

        // Take the mutex before starting the turn
        if (xSemaphoreTake(motorControlMutex, portMAX_DELAY) == pdTRUE) {
            // Start the in-place left rotation
            leftMotor.reverse(50);  // Adjust speed percentage as needed
            rightMotor.forward(50); // Adjust speed percentage as needed

            // Run for the calculated number of ticks
            for (uint32_t i = 0; i < ticks; i++) {
                if (stopTask) break;
                vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL_MS));
            }

            // Stop the motors
            leftMotor.brake();
            rightMotor.brake();

            // Release the mutex
            xSemaphoreGive(motorControlMutex);
        }
    }
}

// Right turn task
void rightTurnTask(void *pvParameters) {
    uint32_t ticks = *((uint32_t *)pvParameters); // Extract the ticks parameter

    while (true) {
        // Suspend this task until it's triggered
        vTaskSuspend(NULL);

        // Take the mutex before starting the turn
        if (xSemaphoreTake(motorControlMutex, portMAX_DELAY) == pdTRUE) {
            // Start the in-place right rotation
            leftMotor.forward(50);  // Adjust speed percentage as needed
            rightMotor.reverse(50); // Adjust speed percentage as needed

            // Run for the calculated number of ticks
            for (uint32_t i = 0; i < ticks; i++) {
                if (stopTask) break;
                vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL_MS));
            }

            // Stop the motors
            leftMotor.brake();
            rightMotor.brake();

            // Release the mutex
            xSemaphoreGive(motorControlMutex);
        }
    }
}

// Forward movement function
void forwardMovement(uint32_t speed) {
    if (xSemaphoreTake(motorControlMutex, portMAX_DELAY) == pdTRUE) {
        // Start the forward movement
        leftMotor.forward(speed);  // Adjust speed percentage as needed
        rightMotor.forward(speed); // Adjust speed percentage as needed
        // No loop or duration; the motors will keep running until interrupted
    }
}

// Function to trigger left turn
void startLeftTurn(uint32_t ticks) {
    stopTask = true; // Set flag to stop current task
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL_MS)); // Give a little time for current task to stop
    if (leftTurnTaskHandle != NULL) {
        stopTask = false; // Reset flag
        xTaskNotify(leftTurnTaskHandle, ticks, eSetValueWithOverwrite); // Pass ticks as notification value
        vTaskResume(leftTurnTaskHandle);
    }
}

// Function to trigger right turn
void startRightTurn(uint32_t ticks) {
    stopTask = true; // Set flag to stop current task
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL_MS)); // Give a little time for current task to stop
    if (rightTurnTaskHandle != NULL) {
        stopTask = false; // Reset flag
        xTaskNotify(rightTurnTaskHandle, ticks, eSetValueWithOverwrite); // Pass ticks as notification value
        vTaskResume(rightTurnTaskHandle);
    }
}

// Function to trigger forward movement
void startForward(uint32_t speed) {
    stopTask = true; // Set flag to stop current task
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL_MS)); // Give a little time for current task to stop
    stopTask = false; // Reset flag
    forwardMovement(speed);
}










