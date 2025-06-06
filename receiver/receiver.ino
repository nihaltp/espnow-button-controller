/*
  Code for receiver using ESP-NOW
  
  Made by: NIHAL T P
  GitHub: https://github.com/nihaltp
  LinkedIn: https://www.linkedin.com/in/nihal-tp/
*/

// Define ESP Boards
#define ESP8266 0
#define ESP32 1

// Define Motor Drivers
#define DRIVER_L298N 0
#define DRIVER_BTS 1

#define BOARD ESP8266              // TODO: Change to: ESP32
#define MOTOR_DRIVER DRIVER_L298N  // TODO: Change to: DRIVER_BTS
#define SERIAL_PORT true           // TODO: Change to: false

#if BOARD == ESP32
  // Board Manager URL: https://dl.espressif.com/dl/package_esp32_index.json
  #include <WiFi.h>        // ESP32 Library
  #include <esp_now.h>     // ESPNOW Library for ESP32
#elif BOARD == ESP8266
  // Board Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
  #include <ESP8266WiFi.h> // ESP8266 Library
  #include <espnow.h>      // ESPNOW Library for ESP8266
#else
  #error "Unsupported BOARD value."
#endif

#warning "Verify pin assignments for motors before running the code."
#if BOARD == ESP32
  #if MOTOR_DRIVER == DRIVER_L298N
    const int L1  = 4;  // Left  1 // TODO: Change Pin
    const int L2  = 5;  // Left  2 // TODO: Change Pin
    const int R1  = 18; // Right 1 // TODO: Change Pin
    const int R2  = 19; // Right 2 // TODO: Change Pin
  #elif MOTOR_DRIVER == DRIVER_BTS
    // These pins should be able to handle 0-255 PWM
    const int L1  = 18; // Left  1 // TODO: Change Pin
    const int L2  = 19; // Left  2 // TODO: Change Pin
    const int R1  = 20; // Right 1 // TODO: Change Pin
    const int R2  = 21; // Right 2 // TODO: Change Pin
  #endif
#elif BOARD == ESP8266
  // These pins should be able to handle 0-255 PWM
  const int L1  = D0; // Left  1 // TODO: Change Pin
  const int L2  = D1; // Left  2 // TODO: Change Pin
  const int R1  = D3; // Right 1 // TODO: Change Pin
  const int R2  = D4; // Right 2 // TODO: Change Pin
#endif

#if MOTOR_DRIVER == DRIVER_L298N
  #if BOARD == ESP32
    // These pins should be able to handle 0-255 PWM
    const int ENL = 22; // Left Enable  // TODO: Change Pin
    const int ENR = 23; // Right Enable // TODO: Change Pin
  #elif BOARD == ESP8266
    const int ENL = D2; // Left Enable  // TODO: Change Pin
    const int ENR = D5; // Right Enable // TODO: Change Pin
  #endif
#endif

const int MAX_SPEED = 255;
const int HALF_SPEED = 127;
const int MIN_SPEED = 0;

#define debugPrint(x)  if (SERIAL_PORT) Serial.print(x)
#define debugPrintln(x) if (SERIAL_PORT) Serial.println(x)

typedef struct packetData {
  uint8_t btnValues;
} packetData;

packetData controls;

// MARK: setup
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  // Initialize ESP-NOW
  int retryCount = 5;
  while (esp_now_init() != 0 && retryCount-- > 0) {
    Serial.println("Error initializing ESP-NOW. Retrying...");
    delay(1000);
  }
  if (retryCount <= 0) {
    Serial.println("ESP-NOW Initialization Failed! Restarting...");
    ESP.restart();
  } else {
    Serial.println("ESP-NOW Initialized Successfully");
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onReceive);
  
  pinMode(L1,  OUTPUT);
  pinMode(L2,  OUTPUT);
  pinMode(R1,  OUTPUT);
  pinMode(R2,  OUTPUT);
  
  #if MOTOR_DRIVER == DRIVER_L298N
    pinMode(ENL, OUTPUT);
    pinMode(ENR, OUTPUT);
  #endif
  
  #if BOARD == ESP32
    const int pwmFrequency = 5000;
    const int pwmResolution = 8;
    
    const int pwmChannels[] = {pwmChannelL1, pwmChannelL2, pwmChannelR1, pwmChannelR2};
    const int pwmPins[] = {0, 1, 2, 3};
    
    for (int i = 0; i < 4; i++) {
      ledcSetup(pwmChannels[i], pwmFrequency, pwmResolution);
      ledcAttachPin(pwmPins[i], pwmChannels[i]);
    }
    
    #if MOTOR_DRIVER == DRIVER_L298N
      const int pwmENChannels[] = {pwmChannelENL, pwmChannelENR};
      const int pwmENPins[] = {4, 5};
      
      for (int i = 0; i < 2; i++) {
        ledcSetup(pwmENChannels[i], pwmFrequency, pwmResolution);
        ledcAttachPin(pwmENPins[i], pwmENChannels[i]);
      }
    #endif
  #endif
}

void loop() {}

// MARK: onRecieve
void onReceive(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
  memcpy(&controls, incomingData, sizeof(controls));
  
  debugPrint  ("Received commands:\t");
  debugPrint  (bitRead(controls.btnValues, 0)); debugPrint("\t");
  debugPrint  (bitRead(controls.btnValues, 1)); debugPrint("\t");
  debugPrint  (bitRead(controls.btnValues, 2)); debugPrint("\t");
  debugPrintln(bitRead(controls.btnValues, 3));
  
  simpleMovements();
}

/**
 * Determines the direction of motor rotation based on control button states.
 * The function checks the states of four control buttons and sets the motor
 * rotation to move in different directions
 * It uses the rotateMotor function to set motor speeds accordingly.
 * If SERIAL_PORT is enabled, it prints the current direction to the Serial monitor.
 * since the buttons are set to INPUT_PULLUP, LOW is pressed and HIGH is released
 */

// MARK: simpleMovements
void simpleMovements() {
  // Extract the button states from the controls structure
  bool forward  = bitRead(controls.btnValues, 0);
  bool backward = bitRead(controls.btnValues, 1);
  bool left     = bitRead(controls.btnValues, 2);
  bool right    = bitRead(controls.btnValues, 3);
  
  // Determine the motor rotation based on button states
  if (!forward && backward) {
    if (!left && right) {
      rotateMotor(HALF_SPEED,MAX_SPEED);    // FORWARD LEFT
      debugPrintln("FORWARD LEFT");
    } else if (!right && left) {
      rotateMotor(MAX_SPEED,HALF_SPEED);    // FORWARD RIGHT
      debugPrintln("FORWARD RIGHT");
    } else {
      rotateMotor(MAX_SPEED,MAX_SPEED);     // FORWARD
      debugPrintln("FORWARD");
    }
  } else if (!backward && forward) {
    if (!left && right) {
      rotateMotor(-HALF_SPEED,-MAX_SPEED);  // BACKWARD LEFT
      debugPrintln("BACKWARD LEFT");
    } else if (!right && left) {
      rotateMotor(-MAX_SPEED,-HALF_SPEED);  // BACKWARD RIGHT
      debugPrintln("BACKWARD RIGHT");
    } else {
      rotateMotor(-MAX_SPEED,-MAX_SPEED);   // BACKWARD
      debugPrintln("BACKWARD");
    }
  } else if (!left && right) {
    rotateMotor(MIN_SPEED,-MAX_SPEED);      // LEFT
    debugPrintln("LEFT");
  } else if (!right && left) {
    rotateMotor(-MAX_SPEED,MIN_SPEED);      // RIGHT
    debugPrintln("RIGHT");
  } else {
    rotateMotor(MIN_SPEED,MIN_SPEED);       // STOP
    debugPrintln("STOP");
  }
}


/**
 * Rotates the motor based on the specified driver type and input speed values.
 * 
 * @param x The speed value for the left motor.
 * @param y The speed value for the right motor.
 */

// MARK: rotateMotor
void rotateMotor(int x, int y) {
  #if MOTOR_DRIVER == DRIVER_BTS
    BTS_movements(x, y);
  #elif MOTOR_DRIVER == DRIVER_L298N
    L298N_movements(x, y);
  #else
    #error "Unsupported MOTOR_DRIVER value."
  #endif
}

/**
 * Sets the motor rotation speed based on the given values.
 * @param x speed for the left motor, positive for forward and negative for backward
 * @param y speed for the right motor, positive for forward and negative for backward
 */

// MARK: movements
#if MOTOR_DRIVER == DRIVER_BTS
void BTS_movements(int x, int y) {
  #if BOARD == ESP8266
    analogWrite(L1, max(0, x));
    analogWrite(L2, max(0, -x));
    analogWrite(R1, max(0, y));
    analogWrite(R2, max(0, -y));
  #elif BOARD == ESP32
    ledcWrite(pwmChannelL1, max(0, x));
    ledcWrite(pwmChannelL2, max(0, -x));
    ledcWrite(pwmChannelR1, max(0, y));
    ledcWrite(pwmChannelR2, max(0, -y));
  #endif
}
#elif MOTOR_DRIVER == DRIVER_L298N
void L298N_movements(int x, int y) {
  digitalWrite(L1, (x > 0) ? HIGH : LOW);
  digitalWrite(L2, (x < 0) ? HIGH : LOW);
  digitalWrite(R1, (y > 0) ? HIGH : LOW);
  digitalWrite(R2, (y < 0) ? HIGH : LOW);
  
  #if BOARD == ESP8266
    analogWrite(ENL, abs(x));
    analogWrite(ENR, abs(y));
  #elif BOARD == ESP32
    ledcWrite(pwmChannelENL, abs(x));
    ledcWrite(pwmChannelENR, abs(y));
  #endif
}
#endif
