/*
  Code for receiver using ESP-NOW
  Made by: NIHAL T P
  GitHub: nihaltp
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

// Define Motor Drivers
#define DRIVER_L298N 0
#define DRIVER_BTS 1
#define MOTOR_DRIVER DRIVER_L298N // Set to DRIVER_BTS or DRIVER_L298N // TODO: Change

const int L1  = D0; // Left  1
const int L2  = D1; // Left  2
const int R1  = D3; // Right 1
const int R2  = D4; // Right 2

#if MOTOR_DRIVER == DRIVER_BTS
constexpr int ENL = D2; // Left Enable (for BTS)
constexpr int ENR = D5; // Right Enable (for BTS)
#endif

const int MAX_SPEED = 255;
const int HALF_SPEED = 127;
const int MIN_SPEED = 0;

#define SERIAL_PORT true  // TODO: Change

typedef struct packetData {
  uint8_t btnValue1;
  uint8_t btnValue2;
  uint8_t btnValue3;
  uint8_t btnValue4;
} packetData;

packetData controls;

void onReceive(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
  memcpy(&controls, incomingData, sizeof(controls));
  #if SERIAL_PORT
    Serial.println("Received commands:");
    Serial.print(controls.btnValue1);   Serial.print("\t");
    Serial.print(controls.btnValue2);   Serial.print("\t");
    Serial.print(controls.btnValue3);   Serial.print("\t");
    Serial.println(controls.btnValue4);
  #endif
  simpleMovements();
}

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
  
  #if MOTOR_DRIVER == DRIVER_BTS
    pinMode(ENL, OUTPUT);
    pinMode(ENR, OUTPUT);
  #endif
}

void loop() {}

/**
 * Determines the direction of motor rotation based on control button states.
 * The function checks the states of four control buttons and sets the motor
 * rotation to move in different directions
 * It uses the rotateMotor function to set motor speeds accordingly.
 * If SERIAL_PORT is enabled, it prints the current direction to the Serial monitor.
 * since the buttons are set to INPUT_PULLUP, LOW is pressed and HIGH is released
 */
void simpleMovements() {
  // Check the state of the control buttons and set motor rotation accordingly
  if (!controls.btnValue1 && controls.btnValue2 && controls.btnValue3 && controls.btnValue4) {
    rotateMotor(MAX_SPEED,MAX_SPEED);    // FORWARD
  } else if (!controls.btnValue1 && !controls.btnValue3 && controls.btnValue2 && controls.btnValue4) {
    rotateMotor(HALF_SPEED,MAX_SPEED);    // FORWARD LEFT
  } else if (!controls.btnValue1 && !controls.btnValue4 && controls.btnValue2 && controls.btnValue3) {
    rotateMotor(MAX_SPEED,HALF_SPEED);    // FORWARD RIGHT
  } else if (!controls.btnValue2 && controls.btnValue1 && controls.btnValue3 && controls.btnValue4) {
    rotateMotor(-MAX_SPEED,-MAX_SPEED);  // BACKWARD
  } else if (!controls.btnValue2 && !controls.btnValue3 && controls.btnValue1 && controls.btnValue4) {
    rotateMotor(-HALF_SPEED,-MAX_SPEED);  // BACKWARD LEFT
  } else if (!controls.btnValue2 && !controls.btnValue4 && controls.btnValue1 && controls.btnValue3) {
    rotateMotor(-MAX_SPEED,-HALF_SPEED);  // BACKWARD RIGHT
  } else if (!controls.btnValue3 && controls.btnValue1 && controls.btnValue2 && controls.btnValue4) {
    rotateMotor(MIN_SPEED,MAX_SPEED);      // LEFT
  } else if (!controls.btnValue4 && controls.btnValue1 && controls.btnValue2 && controls.btnValue3){
    rotateMotor(MAX_SPEED,MIN_SPEED);      // RIGHT
  } else {
    rotateMotor(MIN_SPEED,MIN_SPEED);        // STOP
  }
}


/**
 * Rotates the motor based on the specified driver type and input speed values.
 * 
 * @param x The speed value for the left motor.
 * @param y The speed value for the right motor.
 */

void rotateMotor(int x, int y) {
  #if MOTOR_DRIVER == DRIVER_BTS
    BTS_movements(x, y);
  #elif MOTOR_DRIVER == DRIVER_L298N
    L298N_movements(x, y);
  #else
    #error "Unsupported MOTOR_DRIVER value."
  #endif
  
  #if SERIAL_PORT
    direction(x, y);
  #endif
}

/**
 * Sets the motor rotation speed based on the given values.
 * @param x speed for the left motor, positive for forward and negative for backward
 * @param y speed for the right motor, positive for forward and negative for backward
 */

#if MOTOR_DRIVER == DRIVER_BTS
void BTS_movements(int x, int y) {
  int leftSpeed = (x > 0) ? x : -x;
  int rightSpeed = (y > 0) ? y : -y;
  digitalWrite(L1, (x > 0) ? HIGH : LOW);
  digitalWrite(L2, (x < 0) ? HIGH : LOW);
  analogWrite(ENL, leftSpeed);
  digitalWrite(R1, (y > 0) ? HIGH : LOW);
  digitalWrite(R2, (y < 0) ? HIGH : LOW);
  analogWrite(ENR, rightSpeed);
}
#endif

#if MOTOR_DRIVER == DRIVER_L298N
void L298N_movements(int x, int y) {
  analogWrite(L1, max(0, x));
  analogWrite(L2, max(0, -x));
  analogWrite(R1, max(0, y));
  analogWrite(R2, max(0, -y));
}
#endif

#if SERIAL_PORT
void direction(int x, int y) {
  if (x == MIN_SPEED && y == MIN_SPEED) {
    Serial.println("STOP");
    return;
  }
  
  if (x == MAX_SPEED) {
    if (y == MAX_SPEED) {
      Serial.println("FORWARD");
    } else if (y == HALF_SPEED) {
      Serial.println("FORWARD RIGHT");
    } else if (y == MIN_SPEED) {
      Serial.println("RIGHT");
    }
    return;
  }
  
  if (y == MAX_SPEED) {
    if (x == HALF_SPEED) {
      Serial.println("FORWARD LEFT");
    } else {
      Serial.println("LEFT");
    }
    return;
  }
  
  if (x == -MAX_SPEED) {
    if (y == -MAX_SPEED) {
      Serial.println("BACKWARD");
    } else if (y == -HALF_SPEED) {
      Serial.println("BACKWARD RIGHT");
    }
    return;
  }
  
  if (x == -HALF_SPEED && y == -MAX_SPEED) {
    Serial.println("BACKWARD LEFT");
  }
}
#endif
