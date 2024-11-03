/*
  Code for receiver using ESP-NOW
  Made by: NIHAL T P
  GitHub: nihaltp
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

const int L1  = D0; // Left  1
const int L2  = D1; // Left  2
const int ENL = D2; // Left  Enable
const int R1  = D3; // Right 1
const int R2  = D4; // Right 2
const int ENR = D5; // Right Enable

boolean SERIAL_PORT = true;  // TODO: Change

typedef struct packetData {
  uint8_t btnValue1;
  uint8_t btnValue2;
  uint8_t btnValue3;
  uint8_t btnValue4;
} packetData;

packetData controls;

void onReceive(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
  memcpy(&controls, incomingData, sizeof(controls));
  if (SERIAL_PORT) {
    Serial.println("Received commands:");
    Serial.print(controls.btnValue1);   Serial.print("\t");
    Serial.print(controls.btnValue2);   Serial.print("\t");
    Serial.print(controls.btnValue3);   Serial.print("\t");
    Serial.println(controls.btnValue4);
  }
  simpleMovements();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  while (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
  } Serial.println("ESP-NOW Initialized Successfully");

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onReceive);

  pinMode(L1,  OUTPUT);
  pinMode(L2,  OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(R1,  OUTPUT);
  pinMode(R2,  OUTPUT);
  pinMode(ENR, OUTPUT);
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
  char direction;

  // Check the state of the control buttons and set motor rotation accordingly
  if (!controls.btnValue1 && controls.btnValue2 && controls.btnValue3 && controls.btnValue4) {
    rotateMotor(255,255);    // FORWARD
    direction = "FORWARD";
  } else if (!controls.btnValue1 && !controls.btnValue3 && controls.btnValue2 && controls.btnValue4) {
    rotateMotor(127,255);    // FORWARD LEFT
    direction = "FORWARD LEFT";
  } else if (!controls.btnValue1 && !controls.btnValue4 && controls.btnValue2 && controls.btnValue3) {
    rotateMotor(255,127);    // FORWARD RIGHT
    direction = "FORWARD RIGHT";
  } else if (!controls.btnValue2 && controls.btnValue1 && controls.btnValue3 && controls.btnValue4) {
    rotateMotor(-255,-255);  // BACKWARD
    direction = "BACKWARD";
  } else if (!controls.btnValue2 && !controls.btnValue3 && controls.btnValue1 && controls.btnValue4) {
    rotateMotor(-127,-255);  // BACKWARD LEFT
    direction = "BACKWARD LEFT";
  } else if (!controls.btnValue2 && !controls.btnValue4 && controls.btnValue1 && controls.btnValue3) {
    rotateMotor(-255,-127);  // BACKWARD RIGHT
    direction = "BACKWARD RIGHT";
  } else if (!controls.btnValue3 && controls.btnValue1 && controls.btnValue2 && controls.btnValue4) {
    rotateMotor(0,255);      // LEFT
    direction = "LEFT";
  } else if (!controls.btnValue4 && controls.btnValue1 && controls.btnValue2 && controls.btnValue3){
    rotateMotor(255,0);      // RIGHT
    direction = "RIGHT";
  } else {
    rotateMotor(0,0);        // STOP
    direction = "STOP";
  }
  if (SERIAL_PORT)
  {
    Serial.println(direction);
  }
}

void rotateMotor(int x, int y) {
  int leftSpeed = (x > 0) ? x : -x;
  int rightSpeed = (y > 0) ? y : -y;
  digitalWrite(L1, (x > 0) ? HIGH : LOW);
  digitalWrite(L2, (x < 0) ? HIGH : LOW);
  analogWrite(ENL, leftSpeed);
  digitalWrite(R1, (y > 0) ? HIGH : LOW);
  digitalWrite(R2, (y < 0) ? HIGH : LOW);
  analogWrite(ENR, rightSpeed);
}