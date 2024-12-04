/*
  Code for transmitter using ESP-NOW
  Made by: NIHAL T P
  GitHub: nihaltp
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

// Mac Address of the Receiver
// Serial.println(WiFi.getMacAddress());
uint8_t receiverMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // TODO: Replace Mac Address

// Define Pins For Control Buttons
const int BTN1 = D1; // Forward Button
const int BTN2 = D2; // Backward Button
const int BTN3 = D3; // Left Button
const int BTN4 = D4; // Right Button

#define SERIAL_PORT true  // TODO: Change

typedef struct packetData {
  uint8_t btnValue1;
  uint8_t btnValue2;
  uint8_t btnValue3;
  uint8_t btnValue4;
} packetData;

packetData controls;

void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  #if SERIAL_PORT
    Serial.print("Send Status:\t");
    Serial.println(sendStatus == 0 ? "Success" : "Fail");
  #endif
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  while (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
  } Serial.println("ESP-NOW Initialized Successfully");

  // Register the send callback
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(onDataSent);
  esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
}

void loop() {
  controls.btnValue1 = digitalRead(BTN1);
  controls.btnValue2 = digitalRead(BTN2);
  controls.btnValue3 = digitalRead(BTN3);
  controls.btnValue4 = digitalRead(BTN4);

  #if SERIAL_PORT
    Serial.print("Button Values:\t");
    Serial.print(controls.btnValue1);   Serial.print("\t");
    Serial.print(controls.btnValue2);   Serial.print("\t");
    Serial.print(controls.btnValue3);   Serial.print("\t");
    Serial.println(controls.btnValue4);
  #endif

  // Send the control message
  esp_now_send(receiverMAC, (uint8_t *) &controls, sizeof(controls));
}
