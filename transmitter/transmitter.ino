/*
  Code for transmitter using ESP-NOW
  
  Made by: NIHAL T P
  GitHub: https://github.com/nihaltp
  LinkedIn: https://www.linkedin.com/in/nihal-tp/
*/

// Define ESP Boards
#define ESP8266 0
#define ESP32 1

#define BOARD ESP8266              // TODO: Change to: ESP32
#define SERIAL_PORT true           // TODO: Change to false

#if BOARD == ESP32
  // Board Manager URL: https://dl.espressif.com/dl/package_esp32_index.json
  #include <WiFi.h> // ESP32 Library
  #include <esp_now.h> // ESPNOW Library for ESP32
#elif BOARD == ESP8266
  // Board Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
  #include <ESP8266WiFi.h> // ESP8266 Library
  #include <espnow.h> // ESPNOW Library for ESP8266
#else
  #error "Unsupported BOARD value."
#endif

// Mac Address of the Receiver
// Serial.println(WiFi.getMacAddress());
#warning "Verify the MAC address in the code before running the code."

uint8_t receiverMACs[][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x11, 0x11, 0x11, 0x11, 0x11, 0x11}
}; // TODO: Replace Mac Addresses

// Define Pins For Control Buttons
#warning "Verify pin assignments for buttons before running the code."
#if BOARD == ESP32
  const int BTN1 = 12; // Forward Button   // TODO: Change
  const int BTN2 = 13; // Backward Button  // TODO: Change
  const int BTN3 = 14; // Left Button      // TODO: Change
  const int BTN4 = 15; // Right Button     // TODO: Change
#elif BOARD == ESP8266
  const int BTN1 = D1; // Forward Button   // TODO: Change
  const int BTN2 = D2; // Backward Button  // TODO: Change
  const int BTN3 = D3; // Left Button      // TODO: Change
  const int BTN4 = D4; // Right Button     // TODO: Change
#endif

// x is the content to be printed
#define debugPrint(x)   if (SERIAL_PORT) Serial.print(x)
#define debugPrintln(x) if (SERIAL_PORT) Serial.println(x)

typedef struct packetData {
  uint8_t btnValues;
} packetData;

packetData controls;

#if BOARD == ESP32
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus);
#elif BOARD == ESP8266
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus);
#endif

// MARK: setup
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  while (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
  } Serial.println("ESP-NOW Initialized Successfully");
  
  // Register the send callback
  #if BOARD == ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER); // Set as Controller for ESP8266
  #endif
  
  esp_now_register_send_cb(onDataSent);
  for (auto &mac : receiverMACs) {
  #if BOARD == ESP32
    if (!esp_now_is_peer_exist(mac)) {
      if (esp_now_add_peer(mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != ESP_OK) {
        Serial.println("Failed to add peer!");
      }
    }
  #elif BOARD == ESP8266
    esp_now_add_peer(mac, ESP_NOW_ROLE_SLAVE, 0, NULL, 0);
  #endif
  }
  
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
}

// MARK: loop
void loop() {
  bitWrite(controls.btnValues, 0, digitalRead(BTN1));
  bitWrite(controls.btnValues, 1, digitalRead(BTN2));
  bitWrite(controls.btnValues, 2, digitalRead(BTN3));
  bitWrite(controls.btnValues, 3, digitalRead(BTN4));
  
  debugPrint  ("Buttons Values:\t");
  debugPrint  (bitRead(controls.btnValues, 0)); debugPrint("\t");
  debugPrint  (bitRead(controls.btnValues, 1)); debugPrint("\t");
  debugPrint  (bitRead(controls.btnValues, 2)); debugPrint("\t");
  debugPrintln(bitRead(controls.btnValues, 3));
  
  // Send the control message
  for (auto &mac : receiverMACs) {
    esp_now_send(mac, (uint8_t *) &controls, sizeof(controls));
    delay(10); // Small delay to avoid flooding the network
  }
}

// Callback for data sent
// MARK: callback
#if BOARD == ESP32
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
#elif BOARD == ESP8266
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  debugPrint("Send Status:\t");
  debugPrintln(sendStatus == 0 ? "Success" : "Fail");
}
#endif
