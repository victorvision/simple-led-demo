#include "LumenProtocol.h"

#define LCM_BAUDRATE 115200  // Data transmission rate

// Configuration constants
const uint8_t LED_PIN = 8;              // LED pin
const uint8_t LED_ON = 1;               // Value to turn the LED on
const uint8_t LED_OFF = 0;              // Value to turn the LED off
const uint8_t LED_STATUS_ADDRESS = 20;  // Address for the LED packet
const uint16_t INIT_DELAY = 1000;       // Initial delay

// Defining the packet for LED control
lumen_packet_t led_status = { LED_STATUS_ADDRESS, kS32 };  // Packet with address 20 and int32 data type
lumen_packet_t *current_packet;                            // Pointer to the current packet

// Functions for sending and receiving data via Lumen protocol
extern "C" void lumen_write_bytes(uint8_t *data, uint32_t length) {
  Serial.write(data, length);
}

extern "C" uint16_t lumen_get_byte() {
  if (Serial.available()) {
    return Serial.read();
  }
  return DATA_NULL;
}

// System initialization
void setup() {
  delay(INIT_DELAY);           // Initial delay
  Serial.begin(LCM_BAUDRATE);  // Initialize serial communication
  pinMode(LED_PIN, OUTPUT);    // Set the LED pin as output
}

// Helper function for controlling the LED
void controlLed(int state) {
  if (state == LED_ON) {
    digitalWrite(LED_PIN, HIGH);  // Turn the LED on
  } else if (state == LED_OFF) {
    digitalWrite(LED_PIN, LOW);  // Turn the LED off
  }
}

// Function to process the received packet
void processPacket(lumen_packet_t *packet) {
  if (packet->address == LED_STATUS_ADDRESS) {  // Check if the address is for the LED
    int led_state = packet->data._s32;          // Get the LED state
    controlLed(led_state);                      // Control the LED according to the packet
  }
}

// Main loop
void loop() {
  // Check if there are any available packets
  while (lumen_available() > 0) {
    current_packet = lumen_get_first_packet();  // Get the first available packet
    processPacket(current_packet);              // Process the packet
  }
}
