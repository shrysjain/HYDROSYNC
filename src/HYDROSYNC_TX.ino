/*
HYDROSYNC (Transmitter)
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
*/

// Install dependencies
#include <SPI.h>
#include <RH_RF95.h>

// Define pin configurations
#define RFM95_CS   8
#define RFM95_INT  3
#define RFM95_RST  4

// Define sensor pin
#define SENSOR     13

// Set radio frequency
#define RF95_FREQ  915.0

// Singleton instance of radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // Sensor pin
  pinMode(SENSOR, INPUT);
  
  // Reset pin
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Open serial monitor
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Initializing LoRa radio transmitter!");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Radio initilization
  while (!rf95.init()) {
    Serial.println("LoRa radio initialization failed");
    // Uncomment `#define SERIAL_DEBUG` in `RH_RFM95.cpp` for detailed debug information
    while (1);
  }

  Serial.println("LoRa radio initialized OK");

  // Defaults
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Frequency initialized incorrectly");
    while (1);
  }

  Serial.print("Set frequency to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
}

// Packet counter
int16_t packetnum = 0;

void loop() {
  // Retrieve sensor value
  int sensorValue = digitalRead(SENSOR);

  // Packet tranmission
  delay(1000);
  Serial.println("Transmitting....");

  // Define radio packet
  char radiopacket[25];
  snprintf(radiopacket, sizeof(radiopacket), "%i", sensorValue);
  Serial.print("Sending "); Serial.println(radiopacket);

  // Transmit radio packet
  Serial.println("Sending....");
  delay(10);
  rf95.send((uint8_t *)radiopacket, strlen(radiopacket));

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();

  // Await reply from receiver
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
}
