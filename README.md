# HYDROSYNC: A Smarter Water Bottle ![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

<img src="https://i.ibb.co/sWhRB0Y/image.png" align="right" width="280" height="280">

HYDROSYNC is a 2-part product to help solve the issue of inadequate water consumption. The device measures the amount of water present in a water bottle and notifies users when the bottle requires refilling, while tracking their water intake. It also sends mobile notifications to users about their water consumption, as well as reminders of when to drink water.

HYDROSYNC was created by Bera Gumruk, Shreyas Jain, and Arjun Kode.

<br>
<br>

## Replication

Here are step-by-step instructions on how to replicate the device.

### System Requirements
- **Supported operating systems:** Windows, macOS, Linux
- USB drivers
- 5 GB available RAM and 600 MB available disk space
- Intel pentium 4+ (or equivalent)
- A stable internet connection

### Software Installations
- [Arduino IDE](https://www.arduino.cc/en/software) (Latest)
- [Python](https://www.python.org/downloads/) (3.8+)
- [Anaconda Distribution](https://docs.anaconda.com/free/anaconda/install/) ([Miniconda](https://docs.anaconda.com/free/miniconda/) is untested but may work)

### Library and Board Installation
1. Install the following libraries in the Arduino IDE:
- [RadioHead](https://www.arduino.cc/reference/en/libraries/radiohead/)

2. In the Arduino IDE, go to `Preferences/Settings` and under "Additional Board Manager URLs", and paste in this link: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`

### Arduino Code

1. In the Arduino IDE create a new sketch and paste in the following code:
<details><summary>HYDROSYNC_TX.ino</summary>
<br>

```ino
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
```
</details>

2. On another board, run the following sketch:
<details><summary>HYDROSYNC_RX.ino</summary>
<br>

```ino
/* 
HYDROSYNC (Receiver)
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
*/

// Install dependencies
#include <SPI.h>
#include <RH_RF95.h>

// Define pin configuration
#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   7
  #define RFM95_RST   4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
  #define RFM95_CS   16
  #define RFM95_INT  21
  #define RFM95_RST  17

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM95_CS    4  //
  #define RFM95_INT   3  //
  #define RFM95_RST   2  // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
  #define RFM95_CS    2  // "E"
  #define RFM95_INT  15  // "B"
  #define RFM95_RST  16  // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_CS   10  // "B"
  #define RFM95_INT   9  // "A"
  #define RFM95_RST  11  // "C"

#elif defined(ESP32)  // ESP32 feather w/wing
  #define RFM95_CS   33  // "B"
  #define RFM95_INT  27  // "A"
  #define RFM95_RST  13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
  #define RFM95_CS   11  // "B"
  #define RFM95_INT  31  // "C"
  #define RFM95_RST   7  // "A"

#endif

// Set radio frequency (915mHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // I/O pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Open serial monitor
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Radio initialization
  while (!rf95.init()) {
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }

  // Set transmitter power to 23 dBm
  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      buf[len] = '\0'; // Null-terminate the data
      int sensorValue;

      // Decode packet to plaintext
      sscanf((char*)buf, "%i", &sensorValue);

      // Print packet
      Serial.println(String(sensorValue));

      // Broadcast acknowledgement
      char ack = "ACK";
      rf95.send((uint8_t *)ack, strlen(ack));
      rf95.waitPacketSent();
    } else {
      Serial.println("Receive failed");
    }
  }
}
```
</details>

This code is available for download in `src/` within this repository.

Upload both code samples to RFM9x LoRa radio compatible boards. The following boards are tested:

- Arduino Uno with RFM9x Breakout
- Adafruit Feather M0
- Sparkfun ESP32 Thing Plus

Once these are running, you may disconnect the transmitter from data (but keep it on power) and proceed to the next section.

### Python Code

Create a `conda` environment by running the following commands in your terminal or Anaconda Prompt:

```zsh
conda create -y -n arduino python=3.8

conda activate arduino

conda install pyserial

pip install requests
```

Create the following Python files (in the same directory):

<details><summary>waterLevelHandler.py</summary>
<br>

```py
"""
Water Level Event Handler
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
"""

# Install dependencies
from serial import Serial # type: ignore
from time import sleep
from requests import post
import subprocess

# Open serial monitor
ser = Serial("/dev/cu.usbmodem11301", 115200)
sleep(2)

# Notification helpers
def low_water():
  post("https://ntfy.sh/hydrosync",
    data="Looks like you are running low on water! Refill your water bottle soon to stay hydrated",
    headers={
        "Title": "Low water detected",
        "Priority": "urgent",
        "Tags": "warning"
    })

# Reminders
subprocess.Popen(["python", "reminderHandler.py"])

# Events
while True:
  sleepTime = 1
  value = int(ser.readline().decode())
  
  print(value)
  
  if value == 0:
    low_water()
    sleepTime = 60
  
  sleep(sleepTime)
```
</details>

<details><summary>reminderHandler.py</summary>
<br>

```py
"""
Reminder Event Handler
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
"""

# Install dependencies
from time import sleep
from requests import post

# Notification helpers
def reminder():
  post ("https://ntfy.sh/hydrosync",
      data="It's been a while since you have last had water. Consider taking a sip out of your water bottle :)",
      headers={
          "Title": "Reminder",
          "Priority": "default",
          "Tags": "droplet"
      })

# Events
while True:
  reminder()
  sleep(1800)
```
</details>

Both of these files can be found ready for download in the `src/` directory. Activate the script:

```zsh
python waterLevelHandler.py
```

### Mobile Notifications

HYDROSYNC uses `ntfy`, an HTTP-based pub-sub service for sending mobile notifications to any device. Detailed information can be found at the [documentation](https://docs.ntfy.sh), but for those looking for a quick setup:

1. Download the `ntfy` app on any compatible device
2. Subscribe to the `hydrosync` topic
3. Ensure you have push notifications enabled

With this, HYDROSYNC should be fully set up and ready to go!
