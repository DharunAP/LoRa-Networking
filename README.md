# Esp32:

![Untitled](https://github.com/DharunAP/LoRa-Networking/assets/123437101/b1247f22-10f6-46e1-9ae6-01b87658003b)

<br>
<br>

<img src="https://github.com/DharunAP/LoRa-Networking/assets/123437101/b1247f22-10f6-46e1-9ae6-01b87658003b" alt="Untitled" width="400" height="300">

# **Specifications:**


### **Model :**

- ESP32 WROOM 32 by espressif

### Microcontroller:


- Dual-core Tensilica LX6 microprocessor
- Operating Frequency: Up to 240 MHz
- Ultra-Low Power Co-processor

### Wireless Connectivity:

- Integrated Wi-Fi 802.11 b/g/n (2.4 GHz)
- Bluetooth v4.2 BR/EDR and BLE (Bluetooth Low Energy)

### Memory:

- 520 KB SRAM
- 448 KB ROM (for boot loader and core functions)
- External SPIRAM support up to 16 MB

### Storage:

- Built-in 448 KB ROM
- External storage options:
    - SPI Flash up to 16 MB
    - MicroSD Card

### I/O Pins:


- 38 GPIO pins (General Purpose Input/Output)
- Interfaces for UART, SPI, I2C, I2S, PWM, SDIO, and CAN

### Analog Input:

- 18 channels of 12-bit SAR ADCs (Analog-to-Digital Converter)

### Operating Voltage:

- 3.3V (Not 5V tolerant, level shifting required for interfacing with 5V devices)

### Operating Temperature:

- 40°C to 125°C

### Power Consumption:

- Optimized for low power consumption
- Multiple sleep modes for power saving:
    - Deep Sleep: Shut down most of the ESP32's internal circuits
    - Light Sleep: Maintain enough state to wake up quickly
    - Modem Sleep: Turn off the Wi-Fi modem while keeping the CPU running

### Security:


- Hardware-based security features:
    - Secure Boot
    - Flash Encryption
- Cryptographic support for AES, SHA-2, RSA, ECC, etc.

### Size:

- Compact form factor suitable for various embedded applications

### Other Features:

- Real-Time Clock (RTC) with backup battery support
- Timers for PWM, watchdog, and RTC
- Touch Sensor Inputs
- Temperature Sensor
- Hall Effect Sensor
  
<br>
<br>

  # **Pin Diagram:**
![Untitled (1)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/62f7973f-f47e-41e7-8ed9-0771eb21d187)

<br>
<br>
<br>
<br>

# ESP32 Arduino IDE Setup:

**Installing the board:**

- Go to “Tools”
- Select “Board manager”
- Search “ESP32” in the search bar
- Select the esp32 by espressif
- Click the install button on the right bottom.

**Preferences:**

- Go to “Files”
- Select “Preferences”
- Paste https://dl.espressif.com/dl/package_esp32_index.json this link in the “Additional Board Manager URLs”.

**Driver Installation:**

- https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers click this text.
- Go to downloads and download the version for your specifications
- Extract the files once downloaded
- Click “SETUP” file
- Install the driver by accepting the policies and permissions

**Partition Scheme (Only for Client and Base Stations):**

- Go to “Tools”
- Select “Partition Schemes”
- Select “Minimal Spiffs (1.9MB APP with OTA / 190kb with SPIFFS)”


**Libraries :**

- Go to tools
- Select Manage libraries
- Search for these libraries and install it
    
    Radio Head
    WiFi
    HTTPClient
    WebServer
    Adafruit Unified Sensor
    Adafruit_Sensor
    Adafruit_BMP280
    
    Adafruit_BME280
    
    SoftwareSerial (if not installed)
    TinyGPS++ (if not installed)
    
    SPI (if not installed)

<br>
<br>
<br>
<br>


# ESP32 - LORA :


<br>


## LoRa (Long Range):

![Untitled (3)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/3defc863-1a3b-455f-abc5-dfff57fc3c3f)

<br>

## Specifications:

### Model :

- LoRa sx1278 (RF - 95) by SEMTECH (Antenna Mandatory)

### Electrical Specifications:

- Operating Voltage: Typically 1.8V to 3.7V (varies depending on specific module)
- TX Power Output: Up to 20 dBm (adjustable)
- Receiver Sensitivity: Down to -148 dBm
- Low Current Consumption in sleep mode: Typically less than 1 µA

### Frequency Range:

- 433 MHz, 470 MHz, 868 MHz, or 915 MHz bands (depending on the region and module variant)
- Programmable Frequency Synthesizer with resolution of 61 Hz

### Modulation:

- LoRa modulation with spread spectrum technology
- FSK modulation available for compatibility with other systems

### Data Rate:

- LoRa:
    - Up to 37.5 kbps (LoRa BW = 125 kHz, SF = 12)
    - Lower data rates possible with wider bandwidths and higher spreading factors
- FSK:
    - Programmable data rates up to 300 kbps

### Interface:

- SPI (Serial Peripheral Interface) for communication with microcontrollers or other devices
- Other control signals for configuration and control

### Operating Temperature Range:

- Typically from -40°C to 85°C, but specific module variants may have different ranges

### Other Features:

- Built-in packet engine with CRC (Cyclic Redundancy Check) for data integrity
- Support for frequency hopping to improve robustness in noisy environments
- Flexible power management options to optimize power consumption for different applications
- Support for various modulation schemes and spreading factors to trade off between range and data rate

### Package:

- Available in various packages, including small surface-mount modules for easy integration into designs

### Certification:

- Compliance with relevant regulatory standards, such as FCC (Federal Communications Commission) and ETSI (European Telecommunications Standards Institute), depending on the region and module variant

<br>

## Pin Configurations:

![Untitled (2)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/2d4098b5-1891-490c-b471-68ea30ac7981)

- VCC (3.3V) on SX1278 to 3V3 on ESP32
- GND on SX1278 to GND on ESP32
- SCK on SX1278 to D18 (HSPI CLK) on ESP32
- MISO on SX1278 to D19 (HSPI MISO) on ESP32
- MOSI on SX1278 to D23 (HSPI MOSI) on ESP32
- NSS (CS) on SX1278 to D5 (HSPI CS0) on ESP32
- DIO0 on SX1278 to D26 / D2 on ESP32
- RST on SXX1278 to D14 on ESP32

<br>
<br>
<br>
<br>

# ESP32 - BMP280:


<br>


## BMP280:

![Untitled (4)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/beee8415-939e-4722-9e22-35355fb6a7b8)

<br>

## Specifications :

- The BMP280  by is a barometric pressure sensor developed by Bosch Sensortec.

### Environmental Specifications:

- **Pressure Measurement Range:** 300 hPa to 1100 hPa (altitude range -500 m to 9000 m)
- **Temperature Measurement Range:** -40°C to 85°C

### Accuracy:

- **Pressure:** ±1 hPa (or ±1.7 meters)
- **Temperature:** ±1°C

### Resolution:

- **Pressure:** 0.18 Pa (or 1.52 cm)
- **Temperature:** 0.01°C

### Interfaces:

- **Communication:** I2C (up to 3.4 MHz) or SPI (up to 10 MHz)

### Power Consumption:

- **Current Consumption:** 2.7 μA at 1 Hz (ultra-low power consumption)

### Features:

- **Built-in Temperature Sensor:** Allows compensation for temperature variations during pressure measurements.
- **Digital Filtering:** Helps to reduce short-term pressure fluctuations caused by environmental conditions.
- **Low Power Consumption:** Designed for battery-powered applications and energy-efficient systems.
- **Calibration Coefficients:** Stored on-chip, eliminating the need for external circuitry or calibration.

### Applications:

- Weather Forecasting
- Altitude Measurement (e.g., drones, altimeters)
- Indoor Navigation (e.g., floor detection in smartphones)
- Health and Sports Monitoring (e.g., tracking elevation changes during exercise)
- Industrial Applications (e.g., HVAC systems, environmental monitoring)

<br>

## Pin Configurations :

![Untitled (5)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/f6ee9ef8-1dec-4049-b9e6-c1fb182a84a6)

**BMP280**                 **ESP32** 

VCC              -          3.3v

GND             -          GND

SDA              -          D21

SCK               -          D22

<br>
<br>
<br>
<br>


# ESP32 - BME280

<br>
   
## BME 280:

![Untitled (6)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/42ee76e8-07f0-40bd-8ef5-719db00180d9)

<br>

## Specifications:


### Model:

- The BME280 is a combined sensor developed by Bosch Sensotec.

### Environmental Specifications:

- **Pressure Measurement Range:** 300 hPa to 1100 hPa (altitude range -500 m to 9000 m)
- **Temperature Measurement Range:** -40°C to 85°C
- **Humidity Measurement Range:** 0% to 100% RH (Relative Humidity)

### Accuracy:

- **Pressure:** ±1 hPa (or ±1.7 meters)
- **Temperature:** ±1°C
- **Humidity:** ±3% RH

### Resolution:

- **Pressure:** 0.18 Pa (or 1.52 cm)
- **Temperature:** 0.01°C
- **Humidity:** 0.008% RH

### Interfaces:

- **Communication:** I2C (up to 3.4 MHz) or SPI (up to 10 MHz)

### Power Consumption:

- **Current Consumption:** Varies depending on the operating mode, typically ranges from 0.1 μA to 0.5 mA.

### Features:

- **Integrated Temperature Sensor:** Allows compensation for temperature variations during pressure and humidity measurements.
- **Digital Filtering:** Helps reduce short-term pressure fluctuations caused by environmental conditions.
- **Low Power Consumption:** Designed for battery-powered applications and energy-efficient systems.
- **Calibration Coefficients:** Stored on-chip, eliminating the need for external circuitry or calibration.
- **Optional IIR Filter:** Available for additional filtering of humidity data.
- **Humidity Sensing Accuracy:** Enhanced by factory calibration, compensating for sensor-to-sensor variations.

### Applications:

- **Weather Monitoring:** Provides accurate data for weather forecasting, including temperature, humidity, and pressure measurements.
- **Indoor Environmental Monitoring:** Tracks indoor climate conditions in smart home systems, HVAC (Heating, Ventilation, and Air Conditioning) systems, and other indoor applications.
- **Health and Wellness:** Monitors environmental conditions in wearable devices and health monitoring systems.
- **Industrial Applications:** Utilized for environmental monitoring in industrial automation, HVAC systems, and process control
  
<br>

## Pin Configurations:

![Untitled (7)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/b2e04d3c-333a-4290-8e71-6ab23f275f7b)


**BMP280**                 **ESP32** 

VCC              -          3.3v

GND             -          GND

SDA              -          D21

SCK               -          D22


<br>
<br>
<br>

<br>


# ESP32 - L89 :


<br>


## L89:
![Untitled (8)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/06a7e3f7-622e-4006-8ad4-b28e1759ad41)

<br>

## Specifications:

**Model:**

- L89 GNSS module by Quectel

**GNSS Support**:

- **Constellations**: GPS, IRNSS, GLONASS, BeiDou, Galileo, and QZSS.
- **Frequency Bands**: L1 (1575.42 MHz), L5 (1176.45 MHz), B1 (1561.098 MHz), and E1.

**Antennas and Sensitivity**:

- Dual embedded antennas (patch and chip antenna).
- Integrated LNAs (Low Noise Amplifiers) for improved sensitivity.
- **Tracking Sensitivity**: -147 dBm.
- **Acquisition Sensitivity**: -162 dBm.

**Performance**:

- **TTFF (Time to First Fix)**: Reduced due to multi-band operation.
- Enhanced positioning accuracy, especially in urban environments.

**Interface and Power**:

- **Interfaces**: I2C, UART.
- **Supply Voltage**: 3.1 to 4.3 V.
- **Acquisition Current**: 90 mA.

**Physical and Environmental**:

- **Dimensions**: 25.0 mm × 16.0 mm × 6.8 mm.
- **Weight**: 8.2 g.
- **Operating Temperature**: -40 to 85°C.
- **Compliance**: RoHS compliant.

**Additional Features**:

- Support for DGPS and SBAS (WAAS, EGNOS, MSAS, GAGAN).
- Great anti-jamming performance due to multi-frequency operation.
<br>
<br>

## Pin Configurations:

![Untitled (9)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/7d56d2e5-7d32-4040-ae67-09ddf02e1278)


**L89**                          **ESP32** 

VCC              -          3.3v

GND             -          GND

RX                -          D12 / D16

TX                -          D13 / D17


 
<br>
<br><br>
<br>
  
<br>
  
# Mesh Network - Rhmesh

https://github.com/PaulStoffregen/RadioHead/blob/master/RHMesh.h   🐱

The RHMesh library is part of the RadioHead suite developed by Mike McCauley. It extends the RHRouter class to support mesh networking, enabling multi-hop routing and automatic route discovery in dynamic network topologies. This makes RHMesh particularly suitable for applications where nodes can move or change status, ensuring reliable communication even in fluid environments.
  
  
## Features

1. **Mesh Networking:** Supports multi-hop communication across a network of nodes.
2. **Automatic Route Discovery:** Dynamically discovers routes to destination nodes when needed.
3. **Route Failure Handling:** Detects and responds to route failures, ensuring messages can be rerouted as necessary.
4. **Reliable Hop-to-Hop Delivery:** Uses acknowledgments at each hop to ensure reliable delivery.
5. **Optimized Memory Usage:** Designed to operate within the limited memory constraints of typical Arduino and similar microcontroller environments.
6. **Message Optimisation:** Intermediate nodes cannot decode messages; only destination nodes can unwrap them, ensuring end-to-end encryption and security.
7. **Auto Node-Addition:** New node is automatically added to the RouteTable once its initiated.
8. BroadCast message: Each node can able send a single message to all other node in the route table by using NODE_ID : 255 (or)RH_BROADCAST_ADDRESS.
  
<br>

## Protocol Overview
  
<br>
  
### Message Types

- **RH_MESH_MESSAGE_TYPE_APPLICATION:** Used for application layer messages.
- **RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST:** Broadcasted to discover routes to a destination node.
- **RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE:** Unicast response containing the discovered route.
- **RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE:** Notifies nodes of a failure in the established route.
  

  
### Route Status Code:

- 0 ⇒ RH_ROUTER_ERROR_NONE
- 1 ⇒RH_ROUTER_ERROR_NO_ROUTE
- 2 ⇒RH_ROUTER_ERROR_TIMEOUT
- Not Recognised ⇒ General failure
  
<br>
    
<br>
  
## Detailed Functionality
  
<br>
  
### Initialization and Basic Operation

When an RHMesh node starts, it doesn't have knowledge of any routes. It relies on automatic route discovery to establish paths to other nodes.

**Constructor:**

```cpp
RHMesh(RHGenericDriver& driver, uint8_t thisAddress = 0);

```

- Initializes the mesh node with a given driver and node address.
  
<br>
  
### Route Discovery

1. **Sending a Message**: When a node sends a message using `sendtoWait`, it first checks if a route to the destination exists in its routing table.
2. **Route Request**: If no route is known, it broadcasts a `MeshRouteDiscoveryMessage` with type `RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST`.
3. **Intermediate Nodes**: Nodes receiving the request check if the destination is themselves. If not, they rebroadcast the request, adding themselves to the list of visited nodes.
4. **Destination Node**: When the destination node receives the request, it sends a unicast `MeshRouteDiscoveryMessage` with type `RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE` back to the origin.
5. **Route Formation**: Intermediate nodes use the response to update their routing tables with the route back to the origin and other nodes on the path.
  
<br>
    
  
### Route Failure Handling

When a node cannot deliver a message to the next hop:

1. **Route Failure Message:**
    - It sends a `MeshRouteFailureMessage` to the originator indicating the failure.
2. **Route Deletion:**
    - Intermediate nodes and the originator delete the failed route from their routing tables.
3. **Reattempt:**
    - If a message needs to be sent again, a new route discovery process is initiated.
  
<br>
    
<br>
  
## Class Structure and Methods
  
<br>
    

### Public Methods

- **sendtoWait:**
    - Sends a message to the destination, initiating route discovery if necessary.
    
    ```cpp
    uint8_t sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags = 0);
    
    ```
    
    - **Parameters:**
        - `buf`: Pointer to the message data.
        - `len`: Length of the message.
        - `dest`: Destination node address.
        - `flags`: Optional flags for use by subclasses or application layer.
- **recvfromAck:**
    - Receives a message addressed to this node, sends an acknowledgment, and processes the message.
    
    ```cpp
    bool recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL, uint8_t* hops = NULL);
    
    ```
    
    - **Parameters:**
        - `buf`: Location to copy the received message.
        - `len`: Available space in `buf`, set to the actual number of bytes copied.
        - `source`, `dest`, `id`, `flags`, `hops`: Optional pointers to retrieve additional message details.
- **recvfromAckTimeout:**
    - Similar to `recvfromAck`, but with a timeout parameter.
    
    ```cpp
    bool recvfromAckTimeout(uint8_t* buf, uint8_t* len, uint16_t timeout, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL, uint8_t* hops = NULL);
    
    ```
    
    - **Parameters:**
        - `timeout`: Maximum time to wait in milliseconds.
  
<br>
  
### Protected Methods

- **peekAtMessage:**
    - Inspects received messages and adjusts the routing table if necessary.
    
    ```cpp
    virtual void peekAtMessage(RoutedMessage* message, uint8_t messageLen);
    
    ```
    
    - **Parameters:**
        - `message`: Pointer to the received `RoutedMessage`.
        - `messageLen`: Length of the message.
- **route:**
    - Adjusts the routing table when sending messages.
    
    ```cpp
    virtual uint8_t route(RoutedMessage* message, uint8_t messageLen);
    
    ```
    
    - **Parameters:**
        - `message`: Pointer to the `RoutedMessage` to be sent.
        - `messageLen`: Length of the message.
- **doArp:**
    - Resolves a route for the given address by initiating route discovery.
    
    ```cpp
    virtual bool doArp(uint8_t address);
    
    ```
    
    - **Parameters:**
        - `address`: Physical address to resolve.
- **isPhysicalAddress:**
    - Tests if the given address matches the physical address of this node.
    
    ```cpp
    virtual bool isPhysicalAddress(uint8_t* address, uint8_t addresslen);
    
    ```
    
    - **Parameters:**
        - `address`: Address being tested.
        - `addresslen`: Length of the address.
  
<br>
  
### Private Members

- **_tmpMessage:**
    - A temporary message buffer for internal use.
  
<br>
    
<br>
  
## Example Usage
  
<br>
  
### Basic Initialization

```cpp
#include <RHMesh.h>
#include <RH_RF95.h>

RH_RF95 driver;
RHMesh manager(driver, 1); // Node address is 1

void setup() {
  Serial.begin(9600);
  if (!driver.init()) {
    Serial.println("Driver init failed");
  }
  manager.init();
}

void loop() {
  uint8_t data[] = "Hello World";
  manager.sendtoWait(data, sizeof(data), 2); // Send to node 2
  delay(1000);
}

```
  
<br>
  
### Receiving Messages

```cpp
void loop() {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from)) {
    Serial.print("Received message from node: ");
    Serial.println(from);
    Serial.print("Message: ");
    Serial.println((char*)buf);
  }
  delay(1000);
}

```

The RHMesh library provides a robust framework for implementing mesh networks with dynamic routing and reliable communication. Its design accommodates the constraints of microcontroller environments while offering essential features for complex networking scenarios. By leveraging automatic route discovery and handling route failures, RHMesh ensures resilient and flexible wireless communication.



<br>


<br>
    
---

  
# Mesh Nodes
  
<br>
   
<br>
   
  
## Central Station:
  

## Specifications:

**Components:**

- ESP32
- LoRa -sx1278

- Antenna - 433MHz

**Functionality:** 

- To receive messages from the other nodes
- Update the messages in the database
- To Broadcast Time and Date to all other nodes

**Location:**

- Away from the flood prone area with Wi-Fi Connectivity.
  
<br>
    
<br>
  
## Code:

```cpp
// COM -
//==========================================================================================================
//                                              L I B R A R I E S                                           |
                                                                                                            
//==========================================================================================================
// Include necessary libraries
#include <SPI.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <Arduino.h>
#include "time.h"
#include "esp_task_wdt.h"

//==========================================================================================================
//                                              I N I T I A L I S A T I O N                                 |
                                                                                                            
//==========================================================================================================
// NTP server for time synchronization
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT+5:30
const int daylightOffset_sec = 3600;  // Daylight saving offset

// Constants for message receiving timeout and retry delay
const uint16_t RECV_TIMEOUT = 200;  // Milliseconds
const uint16_t SEND_RETRY_DELAY = 100;  // Milliseconds

// WiFi credentials
const char* ssid = "POCO";  // Replace with your WiFi SSID
const char* password = "hellobrooo";  // Replace with your WiFi password
const char* serverUrl = "http://192.168.112.1:8000/user/loraSend";  // Replace with your server endpoint

// Global variables
bool flg = false;
String TimeData = "";

// Define pins for the RF module
#define RFM95_CS 5  // Chip select
#define RFM95_RST 14  // Reset
#define RFM95_INT 26  // Interrupt

// Unique node ID for this node in the mesh network
#define NODE_ID 1

// Initialize the RF driver and mesh manager
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHMesh manager(rf95, NODE_ID);

// Task handles for the FreeRTOS tasks
TaskHandle_t Task1;
TaskHandle_t Task2;

//==========================================================================================================
//                                              S E T U P                                                   |
                                                                                                            
//==========================================================================================================
void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate

  // Reset the RF module
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the RF module
  if (!rf95.init()) {
    Serial.println("RF95 init failed");
    while (1);  // Halt if initialization fails
  }
  rf95.setFrequency(433.0);  // Set the frequency to 433 MHz

  // Initialize the mesh network
  if (!manager.init()) {
    Serial.println("Mesh init failed");
    while (1);  // Halt if initialization fails
  }
  rf95.setTxPower(5);  // Set transmission power
  Serial.println("Mesh init successful");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi");

  // Initialize and get the current time from the NTP server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // Create two FreeRTOS tasks pinned to different cores
  xTaskCreatePinnedToCore(Task1code, "Task1", 20000, NULL, 2, &Task1, 0);  // Increased priority
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
}

//==========================================================================================================
void loop() {
  // Empty loop since we are using tasks
}
//==========================================================================================================
//                                              C O R E - 0                                                 |
                                                                                                            
//==========================================================================================================
void Task1code(void *pvParameters) {
  esp_task_wdt_add(NULL);  // Add the current task to the watchdog timer
  Serial.println("Task1 started");

  for (;;) {
    esp_task_wdt_reset();  // Feed the watchdog timer
    printLocalTime();  // Print the local time
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Yield to other tasks
    esp_task_wdt_reset();  // Feed the watchdog timer
  }
}

//==========================================================================================================
//                                              C O R E - 1                                                 |
                                                                                                            
//==========================================================================================================
void Task2code(void *pvParameters) {
  esp_task_wdt_add(NULL);  // Add the current task to the watchdog timer
  for (;;) {
    esp_task_wdt_reset();  // Feed the watchdog timer

    // Buffer for receiving messages
    uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from, to, id, flags, hops;

    // Receive messages with a timeout
    if (manager.recvfromAckTimeout(buf, &len, RECV_TIMEOUT, &from, &to, &id, &flags, &hops)) {
      Serial.print("Received message from node ");
      Serial.print(from);
      Serial.print(" to node ");
      Serial.print(to);
      Serial.print(": ");
      Serial.println((char*)buf);  // Print the received message
      
      //String abc = (char*)buf ;
      // sendDatas(String(abc);  // Send data to the server (currently commented out)
    } else {
      Serial.println("No message received");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Yield to other tasks
    esp_task_wdt_reset();  // Feed the watchdog timer
  }
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 1                         |
                                                                                                            
//==========================================================================================================
void sendDatas(String data) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // Set server URL
    http.begin(client, serverUrl);

    // Set headers
    http.addHeader("Content-Type", "text/plain");

    // Send POST request
    int httpResponseCode = http.POST(data);
    Serial.println(httpResponseCode);

    // Check for response
    if (httpResponseCode > 0) {
      Serial.print("HTTP POST request successful, response code: ");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.println("Response: " + response);
    } else {
      Serial.print("Error sending HTTP POST request, error code: ");
      Serial.println(httpResponseCode);
    }

    // Close connection
    http.end();
  } else {
    Serial.println("WiFi connection lost, reconnecting...");
    WiFi.reconnect();
  }
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 0                         |
                                                                                                            
//==========================================================================================================
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  //Different time formats
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");

  // Extract minute and second as strings
  char timeMin[3];
  strftime(timeMin, 3, "%M", &timeinfo);
  Serial.print("Minute: ");
  Serial.println(timeMin);

  char timeSec[3];
  strftime(timeSec, 3, "%S", &timeinfo);
  Serial.print("Second: ");
  Serial.println(timeSec);

  // Convert seconds to integer
  int sec = atoi(timeSec);

  // Check if the seconds are equal to 15, 30, 45, 0, 20, 10, or 50
  if (sec == 15 || sec == 30 || sec == 45 || sec == 00 || sec == 20) {
    Serial.println("The second is either 15, 30, 45, 0, 20");
    TimeData = "";
    TimeData += timeMin[0];
    TimeData += timeMin[1];
    TimeData += timeSec[0];
    TimeData += timeSec[1];

    // Prepare the message for sending
    uint8_t buff[TimeData.length() + 1];
    TimeData.getBytes(buff, sizeof(buff));
    uint8_t len = TimeData.length() + 1;
    Serial.print("TimeData=");
    Serial.println(TimeData);

    // Send the message
    uint8_t result = manager.sendtoWait(buff, len, RH_BROADCAST_ADDRESS);
    Serial.println("Message SENDING");
    Serial.print("Result=");
    Serial.println(result);

    // Handle send errors and retry if necessary
    if (result != RH_ROUTER_ERROR_NONE) {
      handleSendError(result);
      vTaskDelay(SEND_RETRY_DELAY / portTICK_PERIOD_MS);  // Delay before retrying
      esp_task_wdt_reset();  // Feed the watchdog timer

      //Sending message Again
      result = manager.sendtoWait(buff, len, RH_BROADCAST_ADDRESS);
      Serial.println("Retrying Message SENDING");
      Serial.print("Result=");
      Serial.println(result);

      //If the meesage if not success
      if (result != RH_ROUTER_ERROR_NONE) {
          handleSendError(result);
      } else {
        //if result code is equal to RH_ROUTER_ERROR_NONE (it means 0)
          Serial.println("Message forwarded successfully to the next hop.");
      }
  } else {
    //if result code is equal to RH_ROUTER_ERROR_NONE (it means 0)
      Serial.println("Message forwarded successfully to the next hop.");
  }

  } else {
    //if second is not equal to 15, 30, 45 , 0 or 20
    Serial.println("The second is not 0 , 20 , 15, 30, or 45.");
  }
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   B O T H   C O R E                   |
                                                                                                            
//==========================================================================================================

//Result status if it fails 
void handleSendError(uint8_t result) {
  if (result == RH_ROUTER_ERROR_NO_ROUTE) {
    //result code = 1
    Serial.println("Error: No route to destination node.");
    
  } 
  else if (result == RH_ROUTER_ERROR_TIMEOUT) {
    //result code = 2
    Serial.println("Error: Timeout waiting for ACK.");
  } 
  else {
    
    // result code no recognised
    Serial.println("Error: General send failure.");
  }
}
```
  
<br>
    
  
## Explanation:
  

   
   
### Libraries and Global Constants

1. *Libraries*: The code includes several libraries for different functionalities:
    - *SPI.h*: Enables SPI communication.
    - *LoRa.h*: Manages LoRa communication.
    - *WiFi.h*: Handles WiFi connectivity.
    - *HTTPClient.h*: Facilitates HTTP communication.
    - *RHMesh.h* and *RH_RF95.h*: Support the mesh network protocol and RF module.
    - *Arduino.h*: Standard Arduino functions.
    - *time.h*: For time-related functions.
    - *esp_task_wdt.h*: Manages the watchdog timer to reset the system in case tasks become unresponsive.
2. *NTP Server Settings*: These constants configure the Network Time Protocol (NTP) server for time synchronization:
    - ntpServer: The NTP server URL.
    - gmtOffset_sec: The GMT offset in seconds.
    - daylightOffset_sec: The daylight saving time offset in seconds.
3. *WiFi Credentials*: These are the SSID and password for the WiFi network that the ESP32 will connect to, along with the server URL for data transmission.
4. *Mesh Network Settings*: Configuration for the RF module:
    - Defines chip select, reset, and interrupt pins.
    - Assigns a unique node ID to this node within the mesh network.
5. *FreeRTOS Task Handles*: Task handles for the two FreeRTOS tasks that will run concurrently.
  
<br>
   
### Setup Function

1. *Serial Communication*: Initializes serial communication at 9600 baud rate for debugging and logging purposes.
2. *RF Module Initialization*:
    - The RF module is reset by toggling the reset pin.
    - Initializes the RF module and sets the frequency to 433 MHz.
    - Initializes the mesh network with the node ID and sets the transmission power.
3. *WiFi Connection*:
    - Connects the ESP32 to the specified WiFi network.
    - Continuously checks connection status and waits until the device is connected.
4. *Time Synchronization*:
    - Configures the system time using the specified NTP server.
    - Calls a function to print the local time, ensuring synchronization.
5. *FreeRTOS Tasks Creation*:
    - Creates two FreeRTOS tasks with different priorities and assigns them to separate cores of the ESP32.
  
<br>
   
### Loop Function

The loop function is empty as the main functionalities are managed by the two FreeRTOS tasks.
  
<br>
   
### Task1code Function

1. *Watchdog Timer*:
    - Adds the current task to the watchdog timer to prevent it from hanging.
2. *Time Synchronization*:
    - Prints the local time every second to keep track of it.
    - The task delays for one second, allowing other tasks to execute.
  
<br>
   
### Task2code Function

1. *Watchdog Timer*:
    - Adds the current task to the watchdog timer.
2. *Message Handling*:
    - Creates a buffer for receiving messages.
    - Tries to receive messages with a timeout. If a message is received, it prints and sends the data to the server. If no message is received within the timeout, it logs that no message was received.
    - The task delays for 100 milliseconds, allowing other tasks to run.
  
<br>
   
### sendDatas Function

1. *WiFi Connection Check*:
    - Checks if the WiFi is connected.
2. *HTTP POST Request*:
    - Creates an HTTP POST request to send the received data to the specified server.
    - Logs the response code and response body for verification.
3. *Error Handling*:
    - If the WiFi connection is lost, the function attempts to reconnect.
  
<br>
   
### printLocalTime Function

1. *Time Information*:
    - Obtains and prints the local time in various formats.
2. *Time Variables Extraction*:
    - Extracts the minute and second as strings.
3. *Time Conditions Check*:
    - Checks if the current second matches specific values (15, 30, 45, 0, 10, 20, or 50).
    - If the condition is met, prepares a message and sends it to all nodes in the mesh network using the broadcast address.
4. *Message Sending*:
    - Sends the prepared message to the broadcast address and logs the result.
    - If the send operation fails, handles the error and retries after a delay.
  
<br>
   
### Key Concepts and Functionalities

1. *Task Management*: Uses FreeRTOS tasks to handle time synchronization and message handling concurrently, enabling efficient multitasking on the ESP32.
2. *Mesh Networking*: Uses the RHMesh library to create a mesh network, allowing nodes to communicate with each other.
3. *Time Synchronization*: Synchronizes system time with an NTP server to ensure accurate timekeeping across all nodes.
4. *WiFi Communication*: Connects to a WiFi network and sends data to a specified server using HTTP POST requests.
5. *Watchdog Timer*: Prevents tasks from stalling by resetting the system if tasks become unresponsive.
   
<br>
     

 
### Workflow Summary

1. *Setup*:
    - Initializes serial communication, RF module, WiFi connection, and time synchronization.
    - Creates two tasks for time synchronization and message handling.
2. *Task1code*:
    - Continuously prints the local time every second.
3. *Task2code*:
    - Continuously checks for incoming messages and sends them to the server if received.
4. *printLocalTime*:
    - Checks the current time every second. If the second matches specific values, prepares and broadcasts a time update message to all nodes.
5. *sendDatas*:
    - Sends received data to a server using HTTP POST requests, with error handling and reconnection logic for WiFi.

The code leverages LoRa for long-range, low-power wireless communication and WiFi for internet connectivity, making it suitable for IoT applications that require reliable communication over large areas.

---
  
<br>
   
<br>
   

 
# Client Node:
  
<br>
 
## Specifications:

**Components:**

- ESP32
- LoRa -sx1278
- Antenna - 433MHz
- Quectel Navic L89

**Functionality:** 

- Host server through ESP32 (”Host”) and receives messages (as request) from the mobile application (”Client”). vn
- Send the received messages to the Base Station
- If time and date is received from the Central Station, it updates its date and time.
- Send message consists of GPS data and updated date time
- If any messages received from other nodes, forward to the next node (nearest to destination).
- Perform FUOTA (Firmware Update Over The Air) if updated firmware available.

**Location:**

- Present In the flood prone area.
  
<br>
     
<br>
  
 
## Code:

```cpp
// COM -
//==========================================================================================================
//                                              L I B R A R I E S                                           |
                                                                                                            
//==========================================================================================================

// Includes for the RFM95 Mesh Networking and GPS modules
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Includes for Wi-Fi, HTTP and OTA updates for ESP32
#include <WiFi.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Arduino.h>

/==========================================================================================================
//                                              I N I T I A L I S A T I O N                                 |
                                                                                                            
//==========================================================================================================
// Pin definitions for RFM95 module
#define RFM95_CS 5       // Chip Select pin for RFM95
#define RFM95_RST 14     // Reset pin for RFM95
#define RFM95_INT 26     // Interrupt pin for RFM95

// Node IDs for the mesh network
#define NODE_ID 3        // Unique ID for this node
#define DEST_NODE_ID 1   // Destination node ID is Base station

// GPS module pin definitions and settings
#define ARDUINO_GPS_RX 16   // GPS module RX pin
#define ARDUINO_GPS_TX 17   // GPS module TX pin
#define GPS_BAUD_RATE 9600  // GPS module baud rate
#define SerialMonitor Serial // Serial monitor

// Initializing the RF driver and mesh manager
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHMesh manager(rf95, NODE_ID);

// Initializing SoftwareSerial for GPS communication
SoftwareSerial gpsPort(ARDUINO_GPS_TX, ARDUINO_GPS_RX);
TinyGPSPlus gps;

// Variables to store GPS coordinates
float latitude = 19.000;
float longitude = 18.600;
int hr = 0;
int mi = 0;
int sec = 0;

// Access Point credentials
const char* sssid = "ESP32_Hotspot";
const char* passsword = "123456789";

// Current firmware version
const String currentVersion = "1.0.0";

// WebServer instance on port 80
WebServer server(80);

// Function prototypes for handling various tasks

void handleRoot();
void handleData();
void handleVersionCheck();

// Task handles for FreeRTOS tasks
TaskHandle_t Task1;
TaskHandle_t Task2;

//==========================================================================================================
//                                              S E T U P                                                   |
                                                                                                            
//==========================================================================================================

void setup() {
  Serial.begin(9600);
  gpsPort.begin(GPS_BAUD_RATE);
  SerialMonitor.begin(9600);

  // Reset the RFM95 module
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the RF95 module
  if (!rf95.init()) {
    Serial.println("RF95 init failed");
    while (1);
  }
  rf95.setFrequency(433.0); // Set the frequency to 433 MHz

  // Initialize the mesh network manager
  if (!manager.init()) {
    Serial.println("Mesh init failed");
    while (1);
  }
  rf95.setTxPower(5); // Set transmission power
  Serial.println("Mesh init successful");

  // Start Wi-Fi Access Point
  WiFi.softAP(sssid, passsword);
  server.on("/", handleRoot); // Define route for root URL
  server.on("/endpoint", handleData); // Define route for data endpoint
  server.on("/version", handleVersionCheck); // Define route for version check
  server.begin(); // Start the server
  Serial.println("HTTP server started");

  // Create FreeRTOS tasks for concurrent execution
  xTaskCreatePinnedToCore(
    Task1code,    // Function to be called
    "Task1",      // Name of the task
    10000,        // Stack size (bytes)
    NULL,         // Parameter to pass to the task
    1,            // Task priority
    &Task1,       // Task handle
    0);           // Core to run the task on (0)

  xTaskCreatePinnedToCore(
    Task2code,    // Function to be called
    "Task2",      // Name of the task
    10000,        // Stack size (bytes)
    NULL,         // Parameter to pass to the task
    1,            // Task priority
    &Task2,       // Task handle
    1);           // Core to run the task on (1)
}

//=========================================================================================================
void loop() {
  // Do nothing in the main loop                                                                           |
}
//=========================================================================================================

//==========================================================================================================
//                                              C O R E - 0                                                 |
                                                                                                            
//==========================================================================================================
// Task1 function for handling GPS data ---> CORE 0
void Task1code(void * pvParameters) {
  for (;;) {
    // Read data from GPS module
    while (gpsPort.available() > 0) {
      char c = gpsPort.read();
      Serial.write(c);         
      gps.encode(c);            
    }

    // Forward data from Serial to GPS module
    while (Serial.available() > 0) {
      gpsPort.write(Serial.read()); 
    }

    // If GPS location is updated, store the new coordinates
    if (gps.location.isUpdated()) {
      Serial.println("L");
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6); 
      latitude = gps.location.lat();
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6); 
      longitude = gps.location.lng();
    }
 
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to yield control
  }
}

//==========================================================================================================
//                                              C O R E - 1                                                 |
                                                              //                                            |
//==========================================================================================================

// Task2 function for handling server requests and mesh communication   ---> CORE 1
void Task2code(void * pvParameters) {
  for (;;) {
    server.handleClient(); // Handle HTTP requests

    uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from, to, id, flags, hops;

    // Receive and process messages from the mesh network
    if (manager.recvfromAck(buf, &len, &from, &to, &id, &flags, &hops)) {
      if (from == 1) {
        
        // When the time is updated by central station is given to this node. 
        // Its getting the values from sensor and updating time with it. 
        // Sending the datas( parameters and givem time).
        
        Serial.print("Received message from node "); 
        Serial.print(from);
        Serial.print(" to node "); 
        Serial.print(to);
        Serial.print(": "); 
        Serial.println((char*)buf);

        //Getting the Hour and Minute from central station in Integer format
        int hr = (buf[0] - '0') * 10 + (buf[1] - '0');
        int mi = (buf[2] - '0') * 10 + (buf[3] - '0');

        //String of combinig all parameters from the sensor to single string
        char dat[50];
        snprintf(dat, sizeof(dat), "%02d%02d%.3f%.3f", hr, mi, latitude, longitude);
        Serial.print("Data=");
        Serial.println(dat);
        
        //Conversion of String to the unit8_t(buf) and len of buf
        int dataLength = strlen(dat);
        uint8_t buff[RH_MESH_MAX_MESSAGE_LEN];
        memcpy(buff, dat, dataLength + 1); // Include null terminator
        len = dataLength + 1;
        uint8_t cent = 1;
        
        //delay (specifying unique delay for each node by multiplying the node_ID to the 1200. So every Node sending will have interval of 1200ms from next node. So we can reducing the data traffic at central station. 
        vTaskSuspend(NULL);  
        vTaskDelay(NODE_ID * 1200 / portTICK_PERIOD_MS);
        
        //Sending messages
        uint8_t result = manager.sendtoWait(buff, len, cent);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);
        
        //Printing the result Status
        if (result == RH_ROUTER_ERROR_NONE) {
          //result code = 0
          Serial.println("Message forwarded successfully to the next hop.");
        } else {
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          handleSendError(result);

          // Retry sending message
          result = manager.sendtoWait(buff, len, cent);
          Serial.println("Message SENDING");
          Serial.print("result=");
          Serial.println(result);
          //Printing the result Status
          if (result == RH_ROUTER_ERROR_NONE) {
            //result code = 0
            Serial.println("Message forwarded successfully to the next hop.");
          } else {
            Serial.print("Message send failed, error code: ");
            Serial.println(result);
            handleSendError(result);
          }
        }
      } else {
        // Forward message to the destination node
        uint8_t result = manager.sendtoWait(buf, len, to);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);

        //printing the reult Status
        if (result == RH_ROUTER_ERROR_NONE) {
          //result  code =0
          Serial.println("Message forwarded successfully to the next hop.");
        } else {
          
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          //Calling the function 
          handleSendError(result);

          // Retry sending message
          result = manager.sendtoWait(buf, len, to);
          Serial.println("Message SENDING");
          Serial.print("result=");
          Serial.println(result);

          //Pritnting the result Status
          if (result == RH_ROUTER_ERROR_NONE) {
            //result code = 0
            Serial.println("Message forwarded successfully to the next hop.");
          } else {
            Serial.print("Message send failed, error code: ");
            Serial.println(result);
            handleSendError(result);
          }
        }
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to yield control
  }
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 1                         |
                                                                                                            
//==========================================================================================================
// Handle errors in sending messages over the mesh network
void handleSendError(uint8_t result) {
  if (result == RH_ROUTER_ERROR_NO_ROUTE) {
    //result = 1
    Serial.println("Error: No route to destination node.");
  } else if (result == RH_ROUTER_ERROR_TIMEOUT) {
    //result = 2
    Serial.println("Error: Timeout waiting for ACK.");
  } else {
    //reult = not recongnised
    Serial.println("Error: General send failure.");
  }
}

void handleUpdate() {
  // Get the current upload status and data
  HTTPUpload& upload = server.upload();

  // If the upload has started
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Update: %s\n", upload.filename.c_str());  // Print the filename of the firmware being uploaded

    // Start the update with an unknown size
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);  // Print any errors that occur during the start of the update
    }
  } 
  // If data is being written to the flash memory
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Write the received buffer to the update and check if the size matches
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);  // Print any errors that occur during writing
    }
  } 
  // If the upload has finished
  else if (upload.status == UPLOAD_FILE_END) {
    // End the update and set the final size
    if (Update.end(true)) {
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);  // Print the total size of the uploaded firmware

      // Check if the update is finished
      if (Update.isFinished()) {
        esp_ota_mark_app_valid_cancel_rollback();  // Mark the update as valid and cancel any rollback
        ESP.restart();  // Restart the ESP to apply the update
      } else {
        Serial.println("Update not finished? Something went wrong!");  // Print an error if the update is not finished
        esp_ota_mark_app_invalid_rollback_and_reboot();  // Mark the update as invalid and rollback
      }
    } else {
      Update.printError(Serial);  // Print any errors that occur during the end of the update
    }
  } 
  // If there is an error with the upload
  else {
    Update.printError(Serial);  // Print the error
  }
}

// Handle root URL request
void handleRoot() {
  Serial.println("Received request...");
  server.send(200, "text/plain", "success");
}

// Handle data endpoint request
void handleData() {
  if (server.method() == HTTP_POST) {
    String dat = server.arg("data");
    Serial.println("Received data: " + dat);

    
      uint8_t buff[dat.length() + 1];
      dat.getBytes(buff, sizeof(buff));
      uint8_t len = dat.length() + 1;
      uint8_t to = DEST_NODE_ID;
      uint8_t result = manager.sendtoWait(buff, len, to);
      Serial.println("Message SENDING");
      Serial.print("result=");
      Serial.println(result);

      //Printing the result Status
      if (result == RH_ROUTER_ERROR_NONE) {
        //Result code = 0
        Serial.println("Message forwarded successfully to the next hop.");
      } else {
        Serial.print("Message send failed, error code: ");
        Serial.println(result);
        handleSendError(result);

        // Retry sending message
        result = manager.sendtoWait(buff, len, to);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);
        
        //Printing the result Status
        if (result == RH_ROUTER_ERROR_NONE) {
          //Result code = 0
          Serial.println("Message forwarded successfully to the next hop.");
        } else {
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          handleSendError(result);
        }
      }
    
    server.send(200, "text/plain", "Data received by ESP32");
  }
}

// Handle version check endpoint request
void handleVersionCheck() {
  server.send(200, "text/plain", currentVersion);
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 0                         |
                                                                                                            
//==========================================================================================================

//                                                NO specific functions
```
    
<br>
 
<br>
 
## Explanation:
  

 
### **Libraries:**

- **RFM95 and GPS Libraries:** Includes libraries for mesh networking (RHMesh, RH_RF95) and GPS communication (TinyGPS++, SoftwareSerial).
- **ESP32 Libraries:** Includes libraries for Wi-Fi, HTTP, OTA updates, and ESP32 specific operations (WiFi, HTTPClient, Update, esp_ota_ops, WebServer, Arduino).
  
<br>
 
### **Initialization:**

- **Pin Definitions:** Defines the pins used for the RFM95 module (chip select, reset, interrupt), GPS module (RX, TX), and serial monitor.
- **Node IDs:** Sets the unique ID for the current node and the destination node ID for the base station.
- **GPS Settings:** Sets the GPS baud rate and initializes the serial monitor.
- **Mesh Network:** Initializes the RF driver and mesh manager with the node ID.
- **Wi-Fi Credentials:** Sets Wi-Fi credentials for connection and access point.
- **Web Server:** Creates a WebServer instance on port 80.
- **FreeRTOS Tasks:** Task handles for managing FreeRTOS tasks.
  
<br>
 
### **Setup Function:**

- **Serial Communication:** Initializes serial communication for debugging and GPS.
- **RFM95 Initialization:** Resets and initializes the RFM95 module, setting its frequency to 433 MHz and transmission power.
- **Mesh Network Initialization:** Initializes the mesh network manager.
- **Wi-Fi Access Point:** Starts the Wi-Fi access point and sets up HTTP server routes for handling requests.
- **FreeRTOS Tasks:** Creates and pins two FreeRTOS tasks (Task1code and Task2code) to specific cores for concurrent execution.
  
<br>
 
### **Main Loop:**

- The main loop is empty because the code uses FreeRTOS tasks to handle operations concurrently.
  
<br>
 
### **Task1code (Core 0):**

- **GPS Data Handling:** Continuously reads data from the GPS module and serial port, updating the GPS location if new data is available.
- **Latitude and Longitude Storage:** Stores the updated latitude and longitude values when GPS location is updated.
- **Task Delay:** Delays the task for a short period to yield control.
  
<br>
 
### **Task2code (Core 1):**

- **HTTP Server Handling:** Handles HTTP client requests.
    - **Request Message:** Received requests are converted into message formats and send to the Base Station.
    - **OTA update:** If upadate request received, it calls the *“handleUpdate”*  function*.*
- **Mesh Network Communication:** Receives and processes messages from the mesh network.
    - **Message Handling:** Processes messages from the central station, updates the time, and GPS data back to the Base station.
    - **Message Forwarding:** Forwards messages to the destination node.
- **Task Delay:** Delays the task for a short period to yield control.
  
<br>
 
### Functions:

**HandleSendError Function:**

- **Error Handling:** Handles errors that occur during message sending in the mesh network, including no route, timeout, and general failures.

**Update Function:**

- **OTA Update Process:** Checks for firmware updates, compares the current version with the new version, and performs an OTA update if a new version is available.
    - **Server Upload:** Receives the firmware through server (uploaded by client) and download the firmware.
    - **Update Handling:** Handles the OTA updates, including starting the update, writing the firmware, and validating the update.
        - If firmware update is success, Reboot the board.
        - If firmware update fails, Rollback to the old version (partition schemes).

**HTTP Request Handlers:**

- **handleRoot:** Handles requests to the root URL and responds with a success message.
- **handleData:** Handles data endpoint requests, including OTA update initiation and forwarding messages in the mesh network.
- **handleVersionCheck:** Handles version check endpoint requests and responds with the current firmware version.
- **handleUpdate:** Handles OTA updates for firmware, if it fails rollbacks to original version through the partitions schemes.
  
<br>
   
<br>
   
<br>
   
<br>
 
# Base Node:
  
<br>

 
## Specifications:

**Components:**

- ESP32
- LoRa -sx1278
- Antenna - 433MHz
- BME 280

**Functionality:** 

- Send the received messages to the Central Station
- If time and date is received from the Central Station, it updates its date and time.
- Send message consists of Sensor data and updated date time
- Perform FUOTA (Firmware Update Over The Air) if updated firmware available.
- If any messages received from other nodes, forward to the next node (nearest to destination).

**Location:**

- Present In the flood prone area.
  
<br>
<br>
 
## Code:

```cpp

// COM -
//==========================================================================================================
//                                              L I B R A R I E S                                           |
                                                                                                            
//==========================================================================================================
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <WiFi.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//==========================================================================================================
//                                              I N I T I A L I S A T I O N                                 |
                                                                                                            
//==========================================================================================================
// Task Handles for FreeRTOS tasks
TaskHandle_t Task1;
TaskHandle_t Task2;

// LoRa Parameters
#define RFM95_CS 5    // Chip Select pin for RF95
#define RFM95_RST 14  // Reset pin for RF95
#define RFM95_INT 26  // Interrupt pin for RF95
#define NODE_ID 2     // Unique node ID for this device
#define DEST_NODE_ID 1 // Destination node ID to send messages is Central Station

// Instantiate the RF95 radio and mesh manager
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHMesh manager(rf95, NODE_ID);

// AP (Access Point) Credentials for the ESP32's hotspot
const char* sssid = "ESP32_Hotspot";
const char* passsword = "123456789";

// Current Firmware Version
const String currentVersion = "1.0.0";

// GPS Coordinates (simulated)
float latitude = 19.0000;
float longitude = 18.6000;

// Define I²C pins for BMP280 sensor
#define SDA_PIN 21
#define SCL_PIN 22

// WebServer instance on port 80
WebServer server(80);

// Function declarations
void handleRoot();
void handleData();
void handleVersionCheck();

// Instantiate BMP280 sensor object
Adafruit_BME280 bmp; // I²C

//==========================================================================================================
//                                              S E T U P                                                   |
                                                                                                            
//==========================================================================================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize I²C communication
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the BMP280 sensor
  if (!bmp.begin(0x76)) { // Default I²C address for BMP280 is 0x76
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1); // Halt if sensor initialization fails
  }

  // Reset RF95 module
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize RF95 module
  if (!rf95.init()) {
    Serial.println("RF95 init failed");
    while (1); // Halt if RF95 initialization fails
  }

  // Set RF95 parameters
  rf95.setFrequency(433.0); // Set frequency to 433 MHz
  rf95.setTxPower(5); // Set transmission power

  // Initialize mesh manager
  if (!manager.init()) {
    Serial.println("Mesh init failed");
    while (1); // Halt if mesh initialization fails
  }
  Serial.println("Mesh init successful");

  // Start Wi-Fi AP (Access Point)
  WiFi.softAP(sssid, passsword);
  server.on("/", handleRoot); // Define HTTP endpoint for root
  server.on("/endpoint", handleData); // Define HTTP endpoint for data
  server.on("/version", handleVersionCheck); // Define HTTP endpoint for version check
  server.begin(); // Start the HTTP server
  Serial.println("HTTP server started");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
}

//=========================================================================================================
void loop() {
  // Main loop is empty because all functionality is handled in tasks                                      |
}
//=========================================================================================================

//==========================================================================================================
//                                              C O R E - 0                                                 |
                                                                                                            
//==========================================================================================================

// Task to handle HTTP server requests
void Task1code(void * pvParameters) {
  for (;;) {
    server.handleClient(); // Handle incoming client requests
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for stability
  }
}

//==========================================================================================================
//                                              C O R E - 1                                                 |
                                                                                                            
//==========================================================================================================

// Task to handle mesh network communication
void Task2code(void * pvParameters) {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN]; // Buffer to store incoming messages
  uint8_t len = sizeof(buf);
  uint8_t from, to, id, flags, hops;

  for (;;) {
    // Check if a message is received
    if (manager.recvfromAck(buf, &len, &from, &to, &id, &flags, &hops)) {
      Serial.print("Received message from node "); 
      Serial.print(from);
      Serial.print(" to node "); 
      Serial.print(to);
      Serial.print(": "); 
      Serial.println((char*)buf);

      // If the message is from node 1, process it
      if (from == 1) {
        // Read sensor data from BMP280
        float temperature = bmp.readTemperature();
        float pressure = bmp.readPressure();
    
        // Print the readings to the Serial Monitor
        Serial.print("Temperature = ");
        Serial.print(temperature);
        Serial.println(" *C");
    
        Serial.print("Pressure = ");
        Serial.print(pressure / 100.0F); // Convert to hPa
        Serial.println(" hPa");
    
        // Calculate humidity
        float alti = bmp.readHumidity();
        Serial.print("Approx. Humidity = ");
        Serial.print(alti);
        Serial.println(" %");

        // Extract time from received buffer
        int hr = (buf[0] - '0') * 10 + (buf[1] - '0');
        int mi = (buf[2] - '0') * 10 + (buf[3] - '0');

        // Format the data string to send
        char dat[50];
        //combining into a single string using snprintf = format =>(char *t ,size , format , ...)
        snprintf(dat, sizeof(dat), "%02d%02d%.3f%.3f%.1f%.1f%.1f", hr, mi, latitude, longitude, temperature, pressure, alti);
        Serial.print("Data=");
        Serial.println(dat);

        // Prepare the data for sending
        int dataLength = strlen(dat);
        uint8_t buff[RH_MESH_MAX_MESSAGE_LEN];
        memcpy(buff, dat, dataLength + 1);  // Include null terminator
        len = dataLength + 1;
      
        uint8_t cent = 1; // Central node ID
      
        vTaskDelay(NODE_ID * 1200 / portTICK_PERIOD_MS); // Delay based on node ID
        uint8_t result = manager.sendtoWait(buff, len, cent);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);

        // Check the result of the send operation
        if (result == RH_ROUTER_ERROR_NONE) {
          //result code = 0
          Serial.println("Message forwarded successfully to the next hop.");
        } else {
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          handleSendError(result); 
          
          // Handle send error and retry
          result = manager.sendtoWait(buff, len, cent);
          Serial.println("Message SENDING");
          Serial.print("result=");
          Serial.println(result);
          if (result == RH_ROUTER_ERROR_NONE) {
            //result code = 0
            Serial.println("Message forwarded successfully to the next hop.");
          } else {
            Serial.print("Message send failed, error code: ");
            Serial.println(result);
            handleSendError(result); // Handle send error and retry
          }
        }
      } else {
        // Forward the message to its intended recipient
        uint8_t result = manager.sendtoWait(buf, len, to);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);

        
        //printing the result status
        if (result == RH_ROUTER_ERROR_NONE) {
          //result = 0(code)
          Serial.println("Message forwarded successfully to the next hop.");
          
        } else {
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          handleSendError(result); // Handle send error and retry
          result = manager.sendtoWait(buf, len, to);
          Serial.println("Message SENDING");
          Serial.print("result=");
          Serial.println(result);
          
          //printing the result status
          if (result == RH_ROUTER_ERROR_NONE) {
             //result  = 0 (code)
            Serial.println("Message forwarded successfully to the next hop.");
            
          } else {
            Serial.print("Message send failed, error code: ");
            Serial.println(result);
            handleSendError(result); // Handle send error and retry
          }
        }
      }
    } else {
      Serial.println("No message ");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for stability
  }
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 1                         |
                                                                                                            
//==========================================================================================================
// Function to handle different send errors
void handleSendError(uint8_t result) {
  if (result == RH_ROUTER_ERROR_NO_ROUTE) {
    //result code =1
    Serial.println("Error: No route to destination node.");
  } else if (result == RH_ROUTER_ERROR_TIMEOUT) {
    //result code = 2
    Serial.println("Error: Timeout waiting for ACK.");
  } else {
    Serial.println("Error: General send failure.");
  }
}

void handleUpdate() {
  // Get the current upload status and data
  HTTPUpload& upload = server.upload();

  // If the upload has started
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Update: %s\n", upload.filename.c_str());  // Print the filename of the firmware being uploaded

    // Start the update with an unknown size
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);  // Print any errors that occur during the start of the update
    }
  } 
  // If data is being written to the flash memory
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Write the received buffer to the update and check if the size matches
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);  // Print any errors that occur during writing
    }
  } 
  // If the upload has finished
  else if (upload.status == UPLOAD_FILE_END) {
    // End the update and set the final size
    if (Update.end(true)) {
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);  // Print the total size of the uploaded firmware

      // Check if the update is finished
      if (Update.isFinished()) {
        esp_ota_mark_app_valid_cancel_rollback();  // Mark the update as valid and cancel any rollback
        ESP.restart();  // Restart the ESP to apply the update
      } else {
        Serial.println("Update not finished? Something went wrong!");  // Print an error if the update is not finished
        esp_ota_mark_app_invalid_rollback_and_reboot();  // Mark the update as invalid and rollback
      }
    } else {
      Update.printError(Serial);  // Print any errors that occur during the end of the update
    }
  } 
  // If there is an error with the upload
  else {
    Update.printError(Serial);  // Print the error
  }
}

// HTTP handler for root URL "/"
void handleRoot() {
  Serial.println("Received request...");
  server.send(200, "text/plain", "success");
}

// HTTP handler for "/endpoint" URL
void handleData() {
  if (server.method() == HTTP_POST) {
    String dat = server.arg("data");
    Serial.println("Received data: " + dat);

     

      //conversion of string to unit8_t format
      uint8_t buff[dat.length() + 1];
      //copying the bytes
      dat.getBytes(buff, sizeof(buff));
      uint8_t len = dat.length() + 1;
      uint8_t to = DEST_NODE_ID;

      //Sending
      uint8_t result = manager.sendtoWait(buff, len, to);
      Serial.println("Message SENDING");
      Serial.print("result=");
      Serial.println(result);
      
      //printing the result status
      if (result == RH_ROUTER_ERROR_NONE) {
        Serial.println("Message forwarded successfully to the next hop.");
      } else {
        Serial.print("Message send failed, error code: ");
        Serial.println(result);
        handleSendError(result);
        
        // Handle send error and retry
        result = manager.sendtoWait(buff, len, to);
        Serial.println("Message SENDING");
        Serial.print("result=");
        Serial.println(result);
        //printing the result status
        if (result == RH_ROUTER_ERROR_NONE) {
          
          //result code = 0 
          Serial.println("Message forwarded successfully to the next hop.");
        } 
        else {
          Serial.print("Message send failed, error code: ");
          Serial.println(result);
          handleSendError(result); // Handle send error and retry
        }
      }
    
    //sending the success status code =200
    server.send(200, "text/plain", "Data received by ESP32");
  }
}

// HTTP handler for "/version" URL
void handleVersionCheck() {
  server.send(200, "text/plain", currentVersion); // Send current version as response
}

//==========================================================================================================
//                                         F U N C T I O N S    F O R   C O R E - 0                         |
//                                                                                                          |
//==========================================================================================================

//                                                NO specific functions
```
  
<br>
 
<br>
 
## Explanation:
  
 
### **Libraries:**

- **RFM95 and BME280 Libraries:** Includes libraries for mesh networking (RHMesh, RH_RF95) and sensor communication (Adafruit BME 280 ,Adafruit Unified Sensor).
- **ESP32 Libraries:** Includes libraries for Wi-Fi, HTTP, OTA updates, and ESP32 specific operations (WiFi, HTTPClient, Update, esp_ota_ops, WebServer, Arduino).
  
<br>
 
### **Initialization:**

- **Pin Definitions:** Defines the pins used for the RFM95 module (chip select, reset, interrupt), GPS module (RX, TX), and serial monitor.
- **Node IDs:** Sets the unique ID for the current node and the destination node ID for the base station.
- **Sensor Pin Definitions:** Defines the SDA and SCK pins of BME280.
- **Mesh Network:** Initializes the RF driver and mesh manager with the node ID.
- **Wi-Fi Credentials:** Sets Wi-Fi credentials for connection and access point.
- **Web Server:** Creates a WebServer instance on port 80.
- **FreeRTOS Tasks:** Task handles for managing FreeRTOS tasks.
  
<br>
 
### **Setup Function:**

- **Serial Communication:** Initializes serial communication for debugging and GPS.
- **RFM95 Initialization:** Resets and initializes the RFM95 module, setting its frequency to 433 MHz and transmission power.
- **Mesh Network Initialization:** Initializes the mesh network manager.
- **Wi-Fi Access Point:** Starts the Wi-Fi access point and sets up HTTP server routes for handling requests.
- **FreeRTOS Tasks:** Creates and pins two FreeRTOS tasks (Task1code and Task2code) to specific cores for concurrent execution.
  
<br>
 
### **Main Loop:**

- The main loop is empty because the code uses FreeRTOS tasks to handle operations concurrently.
  
<br>
 
### **Task1code (Core 0):**

- **HTTP Server Handling:** Handles HTTP client requests.
    - **Request Message:** Received requests are converted into message formats and send to the Base Station.
    - **OTA update:** If update request is received it calls the *“handleUpdate”* function*.*
  
<br>
 
### **Task2code (Core 1):**

- **Mesh Network Communication:** Receives and processes messages from the mesh network.
    - **Message Handling:** Processes messages from the central station, updates the time, and sensor data back to the Base station.
    - **Message Forwarding:** Forwards messages to the destination node.
  
<br>
 
### Functions:

**HandleSendError Function:**

- **Error Handling:** Handles errors that occur during message sending in the mesh network, including no route, timeout, and general failures.

**handleUpdate Function:**

- **OTA Update Process:** Checks for firmware updates, compares the current version with the new version, and performs an OTA update if a new version is available.
    - **Server Upload:** Receives the firmware from server uploaded by client (mobile) and download the firmware.
    - **Update Handling:** Handles the OTA updates, including starting the update, writing the firmware, and validating the update.
        - If firmware update is success, Reboot the board.
        - If firmware update fails, Rollback to the old version (partition schemes).

**HTTP Request Handlers:**

- **handleRoot:** Handles requests to the root URL and responds with a success message.
- **handleData:** Handles data endpoint requests, including OTA update initiation and forwarding messages in the mesh network.
- **handleVersionCheck:** Handles version check endpoint requests and responds with the current firmware version.
- **handleUpdate:** Handles OTA updates for firmware, if it fails rollbacks to original version through the partitions schemes
