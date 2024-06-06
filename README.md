# Esp32:

![Untitled](https://github.com/DharunAP/LoRa-Networking/assets/123437101/b1247f22-10f6-46e1-9ae6-01b87658003b)

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

  # **Pin Diagram:**
![Untitled (1)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/62f7973f-f47e-41e7-8ed9-0771eb21d187)

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
# ESP32 - LORA :

### LoRa (Long Range):

![Untitled (3)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/3defc863-1a3b-455f-abc5-dfff57fc3c3f)

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

# ESP32 - BMP280:

## BMP280:

![Untitled (4)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/beee8415-939e-4722-9e22-35355fb6a7b8)

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

## Pin Configurations :

![Untitled (5)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/f6ee9ef8-1dec-4049-b9e6-c1fb182a84a6)

**BMP280**                 **ESP32** 

VCC              -          3.3v

GND             -          GND

SDA              -          D21

SCK               -          D22

# ESP32 - BME280

## BME 280:

![Untitled (6)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/42ee76e8-07f0-40bd-8ef5-719db00180d9)

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

Pin Configuration:

![Untitled (7)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/b2e04d3c-333a-4290-8e71-6ab23f275f7b)


**BMP280**                 **ESP32** 

VCC              -          3.3v

GND             -          GND

SDA              -          D21

SCK               -          D22



# ESP32 - L89 :

![Untitled (8)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/06a7e3f7-622e-4006-8ad4-b28e1759ad41)

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

## Pin Configurations:

![Untitled](https://prod-files-s![Untitled (9)](https://github.com/DharunAP/LoRa-Networking/assets/123437101/2a0962b9-a3a6-40c5-ba16-538216db7401)

**L89**                          **ESP32** 

VCC              -          3.3v

GND             -          GND

RX                -          D12 / D16

TX                -          D13 / D17


