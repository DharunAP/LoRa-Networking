

// COM -
//==========================================================================================================
//                                              L I B R A R I E S                                           |
//                                                                                                          |
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
#include <Adafruit_BMP280.h>



//==========================================================================================================
//                                              I N I T I A L I S A T I O N                                 |
//                                                                                                          |
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
Adafruit_BMP280 bmp; // I²C




//==========================================================================================================
//                                              S E T U P                                                   |
//                                                                                                          |
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
//                                                                                                          |
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
//                                                                                                          |
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
    
        // Optional: Calculate altitude based on sea level pressure (1013.25 hPa)
        float alti = bmp.readAltitude(1013.25);
        Serial.print("Approx. Altitude = ");
        Serial.print(alti);
        Serial.println(" m");

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
//                                                                                                          |
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
