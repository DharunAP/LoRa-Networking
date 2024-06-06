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
