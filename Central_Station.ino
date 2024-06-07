// COM -13
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
  
  // Print different time formats
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // Other prints omitted for brevity

  // Extract day, month, year, minute, and second as strings
  char timeDay[3], timeMonth[3], timeYear[5], timeMin[3], timeSec[3];
  strftime(timeDay, 3, "%d", &timeinfo);
  strftime(timeMonth, 3, "%m", &timeinfo);
  strftime(timeYear, 5, "%Y", &timeinfo);
  strftime(timeMin, 3, "%M", &timeinfo);
  strftime(timeSec, 3, "%S", &timeinfo);

  Serial.print("Day: ");
  Serial.println(timeDay);
  Serial.print("Month: ");
  Serial.println(timeMonth);
  Serial.print("Year: ");
  Serial.println(timeYear);
  Serial.print("Minute: ");
  Serial.println(timeMin);
  Serial.print("Second: ");
  Serial.println(timeSec);

  // Convert seconds to integer
  int sec = atoi(timeSec);
  Serial.print("Sec ----->");
  Serial.println(sec);

  // Check if the seconds are equal to 15, 30, 45, 0, or 20
  if (sec == 15 || sec == 30 || sec == 45 || sec == 0 || sec == 20) {
    Serial.println("The second is either 15, 30, 45, 0, or 20");

    // Construct TimeData string with date and time in numeric format
    TimeData = "";
    TimeData += timeMin[0];
    TimeData += timeMin[1];
    TimeData += timeSec[0];
    TimeData += timeSec[1];
    TimeData += timeDay[0];
    TimeData += timeDay[1];
    TimeData += timeMonth[0];
    TimeData += timeMonth[1];
    TimeData += timeYear[2];
    TimeData += timeYear[3];

    // Log the constructed TimeData
    Serial.print("TimeData=");
    Serial.println(TimeData);

    // Prepare the message for sending
    uint8_t buff[TimeData.length() + 1];
    TimeData.getBytes(buff, sizeof(buff));
    uint8_t len = TimeData.length() + 1;

    // Send the message to other nodes
    uint8_t result = manager.sendtoWait(buff, len, RH_BROADCAST_ADDRESS);
    Serial.println("Message SENDING");
    Serial.print("Result=");
    Serial.println(result);

    // Handle send errors and retry if necessary
    if (result != RH_ROUTER_ERROR_NONE) {
      handleSendError(result);  // Log the error
      vTaskDelay(SEND_RETRY_DELAY / portTICK_PERIOD_MS);  // Delay before retrying
      esp_task_wdt_reset();  // Feed the watchdog timer

      // Sending message again
      result = manager.sendtoWait(buff, len, RH_BROADCAST_ADDRESS);
      Serial.println("Retrying Message SENDING");
      Serial.print("Result=");
      Serial.println(result);

      // If the message is not successful
      if (result != RH_ROUTER_ERROR_NONE) {
        handleSendError(result);  // Log the error
      } else {
        // If result code is equal to RH_ROUTER_ERROR_NONE (it means 0)
        Serial.println("Message forwarded successfully to the next hop.");
      }
    } else {
      // If result code is equal to RH_ROUTER_ERROR_NONE (it means 0)
      Serial.println("Message forwarded successfully to the next hop.");
    }
  } else {
    // If second is not equal to 15, 30, 45, 0, or 20
    Serial.println("The second is not 0, 20, 15, 30, or 45.");
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
