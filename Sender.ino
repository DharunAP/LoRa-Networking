// SENDER WHITE WHITE D2
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#define RFM95_CS 15 // D8
#define RFM95_RST 16 // D0
#define RFM95_INT 4 // D2
// Unique node ID for this node
#define NODE_ID 1
// Destination node ID
#define DEST_NODE_ID 3
// Initialize the RF driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// Initialize the mesh manager
RHMesh manager(rf95, NODE_ID);
void setup() {
    Serial.begin(115200);
    // Reset the module (if required)
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    if (!rf95.init()) {
        Serial.println("RF95 init failed");
        while (1);
    }
    rf95.setFrequency(433.0); // Set to 915 MHz or your desired frequency
    if (!manager.init()) {
        Serial.println("Mesh init failed");
        while (1);
    }
    Serial.println("Mesh init successful");
}
void loop() {
    uint8_t data[] = "Hello from node 1";
    uint8_t len = sizeof(data);
    Serial.println("Attempting to send message...");
    // Send the message to the destination node
    uint8_t result = manager.sendtoWait(data, len, DEST_NODE_ID);
    if (result == RH_ROUTER_ERROR_NONE) {
        Serial.println("Message sent successfully");
    } else {
        Serial.print("Message send failed, error code: ");
        Serial.println(result);
        // Handle specific error cases
        if (result == RH_ROUTER_ERROR_NO_ROUTE) {
            Serial.println("Error: No route to destination node.");
        } else if (result == RH_ROUTER_ERROR_TIMEOUT) {
            Serial.println("Error: Timeout waiting for ACK.");
        } else {
            Serial.println("Error: General send failure.");
        }
    }
    //--------------------------------------------------------------------------------------
    // Print routing table for the desired destination node
    Serial.println("Routing table:");
    RHMesh::RoutingTableEntry entry;
    if (manager.getRouteTo(DEST_NODE_ID)) {
        Serial.print("Dest: ");
        Serial.print(entry.dest);
        Serial.print(" Next hop: ");
        Serial.print(entry.next_hop);
        Serial.print(" State: ");
        Serial.println(entry.state);
    }
    // Add a delay to avoid flooding the network
    delay(5000);
}