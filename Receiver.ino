// RECVR GREEN D0 GREEN GND
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
// Define pins for your RF module
#define RFM95_CS 15
#define RFM95_RST 16
#define RFM95_INT 4
#define DEST_NODE_ID 3
// Unique node ID for this node
#define NODE_ID 3
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
    uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    uint8_t to;
    uint8_t id;
    uint8_t flags;
    uint8_t hops;
    // Check for received messages
    if (manager.recvfromAck(buf, &len, &from, &to, &id, &flags, &hops)) {
        Serial.print("Received message from node ");
        Serial.print(from);
        Serial.print(" to node ");
        Serial.print(to);
        Serial.print(": ");
        Serial.println((char*)buf);
    } else {
        Serial.println("No message received");
    }
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
    // Add a delay to avoid flooding the serial output
    delay(1000);
}
