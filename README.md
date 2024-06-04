LoRa Mesh Network
It comprised of totally three parts of network.
- Client Nodes
- Master client
-
- ByPass Nodes (Base Station)
- Central Nodes
Network Workflow:
§ Network model : Mesh Network
§ Base Station Node act as mesh network towers.
§ The targeted area is split into grids of square.
§ Each base station planted in each grid.
§ When a message is received from one node.
§ That receiver node sends back the message to
nearer node with respect to the central station
§ The nodes will continuously passes the data until the
data received to the Central station.
Here is an improved version of your description with errors corrected and
more clarity added:
Time Synchronization for LoRa Communication
Key Points:
- Single Message Reception: LoRa cannot receive multiple messages
simultaneously.
- Payload Size: Limited to 255 bytes.
- Data Loss: Multiple messages can lead to data loss; synchronization
prevents this.
- Allotted Time Slots: Each node has a specific time to send messages,
reducing latency and preventing data loss.
- RTC Module: Each base station uses an RTC module on the ESP32 to
manage time slots.
- No Data Traffic: Only one node transmits at a time, avoiding traffic and
latency issues.
Node Details
Client Nodes:
- Connection: Connect to the LoRa node via QR code using WiFi (no
internet).
- Mobile App Interface:
- Programmed in React Native.
- Provides a user-friendly interface with dropdown lists and text areas
for emergency information.
Operation:
- Sends user requirements to the master client LoRa.
- Portable and operates on channels 435, 437, 439 MHz.
Master Client Node:
- Storage Function: Acts as a reservoir for client requests.
- Synchronization Check: Sends an initiating message to verify
synchronization.
- Message Transmission: Sends client messages to the Parent Base
Station.
- WiFi Messaging: Also allows WiFi messages from users without client
nodes.
- Channel Operation:
- Receives on client channels.
- Sends on base station channels.
Base Station:
- Function: Sends and receives messages.
- Operation: Relays messages to the nearest LoRa base node.
- Channels: Operates on 441, 443, 445 MHz.
Central Station:
- Final Receiver: Acts as a bridge between the database and network
module.
- Database Update: Updates received messages in the database.
Methods Used
Addressing Encoding:
- Encryption: Each node has a unique address key. Messages are
verified by matching addresses, allowing only the intended node to
process the message.
Defective Checking:
- Decoy Messages: Sent before original messages to detect defective
nodes. If a node is defective, it switches to a nearby channel.
Black Box Mode:
- Message Storage: If no nearby non-defective nodes are available, the
node stores messages in EEPROM. It acts as a black box, storing all
received messages for later retrieval.
This structured approach ensures efficient and reliable communication in
the network, reducing the chance of data loss and improving overall
system performance.
Proof for the flood detection parameter :
Flood detection parameters :
https://www.researchgate.net/publication/351295316_A_Study_on_Iot_B
ased_Flood_Detection_Management_System
Based on research, here are specific parameter values indicating flood
conditions:
1. Temperature: Research suggests that temperatures above 30°C
(86°F) can lead to increased moisture in the air, contributing to heavy
rainfall and potential flooding.
2. Humidity: Relative humidity levels above 80% are commonly
associated with conditions that can lead to flooding due to the high
moisture content in the atmosphere.
3. Atmospheric Pressure: Low atmospheric pressure, particularly below
1000 hPa, often precedes severe weather events, including heavy
rainfall that can cause flooding .
4. Light Intensity: Significant reductions in light intensity due to thick
cloud cover can be observed during heavy rain events. However,
specific values are less commonly cited; instead, a noticeable drop from
normal daylight levels is a general indicator .
These values are typically used in conjunction to predict and monitor
flood conditions. You can access the full research papers for more
detailed insights and data:
- Algorithm to Predict the Rainfall Starting Point as a Function of
Atmospheric Pressure, Humidity, and Dewpoint
[MDPI](https://www.mdpi.com/2225-1154/8/11/120)
- Real-Time Monitoring of Environmental Parameters Using
IoT[Springer](https://link.springer.com/article/10.1007/springerlink:10.100
7/s00170-018-1921-3)
Pressure : significant drop of 10hpa
Humidity : above 80%
If we need more precise values , we can use rain depth sensor.
Working on new header file for mesh
Non adaptive mesh headers :
Painless mesh :
· Only compatible for the esp 32 with WiFi connection .Cant use the
LoRa connectivity.
Mechastic : Not used for normal boards
● Adaptive to : LILYGO® TTGO T-Beam (>V1.1 recommended)
● LILYGO® TTGO Lora (>V2.1 recommended)
● Nano G1
● Station G1
● Heltec V3 and Wireless Stick Lite V3
● RAK11200 Core module for RAK WisBlock modular boards
Hackster IO :
Not for ESp32 or esp8266
Only for :
· LILYGO TTGO LoRa32 T3
· Heltec WFi LoRa 32.
MeshCom 4.0 :
· Not a header , it’s a firmware .
· Uses Command line for coding .
· Not enough documentation .
Adaptive Header File For MESH : RHMesh
Best features :
Adaptive : adaptive for Esp32 and Esp8266
Auto Route finding : Finds route automatically
Updating Route table : new node will updated in an instant
New route : If a route is failed , a new route will be found in the next
iteration
Broadcast storms : Has less storms and data traffic .
Cons :
Latency is high compared to normal transmission of signals due to
internal route finding and updating route table.
Not enough documentation or sources .
Testing : the modules are able to find the next id with no problem. Since
lora range is high , I cant able to test its features well.
It is trusted by many other programmeers and used as a layer to create
other mesh libraries. So its features are achieved by it.

NAVIC module :
● Its used for the gps tracking in Marine navigations.
● Its a GNSS module : GNSS: A GNSS receiver can use signals from
any positioning satellite, not just those in the GPS system. This
flexibility makes GNSS receivers more accurate and reliable than
GPS technology alone.
Pros :
Higher accuracy
3D dimensional mapping
GSV - tells no of satellites
Parameters :
● Altitude
● Latitude
● Longitude
● GSV
● Date
● Time
● ,
Problems faced :
● While using software serial i tried to connect to pins of TX2 - D8 , RX2 - D7 . It
doesnt print values .
● So it means that we can only use the non tx and rx pins for the software
serials.
● Always get a external antenna of the configured frequency and monitor in a
open sky place.
● Compatible with any Boards.
● I used the Tiny gps to convert the values from NMEA to Human readable
format.
