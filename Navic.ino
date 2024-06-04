#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX D5
#define ARDUINO_GPS_TX D6
#define GPS_BAUD_RATE 9600
#define SerialMonitor Serial
SoftwareSerial gpsPort(ARDUINO_GPS_TX, ARDUINO_GPS_RX);
TinyGPSPlus gps;
void setup()
{
gpsPort.begin(GPS_BAUD_RATE);
SerialMonitor.begin(9600);
}
void loop() {
//Getting val
while (gpsPort.available() > 0) {
char c = gpsPort.read();
Serial.write(c);
gps.encode(c);
}
// adding to gps (not necessary but as per snippet . May be useful Later)
while (Serial.available() > 0) {
gpsPort.write(Serial.read());
}
//updt
if (gps.location.isUpdated()) {
Serial.println("L");
Serial.print("Latitude: ");
Serial.println(gps.location.lat(), 6);
Serial.print("Longitude: ");
Serial.println(gps.location.lng(), 6);
}
}