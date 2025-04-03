#include <SoftwareSerial.h>

SoftwareSerial Dta(4, 3);

void setup() {
    Serial.begin(9600);
    Dta.begin(9600);
}

void loop() {
    while (Dta.available() > 0) {
        byte gpsData = Dta.read(); // You can get raw data from GPS module
        Serial.write(gpsData);
    }
}
