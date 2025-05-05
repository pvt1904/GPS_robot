#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;


void compass_setup(){
  Wire.begin();
  Wire.setClock(400000);
  compass.init();
  compass.setMode(0x01, 0x0C, 0x00, 0xC0);   // Continuous | 200 Hz | 2 g | OSR 64
  compass.setSmoothing(3, false);            // smooth data
}

float heading;
void compass_read() {
  compass.read();
  heading = atan2(compass.getY(), compass.getX()) * 180.0 / PI;
  if (heading < 0) heading += 360;
}

void setup() {

  Serial.begin(115200);

}

void loop() {
  Serial.printf("Heading: %.1f°\n", heading);
  delay(100);  
}