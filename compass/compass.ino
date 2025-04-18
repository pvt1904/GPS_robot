#include <QMC5883LCompass.h>

QMC5883LCompass compass;
//SCL pin 22
//SDA pin 21
void setup() {
  Serial.begin(115200);
  compass.init();
  
  // Apply smoothing for more stable readings
  compass.setSmoothing(10, true);  
}

void loop() {
  // Read compass values
  compass.read();

  // Get XYZ magnetometer readings
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  // Compute heading angle (in degrees)
  float heading = atan2(y, x) * 180.0 / PI;

  // Ensure heading is in range [0, 360]
  if (heading < 0) {
    heading += 360;
  }

  // Print the values
  Serial.print("X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.print(y);
  Serial.print("  Z: "); Serial.print(z);
  Serial.print("  Heading: "); Serial.print(heading);
  Serial.println("°");

  delay(250);
}
