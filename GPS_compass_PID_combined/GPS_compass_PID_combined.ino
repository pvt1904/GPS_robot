#include <Wire.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// =================== CONFIG ===================
#define ENA  12  // Left motor PWM
#define ENB  14  // Right motor PWM
#define IN1  26
#define IN2  27
#define IN3  32
#define IN4  33

#define PWM_FREQ 20000
#define PWM_RES  8

const uint8_t BASE_SPEED = 180;
const uint8_t MAX_OUTPUT = 50;

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_SERIAL_NUM 2
#define GPS_BAUD_RATE 9600

const double TARGET_LAT = 10.77250;     // Target coordinates
const double TARGET_LON = 106.65891;
const double ARRIVAL_THRESHOLD_METERS = 3.0; // GPS accuracy: 1-5 meters under open sky conditions

// PID Constants
float Kp = 2.5, Ki = 0.03, Kd = 0.08;
float integralTerm = 0, lastError = 0;
uint32_t lastPIDTime = 0;

// =================== OBJECTS ===================
QMC5883LCompass compass;
TinyGPSPlus gps;
HardwareSerial GPS_serial(GPS_SERIAL_NUM);

// =================== FUNCTIONS ===================
// =================== MOTOR ===================
void motorSetup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWriteFrequency(PWM_FREQ);
}

void setMotorPWM(uint8_t pin, uint8_t duty) {
  static bool pwm_initialized = false;
  if (!pwm_initialized) {
    ledcAttachPin(ENA, 0);  // Channel 0
    ledcAttachPin(ENB, 1);  // Channel 1
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    pwm_initialized = true;
  }

  if (pin == ENA) ledcWrite(0, duty);
  else if (pin == ENB) ledcWrite(1, duty);
}

void driveMotors(int16_t left, int16_t right) {
  if (left >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); left = -left; }
  setMotorPWM(ENA, left);

  if (right >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); right = -right; }
  setMotorPWM(ENB, right);
}

// =================== COMPASS ===================
void compassSetup() {
  Wire.begin();
  Wire.setClock(400000);
  compass.init();
  compass.setMode(0x01, 0x0C, 0x00, 0xC0);
  compass.setSmoothing(3, false);
}

float compassRead() {
  compass.read();
  float heading = atan2(compass.getY(), compass.getX()) * 180.0 / PI;
  return (heading < 0) ? heading + 360 : heading;
}

// =================== GPS ===================
void GPS_setup() {
  GPS_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

bool GPS_read(double &lat, double &lon, int &sat) {
  while (GPS_serial.available()) gps.encode(GPS_serial.read());

  if (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    sat = gps.satellites.value();
    return true;
  }

  sat = gps.satellites.value();
  return false;
}

float haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

float bearingToTarget(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) -
            sin(lat1) * cos(lat2) * cos(dLon);
  float bearing = atan2(y, x) * 180.0 / PI;
  return fmod(bearing + 360.0, 360.0);
}

// =================== PID ===================
float updatePID(float error) {
  uint32_t now = millis();
  float dt = max((now - lastPIDTime) / 1000.0, 0.01);
  lastPIDTime = now;

  integralTerm += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  return Kp * error + Ki * integralTerm + Kd * derivative;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  motorSetup();
  compassSetup();
  GPS_setup();
  delay(3000);
}

// =================== LOOP ===================
void loop() {
  double lat, lon;
  int sats;

  if (GPS_read(lat, lon, sats)) {
    float dist = haversineDistance(lat, lon, TARGET_LAT, TARGET_LON);
    Serial.printf("Sat: %d | Dist: %.2f m\n", sats, dist);

    if (dist < ARRIVAL_THRESHOLD_METERS) {
      Serial.println("🎯 Destination reached.");
      driveMotors(0, 0);
      while (1);
    }

    float currentHeading = compassRead();
    float targetHeading = bearingToTarget(lat, lon, TARGET_LAT, TARGET_LON);

    float error = currentHeading - targetHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    float pidOutput = updatePID(error);
    pidOutput = constrain(pidOutput, -MAX_OUTPUT, MAX_OUTPUT);

    int leftMotor = BASE_SPEED - pidOutput;
    int rightMotor = BASE_SPEED + pidOutput;

    driveMotors(leftMotor, rightMotor);

    Serial.printf("Heading: %.1f | Target: %.1f | Error: %.1f | PID: %.1f\n",
                  currentHeading, targetHeading, error, pidOutput);
  }

  delay(100);
}
