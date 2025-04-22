#include <Wire.h>
#include <QMC5883LCompass.h>

/* ---------- Motor pins (L298N) ---------- */
#define ENA  12   // PWM left
#define ENB  14   // PWM right
#define IN1  26
#define IN2  27
#define IN3  32
#define IN4  33

const int PWM_FREQ = 20000;     // 20 kHz
const int PWM_RES  = 8;         // 8‑bit (0‑255)

QMC5883LCompass compass;

/* ---------- PID gains (tune) ---------- */
float Kp = 2.5, Ki = 0.05, Kd = 0.5;

/* ---------- Runtime vars ---------- */
float setpoint, integralTerm = 0, lastError = 0;
uint32_t lastPIDTime = 0, startRunTime = 0;

const uint8_t BASE_SPEED = 180;        // fwd speed
const uint8_t MAX_OUTPUT = 100;        // diff limit

/* ---------- Helper ---------- */
void setMotorPWM(uint8_t pin, uint8_t duty) {
  analogWrite(pin, constrain(duty, 0, 255));
}

void driveMotors(int16_t left, int16_t right)
{
  /* Left */
  if (left >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); left = -left; }
  setMotorPWM(ENA, left);

  /* Right */
  if (right >= 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); right = -right; }
  setMotorPWM(ENB, right);
}

float readHeading()
{
  compass.read();
  float h = atan2(compass.getY(), compass.getX()) * 180.0 / PI;
  if (h < 0) h += 360;
  return h;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();  Wire.setClock(400000);

  /* Compass */
  compass.init();
  compass.setMode(0x01, 0x0C, 0x00, 0xC0);   // 200 Hz
  compass.setSmoothing(3, false);

  /* Motor pins */
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  /* --- PWM via analogWrite --- */
  analogWriteFrequency(ENA, PWM_FREQ);
  analogWriteFrequency(ENB, PWM_FREQ);
  // analogWriteResolution(PWM_RES);   // ESP32 default 8 bit – gọi nếu muốn đổi toàn cục

  /* Heading setpoint = initial + 30° */
  delay(500);
  float initH = readHeading();
  setpoint = fmod(initH + 30.0, 360.0);

  Serial.printf("Init: %.2f°  Setpoint: %.2f°\n", initH, setpoint);
  Serial.println("Wait 3 s…");
  delay(3000);

  startRunTime = lastPIDTime = millis();
}

void loop()
{
  if (millis() - startRunTime >= 10000UL) {      // stop after 10 s
    driveMotors(0, 0);  while (true);
  }

  uint32_t now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt < 0.02) return;                         // 50 Hz loop
  lastPIDTime = now;

  float heading = readHeading();
  float error = setpoint - heading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integralTerm += error * dt;
  float derivative = (error - lastError) / dt;
  float output = Kp*error + Ki*integralTerm + Kd*derivative;
  lastError = error;

  output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);
  int16_t left  = BASE_SPEED + output;
  int16_t right = BASE_SPEED - output;
  driveMotors(left, right * 1.35);

  Serial.printf("H=%.1f Err=%.1f L=%d R=%d\n", heading, error, left, right);
}
