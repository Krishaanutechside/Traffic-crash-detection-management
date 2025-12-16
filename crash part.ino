#include <Wire.h>

const int MPU = 0x68;    // MPU6050 I2C address
const int buzzerPin = 8; // Buzzer on D8
bool crashDetected = false;
unsigned long crashTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Wake up MPU6050 (disable sleep mode)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("MPU6050 Crash Detector Ready");
}

void loop() {
  int16_t AcX, AcY, AcZ;

  // Read accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Convert to g-force
  float Ax = AcX / 16384.0;
  float Ay = AcY / 16384.0;
  float Az = AcZ / 16384.0;
  float totalAccel = sqrt(Ax * Ax + Ay * Ay + Az * Az);

  Serial.println(totalAccel);

  // ✅ Sensitivity tuned for small toy crash (Hot Wheels impact)
  // Ignore small servo vibrations (< 2g)
  if (totalAccel > 1.2 && !crashDetected) {
    crashDetected = true;
    crashTime = millis();
    Serial.println("🚨 Crash Detected!");
    digitalWrite(buzzerPin, HIGH);
  }

  // ⏱ Buzzer ON for 3 seconds only
  if (crashDetected && millis() - crashTime >= 3000) {
    digitalWrite(buzzerPin, LOW);
    crashDetected = false;
  }

  delay(100);
}
