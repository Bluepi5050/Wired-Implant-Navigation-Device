#include <Wire.h>
#include "MPU6050.h"
// *IMPORTANT*
/*

- Used variable resistor instead of button
  - Threshold is 100 ==> if the analogRead goes over 100, it is the same as "button pressed"
- Converted MPU sensor values into "degrees" (ex. 10 degrees, 20 degrees, 45 degrees...)

*/

// ---------- User settings ----------
const float  ALPHA          = 0.98f;     // Complementary filter weight (gyro)
const uint16_t CALIB_SAMPLES = 1000;     // Gyro bias samples at startup
const float  GYRO_SENS      = 131.0f;    // LSB per deg/s at ±250 dps
const float  ACC_SENS       = 16384.0f;  // LSB per g at ±2g

// ---------- Sensor struct ----------
struct IMUData {
  MPU6050 mpu;
  int16_t ax, ay, az, gx, gy, gz;   // raw
  float   gbx, gby;                 // gyro bias (deg/s)
  float   roll, pitch;              // filtered angles (deg)
};

IMUData imu1 = {MPU6050(0x68)};
IMUData imu2 = {MPU6050(0x69)};

unsigned long prevMicros;

// ---------- Function prototypes ----------
void initMPU(IMUData &imu, const char* label);
void calibrateGyro(IMUData &imu);
void primeAnglesFromAccel(IMUData &imu);
void updateAngles(IMUData &imu, float dt);

// ================================================================

float roll_diff, pitch_diff;
float roll_diff_B, pitch_diff_B;
int isPressed = 0;
int PHOTO = A0;
int LED_PIN = 7;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast-mode I2C

  initMPU(imu1, "MPU6050 #1");
  initMPU(imu2, "MPU6050 #2");

  delay(500); // settle

  calibrateGyro(imu1);
  calibrateGyro(imu2);

  // Initialize roll/pitch from accel (one read each, no double-reads later)
  primeAnglesFromAccel(imu1);
  primeAnglesFromAccel(imu2);

  prevMicros = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;

  updateAngles(imu1, dt);
  updateAngles(imu2, dt);

  Serial.print("MPU1 (deg)  R:"); Serial.print(imu1.roll, 2);
  Serial.print("  P:");           Serial.print(imu1.pitch, 2);
  Serial.print("   |   MPU2 (deg)  R:"); Serial.print(imu2.roll, 2);
  Serial.print("  P:");                   Serial.println(imu2.pitch, 2);

  delay(10);

  roll_diff = imu1.roll - imu2.roll;
  pitch_diff = imu1.pitch - imu2.pitch;

  if (analogRead(PHOTO) > 100 && isPressed == 0) {

    isPressed = 1;
    roll_diff_B = roll_diff;
    pitch_diff_B = pitch_diff;

  } else if (isPressed == 1) {

    if (abs(roll_diff - roll_diff_B) > 5.00 || abs(pitch_diff - pitch_diff_B) > 5.00) {
          digitalWrite(LED_PIN, HIGH);
        } else {
          digitalWrite(LED_PIN, LOW);
        }

  }

}

// ================================================================
// Initialize MPU settings
void initMPU(IMUData &imu, const char* label) {
  imu.mpu.initialize();

  if (imu.mpu.testConnection()) {
    Serial.print(label); Serial.println(" connected.");
  } else {
    Serial.print(label); Serial.println(" connection FAILED!");
  }

  imu.mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  imu.mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  imu.mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  imu.mpu.setRate(4); // 1kHz/(1+4) = 200 Hz internal; loop uses dt
}

// Gyro bias calibration (keep sensor still)
void calibrateGyro(IMUData &imu) {
  long sx = 0, sy = 0;
  Serial.println("Calibrating gyro... Keep the sensor still.");
  delay(300);

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    imu.mpu.getMotion6(&imu.ax,&imu.ay,&imu.az,&imu.gx,&imu.gy,&imu.gz);
    sx += imu.gx; sy += imu.gy;
    delay(2);
  }
  imu.gbx = (sx / (float)CALIB_SAMPLES) / GYRO_SENS;
  imu.gby = (sy / (float)CALIB_SAMPLES) / GYRO_SENS;

  Serial.print("Gyro bias (deg/s): ");
  Serial.print(imu.gbx,3); Serial.print(", ");
  Serial.println(imu.gby,3);
}

// One-time accel-based angle prime (single read)
void primeAnglesFromAccel(IMUData &imu) {
  imu.mpu.getMotion6(&imu.ax,&imu.ay,&imu.az,&imu.gx,&imu.gy,&imu.gz);

  float axg = imu.ax / ACC_SENS;
  float ayg = imu.ay / ACC_SENS;
  float azg = imu.az / ACC_SENS;

  imu.roll  = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  imu.pitch = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
}

// Single-read update: read once, integrate gyro, compute accel tilt, blend
void updateAngles(IMUData &imu, float dt) {
  // Single I2C read for this update
  imu.mpu.getMotion6(&imu.ax,&imu.ay,&imu.az,&imu.gx,&imu.gy,&imu.gz);

  // Gyro (deg/s) minus bias
  float gxr = (imu.gx / GYRO_SENS) - imu.gbx;
  float gyr = (imu.gy / GYRO_SENS) - imu.gby;

  // Predict using gyro integration
  float roll_gyro  = imu.roll  + gxr * dt;
  float pitch_gyro = imu.pitch + gyr * dt;

  // Accel tilt from the same read
  float axg = imu.ax / ACC_SENS;
  float ayg = imu.ay / ACC_SENS;
  float azg = imu.az / ACC_SENS;
  float roll_acc  = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;

  // Complementary filter
  imu.roll  = ALPHA * roll_gyro  + (1.0f - ALPHA) * roll_acc;
  imu.pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_acc;
}
