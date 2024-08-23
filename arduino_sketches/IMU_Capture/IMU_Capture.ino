// #include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LSM6DS33.h>
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

// #if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
//   Adafruit_Sensor_Calibration_EEPROM cal;
// #else
//   Adafruit_Sensor_Calibration_SDFat cal;
// #endif

const int numSamples = 104;

int samplesRead = numSamples;
int total = 0;

float gyro_offset[3];
const float bias_vector[3] = {-0.08, -0.16, 0.47};
const float correction_matrix[9] = {1.02248, 0.00633, -0.00331,
                                  0.00633, 1.01480, 0.00440, 
                                  -0.00331, 0.00440, 1.01081};

void calibrateGyro() {
  Serial.println("Calculating gyro offsets...");
  delay(1000);
  float min_x, max_x, mid_x;
  float min_y, max_y, mid_y;
  float min_z, max_z, mid_z;
  float x, y, z;
  sensors_event_t gyro;
  if (lsm6ds.gyroscopeAvailable()) {
    gyroscope->getEvent(&gyro);
    min_x = max_x = gyro.gyro.x;
    min_y = max_y = gyro.gyro.y;
    min_z = max_z = gyro.gyro.z;
  }
  delay(10);
  for (int i=0; i<1000; i++) {
    if (lsm6ds.gyroscopeAvailable()) {
      gyroscope->getEvent(&gyro);
      x = gyro.gyro.x;
      y = gyro.gyro.y;
      z = gyro.gyro.z;

      min_x = min(min_x, x);
      min_y = min(min_y, y);
      min_z = min(min_z, z);
    
      max_x = max(max_x, x);
      max_y = max(max_y, y);
      max_z = max(max_z, z);
    
      mid_x = (max_x + min_x) / 2;
      mid_y = (max_y + min_y) / 2;
      mid_z = (max_z + min_z) / 2; 
    }
  }
  gyro_offset[0] = mid_x;
  gyro_offset[1] = mid_y;
  gyro_offset[2] = mid_z;

  for (int i=0; i<3; i++) {
    Serial.println(gyro_offset[i], 6);
  }
  Serial.println("Done calibrating, begin data collection");
  delay(10000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // if (!cal.begin()) {
  //   Serial.println("Failed to initialize calibration helper");
  // }
  // else if (!cal.loadCalibration()) {
  //   Serial.println("no calibration loaded/found");
  // }

  init_sensors();
  setup_sensors();
  calibrateGyro();
  // print the header
  Serial.println("aX,aY,aZ,gX,gY,gZ");
}

void loop() {
  delay(3000);

  float aX, aY, aZ, gX, gY, gZ;
  float ax, ay, az;
  sensors_event_t accel, gyro;

  samplesRead = 0;

  Serial.println(String(++total) + "starting...");
  delay(1000);

  while (samplesRead < numSamples) {
    if (lsm6ds.accelerationAvailable() && lsm6ds.gyroscopeAvailable()) {
      accelerometer->getEvent(&accel);
      gyroscope->getEvent(&gyro);

      // cal.calibrate(gyro);

      ax = accel.acceleration.x - bias_vector[0];
      ay = accel.acceleration.y - bias_vector[1];
      az = accel.acceleration.z - bias_vector[2];

      aX = ax * correction_matrix[0] + ay * correction_matrix[1] + az * correction_matrix[2];
      aY = ax * correction_matrix[3] + ay * correction_matrix[4] + az * correction_matrix[5];
      aZ = ax * correction_matrix[6] + ay * correction_matrix[7] + az * correction_matrix[8];

      gX = gyro.gyro.x - gyro_offset[0];
      gY = gyro.gyro.y - gyro_offset[1];
      gZ = gyro.gyro.z - gyro_offset[2];

      samplesRead++;

      Serial.print(aX, 3);
      Serial.print(',');
      Serial.print(aY, 3);
      Serial.print(',');
      Serial.print(aZ, 3);
      Serial.print(',');
      Serial.print(gX, 3);
      Serial.print(',');
      Serial.print(gY, 3);
      Serial.print(',');
      Serial.print(gZ, 3);
      Serial.println();

      if (samplesRead == numSamples) {
        Serial.println();
      }
    }
  }
}
