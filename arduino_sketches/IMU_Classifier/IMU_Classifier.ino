// #include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LSM6DS33.h>
#include <deque>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

// #if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
//   Adafruit_Sensor_Calibration_EEPROM cal;
// #else
//   Adafruit_Sensor_Calibration_SDFat cal;
// #endif

#include <TensorFlowLite.h>
//#include <tensorflow/lite/micro/all_ops_resolver.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

#include "model.h"

namespace std {
  void __throw_bad_alloc() {
    Serial.println("Unable to allocate memory");
  }

  void __throw_length_error(char const* e) {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}
uint32_t ta = 0; 
const int numSamples = 104;
const int overlap = 20;
const int bufferSize = numSamples;

float gyro_offset[3] = {0.090713, -0.094150, -0.070173};
const float bias_vector[3] = {-0.08, -0.16, 0.47};
const float correction_matrix[9] = {1.02248, 0.00633, -0.00331,
                                  0.00633, 1.01480, 0.00440, 
                                  -0.00331, 0.00440, 1.01081};

std::deque<float> axBuffer, ayBuffer, azBuffer, gxBuffer, gyBuffer, gzBuffer;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
//tflite::AllOpsResolver tflOpsResolver;
tflite::MicroMutableOpResolver<7> tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = {
  "fist",
  "random"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

float ax_min= -12.821 , ay_min= -19.334 , az_min= -11.208 , gx_min= -5.101 , gy_min= -3.888 , gz_min= -2.733;
float ax_max= 15.634 , ay_max= 20.08 , az_max= 19.46 , gx_max= 4.913 , gy_max= 3.583 , gz_max= 3.793;

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
  Serial.println("Done calibrating");
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

  tflOpsResolver.AddConv2D();
  tflOpsResolver.AddMaxPool2D();
  tflOpsResolver.AddReshape();
  tflOpsResolver.AddFullyConnected();
  tflOpsResolver.AddSoftmax();

  tflOpsResolver.AddQuantize();
  tflOpsResolver.AddDequantize();

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}

void loop() {
  //delay(3000);

  float ax, ay, az, aX, aY, aZ;
  sensors_event_t accel, gyro;

  //Serial.println("starting...");
  //delay(1000);
  //Serial.println(lsm6ds.accelerationAvailable() || lsm6ds.gyroscopeAvailable());
  if (lsm6ds.accelerationAvailable() && lsm6ds.gyroscopeAvailable()) {
    // uint32_t tb = millis();
    // Serial.println(tb-ta);
    // ta = tb;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);

    ax = accel.acceleration.x - bias_vector[0];
    ay = accel.acceleration.y - bias_vector[1];
    az = accel.acceleration.z - bias_vector[2];

    aX = ax * correction_matrix[0] + ay * correction_matrix[1] + az * correction_matrix[2];
    aY = ax * correction_matrix[3] + ay * correction_matrix[4] + az * correction_matrix[5];
    aZ = ax * correction_matrix[6] + ay * correction_matrix[7] + az * correction_matrix[8];

    uint32_t t1 = millis();

    axBuffer.push_back(aX);
    ayBuffer.push_back(aY);
    azBuffer.push_back(aZ);
    gxBuffer.push_back(gyro.gyro.x - gyro_offset[0]);
    gyBuffer.push_back(gyro.gyro.y - gyro_offset[1]);
    gzBuffer.push_back(gyro.gyro.z - gyro_offset[2]);

    if (axBuffer.size() > bufferSize) {
      for (int i=0; i<overlap; i++) {
        axBuffer.pop_front();
        ayBuffer.pop_front();
        azBuffer.pop_front();
        gxBuffer.pop_front();
        gyBuffer.pop_front();
        gzBuffer.pop_front();
      }
    }

    // Serial.print(axBuffer[bufferIndex], 3);
    // Serial.println();
    // check if the all the required samples have been read since
    // the last time the significant motion was detected
    if (axBuffer.size() == bufferSize && ayBuffer.size() == bufferSize && azBuffer.size() == bufferSize
        && gxBuffer.size() == bufferSize && gyBuffer.size() == bufferSize && gzBuffer.size() == bufferSize) {
      // aX = accel.acceleration.x;
      // aY = accel.acceleration.y;
      // aZ = accel.acceleration.z;

      // gX = gyro.gyro.x;
      // gY = gyro.gyro.y;
      // gZ = gyro.gyro.z;

      // normalize the IMU data between 0 to 1 and store in the model's
      // input tensor
      for (int i = 0; i < numSamples; i++) {
        tflInputTensor->data.f[i * 6 + 0] = (axBuffer.at(i) - ax_min) / (ax_max - ax_min);
        tflInputTensor->data.f[i * 6 + 1] = (ayBuffer.at(i) - ay_min) / (ay_max - ay_min);
        tflInputTensor->data.f[i * 6 + 2] = (azBuffer.at(i) - az_min) / (az_max - az_min);
        tflInputTensor->data.f[i * 6 + 3] = (gxBuffer.at(i) - gx_min) / (gx_max - gx_min);
        tflInputTensor->data.f[i * 6 + 4] = (gyBuffer.at(i) - gy_min) / (gy_max - gy_min);
        tflInputTensor->data.f[i * 6 + 5] = (gzBuffer.at(i) - gz_min) / (gz_max - gz_min);
        
        // tflInputTensor->data.f[i * 6 + 0] = axBuffer.at(i);
        // tflInputTensor->data.f[i * 6 + 1] = ayBuffer.at(i);
        // tflInputTensor->data.f[i * 6 + 2] = azBuffer.at(i);
        // tflInputTensor->data.f[i * 6 + 3] = gxBuffer.at(i);
        // tflInputTensor->data.f[i * 6 + 4] = gyBuffer.at(i);
        // tflInputTensor->data.f[i * 6 + 5] = gzBuffer.at(i);
      }

      // Run inferencing
      TfLiteStatus invokeStatus = tflInterpreter->Invoke();
      if (invokeStatus != kTfLiteOk) {
        Serial.println("Invoke failed!");
        while (1);
        return;
      }

      //Loop through the output tensor values from the model
      for (int i = 0; i < 1; i++) {
        Serial.print(GESTURES[i]);
        Serial.print(": ");
        Serial.println(tflOutputTensor->data.f[i], 6);
      }
      // for (int i=0; i<104; i++) {
      //   Serial.print(axBuffer[i]);
      //   Serial.print(", ");
      //   Serial.print(ayBuffer[i]);
      //   Serial.print(", ");
      //   Serial.print(azBuffer[i]);
      //   Serial.print(", ");
      //   Serial.print(gxBuffer[i]);
      //   Serial.print(", ");
      //   Serial.print(gyBuffer[i]);
      //   Serial.print(", ");
      //   Serial.println(gzBuffer[i]);
      // }
      
      // if (tflOutputTensor->data.f[0] >= 0.5) {
      //   Serial.print("fist: ");
      //   Serial.println(tflOutputTensor->data.f[0], 6);
      // }

      uint32_t t2 = millis() - t1;
      Serial.println(t2);

      // if (tflOutputTensor->data.f[0] >= 0.999999) {
      //   for (int i=0; i<208; i++) {
      //     Serial.print(axBuffer[i]);
      //     Serial.print(", ");
      //     Serial.print(ayBuffer[i]);
      //     Serial.print(", ");
      //     Serial.print(azBuffer[i]);
      //     Serial.print(", ");
      //     Serial.print(gxBuffer[i]);
      //     Serial.print(", ");
      //     Serial.print(gyBuffer[i]);
      //     Serial.print(", ");
      //     Serial.println(gzBuffer[i]);
      //   }
      //  }
      //Serial.println();
    }
  }
}
