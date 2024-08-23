#include <bluefruit.h>
//#include <Adafruit_LIS3MDL.h>
//#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_NeoPixel.h>
#include <deque>
//#include <algorithm>

#include <TensorFlowLite.h>
//#include <tensorflow/lite/micro/all_ops_resolver.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

#include "model.h"

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "LSM6DS_LIS3MDL.h"

Adafruit_NXPSensorFusion filter; 
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

// OTA DFU service
BLEDfu bledfu;

// Peripheral uart service
BLEUart bleuart;

// Central uart client
BLEClientUart clientUart;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

uint32_t timestamp;
uint32_t fistTimer = 0;

float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float roll, pitch, heading;
int fist = 0;
int fist2 = 0;
int fistCounter = 0;

float gyro_offset[3];
const float bias_vector[3] = {-0.08, -0.16, 0.47};
const float correction_matrix[9] = {1.02248, 0.00633, -0.00331,
                                  0.00633, 1.01480, 0.00440, 
                                  -0.00331, 0.00440, 1.01081};

const int usrBtn = 7;
int btnState = 0;

const int numSamples = 104;
const int overlap = 20;
const int bufferSize = numSamples;
std::deque<float> axBuffer, ayBuffer, azBuffer, gxBuffer, gyBuffer, gzBuffer;

float ax_min= -12.821 , ay_min= -19.334 , az_min= -11.208 , gx_min= -5.101 , gy_min= -3.888 , gz_min= -2.733;
float ax_max= 15.634 , ay_max= 20.08 , az_max= 19.46 , gx_max= 4.913 , gy_max= 3.583 , gz_max= 3.793;

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

namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}

// class YawDriftOffset {
//   private:
//     int buffer_size, yaw_offset, drift_threshold;
//     std::deque<int> yaw_buffer;
  
//   public:
//     YawDriftOffset(int drift_threshold = 1, int buffer_size = 11) {
//       this->yaw_offset = 0;
//       this->drift_threshold = drift_threshold;
//       this->buffer_size = buffer_size;
//     }
    
//     int normalize_yaw(int yaw) {
//       return (yaw % 361 + 361) % 361;
//     }

//     int correct_yaw(int current_yaw) {
//       if (yaw_buffer.size() == buffer_size) {
//         bool first_ten_same = std::all_of(yaw_buffer.begin(), yaw_buffer.begin() + buffer_size - 1,
//           [this](int i) { return (i == yaw_buffer[0]); });
        
//         if (first_ten_same && yaw_buffer[10] != yaw_buffer[0]) {
//           if (current_yaw == yaw_buffer[10]) {
//             yaw_offset += yaw_buffer[10] - yaw_buffer[0];
//             Serial.println("yaw offset updated: " + String(yaw_offset));
//           }
//         }
//       }
      
//       yaw_buffer.push_back(current_yaw);
//       if (yaw_buffer.size() > buffer_size) {
//         yaw_buffer.pop_front();
//       }

//       int result = current_yaw - yaw_offset ;
      
//       return normalize_yaw(result);
//     }
// };

//YawDriftOffset yawOffset;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {

    delay(1);
  }

  pinMode(usrBtn, INPUT_PULLUP);
  
  pixel.begin();
  pixel.setBrightness(20);
  pixel.setPixelColor(0, 200, 0, 100);
  pixel.show();

  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1  
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Aloysius Sense");

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(cent_bleuart_rx_callback);


  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms, interval = arg1, window = arg2.
  //Bluefruit.Scanner.filterUuid(bleuart.uuid);  //commented off so as to detect all BLE devices
  Bluefruit.Scanner.filterRssi(-60);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  // Set up and start advertising
  startAdv();

  //*******Init motion related
  if (!cal.begin()) {
    //Serial.println("Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    //Serial.println("No calibration loaded/found");
  }
  init_motion();

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

  //Serial.println("setup done");
  pixel.clear();   //  Set pixel 0 to (r,g,b) color value
  pixel.show();
}

void calibrateGyro() {
  delay(1000);
  float min_x, max_x, mid_x;
  float min_y, max_y, mid_y;
  float min_z, max_z, mid_z;
  float x, y, z;
  sensors_event_t gyro;
  gyroscope->getEvent(&gyro);
  min_x = max_x = gyro.gyro.x;
  min_y = max_y = gyro.gyro.y;
  min_z = max_z = gyro.gyro.z;
  delayMicroseconds(9765);
  for (int i=0; i<1000; i++) {
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
    delayMicroseconds(9765);
  }
  gyro_offset[0] = mid_x;
  gyro_offset[1] = mid_y;
  gyro_offset[2] = mid_z;

  for (int i=0; i<3; i++) {
    Serial.println(gyro_offset[i], 6);
  }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop() {
  // do nothing, all the work is done in callback
  loop_motion();
  //sendBLEData();

  // static uint32_t previousTime = 0;
  // uint32_t currentTime = millis();

  // if ( currentTime - previousTime >= 10000UL ) {
  //   previousTime = currentTime;
  //   fist = fistGesture();
  // }
  // else {
  //   fist = 0;
  // }
  
}

void prph_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  //Serial.print("[Prph] Connected to ");
  //Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  //Serial.println();
  //Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  uint16_t xx=100;
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  //Serial.print("[Prph] RX: ");
  //Serial.println(str);  

  if ( clientUart.discovered() )
  {
    clientUart.print(str);
  }else
  {
    bleuart.println("[Prph]CentralRole NC");
    delay(xx);
    bleuart.println("ACC:1.0:0.0:-1.0:XX");
    delay(xx);
    bleuart.println("GYR:0.0:1.0:0.1:XX");
    delay(xx);
    bleuart.println("MAG:0.0:0.1:1.0:XX");
    
    //bleuart.println("012345678901234567890123456789");
  }
}

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{

  //************ Below code for connecting to peripheral detected *********
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised  
  // Connect to the device with bleuart service in advertising packet  
  // Bluefruit.Central.connect(report);  //connection api commented off
  // **********************************************************************

  // Call back modified to do reporting of *ALL* detected devices

  //Serial.println("Timestamp Addr              Rssi Data");

  //Serial.printf("%09d ", millis());
  
  // MAC is in little endian --> print reverse
  //Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  //Serial.print(" ");

  //Serial.print(report->rssi);
  //Serial.print("  ");

  //Serial.printBuffer(report->data.p_data, report->data.len, '-');
  //Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void cent_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  //Serial.print("[Cent] Connected to ");
  //Serial.println(peer_name);;

  if ( clientUart.discover(conn_handle) )
  {
    // Enable TXD's notify
    clientUart.enableTXD();
  }else
  {
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  //Serial.println("[Cent] Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param cent_uart Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void cent_bleuart_rx_callback(BLEClientUart& cent_uart)
{
  char str[20+1] = { 0 };
  cent_uart.read(str, 20);
      
  //Serial.print("[Cent] RX: ");
  //Serial.println(str);

  if ( bleuart.notifyEnabled() )
  {
    // Forward data from our peripheral to Mobile
    bleuart.print( str );
  }else
  {
    // response with no prph message
    clientUart.println("[Cent] Peripheral role not connected");
  }  
}

void init_motion(void)
{
  init_sensors();

  setup_sensors();
  calibrateGyro();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000);
}

uint32_t t1 = 0;
int t3 = 0;

void loop_motion(void) 
{
  float ax, ay, az, yaw;
  float gyro_xcal, gyro_ycal, gyro_zcal;
  static uint8_t counter = 0;

  while ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    delay(1);
  }

  timestamp = millis();
  sensors_event_t accel, gyro, mag;
  // Serial.println(lsm6ds.accelerationAvailable());
  // Serial.println(lsm6ds.gyroscopeAvailable());
  if (lsm6ds.accelerationAvailable() && lsm6ds.gyroscopeAvailable() && lis3mdl.magneticFieldAvailable()) {
    
    uint32_t t2 = micros();
    // if (t3 == 1) {
    //  Serial.println(t2-t1);
    // }
    t1 = t2;
    
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    ax = accel.acceleration.x - bias_vector[0];
    ay = accel.acceleration.y - bias_vector[1];
    az = accel.acceleration.z - bias_vector[2];

    accel_x = ax * correction_matrix[0] + ay * correction_matrix[1] + az * correction_matrix[2];
    accel_y = ax * correction_matrix[3] + ay * correction_matrix[4] + az * correction_matrix[5];
    accel_z = ax * correction_matrix[6] + ay * correction_matrix[7] + az * correction_matrix[8];

    gyro_xcal = gyro.gyro.x - gyro_offset[0];
    gyro_ycal = gyro.gyro.y - gyro_offset[1];
    gyro_zcal = gyro.gyro.z - gyro_offset[2];

    gyro_x = gyro_xcal * SENSORS_RADS_TO_DPS;
    gyro_y = gyro_ycal * SENSORS_RADS_TO_DPS;
    gyro_z = gyro_zcal * SENSORS_RADS_TO_DPS;

    cal.calibrate(mag);

    magnetic_x = mag.magnetic.x;
    magnetic_y = mag.magnetic.y;
    magnetic_z = mag.magnetic.z;

    axBuffer.push_back(accel_x);
    ayBuffer.push_back(accel_y);
    azBuffer.push_back(accel_z);
    gxBuffer.push_back(gyro_xcal);
    gyBuffer.push_back(gyro_ycal);
    gzBuffer.push_back(gyro_zcal);

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

    fist = fistGesture();
    if (fist == 1) {
      fist2 = 69;
    }

    filter.update(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, magnetic_x, magnetic_y, magnetic_z);

    // if (counter++ <= PRINT_EVERY_N_UPDATES) {
    //   return;
    // }
  
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    // yaw = int(filter.getYaw());
    // heading = yawOffset.correct_yaw(yaw);


    counter++;
    if (counter >= 10) {
      counter = 0;
      sendBLEData();
      fist2 = 0;
    }
    // Serial.println("Test A = " + String(roll) + "," + String(pitch) + "," + String(heading));
  }
}

uint32_t lastFistTime = 0;
const uint32_t fistTimeOut = 1000;

int fistGesture()
{
  uint32_t curTime = millis();
  if (curTime - lastFistTime < fistTimeOut) {
    return 0;
  }
  t3 = 0;
  if (axBuffer.size() == bufferSize && ayBuffer.size() == bufferSize && azBuffer.size() == bufferSize
    && gxBuffer.size() == bufferSize && gyBuffer.size() == bufferSize && gzBuffer.size() == bufferSize) {
    // normalize the IMU data between 0 to 1 and store in the model's
    // input tensor
    t3 = 1;
    for (int i = 0; i < numSamples; i++) {
      tflInputTensor->data.f[i * 6 + 0] = (axBuffer.at(i) - ax_min) / (ax_max - ax_min);
      tflInputTensor->data.f[i * 6 + 1] = (ayBuffer.at(i) - ay_min) / (ay_max - ay_min);
      tflInputTensor->data.f[i * 6 + 2] = (azBuffer.at(i) - az_min) / (az_max - az_min);
      tflInputTensor->data.f[i * 6 + 3] = (gxBuffer.at(i) - gx_min) / (gx_max - gx_min);
      tflInputTensor->data.f[i * 6 + 4] = (gyBuffer.at(i) - gy_min) / (gy_max - gy_min);
      tflInputTensor->data.f[i * 6 + 5] = (gzBuffer.at(i) - gz_min) / (gz_max - gz_min);
    }

    // Run inferencing
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
      Serial.println("Invoke failed!");
      while (1);
      return 0;
    }

    if (tflOutputTensor->data.f[0] > 0.5) {
      Serial.println();
      Serial.println("SUCCESS");
      Serial.println();
      
      lastFistTime = curTime;

      return 1;
    }
    // if (tflOutputTensor->data.f[0] >= 0.9) {
    //   // Serial.print("fist: ");
    //   // Serial.println(tflOutputTensor->data.f[0], 6);
    //   if (currentTime - lastDetectionTime <= 500) { // Check if within 3 seconds
    //     if (++fistCounter >= 2) {
    //       fistCounter = 0;
    //       if (currentTime - fistTimer > 3000) {
    //         fistTimer = currentTime; 
    //         fist2 = 1;
    //         Serial.println();
    //         Serial.println("SUCCESS");
    //         Serial.println();
    //         return 69;
    //       }
    //     }
    //   }
    //   else {
    //     fistCounter = 1;
    //   } 
    //   lastDetectionTime = currentTime;
    // }
    // else if (currentTime - lastDetectionTime > 500) {
    //   fistCounter = 0;
    // }
  }
  return 0;
}

void sendBLEData(void)
{
  btnState = digitalRead(usrBtn);

  if (btnState == LOW) {
    pixel.setPixelColor(0, 200, 0, 100);   //  Set pixel 0 to (r,g,b) color value
    pixel.show();
  }
  else {
    pixel.clear();   //  Set pixel 0 to (r,g,b) color value
    pixel.show();
  }

  static uint32_t ii=0, xx=10;
  char str[26];
  
  sprintf(str, "%d, %d, %d, %d, %d", int(roll), int(pitch), int(heading), fist2, btnState);
  Serial.println(str);
  bleuart.print(str);
  //delay(xx);

  // sprintf(str, "+");
  // bleuart.print(str);
  // delay(5000);
  
  // sprintf(str, "-");
  // bleuart.print(str);
  // delay(5000);
  // sprintf(str, "\n**Sensors Output**");
  // bleuart.print(str);
  // delay(xx);

  // sprintf(str, "\nAc:%.2f:%.2f:%.2f", accel_x, accel_y, accel_z);  
  // bleuart.print(str);
  // delay(xx);

  // sprintf(str, "\nGy:%.2f:%.2f:%.2f", gyro_x, gyro_y, gyro_z);  
  // bleuart.print(str);
  // delay(xx);

  // sprintf(str, "\nMg:%.2f:%.2f:%.2f", magnetic_x, magnetic_y, magnetic_z);  
  // bleuart.print(str);
  // delay(xx);

  //Serial.println("\nFeatherSenseDemo");
  //Serial.println("------------------");
  
  //Serial.print("Magnetic: ");
  //Serial.print(magnetic_x);
  //Serial.print(" ");
  //Serial.print(magnetic_y);
  //Serial.print(" ");
  //Serial.print(magnetic_z);
  //Serial.println(" uTesla");
  //Serial.print("Acceleration: ");
  //Serial.print(accel_x);
  //Serial.print(" ");
  //Serial.print(accel_y);
  //Serial.print(" ");
  //Serial.print(accel_z);
  //Serial.println(" m/s^2");
  //Serial.print("Gyro: ");
  //Serial.print(gyro_x);
  //Serial.print(" ");
  //Serial.print(gyro_y);
  //Serial.print(" ");
  //Serial.print(gyro_z);
  //Serial.println(" dps");
}