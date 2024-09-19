#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

// OTA DFU service
BLEDfu bledfu;

// Peripheral uart service
BLEUart bleuart;

// Central uart client
BLEClientUart clientUart;

const int usrBtn = 7;
int btnState = 0;
int btnTrig = 0;
bool btnActive = false;
bool longPressActive = false;
long btnTimer = 0;
long longPressTime = 500;
long extraLongPressTime = 2000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(usrBtn, INPUT_PULLUP);
  
  pixel.begin();
  pixel.setBrightness(20);
  pixel.setPixelColor(0, 200, 0, 100);
  pixel.show();

  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1  
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Aloysius Sense 2");

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

  //Serial.println("setup done");
  pixel.clear();   //  Set pixel 0 to (r,g,b) color value
  pixel.show();
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
  static uint16_t counter = 0;

  button_press();
  if (btnState != 0) {
    btnTrig = btnState;
  }

  counter++;
  if (counter >= 50000) {
    counter = 0;
    sendBLEData();
    btnTrig = 0;
  }
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

void button_press()
{
  int action = 0;
  if (digitalRead(usrBtn) == LOW) {
    if (btnActive == false) {
      btnActive = true;
      btnTimer = millis();
    }
    if ((millis() - btnTimer > extraLongPressTime) && (longPressActive == false)) {
      longPressActive = true;
      //extra long press action
      action = 3;
      Serial.println("extra long press");
    }
  }
  else {
    if (btnActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      }
      else {
        if (millis() - btnTimer > longPressTime) {
          //long press action
          action = 2;
          Serial.println("long press");
        }
        else {
          //short press action
          action = 1;
          Serial.println("short press");
        }
      }
      btnActive = false;
    }
  }
  if (btnState) {
    btnState = 0;
  }
  else {
    btnState = action;
  }
}

void sendBLEData(void)
{
  static uint32_t ii=0, xx=10;
  char str[1];
  
  sprintf(str, "%d", btnTrig);
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