/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <neotimer.h>

// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   2  // sparkfun SO pin
#define MAXCS   3  // sparkfun CS pin
#define MAXCLK  4  // sparkfun SCK pin

// initialize the Thermocouple; works fine for Sparkfun or Adafruit breakouts of MAX31855K
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

/* Pyrometer Serivce:                    142b8c89-28d2-4196-a978-6fcc64823422
 * Pyrometer Measurement Charactersitic: 9fe5ea27-1273-47f2-b0a6-abd53bfd3ac6
 */

const uint8_t PYROMETER_UUID_SERVICE[] =
{
    0x22, 0x34, 0x82, 0x64, 0xCC, 0x6F, 0x78, 0xA9,
    0x96, 0x41, 0xD2, 0x28, 0x89, 0x8C, 0x2B, 0x14
};

const uint8_t PYROMETER_UUID_MEASUREMENT[] =
{
    0xC6, 0x3A, 0xFD, 0x3B, 0xD5, 0xAB, 0xA6, 0xB0,
    0xF2, 0x47, 0x73, 0x12, 0x27, 0xEA, 0xE5, 0x9F
};

BLEService        pyro = BLEService(PYROMETER_UUID_SERVICE); //BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic pmes = BLECharacteristic(PYROMETER_UUID_MEASUREMENT);

// BLE Service
BLEDis  bledis;
BLEBas  blebas;

// float for pyrometer measurement in degrees C
double pyroC;

// signed ints for BLE measurement
int16_t measurement;
int16_t measurementPrev;

// Advanced function prototypes
void startAdv(void);
void setupPyro(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void cccd_callback(BLECharacteristic& chr, ble_gatts_evt_write_t* request);

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

// Create shutdown timer
Neotimer shutdownTimer = Neotimer(300000); // 300 second timer (5 minutes)

// Pin that will trigger shutdown
const int SHUTDOWN_PIN =  16;

// Threshhold temperature where the probe is considered not in use
const double SHUTDOWN_THRESHOLD = 35;

void setup()
{

  // set the shutdown pin as output:
  pinMode(SHUTDOWN_PIN, OUTPUT);
  //  start the auto shutdown timer
  shutdownTimer.start();
  
  Serial.begin(115200);
  Serial.println("Open Tire Pyrometer Serial");
  Serial.println("---------------------------\n");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'Open Tire Pyrometer'");
  Bluefruit.setName("Open Tire Pyrometer");
  
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Jenkins Racing Open Source");
  bledis.setModel("Open Tire Pyrometer V0.3");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Pyrometer service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Pyrometer Service");
  setupPyro();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();
  
  Serial.println("\nAdvertising");

}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(pyro);

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

void setupPyro(void)
{
  // Configure the Pyrometer service
  // Supported Characteristics:
  // Name                         UUID                                  Requirement Properties
  // ---------------------------- ------------------------------------- ----------- ----------
  // Pyrometer Measurement        9fe5ea27-1273-47f2-b0a6-abd53bfd3ac6  Mandatory   Notify

  pyro.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Pyrometer Measurement characteristic
  // Permission = Notify
  // Min Len    = 1
  // Max Len    = 4
  //    B0      = UINT8  - Flag (MANDATORY)
  //      b5:7  = Reserved
  //      b4    = Reserved
  //      b3    = Reserved
  //      b1:2  = Sensor status (0 = OK, 1 = Thermo Error, 2 = Future)
  //      b0    = Decimals (0 = None, 1 = One)
  //    B1      = UINT8  - Spare for Future
  //    B2:3    = UINT16  - Pyrometer temperature in C

  pmes.setProperties(CHR_PROPS_NOTIFY);
  pmes.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pmes.setFixedLen(4);
  pmes.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  pmes.begin();
  uint8_t pyroData[4] = { 0b00000001, 0x00, 0x00, 0x00 }; // Set the characteristic to one decimal and status OK
  pmes.notify(pyroData, 4);                               // Use .notify instead of .write!

}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr.uuid == pmes.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Pyrometer Measurement 'Notify' enabled");
        } else {
            Serial.println("Pyrometer Measurement 'Notify' disabled");
        }
    }
}

void loop()
{
   // basic readout test, just print the current internal temp
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());

  uint8_t pyroData[4];

   pyroC = thermocouple.readCelsius();
   if (isnan(pyroC)) {
     Serial.println("Something wrong with thermocouple!");
     // set status bits (error bits); set measurement to max negative value 
     measurement = -2000;
     pyroData[0] = 0b00000011;
     pyroData[1] = 0x00;
     pyroData[2] = (uint8_t)measurement;
     pyroData[3] = (uint8_t)(measurement >> 8);
   } else {
     Serial.print("C = "); 
     Serial.println(pyroC);

     // set status and data with measurement
     // cast double to int with tenths preceision (thermocouple readings are in .25 resoution; hundreths get dropped)
     measurement = pyroC * 10;
     pyroData[0] = 0b00000001;
     pyroData[1] = 0x00;
     pyroData[2] = (uint8_t)measurement;
     pyroData[3] = (uint8_t)(measurement >> 8);

   }

  // useful example for splitting 16bit INT to two BYTES
  //uint8_t hi_lo[] = { (uint8_t)(value >> 8), (uint8_t)value }; // { 0xAA, 0xFF }
  //uint8_t lo_hi[] = { (uint8_t)value, (uint8_t)(value >> 8) }; // { 0xFF, 0xAA }

  if ( Bluefruit.connected() ) {
    // only trigger a new notify() if thercouple changes
    if ( measurement != measurementPrev ) {
      // Note: We use .notify instead of .write!
      // If it is connected but CCCD is not enabled
      // The characteristic's value is still updated although notification is not sent
      if ( pmes.notify(pyroData, sizeof(pyroData)) ){
        Serial.print("Pyrometer updated to: "); Serial.println(measurement); 
      }else{
        Serial.println("ERROR: Notify not set in the CCCD or not connected!");
      }
    }
    // record the prevous value handled by BLE to check for value change
    measurementPrev = measurement;
  }

  // check if the probe is above ambient; reset the shutdown timer if it is
  if(pyroC > SHUTDOWN_THRESHOLD){
    Serial.println("Shutdown Timer was reset");
    shutdownTimer.reset();
    shutdownTimer.start();
  }
  
  // if the probe is at ambient check if the shutdown timer has finished
  if(shutdownTimer.done()){
    Serial.println("Shutdown Timer finished");
    digitalWrite(SHUTDOWN_PIN, HIGH);
  }

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}


/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}


/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 * 
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 * 
 * WARNING: This function MUST NOT call any blocking FreeRTOS API 
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

