
/*********************************************************************
  Craig's Auxilary Bike Display
  - works in sunlight
  - low power (can run for months off button cell)
  - BLE connection to smartphone
  - saves smartphone battery
  - connects to speed/cadence BLE sensors

  - displays:
  Clock, date?
  Tempature
  Altitude (from GPS or pressure sensor?)

  Current/average speed
  Current/averate cadence
  Total distance since ride start
  Total time since ride start
  Cumulative ascent (daily?) - consider using GPS to calibrate altiude initially
  Cumulative descent (daily?)

  Estimated gear ratio (or gear if chainring # teeth known).  Gear ratio/cadence to be constant before changing number.

  Song title+artist (if playing)
  Txts, IMs
  Google maps directions (potentially)

  Acceleration
  Gradient (determine from 3D acceleration?)
  Compass direction?

  Physical reset button could be used, otherwise just reset daily?
  Same or another physical button could be used to set wheel diameter?  Or somehow send commands from phone?

  Data store:
  paired devices (MAC address?)
  each mac address is "E2:CB:25:75:42:75" = 12 bytes = 3* 32 bit values if stored as ASCII chars...
  wheel diameter (for speed calculation)


  watchdog, the following code works - can directly write to registers
  //Configure WDT.
  NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV            = 32769;    // CRV = timeout * 32768 + 1
  NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
  NRF_WDT->TASKS_START    = 1;        // Start WDT

  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;


  User information configuration registers (UICR) are written in the same way as flash. After UICR has been
  written, the new UICR configuration will only take effect after a reset.
  UICR can only be written nWRITE number of times before an erase must be performed using ERASEUICR on
  page 29 or ERASEALL on page 28. The time it takes to write a word to UICR is specified by tWRITE.
  The CPU is halted if the CPU executes code from the flash while the NVMC is writing to the UICR

  Addresses:
  Flash 		0x00000000 - 0x00800000
  Code RAM  	0x00800000
  FICR 		0x10000000
  UICR 		0x10001000
  XIP Flash 	0x12000000 - 0x19FFFFFF

  Data RAM 	0x20000000

*********************************************************************/
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "mbed.h"
#include <time.h>
#include "rtos.h"

//#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#include <Arduino_LSM9DS1.h> // acceleromter, gyro
#include <Arduino_LPS22HB.h> // Arduino_v pressure sensor
#include <Arduino_HTS221.h> // temperature, humidity
#include <ArduinoBLE.h> // https://www.espruino.com/Gadgetbridge
#include <HardwareBLESerial.h> //https://github.com/Uberi/Arduino-HardwareBLESerial
#include <ArduinoJson.h>

// any pins can be used
#define SHARP_SCK  13
#define SHARP_MOSI 11
#define SHARP_SS   10

#define BLACK 0
#define WHITE 1

#define LFCLK_FREQUENCY 0x8000  //2^15 ticks/second (32768)
#define RTC_COUNTER_SIZE 0x1000000 // 2^24
#define RTC_ONE_SECOND 0x8000

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

#define BLE_STRING_BUFFER_SIZE 1024
#define BLE_POLL_INTERVAL_S 0.01
#define BLE_POLL_INTERVAL_MS 10

#define BLE_SCAN_SPEED
//#define BLE_SCAN_CADENCE
//#define CADENCE_BLE_NAME "8302-81"
#define SPEED_BLE_NAME "8302-81"
//#define SPEED_BLE_NAME "22914-161"
#define CADENCE_BLE_NAME "22914-161"

#define BATTERY_CHARACTERISTIC "2a19"
#define CSC_CHARACTERISTIC "2a5b"

#define TIRE_CIRCUMFERENCE 2114
#define MM_TO_KM (1000*1000)
#define SECONDS_TO_HOURS (60 * 60)

/* Global Variables

*/
// RTC variables
uint32_t prevRTCCounter = 0;
uint32_t currentUnixTime = 0;
uint32_t currentTimeZone = 0;

// LCD variables
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, DISPLAY_WIDTH, DISPLAY_HEIGHT);

// Internal sensors (I2C)
float pressure;
float referencePressure = 1024;
float altitude;
float temperature, humidity;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;
float magnet_x, magnet_y, magnet_z, magnet_heading;

// BLE sensors
BLEDevice cadencePeripheral, speedPeripheral;
BLECharacteristic cadenceBatteryCharacteristic, cadenceCSCMeasurementCharacteristic;
BLECharacteristic speedBatteryCharacteristic, speedCSCMeasurementCharacteristic;

uint8_t speedBattValue = 0;
uint32_t speedTimePrev = 0;
uint32_t speedRevPrev = 0;

uint8_t cadenceBattValue = 0;
uint32_t cadenceTimePrev = 0;
uint32_t cadenceRevPrev = 0;

float cadenceRpm = 0;
float speedKph = 0;
float maxSpeedKph = 0;

float odometer = 0;
uint32_t movingTimeMS = 0;
float averageSpeed = 0;

float cadenceOdometer = 0;
uint32_t cadenceMovingTimeMS = 0;
float averageCadence = 0;

// BLE Serial - Android connection
HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();
rtos::Thread bleServicingThread;

char bleString[BLE_STRING_BUFFER_SIZE];
bool lineAvailable = false; // flag to control access to bleString between threads

// JSON parsing variables
StaticJsonDocument<300> jsonDoc;
char debugString[200];
const char set_time_string[] = "setTime(";
const char set_time_zone_string[] = "E.setTimeZone(";
const char GB_string[] = "GB({\"t\":\"";

char musicInfoArtist[100];
char musicInfoTrack[100];
char musicState[20];

char notifyId[20];
char notifySource[20];
char notifyTitle[100];
char notifyBody[300];
char notifySender[50];

char callCmd[10];
char callName[50];
char callNumber[30];


/*
  mbed::Ticker bleTicker;

  void blePoller() {
  bleSerial.poll();
  }
*/

// Function Prototypes
void bleServicing();
void setup(void);
void updateRTC();
void updateLocalSensors();
void updateBLESensors();
void generateCardinal(float_t bearing, char *outputArray);
void parseBLESerial();
void blePeripheralDisconnectedHandler(BLEDevice central);
void blePeripheralConnectedHandler(BLEDevice central);
void blePeripheralDiscoveredHandler(BLEDevice central);

/* Initalise everything

*/
void setup(void)
{
  memset(bleString, 0, BLE_STRING_BUFFER_SIZE);
  Serial1.begin(115200);
  Serial1.println("Hello!");

  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // start & clear the display
  display.begin();
  display.clearDisplay();
  display.refresh();

  //BARO.begin(); // baro seems to be takes a second or two to read!
  IMU.begin();
  HTS.begin();

  BLE.begin();
  BLE.setLocalName("Pixl.js_Biyclometer");
  BLE.setDeviceName("Pixl.js_Biyclometer");
  bleSerial.begin(); // sets up nordic uart service

  BLE.setEventHandler(BLEDiscovered, blePeripheralDiscoveredHandler);
  BLE.setEventHandler(BLEConnected, blePeripheralConnectedHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectedHandler);

  // now we've found our devices, advertise so phone can conneect
  BLE.advertise();
  BLE.scan(false);

  bleServicingThread.start(bleServicing);
  //bleTicker.attach(&blePoller, BLE_POLL_INTERVAL_S);
}

/* Callback when a matching peripheral is discovered during scanning

*/
void blePeripheralDiscoveredHandler(BLEDevice peripheral)
{
  if (peripheral.hasLocalName())
  {
    Serial1.println(peripheral.localName());
    if (peripheral.localName() == SPEED_BLE_NAME)
    {
      speedPeripheral = peripheral;
      BLE.stopScan();
      speedPeripheral.connect();
      Serial1.println("Speed sensor connected");
      speedPeripheral.discoverAttributes();
      speedBatteryCharacteristic = speedPeripheral.characteristic(BATTERY_CHARACTERISTIC);
      speedCSCMeasurementCharacteristic = speedPeripheral.characteristic(CSC_CHARACTERISTIC);
      speedCSCMeasurementCharacteristic.subscribe();
      speedBatteryCharacteristic.subscribe();
      speedBatteryCharacteristic.readValue(speedBattValue);
#ifdef BLE_SCAN_CADENCE
      if (!cadencePeripheral.connected())
      {
        BLE.scanForName(CADENCE_BLE_NAME);
      }
#endif
    }
    if (peripheral.localName() == CADENCE_BLE_NAME)
    {
      cadencePeripheral = peripheral;
      cadencePeripheral.connect();
      Serial1.println("Cadence sensor connected");
      cadencePeripheral.discoverAttributes();
      cadenceBatteryCharacteristic = cadencePeripheral.characteristic(BATTERY_CHARACTERISTIC);
      cadenceCSCMeasurementCharacteristic = cadencePeripheral.characteristic(CSC_CHARACTERISTIC);
      cadenceCSCMeasurementCharacteristic.subscribe();
      cadenceBatteryCharacteristic.subscribe();
      cadenceBatteryCharacteristic.readValue(cadenceBattValue);
#ifdef BLE_SCAN_SPEED
      if (!speedPeripheral.connected())
      {
        BLE.scanForName(SPEED_BLE_NAME);
      }
#endif
    }
  }
}

/* Callback when any connection happens!

*/
void blePeripheralConnectedHandler(BLEDevice central)
{
  //BLEDevice android = BLE.central();
  if ( BLE.central())
  {
    // also need to change BLE polling interval back to 10ms to catch all the events!
    //char tempDispString[100];
    //sprintf(tempDispString, "%s connected", android.address());
    //Serial1.println(tempDispString);
    BLE.stopAdvertise();
    Serial1.println("Central device connected, advertising off");
  }
}

void blePeripheralDisconnectedHandler(BLEDevice central)
{
  Serial1.println("Device disconnected");
  if (!BLE.central()) // this if statement doesn't work.. never gets executed...
  {
    BLE.advertise();
    Serial1.println("advertising on");
    // also need to change BLE polling interval and let the processor sleep a bit to save battery life!
  }
}


/* Main thread
    - retreives data from sensors
    - redraws display
    - only really needs to update about every 0.5 - 1 second when moving
    - or every minute when not moving
*/
void loop(void)
{
  char tempDispString[200];
  char tempDispString2[200];
  int16_t x1, y1, x2, y2, x3, y3, x4, y4, x5, y5;
  uint16_t w1, w2, w3, w4, w5, h1, h2, h3, h4, h5;
  uint32_t tempInt;
  char currentTime[50];
  uint32_t dispCurrentLowerOffset = DISPLAY_HEIGHT;
  time_t unixSeconds;

  /***** Thread Initialisation *****/
  display.clearDisplay();
  display.setTextSize(1); // need a font size between 1 and 2... or maybe just a size 2 font with proper anti-aliasing!
  display.setTextColor(BLACK);
  display.setCursor(0, 0);

  /***** Update Sensors  *****/
  updateLocalSensors();
  updateRTC(); // less than 1ms to execute
  updateBLESensors();

  /***** Parse Data  *****/
  parseBLESerial(); // 7ms to execute including dumping string out serial port.


#define COLUMN_1_WIDTH 125
#define COLUMN_2_WIDTH 125
#define COLUMN_3_WIDTH 70


  /***** Speed box drawing *****/
#define DISP_BOX_SPEED_OFFSET_X 0
#define DISP_BOX_SPEED_OFFSET_Y 0
#define DISP_BOX_SPEED_SIZE_X COLUMN_1_WIDTH
#define DISP_BOX_SPEED_SIZE_Y 50

  display.setFont(&FreeSansBold24pt7b);
  sprintf(tempDispString, "%.0f", floor(speedKph));
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setFont(&FreeSans9pt7b);
  sprintf(tempDispString2, ".%.0f km/h", (speedKph - floor(speedKph)) * 10);
  display.getTextBounds(tempDispString2, 0, 0, &x2, &y2, &w2, &h2);

  display.setCursor(DISP_BOX_SPEED_OFFSET_X + ((DISP_BOX_SPEED_SIZE_X - w1 - w2) / 2), DISP_BOX_SPEED_OFFSET_Y + h1);
  display.setFont(&FreeSansBold24pt7b);
  display.print(tempDispString);

  display.setFont(&FreeSans9pt7b);
  display.setCursor(DISP_BOX_SPEED_OFFSET_X + ((DISP_BOX_SPEED_SIZE_X - w1 - w2) / 2) + w1 + 5, DISP_BOX_SPEED_OFFSET_Y + h1 - h2 / 2);
  display.print(tempDispString2);

  display.drawFastHLine(DISP_BOX_SPEED_OFFSET_X, DISP_BOX_SPEED_OFFSET_Y + DISP_BOX_SPEED_SIZE_Y, DISP_BOX_SPEED_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_SPEED_OFFSET_X + DISP_BOX_SPEED_SIZE_X, DISP_BOX_SPEED_OFFSET_Y, DISP_BOX_SPEED_OFFSET_Y + DISP_BOX_SPEED_SIZE_Y, BLACK);


  /***** Odometer box drawing *****/
#define DISP_BOX_ODO_OFFSET_X 0
#define DISP_BOX_ODO_OFFSET_Y DISP_BOX_SPEED_SIZE_Y
#define DISP_BOX_ODO_SIZE_X COLUMN_1_WIDTH
#define DISP_BOX_ODO_SIZE_Y 35
#define DISP_BOX_ODO_VALUE_X_OFFSET DISP_BOX_ODO_OFFSET_X + 30
#define DISP_BOX_ODO_UNITS_X_OFFSET DISP_BOX_ODO_OFFSET_X + DISP_BOX_ODO_SIZE_X-15
#define DISP_BOX_ODO_UNITS_Y_OFFSET DISP_BOX_ODO_OFFSET_Y + 10

  display.setFont();
  display.setCursor(DISP_BOX_ODO_OFFSET_X, DISP_BOX_ODO_UNITS_Y_OFFSET);
  display.print("Odo:");
  display.setCursor(DISP_BOX_ODO_UNITS_X_OFFSET, DISP_BOX_ODO_UNITS_Y_OFFSET);
  display.print("km");

  display.setFont(&FreeSans12pt7b);
  sprintf(tempDispString, "%5.2f", 537.84/*odometer*/);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);

  display.setCursor(DISP_BOX_ODO_VALUE_X_OFFSET, DISP_BOX_ODO_OFFSET_Y + h1 + 4);
  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_ODO_OFFSET_X, DISP_BOX_ODO_OFFSET_Y + DISP_BOX_ODO_SIZE_Y, DISP_BOX_ODO_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_ODO_OFFSET_X + DISP_BOX_ODO_SIZE_X, DISP_BOX_ODO_OFFSET_Y, DISP_BOX_ODO_SIZE_Y, BLACK);

  /***** Moving Time box drawing *****/
#define DISP_BOX_MOV_TIME_OFFSET_X 0
#define DISP_BOX_MOV_TIME_OFFSET_Y DISP_BOX_ODO_OFFSET_Y+DISP_BOX_ODO_SIZE_Y
#define DISP_BOX_MOV_TIME_SIZE_X COLUMN_1_WIDTH
#define DISP_BOX_MOV_TIME_SIZE_Y 35
#define DISP_BOX_MOV_TIME_VALUE_X_OFFSET DISP_BOX_MOV_TIME_OFFSET_X +20
#define DISP_BOX_MOV_TIME_UNITS_X_OFFSET DISP_BOX_MOV_TIME_OFFSET_X + DISP_BOX_MOV_TIME_SIZE_X-20
#define DISP_BOX_MOV_TIME_UNITS_Y_OFFSET DISP_BOX_MOV_TIME_OFFSET_Y + 10
/*
  display.setFont();
  display.setCursor(DISP_BOX_MOV_TIME_OFFSET_X, DISP_BOX_MOV_TIME_OFFSET_Y + 3);
  display.print("Moving");
  display.setCursor(DISP_BOX_MOV_TIME_OFFSET_X, DISP_BOX_MOV_TIME_OFFSET_Y + 13);
  display.print("Time:");
*/
  display.setFont(&FreeSans12pt7b);
  unixSeconds = movingTimeMS / 1000;
  strftime(tempDispString, 50, "%k:%M:%S", localtime(&unixSeconds));
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor(DISP_BOX_MOV_TIME_VALUE_X_OFFSET, DISP_BOX_MOV_TIME_OFFSET_Y + h1 + 4);

  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_MOV_TIME_OFFSET_X, DISP_BOX_MOV_TIME_OFFSET_Y + DISP_BOX_MOV_TIME_SIZE_Y, DISP_BOX_MOV_TIME_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_MOV_TIME_OFFSET_X + DISP_BOX_MOV_TIME_SIZE_X, DISP_BOX_MOV_TIME_OFFSET_Y, DISP_BOX_MOV_TIME_SIZE_Y, BLACK);

  /***** Average Speed box drawing *****/
#define DISP_BOX_AV_SPD_OFFSET_X 0
#define DISP_BOX_AV_SPD_OFFSET_Y DISP_BOX_MOV_TIME_OFFSET_Y+DISP_BOX_MOV_TIME_SIZE_Y
#define DISP_BOX_AV_SPD_SIZE_X COLUMN_1_WIDTH
#define DISP_BOX_AV_SPD_SIZE_Y 35
#define DISP_BOX_AV_SPD_VALUE_X_OFFSET DISP_BOX_AV_SPD_OFFSET_X + 45
#define DISP_BOX_AV_SPD_UNITS_X_OFFSET DISP_BOX_AV_SPD_OFFSET_X + DISP_BOX_AV_SPD_SIZE_X-25
#define DISP_BOX_AV_SPD_UNITS_Y_OFFSET DISP_BOX_AV_SPD_OFFSET_Y + 10

  display.setFont();
  display.setCursor(DISP_BOX_AV_SPD_OFFSET_X, DISP_BOX_AV_SPD_OFFSET_Y + 3);
  display.print("Average");
  /*
  display.setCursor(DISP_BOX_AV_SPD_OFFSET_X, DISP_BOX_AV_SPD_OFFSET_Y + 13);
  display.print("Speed:");
  */

  display.setCursor(DISP_BOX_AV_SPD_UNITS_X_OFFSET, DISP_BOX_AV_SPD_UNITS_Y_OFFSET);
  display.print("km/h");

  display.setFont(&FreeSans12pt7b);
  sprintf(tempDispString, "%3.1f", 25.4/*averageSpeed*/);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);

  display.setCursor(DISP_BOX_AV_SPD_VALUE_X_OFFSET, DISP_BOX_AV_SPD_OFFSET_Y + h1 + 4);
  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_AV_SPD_OFFSET_X, DISP_BOX_AV_SPD_OFFSET_Y + DISP_BOX_AV_SPD_SIZE_Y, DISP_BOX_AV_SPD_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_AV_SPD_OFFSET_X + DISP_BOX_AV_SPD_SIZE_X, DISP_BOX_AV_SPD_OFFSET_Y, DISP_BOX_AV_SPD_SIZE_Y, BLACK);


  /***** Cadence box drawing *****/
#define DISP_BOX_CADENCE_OFFSET_X COLUMN_1_WIDTH
#define DISP_BOX_CADENCE_OFFSET_Y 0
#define DISP_BOX_CADENCE_SIZE_X COLUMN_2_WIDTH
#define DISP_BOX_CADENCE_SIZE_Y 50
  display.setFont(&FreeSansBold24pt7b);
  sprintf(tempDispString, "%.f", cadenceRpm);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setFont();
  display.getTextBounds("rpm", 0, 0, &x2, &y2, &w2, &h2);
  display.setCursor(DISP_BOX_CADENCE_OFFSET_X + ((DISP_BOX_CADENCE_SIZE_X - w1 - w2) / 2), DISP_BOX_CADENCE_OFFSET_Y + h1);
  display.setFont(&FreeSansBold24pt7b);
  display.println(tempDispString);
  display.setFont();
  display.setCursor(DISP_BOX_CADENCE_OFFSET_X + ((DISP_BOX_CADENCE_SIZE_X - w1 - w2) / 2) + w1, DISP_BOX_CADENCE_OFFSET_Y + h1 - h2 / 2);
  display.println(" rpm");

  display.drawFastHLine(DISP_BOX_CADENCE_OFFSET_X, DISP_BOX_CADENCE_OFFSET_Y + DISP_BOX_CADENCE_SIZE_Y, DISP_BOX_CADENCE_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_CADENCE_OFFSET_X + DISP_BOX_CADENCE_SIZE_X, DISP_BOX_CADENCE_OFFSET_Y, DISP_BOX_CADENCE_OFFSET_Y + DISP_BOX_CADENCE_SIZE_Y, BLACK);


  /***** Cadence Odometer box drawing *****/
#define DISP_BOX_CAD_ODO_OFFSET_X COLUMN_1_WIDTH
#define DISP_BOX_CAD_ODO_OFFSET_Y DISP_BOX_CADENCE_SIZE_Y
#define DISP_BOX_CAD_ODO_SIZE_X COLUMN_2_WIDTH
#define DISP_BOX_CAD_ODO_SIZE_Y 35
#define DISP_BOX_CAD_ODO_VALUE_X_OFFSET DISP_BOX_CAD_ODO_OFFSET_X + 30
#define DISP_BOX_CAD_ODO_UNITS_X_OFFSET DISP_BOX_CAD_ODO_OFFSET_X + DISP_BOX_CAD_ODO_SIZE_X-25
#define DISP_BOX_CAD_ODO_UNITS_Y_OFFSET DISP_BOX_CAD_ODO_OFFSET_Y + 10

  display.setFont();
/*  
  display.setCursor(DISP_BOX_CAD_ODO_OFFSET_X, DISP_BOX_CAD_ODO_OFFSET_Y + 3);
  display.print("Cad");
  display.setCursor(DISP_BOX_CAD_ODO_OFFSET_X, DISP_BOX_CAD_ODO_OFFSET_Y + 13);
  display.print("Odo:");
  */
  display.setCursor(DISP_BOX_CAD_ODO_UNITS_X_OFFSET, DISP_BOX_CAD_ODO_UNITS_Y_OFFSET);
  display.print("revs");

  display.setFont(&FreeSans12pt7b);
  sprintf(tempDispString, "%3.f", 537.84/*cadenceOdometer*/);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor(DISP_BOX_CAD_ODO_VALUE_X_OFFSET, DISP_BOX_CAD_ODO_OFFSET_Y + h1 + 4);
  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_CAD_ODO_OFFSET_X, DISP_BOX_CAD_ODO_OFFSET_Y + DISP_BOX_CAD_ODO_SIZE_Y, DISP_BOX_CAD_ODO_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_CAD_ODO_OFFSET_X + DISP_BOX_CAD_ODO_SIZE_X, DISP_BOX_CAD_ODO_OFFSET_Y, DISP_BOX_CAD_ODO_SIZE_Y, BLACK);

  /***** Cadence Moving Time box drawing *****/
#define DISP_BOX_CAD_MOV_TIME_OFFSET_X COLUMN_1_WIDTH
#define DISP_BOX_CAD_MOV_TIME_OFFSET_Y  DISP_BOX_CAD_ODO_OFFSET_Y+DISP_BOX_CAD_ODO_SIZE_Y
#define DISP_BOX_CAD_MOV_TIME_SIZE_X COLUMN_2_WIDTH
#define DISP_BOX_CAD_MOV_TIME_SIZE_Y 35
#define DISP_BOX_CAD_MOV_TIME_VALUE_X_OFFSET DISP_BOX_CAD_MOV_TIME_OFFSET_X + 20
#define DISP_BOX_CAD_MOV_TIME_UNITS_X_OFFSET DISP_BOX_CAD_MOV_TIME_OFFSET_X + DISP_BOX_CAD_MOV_TIME_SIZE_X-20
#define DISP_BOX_CAD_MOV_TIME_UNITS_Y_OFFSET DISP_BOX_CAD_MOV_TIME_OFFSET_Y + 10

  display.setFont();
  /*
  display.setCursor(DISP_BOX_CAD_MOV_TIME_OFFSET_X, DISP_BOX_CAD_MOV_TIME_OFFSET_Y + 3);
  display.print("Cad Mov");
  display.setCursor(DISP_BOX_CAD_MOV_TIME_OFFSET_X, DISP_BOX_CAD_MOV_TIME_OFFSET_Y + 13);
  display.print("Time:");
*/
  display.setFont(&FreeSans12pt7b);
  unixSeconds = cadenceMovingTimeMS / 1000;
  strftime(tempDispString, 50, "%k:%M:%S", localtime(&unixSeconds));
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor(DISP_BOX_CAD_MOV_TIME_VALUE_X_OFFSET, DISP_BOX_CAD_MOV_TIME_OFFSET_Y + h1 + 4);
  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_CAD_MOV_TIME_OFFSET_X, DISP_BOX_CAD_MOV_TIME_OFFSET_Y + DISP_BOX_CAD_MOV_TIME_SIZE_Y, DISP_BOX_CAD_MOV_TIME_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_CAD_MOV_TIME_OFFSET_X + DISP_BOX_CAD_MOV_TIME_SIZE_X, DISP_BOX_CAD_MOV_TIME_OFFSET_Y, DISP_BOX_CAD_MOV_TIME_SIZE_Y, BLACK);

  /***** Average Cadence box drawing *****/
#define DISP_BOX_AV_CAD_OFFSET_X COLUMN_1_WIDTH
#define DISP_BOX_AV_CAD_OFFSET_Y DISP_BOX_CAD_MOV_TIME_OFFSET_Y+DISP_BOX_CAD_MOV_TIME_SIZE_Y
#define DISP_BOX_AV_CAD_SIZE_X COLUMN_2_WIDTH
#define DISP_BOX_AV_CAD_SIZE_Y 35
#define DISP_BOX_AV_CAD_VALUE_X_OFFSET DISP_BOX_AV_CAD_OFFSET_X + 45
#define DISP_BOX_AV_CAD_UNITS_X_OFFSET DISP_BOX_AV_CAD_OFFSET_X + DISP_BOX_AV_CAD_SIZE_X-20
#define DISP_BOX_AV_CAD_UNITS_Y_OFFSET DISP_BOX_AV_CAD_OFFSET_Y + 10

  display.setFont();
  /*
  display.setCursor(DISP_BOX_AV_CAD_OFFSET_X, DISP_BOX_AV_CAD_OFFSET_Y + 3);
  display.print("Av.");
  display.setCursor(DISP_BOX_AV_CAD_OFFSET_X, DISP_BOX_AV_CAD_OFFSET_Y + 13);
  display.print("Cad:");
*/
  display.setCursor(DISP_BOX_AV_CAD_UNITS_X_OFFSET, DISP_BOX_AV_CAD_UNITS_Y_OFFSET);
  display.print("rpm");

  display.setFont(&FreeSans12pt7b);
  sprintf(tempDispString, "%3.1f", 25.4/*averageCadence*/);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);

  display.setCursor(DISP_BOX_AV_CAD_VALUE_X_OFFSET, DISP_BOX_AV_CAD_OFFSET_Y + h1 + 4);
  display.print(tempDispString);

  display.drawFastHLine(DISP_BOX_AV_CAD_OFFSET_X, DISP_BOX_AV_CAD_OFFSET_Y + DISP_BOX_AV_CAD_SIZE_Y, DISP_BOX_AV_CAD_SIZE_X, BLACK);
  display.drawFastVLine(DISP_BOX_AV_CAD_OFFSET_X + DISP_BOX_AV_CAD_SIZE_X, DISP_BOX_AV_CAD_OFFSET_Y, DISP_BOX_AV_CAD_SIZE_Y, BLACK);


  /***** Time Box Drawing  *****/
#define DISP_BOX_TIME_OFFSET_X COLUMN_1_WIDTH + COLUMN_2_WIDTH
#define DISP_BOX_TIME_OFFSET_Y 60
#define DISP_BOX_TIME_SIZE_X COLUMN_3_WIDTH
#define DISP_BOX_TIME_SIZE_Y 60
  // time
  unixSeconds = currentUnixTime + (currentTimeZone * SECONDS_TO_HOURS);
  strftime(tempDispString, 50, "%k:%M", localtime(&unixSeconds));
  display.setFont(&FreeSans12pt7b);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  h1 = h1 + 5;
  display.setCursor(((DISP_BOX_TIME_SIZE_X - w1) / 2) + DISP_BOX_TIME_OFFSET_X, DISP_BOX_TIME_OFFSET_Y + h1);
  display.print(tempDispString);
  // temp
  sprintf(tempDispString, "%.1f 'C", temperature);
  display.setFont();
  display.getTextBounds(tempDispString, 0, 0, &x2, &y2, &w2, &h2);
  display.setCursor(((DISP_BOX_TIME_SIZE_X - w2) / 2) + DISP_BOX_TIME_OFFSET_X, DISP_BOX_TIME_OFFSET_Y + h1 + h2);
  display.print(tempDispString);
  // humidity
  sprintf(tempDispString, "%.f %%RH", humidity);
  display.getTextBounds(tempDispString, 0, 0, &x3, &y3, &w3, &h3);
  display.setCursor(((DISP_BOX_TIME_SIZE_X - w3) / 2) + DISP_BOX_TIME_OFFSET_X, DISP_BOX_TIME_OFFSET_Y + h1 + h2 + h3 + 3);
  display.print(tempDispString);
  // box
  display.drawFastHLine(DISP_BOX_TIME_OFFSET_X, DISP_BOX_TIME_OFFSET_Y, DISP_BOX_TIME_SIZE_X, BLACK);
  display.drawFastHLine(DISP_BOX_TIME_OFFSET_X, DISP_BOX_TIME_SIZE_Y + DISP_BOX_TIME_OFFSET_Y, DISP_BOX_TIME_SIZE_X, BLACK);

  /***** Heartrate Box Drawing  *****/
#define DISP_BOX_HR_OFFSET_X COLUMN_1_WIDTH + COLUMN_2_WIDTH
#define DISP_BOX_HR_OFFSET_Y DISP_BOX_TIME_OFFSET_Y+DISP_BOX_TIME_SIZE_Y
#define DISP_BOX_HR_SIZE_X COLUMN_3_WIDTH
#define DISP_BOX_HR_SIZE_Y 40
#define DISP_BOX_HR_VALUE_X DISP_BOX_HR_OFFSET_X+25
#define DISP_BOX_HR_UNITS_X DISP_BOX_HR_OFFSET_X+20

  display.setFont(&FreeSans12pt7b);
  sprintf(tempDispString, "%3.f", 125 /*heartRate*/);
  display.getTextBounds(tempDispString, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor(DISP_BOX_HR_VALUE_X, DISP_BOX_HR_OFFSET_Y + h1+3);
  display.print(tempDispString);
  
  display.setFont();
  display.setCursor(DISP_BOX_HR_UNITS_X, DISP_BOX_HR_OFFSET_Y + h1+7);
  display.print(" bpm");

  display.drawFastHLine(DISP_BOX_HR_OFFSET_X, DISP_BOX_HR_SIZE_Y + DISP_BOX_HR_OFFSET_Y, DISP_BOX_HR_SIZE_X, BLACK);




  /***** Android Music Box Drawing *****/
#define DISP_BOX_MUSIC_SIZE_X DISPLAY_WIDTH
#define DISP_BOX_MUSIC_SIZE_Y 25
#define DISP_BOX_MUSIC_OFFSET_X 0
#define DISP_BOX_MUSIC_OFFSET_Y (dispCurrentLowerOffset - DISP_BOX_MUSIC_SIZE_Y)

  if (musicInfoTrack[0] != 0)
  {
    display.drawFastHLine(0, DISP_BOX_MUSIC_OFFSET_Y, DISPLAY_WIDTH, BLACK);
    sprintf(tempDispString, "%s - %s", musicInfoTrack, musicInfoArtist);
    display.setFont(&FreeSans9pt7b);
    display.getTextBounds(tempDispString, 0, DISP_BOX_MUSIC_OFFSET_Y, &x1, &y1, &w1, &h1);
    //    display.print("Width: ");
    //    display.println(w1);
    //    display.print("Height: ");
    //    display.println(h1);
    if (w1 < DISP_BOX_MUSIC_SIZE_X)
    { // centre text!
      display.setCursor(((DISP_BOX_MUSIC_SIZE_X - w1) / 2), DISP_BOX_MUSIC_OFFSET_Y + h1);
    }
    else
    { // split text!  Or scroll text!
      // currently there is a bug here.. it doesn't display anything if off the screen...
      display.setCursor(0, DISP_BOX_MUSIC_OFFSET_Y + h1);
    }
    display.print(tempDispString);
    //Serial1.print(tempDispString);
    /*
      display.getTextBounds(musicState,0,DISPLAY_HEIGHT-25,&x3,&y3,&w3,&h3);
      display.setCursor(((DISPLAY_WIDTH - w3)/2), DISPLAY_HEIGHT-10);
      display.print(musicState);
    */
    dispCurrentLowerOffset -= DISP_BOX_MUSIC_SIZE_Y;
  }


  /***** Android Notification Box Drawing *****/
#define DISP_BOX_NOTIFY_SIZE_X DISPLAY_WIDTH
#define DISP_BOX_NOTIFY_SIZE_Y 40
#define DISP_BOX_NOTIFY_OFFSET_X 0
#define DISP_BOX_NOTIFY_OFFSET_Y (dispCurrentLowerOffset - DISP_BOX_MUSIC_SIZE_Y)
#define DISP_BOX_NOTIFY_TITLE_OFFSET_Y (DISP_BOX_NOTIFY_OFFSET_Y + 3)
#define DISP_BOX_NOTIFY_TITLE_UNDERLINE_OFFSET_Y (DISP_BOX_NOTIFY_OFFSET_Y + 12)
#define DISP_BOX_NOTIFY_BODY_OFFSET_Y (DISP_BOX_NOTIFY_OFFSET_Y + 15)
  if (notifyId[0] != 0)
  {
    display.drawFastHLine(0, DISP_BOX_NOTIFY_OFFSET_Y, DISPLAY_WIDTH, BLACK);
    sprintf(tempDispString, "%s - %s - %s", notifySource, notifyTitle, notifySender);
    display.setFont();
    display.getTextBounds(tempDispString, 0, DISP_BOX_NOTIFY_OFFSET_Y, &x1, &y1, &w1, &h1);
    display.setCursor(((DISP_BOX_NOTIFY_SIZE_X - w1) / 2), DISP_BOX_NOTIFY_TITLE_OFFSET_Y);
    display.print(tempDispString);
    display.drawFastHLine(((DISP_BOX_NOTIFY_SIZE_X - w1) / 2), DISP_BOX_NOTIFY_TITLE_UNDERLINE_OFFSET_Y, w1, BLACK);

    display.setCursor(0, DISP_BOX_NOTIFY_BODY_OFFSET_Y);
    display.getTextBounds(tempDispString, 0, DISP_BOX_NOTIFY_BODY_OFFSET_Y, &x1, &y1, &w1, &h1);
    if (w1 < DISP_BOX_NOTIFY_SIZE_X)
    {
      display.print(notifyBody);
    }
    else
    { // still need to sort out breaking sentence on the nearest space rather than in the middle of a word
      display.println(notifyBody);
      display.println(notifyBody + (strlen(notifyBody) / 2));
    }

    dispCurrentLowerOffset -= DISP_BOX_NOTIFY_SIZE_Y;
  }


  /***** Android Incoming Calls Box Drawing *****/
#define DISP_BOX_CALLS_SIZE_X DISPLAY_WIDTH
#define DISP_BOX_CALLS_SIZE_Y 25
#define DISP_BOX_CALLS_OFFSET_X 0
#define DISP_BOX_CALLS_OFFSET_Y (dispCurrentLowerOffset - DISP_BOX_CALLS_SIZE_Y)
#define DISP_BOX_CALLS_TEXT_OFFSET_Y (DISP_BOX_NOTIFY_OFFSET_Y + 10)

  if (strcmp(callCmd, "incoming") == 0)
  {

    display.drawFastHLine(0, DISP_BOX_CALLS_OFFSET_Y, DISPLAY_WIDTH, BLACK);
    sprintf(tempDispString, "Incoming: %s", callName);
    display.setFont(&FreeSans9pt7b);
    display.getTextBounds(tempDispString, 0, DISP_BOX_CALLS_TEXT_OFFSET_Y, &x1, &y1, &w1, &h1);
    display.setCursor(((DISP_BOX_CALLS_SIZE_X - w1) / 2), DISP_BOX_NOTIFY_OFFSET_Y + h1);
    display.print(tempDispString);
    dispCurrentLowerOffset -= DISP_BOX_CALLS_SIZE_Y;
  }

  else if (strcmp(callCmd, "accept") == 0)
  {
    display.drawFastHLine(0, DISP_BOX_CALLS_OFFSET_Y, DISPLAY_WIDTH, BLACK);
    sprintf(tempDispString, "Accept?: %s", callName);
    display.setFont(&FreeSans9pt7b);
    display.getTextBounds(tempDispString, 0, DISP_BOX_CALLS_TEXT_OFFSET_Y, &x1, &y1, &w1, &h1);
    display.setCursor(((DISP_BOX_CALLS_SIZE_X - w1) / 2), DISP_BOX_NOTIFY_OFFSET_Y + h1);
    display.print(tempDispString);
    dispCurrentLowerOffset -= DISP_BOX_CALLS_SIZE_Y;
  }

  else if (strcmp(callCmd, "start") == 0)
  {
    display.drawFastHLine(0, DISP_BOX_CALLS_OFFSET_Y, DISPLAY_WIDTH, BLACK);
    sprintf(tempDispString, "Call underway: %s", callName);
    display.setFont(&FreeSans9pt7b);
    display.getTextBounds(tempDispString, 0, DISP_BOX_CALLS_TEXT_OFFSET_Y, &x1, &y1, &w1, &h1);
    display.setCursor(((DISP_BOX_CALLS_SIZE_X - w1) / 2), DISP_BOX_NOTIFY_OFFSET_Y + h1);
    display.print(tempDispString);
    dispCurrentLowerOffset -= DISP_BOX_CALLS_SIZE_Y;
  }
/*
  display.setFont();
  display.setCursor(0, DISPLAY_HEIGHT - 100);
  display.print("Debug: ");
  display.println(debugString);
*/

  /***** Compass Drawing *****/
  // point 1 at magnet_heading
#define COMPASS_TRIANGLE_RADIUS 15
#define COMPASS_X_OFFSET DISPLAY_WIDTH-COMPASS_TRIANGLE_RADIUS-20
#define COMPASS_Y_OFFSET COMPASS_TRIANGLE_RADIUS+5

  // circle centre at COMPASS_X_OFFSET, COMPASS_Y_OFFSET
  display.drawCircle(COMPASS_X_OFFSET, COMPASS_Y_OFFSET, COMPASS_TRIANGLE_RADIUS, BLACK);

  x1 = (COMPASS_TRIANGLE_RADIUS + 5) * cos(magnet_heading) + COMPASS_X_OFFSET;
  y1 = (COMPASS_TRIANGLE_RADIUS + 5) * sin(magnet_heading) + COMPASS_Y_OFFSET;
  // point 2 at magnet_heading + 2*PI/3
  float_t heading2 = magnet_heading + 2 * PI / 3;
  if (heading2 > 2 * PI)
    heading2 -= 2 * PI;
  x2 = COMPASS_TRIANGLE_RADIUS * cos(heading2) + COMPASS_X_OFFSET;
  y2 = COMPASS_TRIANGLE_RADIUS * sin(heading2) + COMPASS_Y_OFFSET;
  float_t heading3 = magnet_heading + 4 * PI / 3;
  if (heading3 > 2 * PI)
    heading3 -= 2 * PI;
  x3 = COMPASS_TRIANGLE_RADIUS * cos(heading3) + COMPASS_X_OFFSET;
  y3 = COMPASS_TRIANGLE_RADIUS * sin(heading3) + COMPASS_Y_OFFSET;
  display.fillTriangle(x1, y1, x2, y2, x3, y3, BLACK);

  // print compass heading
  display.setCursor(COMPASS_X_OFFSET - 5, COMPASS_Y_OFFSET + COMPASS_TRIANGLE_RADIUS + 7);

  char cardinal[10];
  generateCardinal(magnet_heading + (2 * PI / 3), cardinal);
  display.print(cardinal);


  /***** Thread Housekeeping *****/

  display.refresh(); // 29ms refresh time
  thread_sleep_for(500); // ms

  //delay(50); // will put micro into power saving mode... but this not good if have multiple threads!


  /***** Gyro, Acceleration Drawing *****/
  /*
    display.print("Gyroscope      x: ");
    display.println(gyro_x);
    display.print("               y: ");
    display.println(gyro_y);
    display.print("               z: ");
    display.println(gyro_z);


    //display.println("");

    display.print("Acceleration   x: ");
    display.println(accel_x);
    display.print("               y: ");
    display.println(accel_y);
    display.print("               z: ");
    display.println(accel_z);
  */

}

/* BLE Servicing Thread
    needs to be called about every 10ms otherwise when sending lots of data it will crash the processor
    Need to look further into driver to figure out why.

*/
void bleServicing()
{
  while (true)
  {
    //bleSerial.poll();
    // calling if(bleSerial) returns BLE.connected() which calls as poll() deep in BLE driver!!

    // if(BLE.central()) is actually the command to check if Android Gadgetbridge is connected.
    if (bleSerial && bleSerial.availableLines() >= 1 && !lineAvailable) {
      bleSerial.readLine(bleString, sizeof(bleString));
      lineAvailable = true;
    }
    thread_sleep_for(BLE_POLL_INTERVAL_MS); // ms
  }
}



/* Parse any incoming notifications from Android/GadgetBridge
  takes about 7ms to execute including dumping string out serial port
*/
void parseBLESerial()
{
  if (lineAvailable)
  {
    Serial1.println(bleString); // 4ms to load string and send out serial port

    // return pointer to string within string
    char * timeSetStrPtr = strstr(bleString, set_time_string);
    char * jsonStrPtr = strstr(bleString, GB_string);

    if (timeSetStrPtr != NULL)  // strncmp(bleString + 3, set_time_string, 8) == 0)
    {
      // it's a set time command, unix time format!
      // lets convert the time to a proper number!
      currentUnixTime = atoi(timeSetStrPtr + 8); //bleString + 11);
      // return pointer to string within string
      char * timeZoneSetStrPtr = strstr(bleString, set_time_zone_string);
      if (timeZoneSetStrPtr != NULL)
      {
        currentTimeZone = atoi(timeZoneSetStrPtr + 14); // bleString + 37);
      }
      strcpy(debugString, "Time sync sucessful\0");
    }

    else if (jsonStrPtr != NULL)
    {
      DeserializationError dError = deserializeJson(jsonDoc, jsonStrPtr + 3);
      if (!dError)
      {
        strcpy(debugString, "json decoded sucessfully");
        const char * jType = jsonDoc["t"];

        if (strcmp(jType, "notify") == 0)
        {
          // fields are "id", "src", "title", "body" , "sender", "tel"
          // "src" can be "Hangouts" etc
          strcpy(notifyId, jsonDoc["id"]);
          strcpy(notifySource, jsonDoc["src"]);
          strcpy(notifyTitle, jsonDoc["title"]);
          strncpy(notifyBody, jsonDoc["body"], 200);
          strcpy(notifySender, jsonDoc["sender"]); // this doesn't seem to work...
        }
        else if (strcmp(jType, "notify-") == 0)
        {
          notifyId[0] = 0;
          notifySource[0] = 0;
          notifyTitle[0] = 0;
          notifyBody[0] = 0;
          notifySender[0] = 0;
        }
        else if (strcmp(jType, "musicinfo") == 0)
        {
          // fields are "artist", "album", "track", "dur", "c", "n"
          strcpy(musicInfoArtist, jsonDoc["artist"]);
          strcpy(musicInfoTrack, jsonDoc["track"]);
        }
        else if (strcmp(jType, "musicstate") == 0)
        {
          // fields are "state", "position", "shuffle", "repeat"
          strcpy(musicState, jsonDoc["state"]);
        }
        else if (strcmp(jType, "call") == 0)
        {
          // fields are "cmd", "name", "number",
          strcpy(callCmd, jsonDoc["cmd"]);
          strcpy(callName, jsonDoc["name"]);
          strcpy(callNumber, jsonDoc["number"]);
          if (strcmp(callCmd, "reject") == 0)
          {
            callCmd[0] = 0;
          }
        }
      }
      else {
        // what is the deserialization error?
        if (dError == DeserializationError::IncompleteInput)
        {
          strcpy(debugString, "Incomplete Input");
        }
        else if (dError == DeserializationError::InvalidInput)
        {
          strcpy(debugString, "Invalid Input");
        }
        else if (dError == DeserializationError::NoMemory)
        {
          strcpy(debugString, "No Memory");
        }
        else if (dError == DeserializationError::NotSupported)
        {
          strcpy(debugString, "Not Supported");
        }
        else if (dError == DeserializationError::TooDeep)
        {
          strcpy(debugString, "Too Deep");
        }
        else
        {
          strcpy(debugString, "json decode unsuccessful");
        }
      }
    }
    bleString[0] = '\0';
    lineAvailable = false;
  }
}

/* Calculate a Cardinal direction from the bearing (in Radians)

*/
void generateCardinal(float_t bearing, char *outputArray)
{
#define COMPASS_N 0
#define COMPASS_NE PI/4
#define COMPASS_E PI/2
#define COMPASS_SE ((PI/2) + (PI/4))
#define COMPASS_S PI
#define COMPASS_SW (PI + (PI/4))
#define COMPASS_W (PI + (PI/2))
#define COMPASS_NW ((2*PI)-(PI/4))
#define COMPASS_SPAN (PI/8)

  if (bearing > 2 * PI && bearing < 4 * PI)
  {
    bearing -= 2 * PI;
  }

  if (bearing > 2 * PI)
  {
    sprintf(outputArray, "Max");
  }
  else if (bearing < 0)
  {
    sprintf(outputArray, "Neg");
  }

  else if ((magnet_heading > COMPASS_NW + COMPASS_SPAN) && (magnet_heading <= COMPASS_N + COMPASS_SPAN))
  {
    sprintf(outputArray, "N");
  }
  else if ((magnet_heading > COMPASS_NE - COMPASS_SPAN) && (magnet_heading <= COMPASS_NE + COMPASS_SPAN))
  {
    sprintf(outputArray, "N-E");
  }
  else if ((magnet_heading > COMPASS_E - COMPASS_SPAN) && (magnet_heading <= COMPASS_E + COMPASS_SPAN))
  {
    sprintf(outputArray, "E");
  }
  else if ((magnet_heading > COMPASS_SE - COMPASS_SPAN) && (magnet_heading <= COMPASS_SE + COMPASS_SPAN))
  {
    sprintf(outputArray, "S-E");
  }
  else if ((magnet_heading > COMPASS_S - COMPASS_SPAN) && (magnet_heading <= COMPASS_S + COMPASS_SPAN))
  {
    sprintf(outputArray, "S");
  }
  else if ((magnet_heading > COMPASS_SW - COMPASS_SPAN) && (magnet_heading <= COMPASS_SW + COMPASS_SPAN))
  {
    sprintf(outputArray, "S-W");
  }
  else if ((magnet_heading > COMPASS_W - COMPASS_SPAN) && (magnet_heading <= COMPASS_W + COMPASS_SPAN))
  {
    sprintf(outputArray, "W");
  }
  else if ((magnet_heading > COMPASS_NW - COMPASS_SPAN) && (magnet_heading <= COMPASS_NW + COMPASS_SPAN))
  {
    sprintf(outputArray, "N-W");
  }
}


/*
     Call regulary (before RTC timer overflows) to updated timebase variable.
     Time will overflow after 8.5 minutes.. no need to call very often!
     - less than 1ms to execute
     NRF_RTC1->PRESCALER = 0 // as set by mbed library
     RTC_COUNTER_SIZE / (LFCLK_FREQUENCY / (NRF_RTC1->PRESCALER+1)) = 512 seconds = 8.5 minutes

*/
void updateRTC()
{
  uint32_t increment = 0;

  if (prevRTCCounter < NRF_RTC1->COUNTER) // see if value is more than 1 second...
  {
    // the standard case
    if ((prevRTCCounter + RTC_ONE_SECOND) < NRF_RTC1->COUNTER)
    {
      // more than one second has passed, increment counter
      increment = (NRF_RTC1->COUNTER - prevRTCCounter) / RTC_ONE_SECOND;
      currentUnixTime = currentUnixTime + increment;
      prevRTCCounter = (prevRTCCounter + (increment * RTC_ONE_SECOND)) % RTC_COUNTER_SIZE;

    }
  }
  else
  {
    // the odd case... NRF_RTC1->COUNTER has reset, prevRTCounter hasn't yet
    uint32_t artificalCounter = RTC_COUNTER_SIZE + NRF_RTC1->COUNTER;
    if ((prevRTCCounter + RTC_ONE_SECOND) < artificalCounter)
    {
      // more than one second has passed, increment counter
      increment = (artificalCounter - prevRTCCounter) / RTC_ONE_SECOND;
      currentUnixTime = currentUnixTime + increment;
      prevRTCCounter = (prevRTCCounter + (increment * RTC_ONE_SECOND)) % RTC_COUNTER_SIZE;
    }
  }
}

/* Grab and process data from internal sensors (acc, gyro etc) and update global variables

*/
void updateLocalSensors()
{
  //pressure = BARO.readPressure(MILLIBAR); // convert kPa to hPa - this seems to slow things down a lot... check execution time!
  //altitude = 44330 * (1 - pow(((pressure) / referencePressure), (1 / 5.255)));  // occasionally getting NaN from it - something wrong here?
#define ALT_A (101325.0)
#define ALT_B (-2.25577E-05)
#define ALT_C (5.25588)
  //  altitude = ((pow((pressure/ALT_A),(1/ALT_C))-1)/ALT_B);// too variable!

  temperature = HTS.readTemperature();
  humidity    = HTS.readHumidity();

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magnet_x, magnet_y, magnet_z);
    magnet_heading = atan2(magnet_x, magnet_y);
    if (magnet_heading < 0)
    {
      magnet_heading += 2 * PI;
    }
    //magnet_heading = magnet_heading * 180/PI; // convert to degrees
  }
}

#define RPM_CONV_NUMERATOR (60*1024)
#define SPEED_CONV_NUMERATOR (60*60)
#define SPEED_CONV_DENOMINATOR 1024
#define TIME_1024_TO_MS_NUMERATOR (1000)
#define TIME_1024_TO_MS_DENOMINATOR (1024)
#define NUM_CSC_BYTES_READ 7
#define MAX_16_BIT_NUM 65535
#define MS_TO_S (1000)
#define MS_TO_HOURS (1000*60*60)


/* Grab and process data from BLE sensors (cadence, speed etc)

*/
void updateBLESensors()
{
  uint32_t bytesRead;
  uint8_t cadenceValue[NUM_CSC_BYTES_READ];
  uint32_t cadenceTime;
  uint32_t cadenceRev;
  float cadenceRevDiff = 0;
  float cadenceTimeDiff = 0;

  uint8_t speedValue[NUM_CSC_BYTES_READ];
  uint32_t speedTime;
  uint32_t speedRev;
  float speedRevDiff = 0;
  float speedTimeDiff = 0;

  if (cadenceCSCMeasurementCharacteristic)
  {
    if (cadenceCSCMeasurementCharacteristic.valueUpdated())
    {
      bytesRead = cadenceCSCMeasurementCharacteristic.readValue(cadenceValue, NUM_CSC_BYTES_READ);
      cadenceRev = (uint16_t)cadenceValue[2] << 8 | (uint16_t)cadenceValue[1];
      cadenceTime = (uint16_t)cadenceValue[4] << 8 | (uint16_t)cadenceValue[3];
      if (cadenceTimePrev == 0)
      {
        cadenceTimePrev = cadenceTime; // if it is the first sample, don't use it
      }
      else if (cadenceTime > cadenceTimePrev)
      {
        cadenceTimeDiff = cadenceTime - cadenceTimePrev;
        cadenceTimePrev = cadenceTime;
      }
      else
      {
        cadenceTimeDiff = cadenceTime + (MAX_16_BIT_NUM - cadenceTimePrev);
        cadenceTimePrev = cadenceTime;
      }
      if (cadenceRevPrev == 0)
      {
        cadenceRevPrev = cadenceRev; // if it is the first sample, don't use it
      }
      else if (cadenceRev > cadenceRevPrev)
      {
        cadenceRevDiff = cadenceRev - cadenceRevPrev;
        cadenceRevPrev = cadenceRev;
      }
      else
      {
        cadenceRevDiff = cadenceTime + (MAX_16_BIT_NUM - cadenceRevPrev);
        cadenceRevPrev = cadenceRev;
      }
      if (cadenceTimeDiff != 0)
      {
        cadenceRpm = (RPM_CONV_NUMERATOR * cadenceRevDiff) / cadenceTimeDiff;
        cadenceOdometer += cadenceRevDiff;
        cadenceMovingTimeMS += (TIME_1024_TO_MS_NUMERATOR * cadenceTimeDiff) / TIME_1024_TO_MS_DENOMINATOR;
        averageCadence = (MS_TO_S * cadenceOdometer) / cadenceMovingTimeMS;
      }
    }
    if (cadenceBatteryCharacteristic.valueUpdated())
    {
      cadenceBatteryCharacteristic.readValue(cadenceBattValue);
    }
  }
  if (speedCSCMeasurementCharacteristic)
  {
    if (speedCSCMeasurementCharacteristic.valueUpdated())
    {
      bytesRead = speedCSCMeasurementCharacteristic.readValue(speedValue, NUM_CSC_BYTES_READ);
      speedRev = (uint16_t)speedValue[2] << 8 | (uint16_t)speedValue[1];
      speedTime = (uint16_t)speedValue[6] << 8 | (uint16_t)speedValue[5];
      if (speedTimePrev == 0)
      {
        speedTimePrev = speedTime; // if it is the first sample, don't use it
      }
      else if (speedTime >= speedTimePrev)
      {
        speedTimeDiff = speedTime - speedTimePrev;
        speedTimePrev = speedTime;
      }
      else
      {
        speedTimeDiff = speedTime + (MAX_16_BIT_NUM - speedTimePrev);
        speedTimePrev = speedTime;
      }
      if (speedRevPrev == 0)
      {
        speedRevPrev = speedRev; // if it is the first sample, don't use it for calc
      }
      else if (speedRev >= speedRevPrev)
      {
        speedRevDiff = speedRev - speedRevPrev;
        speedRevPrev = speedRev;
      }
      else
      {
        speedRevDiff = speedRev + (MAX_16_BIT_NUM - speedRevPrev);
        speedRevPrev = speedRev;
      }
      if (speedTimeDiff != 0)
      {
        speedKph = (SPEED_CONV_NUMERATOR * TIRE_CIRCUMFERENCE * speedRevDiff) / (SPEED_CONV_DENOMINATOR * speedTimeDiff);
        odometer += (TIRE_CIRCUMFERENCE * speedRevDiff) / MM_TO_KM;
        movingTimeMS += (TIME_1024_TO_MS_NUMERATOR * speedTimeDiff) / TIME_1024_TO_MS_DENOMINATOR;
        averageSpeed = (MS_TO_HOURS * odometer) / (movingTimeMS);
      }
      if (speedKph > maxSpeedKph)
      {
        maxSpeedKph = speedKph;
      }
    }
    if (speedBatteryCharacteristic.valueUpdated())
    {
      speedBatteryCharacteristic.readValue(speedBattValue);
    }
  }
}
/// EOF
