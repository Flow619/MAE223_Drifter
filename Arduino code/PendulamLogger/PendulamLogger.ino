

//This is code actually written by us using snips from other example codes
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "AK09918.h"
#include "ICM20600.h"

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL 250  // mills between entries (reduce to take more/faster data)
// how many millisecconds you want to log data for. 300000 is five minutes
#define logDuration 120000
// how many milliseconds before writing the logged data permanently to disk
#define SYNC_INTERVAL 10000    // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;         // time of last sync()
unsigned long lastSample = 0;  //time after last sample

// Define Pins
const int redLEDpin = 3;
const int greenLEDpin = 2;  //not sure about these, maybe switch if theyre wrong

const int chipSelect = 8;  // for the data logging shield, we use a jumper to pin 8
const int IMUpower = 19;   //power for the Accelerometer
const int IMUground = 18;  // ground for the accelerometer

const bool ECHO_TO_SERIAL = 1;  //set to 0 to stop communication over serial line

RTC_PCF8523 RTC;  // define the Real Time Clock object
File logfile;     // define the logging file object

// acceleramter stuff.
AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int32_t offset_x, offset_y, offset_z;
double roll, pitch;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_lajolla = +11.0;

int16_t gyroX, gyroY, gyroZ;  // defining gryo varibles not called in example code






void setup() {
  // adding hardcoded offset data we got from a few calabrations. We did this to
  //avoid running a calabration on every startup
  offset_x = -10;
  offset_y = -54;
  offset_z = 0;


  Serial.begin(9600);
  Serial.println();

  //setup sd card chip select
  pinMode(chipSelect, OUTPUT);
  pinMode(10, OUTPUT);  //example code says to set pin 10 as an output even if its not used.

  //setup debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  //setup and send power up the IMU
  pinMode(IMUpower, OUTPUT);
  pinMode(IMUground, OUTPUT);
  digitalWrite(IMUground, 0);
  digitalWrite(IMUpower, 1);
  delay(20);  // give some time for power to move before sending commands

  Wire.begin();  //initialize spi and i2c communication

  //initialize the 2 chips on the 9 axis in high power mode
  err = ak09918.initialize();
  icm20600.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[] = "LOG00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = i / 10 + '0';
    filename[4] = i % 10 + '0';
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (!logfile) {
    error("couldnt create file");
  }
  Serial.print("Logging to: ");
  Serial.println(filename);
  //connect to RTC
  if (!RTC.begin()) {
    logfile.println("RTC failed");
    if (ECHO_TO_SERIAL == 1) {  //ECHO_TO_SERIAL
      Serial.println("RTC failed");
    }
  }
  logfile.println("millis,stamp,datetime,accX,accY,accZ,gyroX,gyroY,gyroZ,compX,compY,compZ");
  if (ECHO_TO_SERIAL == 1) {  //ECHO_TO_SERIAL
    Serial.println("millis,stamp,datetime,accX,accY,accZ,gyroX,gyroY,gyroZ,compX,compY,compZ");
  }
  // IMU axis stuff
  err = ak09918.isDataReady();
  while (err != AK09918_ERR_OK) {
    Serial.println("Waiting Sensor");
    delay(100);
    //ak09918.getData(&x, &y, &z); //I think we dont need this but it was working with it. worth testing
    err = ak09918.isDataReady();
    digitalWrite(redLEDpin, HIGH);
  }

  lastSample = millis();
}

void loop() {

  // only tate another sample if it's been long enough
  if ((millis() - lastSample) < LOG_INTERVAL) return;


  lastSample = millis();            // this sets the time at the start of the sample
  DateTime now;                     // this just changes DateTime command into now i think
  digitalWrite(greenLEDpin, HIGH);  //turn on green LED so we know we are logging

  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);  // milliseconds since start
  logfile.print(", ");
  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime());  // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  // get acceleration
  acc_x = icm20600.getAccelerationX();
  acc_y = icm20600.getAccelerationY();
  acc_z = icm20600.getAccelerationZ();
  // get gryo
  gyroX = icm20600.getGyroscopeX();
  gyroY = icm20600.getGyroscopeY();
  gyroZ = icm20600.getGyroscopeZ();
  // get compass
  ak09918.getData(&x, &y, &z);
  x = x - offset_x;
  y = y - offset_y;
  z = z - offset_z;
  //log acceleration
  logfile.print(",  ");
  logfile.print(acc_x);
  logfile.print(",  ");
  logfile.print(acc_y);
  logfile.print(",  ");
  logfile.print(acc_z);
  logfile.print("");
  //log gyro
  logfile.print(",  ");
  logfile.print(gyroX);
  logfile.print(",  ");
  logfile.print(gyroY);
  logfile.print(",  ");
  logfile.print(gyroZ);
  logfile.print("");
  //log compass
  logfile.print(",  ");
  logfile.print(x);
  logfile.print(",  ");
  logfile.print(y);
  logfile.print(",  ");
  logfile.print(z);
  logfile.println("");

  if (ECHO_TO_SERIAL == 1) {  //ECHO_TO_SERIAL
    Serial.print(m);          // milliseconds since start
    Serial.print(", ");
    Serial.print(now.unixtime());
    Serial.print(", ");
    Serial.print('"');
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.print('"');
    Serial.print(",   ");
    Serial.print(acc_x);
    Serial.print(",  ");
    Serial.print(acc_y);
    Serial.print(",  ");
    Serial.print(acc_z);
    Serial.print(",   ");
    Serial.print(gyroX);
    Serial.print(",  ");
    Serial.print(gyroY);
    Serial.print(",  ");
    Serial.print(gyroZ);
    Serial.print(",   ");
    Serial.print(x);
    Serial.print(",  ");
    Serial.print(y);
    Serial.print(",  ");
    Serial.print(z);
    Serial.println("");
  }
  digitalWrite(greenLEDpin, LOW);  //green led off to show no longer logging

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink RED LED to show we are syncing data to the card & updating!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);

  //this ends the code and closes the file after the set log duration
  if (millis() > logDuration) {
    logfile.close();
    digitalWrite(redLEDpin, HIGH);
    digitalWrite(greenLEDpin, HIGH);
    while (1)
      ;
  }
}

void error(char *str) {
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while (1)
    ;
}