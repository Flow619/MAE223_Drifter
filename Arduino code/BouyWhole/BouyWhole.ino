

//This is code started from Pendulam Logger and trying to inpliment the whole code
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "AK09918.h"
#include "ICM20600.h"
#include "BotleticsSIM7000.h"



// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL 200  // mills between entries (reduce to take more/faster data)
// how many millisecconds you want to log data for. 300000 is five minutes
#define logDuration 180000
// how many milliseconds before writing the logged data permanently to disk
#define SYNC_INTERVAL 10000    // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;         // time of last sync()
unsigned long lastSample = 0;  //time after last sample

// Define Pins
const int redLEDpin = 3;
const int greenLEDpin = 2;  // two led pins on sheild for debugging
const int chipSelect = 8;   // for the data logging shield, we use a jumper to pin 8
const int IMUpower = 19;    //power for the Accelerometer
const int IMUground = 18;   // ground for the accelerometer
const int PWRKEY = 6;
const int RST = 7;
#define TX 10  // Microcontroller RX
#define RX 11  // Microcontroller TX

#include <SoftwareSerial.h>
SoftwareSerial modemSS = SoftwareSerial(TX, RX);
SoftwareSerial *modemSerial = &modemSS;
Botletics_modem_LTE modem = Botletics_modem_LTE();


const bool ECHO_TO_SERIAL = 1;  //set to 0 to stop communication over serial line
bool connectFlag = 0;

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

// define varaibles for reading gps
char imei[16] = { 0 };   // Use this for device ID
uint16_t battLevel = 0;  // Battery level (percentage)
float latitude, longitude, speed_kph, heading, altitude, second;
uint8_t counter = 0;
char URL[200];  // Make sure this is long enough for your request URL
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12], headBuff[12], altBuff[12], tempBuff[12], battBuff[12];




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



  //setup mobile chip
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);  // Default state
  pinMode(PWRKEY, OUTPUT);
  delay(20);  // give some time for power to move before sending commands
              // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(PWRKEY);  // Power on the module
  moduleSetup();          // Establishes first-time serial comm and prints IMEI

  // Set modem to full functionality
  modem.setFunctionality(1);                // AT+CFUN=1
  modem.setNetworkSettings(F("hologram"));  // For Hologram SIM card
  modem.enableGPS(true);                    //Turn GPS on
  modem.enableGPRS(true);                   // Enable data




  Wire.begin();  //initialize spi and i2c communication

  //initialize the 2 chips on the 9 axis in high power mode
  err = ak09918.initialize();
  icm20600.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  // see if the card is present and can be initialized:
  SD.begin(chipSelect);
  Serial.println("card initialized.");


  //connect to RTC
  if (!RTC.begin()) {
    logfile.println("RTC failed");
    if (ECHO_TO_SERIAL == 1) {  //ECHO_TO_SERIAL
      Serial.println("RTC failed");
    }
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

  // create a new file
  DateTime now;  // this just changes DateTime command into now i think
  now = RTC.now();
  char bufDate[] = "DDxhhxmm";
  String filename = now.toString(bufDate);
  filename.concat(".csv");
  logfile = SD.open(filename, FILE_WRITE);
  Serial.print("Logging to: ");
  Serial.println(filename);
  logfile.println("millis,stamp,datetime,accX,accY,accZ,gyroX,gyroY,gyroZ,compX,compY,compZ");
  if (ECHO_TO_SERIAL == 1) {  //ECHO_TO_SERIAL
    Serial.println("millis,stamp,datetime,accX,accY,accZ,gyroX,gyroY,gyroZ,compX,compY,compZ");
  }
  long TimeOpen = millis();
  while ((millis() - TimeOpen) < logDuration) {  //write to this file for logDuration

    // only tate another sample if it's been long enough
    if ((millis() - lastSample) >= LOG_INTERVAL) {
      lastSample = millis();            // this sets the time at the start of the sample
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
        Serial.print(m);
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
      if ((millis() - syncTime) > SYNC_INTERVAL) {
        syncTime = millis();

        // blink RED LED to show we are syncing data to the card & updating!
        digitalWrite(redLEDpin, HIGH);
        logfile.flush();
        digitalWrite(redLEDpin, LOW);
      }
    }
  }
  //This is outside the the log duration loop
  logfile.close();
  digitalWrite(redLEDpin, HIGH);
  digitalWrite(greenLEDpin, HIGH);
  Serial.println("outside loop");
  if (connectFlag == false) {
    moduleSetup();
  }

  if (modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    if (ECHO_TO_SERIAL == 1) {
      Serial.println(F("Found location"));
      Serial.println(F("---------------------"));
      Serial.print(F("Latitude: "));
      Serial.println(latitude, 6);
      Serial.print(F("Longitude: "));
      Serial.println(longitude, 6);
      Serial.print(F("Speed: "));
      Serial.println(speed_kph);
      Serial.print(F("Heading: "));
      Serial.println(heading);
      Serial.print(F("Altitude: "));
      Serial.println(altitude);
      Serial.println(F("---------------------"));
    }
    // Format the floating point numbers
    dtostrf(latitude, 1, 6, latBuff);
    dtostrf(longitude, 1, 6, longBuff);
    dtostrf(speed_kph, 1, 0, speedBuff);
    dtostrf(heading, 1, 0, headBuff);
    dtostrf(altitude, 1, 1, altBuff);

    // GET request
    counter = 0;  // This counts the number of failed attempts tries

    sprintf(URL, "http://dweet.io/dweet/for/%s?lat=%s&long=%s&speed=%s&head=%s&alt=%s", imei, latBuff, longBuff,
            speedBuff, headBuff, altBuff);

    while (counter < 3 && !modem.postData("GET", URL)) {
      Serial.println(F("Failed to post data, retrying..."));
      counter++;  // Increment counter
      delay(100);
    }
  }

  Serial.println("starting again");
}

void moduleSetup() {
  // SIM7000 takes about 3s to turn on and SIM7500 takes about 15s
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
  modemSS.begin(115200);  // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  modemSS.println("AT+IPR=9600");  // Set baud rate
  delay(100);                      // Short pause to let the command run
  modemSS.begin(9600);
  if (!modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    connectFlag = 0;
    return;  // Don't proceed if it couldn't find the device
  }
  Serial.println(F("Modem is OK"));
  Serial.print(F("Found "));
  Serial.println(F("SIM7000"));
  connectFlag = 1;
  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: ");
    Serial.println(imei);
  }
}
