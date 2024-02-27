#include "BotleticsSIM7000.h"
#define SIMCOM_7000
#define PROTOCOL_HTTP_POST

// For botletics SIM7000 shield
#define PWRKEY 6
#define RST 7
//#define DTR 8 // Connect with solder jumper
//#define RI 9 // Need to enable via AT commands
#define TX 10  // Microcontroller RX
#define RX 11  // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

// this is a large buffer for replies
//char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial modemSS = SoftwareSerial(TX, RX);

SoftwareSerial *modemSerial = &modemSS;

Botletics_modem_LTE modem = Botletics_modem_LTE();

#define samplingRate 30

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

char imei[16] = { 0 };   // Use this for device ID
uint16_t battLevel = 0;  // Battery level (percentage)
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t counter = 0;

char URL[200];  // Make sure this is long enough for your request URL
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
  headBuff[12], altBuff[12], tempBuff[12], battBuff[12];

void setup() {
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);  // Default state

  pinMode(PWRKEY, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("Trying to Dweet location"));

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(PWRKEY);  // Power on the module
  moduleSetup();          // Establishes first-time serial comm and prints IMEI

  // Set modem to full functionality
  modem.setFunctionality(1);                // AT+CFUN=1
  modem.setNetworkSettings(F("hologram"));  // For Hologram SIM card
  modem.enableGPS(true);                    //Turn GPS on
  modem.enableGPRS(true);                   // Enable data

  // Connect to cell network and verify connection
  // If unsuccessful, keep retrying every 2s until a connection is made
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000);  // Retry every 2s
  }
  Serial.println(F("Connected to cell network!"));
}

void loop() {

  while (!modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000);  // Retry every 2s
  }
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
    delay(1000);
  }
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
    while (1)
      ;  // Don't proceed if it couldn't find the device
  }
  Serial.println(F("Modem is OK"));
  Serial.print(F("Found "));
  Serial.println(F("SIM7000"));

  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: ");
    Serial.println(imei);
  }
}
bool netStatus() {
  int n = modem.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}