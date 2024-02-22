#include "BotleticsSIM7000.h"
#define SIMCOM_7000

// For botletics SIM7000 shield
#define PWRKEY 6
#define RST 7
//#define DTR 8 // Connect with solder jumper
//#define RI 9 // Need to enable via AT commands
#define TX 10  // Microcontroller RX
#define RX 11  // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial modemSS = SoftwareSerial(TX, RX);

SoftwareSerial *modemSerial = &modemSS;

Botletics_modem_LTE modem = Botletics_modem_LTE();

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = { 0 };  // MUST use a 16 character buffer for IMEI!

void setup() {
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);  // Default state

  pinMode(PWRKEY, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("Trying to Dweet location"));

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(PWRKEY);  // Power on the module

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

  type = modem.type();
  uint8_t imeiLen = modem.getIMEI(imei);

  // Set modem to full functionality
  modem.setFunctionality(1);  // AT+CFUN=1

  modem.setNetworkSettings(F("hologram"));  // For Hologram SIM card

  modem.enableGPS(true);   //Turn GPS on
  modem.enableGPRS(true);  // Enable data
  myDweet("network enabled");
}

void loop() {
  delay(10000);
  float latitude, longitude, speed_kph, heading, altitude, second;
  if (modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {  // Use this line instead if you don't want UTC time
    Serial.println(F("---------------------"));
    //Serial.print(F("Latitude: "));
    //Serial.println(latitude, 6);
    //Serial.print(F("Longitude: "));
    //Serial.println(longitude, 6);
    String myLocation = "Latitude: " + String(latitude, 6) + "    Longitude: " +String(longitude, 6);
    Serial.println(myLocation);
    myDweet(myLocation);
  }
  else {
   // myDweet("no GPS");
   Serial.println("no GPS yet");
  }
}

void myDweet(String text) {
  // Post data to website via 2G or LTE CAT-M/NB-IoT

  uint16_t battLevel;
  if (!modem.getBattVoltage(&battLevel)) battLevel = 3800;  // Use dummy voltage if can't read


  // Make sure these buffers are long enough for your request URL
  char URL[150];


  // Construct the appropriate URL's and body, depending on request type
  // Use IMEI as device ID for this example

  // GET request
  sprintf(URL, "dweet.io/dweet/for/%s?location=%s&batt=%i", imei, text, battLevel);  // No need to specify http:// or https://

  modem.postData("GET", URL);
}