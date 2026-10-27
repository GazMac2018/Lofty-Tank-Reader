/*    Tank Level Detector - Receiver Unit (Master Unit) - Version 2

      G. McAlister

      Project started 18 October 2023

      **************************************************************************************

      Modification : 11 September 2024

      This modification has the following proceedure..

      Version 2

      1.  The Master Unit does NOT request data..  The former program, requested data every
          3 seconds and would create a lot of LoRa data packet traffic.

      2.  The Master Unit now sits quietly waiting for Data Packets and checks the data.  If
          the data falls within the required parameters then all is good..

      3.  For the Header Unit - the Sender Unit is informed that the Data needs to be Repeated
          or is sent to Sleep.

      4.  For the Unit that are on mains power -  These Units will only send Data if the level
          changes by more than 100mm (or a prescibed amount).

      5,  It may be needed for the Master Unit to Ping the Sender Units every hour or so just
          to check that they are alive..

      6.  Touch Screen has been implemented - Touch centre of screen to toggle screen ON or OFF (or Very Dim)

      ****************************************************************************************

      Board Management : >>  Tried ESP32S3 Dev Module - complied but haven't tried it on the Master Unit..  
      (Default was LOLIN D32)
      Partition Scheme : Minimal Spiffs (Large Apps with OTA)

      Using ::::
      HC SR04 - Ultrasonic Distance Sensor
      LoRa 915Mhz Transceiver

      This Script is the Receiver Unit (Master Unit)

      and so

      Using LoRa to Receive data from any number of Send Units

      Receive Units script: MasterRecUnit-WaterLevel-LoRa-Web-V2.ino

      The Receiver Unit has a LoRa id FF hex byte

      The TFT Screen on the Receive Unit (Master Unit) is used to show
      all Tank Data

      The Send Units MUST be Named the following (Case Sensitive)
      1.  Header   (LoRa id AA hex byte)
      2.  DBL Main (LoRa id BB hex byte)
      3.  uDeck    (LoRa id CC hex byte)

      Pins Used for TFT Screen
                      GPIO  TFT     ESP32 (Location)
      #define TFT_MISO 19    9  -->  12
      #define TFT_MOSI 23    6  -->  18
      #define TFT_SCLK 18    7  -->  11
      #define TFT_CS   15    3  -->   4        Chip select control pin
      #define TFT_DC    2    5  -->   5        Data Command control pin
      #define TFT_RST   4    4  -->   7        Reset pin (could connect to RST pin)

      //  Fad the LED Screen
      ScreenLED = 22;       // LED connected to digital pin 22  (17 on Weather Clock Project

      //  For Touch Prog
      #define TOUCH_CS 5

      const int LoRaCSpin = 17;      // LoRa radio chip select
      const int resetPin = 16;       // LoRa radio reset
      const int irqPin = 14;         // change for your board; must be a hardware interrupt pin
*/

#include <TFT_eSPI.h>  // TFT screen-specific library
#include <SPI.h>
#include <WiFi.h>
#include <time.h>

#include <LoRa.h>

#include "soc/soc.h"           //  only used to avoid BrownOut issue and used with..
#include "soc/rtc_cntl_reg.h"  //  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them


#include <WebServer.h>  //https://github.com/bbx10/WebServer_tng  //  Web Server
WebServer server(80);   //  Web Server

#include <Preferences.h>

#define RW_MODE false  //  Preferences variables
#define RO_MODE true

Preferences preferences;

//  EEPROM Variables using the ESP32 #include <Preferences.h>
int TrueData = 99;

//  For Touch Prog
#include <XPT2046_Touchscreen.h>

//  For Touch Prog
#define TOUCH_CS 5  //  was CS_PIN

XPT2046_Touchscreen ts(TOUCH_CS);

//  Touch variables
int PX;
int MaxPX;
int MinPX = 10000;
int PY;
int MaxPY;
int MinPY = 10000;
boolean ScreenON = true;
boolean TouchDone = true;

//  Oppo Phone
//const char* ssid     = "GazOppoA91";
//const char* password = "twinhammer";
//String WebTitle = "BALVENIE WATER TANK";
//boolean TouchEnabled = true;  //  enable Touch for Screen dimming..

//  Home LAN
const char* ssid = "Telstra9E2B8C";
const char* password = "GazGibson2017";
String WebTitle = "BALVENIE WATER TANK";
boolean TouchEnabled = true;  //  enable Touch for Screen dimming..

//  Lofty
//const char* ssid     = "NetComm 9621";
//const char* password = "Devezefexa";
//String WebTitle = "LOFTY WATER INC";
//boolean TouchEnabled = false;  //  enable Touch for Screen dimming..

// Set the IP address, gateway, and subnet mask
//IPAddress ip(10.0.0.229); // Replace with your desired static IP address
//IPAddress gateway(10.0.0.138); // Replace with your gateway address
//IPAddress subnet(255, 255, 255, 0); // Replace with your subnet mask

long WebRSSI;

TFT_eSPI tft = TFT_eSPI();            // Declare object "tft"
TFT_eSprite spr = TFT_eSprite(&tft);  // Declare Sprite object "spr" with pointer to "tft" object

//  Fad the LED Screen
int ScreenLED = 22;  // LED connected to digital pin 22

//  Special Boolean Variable for SerialPrint
boolean Serial_ON_Header = true;  //  False will not display any SerialPrint Data
boolean Serial_ON_HeaderRec = true;
boolean Serial_ON = true;

boolean ReqData = true;  //  This happens at Boot Time and Every 20 minutes excluding Header Tank.
unsigned long ReqDataDelay = millis();
int StartReqData = 0;  //  changes to 2000 for 2nd and remaining Units
int UnitCnt = 1;       //  Tank Cnt at Boot Time and Every 20 minutes.

// Width and height of each Tank Area Sprite
#define SprWIDTH 105
#define SprHEIGHT 120

// Set delay after plotting the sprite
#define SprDELAY 20

// Create the global variable
//String http;
int FontType;

int status = WL_IDLE_STATUS;  //status of wifi
boolean WiFiOK = false;

//  LoRa varibles   ######################################################

const int LoRaCSpin = 17;  // LoRa radio chip select
const int resetPin = 16;   // LoRa radio reset
const int irqPin = 14;     // change for your board; must be a hardware interrupt pin

String TheMessage;  // SendMessageNow message

unsigned long lastSendTime = millis();  // last send time
String RecMessage;

byte localAddress = 0xFF;  // Writes address of this device
byte destination;          // Writes destination to send to Selected Tank in LOOP
// 0xAA or 0xBB or 0xCC

byte recipient;  // Reads recipient address <> 0xFF
byte sender;     // Reads sender address < 0xAA or 0xBB or 0xCC

byte incomingTankName;  // incoming msg ID
byte incomingLength;    // incoming msg length sent by Sensor or Master

unsigned long LastDelay = millis();
unsigned long RequestDelay = 1000;  // initial 1 sec then 10 sec

unsigned long WebDelay = 10000;  //  10 sec
unsigned long LastWebDelay = millis();

unsigned long WiFiCheckDelay = millis();
unsigned long WiFiCheckTime = 300000;  //  5 minutes == 5 * 60 * 1000 ==  300000 msec

int WiFiCnt = 0;

boolean PrntData = false;

String LoRaRSSI;
String PrintOut;
int BkgCol;
int TankID;  //  number not letter..

//  LoRa varibles   ######################################################

int MaxNoUnit = 1;  //  Set for 1 Tank - Remove or Modify for more tanks

String TankNameLoc[5] = { "MT", "Header", "DBL Main", "uDeck", "MT" };  //  [0], ... [3]
//    some defaults that should be saved in EEPROM
float TankDia[5] = { 0, 1.8, 1.8, 1.8, 0 };  //  Defaults  -  Warning - Save to EEPROM at Line 500 (approx)
float TankHeight[5] = { 0, 2.0, 2.0, 2.0, 0 };
float HgtSensor[5] = { 0, 0.15, 0.15, 0.15, 0 };                         //  Height of Sensor to Max Height of Water
int NumbTanks[5] = { 0, 1, 1, 1, 0 };                                    //  Number Of Tanks at TankNameLoc
int SleepTime[5] = { 0, 20, 20, 20, 0 };                                 // default sleeptime set at 20 mins
String SleepMode[5] = { "BootUp", "Sleep", "Awake", "Awake", "Awake" };  //  Awake or Sleep

boolean SetHeaderSleep = false;
boolean LstSetHeaderSleep = false;
//boolean Mode_1_Done = false;
String SndSleepMode[5] = { "BootUp", "Sleep", "Sleep", "Sleep", "Sleep" };  //  Awake or Sleep
String LstSleepMode[5] = { "BootUp", "Sleep", "Sleep", "Sleep", "Sleep" };
//boolean CalcSleepDone = false;
String TimeStamp[5] = { "", "", "", "", "" };

unsigned long HeaderAwakeTime = millis();  //  Start counting in case Awake Mode sent and forgotten about 15 mins max..

//  If any of the above change then put in EEPROM
float LstTankDia[5] = { 0, 2.5, 2.5, 1.8, 0 };
float LstTankHeight[5] = { 0, 2, 2.4, 1.8, 0 };
int PercentVol[5] = { 0, 0, 0, 0, 0 };
int LstNumbTanks[5] = { 0, 1, 1, 1, 0 };  //  Number Of Tanks at TankNameLoc
//long LstSleepTime[5] = { 0, 60, 60, 60, 0 };  // default sleeptime set at 10 mins

int CurrentVol[5];
int TankSize[5];
String RSSILoRa[5] = { "down", "down", "down", "down", "down" };  //  or "Sleep"
String Dist[5];
String Temp[5];
String PowerVolts[5];
String BatteryVolts[5];
String SolarVolts[5];

float HeightPercent[5];
float CalcHt[5] = { 0, 150, 150, 150, 0 };
String DataSent[5] = { "NA", "NA", "NA", "NA", "NA" };  //  Will be set to Sent or Decd

String RecTemp;

String PowVolts;
String BatVolts;
String SolVolts;

//  Maybe use a structure type instead of an array
//  Array for History - First calc for 8 days at 24 * 3 (every 20 minutes) = 576
//  Special NOTE:  After a Board Update initial 720 (10 Days) fails compile and
//  some array variables beyond 600 were corrupted - 72 readings in 1 day
String HistoryData[505][8];  // [505][8] max 72 readings per day (7) = 504 plus [0] for heading..
int MaxMem = 505;
int HistoryCnt = 1;  //  cycles 0 > 576 then starts again
String HistoryOutput = "Screen";

String BatWarning = "";

String RecTimeSleeping;
String RecWake;
// float NxtAwakeTime = 60;
String NxtAwakeTimeAmPm[5] = { "NA", "NA", "NA", "NA", "NA" };

int TankLocCnt = 1;  //  Number of Tank Locations
int i;

float Pi = 3.14159;

boolean NewData = false;
int BMx;
int BMy;
float FullTank;
int WaterLevelPerct;

//  Singal Indicators  --  Not used in this Upgrade
//int SignalX;
//int SignalY;
//int TFT_LED_Dia = 8;

String RecTankNameLoc;
float LstRecWaterLevel;
String RecWLdist;     //  Recieved Water Level from Sensor
float RecWaterLevel;  //  RecWLdist - 220mm
int RecWaterLevelPerct;
int Wx;
int Wy;
int Wh;
int WxL;
int SprTopY = 30;
int SprBotY = SprTopY + 90;

long DummyDataWait = millis();
boolean GetDumData = true;

// setting PWM properties
const int PWMFreq = 5000;  //  5 KHz
const int PWMResolution = 10;
int MaxBright = 750;   //  was 750;  // (int)(pow(2, PWMResolution) - 1);  //  1023
int Brightness = 300;  //  Max 1023
int tft_StrLine;
int MinBright = 10;
boolean MinBrightSw = true;

// defines pins numbers for Distance Sensor
//const int trigPin = 25;   // was 5 on Arduino UNO
//const int echoPin = 26;  // was 6

int LnSpc = 25;
int NxtLneY = 0;
int ColDisp = 0;
int TankNumbSpc;

//   Time Clock Variables  *****************************************************
String TimeDate;
const char* ntpServer = "pool.ntp.org";  //  Internet Time website
const long gmtOffset_sec = 36000;        //  +10 * 60 * 60  ==  10 hours before GMT
const int daylightOffset_sec = 0;        //  0 == NO,  3600 == YES
int Hour;
int Minute;
double Second;
int Year;
int Month;
int TwHour;
int TwMin;
int TwSec;
int DispHour;
String StrDispHour;  // For displying 12 instead of 0
String StrMin;
String StrSec;
String StrYear;
String StrMonth;
String StrDay;
String AmPm;

String LongTime;
String ShortTime;
String DateCalc;
int ScreenColour;
int LastHour = 0;
int LastMin = 0;
int LastSec = 0;
//int LastDay;

//int NextDay;
int wDay;  //  day of week [0,6] (Sunday = 0)
int nDay;  //  Day Date ==  1 to 28, 30 or 31
String CurrentDate;
int LstDay;  //  for Time update at Midnight

//int DayTxtY = 10;

boolean AfterMidnight = true;  //  should be false but want to test
String MidNightTime = "None Yet";
String AfterMidNightTime = "Not Yet";  // these 2 time should be very close. Displayed on MainMenu.

int TimeSync = false;

//  Added to convert timeinfo.tm_wday to String == Day of the Week
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//char monthsOfYear[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
//int MonthDays[12] =          {31,   28,    31,     30,   31,    30,    31,    31,    30,    31,    30,    31};

unsigned long FirstMillis = millis();
int DoShowTime = 0;
boolean ReTryTime = false;
unsigned long ReTryMillis = millis();
boolean GetTimeSilent = false;


//  Setup Graphics (Converted  at: https://www.mischianti.org/images-to-byte-array-online-converter-cpp-arduino/
extern uint8_t LoftyTank[];  //  Graphics displayed when connecting to the Internet
extern uint8_t LoftyWaterLevel[];
extern uint8_t LoftyTankTitle[];
extern uint8_t WiFiIcon[];
extern uint8_t NoWiFiIcon[];

extern uint8_t BalvenieTankTitle[];

#define __FILENAME__ (strrchr("/"__FILE__, '/') + 1)

void setup()  //  ###################################################################
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // prevent brownouts by silencing them

  Serial.begin(115200);
  delay(500);

  Serial.flush();
  Serial.println();
  Serial.println("*********************************");
  Serial.println("Starting Setup for");
  Serial.println(__FILENAME__);  //  Printsout /Dir/Filename
  Serial.println("Version v-2");
  Serial.println("by G.McAlister - 14 March 2024");
  Serial.println("**********************************");

  delay(1000);

  //  For Touch Prog
  pinMode(TFT_CS, OUTPUT);    //  Pin 15
  pinMode(TOUCH_CS, OUTPUT);  //   Pin 5
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  HistoryData[0][0] = "  Date   ";
  HistoryData[0][1] = "  Time  ";
  HistoryData[0][2] = " Vol ";
  HistoryData[0][3] = " Dist";
  HistoryData[0][4] = " Pow  ";
  HistoryData[0][5] = "Bat  ";
  HistoryData[0][6] = "Sol  ";
  HistoryData[0][7] = "Temp";

  if (MaxNoUnit == 1)
    {
      ReqData = false;  //  No Need to Request Data from any Tank Locations
    }


  if (Serial_ON)
    {
      Serial.println();
      Serial.println("*** WARNING ***  Serial_ON = true - All Units Serial.print WILL be Shown");
    }
  else
    {
      Serial.println("Serial_ON = false - All Units Serial.print will NOT be Shown");
    }

  if (Serial_ON_Header)
    {
      Serial.println();
      Serial.println("*** WARNING ***  Serial_ON_Header = true - Header Only Serial.print WILL be Shown");
    }
  else
    {
      Serial.println("Serial_ON_Header = false - Header Only Serial.print will NOT be Shown");
    }

  Serial.println();

  // Create a sprite of defined size
  spr.createSprite(SprWIDTH, SprHEIGHT);

  delay(100);

  SwitchOffLoRa();

  //
  ledcAttach(ScreenLED, PWMFreq, PWMResolution);
  ledcWrite(ScreenLED, Brightness);  //  Turn on LED to Inital Brightness

  digitalWrite(TFT_CS, HIGH);  //  Not sure why this is here??

  tft.init();
  delay(100);

  tft.setRotation(1);
  tft.invertDisplay(1);  // Red Tab == 1  Green Tab == 0

  //  For Touch Prog
  //ts.begin();

  if (!ts.begin())
    {
      Serial.println("Couldn't start touchscreen controller");
      //while (1);
    }

  //  Set up Background colour for TFT
  byte R = 0, G = 0, B = 4;  //  2 = Roolly Dark Blue :)
  long RGB = ((long)R << 16L) | ((long)G << 8L) | (long)B;

  //  Print RGB colour in HEX

  if (Serial_ON)
    {
      Serial.println("Touchscreen started");
      Serial.print("Background colour == ");
      Serial.println(RGB, HEX);
    }

  BkgCol = RGB;  //  TFT_BLACK
  tft.fillScreen(BkgCol);

  tft.setTextSize(1);
  FontType = 2;
  tft.setTextFont(FontType);    //  Available Fonts
  tft.setTextColor(TFT_WHITE);  //  No Background Colour means Transparent Background with Font

  spr.setTextColor(TFT_WHITE);

  tft_StrLine = 10;
  tft.drawCentreString("Master Unit V2", 160, tft_StrLine, 4);

  SwitchOnLoRa();

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(LoRaCSpin, resetPin, irqPin);  // set CS, reset, IRQ pin
  LoRa.setTxPower(17);                        //  default == 17  max 20
  LoRa.setSpreadingFactor(7);                 // was 12 defaults to 7 - values range 6 and 12

  if (!LoRa.begin(915E6))
    {  // initialize ratio at 915 MHz
      Serial.println("LoRa init failed. Check your connections.");

      SwitchOffLoRa();  //  To Write to TFT Screen

      tft_StrLine = tft_StrLine + 35;
      tft.drawCentreString("LoRa failed.", 160, 90, 4);

      tft_StrLine = tft_StrLine + 35;
      tft.drawCentreString("Check connections", 160, 120, 4);

      while (true)  // Indefinately LOOP - stops the program from advancing
        {
          //  Does noting but LOOP<>LOOP
        }
    }
  else
    {
      Serial.println("LoRa init succeeded.");

      SwitchOffLoRa();  //  To Write to TFT Screen

      tft_StrLine = tft_StrLine + 35;
      tft.drawCentreString("LoRa is Working.", 160, tft_StrLine, 4);

      delay(500);

      if (Serial_ON)
        {
          tft_StrLine = tft_StrLine + 35;
          tft.drawCentreString("Serial Print == true", 160, tft_StrLine, 4);
          delay(500);
        }
    }

  // Create the objects for server and client
  connectToWifi();  //  Web Server

  GetWWWTime();

  BeginServer();  //  Web Server

  //  WARNING  WARNING  WARNING  WARNING  WARNING  WARNING  WARNING  WARNING  WARNING
  //  For NEW ESP32 - Uncomment To save new Default Data
  //  SaveTankData();

  GetTankData();  //  using the ESP32 #include <Preferences.h>
}

void connectToWifi()  //  ****************************   Web Server
{
  //  WiFi.reconnect()   To reconnect to Wi-Fi after a connection is lost - if needed  Try ping 10.0.0.1
  //  if (WiFi.status() != WL_CONNECTED)  This can check if connected to WiFi
  //  WiFi.disconnect();

  WiFi.mode(WIFI_STA);

  delay(2000);

  WiFi.begin(ssid, password);  //  If use Char*

  String PrintOut = "WiFi>" + String(ssid);

  tft_StrLine = tft_StrLine + 50;
  tft.drawCentreString(PrintOut, 160, tft_StrLine, 4);

  Serial.println(PrintOut);

  PrintOut = ".";
  tft_StrLine = tft_StrLine + 30;

  WiFiOK = false;

  WiFiCnt = 0;

  while (WiFi.status() != WL_CONNECTED && WiFiCnt < 20)
    {
      WiFiCnt++;

      delay(500);

      tft.drawCentreString(PrintOut, 160, tft_StrLine, 4);

      Serial.print(".");
      PrintOut = PrintOut + ".";
    }
  Serial.println();

  if (WiFiCnt < 11)
    {
      WiFiOK = true;

      String WiFiName = WiFi.localIP().toString().c_str();  //.c_str();

      tft_StrLine = tft_StrLine + 30;
      tft.drawCentreString(WiFiName, 160, tft_StrLine, 4);

      WebRSSI = WiFi.RSSI();

      String WiFiRSSI = "WiFi.RSSI : " + String(WebRSSI);

      tft_StrLine = tft_StrLine + 30;
      tft.drawCentreString(WiFiRSSI, 160, tft_StrLine, 4);

      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      delay(5000);  //  Need this delay time for user to read IP address
    }
  else
    {
      WiFiOK = false;

      if (Serial_ON_Header)
        {
          Serial.println("No WiFi - Check Later");
        }
      String PrintOut = "No WiFi - Check Later";

      tft_StrLine = tft_StrLine + 50;
      tft.drawCentreString(PrintOut, 160, tft_StrLine, 4);

      delay(5000);
    }

  tft.fillScreen(BkgCol);
}

void GetTankData()  //  using the ESP32 #include <Preferences.h>
{
  //Serial.println("void GetTankData()");

  preferences.begin("TankData", RO_MODE);
  delay(10);

  TrueData = preferences.getUInt("TrueData", 0);  // preference.getUInt("Var", default value)

  if (TrueData == 99)
    {
      TankHeight[1] = preferences.getFloat("TankHgt1", 0);
      LstTankHeight[1] = TankHeight[1];

      TankDia[1] = preferences.getFloat("TankDia1", 0);
      LstTankDia[1] = TankDia[1];

      NumbTanks[1] = preferences.getFloat("NumbTanks1", 0);
      LstNumbTanks[1] = NumbTanks[1];

      TankHeight[2] = preferences.getFloat("TankHgt2", 0);
      LstTankHeight[2] = TankHeight[2];

      TankDia[2] = preferences.getFloat("TankDia2", 0);
      LstTankDia[2] = TankDia[2];

      NumbTanks[2] = preferences.getFloat("NumbTanks2", 0);
      LstNumbTanks[2] = NumbTanks[2];

      TankHeight[3] = preferences.getFloat("TankHgt3", 0);
      LstTankHeight[3] = TankHeight[3];

      TankDia[3] = preferences.getFloat("TankDia3", 0);
      LstTankDia[3] = TankDia[3];

      NumbTanks[3] = preferences.getFloat("NumbTanks3", 0);
      LstNumbTanks[3] = NumbTanks[3];
    }
  preferences.end();

  if (TankHeight[1] < 1)
    {
      for (int i = 0; i < 5; i++)
        {
          TankDia[i] = 1.8;
          TankHeight[i] = 1.8;
          NumbTanks[i] = 1;  //  Number Of Tanks at TankNameLoc
        }
    }

  if (Serial_ON)
    {
      Serial.println("Getting Data from EEPROM");

      Serial.print("TrueData = ");
      Serial.println(TrueData);

      Serial.print("TankDia1 = ");
      Serial.println(TankDia[1]);

      Serial.print("TankHgt1 = ");
      Serial.println(TankHeight[1]);

      Serial.print("NumbTanks1 = ");
      Serial.println(NumbTanks[1]);

      Serial.print("TankDia2 = ");
      Serial.println(TankDia[2]);

      Serial.print("TankHgt2 = ");
      Serial.println(TankHeight[2]);

      Serial.print("NumbTanks2 = ");
      Serial.println(NumbTanks[2]);

      Serial.print("TankDia3 = ");
      Serial.println(TankDia[3]);

      Serial.print("TankHgt3 = ");
      Serial.println(TankHeight[3]);

      Serial.print("NumbTanks3 = ");
      Serial.println(NumbTanks[3]);
    }
}

void SaveTankData()  //  using the ESP32 #include <Preferences.h>
{
  if (Serial_ON)
    {
      Serial.println("***************Saving EEPROM*******************");

      Serial.print("TrueData = ");
      Serial.println(TrueData);

      Serial.print("TankDia1 = ");
      Serial.println(TankDia[1]);

      Serial.print("TankHgt1 = ");
      Serial.println(TankHeight[1]);

      Serial.print("NumbTanks1 = ");
      Serial.println(NumbTanks[1]);

      Serial.print("TankDia2 = ");
      Serial.println(TankDia[2]);

      Serial.print("TankHgt2 = ");
      Serial.println(TankHeight[2]);

      Serial.print("NumbTanks2 = ");
      Serial.println(NumbTanks[2]);

      Serial.print("TankDia3 = ");
      Serial.println(TankDia[3]);

      Serial.print("TankHgt3 = ");
      Serial.println(TankHeight[3]);

      Serial.print("NumbTanks3 = ");
      Serial.println(NumbTanks[3]);

      Serial.println("2 Sec Delay");

      delay(2000);
    }
  //  Save All EEPROM

  preferences.begin("TankData", RW_MODE);

  preferences.putUInt("TrueData", 99);

  preferences.putFloat("TankHgt1", TankHeight[1]);
  preferences.putFloat("TankDia1", TankDia[1]);
  preferences.putFloat("NumbTanks1", NumbTanks[1]);

  preferences.putFloat("TankHgt2", TankHeight[2]);
  preferences.putFloat("TankDia2", TankDia[2]);
  preferences.putFloat("NumbTanks2", NumbTanks[2]);

  preferences.putFloat("TankHgt3", TankHeight[3]);
  preferences.putFloat("TankDia3", TankDia[3]);
  preferences.putFloat("NumbTanks3", NumbTanks[3]);

  // Close the Preferences
  preferences.end();

  delay(1000);
}

void GetWWWTime()  //  *****************************************************
{
  //Serial.println("void GetWWWTime()");

  ReTryTime = false;

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  //  This one line get GMT time.. :)

  struct tm timeinfo;

  int FailCnt = 0;

  while (!getLocalTime(&timeinfo))  //  Stay in this Loop until Time Server located
    {
      FailCnt++;
      if (Serial_ON_Header)
        {
          Serial.print("Failed to obtain time :");
          Serial.println(FailCnt);
        }

      SwitchOffLoRa();

      if (!GetTimeSilent)
        {

          tft.fillScreen(TFT_RED);
          tft.setTextColor(TFT_WHITE, TFT_RED);
          tft.setTextSize(2);
          tft.drawCentreString("Failed to obtain time", 160, 70, 1);
          tft.drawCentreString("No Internet??", 160, 100, 1);
          tft.drawNumber(FailCnt, 150, 140, 1);

          delay(2000);
        }

      SwitchOnLoRa();

      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  //  This one line get GMT time.. :)
      tm timeinfo;

      //  ReTryTime is not used in this Project
      if (FailCnt > 2)
        {
          ReTryTime = true;
          ReTryMillis = millis();

          if (Hour == 0)
            {
              Hour = 9;
              Minute = 0;
            }

          break;
        }
    }


  //  ReTryTime is not used in this Project - leave as is in case for later
  if (!ReTryTime)  //  If break from Loop above - don't corrupt Time variables.
    {

      Year = timeinfo.tm_year + 1900;  //  Add 1900
      //Serial.println(Year);

      Month = timeinfo.tm_mon + 1;  //  Where Jan = timeinfo.tm_mon + 1 for RTC
      //Serial.println(Month);

      nDay = timeinfo.tm_mday;  //  Day Date Number 1 -> 28, 30 or 31
      //Serial.print("First time get nDay == ");
      //Serial.println(nDay);

      CurrentDate = String(nDay) + "/" + String(Month) + "/" + String(Year);

      wDay = timeinfo.tm_wday;  //  day of week [0,6] (Sunday = 0)

      LstDay = nDay;  //  used for Time update at Midnight.

      Hour = timeinfo.tm_hour;

      Minute = timeinfo.tm_min;

      //Serial.println(Minute);

      Second = timeinfo.tm_sec;

      // if(Second + 1 < 60)
      //  {
      //    Second = Second + 1;
      //  }

      //Serial.println(Second);
    }

  SwitchOffLoRa();

  tft.fillScreen(BkgCol);
  tft.setTextSize(1);
  FontType = 2;
  tft.setTextFont(FontType);    //  Available Fonts
  tft.setTextColor(TFT_WHITE);  //  No Background Colour means Transparent Background with Font

  SwitchOnLoRa();

  //  Write Title and Draw Lines
  DrawLinesLandscape();

  //  GAZ TIME CHECK - Start Time anytime HERE
  /*  if(true)  //  **************Testing***************
      {
       Hour = 8;     //  ***************Testing******************
       Minute = 54;  //  *************Testing******************
      // DoOnce = false;

       Serial.print("Reset Time to before ");
       Serial.print(Hour + 1);
       Serial.println(" <<<<<<<<<<<<<<<");
       Serial.println();
       tft.setTextSize(1);
      }*/


  if (Serial_ON)
    {
      //  To Set Time in RTC from Internet Time - Put updated Time into RTC
      //  rtc.adjust(DateTime(Year, Month, nDay, Hour, Minute, Second));
      Serial.println("******************************************************");
      Serial.print("NTP Day & Time : ");
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      Serial.println("******************************************************");
    }

  TimeSync = true;  //  This should only be here

  FirstMillis = millis();  //  Using this method eliminates possible millis() overflow after 49+ days

  /*  //  This method will be affected by millis() overflow at 49+ days
    CountSec = 1;        //  need to reset at MidNight
    targetTime = FirstMillis + CountSec * 1000;  //  this ensure that the time is locked to internal ESP32 clock
  */


  //  Found this -->  to put Above Formatted &timeinfo Output into a single string
  /*
    char timeStringBuff[50]; //50 chars should be enough
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    //print like "const char*"
    Serial.println(timeStringBuff);

    //Optional: Construct String object
    String asString(timeStringBuff);*/
}

void TouchPxYx()  //  ********************************************************
{
  if (ts.touched() && !TouchDone)
    {

      //    Serial.println("void TouchPxYx()");

      //    Serial.print("ts.touched() == ");
      //    Serial.println(ts.touched());
      //    Serial.print("TouchDone == ");
      //    Serial.println(TouchDone);

      TS_Point p = ts.getPoint();

      //Serial.print("Pressure = ");
      //Serial.print(p.z);
      // Serial.print(", x = ");
      //Serial.print(p.x);

      PX = p.x;

      //Serial.print(", y = ");
      //Serial.print(p.y);

      PY = p.y;

      //delay(250);

      //Serial.println();

      //  MaxPX MaxPY etc below ONLY used to determine the limits of the Touch Screen
      //  if( MaxPX < PX) MaxPX = PX;  //  3931
      //    if( MaxPY < PY) MaxPY = PY;  //  3783
      //    if( MinPX > PX) MinPX = PX;  //  245
      //    if( MinPY > PY) MinPY = PY;  //  285

      //     Serial.print("MaxPX = ");
      //     Serial.print(MaxPX);
      //     Serial.print("  MaxPY = ");
      //     Serial.print(MaxPY);
      //     Serial.print("  MinPX = ");
      //     Serial.print(MinPX);
      //     Serial.print("  MinPY = ");
      //     Serial.println(MinPY);

      PX = map(PX, 245, 3931, 0, 350);
      PY = map(PY, 285, 3783, 0, 240);

      //      Serial.print("PX = ");
      //      Serial.print(PX);
      //      Serial.print("   PY = ");
      //      Serial.println(PY);

      TouchDone = true;  //  stop multiple taps on screen
    }
}

void TouchSpot()  //  For Touch Prog *************************************************
{

  SwitchOffLoRa();  //  To Test for Touch on TFT Screen trun OFF LoRa

  TouchDone = false;

  TouchPxYx();

  SwitchOnLoRa();

  if (TouchDone)
    {

      //    Serial.println("We have a Touch");

      //  #########################################################################

      // Turn Off Screen if Touched in middle of screen == 320 x 240
      //  define a square 60 x 60
      // so half is 160 x 120  X between 100 & 220  and  Y between 60 & 180

      if (PX > 100 && PX < 220 && PY > 60 && PY < 180)  //  Middle of Screen
        {
          if (ScreenON)
            {
              ScreenON = false;

              //        Serial.println("Turning Screen OFF");

              //  Turn Screen OFF

              ledcWrite(ScreenLED, 10);  //  Turn on LED to Low
            }
          else
            {
              ScreenON = true;

              //        Serial.println("Turning Screen ON");

              //  Turn Screen ON

              ledcWrite(ScreenLED, Brightness);  //  Turn on LED to Normal Brightness
            }
        }

      SwitchOffLoRa();  //  To Test for Touch on TFT Screen

      while (ts.touched())  //  Loops here while Screen Touched..  Stops Screen blinking
        {
          delay(10);
        }

      SwitchOnLoRa();
    }
}

void ShowTime()  //  *************************************************
{

  boolean ChangeDisplay = false;
  boolean SecChanged = false;
  int Interval = 0;

  //if(targetTime < millis())  //  This method will be affected by millis() overflow
  //        3020           999  (First DoShowTime == 0)
  if ((millis() - FirstMillis) > DoShowTime)  //  Using this method, eliminates possible millis() overflow after 49+ days
    {
      DoShowTime = 972;

      Interval = millis() - FirstMillis;  // 3020 - 1000 == 2020 or 2.020 seconds or 00000202 - x9999999 == 203

      FirstMillis = millis();  //  using 999 because getting to this line takes about 1 ms..  apparently?

      //Serial.print("FirstMillis == ");
      //Serial.println(FirstMillis);

      //  Rather than adding 1 Second >>> Add the Inverval, keeping the decimal - Second is a Double type Varible
      Second = Second + (float(Interval) / 1000);

      if (Second > 59.9)  //  60 is not greater than 60 & 60 >> 0
        {                 // Check for roll-over
          // Second = 0;                   // Reset seconds to zero

          //  Rather than Resetting Second to 0 >> Substract 60 to keep decimal seconds
          Second = Second - 60;

          Minute++;           //   mm++;   // Advance minute
          if (Minute > 59)    //  was mm
            {                 // Check for roll-over
              Minute = 0;     //  mm = 0;
              Hour++;         //  hh++;     // Advance hour
              if (Hour > 23)  //  was hh
                {             // Check for 24hr roll-over (could roll-over on 13)
                  Hour = 0;   //   hh = 0;  // 0 for 24 hour clock, set to 1 for 12 hour clock
                }
            }
        }
      //Serial.print("Second == ");
      //Serial.println(Second);

      if (LastSec != Second)  //  using this to stop huge coloured screen error
        {
          LastSec = Second;
          SecChanged = true;
        }

      if (LastMin != Minute)  //  should be using this
        {
          LastMin = Minute;
          ChangeDisplay = true;

          CalcSleep();  //  Recalculate Sleep Time every 1 minute

          if (Serial_ON)
            {
              Serial.println(" Line 1120  -  Calc SleepTime every 1 minute.");
            }

          //  Every 20 minutes request data from DBL_Main and uDeck Tank Units...
          if (Minute == 00 || Minute == 20 || Minute == 40 && MaxNoUnit != 1)
            {
              if (Serial_ON)
                {
                  Serial.println(" Line 1128  -  20 mins - send request for Data ");
                }

              ReqData = true;    //  Request Data to check Sensor in Alive
              StartReqData = 0;  //  Do request instantly for first time
              UnitCnt = 2;       //  Will only request Data from DBL_Main and uDeck (not Header)
            }
        }
    }

  if (SecChanged)  //  The TTF Screen for Time is only updated every second
    {
      //  Check for MidNight to sync Internet Time
      if (Hour == 0 && !TimeSync)  // at MidNight update Time with ntpServer
        {
          MidNightUpdate();
          // if (BigDayDisplay) DoBigDay = true;
        }

      if (Hour != 0)
        {
          TimeSync = false;  //  set to false for next MidNight
        }

      AmPm = "am";
      TwHour = Hour;  //  24 hour time
      TwMin = Minute;
      TwSec = Second;
      DispHour = TwHour;

      if (TwHour == 0)
        DispHour = 12;

      if (DispHour > 12)
        {
          DispHour = DispHour - 12;
        }

      if (TwHour > 11)
        {
          AmPm = "pm";
        }

      StrDispHour = String(DispHour);
      if (StrDispHour.length() < 2)
        {
          StrDispHour = StrDispHour;
        }

      StrMin = String(Minute);
      if (StrMin.length() < 2)
        {
          StrMin = "0" + StrMin;
        }

      StrSec = String(int(Second));
      if (StrSec.length() < 2)
        {
          StrSec = "0" + StrSec;
        }

      LongTime = StrDispHour + ":" + StrMin + ":" + StrSec + AmPm;

      ShortTime = StrDispHour + ":" + StrMin + AmPm;

      if (AfterMidnight)
        {
          //Serial.print("Time After Midnight Update == ");
          //Serial.println(LongTime);
          AfterMidNightTime = LongTime;
          AfterMidnight = false;
        }

      if (ChangeDisplay)  //  The Time display is only updated every Minute
        {

          SwitchOffLoRa();  //  To Write to TFT Screen

          //            x   y  xw  yh
          tft.fillRect(260, 0, 60, 25, BkgCol);

          tft.setTextSize(1);
          FontType = 2;
          //tft.setTextFont(FontType);  //  Available Fonts
          tft.setTextColor(TFT_WHITE);  //  No Background Colour means Transparent Background with Font

          //  String, X, Y, FontType

          tft.drawCentreString(ShortTime, 290, 5, FontType);

          // tft.print(ShortTime);

          SwitchOnLoRa();
        }
    }
}

void MidNightUpdate()  //  ********************************************
{
  //Serial.println("void MidNightUpdate()");

  TimeSync = true;

  //Serial.print("Time before Update - ");
  //Serial.println(LongTime);  //  LongTime == __0:00:00am   __9:04:56pm
  MidNightTime = LongTime;

  AfterMidnight = true;

  //CheckInternet();

  GetWWWTime();

  DrawLinesLandscape();
}

void BeginServer()  //  ****************************   Web Server
{                   // Should be NO Spaces between "/"  and check below..
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void handleRoot()  //  ****************************   Web Server
{
  //PrntData = false;  //  change this to  true  to show Data History

  if (!PrntData)
    {
      server.send(200, "text/html", getPage());
    }
  else
    {
      server.send(200, "text/html", dataPage());
    }
}

//  mymap is not used here..  Refer SlaveSenderUnit-WaterLevel-LoRa.ino
/* float mymap(float x, float in_min, float in_max, float out_min, float out_max)
  {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }*/

void SprHeaderLandS()  //  **********************************************************************
{
  // Width and height of each Tank Area Sprite
  //#define SprWIDTH = 105;
  //#define SprHEIGHT 96

  if (Serial_ON_Header)
    {
      Serial.println("Drawing Sprite Header Tank");
    }

  spr.createSprite(SprWIDTH, SprHEIGHT);

  // Fill the whole sprite with BkgCol (Sprite is in memory so not visible yet)
  spr.fillSprite(BkgCol);

  spr.drawCentreString(RecTankNameLoc, 55, 0, FontType);  //  Put Text in Centre of Sprite

  //  Draw Tank Graphics under Tank Name x == 25 & y == 49 - 25 == 24
  BMx = 20;
  BMy = 24;
  spr.drawBitmap(BMx, BMy, LoftyTank, 65, 47, TFT_GREEN);  //  Draw Tank Graphics

  TimeStamp[1] = LongTime;

  Dist[1] = String(RecWaterLevel);

  //  Test to avoid Divide by Zero (0)
  if (TankHeight[1] == 0) TankHeight[1] = 0.1;

  HeightPercent[1] = (TankHeight[1] - RecWaterLevel) / TankHeight[1];

  RSSILoRa[1] = LoRaRSSI;

  //  Setup Voltage Warning
  BatWarning = "";

  if (BatVolts.toFloat() < 3.50 && BatVolts.toFloat() > 3.40)
    {
      BatWarning = " Low";
    }

  if (BatVolts.toFloat() < 3.40 && BatVolts.toFloat() > 3.30)
    {
      BatWarning = " Critical";
    }

  if (BatVolts.toFloat() < 3.30)
    {
      BatWarning = " Shut Down - 60m";
      // Change to Sleep if set to Awake
      SndSleepMode[1] = "Sleep";

      if (Serial_ON_Header)
        {
          Serial.print("BatVolts == ");
          Serial.println(BatVolts);
        }
    }

  //  WebPage Graphics Tank Percentage
  CalcHt[1] = 150 * (1 - HeightPercent[1]);
  PowerVolts[1] = PowVolts;
  BatteryVolts[1] = BatVolts;
  SolarVolts[1] = SolVolts;

  if (Serial_ON_Header || Serial_ON)
    {
      Serial.print("NumbTanks[1] > ");
      Serial.print(NumbTanks[1]);
      Serial.print("   TankHeight[1] > ");
      Serial.print(TankHeight[1]);
      Serial.print("   RecWaterLevel > ");
      Serial.print(RecWaterLevel);
      Serial.print("   HeightPercent[1]  > ");
      Serial.println(HeightPercent[1]);

      Serial.print("Power Volts[1]  > ");
      Serial.print(PowerVolts[1]);

      Serial.print("  BatteryVolts [1]  > ");
      Serial.print(BatteryVolts[1]);

      Serial.print("   SolarVolts[1]  > ");
      Serial.print(SolarVolts[1]);

      Serial.print("  RecWake Awake or Sleep >");
      Serial.print(RecWake);
      Serial.println("<");
    }

  Wx = BMx + 3;
  Wy = BMy + 44;  //  47 == height of BitMap

  Wh = 40 * HeightPercent[1];  //  40 = Height Inside Dimension of Tank
  WxL = BMx + 61;

  int WL;

  //  Draw Water in Tank - Series of Lines from Bottom of Tank to Top of Water Level

  for (WL = 0; WL < Wh - 1; WL++)
    {
      spr.drawLine(Wx, Wy - WL, WxL, Wy - WL, TFT_CYAN);
    }

  spr.drawBitmap(Wx, Wy - WL, LoftyWaterLevel, 62, 2, TFT_CYAN);  //  Draw Wavey Top Water Line

  CurrentVol[1] = Pi * (TankHeight[1] * HeightPercent[1]) * sq(TankDia[1] / 2) * NumbTanks[1] * 1000;  //  Current Volume

  TankSize[1] = Pi * TankHeight[1] * sq(TankDia[1] / 2) * NumbTanks[1] * 1000;

  PercentVol[1] = int(float(CurrentVol[1]) / float(TankSize[1]) * 100);  //  Calc Current Vol in Percentage

  if (CurrentVol[1] < 0) CurrentVol[1] = 0;

  PrintOut = String(CurrentVol[1]) + " Lts";
  spr.drawCentreString(PrintOut, BMx + 32, 76, FontType);

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);             // Gives time to Update the Sprite - I think..  20ms
  spr.pushSprite(0, SprTopY);  //  Tank Sprite

  //  ***************************
  spr.fillSprite(BkgCol);  //  Create New/Next Text Sprite

  int x = 54;
  NxtLneY = 5;

  PrintOut = "RSSI: " + LoRaRSSI;  // default Printout
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;

  PrintOut = "Dist: " + Dist[1];
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  PrintOut = "Time: " + ShortTime;
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  Temp[1] = RecTemp;
  PrintOut = "Temp: " + Temp[1];
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  BatteryVolts[1] = BatVolts;
  PrintOut = "Battery: " + BatteryVolts[1] + "v";
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;

  PrintOut = "Power: " + PowerVolts[1] + "v";
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;  //  For SleepTime Countdown

  if (RecWake == "Awake" || SndSleepMode[1] == "Awake")
    {
      SleepMode[1] = "Awake";
      spr.drawCentreString("Mode - Awake", x, NxtLneY, FontType);
    }
  else if (RecWake == "Sleep" || SndSleepMode[1] == "Sleep")
    {
      SleepMode[1] = "Sleep";
      spr.drawCentreString("Mode - Sleep", x, NxtLneY, FontType);
    }
  else
    {
      PrintOut = RecWake + " - " + SleepMode[1];
      spr.drawCentreString(PrintOut, x, NxtLneY, FontType);
      //  SleepMode[1] = "Sleep";
    }

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);             // Gives time to Update the Sprite - I think..  20ms
  spr.pushSprite(0, SprBotY);  //  Tank Sprite

  //  Clear Sprite Data
  spr.createSprite(1, 1);
  spr.fillSprite(BkgCol);
  spr.pushSprite(0, 239);  //  Tank Sprite

  // delay(10);  //  may be needed

  SwitchOnLoRa();
}

void SprDBL_MainLandS()  //  **********************************************************************
{
  // Width and height of each Tank Area Sprite
  //#define SprWIDTH  105
  //#define SprHEIGHT 96

  if (Serial_ON_Header)
    {
      Serial.println("Drawing Sprite DBL_Main");
    }

  spr.createSprite(SprWIDTH, SprHEIGHT);

  // Fill the whole sprite with BkgCol (Sprite is in memory so not visible yet)
  spr.fillSprite(BkgCol);

  spr.drawCentreString(RecTankNameLoc, 55, 0, FontType);  //  Put Text in Centre of Sprite

  //  Draw Tank Graphics under Tank Name x == 25 & y == 49 - 25 == 24
  BMx = 20;
  BMy = 24;
  spr.drawBitmap(BMx, BMy, LoftyTank, 65, 47, TFT_GREEN);  //  Draw Tank Graphics

  TimeStamp[2] = LongTime;

  Dist[2] = String(RecWaterLevel);

  //  Test to avoid Divide by Zero (0)
  if (TankHeight[2] == 0) TankHeight[2] = 0.1;

  HeightPercent[2] = (TankHeight[2] - RecWaterLevel) / TankHeight[2];

  if (Serial_ON_Header || Serial_ON)
    {
      Serial.print("HeightPercent[2] > ");
      Serial.println(HeightPercent[2]);
    }

  RSSILoRa[2] = LoRaRSSI;

  //  WebPage Graphis Tank Percentage
  CalcHt[2] = 150 * (1 - HeightPercent[2]);

  if (Serial_ON)
    {
      Serial.print("NumbTanks[2] > ");
      Serial.print(NumbTanks[2]);
      Serial.print("   TankHeight[2] > ");
      Serial.print(TankHeight[2]);
      Serial.print("   RecWaterLevel > ");
      Serial.print(RecWaterLevel);
      Serial.print("   HeightPercent[2]  > ");
      Serial.print(HeightPercent[2]);

      Serial.print("  RecWake Awake or Sleep >");
      Serial.println(RecWake);
    }

  Wx = BMx + 3;
  Wy = BMy + 44;  //  47 == height of BitMap

  Wh = 40 * HeightPercent[2];  //  40 = Height Inside Dimension of Tank
  WxL = BMx + 61;

  int WL;

  //  Draw Water in Tank - Series of Lines from Bottom of Tank to Top of Water Level

  for (WL = 0; WL < Wh - 1; WL++)
    {
      spr.drawLine(Wx, Wy - WL, WxL, Wy - WL, TFT_CYAN);
    }

  spr.drawBitmap(Wx, Wy - WL, LoftyWaterLevel, 62, 2, TFT_CYAN);  //  Draw Wavey Top Water Line

  CurrentVol[2] = Pi * (TankHeight[2] * HeightPercent[2]) * sq(TankDia[2] / 2) * NumbTanks[2] * 1000;

  TankSize[2] = Pi * TankHeight[2] * sq(TankDia[2] / 2) * NumbTanks[2] * 1000;

  PercentVol[2] = int(float(CurrentVol[2]) / float(TankSize[2]) * 100);  //  Calc Current Vol in Percentage

  if (CurrentVol[2] < 0) CurrentVol[2] = 0;

  PrintOut = String(CurrentVol[2]) + " Lts";
  spr.drawCentreString(PrintOut, BMx + 32, 76, FontType);

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);               // Gives time to Update the Sprite - I think..  10ms
  spr.pushSprite(108, SprTopY);  //  Tank Sprite

  SwitchOnLoRa();

  //  ***************************
  spr.fillSprite(BkgCol);

  int x = 54;
  NxtLneY = 5;

  // PrintOut = "RSSI: " + LoRaRSSI;
  // spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;

  PrintOut = "Dist: " + Dist[2];
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  PrintOut = "Time: " + ShortTime;
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  // Temp[2] = RecTemp;
  // PrintOut = "Temp: " + Temp[2];
  // spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  // PowerVolts[2] = RecVolts;
  // PrintOut = "Volts: " + PowerVolts[2];
  // spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;  //  For SleepTime Countdown
  if (RecWake == "Awake")
    {
      SleepMode[2] = "Awake";
      spr.drawCentreString("Mode - Awake", x, NxtLneY, FontType);
    }

  if (RecWake == "Sleep")
    {
      SleepMode[2] = "Sleep";
      spr.drawCentreString("Mode - Sleep", x, NxtLneY, FontType);
    }

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);               // Gives time to Update the Sprite - I think.. 10ms
  spr.pushSprite(108, SprBotY);  //  Tank Sprite

  //  Clear Sprite Data
  spr.createSprite(10, 10);
  spr.fillSprite(BkgCol);
  spr.pushSprite(0, 239);  //  Tank Sprite

  SwitchOnLoRa();
}

void SpruDeckLandS()  //  **********************************************************************
{
  // Width and height of each Tank Area Sprite
  //#define SprWIDTH  118
  //#define SprHEIGHT 96

  if (Serial_ON_Header)
    {
      Serial.println("Drawing Sprite uDeck");
    }

  spr.createSprite(SprWIDTH, SprHEIGHT);

  // Fill the whole sprite with BkgCol (Sprite is in memory so not visible yet)
  spr.fillSprite(BkgCol);

  spr.drawCentreString(RecTankNameLoc, 55, 0, FontType);  //  Put Text in Centre of Sprite

  //  Draw Tank Graphics under Tank Name x == 25 & y == 49 - 25 == 24
  BMx = 20;
  BMy = 24;
  spr.drawBitmap(BMx, BMy, LoftyTank, 65, 47, TFT_GREEN);  //  Draw Tank Graphics

  TimeStamp[3] = LongTime;

  Dist[3] = String(RecWaterLevel);

  //  Test to avoid Divide by Zero (0)
  if (TankHeight[3] == 0) TankHeight[3] = 0.1;

  HeightPercent[3] = (TankHeight[3] - RecWaterLevel) / TankHeight[3];

  RSSILoRa[3] = LoRaRSSI;

  //  WebPage Graphis Tank Percentage  150 == Height of Tank Graphics
  CalcHt[3] = 150 * (1 - HeightPercent[3]);

  if (Serial_ON)
    {
      Serial.print("NumbTanks[3] > ");
      Serial.print(NumbTanks[3]);
      Serial.print("   TankHeight[3] > ");
      Serial.print(TankHeight[3]);
      Serial.print("   RecWaterLevel : ");
      Serial.print(RecWaterLevel);
      Serial.print("   HeightPercent[3]  > ");
      Serial.print(HeightPercent[3]);

      Serial.print("  RecWake Awake or Sleep >");
      Serial.println(RecWake);
    }

  Wx = BMx + 3;
  Wy = BMy + 44;  //  47 == height of BitMap

  Wh = 40 * HeightPercent[3];  //  40 = Height Inside Dimension of Tank
  WxL = BMx + 61;

  int WL;

  //  Draw Water in Tank - Series of Lines from Bottom of Tank to Top of Water Level

  for (WL = 0; WL < Wh - 1; WL++)
    {
      spr.drawLine(Wx, Wy - WL, WxL, Wy - WL, TFT_CYAN);
    }

  spr.drawBitmap(Wx, Wy - WL, LoftyWaterLevel, 62, 2, TFT_CYAN);  //  Draw Wavey Top Water Line

  CurrentVol[3] = Pi * (TankHeight[3] * HeightPercent[3]) * sq(TankDia[3] / 2) * NumbTanks[3] * 1000;

  TankSize[3] = Pi * TankHeight[3] * sq(TankDia[3] / 2) * NumbTanks[3] * 1000;

  PercentVol[3] = int(float(CurrentVol[3]) / float(TankSize[3]) * 100);  //  Calc Current Vol in Percentage

  if (CurrentVol[3] < 0) CurrentVol[3] = 0;

  PrintOut = String(CurrentVol[3]) + " Lts";
  spr.drawCentreString(PrintOut, BMx + 32, 76, FontType);

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);               // Gives time to Update the Sprite - I think.. 10ms
  spr.pushSprite(215, SprTopY);  //  Tank Sprite

  SwitchOnLoRa();

  //  ***************************
  spr.fillSprite(BkgCol);

  int x = 54;
  NxtLneY = 5;

  PrintOut = "RSSI: " + LoRaRSSI;
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;

  PrintOut = "Dist: " + Dist[3];
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  PrintOut = "Time: " + ShortTime;
  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  //  Temp[3] = RecTemp;
  //  PrintOut = "Temp: " + Temp[3];
  //  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;
  //  PowerVolts[3]  = PowVolts;
  //  PrintOut = "Volts: " + PowerVolts[3];
  //  spr.drawCentreString(PrintOut, x, NxtLneY, FontType);

  NxtLneY = NxtLneY + LnSpc;  //  For SleepTime Countdown
  if (RecWake == "Awake")
    {
      SleepMode[3] = "Awake";
      spr.drawCentreString("Mode - Awake", x, NxtLneY, FontType);
    }

  if (RecWake == "Sleep")
    {
      SleepMode[3] = "Sleep";
      spr.drawCentreString("Mode - Sleep", x, NxtLneY, FontType);
    }

  SwitchOffLoRa();  //  To Write to TFT Screen

  delay(SprDELAY);               // Gives time to Update the Sprite - I think..  10ms
  spr.pushSprite(215, SprBotY);  //  Tank Sprite

  //  Clear Sprite Data
  spr.createSprite(10, 10);
  spr.fillSprite(BkgCol);
  spr.pushSprite(0, 239);  //  Tank Sprite

  SwitchOnLoRa();
}

//  ****************************************************************************************
void drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color)
{
  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for (j = 0; j < h; j++)
    {
      for (i = 0; i < w; i++)
        {
          if (i & 7) byte <<= 1;
          else byte = pgm_read_byte(bitmap + j * byteWidth + i / 8);
          if (byte & 0x80) tft.drawPixel(x + i, y + j, color);
        }
    }
}

void DrawLinesLandscape()
{

  SwitchOffLoRa();  //  To Write to TFT Screen

  tft.fillScreen(BkgCol);

  if (WebTitle == "BALVENIE WATER TANK")
    {
      tft.drawBitmap(34, 1, BalvenieTankTitle, 221, 24, TFT_YELLOW);
    }
  else
    {
      tft.drawBitmap(34, 1, LoftyTankTitle, 221, 24, TFT_YELLOW);
    }

  //  Top Horiz Line
  //           x, y, x2, y2
  tft.drawLine(0, 25, 319, 25, TFT_LIGHTGREY);  //  2 lines thick
  tft.drawLine(0, 26, 319, 26, TFT_LIGHTGREY);

  // 1st Vert Line
  tft.drawLine(106, 26, 106, 239, TFT_LIGHTGREY);
  tft.drawLine(107, 26, 107, 239, TFT_LIGHTGREY);

  // 2nd Vert Line
  tft.drawLine(213, 26, 213, 239, TFT_LIGHTGREY);
  tft.drawLine(214, 26, 214, 239, TFT_LIGHTGREY);

  ShowMyWifi();

  ShowTime();

  SwitchOnLoRa();
}

void ShowMyWifi()
{
  if (WiFiOK)
    {
      SwitchOffLoRa();  //  To Write to TFT Screen

      tft.fillRect(0, 0, 24, 24, BkgCol);
      tft.drawBitmap(0, 0, WiFiIcon, 24, 24, TFT_WHITE);

      SwitchOnLoRa();
    }
  else
    {
      SwitchOffLoRa();  //  To Write to TFT Screen

      tft.fillRect(0, 0, 24, 24, BkgCol);
      tft.drawBitmap(0, 0, NoWiFiIcon, 24, 24, TFT_WHITE);

      SwitchOnLoRa();
    }
}

void SwitchOnLoRa()  //  **************************************************
{
  delay(20);

  digitalWrite(TFT_CS, HIGH);    //  deSelect TFT
  digitalWrite(TOUCH_CS, HIGH);  //  deSlect Touch
  digitalWrite(LoRaCSpin, LOW);  //  Select LoRa  <<<<<<<<<<<<<<<<<<<<<<<<<<<

  delay(20);
}

void SwitchOffLoRa()  //  **************************************************
{

  delay(20);
  digitalWrite(LoRaCSpin, HIGH);  //  deSelect LoRa  >>>>>>>>>>>>>>>>>>>>>>>>>>>>
  digitalWrite(TFT_CS, LOW);      //  Select TFT
  digitalWrite(TOUCH_CS, LOW);    //  Select Touch
  delay(20);
}

void onReceive(int packetSize)  //  *******************************************
{
  if (packetSize != 0)
    {
      // read packet header bytes:  First 2 bytes are Recipient and Sender
      recipient = LoRa.read();  // recipient address

      sender = LoRa.read();  // sender address

      if (Serial_ON)
        {
          Serial.print("   recipient =");
          Serial.print(recipient);
          Serial.print("  sender    =");
          Serial.println(sender);
        }

      // read packet header bytes: 3rd byte is length of text NOTE: Must be smaller that 256 chars..
      incomingLength = LoRa.read();  // incoming msg length sent to check Data

      //  Next collect the RSSI
      LoRaRSSI = String(LoRa.packetRssi());  //  This is not sent or received - it is part of the LoRa send/recieve routine

      //  clear Data collecting string
      RecMessage = "";

      while (LoRa.available())  //  Read all chars of recieved string
        {
          RecMessage += (char)LoRa.read();
        }

      if (recipient != 0xFF)  //  0xFF == Master
        {
          if (Serial_ON)
            {
              Serial.println("####vvvv####vvvv####vvvv####vvvv####vvvv####vvvv####");
              Serial.print("This RecMessage is NOT for Master Unit, ");
              Serial.print("packetSize =");
              Serial.print(packetSize);
              Serial.print("   recipient =");
              Serial.print(recipient, HEX);
              Serial.print("   from > ");
              Serial.println(sender, HEX);
              Serial.print("RecMessage >");
              Serial.println(RecMessage);
              Serial.print("LoRaRSSI >");
              Serial.println(LoRaRSSI);
            }
          return;
        }

      if (Serial_ON)
        {
          Serial.println("####========== Data Received ==========####");
          Serial.print("Rec packetSize > ");
          Serial.print(packetSize);
          Serial.print(" - recipient > ");
          Serial.print(recipient, HEX);
          Serial.print(" -  From Sender > ");
          Serial.println(sender, HEX);
          Serial.print("Message >");
          Serial.println(RecMessage);
          Serial.print("LoRaRSSI > ");
          Serial.println(LoRaRSSI);
        }

      //  Check incomingLength with RecMessage.length();
      if (incomingLength != RecMessage.length())
        {
          Serial.println("*****  ERROR: RecMessage length does not match incoming Data length");
          Serial.println("There may be a problem with the Data");

          ExtractData(RecMessage);

          Serial.print("Received Message >> ");
          Serial.println(RecMessage);
          Serial.print("LoRaRSSI > ");
          Serial.println(LoRaRSSI);

          return;  // skip rest of function
        }
      if (Serial_ON)
        {
          Serial.println("Line 1879 - Go ExtractData(RecMessage)");
        }

      ExtractData(RecMessage);

      //  Check first 2 bytes for correct ID of Recipient and Sender
      if (recipient == localAddress && RecTankNameLoc == "Header")  // 0xFF ==  Master Unit
        {
          if (RecWake == "BootUp")
            {
              if (Serial_ON_HeaderRec)
                {
                  Serial.println(">>>>>>>>>>>>>  Header BootUp Message  <<<<<<<<<<<<<<<<<<");
                  Serial.print("Line 1894 - Received BootUp - Sending ");
                  Serial.println(RecWake);
                  Serial.print("Received Message >> ");
                  Serial.println(RecMessage);
                }

              // RecWake = SndSleepMode[1];

              RequestData(1);  //  Send Sleep or Awake back to Header TankStoreSortedHistory();

              HeaderAwakeTime = millis();  //  Start counting in case Awake Mode sent and forgotten about 15 mins max..
            }
        }
      else  // recipent IS Master and "Awake" from other Send Units
        {
          if (Serial_ON)
            {
              Serial.println("####vvvv####vvvv####vvvv####vvvv####vvvv####vvvv####");
              Serial.print("Recieved Info from >");
              Serial.print(RecTankNameLoc);
              Serial.print("<    <<<<>>>>>>  LoRaRSSI >");
              Serial.println(LoRaRSSI);
              Serial.print("RecMessage >");
              Serial.println(RecMessage);
            }
        }

      ShowTFTdata();

      if (RecTankNameLoc == "Header")
        {
          StoreSortedHistory();
        }
    }
  packetSize = 0;
}

void ExtractData(String RecNewMessage)  //  ************************************
{

  RecNewMessage = " " + RecNewMessage;  //  Gotta fix this

  int incomingLength = RecNewMessage.length();

  int StPos = 0;
  int FinPos;
  int VarCnt = 1;  //  Part of message to seperate into Variables

  for (FinPos = 0; FinPos < incomingLength; FinPos++)
    {
      // Serial.print("Char > ");
      // Serial.println(RecNewMessage.substring(FinPos, FinPos + 1));

      if (RecNewMessage.substring(FinPos, FinPos + 1) == ":")
        {

          switch (VarCnt)
            {
              case 1:  // TankNameLoc  re Tank Name
                RecTankNameLoc = RecNewMessage.substring(StPos + 1, FinPos);

                //  Serial.print("RecTankNameLoc > ");
                //  Serial.println(RecTankNameLoc);

                if (RecTankNameLoc == "Header")
                  {
                    if (Serial_ON_HeaderRec)
                      {
                        Serial.println("               TankName  WatLv Temp  Pow   Bat   Sol  STm  Message");
                        Serial.print("Line 1961 - RecNewMessage > ");
                        Serial.println(RecNewMessage);
                      }
                  }

                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 2:  // Water Level Distance
                RecWLdist = RecNewMessage.substring(StPos, FinPos);

                //  Serial.print("RecWLdist > ");
                //  Serial.println(RecWLdist);

                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 3:  // Temperature
                RecTemp = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 4:  // Power Volts
                PowVolts = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 5:  // Battery Volts
                BatVolts = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 6:  // Solar Volts
                SolVolts = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 7:  // Sleep Time
                RecTimeSleeping = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 8:  // BootUp - Awake or Sleep
                RecWake = RecNewMessage.substring(StPos, FinPos);
                StPos = FinPos + 1;
                VarCnt++;
                break;
            }
        }
    }
}

void ShowTFTdata()  //  *******************************************
{
  FontType = 2;
  LnSpc = 16;

  if (RecTankNameLoc == "Header")
    {
      if (Serial_ON_HeaderRec)
        {
          Serial.println("####================####");
          Serial.print("Line 2030 - TankNameLoc >");
          Serial.println(RecTankNameLoc);

          Serial.print("Recieved Message at >");
          Serial.println(LongTime);

          Serial.print("Sensor Water (Actual) Distance >");
          Serial.println(RecWLdist);  //  from sensor before substraction

          Serial.print("Temp > ");
          Serial.println(RecTemp);

          Serial.print("Power Volts Reads >");
          Serial.println(PowVolts);

          Serial.print("Battery Volts Reads >");
          Serial.println(BatVolts);

          Serial.print("Solar Volts Reads >");
          Serial.println(SolVolts);

          Serial.print("Rec Sleep Time >");
          Serial.println(RecTimeSleeping);

          Serial.print("RecWake (Awake or Sleep) >");
          Serial.print(RecWake);
          Serial.println("< ");
        }

      DataSent[1] = "Recd";  //  Data Sent & Received

      RecWaterLevel = RecWLdist.toFloat() - HgtSensor[1];  //  Substract Height of Sensor to Get Height of Water

      if (RecWaterLevel < 0) RecWaterLevel = 0;  //  FULL TANK

      //  Closest Sensor can read accurately is 220mm - anything less is Zero.

      if (RecWaterLevel != LstRecWaterLevel)
        {
          HeaderAwakeTime = millis();  //  Start counting in case Awake Mode sent and forgotten about 15 mins max..

          //  Is reset if Distance Changes
          LstRecWaterLevel = RecWaterLevel;
        }

      SprHeaderLandS();
    }

  if (RecTankNameLoc == "DBL Main")
    {
      NxtLneY = 135;

      DataSent[2] = "Recd";  //  Data Sent & Received

      RecWaterLevel = RecWLdist.toFloat() - HgtSensor[2];  //  Substract Height of Sensor to Get Height of Water

      if (RecWaterLevel < 0) RecWaterLevel = 0;  //  FULL TANK

      if (Serial_ON)
        {
          Serial.println("####================####");
          Serial.print("TankNameLoc >");
          Serial.println(RecTankNameLoc);

          Serial.print("Water Distance >");
          Serial.println(RecWaterLevel);

          // Serial.print("Temp >");
          // Serial.println(RecTemp);

          // Serial.print("Bat Volts >");
          // Serial.println(BatVolts);

          Serial.print("Sleep Time >");
          Serial.println(RecTimeSleeping);

          Serial.print("RecWake (Awake or Sleep) >>");
          Serial.println(RecWake);
        }

      SprDBL_MainLandS();
    }

  if (RecTankNameLoc == "uDeck")
    {
      NxtLneY = 220;

      DataSent[3] = "Recd";  //  Data Sent & Received

      RecWaterLevel = RecWLdist.toFloat() - HgtSensor[3];  //  Substract Height of Sensor to Get Height of Water

      if (RecWaterLevel < 0) RecWaterLevel = 0;  //  FULL TANK


      if (Serial_ON)
        {
          Serial.println("####================####");
          Serial.print("TankNameLoc >");
          Serial.println(RecTankNameLoc);

          Serial.print("Water Level Distance >");
          Serial.println(RecWLdist);

          // Serial.print("Temp >");
          // Serial.println(RecTemp);

          // Serial.print("Bat Volts >");
          // Serial.println(BatVolts);

          Serial.print("Sleep Time >");
          Serial.println(RecTimeSleeping);

          Serial.print("RecWake (Awake or Sleep) >>>");
          Serial.println(RecWake);
        }

      SpruDeckLandS();
    }

  delay(10);  //  was 100
}

void sendMessage(String SendMessageNow)  //  **************************************
{
  //Serial.println("void sendMessage(String SendMessageNow)");

  //digitalWrite(SendLED, HIGH);  //  Green

  //Serial.println("Sending Tank Name " + TankNameLoc);

  if (Serial_ON)
    {
      Serial.print("Line 2162 -  >> sender ");
      Serial.print(localAddress, HEX);
      Serial.print(" > destination ");
      Serial.print(destination, HEX);

      Serial.print(" - Sending Message == ");
      Serial.println(SendMessageNow);
    }

  LoRa.beginPacket();  // start packet

  LoRa.write(destination);   // add destination address
  LoRa.write(localAddress);  // add sender address

  //LoRa.write(msgCount);                 // add SendMessageNow ID

  LoRa.write(SendMessageNow.length());  // add payload  length
  LoRa.print(SendMessageNow);           // add payload

  delay(10);  //  was 20

  LoRa.endPacket();  // finish packet and send it

  //msgCount++;                           // increment SendMessageNow ID

  //delay(250);  //  delay only for Green LED
  // digitalWrite(SendLED, LOW);
}

void RequestData(int TankNumb)  //  *******************************************
{
  SwitchOnLoRa();

  if (TankNumb == 1)  //  Header Tank
    {

      //  Display request from WebPage for Sleep or Awake
      // Serial.print("**************Mode_1_Done 0=false 1=true >");
      // Serial.println(Mode_1_Done);
      // Serial.print("SndSleepMode[1] >");
      // Serial.println(SndSleepMode[1]);

      destination = 0xAA;  //  Header Tank

      CalcSleep();  //  always calc Sleep/Awake time so it's ready to send..

      if (Serial_ON_Header || Serial_ON)
        {
          Serial.print("Line 2210 - Current Time: ");
          Serial.print(LongTime);
          Serial.print("   CalcSleep Done");
          Serial.print("   SleepTime[1]");
          Serial.print(String(SleepTime[1]));
          Serial.print("  SleepMode[1] set to Sleep? >");
          Serial.println(SleepMode[1]);

          if (SleepMode[1] == "Sleep")
            {
              //Serial.println("Send Header to Sleep  zzzzzZZZZZZzzzzz......");
              Serial.print("Line 2219 - Sent Header to Sleep @ ");
              Serial.print(LongTime);
              Serial.print("   NxtAwakeTimeAmPm[1] ==");
              Serial.println(NxtAwakeTimeAmPm[1]);
            }
        }

      //  Dummy or "Header"      Awake or Sleep            20 minutes or less
      TheMessage = "Header";
      TheMessage = TheMessage + ":" + SleepMode[1] + ": " + String(SleepTime[1]) + ":";

      if (Serial_ON_Header || Serial_ON)
        {
          Serial.print("Line 2232 - Sending Message == ");
          Serial.println(TheMessage);
        }

      sendMessage(TheMessage);

      DataSent[TankNumb] = "Sent";  //  Set this to see changes..

      if (Serial_ON)
        {
          Serial.print("Sending Request to > ");
          Serial.println(destination, HEX);
        }
      //  End Header **********************
    }

  if (TankNumb == 2)
    {

      destination = 0xBB;  //  DBL Main
      TheMessage = "DBL Main";

      sendMessage(TheMessage);

      DataSent[TankNumb] = "Sent";

      if (Serial_ON)
        {
          Serial.print("Sending Request to DBL Main > ");
          Serial.println(destination, HEX);
        }
    }

  if (TankNumb == 3)
    {
      //  Send Request to Tank 3

      destination = 0xCC;  //  uDeck
      TheMessage = "uDeck";

      sendMessage(TheMessage);

      DataSent[TankNumb] = "Sent";

      if (Serial_ON)
        {
          Serial.print("Sending Request to uDeck > ");
          Serial.println(destination, HEX);
        }
    }
}

void CalcSleep()  //  ************************************************
{
  SleepTime[1] = 60 - TwMin;  //  Number of minutes to next Hour..  if TwMin = 8 => 52

  int Cnt = 1;  //  if less than 20mins then TwMin > 40

  while (SleepTime[1] > 20)  //  Keeps cycling until SleepTime[1] is less than 20
    {
      SleepTime[1] = SleepTime[1] - 20;

      Cnt = Cnt + 1;
    }

  if (SleepTime[1] <= 2) SleepTime[1] = SleepTime[1] + 20;  //  Seems to be needed..

  if (Cnt == 1)  //  Therefore TwMin > 40 next 20 mins will take it over the hour mark
    {
      if (TwHour + 1 <= 11)  //  work out AM or PM  - if TwHour = 12 ( + 1) == 13..  Therefore 12pm
        {                    //  likewise if TwHour = 11 (+ 1) == 12..  Therefore 11am
          NxtAwakeTimeAmPm[1] = String(TwHour + 1) + ":00am";
        }
      else if (TwHour + 1 == 12)
        {
          NxtAwakeTimeAmPm[1] = String(TwHour + 1) + ":00pm";
        }
      else
        {
          NxtAwakeTimeAmPm[1] = String(TwHour + 1 - 12) + ":00pm";
        }
    }

  if (Cnt == 2)  //  Therefore TwMin > 20 mins  >>>  next min is 40
    {
      if (TwHour < 12)
        {
          NxtAwakeTimeAmPm[1] = String(TwHour) + ":40" + "am";
        }
      else if (TwHour == 12)
        {
          NxtAwakeTimeAmPm[1] = String(TwHour) + ":40pm";
        }
      else
        {
          NxtAwakeTimeAmPm[1] = String(TwHour - 12) + ":40pm";
        }
    }


  if (Cnt == 3)  //  Therefore TwMin > 00 mins  >>>  next min is 20
    {
      if (TwHour < 12)
        {
          NxtAwakeTimeAmPm[1] = String(TwHour) + ":20" + "am";
        }
      else if (TwHour == 12)
        {
          NxtAwakeTimeAmPm[1] = String(TwHour) + ":20pm";
        }
      else
        {
          NxtAwakeTimeAmPm[1] = String(TwHour - 12) + ":20pm";
        }
    }

  if (Serial_ON_Header)
    {
      Serial.print("Line 2350 - TwHour ==");
      Serial.print(TwHour);

      Serial.print("  TwMin ==");
      Serial.print(TwMin);

      Serial.print("  Sleep Time ==");
      Serial.println(SleepTime[1]);
    }
}

void CheckWebChanges()  //  *******************************************************
{

  //  Check for changes in Numb of Tanks  ****************************
  if (server.hasArg("NumbTanks_1"))
    {
      // Extract the updated value from the form data
      float newNumbTanks = server.arg("NumbTanks_1").toFloat();

      // Update the NumbTanks[1] variable
      NumbTanks[1] = newNumbTanks;

      if (NumbTanks[1] != LstNumbTanks[1])
        {
          if (Serial_ON_Header || Serial_ON)
            {
              Serial.print("*********** Update EEPROMs for NumbTanks[1] :");
              Serial.println(NumbTanks[1]);
            }
          SaveTankData();
          LstNumbTanks[1] = NumbTanks[1];
        }
    }

  if (server.hasArg("NumbTanks_2"))
    {
      // Extract the updated value from the form data
      float newNumbTanks = server.arg("NumbTanks_2").toFloat();

      // Update the NumbTanks[2] variable
      NumbTanks[2] = newNumbTanks;

      if (NumbTanks[2] != LstNumbTanks[2])
        {
          if (Serial_ON_Header || Serial_ON)
            {
              Serial.print("*********** Update EEPROMs for NumbTanks[2] :");
              Serial.println(NumbTanks[2]);
            }
          SaveTankData();
          LstNumbTanks[2] = NumbTanks[2];
        }
    }

  if (server.hasArg("NumbTanks_3"))
    {
      // Extract the updated value from the form data
      float newNumbTanks = server.arg("NumbTanks_3").toFloat();

      // Update the NumbTanks[3] variable
      NumbTanks[3] = newNumbTanks;

      if (NumbTanks[3] != LstNumbTanks[3])
        {
          if (Serial_ON_Header || Serial_ON)
            {
              Serial.print("*********** Update EEPROMs for NumbTanks[3] :");
              Serial.println(NumbTanks[3]);
            }
          SaveTankData();
          LstNumbTanks[3] = NumbTanks[2];
        }
    }

  //  Check for changes in Tank Heights  ****************************
  if (server.hasArg("TankHeight_1"))
    {
      // Extract the updated value from the form data
      float newTankHeight = server.arg("TankHeight_1").toFloat();

      // Update the TankHeight[1] variable
      TankHeight[1] = newTankHeight;

      if (TankHeight[1] != LstTankHeight[1])
        {
          if (Serial_ON_Header || Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankHeight[1]");
            }
          SaveTankData();
          LstTankHeight[1] = TankHeight[1];
        }
    }

  if (server.hasArg("TankHeight_2"))
    {
      // Extract the updated value from the form data
      float newTankHeight = server.arg("TankHeight_2").toFloat();

      // Update the TankHeight[2] variable
      TankHeight[2] = newTankHeight;

      if (TankHeight[2] != LstTankHeight[2])
        {
          if (Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankHeight[2]");
            }
          SaveTankData();
          LstTankHeight[2] = TankHeight[2];
        }
    }

  if (server.hasArg("TankHeight_3"))
    {
      // Extract the updated value from the form data
      float newTankHeight = server.arg("TankHeight_3").toFloat();

      // Update the TankHeight[3] variable
      TankHeight[3] = newTankHeight;

      if (TankHeight[3] != LstTankHeight[3])
        {
          if (Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankHeight[3]");
            }
          SaveTankData();
          LstTankHeight[3] = TankHeight[3];
        }
    }

  //  Check for changes in Tank Diameters  ****************************
  if (server.hasArg("TankDia_1"))
    {
      // Extract the updated value from the form data
      float newTankDia = server.arg("TankDia_1").toFloat();

      // Update the TankHeight[1] variable
      TankDia[1] = newTankDia;

      if (TankDia[1] != LstTankDia[1])
        {
          if (Serial_ON_Header || Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankDia[1]");
            }
          SaveTankData();
          LstTankDia[1] = TankDia[1];
        }
    }

  if (server.hasArg("TankDia_2"))
    {
      // Extract the updated value from the form data
      float newTankDia = server.arg("TankDia_2").toFloat();

      // Update the TankHeight[2] variable
      TankDia[2] = newTankDia;

      if (TankDia[2] != LstTankDia[2])
        {
          if (Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankDia[2]");
            }
          SaveTankData();
          LstTankDia[2] = TankDia[2];
        }
    }

  if (server.hasArg("TankDia_3"))
    {
      // Extract the updated value from the form data
      float newTankDia = server.arg("TankDia_3").toFloat();

      // Update the TankHeight[3] variable
      TankDia[3] = newTankDia;

      if (TankDia[3] != LstTankDia[3])
        {
          if (Serial_ON)
            {
              Serial.println("*********** Update EEPROMs for TankDia[3]");
            }
          SaveTankData();
          LstTankDia[3] = TankDia[3];
        }
    }

  //  Check for changes in SleepMode[1]  ****************************
  if (server.hasArg("Mode_1"))  // && !Mode_1_Done)
    {
      LstSleepMode[1] = SndSleepMode[1];

      // Extract the updated value from the form data
      String newMode = server.arg("Mode_1");

      // Update the Mode_1 variable
      if (newMode == "Sleep")
        {
          SleepMode[1] = "Sleep";
          SndSleepMode[1] = "Sleep";

          SetHeaderSleep = true;

          if (LstSleepMode[1] != SleepMode[1])
            {
              if (Serial_ON_Header)
                {
                  Serial.println("******** NewMode changed to Sleep **********");
                }
            }
        }

      if (newMode == "Awake")
        {
          SleepMode[1] = "Awake";
          SndSleepMode[1] = "Awake";
          SetHeaderSleep = false;

          HeaderAwakeTime = millis();  //  Start counting in case Awake Mode sent and forgotten about 15 mins max..
          //  Is reset if Distance Changes

          if (LstSleepMode[1] != SleepMode[1])
            {
              if (Serial_ON_Header)
                {
                  Serial.println("******** NewMode changed to Awake **********");
                }
            }
        }

      if (LstSleepMode[1] != SndSleepMode[1])  //  SleepMode has Changed - Send to Header..
        {
          RequestData(1);  //  Send new Sleep Mode to Sender Unit

          HeaderAwakeTime = millis();  //  Start counting in case Awake Mode sent and forgotten about 15 mins max..

          LstSleepMode[1] = SleepMode[1];

          ShowTFTdata();  //  Update TFT Screen..
        }
    }

  //  Check for changes in Screen Output  ****************************
  if (server.hasArg("Mode_4"))
    {
      String ScreenMode = "Screen";
      // Extract the updated value from the form data
      ScreenMode = server.arg("Mode_4");

      // Update the Mode_1 variable
      if (ScreenMode == "Screen")
        {
          //Serial.print("Goto Screen -->  ");
          //Serial.println(ScreenMode);
          PrntData = false;
          HistoryOutput = "Screen";
        }
      else
        {
          // Serial.print("Goto Screen -->  ");
          //Serial.println(ScreenMode);
          PrntData = true;
          HistoryOutput = "History";
        }
    }
}

void CheckSleepHeader()  //  *****************************************
{
  if (SndSleepMode[1] == "Sleep" && !SetHeaderSleep)
    {
      SetHeaderSleep = true;
    }

  if (SetHeaderSleep)  //  && DataSent[1] == "Recd"
    {
      //Serial.println("if (SetHeaderSleep && !CalcSleepDone)");

      if (LstSetHeaderSleep != SetHeaderSleep)
        {
          if (Serial_ON)
            {
              Serial.print("SetHeaderSleep 1 == true: ");
              Serial.println(SetHeaderSleep);
            }

          LstSetHeaderSleep = SetHeaderSleep;
        }

      SleepMode[1] = "Sleep";  //  This when sent, puts the Header Tank Sensor to Sleep

      SetHeaderSleep = false;
      //Mode_1_Done = false;
      SndSleepMode[1] = "Sleep";  //  Set to Sleep - Send to Header Tank
    }

  if (DataSent[1] == "Recd" && SndSleepMode[1] != "Sleep")
    {
      NxtAwakeTimeAmPm[1] = "NA";
    }
}

void StoreSortedHistory()
{
  if (HistoryCnt == 1)  //  Store first data in first spot
    {
      HistoryData[1][0] = CurrentDate;
      HistoryData[1][1] = TimeStamp[1];
      HistoryData[1][2] = String(CurrentVol[1]);
      HistoryData[1][3] = Dist[1];
      HistoryData[1][4] = PowerVolts[1];
      HistoryData[1][5] = BatteryVolts[1];
      HistoryData[1][6] = SolarVolts[1];
      HistoryData[1][7] = Temp[1];

      if (Serial_ON_HeaderRec)
        {
          Serial.println("Line 2671 - >>>><<<<  History Count 1 ==  >>>><<<<");
        }

      HistoryCnt++;
    }
  else  //  Move data up 1 slot up and store new data in spot [1]
    {
      //  move data up before storing new data
      for (int i = HistoryCnt - 1; i >= 1; i--)  //  count down from max HistoryCnt down to 1
        {
          HistoryData[i + 1][0] = HistoryData[i][0];
          HistoryData[i + 1][1] = HistoryData[i][1];
          HistoryData[i + 1][2] = HistoryData[i][2];
          HistoryData[i + 1][3] = HistoryData[i][3];
          HistoryData[i + 1][4] = HistoryData[i][4];
          HistoryData[i + 1][5] = HistoryData[i][5];
          HistoryData[i + 1][6] = HistoryData[i][6];
          HistoryData[i + 1][7] = HistoryData[i][7];
        }

      //  Store Latest Data in top spot
      HistoryData[1][0] = CurrentDate;
      HistoryData[1][1] = TimeStamp[1];
      HistoryData[1][2] = String(CurrentVol[1]);
      HistoryData[1][3] = Dist[1];
      HistoryData[1][4] = PowerVolts[1];
      HistoryData[1][5] = BatteryVolts[1];
      HistoryData[1][6] = SolarVolts[1];
      HistoryData[1][7] = Temp[1];

      HistoryCnt++;

      if (Serial_ON_HeaderRec)
        {
          Serial.print("Line 2705 - >>>> HistoryCnt++ <<<<  History Count 2 = >");
          Serial.println(HistoryCnt);
        }

      if (HistoryCnt > MaxMem)
        {
          HistoryCnt = MaxMem;  //  Max Memory allaction is 720
        }
    }

  if (Serial_ON_HeaderRec)
    {
      Serial.println("Line 2773 - =========== History Data ==================================");

      //  Printout all History Data
      for (int i = 0; i < HistoryCnt; i++)  //  HistoryData[0][j]) contains Column Names
        {
          for (int j = 0; j <= 7; j++)
            {
              Serial.print(HistoryData[i][j]);
              Serial.print(" ");
            }
          Serial.println();
        }
      Serial.println("============ History Data =================================");
    }
}

void CheckHeaderAwake()  //  **********************************************************
{
  //  Cheak Header is not in Awake Mode and forgotten about..  15mins after last diff in Distance..

  if (SleepMode[1] == "Awake" && DataSent[1] == "Recd")
    {                                           //  If Awake Mode sent and forgotten about 15 mins max..
      if (millis() - HeaderAwakeTime > 120000)  //  15min * 60 * 1000 == 900000 msec  Testing >> 2min x 60 x 1000 = 120000
        {
          SleepMode[1] = "Sleep";

          SndSleepMode[1] = "Sleep";

          if (Serial_ON_Header)
            {
              Serial.println("*******  You Forgot about Header - Header is going to Sleep  *******");
            }

          //  Send Header to Sleep - DO NOT ASK FOR A REPLY.. Header should wake up in 5 mins and proceed as normal.
          RequestData(1);  //  send Header to sleep
        }
    }
}

void loop()  //  **********************************************************************
{
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  if (TouchEnabled)
    {
      TouchSpot();
    }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  ShowTime();

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  server.handleClient();  //  Web Server

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  if (ReqData && MaxNoUnit > 1)
    {
      //  needs timing with millis()
      if (millis() - ReqDataDelay >= StartReqData)  //  0 for 1st then 2sec for 2nd & remaining tank sensors
        {
          if (Serial_ON)
            {
              Serial.println(">>>>>>><<<<<<<<>>>>>>><<<<<<<<>>>>>>><<<<<<<<>>>>>>><<<<<<<<");
              Serial.print(">>>>  Init Boot - Request Data from Unit : ");
              Serial.println(TankNameLoc[UnitCnt]);
            }

          ReqDataDelay = millis();

          RequestData(UnitCnt);

          StartReqData = 2000;

          UnitCnt++;

          if (UnitCnt > MaxNoUnit)  //  MaxNoUnit
            {
              ReqData = false;  //  clears ReqData..

              if (Serial_ON)
                {
                  Serial.println(">>>>>>>>>>>>>>> All Unit's data has been requested...<<<<<<<<<<<<<<<<<");
                }
            }
        }
    }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  //  Every 5 minutes, check the WiFi is working - If NOT, Reconnect
  if (millis() - WiFiCheckDelay >= WiFiCheckTime)  //  5 minutes == 5 * 60 * 1000 ==  300000 msec
    {
      WiFiCheckDelay = millis();

      if (Serial_ON_Header)
        {
          Serial.println("Checking WiFi connection <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        }

      if (WiFi.status() != WL_CONNECTED)
        {
          Serial.print("WiFi connection LOST @ ");
          Serial.println(LongTime);
          //  Serial.println("Reconnecting WiFi");

          WiFiOK = false;

          //  ShowMyWifi();  //  Show WiFi Icon

          WiFiCnt = 0;  //  reset WiFiCnt

          // WiFi.disconnect();

          WiFi.mode(WIFI_STA);

          delay(100);

          WiFi.begin(ssid, password);  //  If use Char*

          //  Try to Reconnect
          while (WiFi.status() != WL_CONNECTED && WiFiCnt < 10)
            {
              WiFiCnt++;

              //WiFi.reconnect();

              if (Serial_ON_Header)
                {
                  Serial.print(".");
                  Serial.print(WiFiCnt);
                }
              delay(500);
            }

          Serial.println();

          if (WiFi.status() != WL_CONNECTED)
            {
              Serial.println("WiFi Did Not Reconnected");
              //  Try again in 1 mins
              WiFiCheckTime = 60000;  //  recheck in 1 mins == 1 * 60 * 1000 == 60000
              WiFiOK = false;
            }
          else
            {
              Serial.println("WiFi Reconnected");
              WiFiOK = true;
              WiFiCheckTime = 300000;
            }
        }
      else
        {
          Serial.print("WiFi Tested & OK @ ");
          Serial.println(LongTime);
          WiFiCheckTime = 300000;  //  recheck in 5 mins
          WiFiOK = true;
        }

      ShowMyWifi();  //  Show WiFi Icon
    }

    // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  CheckWebChanges();

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  CheckSleepHeader();

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  // parse for a packet, and call onReceive with the result:
  //onReceive(LoRa.parsePacket());

  CheckHeaderAwake();  //  Cheak Header is not in Awake Mode and forgotten about..

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  if (ReTryTime && millis() - ReTryMillis > 300000)  // Retry in 5 minutes = 30000 msec
    {
      if (Serial_ON)
        {
          Serial.println("Time Update failed - Try again now");
        }
      GetTimeSilent = true;
      GetWWWTime();
    }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

String getPage()
{  //                                                     Pages Refreshes every 10 secs
  String page = "<!DOCTYPE html><html><head> <meta http-equiv=\"Refresh\" content=\'10\'>";

  page += "<style> table, th, td {border: 1px solid; border-collapse: collapse; border-color:#0000ff;}";  //  solid black;  1px solid
  page += "body{background-color:powderblue;}";

  page += "h1{color:blue; font-family:verdana; font-size:200%; text-align: center;}";
  page += "h2{color:black; font-family:verdana; font-size:100%; text-align: centre;}</style>";

  page += "<style>.svg-container {position: relative;}";

  page += ".svg-image {position: absolute; top: 0; left: 0; z-index: 1; }";

  page += "</style></head><body>";

  page += "<h1>" + WebTitle + "</h1>";  //  LOFTY WATER INC

  page += "<table style='width:100%'><tr><th>Header Tank</th><th>DBL Main Tanks</th><th>uDeck Tanks</th></tr>";  //  svg-align=center

  page += "<tr>";

  //  =========================  Graphics of Tanks  ==================================================

  // ==========  Tank Graphics Fill with Water colour - 1st Column  Header Tank  ==============================

  page += "<td><div class='svg-container'><svg width=100% height=180><!-- Sets up Space for Graphics -->";                   //
  page += "<rect x=50 y=20 rx=20 ry=10 width=150 height=150 style=fill:blue;stroke:black;stroke-width:5;opacity:1/></svg>";  // Tank

  page += "<svg class='svg-image' width=100% height=180><!-- Show TopCap -->";
  page += "<rect x=70 y=10 rx=3 ry=3 width=40 height=8 style=fill:grey;stroke:black;stroke-width:2;opacity:1/></svg>";  // Top Cap

  //  This next line is Graphics that blanks out top part to show Water Level -->";
  //  If Tank Height is 150 then height 75 (Below) shows Water Level under Halfway -->";
  page += "<svg class='svg-image' width=100% height=180><!-- Show Water Level -->";
  page += "<rect x=50 y=20 rx=10 ry=10 width=150 height=" + String(CalcHt[1]) + " style=fill:lightsteelblue;stroke:grey;stroke-width:5;opacity:1/></svg>";

  page += "<svg class='svg-image' width=100% height=180><!-- Show Text -->";                                                                                                  //  add Volume as Percentage to SVG area
  page += "<text x=80 y=100 font-family='Arial Black' fill='black' stroke-width='2px' stroke='white' font-size='40' fill=black>" + String(PercentVol[1]) + "%</text></svg>";  //  add text to SVG area

  //page += "<text x=80 y=100 font-family='Tahoma' weight='bold' font-size='40' fill=black>" + String(PercentVol[1]) + "%</text></svg>"; //  add text to SVG area

  page += "</div></td>";

  //  ========== Tank Graphics Fill with Water colour - 2nd Column  DBL Main Tank  ==============================

  page += "<td><div class ='svg-container'><svg width=100% height=180>";

  page += "<rect x=50 y=20 rx=20 ry=10 width=150 height=150 style=fill:blue;stroke:black;stroke-width:5;opacity:1/></svg>";

  page += "<svg class='svg-image' width=100% height=180><!-- Show TopCap -->";
  page += "<rect x=70 y=10 rx=3 ry=3 width=40 height=8 style=fill:grey;stroke:black;stroke-width:2;opacity:1/></svg>";  // Top Cap

  //  This next line is Graphics that blanks out top part to show Water Level -->";
  page += "<svg class='svg-image' width=100% height=180><!-- Show Water Level -->";
  page += "<rect x=50 y=20 rx=10 ry=10 width=148 height=" + String(CalcHt[2]) + " style=fill:lightsteelblue;stroke:grey;stroke-width:5;opacity:1/></svg>";

  page += "<svg class='svg-image' width=100% height=180><!-- Show Text -->";                                                                                                  //  add Volume as Percentage to SVG area
  page += "<text x=80 y=100 font-family='Arial Black' fill='black' stroke-width='2px' stroke='white' font-size='40' fill=black>" + String(PercentVol[2]) + "%</text></svg>";  //  add text to SVG area

  page += "</div></td>";  //

  //  ========== Tank Graphics Fill with Water colour - 3rd Column  uDeck Tank  ===================================

  page += "<td><div class='svg-container'><svg width=100% height=180>";  //      text-align: center  svg-align=center

  page += "<rect x=50 y=20 rx=20 ry=10 width=150 height=150 style=fill:blue;stroke:black;stroke-width:5;opacity:1/></svg>";

  page += "<svg class='svg-image' width=100% height=180><!-- Show TopCap -->";
  page += "<rect x=70 y=10 rx=3 ry=3 width=40 height=8 style=fill:grey;stroke:black;stroke-width:2;opacity:1/></svg>";  // Top Cap

  //  This next line is Graphics foe top of Water
  page += "<svg class='svg-image' width=100% height=180><!-- Show Water Level -->";
  page += "<rect x=50 y=20 rx=10 ry=10 width=148 height=" + String(CalcHt[3]) + " style=fill:lightsteelblue;stroke:grey;stroke-width:5;opacity:1/></svg>";

  page += "<svg class='svg-image' width=100% height=180><!-- Show Text -->";                                                                                                  //  add Volume as Percentage to SVG area
  page += "<text x=80 y=100 font-family='Arial Black' fill='black' stroke-width='2px' stroke='white' font-size='40' fill=black>" + String(PercentVol[3]) + "%</text></svg>";  //  add text to SVG area

  page += "</div></td></tr>";

  //  ===========================  End of Graphics of Tanks  =======================================================

  page += "<tr><td>Current Volume = " + String(CurrentVol[1]) + " lts or " + String(CurrentVol[1] / 4.54601, 0) + " gals</td>";
  page += "<td>Current Volume = " + String(CurrentVol[2]) + " lts or " + String(CurrentVol[2] / 4.54601, 0) + " gals</td> ";
  page += "<td>Current Volume = " + String(CurrentVol[3]) + " lts or " + String(CurrentVol[3] / 4.54601, 0) + " gals</td></tr>";

  page += "<tr><td>Date/Time : " + CurrentDate + " : " + TimeStamp[1] + "</td>";
  page += "<td>Date/Time : " + CurrentDate + " : " + TimeStamp[2] + "</td>";
  page += "<td>Date/Time : " + CurrentDate + " : " + TimeStamp[3] + "</td></tr>";

  page += "<tr><td>Full Capacity = " + String(TankSize[1]) + " lts or " + String(TankSize[1] / 4.54601, 0) + " gals</td>";
  page += "<td>Full Capacity = " + String(TankSize[2]) + " lts or " + String(TankSize[2] / 4.54601, 0) + " gals</td>";
  page += "<td>Full Capacity = " + String(TankSize[3]) + " lts or " + String(TankSize[3] / 4.54601, 0) + " gals</td></tr>";


  //  ==========================================================================================

  //  No. of Tanks  1 Input
  page += "<tr><td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='NumbTanks_1'>No.of Tanks : </label>";
  page += "<input size=5 type='number' step='1' id='NumbTanks_1' name='NumbTanks_1' value=" + String(NumbTanks[1]) + ">";
  page += "<input type='submit' value='Send'></td>";

  //  No. of Tanks  2  Input
  page += "<td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='NumbTanks_2'>No.of Tanks : </label>";
  page += "<input size=5 type='number' step='1' id='NumbTanks_2' name='NumbTanks_2' value=" + String(NumbTanks[2]) + ">";
  page += "<input type='submit' value='Send'></td>";

  //  No. of Tanks  3  Input
  page += "<td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='NumbTanks_3'>No.of Tanks : </label>";
  page += "<input size=5 type='number' step='1' id='NumbTanks_3' name='NumbTanks_3' value=" + String(NumbTanks[3]) + ">";
  page += "<input type='submit' value='Send'></td></tr>";

  //  ==========================================================================================

  //  Tank Diameter Input
  page += "<tr><td>";
  page += "<form action='/' method='POST'>";                                                                            //  action_page.php
  page += "<label for='TankDia_1'>Tank Diameter : </label>";                                                            //  <br>
  page += "<input size=5 type='number' step='0.01' id='TankDia_1' name='TankDia_1' value=" + String(TankDia[1]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td>";

  page += "<td>";
  page += "<form action='/' method='POST'>";                                                                             //  action_page.php
  page += "<label for='TankDia_2'>Tank Diameter : </label>";                                                             //  <br>
  page += "<input size=5 type='number' step='0.01' id='TankDiat_2' name='TankDia_2' value=" + String(TankDia[2]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td>";

  page += "<td>";
  page += "<form action='/' method='POST'>";                                                                            //  action_page.php
  page += "<label for='TankDia_3'>Tank Diameter : </label>";                                                            //  <br>
  page += "<input size=5 type='number' step='0.01' id='TankDia_3' name='TankDia_3' value=" + String(TankDia[3]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td></tr>";

  //  ==========================================================================================

  //  Tank Height Input
  page += "<tr><td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='TankHeight_1'>Tank Height : </label>";
  page += "<input size=5 type='number' step='0.01' id='TankHeight_1' name='TankHeight_1' value=" + String(TankHeight[1]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td>";

  page += "<td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='TankHeight_2'>Tank Height : </label>";
  page += "<input size=5 type='number' step='0.01' id='TankHeight_2' name='TankHeight_2' value=" + String(TankHeight[2]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td>";

  page += "<td>";
  page += "<form action='/' method='POST'>";
  page += "<label for='TankHeight_3'>Tank Height : </label>";
  page += "<input size=5 type='number' step='0.01' id='TankHeight_3' name='TankHeight_3' value=" + String(TankHeight[3]) + ">";  //  <br>
  page += "<input type='submit' value='Send'></td></tr>";

  //  ==========================================================================================

  //  Distance Reading - Time Stamp - Power Volts - Battery Volts - Solar Volts - Temperature
  page += "<tr><td>Dist = " + Dist[1] + "m</td>";
  page += "<td>Dist = " + Dist[2] + "m</td>";
  page += "<td>Dist = " + Dist[3] + "m</td></tr>";
  page += "<tr><td>Power     = " + PowerVolts[1] + "v</td><td></td><td></td></tr>";
  page += "<tr><td>Battery   = " + BatteryVolts[1] + "v" + BatWarning + "</td><td></td><td></td></tr>";
  page += "<tr><td>Solar     = " + SolarVolts[1] + "v</td><td></td><td></td></tr>";
  page += "<tr><td>Temp      = " + Temp[1] + "c</td><td></td><td></td></tr>";

  // ==========================================================================================
  page += "<td>";
  page += "<form action='/action_page.php'>";
  page += "<input type='radio' id='sleep' name='Mode_1' value='Sleep'>";
  page += "<label for='sleep'>Sleep</label>";
  page += "<input type='radio' id='awake' name='Mode_1' value='Awake'>";
  page += "<label for='awake'>Awake</label>";
  page += "<input type='submit' value='Submit'>";
  page += "</form> Mode:" + SndSleepMode[1] + "</td>";

  //  This shows "Mode: Awake" for the other Sensors
  page += "<td>Mode:Awake</td><td>Mode:Awake</td>";

  /*   //  Next Lines are if you want to add Sleep mode to other Tank Sensors
    page += "<td>";
    page += "<form action='/action_page.php'>";
    page += "<input type='radio' id='sleep' name='Mode_2' value='Sleep'>";
    page += "<label for='sleep'>Sleep</label>";
    page += "<input type='radio' id='awake' name='Mode_2' value='Awake'>";
    page += "<label for='awake'>Awake</label>";
    page += "<input type='submit' value='Submit'>";
    page += "</form> Mode:" + SndSleepMode[2] + "</td>";

    page += "<td>";
    page += "<form action='/action_page.php'>";
    page += "<input type='radio' id='sleep' name='Mode_3' value='Sleep'>";
    page += "<label for='sleep'>Sleep</label>";
    page += "<input type='radio' id='awake' name='Mode_3' value='Awake'>";
    page += "<label for='awake'>Awake</label>";
    page += "<input type='submit' value='Submit'>";
    page += "</form> Mode:" + SndSleepMode[3] + "</td>";*/

  page += "</tr>";

  page += "<tr><td>Wake Up Time = " + NxtAwakeTimeAmPm[1] + "</td><td></td><td></td>";

  page += "</tr>";

  //  =========================================================================================

  page += "<tr><td>RSSI = " + String(RSSILoRa[1]) + "</td>";
  page += "<td>RSSI = " + String(RSSILoRa[2]) + "</td>";
  page += "<td>RSSI = " + String(RSSILoRa[3]) + "</td></tr>";

  page += "</table>";  //  </body></html>";

  page += "<form action='/' method='POST'>";
  page += "<ul><li>Screen : ";  //  <ul>  This is for New Line
  page += "<input type='submit' value='Refresh'>";

  //  ======================= History Paging  ===============================

  page += "<form>";
  page += "<li><input type='radio' id='screen' name='Mode_4' value='Screen'>";
  page += "<label for='screen'>Screen</label>";
  page += "<input type='radio' id='history' name='Mode_4' value='History'>";
  page += "<label for='history'>History</label>";
  page += "<input type='submit' value='Submit'></li>";
  page += "</form> Mode:" + HistoryOutput;

  page += "</body></html>";

  return page;
}

String dataPage()
{
  String AllData = "<!DOCTYPE html><html><head><meta http-equiv=\"Refresh\" content=\'60\'>";

  AllData += "<style> table, th, td {border: 1px solid; border-collapse: collapse; border-color:#0000ff;}";  //  solid black;
  AllData += "body{background-color:powderblue;}";

  AllData += "h1{color:blue; font-family:verdana; font-size:200%; text-align: center;}";
  AllData += "h2{color:black; font-family:verdana; font-size:100%; text-align: centre;}</style>";

  AllData += "<style>.svg-container {position: relative;}";

  AllData += ".svg-image {position: absolute; top: 0; left: 0; z-index: 1; }";

  AllData += "</style></head><body>";

  AllData += "<h1>" + WebTitle + " - History Data</h1>";  //  LOFTY WATER INC

  AllData += "<table style='width:100%'><tr><th>Header Tank</th></tr>";  //  <th>DBL Main Tanks</th><th>uDeck Tanks</th></tr>";  //  svg-align=center

  AllData += "<tr>";

  //  ======================= Go Back to Normal Screen Data  ===============================

  AllData += "<form>";
  AllData += "<li><input type='radio' id='screen' name='Mode_4' value='Screen'>";
  AllData += "<label for='screen'>Screen</label>";
  AllData += "<input type='radio' id='history' name='Mode_4' value='History'>";
  AllData += "<label for='history'>History</label>";
  AllData += "<input type='submit' value='Submit'></li>";
  AllData += "</form> Mode:" + HistoryOutput + "</td>";

  //  Show Data
  // Serial.println("=========== History Data on History Web Page =============================");

  //  Printout all History Data
  for (int i = 0; i < HistoryCnt; i++)  //  HistoryData[0][j]) contains Column Names
    {
      AllData += "<tr><td>";
      for (int j = 0; j <= 7; j++)
        {
          //   Serial.print(HistoryData[i][j]);
          //   Serial.print(" ");

          AllData += String(HistoryData[i][j]);  //  "<tr><td>" +

          if (j != 7) AllData += " ; ";  //  Doesn't add ";" to end of Line
        }
      //  Serial.println();
    }

  AllData += "</body></html>";

  return AllData;

  //  =========================================================================================
}
