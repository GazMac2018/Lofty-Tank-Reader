/*    Tank Level Detector - Send Unit - Version 2

      G. McAlister   18 October 2023

      *****************************************************************************************
      *
      *     Board Management : >>   LOLIN D32  (Verified - 12 Feb 2025
      *     Partition Scheme : Default
      *     
      *     ESP32-WROOM-DA Module (Certified 12 Feb 2025) Works with ESP32 'G' 
      *     
      ****************************************************************************************

      ************  Error Notes  ******************

      23 Feb 2026 - Header Unit - LEDs were flashing but in constant loop and no readings on 
      Master Unit.. 
      Solution: EEPROM was corrupt - Header name was He@der  where @ was a strange character.
                Reload EEPROM..   - Fixed and working..
      
      *****************************************************************************************
      
      Modification : 11 September 2024

      This modification has the following proceedure..

      1.  As the Master Unit will NOT request data, the Send Unit can NOT wait for a request.

      2.  All Units will only send Data if the Water Level changes by more then 1cm in either
          direction, up or down.

          OR when requested by the Master every 20 minutes..  (Time can be changed in this script)

      3.  The Header Unit upon awakening from Sleep Mode will check the level and it will send data.

      4.  The Send Units that are powered by Mains power will always be "Awake"

      5.  All Fixes or UpDates to be Recorded below in FIXES and named GFxx

      ********************************************************************************************
           FIXES  (All named GFxx)
           GF01 - 29 Dec 2024 - Avoid Looping if Sender Unit message not received


      ********************************************************************************************

      Using ::::
      HC SR04 - Ultrasonic Distance Sensor
      LoRa 915Mhz Transceiver

      This Script is a Sender Unit
      and so
      Using LoRa to send data along with other
      Send Units to a single Reciever Unit

      Send Units script: SendUnit-WaterLevel-LoRa-V2.ino

      BlueTooth is NOT used..

      ESPnow is NOT used..

      TFT Screen is NOT used..

      Varibles for LoRa
      String Unique Name  i.e.  TankName
      float Read Distance

      Receiver Unit Named: RecUnit-WaterLevel-Lora.ino

      ESP Preferebces is used for the EEPROM variable:  TankName

      ESP32 Used Pins List

      DHTPIN 32          //  Temperture DHT sensor used in Some Units (Optional)
      KY-013 Analog Temperature Sensor on Pin32

      RecRedLED = 13;       //  Red    Receive
      SendGrnLED = 12;      //  Green   Send

      LoRa
      csPin = 17;        //  LoRa radio chip select  L4  -- Set to Low to Select
      resetPin = 16;     //  LoRa radio reset  L3
      irqPin = 14;       //  R6  -  change for your board; must be a hardware interrupt pin

      Distance Sensor
      trigPin = 25;      //  defines pins numbers for Distance Sensor
      echoPin = 26;

      Pin for Voltage Monitoring (Optional)
      SVpin = 34    //  Solar Volts Monitor Pin - Max 5v
      BVpin = 35;   //  Battery Volts Monitor Pin - Max 4.2v - Min 3.2v
      PVpin = 33    //  Should always be around 3.2volts
*/

#include <SPI.h>
#include <WiFi.h>

#include <LoRa.h>

#include <math.h>

#include <Preferences.h>

#define RW_MODE false
#define RO_MODE true

Preferences preferences;

//  Special Boolean Variables for SerialPrint
boolean Serial_ON = false;       //  False will not display normal SerialPrint Data
boolean Serial_ON_Head = false;  //  False will not display Important SerialPrint Data
boolean Just_RSSI = false;       //  Just show the RSSI received
boolean Testing = false;        //  if true, ignores critical shut downs for testing only

#include "soc/soc.h"           //  only used to avoid BrownOut issue and used with..
#include "soc/rtc_cntl_reg.h"  //  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

//  Using a unified Temperature Sensor and not a Branded Type
float Temp;
boolean BootUp = true;  //  Do not wait to Send Data - ASAP

float TempCalibrate = 0;  // used for the temperature calbration
//int temperature;
int TempSensorDelay = 500;  //  Minimum Default Delay for Temperature Sensor
uint32_t LastTempSensorDelay = millis();
//float LastTemp;
//float TempC;
const int TempPin = 32;
int Tvo;

// Create the global variable
int incomingLength;
unsigned long long SleepMicroTime;
String MessageMode = "BootUp";  //  From Boot == BootUp to Notify Master to send Sleep or Awake
boolean FreqSet = false;

int SleepTime = 20;  // in minutes
int ReadCnt = 0;
long LastSleepDelay = millis();
int LstSleepTime = SleepTime;
String LstMessageMode = MessageMode;
boolean CriticalBatLev = false;
int CriticalCnt = 0;

//  if unit turns on and there is no response from Master Unit
//  Header Tank will NoResponse Count to n and if no response will shut down for 20 minutes..
//  If Header gets a Response from th Master Unit, the Header Sensor is usually told to go to sleep.
int NoResponseCnt = 0;
uint32_t NoResponseCntDelay = millis();
//  int NoResponseCntDelaySec = 10000;


int SendGrnLED = 12;                    //  was 27;  //  Green   Send
unsigned long StatusGrnLed = millis();  //  Flashes for 100ms every 1 sec
boolean StatusGrnLedON = false;
int GrnLEDTime = 500;

unsigned long StatusAliveGrn = millis();

unsigned long TestingGrnLed = millis();

int RecRedLED = 13;                     //  Red    Receive
unsigned long StatusRedLed = millis();  //  Flashes for 100ms every 1 sec
boolean StatusRedLedON = false;
int RedLEDTime = 1000;

String LoRaRSSI;

//String TankName;
String LstTankName;

//  EEPROM Variables using the ESP32 #include <Preferences.h>
int TrueData;

float LastDist;
float distance = 0;
float TestDist[10];
long duration;
uint32_t CheckLevelTime = millis();

int SVpin = 33;  //  Solar Volts In
int BVpin = 35;  //  Battery Volts Monitor Pin
int PVpin = 34;  //  Power Volts Monitor Pin ~ 3.3v

int RawPowVolts;  //  analogRead(PVpin)
int RawBatVolts;  //  analogRead(BVpin)
int RawSolVolts;  //  analogRead(SVpin)

float BatVolts;
float PowVolts;
float SolVolts;

// qsort() is a Arduino sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

//  LoRa varibles   ######################################################

const int csPin = 17;     // LoRa radio chip select  -- Set to Low to Select
const int resetPin = 16;  // LoRa radio reset
const int irqPin = 14;    // change for your board; must be a hardware interrupt pin

String TankNameLoc;  // TankNameLoc message
String SendMessage;  // Sensor Read Distance

byte localAddress;  // Writes address of this device set by TankNameLoc
// either 0xAA or 0xBB or 0xCC depending Tank Name (EEPROM Stored)
byte destination = 0xFF;  // Writes destination to send to (Master Rec Unit

byte recipient;  // Reads recipient address
byte sender;     // Reads sender address

String RecMessage;

//  LoRa varibles   ######################################################

// defines pins numbers for Distance Sensor
const int trigPin = 25;
const int echoPin = 26;

float mymap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define __FILENAME__ (strrchr("/"__FILE__, '/') + 1)


void setup()  //  ***SETUP***setup***SETUP***setup***SETUP***setup***SETUP***setup***SETUP***setup***
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // prevent brownouts by silencing them
  delay(100);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println();

  if (Serial_ON || Serial_ON_Head)
    {
      Serial.println();
      Serial.println("*********************************");
      Serial.println("Starting Setup for");
      Serial.println(__FILENAME__);  //  Printsout /Dir/Filename
      Serial.println("by G.McAlister - 09 Dec 2023");
      Serial.println("**********************************");
    }

  if (Serial_ON)
    {
      Serial.println("Serial_ON = true - All General Serial.print WILL be Shown");
    }

  if (Serial_ON_Head)
    {
      Serial.println("Serial_ON_Head = true - All Header Serial.print WILL be Shown");
      Serial.println();
    }

  pinMode(RecRedLED, OUTPUT);
  pinMode(SendGrnLED, OUTPUT);

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input

  pinMode(SVpin, INPUT);  //  GPIO 34  Solar Volts Monitor Pin
  pinMode(BVpin, INPUT);  //  GPIO 35  Battery Volts Monitor Pin
  pinMode(PVpin, INPUT);  //  GPIO 33  Power Volts Monitor Pin ~ 3.3v

  // adcAttachPin(BVpin);

  //  This just shows that the ESP32 has booted
  digitalWrite(RecRedLED, HIGH);
  delay(250);
  digitalWrite(RecRedLED, LOW);
  delay(250);
  digitalWrite(SendGrnLED, HIGH);
  delay(250);
  digitalWrite(SendGrnLED, LOW);
  delay(250);

  // ******************************************************************

  //  NOTE: Setup defaults for Sender Units
  //  NOTE: These are unique Location names for the Sender Units
  //  NOTE: Use other names if desired
  //  NOTE: Change in MasterRecUnit as required.
  //  NOTE: Comment out after initial setup and reload.

  //  Uncomment for 1 of the following Units
  //  Upload to Unit
  //  Comment out.  Save
  //  Upload to Unit
  
  /*
    TankNameLoc = "Header";
    LstTankName = TankNameLoc;
    SleepTime = 20; //  20mins
    SaveEEPROM();
  */

  /* TankNameLoc = "DBL Main";
    LstTankName = TankNameLoc;
    SleepTime = 0;
    SaveEEPROM();
  */

  /*TankNameLoc = "uDeck";
     LstTankName = TankNameLoc;
     SleepTime = 0;
     SaveEEPROM();
  */

  //  Then Comment out ALL and reload.

  // ******************************************************************

  GetEEPROM();  //  using the ESP32 #include <Preferences.h>


  if (Serial_ON)
    {
      Serial.print("This Sender Unit is named =");
      Serial.println(TankNameLoc);
    }


  if (TankNameLoc != "Header")
    {
      MessageMode = "Awake";
    }


  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin

  LoRa.setTxPower(17);          //  default == 17  max 20
  LoRa.setSpreadingFactor(12);  // was 12 defaults to 7 - values range 6 and 12

  //Serial.println(version)

  if (!LoRa.begin(915E6))
    {  // initialize ratio at 915 MHz

      if (Serial_ON)
        {
          Serial.println("LoRa init failed. Check your connections.");
        }

      //  Locked Loop to show LoRa is not working.
      while (true)  // Flash RED LED quickly - Indefinately LOOP - stops the program from advancing
        {
          digitalWrite(RecRedLED, HIGH);
          delay(167);
          digitalWrite(RecRedLED, LOW);
          delay(166);
        }
    }
  else
    {
      if (Serial_ON)
        {
          Serial.println("LoRa init succeeded.");
        }
    }
}

void GetEEPROM()  //  using the ESP32 #include <Preferences.h>
{
  //Serial.println("void GetEEPROM()");

  preferences.begin("TankData", RO_MODE);
  delay(10);

  TrueData = preferences.getUInt("TrueData", 0);  // preferebce.getUInt("Var", Pointer)

  if (TrueData == 99)
    {
      TankNameLoc = preferences.getString("TankNameLoc", "");

      LstTankName = TankNameLoc;

      SleepTime = preferences.getUInt("SleepTime", 0);

      LstSleepTime = SleepTime;

      if (TankNameLoc == "Header")  //  Change TankNameLoc "NAME" as required
        {
          localAddress = 0xAA;
        }

      if (TankNameLoc == "DBL Main")  //  Change TankNameLoc "NAME" as required
        {
          localAddress = 0xBB;
        }

      if (TankNameLoc == "uDeck")  //  Change TankNameLoc "NAME" as required
        {
          localAddress = 0xCC;
        }

      if (Serial_ON)
        {
          Serial.println("Getting Data from EEPROM");
          Serial.print("TrueData :");
          Serial.println(TrueData);
          Serial.print("TankNameLoc >");
          Serial.println(TankNameLoc);
          Serial.print("Sleep Time (sec) :");
          Serial.println(SleepTime);
          Serial.print("localAddress :");
          Serial.println(localAddress, HEX);
        }
    }
  preferences.end();
}

void SaveEEPROM()  //  using the ESP32 #include <Preferences.h>
{
  if (Serial_ON)
    {
      //  Save All EEPROM
      Serial.println("Saving EEPROM");
      Serial.print("TrueData = ");
      Serial.println(TrueData);
      Serial.print("TankName = ");
      Serial.println(TankNameLoc);
      Serial.print("Sleep Time = ");
      Serial.println(SleepTime);
    }

  preferences.begin("TankData", RW_MODE);

  preferences.putUInt("TrueData", 99);

  preferences.putString("TankNameLoc", TankNameLoc);
  LstTankName = TankNameLoc;

  preferences.putUInt("SleepTime", SleepTime);

  // Close the Preferences
  preferences.end();

  delay(100);
}

void GetDistance()  //************************************
{
  //Serial.println("void GetDistance()");

  digitalWrite(SendGrnLED, HIGH);  //  Flash Green when reading Distance Sensor
  StatusGrnLedON = true;
  StatusGrnLed = millis();
  GrnLEDTime = 1000;

  LastDist = distance;  //  in metres

  for (int i = 0; i <= 9; i++)  //  get 10 readings and remove 2 lowest
    {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);

      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);

      //  Calculating the distance in cm
      TestDist[i] = duration / 29 / 2;  // converts duration to cm  (1/29 = 0.344) speed of sound cm/us

      // Clear the trigPin in case of echo in the Tank
      delay(50);
    }

  //  Sort the Array

  // int lt[6] = {35, 15, 80, 2, 40, 110};  //  Example only -  see above search for sort_desc
  // Number of items in the array

  //  Sorting Data Lowest to Highest using qsort() function
  int TestDist_length = sizeof(TestDist) / sizeof(TestDist[0]);
  // qsort - last parameter is a function pointer to the sort function
  qsort(TestDist, TestDist_length, sizeof(TestDist[0]), sort_desc);

  float AvgDist = 0;
  //  Get Average the Middle 6 Readings out of the 10 Sorted Readings
  for (int i = 2; i <= 7; i++)
    {
      AvgDist = AvgDist + TestDist[i];
    }

  LastDist = distance;  //  in metres

  distance = AvgDist / 6;

  //  distance is in cm so convert to metres
  distance = float(int(distance)) / 100;

  /* // All Calcs of Water Level is done in Master Unit, not here..
     //Didtance from sensor to Water Level is sent to Master.
    if (TankNameLoc == "Header")
    {
     distance = distance - 0.22;  // 150 plus error of 70mm -  Substract distance top of tank to highest possible waterlevel -
    }

    if (TankNameLoc == "DBL Main")
    {
     distance = distance - 0.15;  //  Substract distance top of tank to highest possible waterlevel
    }

    if (TankNameLoc == "uDeck")
    {
     distance = distance - 0.15;  //  Substract distance top of tank to highest possible waterlevel
    }*/

  if (Serial_ON_Head || Serial_ON)
    {
      Serial.print("Line 505 - LastDist = ");
      Serial.print(LastDist);
      Serial.print("   distance = ");
      Serial.print(distance);
      Serial.print("   Difference = ");
      Serial.println(abs(distance - LastDist));
    }

  if (abs(distance - LastDist) >= 0.01 && !BootUp)
    {
      if (Serial_ON_Head || Serial_ON)
        {
          Serial.println("Line 517 - Send Data to Master Unit");
        }
      SetUpMessage();  //  Send data if different
    }
}

void CheckVoltages()
{
  if (Serial_ON_Head || Serial_ON)
    {
      Serial.println("Line 529 - void CheckVoltages()");
      //  delay(50); // delay so voltage levels settle before readings
    }

  // Check for all volatage
  RawPowVolts = analogRead(PVpin);

  RawBatVolts = analogRead(BVpin);

  RawSolVolts = analogRead(SVpin);

  // Battery  Reading
  //   mymap(value(in), fromLow(in), fromHigh(in), toLow(out), toHigh(out))
  BatVolts = mymap(RawBatVolts, 2090, 3500, 2.9, 4.2);  //  was 3.9k, changed to 4.7k with 10k to Gnd

  //  Special Case - Run off 12v --> 47k To BVpin 10k to Grnd
  //  BatVolts = mymap(RawBatVolts, 535, 2891, 3.2, 14.0);

  //delay(10);// delay so voltage levels settle before readings

  SolVolts = mymap(RawSolVolts, 0, 3785, 0, 5.1);  //  v --> 6.8k To SVpin 10k to Grnd

  //delay(10);// delay so voltage levels settle before readings

  PowVolts = mymap(RawPowVolts, 2785, 4050, 2.6, 3.4);  //  v --> 1.0k To PVpin 10k to Grnd

  //  Check for Critical Low Battery Level
  CheckCriticalBatLev();

  if (Serial_ON_Head || Serial_ON)
    {
      Serial.print("PowVolts : ");
      Serial.print(PowVolts);

      Serial.print("   BatVolts Raw Data : ");
      Serial.print(RawBatVolts);

      Serial.print("   BatVolts : ");
      Serial.print(BatVolts);

      Serial.print("   Solar Raw Data : ");
      Serial.print(RawSolVolts);

      Serial.print("   SolVolts : ");
      Serial.println(SolVolts);
    }
}

void TemperatureSensor()  //  *************************************************************************************************
{
  if (millis() - LastTempSensorDelay > TempSensorDelay)  //  Avoid millis() overflow
    {
      //Serial.println("void TemperatureSensor()");

      //LastTemp = Temp;

      TempSensorDelay = 3000;  // == 2.5 sec  Reset Default Delay for Temperature Sensor
      LastTempSensorDelay = millis();

      Tvo = analogRead(TempPin);

      TempCalibrate = 0;  //  May not be needed

      //  TA0061 Temperature Sensor Module - Range -55c to +125c (No 10k Res required)
      //  float SenTemp = mymap(Tvo, 0, 4095, -55 , 125);  //  Normal
      float SenTemp = mymap(Tvo, -300, 5500, -40, 125);  //  calibrated 28 Oct 2024 - Tested between 6 >> 55
      Temp = SenTemp + TempCalibrate;

      if (Serial_ON_Head)
        {
          Serial.print("Tvo  == ");
          Serial.print(Tvo);
          Serial.print("  Sensor TempC == ");
          Serial.print(SenTemp, 1);
          Serial.print("   Calibration == ");
          Serial.print(TempCalibrate);
          Serial.print("   Temp == ");
          Serial.println(Temp, 1);
        }
    }
}

void SetUpMessage()  //  *****************************************************************************
{
  //  Must have a Delimiter at the END

  if (Serial_ON_Head)
    {
      Serial.println("Line 627 - void SetUpMessage()");
    }

  digitalWrite(RecRedLED, HIGH);  //  Red
  StatusRedLedON = true;
  StatusRedLed = millis();
  RedLEDTime = 1000;  //  Longer time for sending message


  //  Put in dummy Volts data for DBL Main & uDeck
  if (TankNameLoc != "Header")
    {
      Temp = 99;
      PowVolts = 99;
      BatVolts = 99;
      SolVolts = 99;
    }
  else
    {
      TemperatureSensor();  //  min every 5 secs min time;

      CheckVoltages();
    }

  if (Serial_ON_Head || Serial_ON)
    {
      Serial.print("Line 635 - SetUp Message - MessageMode == >");  //  Awake, Sleep or BootUp
      Serial.println(MessageMode);
    }

  //  Send Raw Voltage Values for calibration and checking only
  //             TankNameLoc    Water Level     Temp     Bat Volts   Solar Volts      SleepTime     Wake
  /*SendMessage = TankNameLoc + ": " + String(distance) + ": " + String(Tvo) + ": "
                + String(RawPowVolts,2) + ": " + String(RawBatVolts,2) + ": " + String(RawSolVolts) + ": "
                + String(SleepTime) + ":" + MessageMode + ":";
  */

  //  Send Voltages
  //             TankNameLoc    Water Level     Temp     Bat Volts   Solar Volts      SleepTime     Wake
  SendMessage = TankNameLoc + ": " + String(distance) + ": " + String(Temp, 1) + ": "
                + String(PowVolts, 2) + ": " + String(BatVolts, 2) + ": " + String(SolVolts, 2) + ": "
                + String(SleepTime) + ":" + MessageMode + ":";


  if (Serial_ON_Head || Serial_ON)
    {
      Serial.print("Line 655 - >>>>>>>>>>>>>>>>  SendMessage = ");
      Serial.println(SendMessage);
    }

  sendMessage(SendMessage);  //  Go to Void sendMessage()
}

void ExtractData(String RecNewMessage)  //  *******************************************************************
{
  //  Error here because I need to extract data from Master Unit i.e. SleepTime and mode
  incomingLength = RecNewMessage.length();

  int StPos = 0;
  int FinPos;
  int VarCnt = 1;  //  Part of message to seperate into Variables

  for (FinPos = 0; FinPos < incomingLength; FinPos++)
    {
      //Serial.print("Char > ");
      // Serial.println(RecNewMessage.substring(FinPos, FinPos + 1));

      if (RecNewMessage.substring(FinPos, FinPos + 1) == ":")
        {
          switch (VarCnt)
            {
              case 1:  // TankNameLoc
                TankNameLoc = RecNewMessage.substring(StPos + 1, FinPos);

                if (Serial_ON_Head || Serial_ON)
                  {
                    Serial.print("Line 685 - TankNameLoc > ");
                    Serial.println(TankNameLoc);
                  }

                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 2:  // MessageMode
                MessageMode = RecNewMessage.substring(StPos, FinPos);

                if (Serial_ON_Head || Serial_ON)
                  {
                    Serial.print(" LstMessageMode Awake, Sleep or Repeat = >");
                    Serial.println(LstMessageMode);
                    Serial.print("Rec MessageMode Awake, Sleep or Repeat = >");
                    Serial.println(MessageMode);
                  }
                LstMessageMode = MessageMode;


                StPos = FinPos + 1;
                VarCnt++;
                break;

              case 3:  // SleepTime
                SleepTime = RecNewMessage.substring(StPos, FinPos).toInt();

                if (Serial_ON_Head || Serial_ON)
                  {
                    Serial.print("SleepTime = ");
                    Serial.println(SleepTime);
                  }

                StPos = FinPos + 1;
                VarCnt++;
                break;
            }
        }
    }
}

void onReceive(int packetSize)  //  **************************************************************************************************
{
  if (packetSize != 0)
    {
      digitalWrite(SendGrnLED, HIGH);  //  Flash Green On Receive
      StatusGrnLedON = true;
      StatusGrnLed = millis();
      GrnLEDTime = 500;

      digitalWrite(RecRedLED, HIGH);  //  Flash Red On Receive
      StatusRedLedON = true;
      StatusRedLed = millis();
      RedLEDTime = 500;  //  Shorter Flash for Receiving a Message

      LoRaRSSI = String(LoRa.packetRssi());

      if (Just_RSSI)
        {
          Serial.print("  RSSI > ");
          Serial.println(LoRaRSSI);
        }

      // read packet header bytes:
      recipient = LoRa.read();  // recipient address

      sender = LoRa.read();  // sender address

      if (Serial_ON)
        {
          Serial.println(">>>>> >>>>> Received Request for Data  <<<<< <<<<<");
          Serial.print("Line 757 - Rec packetSize > ");
          Serial.println(packetSize);
          Serial.print("recipient > ");
          Serial.println(recipient, HEX);
          Serial.print("sender > ");
          Serial.println(sender, HEX);
        }

      String incoming = "";  //  clear var

      while (LoRa.available())
        {
          incoming = incoming + (char)LoRa.read();
        }

      RecMessage = incoming;

      int incomingLength = RecMessage.length();

      if (incomingLength != incoming.length())  // check length for error
        {
          if (Serial_ON)
            {
              Serial.println("error: RecMessage length does not match length");
            }
          return;  // skip rest of function
        }

      //if the recipient isn't this device or broadcast,
      if (recipient != localAddress || sender != 0xFF)
        {
          if (Serial_ON)
            {
              Serial.print("Line 790 - This RecMessage is not for me, but for > ");
              Serial.println(recipient, HEX);
              Serial.print("RecMessage >");
              Serial.print(RecMessage);
              Serial.print("  LoRaRSSI >");
              Serial.println(LoRaRSSI);
            }
          return;  // skip rest of function
        }

      // 0xAA or 0xBB or 0xCC
      if (recipient == localAddress && sender == 0xFF)  //  Header, or DBL or uDeck
        {
          if (Serial_ON_Head || Serial_ON)
            {
              Serial.println("####vvvv####vvvv####vvvv####vvvv####vvvv####vvvv####");
              Serial.print("Line 806 - This RecMessage IS for me, ");
              Serial.print(localAddress, HEX);  //  HEX BB == 187  HEX CC == 204
              Serial.print(" from > ");
              Serial.println(sender, HEX);
              Serial.print(" RecMessage > ");
              Serial.println(RecMessage);
            }

          ExtractData(RecMessage);

          if (LstSleepTime != SleepTime)
            {
              SaveEEPROM();
              LstSleepTime = SleepTime;
            }

          if (localAddress == 0xAA)  //  HEX AA == Header
            {
              NoResponseCnt = 0;  //  Clears No Response if Message Received for Header Send Unit
            }
          else  //  Message for BDL or uDeck
            {
              if (Serial_ON_Head)
                {
                  Serial.println("Line 824 - Get Distance Send Message from ");
                  Serial.println(TankNameLoc);
                }

              GetDistance();

              SetUpMessage();
            }

          if (Serial_ON_Head)
            {
              Serial.print("Line 835 - MessageMode = >");
              Serial.println(MessageMode);
            }

          packetSize = 0;  //  clear packetsize

          if (MessageMode == "Sleep")
            {
              //Serial.print("MessageMode = >");
              //Serial.println(MessageMode);

              GoToSleep();  //  Go check if Sleep Mode is True
            }
        }
    }
}

void sendMessage(String sendMessage)  //  **************************************************************************************************
{
  //Serial.println("void sendMessage(String SndMessage)");

  if (Serial_ON_Head || Serial_ON)
    {
      Serial.println("Line 858 - Sending Tank Name >" + TankNameLoc);
      Serial.println("##^^##^^##^^##^^##^^##^^##^^##^^##^^##^^##^^##^^##");
    }

  LoRa.beginPacket();  // start packet

  LoRa.write(destination);           // add destination address
  LoRa.write(localAddress);          // add sender address
  LoRa.write(sendMessage.length());  // add payload  length
  LoRa.print(sendMessage);           // add payload

  LoRa.endPacket();  // finish packet and send it
}

void GoToSleep()  //  **************************************************************************************************
{
  if (Serial_ON_Head || Serial_ON)
    {
      Serial.println("void GoToSleep()");
    }

  digitalWrite(RecRedLED, HIGH);

  if (Serial_ON)
    {
      Serial.print("Going to Sleep...");
    }

  if (CriticalBatLev)  //  Battery Critically Low from Line 935
    {
      SleepTime = 60;  //  BatVolts < 3.3
    }
  //   SleepMicroTime =  300000000; // 10,000,000 == 10 sec   300000000 == 5 mims   10800000000 == 3 hrs
  //   SleepTime == Minutes..  20mins   1000000LL  instead of 1ULL

  SleepMicroTime = 1000000LL * SleepTime * 60;  // 1UUL == Sets up Result Number for UUL == unsigned long long
  //  The Break down >> 1ULL * 60 (mins) * 60 (secs) * 1000 (millisec) * 1000 (microsec)

  if (SleepMicroTime > 14400000000)  //  max 240 minutes 4 Hrs
    {
      SleepMicroTime = 14400000000;
    }

  if (Serial_ON_Head || Serial_ON)
    {
      Serial.print("Going to sleep now for ");
      Serial.print(SleepMicroTime / 1000000 / 60);
      Serial.println(" mins..  zzzZZZZzzz..");

      Serial.flush();
    }

  delay(1000);  //  Just so I can see the Red Led light up for a Second

  esp_sleep_enable_timer_wakeup(SleepMicroTime);  //  sleep for amount of Time in microseconds

  esp_deep_sleep_start();  // Goes to sleep here...
}

void CheckCriticalBatLev()
{
  // Check Battery Level
  if (BatVolts < 3.3)  //  reading is 3.3v
    {
      CriticalBatLev = true;
      CriticalCnt++;

      if (Serial_ON_Head || Serial_ON)
        {
          Serial.print(" CriticalCnt: ");
          Serial.println(CriticalCnt);

          Serial.print("BatVolts : ");
          Serial.print(BatVolts);
          Serial.println("  Line 935 **** Critically Low - Shutting Down Soon");
        }

      if (CriticalCnt >= 3)
        {
          if (Serial_ON_Head || Serial_ON)
            {
              Serial.print(" Going to Sleep for 1 hour");
              Serial.print("  CriticalCnt >= 3 ; ");
              Serial.println(CriticalCnt);
            }

          MessageMode = "BVlow";  //  Battery Voltage Low  BVlow

          //GetDistance();

          // SetUpMessage();

          SleepTime = 60;  //  Set Sleep time for 1 Hour or 60 Minutes

          GoToSleep();
        }
    }
  else
    {
      CriticalBatLev = false;
      CriticalCnt = 0;
      //MessageMode = "Awake";
    }
}

void loop()  //  *********************************************************************
{
  //  If Testing Flashes Green LED every 2 secs
  if (Testing && millis() - TestingGrnLed > 2000)
    {
      digitalWrite(SendGrnLED, HIGH);  //  Green
      StatusGrnLedON = true;
      StatusGrnLed = millis();
      TestingGrnLed = millis();
    }

  //  Flash Green Led repeatedly to show Unit Alive
  if (millis() - StatusAliveGrn > 4000)
    {
      digitalWrite(SendGrnLED, HIGH);  //  Green
      StatusGrnLedON = true;
      StatusAliveGrn = millis();
      StatusGrnLed = millis();
    }

  //  Truns Off Green Led after GrnLEDTime - Flashes when doing a Distance reading, Received Signal or Testing..
  if (StatusGrnLedON && millis() - StatusGrnLed > 20)
    {
      digitalWrite(SendGrnLED, LOW);  //  Green
      StatusGrnLedON = false;
      StatusGrnLed = millis();
    }

  //  Truns Off Red Led after RedLEDTime (1000ms) - Flashes when Signal Received with Green and Seperate Red when Sending
  if (StatusRedLedON && millis() - StatusRedLed > RedLEDTime)
    {
      digitalWrite(RecRedLED, LOW);  //  Red
      StatusRedLedON = false;
      StatusRedLed = millis();
    }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  //  Check the Water Level every 60 sec == 10000  (in Awake Mode)
  if (millis() - CheckLevelTime > 60000 || BootUp)  //  check Level every 60 sec in Awake Mode * OR * NOW if BootUp..
    {
      CheckLevelTime = millis();

      if (BootUp)  //  At start up or when Message not acknowledged
        {
          if (Serial_ON_Head || Serial_ON)
            {
              Serial.print(">>>>>>>> BootUp - Get Tank Level and Send Data Now");
              Serial.println(">>>>>>>> BootUp - Get Tank Level and Send Data Now");
            }
          GetDistance();

          SetUpMessage();

          BootUp = false;  //  Only used at Boot UP
        }
      else
        {
          if (Serial_ON_Head || Serial_ON)
            {
              Serial.println(">>>>>>>> Get Tank Level and Send Data if different");
            }

          GetDistance();  //  only send data every 20 minutes or if changed..

          //  Fix GF01
          if (MessageMode != "Awake")  //  avoids Unit stuck in unknown loop
            {
              SetUpMessage();  //  AND send Message
            }
        }
    }

  if (MessageMode == "Sleep")
    {
      //Serial.print("MessageMode = >");
      //Serial.println(MessageMode);

      GoToSleep();  //  Go check if Sleep Mode is True
    }

  if (TankNameLoc == "Header" && !BootUp)
    {
      //  Because Header sends MessageMode == BootUp, if not changed then No Response from Master
      //  If "BootUp" from Boot Up and Master Down (Off or No WiFi) - Check for Response..
      if (MessageMode == "BootUp")  //  This is sent and if no reply from Master will remain BootUp
        {
          int NoResponseCntDelaySec = 5000;                           //  5 sec
          if (millis() - NoResponseCntDelay > NoResponseCntDelaySec)  //  Avoid millis() overflow
            {
              NoResponseCntDelay = millis();

              NoResponseCnt++;

              if (Serial_ON)
                {
                  Serial.print("MessageMode == ");
                  Serial.print(MessageMode);
                  Serial.print("   And No Response from Master Unit - Times:");
                  Serial.println(NoResponseCnt);
                }
              GetDistance();

              SetUpMessage();  //  For BootUp only
            }

          if (NoResponseCnt > 10 && !Testing)  //  shuts down after 10 attemps response..
            {

              if (Serial_ON_Head || Serial_ON)
                {
                  Serial.println("  No Response from Master Unit - Header Unit Shutting Down for 20 minutes");
                }

              MessageMode = "Sleep";
              // SleepTime = 20;  //  20 mins
              GoToSleep();
            }
        }
    }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}
