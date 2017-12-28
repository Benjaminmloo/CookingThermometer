/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <AdafruitIO_WiFi.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(A5);
Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

const char* WIFI_SSID     = "A380";
const char* WIFI_PASS = "5198236565";
 
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "BML"
#define AIO_KEY         "516088c495ec425d87e264b0b1518bd0"

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *currentTemp = io.feed("currentTemp");
AdafruitIO_Feed *temperatureReached = io.feed("temperatureReached");

#define MENU_DEF 0
#define MENU_SET 1

#define MEAT_BVL 10
#define MEAT_PORK 11
#define MEAT_POULTRY 12
#define MEAT_OTHER 13
#define MEAT_CUSTOM 14

const int numMeats = 5;
const int numCooks[] = {4, 1, 2, 1, 1};

const String meatName[] = {"BVL    ", "PORK   ", "POULTRY", "OTHER  ", "CUSTOM "};
const String meatCook[][4] = {{"MR ", "MED", "WD ", "GND"}, {"ANY"}, {"PCS", "WHL"}, {"N/A"}, {"N/A"}};
int meatTemp[][4] = {{63, 71, 77, 71}, {71}, {74, 82}, {74}, {100}};

int currentMeat;
int currentCook;

const int pinButtonA = 15;
const int pinButtonB = 32;
const int pinButtonC = 14;

int buttonA;
int buttonB;
int buttonC;

int lastButtonA;
int lastButtonB;
int lastButtonC;

int upButton;
int dnButton;

int setTemp;
int tempReached;
int tempReachedLast;
int curScreen;
int alarmOn;

int setSelect;
boolean even  = true;


// The value of the Rref resistor. Use 430.0!
#define RREF 430.0


void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  Serial.print("Connecting to Adafruit IO");
  io.connect();

  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());
  
  oled.clearDisplay();
  oled.init();

  setTemp = 0;
  curScreen = 0;
  alarmOn = 0;

  buttonA = -1;
  buttonB = -1;
  buttonC = -1;

  currentMeat = 0;
  currentCook = 0;
  
  lastButtonA = -1;
  lastButtonB = -1;
  lastButtonC = -1;

  pinMode(pinButtonA, INPUT_PULLUP);
  pinMode(pinButtonB, INPUT_PULLUP);
  pinMode(pinButtonC, INPUT_PULLUP);
}


void loop() {
  io.run();
  uint16_t rtd = max.readRTD();

  oled.clearDisplay();
  oled.setCursor(0,0);

  buttonA = digitalRead(pinButtonA);
  buttonB = digitalRead(pinButtonB);
  buttonC = digitalRead(pinButtonC);

  if (buttonA != lastButtonA) 
  {
    if (buttonA == LOW) 
    {
      oled.print("A");
      if(curScreen == MENU_DEF)
      {
        alarmOn = !alarmOn;
      }else if(curScreen == MENU_SET)
      {
        upButton = true;
      } else
      {
        upButton = false;
      }
    }else
    {
      upButton = false;
    }
  }
  
  if (buttonB != lastButtonB) 
  {
    if (buttonB == LOW) 
    {
      oled.print("B");
      if(curScreen == MENU_DEF)
      {
        curScreen = MENU_SET;
        even = true;
      }
    } 
  }else
  {
    if (buttonB == LOW) 
    {
       curScreen = MENU_DEF;
    }
  }
  
  if (buttonC != lastButtonC) 
  {
    if (buttonC == LOW) 
    {
      oled.print("C");
      if(curScreen == MENU_SET)
      {
        dnButton = true;
      } else
      {
        dnButton = false;
      }
    }else
    {
      dnButton = false; 
    }
  }

  lastButtonA = buttonA;
  lastButtonB = buttonB; 
  lastButtonC = buttonC;
  
 
  
  switch(curScreen)
  {
    case MENU_DEF:
      oled.print("TEMP: ");
      oled.print(max.temperature(100, RREF));
      if(alarmOn)
      {
        if(max.temperature(100, RREF) >= 100)
          oled.print("   ON");
        else if(max.temperature(100, RREF) >= 10)
          oled.print("    ON");
        else
          oled.print("     ON");

        currentTemp->save(max.temperature(100, RREF));

        if(max.temperature(100, RREF) >= meatTemp[currentMeat][currentCook])
        {
          tempReached = 1;
          oled.println(" DNE");
        }else
        {
          tempReached = 0;
          oled.println("");
        }

        if(tempReachedLast != tempReached)
        {
          temperatureReached->save(tempReached);
        }
      }else
      {
        if(max.temperature(100, RREF) >= 100)
          oled.println("  OFF");
        else if(max.temperature(100, RREF) >= 10)
          oled.println("   OFF");
        else
          oled.println("    OFF");
      }
      oled.print("MEAT: ");
      oled.print(meatName[currentMeat]);
      oled.print("COOK:");
      oled.print(meatCook[currentMeat][currentCook]);
      oled.print("SETT: ");
      oled.print(meatTemp[currentMeat][currentCook]);
      break;
    case MENU_SET:
      if(even)
      {
        oled.print("MEAT: ");
        oled.print(meatName[currentMeat]);
        oled.print("COOK:");
        oled.print(meatCook[currentMeat][currentCook]);
        oled.print("SETT: ");
        oled.print(meatTemp[currentMeat][currentCook]);
      }else
      {
        if(setSelect)
        {
          oled.print("MEAT: ");
          oled.print("       ");
          oled.print("COOK:");
          oled.print(meatCook[currentMeat][currentCook]);
          oled.print("SETT: ");
          oled.print(meatTemp[currentMeat][currentCook]);
        }else
        {
          oled.print("MEAT: ");
          oled.print(meatName[currentMeat]);
          oled.print("COOK:");
          oled.print("   ");
          oled.print("SETT: ");
          oled.print(meatTemp[currentMeat][currentCook]);
        }
      }
      if(upButton)
      {
        setSelect = !setSelect;
      }

      if(dnButton)
      {
        if(setSelect)
        {
          currentMeat ++;
          if(currentMeat >= numMeats)
            currentMeat = 0;
          currentCook = 0;
        }else
        {
          currentCook ++;
          if(currentCook >= numCooks[currentMeat])
            currentCook = 0;
        }
      }
      
      break;
  }
  
  oled.display();

//  Serial.print("RTD value: "); Serial.println(rtd);
//  float ratio = rtd;
//  ratio /= 32768;
//  Serial.print("Ratio = "); Serial.println(ratio,8);
//  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
//  Serial.print("Temperature = "); Serial.println(max.temperature(100, RREF));

  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();
  }
  Serial.println();

  even = !even;
  delay(1000);
}
