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
#define AIO_USERNAME    "lloo"
#define AIO_KEY         "4f4b4bc8eeb04d3bb376497471c6e0ec"

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *currentTemp = io.feed("currentTemp");
AdafruitIO_Feed *temperatureReached = io.feed("temperatureReached");

#define SET_NONE 0
#define SET_TEMP 1
#define SET_RPRT 2

const int NUM_SETTINGS = 3;

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
int alarmOn;

int cycleNum;
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
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());

  oled.clearDisplay();
  oled.init();

  setTemp = 70;
  alarmOn = 0;
  setSelect = 0;
  setSelect = 0;
  cycleNum = 0;
  tempReached = 0; 

  buttonA = -1;
  buttonB = -1;
  buttonC = -1;

  lastButtonA = -1;
  lastButtonB = -1;
  lastButtonC = -1;

  pinMode(pinButtonA, INPUT_PULLUP);
  pinMode(pinButtonB, INPUT_PULLUP);
  pinMode(pinButtonC, INPUT_PULLUP);
  
  io.run();
  
  temperatureReached->save(tempReached);
  currentTemp->save(max.temperature(100, RREF));
}


void loop() {
  io.run();
  uint16_t rtd = max.readRTD();

  oled.clearDisplay();
  oled.setCursor(0, 0);

  buttonA = digitalRead(pinButtonA);
  buttonB = digitalRead(pinButtonB);
  buttonC = digitalRead(pinButtonC);

  if (buttonA != lastButtonA)
  {
    if (buttonA == LOW)
    {
      //oled.print("A");
      if (setSelect == SET_TEMP && setTemp < 500)
      {
        setTemp += 5;
      } else if (setSelect == SET_RPRT)
      {
        alarmOn = !alarmOn;
      }
    }
  }

  if (buttonB != lastButtonB)
  {
    if (buttonB == LOW)
    {
      //oled.print("B");
      setSelect ++;
      if (setSelect >= NUM_SETTINGS)
      {
        setSelect = 0;
      }
    }
  }

  if (buttonC != lastButtonC)
  {
    if (buttonC == LOW)
    {
      //oled.print("C");
      if (setSelect == SET_TEMP && setTemp > 0)
      {
        setTemp -= 5;
      } else if (setSelect == SET_RPRT)
      {
        alarmOn = !alarmOn;
      }
    }
  }

  lastButtonA = buttonA;
  lastButtonB = buttonB;
  lastButtonC = buttonC;

  oled.print("TEMP: ");
  oled.print(max.temperature(100, RREF));
  if ((even && setSelect == SET_RPRT) || setSelect != SET_RPRT)
  {
    if (alarmOn)
    {
      if (max.temperature(100, RREF) >= 100)
        oled.print("   ON");
      else if (max.temperature(100, RREF) >= 10)
        oled.print("    ON");
      else
        oled.print("     ON");
        
      if(cycleNum == 0)
      {
        currentTemp->save(max.temperature(100, RREF));
      }

      if (max.temperature(100, RREF) >= setTemp)
      {
        tempReached = 1;
        oled.println(" DNE");
      } else
      {
        tempReached = 0;
        oled.println("");
      }

      if (tempReachedLast != tempReached)
      {
        temperatureReached->save(tempReached);
      }

      tempReachedLast = tempReached;
    } else
    {
      if (max.temperature(100, RREF) >= 100)
        oled.println("  OFF");
      else if (max.temperature(100, RREF) >= 10)
        oled.println("   OFF");
      else
        oled.println("    OFF");
    }
  } else if (!even && setSelect == SET_RPRT)
  {
    oled.println("");
  }

  oled.print("SETT: ");
  if ((setSelect == SET_TEMP && even) || setSelect != SET_TEMP)
  {
    oled.print(setTemp);
  } else
  {
    oled.print("          ");
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
  cycleNum ++;
  cycleNum %= 20;
  delay(10);
}
