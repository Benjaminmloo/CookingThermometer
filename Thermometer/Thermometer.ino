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
#include <AdafruitIO_WiFi.h>
#include "AdafruitIO_WiFi.h"

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(A5);
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

const char* WIFI_SSID = "A380";
const char* WIFI_PASS = "5198236565";

// Adafruit IO
#define AIO_USERNAME    "lloo"
#define AIO_KEY         "4f4b4bc8eeb04d3bb376497471c6e0ec"

#define pinButtonSet    15  //left_b
#define pinButtonDec    32  //cntr_b
#define pinButtonInc    14  //rtgh_b

#define pinBatLvl     A13 //battery voltage reading pin 

#define SET_NONE 0
#define SET_TEMP 1
#define SET_RPRT 2

#define MAX_A_VAL     4095
#define LOGIC_HIGH_V  3.3

#define NUM_SETTINGS 3
#define CYCLES_TO_UPLD 20
#define NUM_SAMPLES 10

#define CHAR_W 6
#define CHAR_H 8

AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, WIFI_SSID, WIFI_PASS);

AdafruitIO_Feed *currentTemp =        io.feed("currentTemp");
AdafruitIO_Feed *temperatureReached = io.feed("temperatureReached");

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

unsigned long lastTime;
unsigned long currTime;

float lastTemp;
float currTemp;

float tempData[NUM_SAMPLES];

bool even = true;


// The value of the Rref resistor. Use 430.0!
#define RREF 430.0


void setup() {
  Serial.begin(115200);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  oled.display();
  delay(500);

  // Clear the display buffer.
  oled.clearDisplay();
  //Send the display buffer
  oled.display();

  //set text parameters
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  max.begin(MAX31865_3WIRE);
  oled.println("Sensor PASS");
  oled.display();
  
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  oled.println("IO ... ");
  oled.display();
  
  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  oled.print(io.statusText());
  oled.display();
  delay(200);
  
  Serial.println();
  Serial.println(io.statusText());

  
  //Initialise parameters
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

  pinMode(pinButtonInc, INPUT_PULLUP);
  pinMode(pinButtonSet, INPUT_PULLUP);
  pinMode(pinButtonDec, INPUT_PULLUP);

  //initialize data sent to aio
  io.run();
  
  temperatureReached->save(tempReached);
  currentTemp->save(max.temperature(100, RREF));
}


/****************************************************
 * PRINT METHODS
 * 
 * each method sets absolute cursor position then
 * print data from starting point
 ****************************************************/
 
void printTemp(float temp){
  oled.setCursor(0, 0);
  
  oled.print("TEMP: ");
  oled.print(temp);
}

void printRun(bool flash){
  oled.setCursor(CHAR_W * 15, CHAR_H * 1);

  //when toggeling alarm status display with the control signal
  //Otherwise just display the status
  if ((flash && setSelect == SET_RPRT) || setSelect != SET_RPRT)
  {
    if (alarmOn)
      oled.print("    ON");
    else
      oled.print("   OFF");
  }
}

void printDone(){
  oled.setCursor(CHAR_W * 17, CHAR_H * 3);
  oled.print("DONE");
}

void printSet(bool flash){
  oled.setCursor(0, CHAR_H * 1);
  oled.print("SETT: ");

  if ((flash && setSelect == SET_TEMP) || setSelect != SET_TEMP)
  {
    oled.print(setTemp);
  }
}

void printBat(float batt){
  oled.setCursor(CHAR_W * 16, 0);
  oled.print(batt);
  oled.print("v");
}

void printTimeEstimate(float timeToDone){
  oled.setCursor(0, CHAR_H * 3);
  oled.print(timeToDone);
  oled.print("min");
}

/****************************************************
 * MAIN LOOP
 ****************************************************/

void loop() {
  int batraw;
  unsigned long period;
  float battery;
  float tempDiff;
  float tempVelo; //temp change per minute
  float timeToFinish;
  
  //Ping adafruit IO
  io.run();
    
  //read new sensor values
  currTemp = max.temperature(100, RREF);

  batraw = analogRead(pinBatLvl);
  //divide by max value/multiply by max possible voltage/double to account for voltage divide circuit  
  battery = (float)(batraw * LOGIC_HIGH_V * 2) / MAX_A_VAL;

  //if the temerature difference between samples is greater than 0.1 degC
  //update estimate
  
  if(abs(currTemp - lastTemp) > 0.1)
  {
    //measure time
    //calculate period, temp delta, velocity of the temperature(converting to degC/min)
    currTime = micros();
    period = currTime - lastTime;
    tempDiff = currTemp - lastTemp;
    
    tempVelo = tempDiff / period;
    tempVelo *= 6e7;


    timeToFinish = (float)(setTemp - currTemp) / tempVelo;

    lastTime = currTime;
    lastTemp = currTemp;
  }
  oled.clearDisplay();

  //store previous button state
  lastButtonA = buttonA;
  lastButtonB = buttonB;
  lastButtonC = buttonC;
  
  //read buttons
  buttonA = digitalRead(pinButtonInc);
  buttonB = digitalRead(pinButtonSet);
  buttonC = digitalRead(pinButtonDec);

  //HANDLE BUTTONS
  //when button changes resulting from high to low (button press)
  //activate
  if (buttonA != lastButtonA)
  {
    if (buttonA == LOW)
    {
      //oled.print("A");
      if (setSelect == SET_TEMP && setTemp < 500)
      {
        setTemp += 5; //incriment temp by 5 when setting that
      } else if (setSelect == SET_RPRT)
      {
        alarmOn = !alarmOn; //invert alarm when setting that
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

  //HANDLE TEMPERATURE

  //same previous temp to detect edges
  tempReachedLast = tempReached;
  
  if (currTemp >= setTemp)
  {
    tempReached = 1;
  } else
  {
    tempReached = 0;
  }

  //when the alarm is active 
  //report every 20 cycles (cycles take just under a second so reports every 20s)
  //20 second period of reports to stay under adafruit IOs limit

  if(alarmOn){
    if(cycleNum == 0)
      currentTemp->save(currTemp);

    //when the timer reaches the value display done, flash the value off every third cycle
    if(tempReached && setSelect == 0 && cycleNum % 3 != 0)
      printDone();

    //on any edge in temp reached, send new value
    if (tempReachedLast != tempReached)
      temperatureReached->save(tempReached);
  }

  //print the rest of the data to the screen buffer
  printTemp(currTemp);
  
  printRun(even);
  
  printSet(even);

  printBat(battery);

  printTimeEstimate(timeToFinish);
  
  oled.display();

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


  //PREP NEXT CYCLE
  even = !even;
  cycleNum ++;
  cycleNum %= CYCLES_TO_UPLD;
}
