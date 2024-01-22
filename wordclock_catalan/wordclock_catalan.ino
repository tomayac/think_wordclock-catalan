/*
 * Sourcecode for wordclock (german)
 *
 * created by techniccontroller 06.12.2020
 *
 * changelog:
 * 03.04.2021: add DCF77 signal quality check
 * 04.04.2021: add update intervall for RTC update
 * 18.10.2021: add nightmode
 * 17.04.2022: fix nightmode condition, clean serial output, memory reduction
 * 13.05.2022: add PIR sensor to interrupt nightmode while PIR_PIN == HIGH (FEATURE-REQUEST)
 */
#include "RTClib.h"             //https://github.com/adafruit/RTClib
#include "DCF77.h"              //https://github.com/thijse/Arduino-DCF77
#include <TimeLib.h>            //https://github.com/PaulStoffregen/Time
#include <Adafruit_GFX.h>       //https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_NeoMatrix.h> //https://github.com/adafruit/Adafruit_NeoMatrix
#include <EEPROM.h>

// definition of pins
#define DCF_PIN 2               // Connection pin to DCF 77 device
#define NEOPIXEL_PIN 6          // Connection pin to Neopixel LED strip
#define SENSOR_PIN A6           // analog input pin for light sensor
#define PIR_PIN 4               // Connection pin to PIR device (HC-SR501, Jumper on H = Repeatable Trigger)

// char array to save time an date as string
char time_s[9];
char date_s[11];

// define parameters for the project
#define WIDTH 11                // width of LED matirx
#define HEIGHT (10+1)           // height of LED matrix + additional row for minute leds
#define EE_ADDRESS_TIME 10      // eeprom address for time value (persist values during power off)
#define EE_ADDRESS_COLOR 20     // eeprom address for color value (persist values during power off)
#define UPPER_LIGHT_THRSH 930   // upper threshold for lightsensor (above this value brightness is always 20%)
#define LOWER_LIGHT_THRSH 800   // lower threshold for lightsensor (below this value brightness is always 100%)
#define CENTER_ROW (HEIGHT/2)   // id of center row
#define CENTER_COL (WIDTH/2)    // id of center column
#define NIGHTMODE_START 0      // start hour of nightmode (0 <=> 00:00)
#define NIGHTMODE_END 7         // end hour of nightmode (7 <=> 7:00)

// create DCF77 object
DCF77 DCF = DCF77(DCF_PIN,digitalPinToInterrupt(DCF_PIN));

// create RTC object
RTC_DS3231 rtc;

// define mapping array for nicer printouts
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// create Adafruit_NeoMatrix object
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(WIDTH, HEIGHT, NEOPIXEL_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS    + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

// define color modes
const uint16_t colors[] = {
  matrix.Color(255, 255, 255),  // white (in this mode together with colorwheel)
  matrix.Color(255, 0, 0),      // red
  matrix.Color(255, 255, 0),    // yellow
  matrix.Color(255, 0, 200),    // magenta
  matrix.Color(128, 128, 128),  // white (darker)
  matrix.Color(0, 255, 0),      // green
  matrix.Color(0, 0, 255) };    // blue

// definition of global variables
int valLight = 100;             // currentvalue of light sensor
uint8_t brightness = 20;        // current brughtness for leds
uint8_t activeColorID = 0;      // current active color mode
int offset = 0;                 // offset for colorwheel
long updateIntervall = 120000;  // Updateintervall 2 Minuten
long updateTimer = 0;           // Zwischenspeicher für die Wartezeit

// representation of matrix as 2D array
uint8_t grid[HEIGHT][WIDTH] = {{0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0}};

// function prototypes
void timeToArray(uint8_t hours, uint8_t minutes);
void printDateTime(DateTime datetime);
void checkDCFSignal();
int DCF77signalQuality(int pulses);
void checkForNewDCFTime(DateTime now);
void gridAddPixel(uint8_t x, uint8_t y);
void gridFlush(void);
void drawOnMatrix(void);
void timeToArray(uint8_t hours, uint8_t minutes);
void printHours(uint8_t hours, uint8_t minutes);

void setup() {
  // enable serial output
  Serial.begin(9600);

  // Measure DCF signal quality
  // checkDCFSignal();

  // Init DCF
  DCF.Start();

  // Init RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  // get active color from last power cycle
  EEPROM.get(EE_ADDRESS_COLOR, activeColorID);

  // check if in the last 3 seconds was a power cycle
  // if yes, so change to next color mode
  long laststartseconds = 0;
  EEPROM.get(EE_ADDRESS_TIME, laststartseconds);
  long currentseconds = rtc.now().secondstime();
  Serial.print("Startseconds: ");
  Serial.println(laststartseconds);
  Serial.print("Currentseconds: ");
  Serial.println(currentseconds);
  if(currentseconds - laststartseconds < 10){
    activeColorID = (activeColorID+1)%7;
    Serial.print("change color to ");
    Serial.println(activeColorID);
    EEPROM.put(EE_ADDRESS_COLOR, activeColorID);
  }
  EEPROM.put(EE_ADDRESS_TIME, currentseconds);
  Serial.print("active color: ");
  Serial.println(activeColorID);

  Serial.println("Buildtime: ");
  Serial.println(F(__DATE__));
  Serial.println(F(__TIME__));

  // check if RTC battery was changed
  if (rtc.lostPower() || DateTime(F(__DATE__), F(__TIME__)) > rtc.now()) {
    Serial.println("RTC lost power or RTC time is behind build time, let's set the time!");
    printDateTime(rtc.now());
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Init LED matrix
  matrix.begin();
  matrix.setBrightness(100);

  // quick matrix test
  for(int i=1; i<(WIDTH*HEIGHT); i++){
    matrix.drawPixel((i-1)%WIDTH, (i-1)/HEIGHT, matrix.Color(0,0,0));
    matrix.drawPixel(i%WIDTH, i/HEIGHT, matrix.Color(120,150,150));
    matrix.show();
    delay(10);
  }
}

void loop() {
  // get light value from light sensor
  valLight = analogRead(SENSOR_PIN);

  // calc led brightness from light value
  if (valLight < LOWER_LIGHT_THRSH){
    brightness = 100;
  }
  else if (valLight > UPPER_LIGHT_THRSH){
    brightness = 20;
  }
  else{
    brightness = 80-((valLight-LOWER_LIGHT_THRSH)*1.0/(UPPER_LIGHT_THRSH-LOWER_LIGHT_THRSH))*80 + 20;
  }

  Serial.print("Light: ");
  Serial.println(valLight);
  Serial.print("Brightness: ");
  Serial.println(brightness);
  matrix.setBrightness(brightness*2);


  // Print current date and time
  DateTime rtctime = rtc.now();
  Serial.print("RTC: ");
  printDateTime(rtctime);

  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");

  // check for a new time on the DCF receiver
  checkForNewDCFTime(rtctime);

  // convert time to a grid representation
  timeToArray(rtctime.hour(), rtctime.minute());
  // clear matrix
  matrix.fillScreen(matrix.Color(0, 0, 0));


  // add condition if nightmode (LEDs = OFF) should be activated
  // turn off LEDs between NIGHTMODE_START and NIGHTMODE_END
  uint8_t nightmode = false;
  uint8_t motionPIR = digitalRead(PIR_PIN);
  if(NIGHTMODE_START > NIGHTMODE_END && (rtctime.hour() >= NIGHTMODE_START || rtctime.hour() < NIGHTMODE_END) && !(motionPIR == HIGH)){
    // nightmode duration over night (e.g. 22:00 -> 6:00)
    nightmode = true;
  }
  else if(NIGHTMODE_START < NIGHTMODE_END && (rtctime.hour() >= NIGHTMODE_START && rtctime.hour() < NIGHTMODE_END) && !(motionPIR == HIGH)){
    // nightmode duration during day (e.g. 18:00 -> 23:00)
    nightmode = true;
  }

  if(!nightmode){
    // nightmode is not active -> draw on Matrix, as normal

    // if color mode is set to 0, a color wheel will be shown in background
    if(activeColorID == 0){
      // draw colorwheel on matrix
      drawCircleOnMatrix(offset);
      offset = (offset + 1)%256;
    }

    // draw grid reprentation of time to matrix
    drawOnMatrix();
  }
  else{
    Serial.println("Nightmode is active -> LED are OFF");
  }

  // send the commands to the LEDs
  matrix.show();

  // change depending on color mode the refreshing time of clock
  if(activeColorID == 0){
    delay(500);
  } else {
    delay(5000);
  }

}

// Draws a 360 degree colorwheel on the matrix rotated by offset
void drawCircleOnMatrix(int offset){
  for(int r = 0; r < HEIGHT; r++){
    for(int c = 0; c < WIDTH; c ++){
      int angle = ((int)((atan2(r - CENTER_ROW, c - CENTER_COL) * (180/M_PI)) + 180) % 360);
      int hue =  (int)(angle * 255.0/360 + offset) % 256;
      matrix.drawPixel(c,r, Wheel(hue));
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return matrix.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return matrix.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return matrix.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Prints given date and time to serial
void printDateTime(DateTime datetime){
  Serial.print(datetime.year(), DEC);
  Serial.print('/');
  Serial.print(datetime.month(), DEC);
  Serial.print('/');
  Serial.print(datetime.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[datetime.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(datetime.hour(), DEC);
  Serial.print(':');
  Serial.print(datetime.minute(), DEC);
  Serial.print(':');
  Serial.print(datetime.second(), DEC);
  Serial.println();
}

void checkDCFSignal(){
  pinMode(DCF_PIN, INPUT);
  Serial.println("Measure signal quality DCF77 (please wait)");
  Serial.println("NO SIGNAL   <- |                                          <- MISERABLE <- |  BAD      <- |          GOOD         | -> BAD       | -> MISERABLE ->");

  //Do measurement over 10 impules, one impulse takes exactly one Second
  int q = DCF77signalQuality(10);
  //If no change between HIGH and LOW was detected at the connection,
  //this means in 99.99% of all cases that the DCF receiver does not work
  //because with extremely poor reception you have changes, but you cannot evaluate them.
  if (!q) {Serial.print("# (Check connection!)");}
  for (int i = 0; i < q; i++) {
    Serial.print(">");
  }
  Serial.println("");

}

// check signalquality of DCF77 receiver (code from Ralf Bohnen, 2013
int DCF77signalQuality(int pulses) {
  int prevSensorValue=0;
  unsigned long loopTime = 10000; //Impuls Länge genau eine Sekunde
  //Da wir ja mitten in einem Impuls einsteigen könnten, verwerfen wir den ersten.
  int rounds = -1;
  unsigned long gagingStart = 0;
  unsigned long waitingPeriod = 0;
  int overallChange = 0;
  int change = 0;

  while (true) {
    //Unsere Schleife soll das Eingangssignal (LOW oder HIGH) 10 mal pro
    //Sekunde messen um das sicherzustellen, messen wir dessen Ausführungszeit.
    gagingStart = micros();
    int sensorValue = digitalRead(DCF_PIN);
    //Wenn von LOW nach HIGH gewechselt wird beginnt ein neuer Impuls
    if (sensorValue==1 && prevSensorValue==0) {
      rounds++;
      if (rounds > 0 && rounds < pulses + 1) {overallChange+= change;}
      if (rounds == pulses) { return overallChange /pulses;}
      change = 0;
    }
    prevSensorValue = sensorValue;
    change++;

    //Ein Wechsel zwichen LOW und HIGH müsste genau alle 100 Durchläufe stattfinden
    //wird er größer haben wir kein Empfang
    //300 habe ich als guten Wert ermittelt, ein höherer Wert würde die Aussage festigen
    //erhöht dann aber die Zeit.
    if (change > 300) {return 0;}
    //Berechnen und anpassen der Ausführungszeit
    waitingPeriod = loopTime - (micros() - gagingStart);
    delayMicroseconds(waitingPeriod);
  }
}

// Checks for new time from DCF77 and updates RTC time
void checkForNewDCFTime(DateTime now){
  // check if new time from DCF77 is available
  time_t DCFtime = DCF.getTime();
  if (DCFtime!=0)
  {
    setTime(DCFtime);
    // print new time from DCF77
    DateTime currentDCFtime = DateTime(year(), month(), day(), hour(), minute(), second());
    Serial.print("Got new time from DCF:");
    printDateTime(currentDCFtime);

    // id update intervall is over update RTC time with DCF time
    if((millis() - updateTimer) > updateIntervall){
      Serial.println("Adjust RTC time");
      rtc.adjust(currentDCFtime);
      updateTimer = millis();
    }
  }
}

// "activates" a pixel in grid
void gridAddPixel(uint8_t x,uint8_t y){
    grid[y][x] = 1;
}

// "deactivates" all pixels in grid
void gridFlush(void){
    //Setzt an jeder Position eine 0
    for(uint8_t i=0; i<HEIGHT; i++){
        for(uint8_t j=0; j<WIDTH; j++){
            grid[i][j] = 0;
        }
    }
}

// draws the grid to the ledmatrix with the current active color
void drawOnMatrix(){
  for(int z = 0; z < HEIGHT; z++){
    for(int s = 0; s < WIDTH; s++){
      if(grid[z][s] != 0){
        Serial.print("1 ");
        matrix.drawPixel(s,z,colors[activeColorID]);
      }
      else{
        Serial.print("0 ");
      }
    }
    Serial.println();
  }
}

/*
  | 0 1 2 3 4 5 6 7 8 9 10|
—————————————————————————————
0 | É S Ó N R L A M U N A | 0
1 | D O S L E S N T R E S | 1
2 | C I N C Q U A R T S U | 2
3 | M E N Y S I E C I N C | 3
4 | D E D'R U N A O N Z E | 4
5 | D U E S T R E S E T D | 5
6 | Q U A T R E D O T Z E | 6
7 | V U I T N O U O N Z E | 7
8 | S I S A M D E U N P M | 8
9 | M E N Y S I A C I N C | 9
—————————————————————————————
  | 0 1 2 3 4 5 6 7 8 9 10|

07:00 Són les set
07:05 Són les set i cinc
07:10 És un quart menys cinc de vuit
07:15 És un quart de vuit
07:20 És un quart i cinc de vuit
07:25 Són dos quarts menys cinc de vuit
07:30 Són dos quarts de vuit
07:35 Són dos quarts i cinc de vuit
07:40 Són tres quarts menys cinc de vuit
07:45 Són tres quarts de vuit
07:50 Són tres quarts i cinc de vuit
07:55 Són les vuit menys cinc
08:00 Són les vuit
*/

void printHours(uint8_t hours, uint8_t minutes) {
  switch(hours)
  {
  case 0:
      Serial.print("DOTZE");
      gridAddPixel(6,6);
      gridAddPixel(7,6);
      gridAddPixel(8,6);
      gridAddPixel(9,6);
      gridAddPixel(10,6);
      break;
  case 1:
      Serial.print("UNA");
      gridAddPixel(4,4);
      gridAddPixel(5,4);
      gridAddPixel(6,4);
      break;
  case 2:
      Serial.print("DUES");
      gridAddPixel(0,5);
      gridAddPixel(1,5);
      gridAddPixel(2,5);
      gridAddPixel(3,5);
      break;
  case 3:
      Serial.print("TRES");
      gridAddPixel(4,5);
      gridAddPixel(5,5);
      gridAddPixel(6,5);
      gridAddPixel(7,5);
      break;
  case 4:
      Serial.print("QUATRE");
      gridAddPixel(0,6);
      gridAddPixel(1,6);
      gridAddPixel(2,6);
      gridAddPixel(3,6);
      gridAddPixel(4,6);
      gridAddPixel(5,6);
      break;
  case 5:
      Serial.print("CINC");
      if (minutes >= 10 && minutes < 55) {
        gridAddPixel(7,9);
        gridAddPixel(8,9);
        gridAddPixel(9,9);
        gridAddPixel(10,9);
      } else {
        gridAddPixel(0,2);
        gridAddPixel(1,2);
        gridAddPixel(2,2);
        gridAddPixel(3,2);
      }
      break;
  case 6:
      Serial.print("SIS");
      gridAddPixel(0,8);
      gridAddPixel(1,8);
      gridAddPixel(2,8);
      break;
  case 7:
      Serial.print("SET");
      gridAddPixel(7,5);
      gridAddPixel(8,5);
      gridAddPixel(9,5);
      break;
  case 8:
      Serial.print("VUIT");
      gridAddPixel(0,7);
      gridAddPixel(1,7);
      gridAddPixel(2,7);
      gridAddPixel(3,7);
      break;
  case 9:
      Serial.print("NOU");
      gridAddPixel(4,7);
      gridAddPixel(5,7);
      gridAddPixel(6,7);
      break;
  case 10:
      Serial.print("DEU");
      gridAddPixel(5,8);
      gridAddPixel(6,8);
      gridAddPixel(7,8);
      break;
  case 11:
      Serial.print("ONZE");
      gridAddPixel(7,7);
      gridAddPixel(8,7);
      gridAddPixel(9,7);
      gridAddPixel(10,7);
      break;
  }
}

void printMenysCinc() {
  Serial.print("MENYS ");
  gridAddPixel(0,3);
  gridAddPixel(1,3);
  gridAddPixel(2,3);
  gridAddPixel(3,3);
  gridAddPixel(4,3);
  Serial.print("CINC ");
  gridAddPixel(7,3);
  gridAddPixel(8,3);
  gridAddPixel(9,3);
  gridAddPixel(10,3);
}

void printICinc() {
  Serial.print("I ");
  gridAddPixel(5,3);
  Serial.print("CINC ");
  gridAddPixel(7,3);
  gridAddPixel(8,3);
  gridAddPixel(9,3);
  gridAddPixel(10,3);
}

void printMenysCincFullHour() {
  Serial.print("MENYS ");
  gridAddPixel(0,9);
  gridAddPixel(1,9);
  gridAddPixel(2,9);
  gridAddPixel(3,9);
  gridAddPixel(4,9);
  Serial.print("CINC ");
  gridAddPixel(7,9);
  gridAddPixel(8,9);
  gridAddPixel(9,9);
  gridAddPixel(10,9);
}

void printICincFullHour() {
  Serial.print("I ");
  gridAddPixel(5,9);
  Serial.print("CINC ");
  gridAddPixel(7,9);
  gridAddPixel(8,9);
  gridAddPixel(9,9);
  gridAddPixel(10,9);
}
void printEs() {
  Serial.print("ÉS ");
  gridAddPixel(0,0);
  gridAddPixel(1,0);
}

void printSon() {
  Serial.print("SÓN ");
  gridAddPixel(1,0);
  gridAddPixel(2,0);
  gridAddPixel(3,0);
}

void printQuartOrQuarts(boolean plural) {
  Serial.print("QUART");
  gridAddPixel(4,2);
  gridAddPixel(5,2);
  gridAddPixel(6,2);
  gridAddPixel(7,2);
  gridAddPixel(8,2);
  if (!plural) {
    Serial.print(" ");
    return;
  }
  Serial.print("S ");
  gridAddPixel(9,2);
}


void printArticle(uint8_t hours) {
  if (hours == 1 || hours == 11) {
      Serial.print("D' ");
      gridAddPixel(2,4);
  } else {
    Serial.print("DE ");
    gridAddPixel(0,4);
    gridAddPixel(1,4);
  }
}

void printLa() {
  Serial.print("LA ");
  gridAddPixel(5,0);
  gridAddPixel(6,0);
}

void printLes() {
  Serial.print("LES ");
  gridAddPixel(3,1);
  gridAddPixel(4,1);
  gridAddPixel(5,1);
}

void printUn() {
  Serial.print("UN ");
  gridAddPixel(8,0);
  gridAddPixel(9,0);
}

void printDos() {
  Serial.print("DOS ");
  gridAddPixel(0,1);
  gridAddPixel(1,1);
  gridAddPixel(2,1);
}

void printTres() {
  Serial.print("TRES ");
  gridAddPixel(7,1);
  gridAddPixel(8,1);
  gridAddPixel(9,1);
  gridAddPixel(10,1);
}

// Converts the given time in grid representation
void timeToArray(uint8_t hours,uint8_t minutes){

  //clear grid
  gridFlush();

  //start filling grid with pixels

  //convert hours to 12h format
  if (hours >= 12) {
    hours -= 12;
  }
  if (minutes >= 10) {
    hours++;
  }

  if (minutes >= 55 || minutes < 10) {
    if (hours == 1) {
      printEs();
      printLa();
    } else {
      printSon();
      printLes();
    }
    printHours(hours, minutes);
    if (minutes >= 55) {
      printMenysCincFullHour();
    }
    else if (minutes >= 5) {
      printICincFullHour();
    }
  }
  // És un quart
  else if (minutes >= 10 && minutes < 25) {
    printEs();
    printUn();
    printQuartOrQuarts(false);
    if (minutes < 15) {
      printMenysCinc();
    } else if (minutes >= 20) {
      printICinc();
    }
    printArticle(hours);
    printHours(hours, minutes);
  }
  // Són dos quarts
  else if (minutes >= 25 && minutes < 40) {
    printSon();
    printDos();
    printQuartOrQuarts(true);
    if (minutes < 30) {
      printMenysCinc();
    } else if (minutes >= 35) {
      printICinc();
    }
    printArticle(hours);
    printHours(hours, minutes);
  }
  // Són tres quarts
  else if (minutes >= 40 && minutes < 55) {
    printSon();
    printTres();
    printQuartOrQuarts(true);
    if (minutes < 45) {
      printMenysCinc();
    } else if (minutes >= 50) {
      printICinc();
    }
    printArticle(hours);
    printHours(hours, minutes);
  }

  //separate LEDs for minutes in an additional row
  {
  switch (minutes%5)
        {
          case 0:
            break;

          case 1:
            gridAddPixel(0,10);
            break;

          case 2:
            gridAddPixel(0,10);
            gridAddPixel(1,10);
            break;

          case 3:
            gridAddPixel(0,10);
            gridAddPixel(1,10);
            gridAddPixel(2,10);
            break;

          case 4:
            gridAddPixel(0,10);
            gridAddPixel(1,10);
            gridAddPixel(2,10);
            gridAddPixel(3,10);
            break;
        }
  }

  Serial.println();
}
