#include <Adafruit_GPS.h>

/* ---------------------------------
   ---------------------------------
    Included Libraries
   ---------------------------------
   --------------------------------- */
#include <SPI_VFD.h>             // 20x2 character SPI Vacuum Flourescent Display library

#include <Wire.h>                // Arduino's wire library, which supports use of the I2C bus
#include <Adafruit_HTU21DF.h>    // HTU21D-F temperature/humiidity sensor library by Adafruit

#include <Adafruit_Sensor.h>     // Unified sensor library by Adafruit
#include <Adafruit_BMP085_U.h>   // BMP180 barometric pressure sensor library by Adafruit

#include <DS1307RTC.h>           // Real-time clock library compatible with DS3231 RTC
#include <Time.h>                // Time library

#include <Adafruit_GPS.h>        // Adafruit GPS library for Ultimate GPS Breakout v3 (PA6H)

#include <Encoder.h>             // Rotary encoder library by Paul Stoffregen


/* ---------------------------------------
   ---------------------------------------
    Declaration of Global Variables/Macros
   ---------------------------------------
   --------------------------------------- */

// Enable Debug output to USB-UART if defined.  No reason to flood the Serial
// output if no debugging is taking place.
#define DEBUG

// Name and version of the program
#define WC_NAME "*** WeatherClock ***"
#define WC_VER "--- ver 1.08.023 ---"

// Easy names to keep track of the pins used by different devices
#define PinInt01       2
#define PinEnc01_1     6
#define PinEnc01_2     5
#define PinEnc01_Sel   4
#define PinVFDSPI_MOSI 8
#define PinVFDSPI_SCK  9
#define PinVFDSPI_CS   10

// Use a smaller, easier to recognize macro for the 1Hz square wave frequency of the Chronodot
#define SQW_FREQ DS3231_SQW_FREQ_1

// Define Real-time Clock object
DS1307RTC RTC = DS1307RTC();

// Define a HardwareSerial variable that can be passed to the GPS lib
HardwareSerial sGPS = Serial1;
Adafruit_GPS GPS(&sGPS);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

// initialize the VFD library with the names of the interface pins
SPI_VFD vfd(PinVFDSPI_MOSI, PinVFDSPI_SCK, PinVFDSPI_CS);

// initialize HTU21DF humidity sensor (I2C)
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// initialize BMP180  barometric sensor (I2C)
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// initialize Rotary Encoder object with its pin names and set the related debounce timer vars
Encoder myEnc(PinEnc01_1, PinEnc01_2);
bool bEncoderChanged01;                  // whether or not the encoder's position has changed
bool bEncoderSelected01 = false;         // whether or not the encoder's select switch has been pressed
bool bEncoderUnselecting01 = false;      // whether or not the encoder's select switch was let go of
bool bAllowUnselect01 = false;           // whether of not we should allow the rotary encoder's select switch to be considered unselectable
int posEnc01_Sel_old = HIGH;             // set the initial state of the rotary encoder's select switch
elapsedMicros timerEncoderDebounce01;    // tracks the number of microseconds since the rotary encoder triggered an interrupt
elapsedMicros timerEncoderSelDebounce01; // tracks the number of microseconds since the rotary encoder select triggered an interrupt
uint16_t iDebounceTime    = 750;         // number of microseconds to wait for the rotary encoder state to become stable
uint16_t iDebounceSelTime = 750;         // number of microseconds to wait for the rotary encoder select switch to become stable
uint16_t iDebounceUnselTime = 750;       // number of microseconds to wait for the rotary encoder select switch to become stable

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

byte charDegree[8] = {    // This degree symbol is drawn on the right edge of the 5x7 matrix, instead of the left
	B00111,
	B00101,
	B00111,
	B00000,
	B00000,
	B00000,
	B00000,
	B00000
};

byte charDegreeF[8] = {    // A tiny degrees Farenheit symbol inside a single 5x7 matrix
	B11011,
	B11010,
	B00011,
	B00010,
	B00010,
	B00000,
	B00000,
	B00000
};

byte charRH[8] = {         // A small r/h symbol inside a single 5x7 matrix
	B10110,
	B11000,
	B10100,
	B10100,
	B00111,
	B00101,
	B00101,
	B00000
};

byte charHg[8] = {         // A small Hg symbol inside a single 5x7 matrix
	B10010,
	B11110,
	B10010,
	B10111,
	B00111,
	B00001,
	B00111,
	B00000
};

// Give names to the positions in the VFD's CGRAM where the custom characters are stored
#define CHAR_DEGREE  0x00
#define CHAR_DEGREEF 0x01
#define CHAR_RELHUM  0x02
#define CHAR_HG      0x03

// Declare the global variables that store the temperature, humidity, and barometric pressure readings, and their averages
float avgTemp = 0.00;
float avgHum  = 0.00;
float avgBMP  = 0.00;

float ar_Temps1[10] = {-273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15}; // Store last ten temperature readings from the HTU21D-F sensor
float ar_Temps2[10] = {-273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15}; // Store last ten temperature readings from the BMP180 sensor
float ar_Temps3[10] = {-273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15, -273.15}; // Store last ten temperature readings from the ChronoDot RTC
float ar_Hums[10]   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                                         // Store last ten humidity readings from the HTU21D-F sensor
float ar_BMPs[10]   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                                         // Store last ten barometric pressure readings from the BMP180 sensor

// Declare variables used to track the position of the rotary encoder and the times when readings/the display should be updated
uint16_t counter = 0;
uint16_t lastCount = 0;
long oldPosition  = -999;

/* ---------------------------------------
   ---------------------------------------
    Function Definitions
   ---------------------------------------
   --------------------------------------- */

/*
 * push - Push a floating point value onto an array, and if the
 *     array is full, drop the oldest value
 */
void push(float data, float* ar_Floats, int iMaxRecords, float fMin)
{
  int iRecords = 0;
  int i;
  for (i = 0; i < iMaxRecords; i++)
  {
    if (ar_Floats[i] != fMin)
      iRecords++;
  }

  if (iRecords == iMaxRecords)
  {
    for (i = 0; i < iMaxRecords - 1; i++)
    {
      ar_Floats[i] = ar_Floats[i + 1];
    }
  }
  if (iRecords < iMaxRecords)
    ar_Floats[iRecords] = data;
  else
    ar_Floats[iMaxRecords - 1] = data;
}

/*
 * getArrayAVG - Calculate the average floating point value of all
 *     the values in the array that aren't equal to fMin, which should
 *     represent a value that is impossible/outside of the measurable
 *     range of the sensor.  Unset positions in the array should be set
 *     to fMin so that this function evaluates successfully.
 */
float getArrayAVG(float* ar_Floats, int iSize, float fMin)
{
  int iRecords = 0;
  float sum = 0.0;
  for (int i = 0; i < iSize; i++)
  {
    if (ar_Floats[i] != fMin)
    {
      sum += ar_Floats[i];
      iRecords++;
    }
  }

  float retval = sum / iRecords;
  return retval;
}

/*
 * EncoderSelect01 - This function carries out the "Select" functionality
 *     triggered when clicking the rotary encoder's switch.
 */
void EncoderSelect01()
{
#ifdef DEBUG
  Serial.println("Rotary Encoder 01's select switch was triggered!");
#endif
}

/*
 * UpdateDisplay_Temp - Update the VFD display with a new temperature
 *     reading.
 */
void UpdateDisplay_Temp()
{
  // Print average Temperature in Farenheit
  float fTempF = ((avgTemp * 1.8) + 32);
  if (fTempF > 100.0)
    vfd.setCursor(0, 1);
  else
    vfd.setCursor(1, 1);
  vfd.print(fTempF, 1);
  vfd.write(CHAR_DEGREEF);
  //vfd.print("F");
}

/*
 * UpdateDisplay_Hmdty - Update the VFD display with a new humidity
 *     reading.
 */
void UpdateDisplay_Hmdty()
{
  // Print average Relative Humidity
  vfd.setCursor(7, 1);
  vfd.print(avgHum, 1);
  vfd.write(CHAR_RELHUM);
}

/*
 * UpdateDisplay_BMP - Update the VFD display with a new barometric
 *     pressure reading.
 */
void UpdateDisplay_BMP()
{
  // Print average Barometric Pressure
  vfd.setCursor(13, 1);
  vfd.print(avgBMP, 2);
  vfd.print("\"");
  vfd.write(CHAR_HG);
}

/*
 * UpdateDisplay_Time - Update the VFD display with the new time and
 *     date.
 */
void UpdateDisplay_Time()
{
  vfd.display();  
  tmElements_t tm;
  if (RTC.read(tm))
  {
    // Print Time and Date
    vfd.setCursor(0, 0);
    if (tm.Hour < 10)
      vfd.print('0');
    vfd.print(tm.Hour);
    vfd.print(':');
    if (tm.Minute < 10)
      vfd.print('0');
    vfd.print(tm.Minute);
    vfd.print(':');
    if (tm.Second < 10)
      vfd.print('0');
    vfd.print(tm.Second);
    vfd.print("  ");
    if (tm.Month < 10)
      vfd.print('0');
    vfd.print(tm.Month);
    vfd.print('/');
    if (tm.Day < 10)
      vfd.print('0');
    vfd.print(tm.Day);
    vfd.print('/');
    vfd.print(tmYearToCalendar(tm.Year));
  }
}

/*
 * isrSecondCounter - This function is called whenever the Chronodot's
 *     SQW pin goes low, thanks to an interrupt.  It increments a counter,
 *     which tells the main loop to update the VFD with the new time/date.
 */
void isrSecondCounter()
{
  UpdateDisplay_Time();
  counter++;
#ifdef DEBUG2
  Serial.println("Interrupt encountered from Chronodot!");
#endif
}

/*
 * isrEncoderSelect01 - This function is called when a user pressed down on
 *     the first rotary encoder's knob, activating the "Select" functionality.
 *     Because this is a hardware switch, we have to account for button
 *     "bounce".
 */
void isrEncoderSelect01()
{
  bEncoderSelected01 = true;
  timerEncoderSelDebounce01 = 0;
#ifdef DEBUG
  Serial.println("Interrupt encountered from Rotary Encoder 01 Select Switch!");
#endif
}

uint32_t timer = millis();

bool getTime(tmElements_t &tm, const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(tmElements_t &tm, const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}


/* ---------------------------------------
   ---------------------------------------
    Arduino Setup
   ---------------------------------------
   --------------------------------------- */

void setup() {
  counter++;
  delay(1000);
  vfd.begin(20, 2);
  vfd.clear();
  vfd.print(WC_NAME);
  vfd.setCursor(0, 1);
  vfd.print(WC_VER);
  delay(3000);

  Serial.begin(115200);
  while (!Serial) {}
  
  vfd.setCursor(0, 1);
  vfd.createChar(CHAR_DEGREEF, charDegreeF);
  vfd.createChar(CHAR_RELHUM, charRH);
  vfd.createChar(CHAR_HG,     charHg);

  /* -------------------------------
     Ultimate GPS (PA6H) Setup
     ------------------------------- */

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  // useInterrupt(true);


  /* -------------------------------
     HTU21DF Setup
     ------------------------------- */

  if (!htu.begin()) {
    Serial.println("ERROR: Couldn\'t find HTU21D-F temperature/humidity sensor!");
    while (1);
  } //*/
  else
  {
#ifdef DEBUG
    Serial.println("HTU21D-F temperature/humidity sensor detected.");
#endif
  }


  /* -------------------------------
     BMP180 Setup
     ------------------------------- */

  if(!bmp.begin())
  {
    Serial.println("ERROR: Couldn't find BMP180 barometric/atmospheric pressure sensor!");
    while(1);
  }
  else
  {
#ifdef DEBUG
    Serial.println("BMP180 barometric/atmospheric pressure sensor detected.");
#endif
  }


  /* -------------------------------
     Chronodot RTC Setup
     ------------------------------- */



  tmElements_t tm;
  if (RTC.read(tm))
  {
//    RTC.SQWEnable(true);
//    RTC.SQWFrequency( SQW_FREQ );
#ifdef DEBUG
    Serial.println("RTC is running.");
    Serial.printf("Compiled date: %s\r\nCompiled time: %s\r\n", __DATE__, __TIME__);
#endif
  }
  else
  {
    tmElements_t compiled;
    getDate(compiled, __DATE__);
    getTime(compiled, __TIME__);
    if (makeTime(tm) < makeTime(compiled)) {
      Serial.println("RTC is older than compile time!  Updating...");
      RTC.write(compiled);
    } //*/
  }

  pinMode(PinInt01, INPUT);
  digitalWrite(PinInt01, HIGH);
  attachInterrupt(PinInt01, isrSecondCounter, FALLING); // Chronodot interrupt 1 is data ready


  /* -------------------------------
     Rotary Encoder Setup
     ------------------------------- */

#ifdef DEBUG
  Serial.println("Rotary Encoder Position: 0");
#endif
  oldPosition = myEnc.read() / 4;
  pinMode(PinEnc01_Sel, INPUT);
  digitalWrite(PinEnc01_Sel, HIGH);
  attachInterrupt(PinEnc01_Sel, isrEncoderSelect01, FALLING); // Attach interrupt to Rotary Encoder 01's select button

  vfd.clear();

  // Ask for firmware version
  sGPS.println(PMTK_Q_RELEASE);
}


/* ---------------------------------------
   ---------------------------------------
    Main Program Loop
   ---------------------------------------
   --------------------------------------- */

void loop() {
  float temp;
  float humidity;
  float pressure;

  // Some logic here to prevent the rapid changes back and forth in encoder position
  // that were being seen.  For some reason it takes almost a whole millisecond for
  // these mechanical rotary encoders to pick a position and stick with it.
  long newPosition = myEnc.read() / 4;
  if (newPosition != oldPosition && !bEncoderChanged01) {
    bEncoderChanged01 = true;
    timerEncoderDebounce01 = 0;
  }
  if (bEncoderChanged01)
  {
    if (timerEncoderDebounce01 > iDebounceTime)
    {
      vfd.setCursor(0, 1);
      // Allow the rotary encoder to set the left/right position of the text on the VFD
      if (newPosition > oldPosition)
        vfd.scrollDisplayRight();
      if (newPosition < oldPosition)
        vfd.scrollDisplayLeft();
      oldPosition = newPosition;
      bEncoderChanged01 = false;
#ifdef DEBUG
    Serial.println(newPosition);
#endif
    }
  }

  uint8_t posEnc01_Sel_new = digitalRead(PinEnc01_Sel);
  if (bEncoderSelected01)
  {
    if (!bAllowUnselect01)
    {
      if (timerEncoderSelDebounce01 > iDebounceSelTime)
      {
        if (posEnc01_Sel_new == LOW && posEnc01_Sel_old == HIGH)
        {
          EncoderSelect01();
          bAllowUnselect01 = true;
          bEncoderUnselecting01 = false;
          bEncoderSelected01 = false;
          posEnc01_Sel_old = LOW;
#ifdef DEBUG
          Serial.println("Rotary Encoder selected!");
#endif
        }
        else
        {
          bEncoderUnselecting01 = false;
          bEncoderSelected01 = false;
          posEnc01_Sel_old = HIGH;
        }
      }
    }
    else
    {
      bEncoderSelected01 = false;
    }
  }

  if (bAllowUnselect01)
  {
    if (!bEncoderUnselecting01 && posEnc01_Sel_new == HIGH)
    {
      timerEncoderSelDebounce01 = 0;
      bEncoderUnselecting01 = true;
    }
    if (bEncoderUnselecting01 && posEnc01_Sel_new == LOW)
    {
      timerEncoderSelDebounce01 = 0;
    }
    if (bEncoderUnselecting01 && timerEncoderSelDebounce01 > iDebounceUnselTime)
    {
      bAllowUnselect01 = false;
      bEncoderUnselecting01 = false;
      posEnc01_Sel_old = HIGH;
#ifdef DEBUG
      Serial.println("Rotary Encoder unselected!");
#endif
    }
  }

  // Every time the Chronodot notifies the Teensy 3.1 that a second has passed,
  // this code should be run...
  if (counter > lastCount)
  {
    // Grab an updated temperature and humidity reading from the HTU21D-F and push
    // those reading onto their arrays.
    temp = htu.readTemperature();
    humidity = htu.readHumidity();
    push(temp, ar_Temps1, 10, -273.15);
    push(humidity, ar_Hums, 10, 0.0);

    // Take a new pressure and temperature reading from the BMP180, normalize the
    // barometric pressure to sea level, and push the readings onto their arrays.
    sensors_event_t event;
    bmp.getEvent(&event);
    pressure = event.pressure;
    if (pressure)
    {
      bmp.getTemperature(&temp);
      push(temp, ar_Temps2, 10, -273.15);
      push(bmp.seaLevelForAltitude(289.0, pressure, temp) * 0.02952998751, ar_BMPs, 10, 0.0);
    }

    // Take a temperature reading from the Chronodot, too, while we're at it... we
    // might as well use its temp readings for our average temperature, too!
    temp = RTC.getTempAsFloat();
    push(temp, ar_Temps3, 10, -273.15);

    // Now that all the temp, pressure, and humidity values have been pushed onto the
    // 10-value arrays, go ahead and average those arrays to get the avgTemp, avgBMP, and
    // avgHum values, then run the UpdateDisplay() function.
    avgHum = getArrayAVG(ar_Hums, 10, 0.0);
    avgBMP = getArrayAVG(ar_BMPs, 10, 0.0);
    avgTemp = (getArrayAVG(ar_Temps1, 10, -273.15) +
               getArrayAVG(ar_Temps2, 10, -273.15) +
               getArrayAVG(ar_Temps3, 10, -273.15)) / 3.0;
               
    if (counter % 10 == 0 || counter == 1)
      UpdateDisplay_BMP(); // Update the barometric pressure reading on the VFD every 10 seconds (doesn't change often)

    if (counter % 5 == 0 || counter == 1)
      UpdateDisplay_Temp(); // Update the temperaturee reading on the VFD every 5 seconds (changes somewhat quickly)

    if (counter % 2 == 0 || counter == 1)
      UpdateDisplay_Hmdty(); // Update the humidity reading on the VFD every 2 seconds (can change very quickly)
  }

  // Now that any use of the counter variable is complete, update the lastCount variable with the new counter value
  lastCount = counter;
}
