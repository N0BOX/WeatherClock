/* ---------------------------------
   ---------------------------------
    Included Libraries
   ---------------------------------
   --------------------------------- */
#include <SPI_VFD.h>

#include <Wire.h>
#include <Adafruit_HTU21DF.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <RTClib.h>
#include <RTC_DS3231.h>

/* ---------------------------------
   ---------------------------------
    Declaration of Global Variables
   ---------------------------------
   --------------------------------- */

// initialize the VFD library with the numbers of the interface pins
SPI_VFD vfd(8, 9, 10);

// initialize HTU21DF humidity sensor (I2C)
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// initialize BMP180  barometric sensor (I2C)
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// initialize the Chronodot RTC library (I2C)
RTC_DS3231 RTC;

byte charDegree[8] = {
	B00111,
	B00101,
	B00111,
	B00000,
	B00000,
	B00000,
	B00000,
	B00000
};

byte charRH[8] = {
	B10110,
	B11000,
	B10100,
	B10100,
	B00111,
	B00101,
	B00101,
	B00000
};

int readout = 0;


/* ---------------------------------
   ---------------------------------
    Arduino Setup
   ---------------------------------
   --------------------------------- */

void setup() {
  vfd.createChar(0x00, charDegree);
  vfd.createChar(0x01, charRH);

  vfd.begin(20, 2);
  // Print a message to the VFD.
//  vfd.setCursor(0, 0);
//  vfd.print("Waiting 4 serial...");

  Serial.begin(115200);
//  while (!Serial);
//  Serial.println("Hello, UART user.");

  vfd.clear();
//  vfd.setCursor(2, 0);
//  vfd.print("UART connected!");


  /* -------------------------------
     20x2 VFD
     -------------------------------
  
  // Print a message to the VFD.
  vfd.clear();
  vfd.setCursor(3, 0);
  vfd.print("Hello, world!"); //*/
  

  /* -------------------------------
     HTU21DF
     ------------------------------- */

  Serial.println("HTU21D-F test");

  if (!htu.begin()) {
    Serial.println("Couldn\'t find HTU21D-F");
    while (1);
  } //*/


  /* -------------------------------
     BMP180
     ------------------------------- */

  Serial.println("Pressure Sensor Test 01");

  if(!bmp.begin())
  {
    Serial.println("Couldn't find BMP180 barometric/atmospheric pressure sensor");
    while(1);
  }
  Serial.println("Pressure Sensor Test 02");


  /* -------------------------------
     Chronodot RTC
     ------------------------------- */

  RTC.begin();
  if (!RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  else
  {
    Serial.println("RTC is running!");
  }

  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    Serial.println("RTC is older than compile time!  Updating");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  } //*/
}


/* ---------------------------------
   ---------------------------------
    Main Program Loop
   ---------------------------------
   --------------------------------- */

void loop() {
  readout++;
  vfd.display();
  
  /* ----------------------------------------------------------------------------------------
     HTU21DF
     ---------------------------------------------------------------------------------------- */
  
  if ((readout % 3) == 2)
  {
    Serial.println("-------------------");
    Serial.println(" readout == humidity");
    Serial.println("--------------------------------------------------------------");
    Serial.print("Temperature:       "); Serial.print(htu.readTemperature()); Serial.println(" C");
    Serial.print("Relative Humidity: "); Serial.print(htu.readHumidity()); Serial.println("%");

    vfd.clear();
    vfd.setCursor(0, 0);
    vfd.print("Temperature: ");
    vfd.print((htu.readTemperature() * 1.8) + 32);
    vfd.write(0x00);
    vfd.print("F");
    vfd.setCursor(0, 1);
    vfd.print("Humidity:    ");
    vfd.print(htu.readHumidity());
    vfd.write(0x25);
    vfd.write(0x01);
  } //*/
  
  /* ----------------------------------------------------------------------------------------
     BMP180
     ---------------------------------------------------------------------------------------- */

  if ((readout % 3) == 1)
  {
    Serial.println("-------------------");
    Serial.println(" readout == barometer");
    Serial.println("--------------------------------------------------------------");
    sensors_event_t event;
    bmp.getEvent(&event);
    
    if (event.pressure)
    {
      Serial.print("Pressure:    ");
      Serial.print(event.pressure);
      Serial.println(" hPa");
      
      float temperature;
      bmp.getTemperature(&temperature);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" C");
  
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      Serial.print("Altitude:    "); 
      Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                          event.pressure,
                                          temperature)); 
      Serial.println(" m");

      vfd.clear();
      vfd.setCursor(0, 0);
      vfd.print("Temperature: ");
      vfd.print((temperature * 1.8) + 32);
      vfd.write(0x00);
      vfd.print("F");
      vfd.setCursor(0, 1);
      vfd.print("Pressure:  ");
      vfd.print((bmp.seaLevelForAltitude(289.0, event.pressure, temperature) * 0.02952998751), 2);
      vfd.print("inHg");
    }
    else
    {
      Serial.println("Sensor error");
    }
  }//*/
  
  /* ----------------------------------------------------------------------------------------
     Chronodot RTC
     ---------------------------------------------------------------------------------------- */

  if ((readout % 3) == 0)
  {
    Serial.println("-------------------");
    Serial.println(" readout == time");
    Serial.println("--------------------------------------------------------------");
    DateTime now = RTC.now();
    RTC.forceTempConv(true);  //DS3231 does this every 64 seconds, we are simply testing the function here

    float temp_float = RTC.getTempAsFloat();

    int16_t temp_word = RTC.getTempAsWord();

    int8_t temp_hbyte = temp_word >> 8;
    int8_t temp_lbyte = temp_word &= 0x00FF;

    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(' ');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.println();

    //Display temps
    Serial.print("Temp as float: ");
    Serial.print(temp_float, DEC);
    Serial.println();
    Serial.print("Temp as word: ");
    Serial.print(temp_hbyte, DEC);
    Serial.print(".");
    Serial.println(temp_lbyte, DEC);

    vfd.clear();
    vfd.print(now.hour(), DEC);
    vfd.print(':');
    vfd.print(now.minute(), DEC);
    vfd.print(':');
    vfd.print(now.second(), DEC);
    vfd.print("   ");
    vfd.print(now.month(), DEC);
    vfd.print('/');
    vfd.print(now.day(), DEC);
    vfd.print('/');
    vfd.print(now.year(), DEC);
    vfd.setCursor(0, 1);
    vfd.print("Temperature: ");
    vfd.print(((temp_float * 1.8) + 32), 2);
    vfd.write(0x00);
    vfd.print("F");
  } //*/

  Serial.println("");
  Serial.println("");
  delay(3000);
}
