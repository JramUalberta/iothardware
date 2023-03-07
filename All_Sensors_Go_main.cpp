//ECE 490/1 Electrical Engineering Capstone 2023 Team 12
//List of all libraries
#include <Arduino.h>
#include <Wire.h> //Analog Sensors
#include <SPI.h>  //Analog Sensors
#include <Adafruit_MCP3008.h> //ADC converter
#include <Adafruit_Sensor.h>//(BME688)
#include "Adafruit_BME680.h"//(BME688)
#include <Adafruit_GPS.h> //(GPS PA1616S)
#include <ESP8266WiFi.h> //Setup on-board wifi

//SENSOR DECLARATIONS:

int counter = 0; //to keep track of the readings outputted

//WIFI DECLARATIONS:

#define networkName "TELUS5817"
#define networkPassword  "9ixthjhv5n"

//MICROPHONE, UV SENSOR, MCP3008 DECLARATIONS:

//PIN CONNECTIONS (MCU to UV SENSOR):
    //Sensor-to-MCU
    //3.3V-to-3.3V
    //Vin-to-ADC_VREF
    //GND-to-AGND
    //EN-to-3.3V
    //OUT-to-MCP3008 Chn 7 -> can switch to A0 on the NodeMCU if desired

//PIN CONNECTIONS (MCU to MCP3008) 
#define CS_PIN D8
#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6

Adafruit_MCP3008 adc;

//previous VU value
int preValue = 0; 
int count = 0;
int uv_sensor_data_in = A0; //get analog data from the ML8511 UV Sensor

float conversion_factor = 3.3/1023.0;      //to convert incoming analog data to voltage

int chan0=0; // read Microphone data from MCP3008 channel 0
int chan7=7; //read UV data from MCP3008 channel 7 

const int sampleTime = 50; // mic
int micOut; // mic
int micOutdb; // mic

//interpolation formula to get UV index

float interpolation(float uv_volt_reading, float volt_in_min, float volt_in_max, float uv_intensity_min, float uv_intensity_max)
{
    return (uv_volt_reading - volt_in_min) * (uv_intensity_max - uv_intensity_min) / (volt_in_max - volt_in_min) + uv_intensity_min;
}

//BME688 Declarations

//PIN CONNECTIONS (MCU to BME688):
    //Sensor-to-MCU
    //SCK-TO-SCL (D1)
    //SDI-TO-SDA(D2)
    //GNDs-TO-GNDs
    //VDD-TO-3.3V    
    //VDDIO-TO-3.3V
    
#define SEALEVELPRESSURE_HPA (1013.25) //(BME688)

Adafruit_BME680 bme; // I2C (BME688)

//GPS DECLARATIONS:

//PIN CONNECTIONS (GPS to NodeMCU):
  //Sensor-to-MCU
  //Vin-to-3.3V
  //TX-to-RX(GPI03)
  //RX-to-TX(GPI01)
  //GND-to-GND  

#define GPSSerial Serial //define hardware serial connection for GPS Sensor

Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis(); //for Serial printing GPS Sensor

//BUZZER DECLARATIONS

int BUZZER = D0;

//SENSOR SETUP:

void setup() {

//WIFI SETUP:

Serial.begin(115200);
Serial.println();

WiFi.begin(networkName, networkPassword);
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}
Serial.println();
//ip address
//Serial.print("Connected, IP address: ");
//Serial.println(WiFi.localIP());

//MICROPHONE, UV SENSOR, MCP3008 SETUP:

  pinMode(uv_sensor_data_in,INPUT); //A0 is an input for any analog data
  adc.begin(CS_PIN);  // Specify CS Pin
  adc.begin(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);  // Software SPI for MCP3008 (sck, mosi, miso, cs);

//BME688 SETUP

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME688 sensor, check wiring!"));
    while (1);
  }

  // Set up oversampling and filter initialization (BME688)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  
  //GPS SETUP
  while (!Serial); //have the sketch wait until Serial is ready
  GPS.begin(9600); // 9600 NMEA is the default baud rate 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //turns on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); //antenna status

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version

//BUZZER SETUP

pinMode(BUZZER, OUTPUT);

}


void loop() {

  Serial.print("Reading ");
  counter = counter + 1;
  Serial.print(counter);
  Serial.println();

  //UV READINGS

  //Reading from the A0 Pin of NodeMCU
   /*float uv_volt_reading = analogRead(uv_sensor_data_in) * conversion_factor;
   float uv_index = interpolation(uv_volt_reading,0.99,2.8,0.0,15.0);

   Serial.print("Voltage Reading from A0: ");
   Serial.print(uv_volt_reading);

   Serial.print(" UV Level: ");
   Serial.print(uv_index);*/

  //Reading from the MCP3008 to NodeMCU
   float mcp3008_volt_reading = adc.readADC(chan0) * conversion_factor;
   float uv_index_mcp3008 = interpolation(mcp3008_volt_reading,0.99,2.8,0.0,15.0);

  //Uncomment to get UV voltage readings
   /*Serial.print(" Voltage Reading from MCP3008 chn 0: ");
   Serial.print(mcp3008_volt_reading);
   Serial.println(F(" V"));*/

   Serial.print("UV Level = ");
   Serial.print(uv_index_mcp3008);
   
   Serial.println();
   delay(50);   

//MIC READINGS
  unsigned long startTime= millis();  // Start of sample window
   unsigned int PTPAmp = 0; 

// Signal variables to find the peak-to-peak amplitude
   int maxAmp = 0;
   int minAmp = 1023;

// Find the max and min of the mic output within the 50 ms timeframe
   while(millis() - startTime < sampleTime) 
   {
      micOut = adc.readADC(chan7);
      if( micOut < 1023) //prevent erroneous readings
      {
        if (micOut > maxAmp)
        {
          maxAmp = micOut; //save only the max reading
        }
        else if (micOut < minAmp)
        {
          minAmp = micOut; //save only the min reading
        }
      }

   }
   PTPAmp = maxAmp - minAmp; // (max amp) - (min amp) = peak-to-peak amplitude
     double micOut_Volts = PTPAmp * conversion_factor; // Convert ADC into voltage

 int fill = map(PTPAmp, 8, 1011, 31, 92); 
   
  //Uncomment to get microphone voltage readings

   /*Serial.print(" Voltage Reading from MCP3008 chn 7: ");
   Serial.print(micOut_Volts);
   Serial.println(F(" V"));*/

   Serial.print(F("Sound = "));
   Serial.print(fill);
   Serial.println(F(" db"));

//If Sound dB exceeds 85 dB, alert user
   if(fill > 31){
    tone(BUZZER,1000);
    delay(2000);
    noTone(BUZZER);
    delay(500);
   }
   delay(50);   

//BME688 READINGS
   // Tell BME688 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));

  //If Temperature exceeds 40*C or -40*C , alert user
   if(bme.temperature > 40 or bme.temperature < -40){
    tone(BUZZER,1000);
    delay(1000);
    noTone(BUZZER);
    delay(500);
   }
  //If TVOC exceeds ??? , alert user
   if(bme.gas_resistance / 1000.0 > 100){
    tone(BUZZER,1000);
    delay(100);
    noTone(BUZZER);
    delay(100);
   }

  Serial.println();
  delay(50); 
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
 // reset the timer
    /*Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);*/
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  Serial.println();
  delay(1000);
}
