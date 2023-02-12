#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP3008.h>

//Pin Connections (MCU to UV Sensor):
    //Sensor-to-MCU
    //3.3V-to-3.3V
    //Vin-to-ADC_VREF
    //GND-to-AGND
    //EN-to-3.3V
    //OUT-to-ADC1

//pin connections (MCU to MCP3008) 
#define CS_PIN D8
#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6

Adafruit_MCP3008 adc;

int count = 0;
int uv_sensor_datain = A0; //get analog data from the ML8511 UV Sensor

float conversion_factor = 3.3/1023.0;      //to convert analog data to voltage

int chan=0; // read from channel 0 from MCP3008

//interpolation formula to get UV index

float interpolation(float uv_volt_reading, float volt_in_min, float volt_in_max, float uv_intensity_min, float uv_intensity_max)
{
    return (uv_volt_reading - volt_in_min) * (uv_intensity_max - uv_intensity_min) / (volt_in_max - volt_in_min) + uv_intensity_min;
}

void setup() {
    Serial.begin(9600);
        while (!Serial);
        pinMode(uv_sensor_datain,INPUT);

  // Specify CS Pin
        adc.begin(CS_PIN);  

  // Software SPI (specify all, use any available digital)
  // (sck, mosi, miso, cs);
        adc.begin(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);
}

//loop to output voltage and UV index values

//there seems to be a discrency between using analogRead and adc.readADC...the latter being more accurate

void loop() {
//Reading from the A0 Pin of NodeMCU
   float uv_volt_reading = analogRead(uv_sensor_datain) * conversion_factor;
   float uv_index = interpolation(uv_volt_reading,0.99,2.8,0.0,15.0);

   Serial.print("Voltage Reading from A0: ");
   Serial.print(uv_volt_reading);

   Serial.print(" UV Level: ");
   Serial.print(uv_index);

//Reading from MCP3008 adc channel 0
   float mcp3008_volt_reading = adc.readADC(chan) * conversion_factor;
   float uv_index_mcp3008 = interpolation(mcp3008_volt_reading,0.99,2.8,0.0,15.0);

   Serial.print(" Voltage Reading from MCP3008 chn 1: ");
   Serial.print(mcp3008_volt_reading);

   Serial.print(" UV Level: ");
   Serial.print(uv_index_mcp3008);
   
   Serial.println();

// To get input from all ADC channels
  /* Serial.println();
     for (int chan=0; chan<8; chan++) {
    Serial.print(adc.readADC(chan)*conversion_factor); Serial.print("\t");
  }

  Serial.print("["); Serial.print(count); Serial.println("]");
  count++;*/
    delay(1000);
}

