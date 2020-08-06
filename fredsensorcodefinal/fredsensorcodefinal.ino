/*
Clear Blue Sea Maddog FRED Sensor Code
Authors: Sophia Austin and Justin Ho
Description: This program allows UV index, pressure, temperature,
             humidity, pH, and sonar distance sensors to communicate
             using I2C communication protocal.
*/

// use 680 ohm resistor for the PH sensor
// use 150 ohm resistor for the sonar
#include <Adafruit_AM2315.h>
#include <Adafruit_VEML6070.h>
#include <Wire.h>
#include "cactus_io_AM2315.h"
//download .zip file here: http://cactus.io/hookups/sensors/temperature-humidity/am2315/hookup-arduino-to-am2315-temp-humidity-sensor
#include "cactus_io_BME280_I2C.h"
//download .zip file here: http://cactus.io/hookups/sensors/barometric/bme280/hookup-arduino-to-bme280-barometric-pressure-sensor
AM2315 am2315;
BME280_I2C bme(0x76);
Adafruit_VEML6070 uv = Adafruit_VEML6070();


const int analogInPin = A0; 
int sensorValue = 0; 
unsigned long int avgValue; 
float b;
int buf[10],temp;

#define ECHOPIN 4// Pin to receive echo pulse
#define TRIGPIN 7// Pin to send trigger pulse

void setup() {
//AM2315 Temp/Humid
  Serial.begin(9600);
  Serial.println("AM2315 Humidity - Temperature Sensor");
  Serial.println("RH\t\tTemp (C)\tTemp (F)");
  if (!am2315.begin()) {
     Serial.println("Sensor not found, check wiring & pullups!");
     while (1);
  }
  if (!bme.begin()) {
Serial.println("Could not find a valid BME280 sensor, check wiring!");
}
  
//Sonar Distance
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  digitalWrite(ECHOPIN, HIGH);
}

void loop() {
//AM2315 Temp/Humidity
  am2315.readSensor();
  Serial.print("AM2315: ");
  Serial.print("Humidity = "); 
  Serial.print(am2315.getHumidity()); 
  Serial.print(" %\t");
  Serial.print("Temperature = "); 
  Serial.print(am2315.getTemperature_C()); 
  Serial.print(" *C\t");
  Serial.print(am2315.getTemperature_F()); 
  Serial.println(" *F\t");

//BME280 Pressure/Temp/Humidity
  bme.setTempCal(-1); //temp was reading high so subtract 1 degree
  bme.readSensor();
  Serial.print("BME280: ");
  Serial.print("Humidity = ");
  Serial.print(bme.getHumidity()); 
  Serial.print(" %\t");
  Serial.print("Temperature = "); 
  Serial.print(bme.getTemperature_C()); 
  Serial.print(" *C\t");
  Serial.print(bme.getTemperature_F()); 
  Serial.println(" *F\t");
  Serial.print("Pressure = "); 
  Serial.print(bme.getPressure_MB()); 
  Serial.print(" mb\t"); // Pressure in millibars

//Adafruit VEML6070 UV
  uv.begin(VEML6070_4_T);
  Serial.print("UV Value = ");
  Serial.println(uv.readUV());

// PH
  for(int i=0;i<10;i++) 
   { 
    buf[i]=analogRead(analogInPin);
    delay(10);
   }
  for(int i=0;i<9;i++)
   {
    for(int j=i+1;j<10;j++)
    {
     if(buf[i]>buf[j])
     {
      temp=buf[i];
      buf[i]=buf[j];
      buf[j]=temp;
     }
    }
   }
   avgValue=0;
   for(int i=2;i<8;i++)
   avgValue+=buf[i];
   float pHVol=(float)avgValue*5.0/1024/6;
   float phValue = -5.70 * pHVol + 21.34;
   Serial.print("pH Sensor Value = ");
   Serial.println(phValue);


// Sonar Distance Sensor
  digitalWrite(TRIGPIN, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW); // Send pin low again
  int distance = pulseIn(ECHOPIN, HIGH,26000); // Read in times pulse
  distance= distance/58;
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println("   cm");                    

  // Add a 2 second delay
  delay(500);
}
