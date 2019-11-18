#include <Arduino.h>

// function declarations
void buildDataContainer();
void readSensorData();
void displaySensorData();

void setup() 
{
  Serial.begin(115200);
  Serial.println("LoRaWAN Workshop 2019");
  Serial.println("Projekt: Wetterstation");

  // put your setup code here, to run once:
}

void loop() 
{
  // put your main code here, to run repeatedly:
  readSensorData();
  displaySensorData();
  buildDataContainer();

  Serial.println("Loop");
  delay(2000);
}

/*
 * Prepare sensordata for LoRaWAN transmission
 */
void buildDataContainer() 
{

}

/*
 * Read sensor data
 */
void readSensorData() 
{
  
}

/*
 * Show sensor data on the display
 */
void displaySensorData() 
{
  
}