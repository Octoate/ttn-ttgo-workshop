#include <Arduino.h>

// function declarations
void buildDataContainer();
void readSensorData();
void displaySensorData();

void setup() 
{
  // put your setup code here, to run once:
}

void loop() 
{
  // put your main code here, to run repeatedly:
  readSensorData();
  displaySensorData();
  buildDataContainer();
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