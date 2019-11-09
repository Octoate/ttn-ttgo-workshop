// include default platform libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// include the display library
#include <U8x8lib.h>
 
// include the BME280 sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// define the I2C pins
// TTGO v2.1 - T3_v1.6
#define I2C_SDA 21
#define I2C_SCL 22

// create the BME280 sensor
#define SEALEVELPRESSURE_HPA (1039.00)
Adafruit_BME280 bme280;

// define the GPIOs for the display
// NOTE: this will cause warning during the compilation, because we overwrite the
//       Arduino settings for this platform here
#define OLED_SCL I2C_SCL        // GPIO 22
#define OLED_SDA I2C_SDA        // GPIO 21
#define OLED_RST U8X8_PIN_NONE  // none
 
// define the display type that we use
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* rst */ OLED_RST);

void setup() {
  // set up serial port for debugging information --> 115200bps
  Serial.begin(115200);
  Serial.println("TTGO LoRa32 v2.1 - BME280 weather station started");

  // initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // initialize BME280 sensor
  // start communication with the BME280
  int status = bme280.begin(0x76);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

  // set up the display
  u8x8.begin(); 
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
}

void loop() {
  // show sensor data on the display
  u8x8.setCursor(0, 0);
  u8x8.clearDisplay();
  u8x8.print("Temp = ");
  u8x8.print(bme280.readTemperature());
  u8x8.println("*C");

  u8x8.print("Pre = ");

  u8x8.print(bme280.readPressure() / 100.0F);
  u8x8.println("hPa");

  u8x8.print("Alt = ");
  u8x8.print(bme280.readAltitude(SEALEVELPRESSURE_HPA));
  u8x8.println("m");

  u8x8.print("Hum = ");
  u8x8.print(bme280.readHumidity());
  u8x8.println("%");

  delay(2000);
}