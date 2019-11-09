#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <U8x8lib.h>
 
#define OLED_SCL 22             // GPIO 22
#define OLED_SDA 21             // GPIO 21
#define OLED_RST U8X8_PIN_NONE  // none
 
// define the display type that we use
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* rst */ OLED_RST);

void setup() {
  // set up the display
  u8x8.begin();
  u8x8.setPowerSave(0);
}

void loop() {
  // Yay... a "Hello World!"
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0,0,"Hello World!");
  delay(2000);
}