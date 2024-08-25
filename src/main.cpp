#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <MAVLink.h>
#include <Arduino.h>

#define I2C_ADDRESS 0x3C

SSD1306AsciiWire oled_display;

#define SERIAL_SPEED 57600

void setup()
{
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000L);
  oled_display.begin(&Adafruit128x64, I2C_ADDRESS);
  oled_display.set2X();
  oled_display.setFont(System5x7);
  oled_display.println("BOOTING");
  oled_display.clear();
  oled_display.set1X();
  oled_display.setFont(System5x7);
}
//------------------------------------------------------------------------------
void loop()
{
  oled_display.println("MAVLink: ");
  if (Serial.available())
  {
    char c = Serial.read();
    oled_display.println(c);
  }
  delay(50);
  oled_display.clear();
}
