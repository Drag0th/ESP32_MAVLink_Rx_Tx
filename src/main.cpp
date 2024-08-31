#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <MAVLinkHandler.h>

#define I2C_ADDRESS 0x3C
#define SERIAL_SPEED 57600
#define RXD2 16
#define TXD2 17

SSD1306AsciiWire oled_display;

void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
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
  request_data_streams();
  MAVLink_receive();
  delay(500);
  oled_display.clear();
  oled_display.set2X();
  oled_display.println("  MAVLink ");
  oled_display.set1X();
  oled_display.print("Lattitude: ");
  oled_display.println(ap_lat / 1E7, 7);
  oled_display.print("Longitude: ");
  oled_display.println(ap_lon / 1E7, 7);
  oled_display.print("Altitude [AGL]: ");
  oled_display.println(ap_alt_ag / 1000);
  oled_display.print("GPS Fixtype: ");
  oled_display.println(ap_fixtype);
  oled_display.print("Battery: ");
  oled_display.print(voltage_battery / 1000);
  oled_display.println("[V]");
  oled_display.print("Battery: ");
  oled_display.print(battery_remaining);
  oled_display.println("%");
}
