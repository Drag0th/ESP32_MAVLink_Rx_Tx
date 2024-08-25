#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <mavlink_types.h>
#include <common/mavlink.h>
#include <Arduino.h>

#define I2C_ADDRESS 0x3C

// ******************************************
// Mavlink Message Types

// Mavlink Header
uint8_t system_id;
uint8_t component_id;
uint8_t target_component;
uint8_t mvType;

// Message #0  HEARTHBEAT
uint8_t ap_type = 0;
uint8_t ap_autopilot = 0;
uint8_t ap_base_mode = 0;
uint32_t ap_custom_mode = 0;
uint8_t ap_system_status = 0;
uint8_t ap_mavlink_version = 0;

// Message #24  GPS_RAW_INT
uint8_t ap_fixtype = 0;     //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t ap_sat_visible = 0; // numbers of visible satelites
uint8_t ap_gps_status = 0;  // (ap_sat_visible*10) + ap_fixtype;
int32_t ap_latitude = 0;    // 7 assumed decimal places
int32_t ap_longitude = 0;   // 7 assumed decimal places
int32_t ap_amsl24 = 0;      // 1000 = 1m
uint16_t ap_eph;            // GPS HDOP horizontal dilution of position (unitless)
uint16_t ap_epv;            // GPS VDOP vertical dilution of position (unitless)
uint16_t ap_vel;            //  GPS ground speed (m/s * 100)
uint16_t ap_cog;            // Course over ground in degrees * 100, 0.0..359.99 degrees.

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap_lat;    // Latitude, expressed as degrees * 1E7
int32_t ap_lon;    // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33; // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag; // Altitude above ground (millimeters)
int16_t ap_vx;     //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;     //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;     // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_hdg;   // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

// Mavlink varaibles
uint32_t hb_millis = 0;
uint16_t hb_count = 0;

//***************************************************
uint8_t len;
uint8_t apo_sysid;
uint8_t apo_compid;
uint8_t apo_targsys;
uint8_t apo_targcomp;
uint8_t sendbuf[128];

SSD1306AsciiWire oled_display;
mavlink_message_t msg, sendmsg;
mavlink_status_t status;
#define SERIAL_SPEED 57600

void MavLink_Receive();
void Send_To_FC(uint32_t msg_id);
void RequestDataStreams();

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
  /*
  oled_display.println("MAVLink: ");
  for (int i = 0; i < 7; i++)
  {
    if (Serial.available())
    {
      uint8_t c = Serial.read();
      oled_display.println(c);
    }
  }
  delay(500);
  oled_display.clear();
  */
  RequestDataStreams();
  MavLink_Receive();
  delay(500);
  oled_display.clear();
  oled_display.println(ap_lat);
  oled_display.println(ap_lon);
  oled_display.println(ap_type);
  oled_display.println(ap_fixtype);
}

void MavLink_Receive()
{

  while (Serial.available())
  {
    uint8_t c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      switch (msg.msgid)
      {

      case MAVLINK_MSG_ID_HEARTBEAT: // #0   http://mavlink.org/messages/common
        ap_type = mavlink_msg_heartbeat_get_type(&msg);
        ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
        ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
        ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
        ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
        ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
        hb_millis = millis();
        break;

      case MAVLINK_MSG_ID_GPS_RAW_INT:                                         // #24
        ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
        ap_sat_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg); // number of visible satelites
        ap_gps_status = (ap_sat_visible * 10) + ap_fixtype;
        if (ap_fixtype > 2)
        {
          ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
          ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
          ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg); // 1m =1000
          ap_eph = mavlink_msg_gps_raw_int_get_eph(&msg);    // GPS HDOP
          ap_epv = mavlink_msg_gps_raw_int_get_epv(&msg);    // GPS VDOP
          ap_vel = mavlink_msg_gps_raw_int_get_vel(&msg);    // GPS ground speed (m/s * 100)
          ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg);    // Course over ground (NOT heading) in degrees * 100
        }
        break;

      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:                              // #33
        ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
        ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
        ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // x Supposedly altitude above mean sea level (millimeters)
        ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
        ap_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
        ap_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
        ap_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
        ap_hdg = mavlink_msg_global_position_int_get_hdg(&msg);             // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees          ap_ap_amsl = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
        break;
      }
    }
  }
}
void Send_To_FC(uint32_t msg_id)
{

  len = mavlink_msg_to_send_buffer(sendbuf, &sendmsg);
  Serial.write(sendbuf, len);
}
void RequestDataStreams()
{ //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD

  apo_sysid = 255;  // Reply to APM FC
  apo_compid = 190; // Reply to APM FC
  apo_targsys = 1;  // FC
  apo_targcomp = 1; // FC

  const int maxStreams = 7;
  const uint8_t mavStreams[] = {
      MAV_DATA_STREAM_RAW_SENSORS,
      MAV_DATA_STREAM_EXTENDED_STATUS,
      MAV_DATA_STREAM_RC_CHANNELS,
      MAV_DATA_STREAM_POSITION,
      MAV_DATA_STREAM_EXTRA1,
      MAV_DATA_STREAM_EXTRA2,
      MAV_DATA_STREAM_EXTRA3};

  const uint16_t mavRates[] = {0x04, 0x0a, 0x04, 0x0a, 0x04, 0x04, 0x04};
  // req_message_rate The requested interval between two messages of this type

  for (int i = 0; i < maxStreams; i++)
  {
    mavlink_msg_request_data_stream_pack(apo_sysid, apo_compid, &sendmsg, apo_targsys, apo_targcomp, mavStreams[i], mavRates[i], 1); // start_stop 1 to start sending, 0 to stop sending
    Send_To_FC(66);
  }
}
