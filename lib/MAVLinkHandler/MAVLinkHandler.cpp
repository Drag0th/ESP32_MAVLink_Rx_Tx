#include <MAVLinkHandler.h>

mavlink_message_t sendmsg;

uint8_t sendbuf[128];
uint8_t len;

uint8_t apo_sysid;
uint8_t apo_compid;
uint8_t apo_targsys;
uint8_t apo_targcomp;
const uint16_t mavRates = 0x01;

mavlink_message_t msg;
mavlink_status_t status;

// Message #0  HEARTHBEAT
uint8_t ap_type = 0;
uint8_t ap_autopilot = 0;
uint8_t ap_base_mode = 0;
uint32_t ap_custom_mode = 0;
uint8_t ap_system_status = 0;
uint8_t ap_mavlink_version = 0;

// Message #1  SYS_STATUS
uint16_t voltage_battery;
int16_t current_battery;
int8_t battery_remaining;
uint16_t drop_rate_comm;

// Message #24  GPS_RAW_INT
uint8_t ap_fixtype = 0;
uint8_t ap_sat_visible = 0;

// Message #33 GLOBAL_POSITION_INT
int32_t ap_lat;
int32_t ap_lon;
int32_t ap_alt_ag;

void send_to_FC(uint32_t msg_id)
{

    len = mavlink_msg_to_send_buffer(sendbuf, &sendmsg);
    Serial.write(sendbuf, len);
}

void request_data_streams() //  REQUEST_DATA_STREAM (#66)
{

    apo_sysid = 255;  // Ground Station system ID
    apo_compid = 1;   // Ground Station component ID
    apo_targsys = 1;  // Flight Controller system ID
    apo_targcomp = 1; // Flight Controller component ID

    const int maxStreams = 4;
    const uint8_t mavStreams[] = {
        MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION};

    for (int i = 0; i < maxStreams; i++)
    {
        mavlink_msg_request_data_stream_pack(apo_sysid, apo_compid, &sendmsg, apo_targsys, apo_targcomp, mavStreams[i], mavRates, 1);
        send_to_FC(MAVLINK_MSG_ID_REQUEST_DATA_STREAM);
    }
}

void MAVLink_receive(BluetoothSerial &SerialBT)
{

    while (Serial.available())
    {
        uint8_t c = Serial.read();
        SerialBT.write(c);
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            switch (msg.msgid)
            {

            case MAVLINK_MSG_ID_HEARTBEAT:
                ap_type = mavlink_msg_heartbeat_get_type(&msg);
                ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
                ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
                ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
                break;

            case MAVLINK_MSG_ID_SYS_STATUS:
                voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&msg);
                current_battery = mavlink_msg_sys_status_get_current_battery(&msg);
                battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
                drop_rate_comm = mavlink_msg_sys_status_get_drop_rate_comm(&msg);
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                ap_sat_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                break;

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                ap_lat = mavlink_msg_global_position_int_get_lat(&msg);
                ap_lon = mavlink_msg_global_position_int_get_lon(&msg);
                ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg);
            }
        }
    }
}