#pragma once
#include <Arduino.h>
#include <mavlink_types.h>
#include <common/mavlink.h>

// Message #0  HEARTHBEAT
extern uint8_t ap_type;
extern uint8_t ap_autopilot;
extern uint8_t ap_base_mode;
extern uint32_t ap_custom_mode;
extern uint8_t ap_system_status;
extern uint8_t ap_mavlink_version;

// Message #1  SYS_STATUS
extern uint16_t voltage_battery;
extern int16_t current_battery;
extern int8_t battery_remaining;
extern uint16_t drop_rate_comm;

// Message #24  GPS_RAW_INT
extern uint8_t ap_fixtype;
extern uint8_t ap_sat_visible;

// Message #33 GLOBAL_POSITION_INT
extern int32_t ap_lat;
extern int32_t ap_lon;
extern int32_t ap_alt_ag;

void send_to_FC(uint32_t msg_id);
void request_data_streams();
void MAVLink_receive();