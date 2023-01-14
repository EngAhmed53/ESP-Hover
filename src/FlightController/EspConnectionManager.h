#include "FlightController/ConnectionManager.cpp"
#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_str_message
{
    char a[128];
} struct_str_message;

typedef struct struct_attiude_message
{
    double pitch_angle;
    double roll_angle;
    double pitch_rate;
    double roll_rate;
    double yaw_rate;
    double pitch_p;
    double pitch_i;
    double pitch_d;
    double roll_p;
    double roll_i;
    double roll_d;
    double yaw_p;
    double yaw_i;
    double pitch_pid;
    double roll_pid;
    double yaw_pid;
    int32_t top_motor_thrust;
    int32_t bottom_motor_thrust;
    int32_t left_motor_thrust;
    int32_t right_motor_thrust;
} struct_attiude_message;

typedef struct struct_receive_message
{
    int thrust;
    double pitch_p;
    double pitch_i;
    double pitch_d;
    double roll_p;
    double roll_i;
    double roll_d;
    double yaw_p;
    double yaw_i;
    double yaw_d;
    bool should_reset_pid;
} struct_receive_message;

void begin_esp_now(ConnectionStateCallback *connectionStateCallback, DroneAttitudeCallback *droneAttitudeCallback, GainCallback *gainCallbak);

bool esp_now_send_str_msg(const char *msg);

bool esp_now_send_attiude_msg(struct_attiude_message attidue);

