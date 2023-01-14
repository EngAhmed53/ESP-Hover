#include "MegunoLink.h"
#include "Filter.h"
#include "Arduino.h"

class MotorSpeedCallback
{
public:
    virtual void onThrustChange(int top_motor_thrust, int bottom_motor_thrust, int right_motor_thrust, int left_motor_thrust);
};

class MotorManagerPlus
{
    
    ExponentialFilter<int> top_motor_thrust_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> bottom_motor_thrust_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> right_motor_thrust_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> left_motor_thrust_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> initial_thrust_filter = ExponentialFilter<int>(80, 0);

    MotorSpeedCallback *motorCallback;

    int initial_thrust, top_motor_thrust, bottom_motor_thrust, right_motor_thrust, left_motor_thrust;

public:
    void begin(MotorSpeedCallback *motorCallback)
    {
        this->motorCallback = motorCallback;
    }

    void set_initial_speed(int initial_speed)
    {
        if (initial_speed == 0)
        {
            initial_thrust_filter.SetCurrent(0);
        }

        initial_thrust_filter.Filter(initial_speed);

        this->initial_thrust = initial_thrust_filter.Current();
    }

    void updat_speeds(float pitch_pid, float roll_pid, float yaw_pid)
    {
        if (initial_thrust > 0)
        {
            update_top_motor_thrust(initial_thrust - pitch_pid + yaw_pid);
            update_bottom_motor_thrust(initial_thrust + pitch_pid + yaw_pid);
            update_right_motor_thrust(initial_thrust - roll_pid - yaw_pid);
            update_left_motor_thrust(initial_thrust + roll_pid - yaw_pid);
        }
        else
        {
            stop_all_motors();
        }

        this->motorCallback->onThrustChange(top_motor_thrust, bottom_motor_thrust, right_motor_thrust, left_motor_thrust);
    }

private:
    void update_top_motor_thrust(int speed)
    {
        speed = constrain(speed, 0, 210);
        top_motor_thrust_filter.Filter(speed);
        top_motor_thrust = top_motor_thrust_filter.Current();
    }

    void update_bottom_motor_thrust(int speed)
    {
        speed = constrain(speed, 0, 210);
        bottom_motor_thrust_filter.Filter(speed);
        bottom_motor_thrust = bottom_motor_thrust_filter.Current();
    }

    void update_right_motor_thrust(int speed)
    {
        speed = constrain(speed, 0, 210);
        right_motor_thrust_filter.Filter(speed);
        right_motor_thrust = right_motor_thrust_filter.Current();
    }

    void update_left_motor_thrust(int speed)
    {
        speed = constrain(speed, 0, 210);
        left_motor_thrust_filter.Filter(speed);
        left_motor_thrust = left_motor_thrust_filter.Current();
    }
    
    void stop_all_motors()
    {
        top_motor_thrust = 0;
        bottom_motor_thrust = 0;
        right_motor_thrust = 0;
        left_motor_thrust = 0;
    }
};