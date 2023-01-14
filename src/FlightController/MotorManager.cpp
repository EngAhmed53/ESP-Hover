#include "MegunoLink.h"
#include "Filter.h"

class MotorSpeedCallback
{
public:
    virtual void onSpeedchange(int tlm_speed, int trm_speed, int blm_speed, int brm_speed);
};

class MotorManager
{
    /*
    tlm = Top Left Motor
    trm = Top Right Motor
    blm = Bottom Left Motor
    brm = Bottom Right Motor
    */
    ExponentialFilter<int> tlm_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> trm_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> blm_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> brm_filter = ExponentialFilter<int>(80, 0);
    ExponentialFilter<int> initial_thrust_filter = ExponentialFilter<int>(20, 0);

    MotorSpeedCallback *motorCallback;

    int initial_speed, tlm_speed, trm_speed, blm_speed, brm_speed;

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

        this->initial_speed = initial_thrust_filter.Current();
    }

    void updat_speeds(double pitch_pid, double roll_pid, double yaw_pid)
    {
        if (initial_speed > 0)
        {
            update_tlm_speed(initial_speed + pitch_pid - roll_pid - yaw_pid);
            update_trm_speed(initial_speed + pitch_pid + roll_pid + yaw_pid);
            update_blm_speed(initial_speed - pitch_pid - roll_pid + yaw_pid);
            update_brm_speed(initial_speed - pitch_pid + roll_pid - yaw_pid);
        }
        else
        {
            stop_all_motors();
        }

        this->motorCallback->onSpeedchange(tlm_speed, trm_speed, blm_speed, brm_speed);
    }

private:
    void update_tlm_speed(int speed)
    {
        if (speed > 255)
        {
            speed = 255;
        }

        if (speed < 0)
        {
            speed = 0;
        }

        tlm_filter.Filter(speed);
        tlm_speed = tlm_filter.Current();
    }

    void update_trm_speed(int speed)
    {
        if (speed > 255)
        {
            speed = 255;
        }

        if (speed < 0)
        {
            speed = 0;
        }
        trm_filter.Filter(speed);
        trm_speed = trm_filter.Current();
    }

    void update_blm_speed(int speed)
    {
        if (speed > 255)
        {
            speed = 255;
        }

        if (speed < 0)
        {
            speed = 0;
        }
        blm_filter.Filter(speed);
        blm_speed = blm_filter.Current();
    }

    void update_brm_speed(int speed)
    {
        if (speed > 255)
        {
            speed = 255;
        }

        if (speed < 0)
        {
            speed = 0;
        }
        brm_filter.Filter(speed);
        brm_speed = brm_filter.Current();
    }
   
    void stop_all_motors()
    {
        tlm_speed = 0;
        trm_speed = 0;
        blm_speed = 0;
        brm_speed = 0;
    }
};