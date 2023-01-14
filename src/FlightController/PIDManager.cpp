#include "MegunoLink.h"
#include "Filter.h"
#include <sstream>

#define MAXIMUM_OUTPUT 210.0
#define MINIMUM_OUTPUT -210.0
#define MAXIMUM_NEGLECTED_ANGLE_ERROR 2.5  // in degress
#define MINIMUM_NEGLECTED_ANGLE_ERROR -2.5 // in degress
#define MAXIMUM_NEGLECTED_YAW_RATE 1       // in degree/sec
#define MINIMUM_NEGLETED_YAW_RATE -1       // in degress/sec

class PIDCallback
{
public:
    virtual void onPIDChange(double pitch_p, double pitch_i, double pitch_d, double roll_p, double roll_i, double roll_d, double yaw_p, double yaw_i, double yaw_d);
};

class PIDManager
{
    double sample_time;

    double *setPoints;
    double *angles;
    double *rates;

    double *pitch_gains;
    double *roll_gains;
    double *yaw_gains;

    double accum_pitch_error, accum_roll_error, accum_yaw_error;

    double p_pitch, p_roll, p_yaw, d_pitch, d_roll, d_yaw, i_pitch, i_roll, i_yaw;

    PIDCallback *pidCallback;

public:
    void begin(PIDCallback *pidCallback)
    {
        this->pidCallback = pidCallback;
    }

    void reset(double pitch_gains[3],
               double roll_gains[3],
               double yaw_gains[2])
    {
        this->pitch_gains = pitch_gains;
        this->roll_gains = roll_gains;
        this->yaw_gains = yaw_gains;

        this->accum_pitch_error = 0;
        this->accum_roll_error = 0;
        this->accum_yaw_error = 0;

        this->p_pitch = 0;
        this->i_pitch = 0;
        this->d_pitch = 0;

        this->p_roll = 0;
        this->i_roll = 0;
        this->d_roll = 0;

        this->p_yaw = 0;
        this->i_yaw = 0;
        this->d_yaw = 0;
    }

    void setSampleTime(double sampleTime)
    {
        this->sample_time = sampleTime;
    }

    void setSetPoints(double setPoints[3])
    {
        this->setPoints = setPoints;
    }

    void setAngles(double angles[2])
    {
        this->angles = angles;
    }

    void setRates(double rates[3])
    {
        this->rates = rates;
    }

    void compute()
    {
        compute_yaw_pid();
        compute_pitch_pid();
        compute_roll_pid();

        this->pidCallback->onPIDChange(p_pitch, i_pitch, d_pitch, p_roll, i_roll, d_roll, p_yaw, i_yaw, d_yaw);
    }

private:
    double setpointToAngle(double setpoint)
    {
        return (setpoint - 0.0) * (20.0 - (-20.0)) / (255 - 0) + (-20.0);
    }

    double setpointToYawRate(double setpoint)
    {
        if (setpoint == 255.0)
            return -65.0;
        else if (setpoint == 127.0)
            return 65.0;
        else
            return 0.0;
    }

    void compute_pitch_pid()
    {
        double pitch_setpoint_as_angle = setpointToAngle(setPoints[0]);

        double error = pitch_setpoint_as_angle - angles[0];

        // prpo
        p_pitch = pitch_gains[0] * error;

        // integral
        if (error >= MAXIMUM_NEGLECTED_ANGLE_ERROR || error <= MINIMUM_NEGLECTED_ANGLE_ERROR)
        {
            double error_area = error * sample_time;
            accum_pitch_error += error_area;
            i_pitch = pitch_gains[1] * accum_pitch_error;
            if (i_pitch > MAXIMUM_OUTPUT)
            {
                i_pitch = MAXIMUM_OUTPUT;
                accum_pitch_error -= error_area;
            }
            else if (i_pitch < MINIMUM_OUTPUT)
            {
                i_pitch = MINIMUM_OUTPUT;
                accum_pitch_error -= error_area;
            }
        }

        // derivative
        d_pitch = pitch_gains[2] * rates[0];
    }

    void compute_roll_pid()
    {
        double roll_setpoint_as_angle = setpointToAngle(setPoints[1]) * -1;

        double error = roll_setpoint_as_angle - angles[1];

        // Prop
        p_roll = roll_gains[0] * error;

        // Integral
        if (error >= MAXIMUM_NEGLECTED_ANGLE_ERROR || error <= MINIMUM_NEGLECTED_ANGLE_ERROR)
        {
            double error_area = error * sample_time;
            accum_roll_error += error_area;
            i_roll = roll_gains[1] * accum_roll_error;

            if (i_roll > MAXIMUM_OUTPUT)
            {
                i_roll = MAXIMUM_OUTPUT;
                accum_roll_error -= error_area;
            }
            else if (i_roll < MINIMUM_OUTPUT)
            {
                i_roll = MINIMUM_OUTPUT;
                accum_roll_error -= error_area;
            }
        }

        // Derivative
        d_roll = roll_gains[2] * rates[1];
    }

    void compute_yaw_pid()
    {

        double yaw_setpoint_as_rate = setpointToYawRate(setPoints[2]);

        double error = yaw_setpoint_as_rate - rates[2];

        // Prop
        p_yaw = yaw_gains[0] * error;

        // Integral
        if (error >= MAXIMUM_NEGLECTED_YAW_RATE || error <= MINIMUM_NEGLETED_YAW_RATE)
        {
            double error_area = error * sample_time;
            accum_yaw_error += error_area;
            i_yaw = yaw_gains[1] * accum_yaw_error;
            if (i_yaw > MAXIMUM_OUTPUT)
            {
                i_yaw = MAXIMUM_OUTPUT;
                accum_yaw_error -= error_area;
            }
            else if (i_yaw < MINIMUM_OUTPUT)
            {
                i_yaw = MINIMUM_OUTPUT;
                accum_yaw_error -= error_area;
            }
        }
        d_yaw = 0;
    }
};