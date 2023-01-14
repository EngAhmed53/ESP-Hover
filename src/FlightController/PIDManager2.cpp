#include <PID_v1.h>

class PIDCallback
{
public:
    virtual void onPIDChange(double pitch_pid, double roll_pid, double yaw_pid);
};

class PIDManagerNEW
{
    double pitch_error, yaw_error, roll_error;
    double pitch_setpoint, roll_setpoint, yaw_setpoint;
    double pitch_pid, yaw_pid, roll_pid;

    PID pitchPID = PID(&pitch_error, &pitch_pid, &pitch_setpoint, 0, 0, 0, REVERSE);
    PID rollPID = PID(&roll_error, &roll_pid, &roll_setpoint, 0, 0, 0, REVERSE);
    PID yawPID = PID(&yaw_error, &yaw_pid, &yaw_setpoint, 0, 0, 0, REVERSE);

    PIDCallback *pidCallback;

public:
    void begin(PIDCallback *pidCallback)
    {
        pitch_setpoint = 0;
        roll_setpoint = 0;
        yaw_setpoint = 0;

        pitchPID.SetMode(AUTOMATIC);
        pitchPID.SetSampleTime(3);
        pitchPID.SetOutputLimits(-255, 255);

        rollPID.SetMode(AUTOMATIC);
        rollPID.SetSampleTime(3);
        rollPID.SetOutputLimits(-255, 255);

        yawPID.SetMode(AUTOMATIC);
        yawPID.SetSampleTime(3);
        yawPID.SetOutputLimits(-255, 255);

        this->pidCallback = pidCallback;
    }

    void reset(double pitch_kp, double pitch_ki, double pitch_kd,
               double roll_kp, double roll_ki, double roll_kd,
               double yaw_kp, double yaw_ki, double yaw_kd)
    {
        pitchPID.SetTunings(pitch_kp, pitch_ki, pitch_kd);
        rollPID.SetTunings(roll_kp, roll_ki, roll_kd);
        yawPID.SetTunings(yaw_kp, yaw_ki, yaw_kd);
    }

    void compute(double pitch_error, double roll_error, double yaw_error, float elapsed_time)
    {
        this->pitch_error = pitch_error;
        this->roll_error = roll_error;
        this->yaw_error = yaw_error;
        pitchPID.Compute();
        rollPID.Compute();
        yawPID.Compute();
        this->pidCallback->onPIDChange(pitch_pid, roll_pid, yaw_pid);
    }
};