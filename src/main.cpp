#include <Arduino.h>
#include "FlightController/IMUManager.cpp"
#include <SoftWire.h>
//#include <analogWrite.h>
#include "FlightController/ConnectionManager.cpp"
#include "FlightController/PIDManager.cpp"
#include "FlightController/MotorManagerPlus.cpp"
#include "MegunoLink.h"
#include "Filter.h"
// #include "FlightController/EspConnectionManager.h"
// #include <sstream>

#define SPEED_RATE 3
#define Serial Serial2

#define TOP_MOTOR_PIN 32
#define TOP_MOTOE_CHANNEL 2

#define BOTTOM_MOTOR_PIN 18
#define BOTTOM_MOTOR_CHANNEL 0

#define RIGHT_MOTOR_PIN 19
#define RIGHT_MOTOR_CHANNEL 3

#define LEFT_MOTOR_PIN 4
#define LEFT_MOTOR_CHANNEL 1

ExponentialFilter<double> current_pitch_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> current_roll_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroZ_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroX_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroY_filter = ExponentialFilter<double>(80, 0);

IMUManager imu;
PIDManager mPID;
MotorManagerPlus motorManager;
ConnectionManager connectionManager;

int initial_speed = 0;

int previous_millis = 0;

bool isConnected = false;
bool did_mesuare_offset = false;

bool shouldLog = false;

double setPoints[3] = {0, 0, 0};
double current_angles[2] = {0, 0};
double current_rates[3] = {0, 0, 0};

double pitch_gains[3] = {0.46, 0.9, 0.35};
double roll_gains[3] = {0.42, 0.85, 0.35};
double yaw_gains[2] = {0.45, 0.9};

double pitch_offset = 0;
double roll_offset = 0;
double yaw_rate_offset = 0;
double pitch_rate_offset = 0;
double roll_rate_offset = 0;

// struct_attiude_message attiude;

class DronPidCallback : public PIDCallback
{
  void onPIDChange(
      double pitch_p, double pitch_i, double pitch_d,
      double roll_p, double roll_i, double roll_d,
      double yaw_p, double yaw_i, double yaw_d)
  {

    double pitch_pid = pitch_p + pitch_i - pitch_d;
    double roll_pid = roll_p + roll_i - roll_d;
    double yaw_pid = yaw_p + yaw_i - yaw_d;

    // check values range
    pitch_pid = pitch_pid > MAXIMUM_OUTPUT ? MAXIMUM_OUTPUT : pitch_pid;
    roll_pid = roll_pid > MAXIMUM_OUTPUT ? MAXIMUM_OUTPUT : roll_pid;
    yaw_pid = yaw_pid > MAXIMUM_OUTPUT ? MAXIMUM_OUTPUT : yaw_pid;

    pitch_pid = pitch_pid < MINIMUM_OUTPUT ? MINIMUM_OUTPUT : pitch_pid;
    roll_pid = roll_pid < MINIMUM_OUTPUT ? MINIMUM_OUTPUT : roll_pid;
    yaw_pid = yaw_pid < MINIMUM_OUTPUT ? MINIMUM_OUTPUT : yaw_pid;

    motorManager.updat_speeds(pitch_pid, roll_pid, yaw_pid);
  }
};

class MotorCallback : public MotorSpeedCallback
{
  void onThrustChange(int top_motor_thrust, int bottom_motor_thrust, int right_motor_thrust, int left_motor_thrust)
  {
    // if (shouldLog)
    // {
    //   attiude.top_motor_thrust = top_motor_thrust;
    //   attiude.bottom_motor_thrust = bottom_motor_thrust;
    //   attiude.left_motor_thrust = left_motor_thrust;
    //   attiude.right_motor_thrust = right_motor_thrust;
    // }

    ledcWrite(LEFT_MOTOR_CHANNEL, left_motor_thrust);
    ledcWrite(RIGHT_MOTOR_CHANNEL, right_motor_thrust);
    ledcWrite(BOTTOM_MOTOR_CHANNEL, bottom_motor_thrust);
    ledcWrite(TOP_MOTOE_CHANNEL, top_motor_thrust);

    //  if (shouldLog)
    //     {
    //       Serial.print(top_motor_thrust);
    //       Serial.print(",");
    //       Serial.print(bottom_motor_thrust);
    //       Serial.print(",");
    //       Serial.print(right_motor_thrust);
    //       Serial.print(",");
    //       Serial.println(left_motor_thrust);
    //     }
    // if (esp_now_send_attiude_msg(attiude))
    // {
    //  // Serial.println("Attidue sent");
    // }
    // else
    // {
    //   Serial.println("Attidue send failed");
    // }
  }
};

class DroneConnectionCallback : public ConnectionStateCallback
{
  void onConnectionStateChange(ConnectionState state)
  {
    isConnected = state == CONNECTED;
    if (isConnected)
    {
      Serial.println("Phone is Connected");
    }
    else
    {
      Serial.println("Phone is Disconnected");
    }
  }
};

class DroneControlCallback : public DroneAttitudeCallback
{
  void onAttitudeChange(unsigned int thrust, unsigned int pitch_setpoint, unsigned int roll_setpoint, unsigned int yaw_setpoint)
  {
    initial_speed = thrust;
    setPoints[0] = static_cast<double>(pitch_setpoint);
    setPoints[1] = static_cast<double>(roll_setpoint);
    setPoints[2] = static_cast<double>(yaw_setpoint);
    // Serial2.print("thrust = ");
    //  Serial2.println(initial_speed);
    vTaskDelay(1);

    // double pitch_setpoint_as_angle = setpointToAngle(setPoints[0]);
    // double roll_setpoint_as_angle = setpointToAngle(setPoints[1]);

    // Serial2.print("pitch_setpint_as_angle = ");
    // Serial2.println(pitch_setpoint_as_angle, 2);

    // Serial2.print("roll_setpint_as_angle = ");
    // Serial2.println(roll_setpoint_as_angle, 2);
  }
};

void sendGeneralMsg(String msg)
{
  if (isConnected)
  {
    connectionManager.write20ByteMsg(std::string(msg.c_str()));
    // esp_now_send_str_msg(msg.c_str());
  }
};

class NewGainCallback : public GainCallback
{
  void onNewGain(double pitch_kp, double pitch_ki, double pitch_kd,
                 double roll_kp, double roll_ki, double roll_kd,
                 double yaw_kp, double yaw_ki, double yaw_kd)
  {
    // if (shouldLog)
    // {
    //   String pitch_gains_str = "New pitch gains = " + String(pitch_kp, 2) + "," + String(pitch_ki, 3) + "," + String(pitch_kd, 3);
    //   String roll_gains_str = "New roll gains = " + String(roll_kp, 3) + "," + String(roll_ki, 3) + "," + String(roll_kd, 3);
    //   String yaw_gains_str = "New yaw gains = " + String(yaw_kp, 2) + " " + String(yaw_ki, 2) + " " + String(yaw_kd, 2);

    //   Serial.println(pitch_gains_str);
    //   Serial.println(roll_gains_str);
    //   Serial.println(yaw_gains_str);
    // }

    pitch_gains[0] = pitch_kp;
    pitch_gains[1] = pitch_ki;
    pitch_gains[2] = pitch_kd;

    roll_gains[0] = roll_kp;
    roll_gains[1] = roll_ki;
    roll_gains[2] = roll_kd;

    yaw_gains[0] = yaw_kp;
    yaw_gains[1] = yaw_ki;

    mPID.reset(pitch_gains, roll_gains, yaw_gains);
  }
};

void mesuareOffsets()
{
  int counter = 0;
  int sumPitch = 0;
  int sumRoll = 0;
  int sumYaw = 0;
  int sumPitchRate = 0;
  int sumRollRate = 0;

  while (counter <= 1049)
  {
    if (imu.imu_loop())
    {
      if (counter >= 999)
      {
        sumPitch += imu.getPitch();
        sumRoll += imu.getRoll();
        sumYaw += imu.getGyroZ();
        sumPitchRate += imu.getGyroY();
        sumRollRate += imu.getGyroX();
      }
      counter++;
      delay(5);
    }
  }

  pitch_offset = static_cast<double>(sumPitch / 50.0);
  roll_offset = static_cast<double>(sumRoll / 50.0);
  yaw_rate_offset = static_cast<double>(sumYaw / 50.0);
  pitch_rate_offset = static_cast<double>(sumPitchRate / 50.0);
  roll_rate_offset = static_cast<double>(sumRollRate / 50.0);

  // if (shouldLog)
  // {
  //   Serial.print("Pitch offs = ");
  //   Serial.print(pitch_offset);

  //   Serial.print("Roll offs = ");
  //   Serial.print(roll_offset);
  // }
}

void initServo()
{
  ledcSetup(TOP_MOTOE_CHANNEL, 500, 8);    // channel 0, 500 hz PWM, 8-bit resolution
  ledcSetup(BOTTOM_MOTOR_CHANNEL, 500, 8); // channel 1, 500 hz PWM, 8-bit resolution
  ledcSetup(RIGHT_MOTOR_CHANNEL, 500, 8);  // channel 2, 500 hz PWM, 8-bit resolution
  ledcSetup(LEFT_MOTOR_CHANNEL, 500, 8);   // channel 3, 500 hz PWM, 8-bit resolution
  ledcAttachPin(TOP_MOTOR_PIN, TOP_MOTOE_CHANNEL);
  ledcAttachPin(BOTTOM_MOTOR_PIN, BOTTOM_MOTOR_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_PIN, RIGHT_MOTOR_CHANNEL);
  ledcAttachPin(LEFT_MOTOR_PIN, LEFT_MOTOR_CHANNEL);
}

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(TOP_MOTOR_PIN, OUTPUT);
  pinMode(BOTTOM_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  // if (shouldLog)
  // {
  //   Serial2.begin(9600);
  // }
  connectionManager.begin(new DroneConnectionCallback(), new DroneControlCallback(), new NewGainCallback());
  // begin_esp_now(new DroneConnectionCallback(), new DroneControlCallback(), new NewGainCallback());
  mPID.begin(new DronPidCallback());
  motorManager.begin(new MotorCallback());
  mPID.reset(pitch_gains, roll_gains, yaw_gains);

  initServo();

  while (!isConnected)
  {
    Serial.println("Waiting for connection");
    delay(2000);
  }

  imu.imuSetup();

  Serial.println("All set!");
}

void calca_attitude()
{
  current_pitch_filter.Filter(static_cast<double>(imu.getPitch()) - pitch_offset);
  current_roll_filter.Filter(static_cast<double>(imu.getRoll()) - roll_offset);

  current_angles[0] = current_pitch_filter.Current();
  current_angles[1] = current_roll_filter.Current();

  gyroZ_filter.Filter(static_cast<double>(imu.getGyroZ()) - yaw_rate_offset);
  gyroY_filter.Filter(static_cast<double>(imu.getGyroY() - pitch_rate_offset));
  gyroX_filter.Filter(static_cast<double>(imu.getGyroX() - roll_rate_offset));

  current_rates[0] = gyroY_filter.Current(); // pitch_rate
  current_rates[1] = gyroX_filter.Current(); // roll_rate
  current_rates[2] = gyroZ_filter.Current(); // yaw_rate

  // if (shouldLog)
  // {
  //   Serial.print(current_angles[0]);
  //   Serial.print(",");
  //   Serial.print(current_angles[1]);
  //   Serial.print(",");
  //   Serial.print(current_rates[0]);
  //   Serial.print(",");
  //   Serial.print(current_rates[1]);
  //   Serial.print(",");
  //   Serial.print(current_rates[2]);
  //   Serial.print(",");
  // }
}

void loop()
{
  if (!did_mesuare_offset)
  {
    mesuareOffsets();
    Serial.println("Offset calc done!");
    delay(1000);
    Serial.println("Offsets: P = " + String(pitch_offset, 2) + ",R = " + String(roll_offset, 2) + ",y = " + String(yaw_rate_offset, 2));
    did_mesuare_offset = true;
  }

  digitalWrite(2, isConnected ? HIGH : LOW);

  if (!isConnected)
  {
    initial_speed = 0;
  }

  if ((millis() - previous_millis) >= SPEED_RATE)
  {
    if (imu.imu_loop())
    {
      calca_attitude();
    }

    motorManager.set_initial_speed(initial_speed);

    if (initial_speed > 0)
    {
      mPID.setSampleTime(static_cast<double>(millis() - previous_millis) / 1000.0);
      mPID.setSetPoints(setPoints);
      mPID.setAngles(current_angles);
      mPID.setRates(current_rates);
      mPID.compute();
    }
    else
    {
      ledcWrite(TOP_MOTOE_CHANNEL, 0);
      ledcWrite(BOTTOM_MOTOR_CHANNEL, 0);
      ledcWrite(RIGHT_MOTOR_CHANNEL, 0);
      ledcWrite(LEFT_MOTOR_CHANNEL, 0);
    }

    previous_millis = millis();
  }
}
