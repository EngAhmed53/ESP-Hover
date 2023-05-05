#include <Arduino.h>
#include "../include/model.h"
#include "../include/BLEManager.h"
#include "../include/PID.h"
#include "../include/FlightRecorder.h"
#include "../include/Log.h"
#include "../include/PlusQuadThrustManager.h"
#include "IMUManager.cpp"
#include "Filter.h"

//#define DebugMode
//#define SerialDebugMode

#define MOTOR_HIGH_BOUNDARY 210.0
#define MOTOR_LOW_BOUNDARY (-210.0)

#define LED_PIN 2

#define MOTOR_A_PIN 32
#define MOTOR_A_CHANNEL 2

#define MOTOR_B_PIN 18
#define MOTOR_B_CHANNEL 0

#define MOTOR_C_PIN 19
#define MOTOR_C_CHANNEL 3

#define MOTOR_D_PIN 4
#define MOTOR_D_CHANNEL 1


ExponentialFilter<double> pitch_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> roll_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroZ_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroX_filter = ExponentialFilter<double>(80, 0);
ExponentialFilter<double> gyroY_filter = ExponentialFilter<double>(80, 0);

IMUManager imu;

std::shared_ptr<BLEManager> bleManager = std::make_shared<BLEManager>();

PID pitchPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
PID rollPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
PID yawPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);

PlusQuadThrustManager thrustManager(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY, 80);

FlightRecorder flightRecorder([](std::vector<uint8_t> &&buffer) {
    bleManager->writeRecords(std::move(buffer));
});

FlightRecord record;

unsigned long previous_millis = 0;

bool isConnected = false;
bool isArmed = false;

double current_angles[2] = {0, 0};
double current_rates[3] = {0, 0, 0};

double pitch_offset = 0;
double roll_offset = 0;
double yaw_rate_offset = 0;
double pitch_rate_offset = 0;
double roll_rate_offset = 0;

SemaphoreHandle_t xMutex{xSemaphoreCreateMutex()};
QueueHandle_t attitudeQueueHandle = xQueueCreate(100, sizeof(AttitudeModel));
QueueHandle_t gainsQueueHandle = xQueueCreate(1, sizeof(AttitudeModel));
TickType_t sendWaitTime = pdMS_TO_TICKS(10);
TickType_t receiveWaitTime = pdMS_TO_TICKS(1);

void onConnectionChangeCallback(ConnectionState state) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    isConnected = state == ConnectionState::CONNECTED;
    xSemaphoreGive(xMutex);
    digitalWrite(LED_PIN, isConnected ? HIGH : LOW);
}

double setPointToAngle(int setPoint) {
    return (setPoint - 0.0) * (20.0 - (-20.0)) / (255 - 0) + (-20.0);
}

double setPointToYawRate(int setPoint) {
    if (setPoint == 255)
        return -65.0;
    else if (setPoint == 127)
        return 65.0;
    else
        return 0.0;
}

void onAttitudeChangeCallback(int thrust, int pitch, int roll, int yaw) {
    AttitudeModel model = AttitudeModel{
            .thrust = thrust,
            .pitchSetPoint = setPointToAngle(pitch),
            .rollSetPoint = setPointToAngle(roll) * -1,
            .yawSetPoint = setPointToYawRate(yaw)
    };

    if (xQueueSend(attitudeQueueHandle, &model, sendWaitTime) == pdPASS) {
#ifdef DebugMode
        Log::d("Enqueue new attitude success");
#endif
    } else {
#ifdef DebugMode
        Log::d("Enqueue new attitude failed");
#endif
    }

#ifdef DebugMode
    Log::d("thrust = " + std::to_string(thrust));
    Log::d("pitch = " + std::to_string(pitch));
    Log::d("roll = " + std::to_string(roll));
    Log::d("yaw = " + std::to_string(yaw));
#endif
}

void onNewGainCallback(const std::vector<double> &&gains) {
    xSemaphoreTake(xMutex, portMAX_DELAY);

    pitchPID.Gains(gains[0], gains[1], gains[2]);
    rollPID.Gains(gains[3], gains[4], gains[5]);
    yawPID.Gains(gains[5], gains[7], gains[8]);

    xSemaphoreGive(xMutex);
}

void measureOffsets() {
    int counter = 0;
    int sumPitch = 0;
    int sumRoll = 0;
    int sumYaw = 0;
    int sumPitchRate = 0;
    int sumRollRate = 0;

    while (counter <= 1049) {
        if (imu.imu_loop()) {
            if (counter >= 999) {
                sumPitch += static_cast<int>(imu.getPitch());
                sumRoll += static_cast<int>(imu.getRoll());
                sumYaw += static_cast<int>(imu.getGyroZ());
                sumPitchRate += static_cast<int>(imu.getGyroY());
                sumRollRate += static_cast<int>(imu.getGyroX());
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
}

void setupPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    pinMode(MOTOR_C_PIN, OUTPUT);
    pinMode(MOTOR_D_PIN, OUTPUT);

    ledcSetup(MOTOR_A_CHANNEL, 500, 8); // channel 0, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_B_CHANNEL, 500, 8); // channel 1, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_C_CHANNEL, 500, 8); // channel 2, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_D_CHANNEL, 500, 8); // channel 3, 500 hz PWM, 8-bit resolution

    ledcAttachPin(MOTOR_A_PIN, MOTOR_A_CHANNEL);
    ledcAttachPin(MOTOR_B_PIN, MOTOR_B_CHANNEL);
    ledcAttachPin(MOTOR_C_PIN, MOTOR_C_CHANNEL);
    ledcAttachPin(MOTOR_D_PIN, MOTOR_D_CHANNEL);
}

void calculateAttitude() {
    imu.imu_loop();

    pitch_filter.Filter(static_cast<double>(imu.getPitch()) - pitch_offset);
    roll_filter.Filter(static_cast<double>(imu.getRoll()) - roll_offset);

    current_angles[0] = pitch_filter.Current();
    current_angles[1] = roll_filter.Current();

    gyroZ_filter.Filter(static_cast<double>(imu.getGyroZ()) - yaw_rate_offset);
    gyroY_filter.Filter(static_cast<double>(imu.getGyroY() - pitch_rate_offset));
    gyroX_filter.Filter(static_cast<double>(imu.getGyroX() - roll_rate_offset));

    current_rates[0] = gyroY_filter.Current(); // pitch_rate
    current_rates[1] = gyroX_filter.Current(); // roll_rate
    current_rates[2] = gyroZ_filter.Current(); // yaw_rate

#if defined(DebugMode) || defined(SerialDebugMode)
    Log::print(std::to_string(current_angles[0]) + ",");
    Log::print(std::to_string(current_angles[1]) + ",");
    Log::print(std::to_string(current_rates[0]) + ",");
    Log::print(std::to_string(current_rates[1]) + ",");
    Log::print(std::to_string(current_rates[2]) + ",");
#endif
}

void stopAllMotors() {
    ledcWrite(MOTOR_D_CHANNEL, 0);
    ledcWrite(MOTOR_C_CHANNEL, 0);
    ledcWrite(MOTOR_B_CHANNEL, 0);
    ledcWrite(MOTOR_A_CHANNEL, 0);
}

void setup() {
#if defined(DebugMode) || defined(SerialDebugMode)
    Serial.begin(9600);
#endif

    setupPins();

    bleManager->setConnectionCallback(onConnectionChangeCallback);
    bleManager->setAttitudeCallback(onAttitudeChangeCallback);
    bleManager->setGainsCallback(onNewGainCallback);

    bleManager->init();

    while (!isConnected) {
        Log::d("Waiting for connection");
        delay(1000);
    }

    imu.imuSetup();
    delay(1000);
    measureOffsets();

#if defined(DebugMode) || defined(SerialDebugMode)
    Log::d("Offset calc done!");
    std::string msg = "Offsets: Pitch = " + std::to_string(pitch_offset) + ", Roll = " + std::to_string(roll_offset) +
                      ", Yaw = " + std::to_string(yaw_rate_offset);
    Log::d(msg);
    Log::d("All set!");
#endif
}

void loop() {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    if (!isConnected) {
        xSemaphoreGive(xMutex);
        // Stop all motors, reset thrust filters and disarm if not connected
        stopAllMotors();
        thrustManager.resetMotorsFilters();
        isArmed = false;
        delay(100);
        return;
    }

    double sampleTime = static_cast<double>(millis() - previous_millis) / 1000.0;
    previous_millis = millis();

    AttitudeModel receivedModel{};
    if (xQueueReceive(attitudeQueueHandle, &receivedModel, receiveWaitTime) == pdPASS) {

#ifdef DebugMode
        Log::d("************************Received an attitude model************************");
#endif
        pitchPID.SetPoint(receivedModel.pitchSetPoint);
        rollPID.SetPoint(receivedModel.rollSetPoint);
        yawPID.SetPoint(receivedModel.yawSetPoint);
        thrustManager.updateBaseThrust(receivedModel.thrust);

        isArmed = receivedModel.thrust > 0;
    } else {
#ifdef DebugMode
        Log::d("Couldn't receive the attitude model in time");
#endif
    }

    if (!isArmed) {
        xSemaphoreGive(xMutex);
        stopAllMotors();
        thrustManager.resetMotorsFilters();
        delay(100);
        //flightRecorder.flushNow();
        return;
    }

    calculateAttitude();

    PIDModel pitchPIDModel = pitchPID.Compute(sampleTime, current_angles[0], current_rates[0]);
    PIDModel rollPIDModel = rollPID.Compute(sampleTime, current_angles[1], current_rates[1]);
    PIDModel yawPIDModel = yawPID.Compute(sampleTime, current_rates[2], 0);

    xSemaphoreGive(xMutex);

    double pitch_pid = pitchPIDModel.total();
    double roll_pid = rollPIDModel.total();
    double yaw_pid = yawPIDModel.total();

    pitch_pid = pitch_pid > MOTOR_HIGH_BOUNDARY ? MOTOR_HIGH_BOUNDARY : pitch_pid;
    roll_pid = roll_pid > MOTOR_HIGH_BOUNDARY ? MOTOR_HIGH_BOUNDARY : roll_pid;
    yaw_pid = yaw_pid > MOTOR_HIGH_BOUNDARY ? MOTOR_HIGH_BOUNDARY : yaw_pid;

    pitch_pid = pitch_pid < MOTOR_LOW_BOUNDARY ? MOTOR_LOW_BOUNDARY : pitch_pid;
    roll_pid = roll_pid < MOTOR_LOW_BOUNDARY ? MOTOR_LOW_BOUNDARY : roll_pid;
    yaw_pid = yaw_pid < MOTOR_LOW_BOUNDARY ? MOTOR_LOW_BOUNDARY : yaw_pid;

    ThrustModel thrustModel = thrustManager.calculateMotorsThrust(pitch_pid, roll_pid, yaw_pid);

    ledcWrite(MOTOR_D_CHANNEL, thrustModel.motor_d_thrust);
    ledcWrite(MOTOR_C_CHANNEL, thrustModel.motor_c_thrust);
    ledcWrite(MOTOR_B_CHANNEL, thrustModel.motor_b_thrust);
    ledcWrite(MOTOR_A_CHANNEL, thrustModel.motor_a_thrust);


#if defined(DebugMode) || defined(SerialDebugMode)
    Log::print(std::to_string(sampleTime) + ",");

    Log::print(std::to_string(pitchPIDModel.proportional) + ",");
    Log::print(std::to_string(pitchPIDModel.integral) + ",");
    Log::print(std::to_string(pitchPIDModel.derivative) + ",");
    Log::print(std::to_string(pitchPIDModel.total()) + ",");

    Log::print(std::to_string(rollPIDModel.proportional) + ",");
    Log::print(std::to_string(rollPIDModel.integral) + ",");
    Log::print(std::to_string(rollPIDModel.derivative) + ",");
    Log::print(std::to_string(rollPIDModel.total()) + ",");

    Log::print(std::to_string(yawPIDModel.proportional) + ",");
    Log::print(std::to_string(yawPIDModel.integral) + ",");
    Log::print(std::to_string(yawPIDModel.total()) + ",");

    Log::print(std::to_string(thrustModel.motor_a_thrust) + ",");
    Log::print(std::to_string(thrustModel.motor_b_thrust) + ",");
    Log::print(std::to_string(thrustModel.motor_c_thrust) + ",");
    Log::d(static_cast<double>(thrustModel.motor_d_thrust));
#endif

    record = FlightRecord{
            .millis = millis(),

            .pitchSetPoint = (uint8_t) pitchPID.SetPoint(),
            .rollSetPoint = (uint8_t) rollPID.SetPoint(),
            .yawSetPoint = (uint8_t) yawPID.SetPoint(),

            .altitude = (int16_t) thrustManager.baseThrust(),

            .pitch = static_cast<int16_t>(current_angles[0]),
            .roll = static_cast<int16_t>(current_angles[1]),
            .yawRate = static_cast<int16_t>(current_rates[2]),

            .pitch_p = static_cast<int16_t>(pitchPIDModel.proportional),
            .pitch_i = static_cast<int16_t>(pitchPIDModel.integral),
            .pitch_d = static_cast<int16_t>(pitchPIDModel.derivative),

            .roll_p = static_cast<int16_t>(rollPIDModel.proportional),
            .roll_i = static_cast<int16_t>(rollPIDModel.integral),
            .roll_d = static_cast<int16_t>(rollPIDModel.derivative),

            .yaw_p = static_cast<int16_t>(yawPIDModel.proportional),
            .yaw_i = static_cast<int16_t>(yawPIDModel.integral),
            .yaw_d = static_cast<int16_t>(yawPIDModel.derivative),

            .motor_a_thrust = (uint8_t) thrustModel.motor_a_thrust,
            .motor_b_thrust = (uint8_t) thrustModel.motor_b_thrust,
            .motor_c_thrust = (uint8_t) thrustModel.motor_c_thrust,
            .motor_d_thrust = (uint8_t) thrustModel.motor_d_thrust
    };


    // flightRecorder.insertAndFlushIfReady(record);
}
