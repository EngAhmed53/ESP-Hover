#include <NimBLEDevice.h>
#include <Arduino.h>

enum ConnectionState
{
    CONNECTED,
    DISCONNECTED
};

class ConnectionStateCallback
{
public:
    virtual void onConnectionStateChange(ConnectionState state);
};

class GainCallback
{
public:
    virtual void onNewGain(double pitch_kp, double pitch_ki, double pitch_kd,
                           double roll_kp, double roll_ki, double roll_kd,
                           double yaw_kp, double yaw_ki, double yaw_kd);
};

class DroneAttitudeCallback
{
public:
    virtual void onAttitudeChange(unsigned int thrust, unsigned int pitch_setpoint, unsigned int roll_setpoint, unsigned int yaw_setpoint);
};

class ConnectionManager
{

#define DRONE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ATTITUDE_CHARACTERISTIC_UUID "fbf426ba-5f90-11ec-bf63-0242ac130002"
#define ANGLES_CHARACTERISTIC_UUID "450f2462-5f91-11ec-bf63-0242ac130002"
#define PID_CHARACTERISTIC_UUID "029d7bb4-5f92-11ec-bf63-0242ac130002"
#define MOTOR_SPEED_CHARACTERISTIC_UUID "9178300e-5ffb-11ec-bf63-0242ac130002"
#define GENERAL_MSG_CHARACTERISTIC_UUID "3e56610a-5ffb-11ec-bf63-0242ac130002"
#define NEW_GAIN_CHARACTERISTIC_UUID "ec743a01-5ffb-11ec-bf63-0242ac130002"

    DroneAttitudeCallback *droneAttitudeCallback;
    ConnectionStateCallback *connectionStateCallback;
    GainCallback *gainCallback;

    BLECharacteristic *pThrustCharacteristic, *pGainCharacteristic, *pAnglesCharacteristic, *pPIDsCharacteristic, *pMotorSpeedCharacteristic, *pGeneralMsgCharacteristic;

    class ServerCallback : public BLEServerCallbacks
    {
        ConnectionManager *connectionManager;

    public:
        ServerCallback(ConnectionManager *connectionManager)
        {
            this->connectionManager = connectionManager;
        }

        void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
        {
            pServer->updateConnParams(desc->conn_handle, 6, 8, 0, 60);
            this->connectionManager->connectionStateCallback->onConnectionStateChange(CONNECTED);
        }

        void onDisconnect(NimBLEServer *pServer)
        {
            this->connectionManager->connectionStateCallback->onConnectionStateChange(DISCONNECTED);
        }
    };

    class AttitudeCharCallbacks : public BLECharacteristicCallbacks
    {
        ConnectionManager *connectionManager;

    public:
        AttitudeCharCallbacks(ConnectionManager *connectionManager)
        {
            this->connectionManager = connectionManager;
        }

        void onWrite(BLECharacteristic *pCharacteristic)
        {
            std::string data = pCharacteristic->getValue();

            std::string thrust_hex = data.substr(0, 2);
            std::string pitch_setpoint_hex = data.substr(2, 2);
            std::string roll_setpoint_hex = data.substr(4, 2);
            std::string yaw_setpoint_hex = data.substr(6, 2);

            unsigned int thrust = strtol(thrust_hex.c_str(), nullptr, 16);
            unsigned int pitch_setpoint = strtol(pitch_setpoint_hex.c_str(), nullptr, 16);
            unsigned int roll_setpoint = strtol(roll_setpoint_hex.c_str(), nullptr, 16);
            unsigned int yaw_setpoint = strtol(yaw_setpoint_hex.c_str(), nullptr, 16);

            this->connectionManager->droneAttitudeCallback->onAttitudeChange(thrust, pitch_setpoint, roll_setpoint, yaw_setpoint);
        }
    };

    class NewGainCharCallbacks : public BLECharacteristicCallbacks
    {
        ConnectionManager *connectionManager;

    public:
        NewGainCharCallbacks(ConnectionManager *connectionManager)
        {
            this->connectionManager = connectionManager;
        }

        void onWrite(BLECharacteristic *pCharacteristic)
        {
            std::string data = pCharacteristic->getValue();

            std::string pitch_kp_hex = data.substr(0, 2);
            std::string pitch_ki_hex = data.substr(2, 2);
            std::string pitch_kd_hex = data.substr(4, 2);

            std::string roll_kp_hex = data.substr(6, 2);
            std::string roll_ki_hex = data.substr(8, 2);
            std::string roll_kd_hex = data.substr(10, 2);

            std::string yaw_kp_hex = data.substr(12, 2);
            std::string yaw_ki_hex = data.substr(14, 2);
            std::string yaw_kd_hex = data.substr(16, 2);

            std::string hold_kp_hex = data.substr(18, 2);
            std::string hold_ki_hex = data.substr(20, 2);
            std::string hold_kd_hex = data.substr(22, 2);

            int pitch_kp_raw = strtol(pitch_kp_hex.c_str(), nullptr, 16);
            int pitch_ki_raw = strtol(pitch_ki_hex.c_str(), nullptr, 16);
            int pitch_kd_raw = strtol(pitch_kd_hex.c_str(), nullptr, 16);

            int roll_kp_raw = strtol(roll_kp_hex.c_str(), nullptr, 16);
            int roll_ki_raw = strtol(roll_ki_hex.c_str(), nullptr, 16);
            int roll_kd_raw = strtol(roll_kd_hex.c_str(), nullptr, 16);

            int yaw_kp_raw = strtol(yaw_kp_hex.c_str(), nullptr, 16);
            int yaw_ki_raw = strtol(yaw_ki_hex.c_str(), nullptr, 16);
            int yaw_kd_raw = strtol(yaw_kd_hex.c_str(), nullptr, 16);

            int hold_kp_raw = strtol(hold_kp_hex.c_str(), nullptr, 16);
            int hold_ki_raw = strtol(hold_ki_hex.c_str(), nullptr, 16);
            int hold_kd_raw = strtol(hold_kd_hex.c_str(), nullptr, 16);

            this->connectionManager->gainCallback->onNewGain(pitch_kp_raw / 100.0, pitch_ki_raw / 100.0, pitch_kd_raw / 100.0,
                                                             roll_kp_raw / 100.0, roll_ki_raw / 100.0, roll_kd_raw / 100.0,
                                                             yaw_kp_raw / 100.0, yaw_ki_raw / 100.0, yaw_kd_raw / 100.0,
                                                             hold_kp_raw / 100.0, hold_ki_raw / 100.0, hold_kd_raw / 100.0);
        }
    };

    class AnglesCharCallbacks : public BLECharacteristicCallbacks
    {
        void onWrite(BLECharacteristic *pCharacteristic)
        {
            std::string value = pCharacteristic->getValue();
        }
    };

    class PIDsCharCallbacks : public BLECharacteristicCallbacks
    {
        void onWrite(BLECharacteristic *pCharacteristic)
        {
            std::string value = pCharacteristic->getValue();
        }
    };

    class MotorSpeedCharCallbacks : public BLECharacteristicCallbacks
    {
        void onWrite(BLECharacteristic *pCharacteristic)
        {
            std::string value = pCharacteristic->getValue();
        }
    };

public:
    void begin(ConnectionStateCallback *connectionStateCallback, DroneAttitudeCallback *droneAttitudeCallback, GainCallback *gainCallbak)
    {

        this->droneAttitudeCallback = droneAttitudeCallback;
        this->connectionStateCallback = connectionStateCallback;
        this->gainCallback = gainCallbak;

        BLEDevice::init("Eve-01");

        BLEServer *pServer = BLEDevice::createServer();
        pServer->advertiseOnDisconnect(true);
        pServer->setCallbacks(new ServerCallback(this));

        BLEService *pService = pServer->createService(DRONE_SERVICE_UUID);

        this->pThrustCharacteristic = pService->createCharacteristic(ATTITUDE_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        this->pThrustCharacteristic->setCallbacks(new AttitudeCharCallbacks(this));

        this->pGainCharacteristic = pService->createCharacteristic(NEW_GAIN_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        this->pGainCharacteristic->setCallbacks(new NewGainCharCallbacks(this));

        this->pAnglesCharacteristic = pService->createCharacteristic(ANGLES_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
        this->pAnglesCharacteristic->setCallbacks(new AnglesCharCallbacks());

        this->pPIDsCharacteristic = pService->createCharacteristic(PID_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
        this->pPIDsCharacteristic->setCallbacks(new PIDsCharCallbacks());

        this->pMotorSpeedCharacteristic = pService->createCharacteristic(MOTOR_SPEED_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
        this->pMotorSpeedCharacteristic->setCallbacks(new MotorSpeedCharCallbacks());

        this->pGeneralMsgCharacteristic = pService->createCharacteristic(GENERAL_MSG_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

        pService->start();

        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->start();
    }

    void writeAngles(std::string value)
    {
        this->pAnglesCharacteristic->setValue(value);
        this->pAnglesCharacteristic->notify(true);
    }

    void writePIDs(std::string value)
    {
        this->pPIDsCharacteristic->setValue(value);
        this->pPIDsCharacteristic->notify(true);
    }

    void writeMotorCurrentThrust(std::string value)
    {
        this->pMotorSpeedCharacteristic->setValue(value);
        this->pMotorSpeedCharacteristic->notify(true);
    }

    void write20ByteMsg(std::string msg)
    {
        this->pGeneralMsgCharacteristic->setValue(msg);
        this->pGeneralMsgCharacteristic->notify(true);
    }
};
