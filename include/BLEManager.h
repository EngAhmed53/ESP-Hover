//
// Created by ahmed on 4/25/2023.
//

#ifndef ESP_HOVER_BLEMANAGER_H
#define ESP_HOVER_BLEMANAGER_H

#define DRONE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHARACTERISTIC_UUID "fbf426ba-5f90-11ec-bf63-0242ac130002"
#define DATA_CHARACTERISTIC_UUID "3e56610a-5ffb-11ec-bf63-0242ac130002"
#define COMMAND_CHARACTERISTIC_UUID "ec743a01-5ffb-11ec-bf63-0242ac130002"

#include "model.h"
#include <NimBLEDevice.h>
#include <functional>
#include <vector>
#include <memory>

class BLEManager : public std::enable_shared_from_this<BLEManager> {
public:

    ~BLEManager();

    void init();

    void setConnectionCallback(std::function<void(ConnectionState)> callback);

    void setGainsCallback(std::function<void(const std::vector<double> &&)> callback);

    void setAttitudeCallback(std::function<void(int, int, int, int)> callback);

    const std::function<void(ConnectionState)> &getConnectionStateCallback() const;

    const std::function<void(const std::vector<double> &&)> &getGainsCallback() const;

    const std::function<void(int, int, int, int)> &getAttitudeCallback() const;

    void writeRecords(std::vector<uint8_t> &&records);

    class ServerCallbacks : public BLEServerCallbacks {
    public:
        explicit ServerCallbacks(const std::shared_ptr<BLEManager> &connectionManager);

    private:
        std::shared_ptr<BLEManager> _connectionManager;

        void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override;

        void onDisconnect(NimBLEServer *pServer) override;

        void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) override;
    };

    class BaseCharacteristicCallbacks : public BLECharacteristicCallbacks {
    public:
        explicit BaseCharacteristicCallbacks(const std::shared_ptr<BLEManager> &connectionManager);

    protected:
        std::shared_ptr<BLEManager> _connectionManager;
    };

    class ControlCharCallbacks : public BaseCharacteristicCallbacks {
    public:
        explicit ControlCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager);

        void onWrite(BLECharacteristic *pCharacteristic) override;
    };

    class CommandsCharCallbacks : public BaseCharacteristicCallbacks {
    public:
        explicit CommandsCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager);

        void onWrite(BLECharacteristic *pCharacteristic) override;
    };

    class DataCharCallbacks : public BaseCharacteristicCallbacks {
    public:
        explicit DataCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager);
    };

private:
    std::function<void(ConnectionState state)> _connectionCallback;
    std::function<void(const std::vector<double> &&gains)> _gainsCallback;
    std::function<void(int thrust, int pitch, int roll, int yaw)> _attitudeCallback;

    BLECharacteristic *_controlCharacteristic, *_commandCharacteristic, *_dataCharacteristic;

    std::unique_ptr<ServerCallbacks> _serverCallbacks;
    std::unique_ptr<ControlCharCallbacks> _controlCharCallbacks;
    std::unique_ptr<CommandsCharCallbacks> _commandsCharCallback;
    std::unique_ptr<DataCharCallbacks> _dataCharCallback;
};

#endif //ESP_HOVER_BLEMANAGER_H
