//
// Created by ahmed on 4/25/2023.
//

#include "BLEManager.h"

BLEManager::~BLEManager() {
    BLEDevice::deinit(true);
}

void BLEManager::init() {
    _serverCallbacks = std::unique_ptr<ServerCallbacks>(new ServerCallbacks(shared_from_this()));
    _controlCharCallbacks = std::unique_ptr<ControlCharCallbacks>(new ControlCharCallbacks(shared_from_this()));
    _commandsCharCallback = std::unique_ptr<CommandsCharCallbacks>(new CommandsCharCallbacks(shared_from_this()));
    _dataCharCallback = std::unique_ptr<DataCharCallbacks>(new DataCharCallbacks(shared_from_this()));

    BLEDevice::init("Eve-01");
    BLEDevice::setMTU(517);

    BLEServer *pServer = BLEDevice::createServer();
    pServer->advertiseOnDisconnect(true);
    pServer->setCallbacks(_serverCallbacks.get());

    BLEService *pService = pServer->createService(DRONE_SERVICE_UUID);

    _controlCharacteristic = pService->createCharacteristic(CONTROL_CHARACTERISTIC_UUID);
    _controlCharacteristic->setCallbacks(_controlCharCallbacks.get());

    _commandCharacteristic = pService->createCharacteristic(COMMAND_CHARACTERISTIC_UUID);
    _commandCharacteristic->setCallbacks(_commandsCharCallback.get());

    _dataCharacteristic = pService->createCharacteristic(DATA_CHARACTERISTIC_UUID,
                                                         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    _dataCharacteristic->setCallbacks(_dataCharCallback.get());

    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}

void BLEManager::setConnectionCallback(std::function<void(ConnectionState)> callback) {
    _connectionCallback =  std::move(callback);
}

void BLEManager::setGainsCallback(std::function<void(const std::vector<double> &&)> callback) {
    _gainsCallback =  std::move(callback);
}

void BLEManager::setAttitudeCallback(std::function<void(int, int, int, int)> callback) {
    _attitudeCallback = std::move(callback);
}

const std::function<void(ConnectionState)> &BLEManager::getConnectionStateCallback() const {
    return _connectionCallback;
}

const std::function<void(const std::vector<double> &&)> &BLEManager::getGainsCallback() const {
    return _gainsCallback;
}

const std::function<void(int, int, int, int)> &BLEManager::getAttitudeCallback() const {
    return _attitudeCallback;
}

void BLEManager::writeRecords(std::vector<uint8_t> &&records) {
    _dataCharacteristic->setValue(records.data(), records.size());
    _dataCharacteristic->notify(true);
}

// ServerCallbacks
BLEManager::ServerCallbacks::ServerCallbacks(const std::shared_ptr<BLEManager> &connectionManager) {
    _connectionManager = connectionManager;
}

void BLEManager::ServerCallbacks::onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    pServer->updateConnParams(desc->conn_handle, 6, 8, 0, 60);
}

void BLEManager::ServerCallbacks::onDisconnect(NimBLEServer *pServer) {
    _connectionManager->getConnectionStateCallback()(ConnectionState::DISCONNECTED);
}

void BLEManager::ServerCallbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) {
    _connectionManager->getConnectionStateCallback()(ConnectionState::CONNECTED);
}
// ServerCallbacks


// BaseCharacteristicCallbacks
BLEManager::BaseCharacteristicCallbacks::BaseCharacteristicCallbacks(
        const std::shared_ptr<BLEManager> &connectionManager) {
    _connectionManager = connectionManager;
}
// BaseCharacteristicCallbacks


// ControlCharCallbacks
BLEManager::ControlCharCallbacks::ControlCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager)
        : BaseCharacteristicCallbacks(connectionManager) {}

void BLEManager::ControlCharCallbacks::onWrite(NimBLECharacteristic *pCharacteristic) {
    std::string data = pCharacteristic->getValue();

    int thrust = std::stoi(data.substr(0, 2), nullptr, 16);
    int pitch = std::stoi(data.substr(2, 2), nullptr, 16);
    int roll = std::stoi(data.substr(4, 2), nullptr, 16);
    int yaw = std::stoi(data.substr(6, 2), nullptr, 16);

    _connectionManager->getAttitudeCallback()(thrust, pitch, roll, yaw);
}
// ControlCharCallbacks


// CommandsCharCallbacks
BLEManager::CommandsCharCallbacks::CommandsCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager)
        : BaseCharacteristicCallbacks(connectionManager) {}

void BLEManager::CommandsCharCallbacks::onWrite(NimBLECharacteristic *pCharacteristic) {
    std::vector<double> gains{};
    std::string data = pCharacteristic->getValue();

    for (int i = 0; i < 9; i++) {
        double gain = std::stoi(data.substr(i * 2, 2), nullptr, 16) / 100.0;
        gains.push_back(gain);
    }

    _connectionManager->getGainsCallback()(std::move(gains));
}
// CommandsCharCallbacks


// DataCharCallbacks
BLEManager::DataCharCallbacks::DataCharCallbacks(const std::shared_ptr<BLEManager> &connectionManager)
        : BaseCharacteristicCallbacks(connectionManager) {}
// DataCharCallbacks