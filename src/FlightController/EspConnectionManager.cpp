#include <esp_now.h>
#include <WiFi.h>
#include "EspConnectionManager.h"

DroneAttitudeCallback *thrustCB;
ConnectionStateCallback *connectionStateCB;
GainCallback *gainCB;

uint8_t controllerAddress[6] = {0x24, 0x0A, 0xC4, 0x8B, 0x6F, 0xC0};

struct_receive_message receivedData;
struct_str_message sendData;

esp_now_peer_info_t peerInfo;
int count = 0;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Data received: ");
    Serial.println(len);
    if (receivedData.should_reset_pid)
    {
        gainCB->onNewGain(
            receivedData.pitch_p, receivedData.pitch_i, receivedData.pitch_d,
            receivedData.roll_p, receivedData.roll_i, receivedData.roll_d,
            receivedData.yaw_p, receivedData.yaw_i, receivedData.yaw_d);
    }
    else
    {
       // thrustCB->onAttitudeChange(receivedData.thrust);
    }
}

static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void begin_esp_now(ConnectionStateCallback *connectionStateCallback, DroneAttitudeCallback *droneAttitudeCallback, GainCallback *gainCallbak)
{
    thrustCB = droneAttitudeCallback;
    connectionStateCB = connectionStateCallback;
    gainCB = gainCallbak;

    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);

    connectionStateCB->onConnectionStateChange(CONNECTED);
}

bool esp_now_send_str_msg(const char *msg)
{
    strcpy(sendData.a, msg);

    esp_err_t result = esp_now_send(controllerAddress, (uint8_t *)&sendData, sizeof(sendData));

    return result == ESP_OK;
}

bool esp_now_send_attiude_msg(struct_attiude_message attidue)
{
    if (count++ % 5 == 0)
    {
        esp_err_t result = esp_now_send(controllerAddress, (uint8_t *)&attidue, sizeof(attidue));
        return result == ESP_OK;
    }

    return true;
}
