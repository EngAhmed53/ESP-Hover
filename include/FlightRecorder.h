//
// Created by ahmed on 4/29/2023.
//

#ifndef ESP_HOVER_FLIGHTRECORDER_H
#define ESP_HOVER_FLIGHTRECORDER_H

#define MAX_RECORDS_CAPACITY 13

#include "model.h"
#include <vector>
#include <functional>

class FlightRecorder {
public:
    explicit FlightRecorder(const std::function<void(std::vector<uint8_t> &&)>& onReadyToFlush);

    void insertAndFlushIfReady(FlightRecord record);

    void flushNow();

private:
    std::vector<FlightRecord> _records;
    std::function<void(std::vector<uint8_t> &&)> _onReadyToFlush;

    void flushRecords();
};

#endif //ESP_HOVER_FLIGHTRECORDER_H
