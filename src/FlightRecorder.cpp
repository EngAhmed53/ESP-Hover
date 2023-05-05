//
// Created by ahmed on 4/25/2023.
//

#include"FlightRecorder.h"
#include "Log.h"

FlightRecorder::FlightRecorder(const std::function<void(std::vector<uint8_t> &&)> &onReadyToFlush) : _onReadyToFlush(
        onReadyToFlush) {}

void FlightRecorder::insertAndFlushIfReady(FlightRecord record) {
    _records.emplace_back(record);
    //Log::d("records size = " + std::to_string(_records.size()));
    if (_records.size() >= MAX_RECORDS_CAPACITY) flushRecords();
}

void FlightRecorder::flushNow() {
    if (!_records.empty()) flushRecords();
}

void FlightRecorder::flushRecords() {
    std::vector<uint8_t> buffer;

    for (FlightRecord record: _records) {
        size_t recordSize = sizeof(record);
        buffer.insert(buffer.end(), (uint8_t *) &record, (uint8_t *) &record + recordSize);
    }
    //Log::d("sending records");
    _records.clear();
    _onReadyToFlush(std::move(buffer));
}

