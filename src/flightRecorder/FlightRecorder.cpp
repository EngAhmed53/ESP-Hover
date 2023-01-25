#include <Arduino.h>

#define MAX_RECORDS_CAPACITY 13

using namespace std;

typedef struct __attribute__((__packed__)) PackedFlightRecord
{
    uint32_t millis; // 4 bytes

    uint8_t pitch_setpoint; // 3 bytes
    uint8_t roll_setpoint;
    uint8_t yaw_setpoint;

    int16_t alttiude; // 2 bytes

    int16_t pitch_angle; // 6 bytes
    int16_t roll_angle;
    int16_t yaw_rate;

    int16_t pitch_p; // 6 bytes
    int16_t pitch_i;
    int16_t pitch_d;

    int16_t roll_p; // 6 bytes
    int16_t roll_i;
    int16_t roll_d;

    int16_t yaw_p; // 6 bytes
    int16_t yaw_i;
    int16_t yaw_d;

    uint8_t top_motor_thrust; // 4 bytes
    uint8_t bottom_motor_thrust;
    uint8_t right_motor_thrust;
    uint8_t left_motor_thrust;
};

class FlightRecorderCallback
{
public:
    virtual void onRecordsReady(vector<uint8_t> records);
};

class FlightRecorder
{
private:
    vector<PackedFlightRecord> records;

    FlightRecorderCallback *callback;

    template <typename T>
    vector<vector<T>> chunkVector(const vector<T> &source, size_t chunkSize)
    {
        vector<vector<T>> result;
        result.reserve((source.size() + chunkSize - 1) / chunkSize);

        auto start = source.begin();
        auto end = source.end();

        int count = 0;

        while (start != end)
        {
            Serial.print("Chunk loop = ");
            Serial.println(count++);

            auto next = std::distance(start, end) >= chunkSize
                            ? start + chunkSize
                            : end;

            result.emplace_back(start, next);
            start = next;
        }

        return result;
    }
    void flushRecords()
    {
        vector<uint8_t> buffer;

        for (PackedFlightRecord record : records)
        {

            // Serial2.println(record.pitch_angle);
            // Serial2.println(record.roll_angle);

            size_t recordSize = sizeof(record);
            buffer.insert(buffer.end(), (uint8_t *)&record, (uint8_t *)&record + recordSize);
        }
        // Serial.print("Records buffer to print size = ");
        // Serial.println(buffer.size());
        // for (size_t i = 0; i < buffer.size(); i++)
        // {
        //     Serial2.print(buffer[i]);
        //     Serial2.print(",");
        // }

        records.clear();
        callback->onRecordsReady(buffer);
    }

public:
    void begin(FlightRecorderCallback *callback)
    {
        this->callback = callback;
    }

    void insertAndFlushIfReady(PackedFlightRecord record)
    {
        records.push_back(record);

        if (records.size() >= MAX_RECORDS_CAPACITY)
        {
            // Serial.print("Will send records at millis = ");
            // Serial.println(millis());
            flushRecords();
        }
    }
};