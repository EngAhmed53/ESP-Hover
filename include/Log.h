//
// Created by ahmed on 4/30/2023.
//

#ifndef ESP_HOVER_LOG_H
#define ESP_HOVER_LOG_H

#include <Arduino.h>

#define Serial Serial2

class Log {
public:
    static void d(const std::string& msg);
    static void print(const std::string& msg);
    static void d(const double& msg);
};


#endif //ESP_HOVER_LOG_H
