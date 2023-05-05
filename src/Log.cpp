//
// Created by ahmed on 4/30/2023.
//

#include "Log.h"


void Log::print(const std::string& msg) {
    Serial.print(msg.c_str());
}

void Log::d(const std::string& msg) {
    Serial.println(msg.c_str());
}

void Log::d(const double& value) {
    Serial.println(value);
}
