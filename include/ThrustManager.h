//
// Created by ahmed on 4/23/2023.
//

#ifndef ESP_HOVER_THRUSTMANAGER_H
#define ESP_HOVER_THRUSTMANAGER_H

#include "model.h"
#include <Arduino.h>

class ThrustManager {

public:
    ThrustManager();

    void updateBaseThrust(int thrust);

    int baseThrust() const;

    virtual ThrustModel calculateMotorsThrust(double pitchPID, double rollPID, double yawPID) = 0;

protected:
    int _baseThrust{0};
};

#endif //ESP_HOVER_THRUSTMANAGER_H
