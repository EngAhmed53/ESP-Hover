//
// Created by ahmed on 4/20/2023.
//

#ifndef ESP_HOVER_PID_H
#define ESP_HOVER_PID_H

#include <Arduino.h>
#include <utility>
#include "model.h"

class PID {

public:
    PID(double highBoundary, double lowBoundary, double maxNeglectedError = 3.0,
                 double minNeglectedError = -3.0);

    void SetPoint(double setPoint);

    double SetPoint() const;

    void Gains(double kp, double ki, double kd);

    PIDModel Compute(double sampleTime, double currentAngle, double rate);

private:
    double _highBoundary{0}, _lowBoundary{0}, _maxNeglectedError{0}, _minNeglectedError{0};
    double _setPoint{0};
    double _kp{0}, _ki{0}, _kd{0};
    double _accumError{0};
};

#endif //ESP_HOVER_PID_H
