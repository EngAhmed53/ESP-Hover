//
// Created by ahmed on 4/24/2023.
//

#include "../include/PlusQuadThrustManager.h"

PlusQuadThrustManager::PlusQuadThrustManager(double highBoundary, double lowBoundary, int filterWeight)
        : ThrustManager(), _motorAFilter(filterWeight, 0),
          _motorBFilter(filterWeight, 0),
          _motorCFilter(filterWeight, 0),
          _motorDFilter(filterWeight, 0),
          _highBoundary(highBoundary),
          _lowBoundary(lowBoundary) {}

ThrustModel PlusQuadThrustManager::calculateMotorsThrust(double pitchPID, double rollPID, double yawPID) {
    ThrustModel model{};
    model.motor_a_thrust = calculateSingleMotorThrust(_motorAFilter, (_baseThrust - pitchPID + yawPID));
    model.motor_b_thrust = calculateSingleMotorThrust(_motorBFilter, (_baseThrust + pitchPID + yawPID));
    model.motor_c_thrust = calculateSingleMotorThrust(_motorCFilter, (_baseThrust - rollPID - yawPID));
    model.motor_d_thrust = calculateSingleMotorThrust(_motorDFilter, (_baseThrust + rollPID - yawPID));

    return model;
}

int PlusQuadThrustManager::calculateSingleMotorThrust(ExponentialFilter<double> &motorFilter, double speed) {
    double constrainedSpeed = constrain(speed, _lowBoundary, _highBoundary);
    motorFilter.Filter(constrainedSpeed);
    return static_cast<int>(motorFilter.Current());
}

void PlusQuadThrustManager::resetMotorsFilters() {
    _motorAFilter.SetCurrent(0);
    _motorBFilter.SetCurrent(0);
    _motorCFilter.SetCurrent(0);
    _motorDFilter.SetCurrent(0);
}
