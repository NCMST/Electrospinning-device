#include "mwc_stepper.h"
#include <Arduino_FreeRTOS.h>


MWCSTEPPER::MWCSTEPPER(uint8_t _enPin, uint8_t _dirPin, uint8_t _stepPin)
{
    stepPin = _stepPin;
    dirPin = _dirPin;
    enPin = _enPin;
}

bool MWCSTEPPER::init() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);
    return true;
}

uint64_t MWCSTEPPER::set(bool _dir, float _rpm, uint16_t _pulse) {
    digitalWrite(dirPin, _dir);
    rpm_t = uint64_t(60000000 / (2 * _pulse * _rpm)); 
    return rpm_t;
}

bool MWCSTEPPER::run(uint64_t _rpmt) {
    digitalWrite(stepPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(_rpmt / 1000)); 
    digitalWrite(stepPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(_rpmt / 1000)); 
    return true;
}

bool MWCSTEPPER::run() {
    digitalWrite(stepPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(rpm_t / 1000)); // miliseconds
    digitalWrite(stepPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(rpm_t / 1000)); 
    return true;
}

bool MWCSTEPPER::active(bool _active) {
    digitalWrite(enPin, _active);
    return true;
}