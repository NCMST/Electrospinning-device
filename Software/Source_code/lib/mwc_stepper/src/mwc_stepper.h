// stepper.h

#ifndef _MWC_STEPPER_h
#define _MWC_STEPPER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

class MWCSTEPPER {
	
private:
	uint8_t enPin;
	uint8_t stepPin;
	uint8_t dirPin;
    uint64_t rpm_t=500;
public:
	MWCSTEPPER(uint8_t _enPin, uint8_t _dirPin, uint8_t _stepPin);
	bool init();
	bool active(bool _active);
	bool run(uint64_t _rpmt);
	bool run();
	uint64_t set(bool _dir, float _rpm, uint16_t _pulse);
};


#endif
