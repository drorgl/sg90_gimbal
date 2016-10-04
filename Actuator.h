// Actuator.h

#ifndef _ACTUATOR_h
#define _ACTUATOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>

#ifdef VM_DEBUG
#define logger(data){ \
	Serial.print(data); \
}

#define loggerln(data){ \
	Serial.println(data); \
}
#else
#define logger(data){ \
}
#define loggerln(data){ \
}
#endif

#define PITCH_SERVO_PIN 8
#define ROLL_SERVO_PIN 9


#define PITCH_CENTER_ANGLE 90
#define ROLL_CENTER_ANGLE 95

//Servo min/max usec values, should be according to specs but SG90 measured these values
#define SERVO_MIN 555
#define SERVO_MAX 2120
//min/max angle the servo can rotate, SG90 specifies 180 but reality is different
#define SERVO_ANGLE_MIN 15
#define SERVO_ANGLE_MAX 165


class Actuator {
private:
	Servo _pitch_servo;
	Servo _roll_servo;

	float _pitch_center;
	float _roll_center;

	float prepare_move(float value);

public:
	Actuator();

	void initialize(byte pitch_pin, byte roll_pin);

	void set_pitch_center_angle(float pitch_center);
	void set_roll_center_angle(float roll_center);

	void test_axis();

	void write_pitch(float pitch_angle);
	void write_roll(float roll_angle);
};



#endif

