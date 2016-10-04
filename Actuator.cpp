// 
// 
// 

#include "Actuator.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Actuator::Actuator() {
}

void Actuator::initialize(byte pitch_pin, byte roll_pin) {
	logger("Actuator::initialize, pitch pin ");
	logger(pitch_pin);
	logger(", roll pin ");
	loggerln(roll_pin);

	_pitch_servo.attach(pitch_pin);
	_roll_servo.attach(roll_pin);
}

void Actuator::set_pitch_center_angle(float pitch_center) {
	_pitch_center = pitch_center;
}
void Actuator::set_roll_center_angle(float roll_center) {
	_roll_center = roll_center;
}

void Actuator::test_axis() {
	loggerln("test_axis");
	write_pitch(0);
	delay(200);
	write_pitch(180);
	delay(200);
	write_pitch(_pitch_center);

	_roll_servo.write(0);
	delay(200);
	_roll_servo.write(180);
	delay(200);
	_roll_servo.write(_roll_center);
}

float Actuator::prepare_move(float value) {
	if (value < SERVO_ANGLE_MIN) value = SERVO_ANGLE_MIN;
	if (value > SERVO_ANGLE_MAX) value = SERVO_ANGLE_MAX;
	return fmap(value, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_MIN, SERVO_MAX);
}

void Actuator::write_pitch(float pitch_angle) {
	logger("write_pitch ");
	loggerln(pitch_angle);
	_pitch_servo.writeMicroseconds(prepare_move(pitch_angle + _pitch_center));
}
void Actuator::write_roll(float roll_angle) {
	logger("write_roll ");
	loggerln(roll_angle);
	_roll_servo.writeMicroseconds(prepare_move(roll_angle + _roll_center));
}