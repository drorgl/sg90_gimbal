//#include <Kalman.h>
#include "Gyroscope.h"
#include "Actuator.h"


//#define DEBUG_MESSAGES

#define PITCH_SERVO_PIN 8
#define ROLL_SERVO_PIN 9

//#define PITCH_CENTER_ANGLE 90
//#define ROLL_CENTER_ANGLE 90

Actuator actuator;


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
Gyroscope gyroscope(NULL, INTERRUPT_PIN);


#define STABILITY_ITERATIONS 500
#define STABILITY_TOLERANCE 0.02


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


float current_angles[3];

bool first_stable = true;


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

void setup() {
#ifdef _DEBUG
	Serial.begin(115200);
#endif

	logger("starting...");

	logger("initializing actuator");
	actuator.initialize(PITCH_SERVO_PIN, ROLL_SERVO_PIN);
	logger("initializing pitch center angle");
	actuator.set_pitch_center_angle(PITCH_CENTER_ANGLE);
	logger("initializing roll center angle");
	actuator.set_roll_center_angle(ROLL_CENTER_ANGLE);

	logger("initializing gyroscope");
	gyroscope.initialize(STABILITY_ITERATIONS, STABILITY_TOLERANCE);

	pinMode(LED_PIN, OUTPUT);
}

void loop() {
	gyroscope.loop();

	if (gyroscope.is_stable()) {
		if (first_stable) {
			first_stable = false;
			actuator.test_axis();
		}
		

		gyroscope.read_ypr(current_angles);
		logger("read_ypr \t");
		logger(current_angles[0]);
		logger("\t");
		logger(current_angles[1]);
		logger("\t");
		loggerln(current_angles[2]);


		actuator.write_pitch(current_angles[1]);
		actuator.write_roll(current_angles[2]);
	}


	blinkState = !blinkState;
	digitalWrite(LED_PIN, blinkState);
}
