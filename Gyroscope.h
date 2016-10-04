// Gyroscope.h

#ifndef _GYROSCOPE_h
#define _GYROSCOPE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define MPU6050_INCLUDE_DMP_MOTIONAPPS41
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

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

class Gyroscope {
private:
	MPU6050 mpu;

	Stream * _logger;

	byte _intrrupt_pin;

	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	bool dmpReady = false;  // set true if DMP init was successful
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer


							// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


	float current_angles[3];


	float _stability_last_values[3];
	int _stability_countdown;
	float _stability_tolerance;
	int _stability_iterations;
	bool _stability_is_stable;



public:
	Gyroscope(Stream * logger, byte interrupt_pin);

	bool initialize(int iterations, float tolerance);

	void loop();

	bool is_stable();

	void read_ypr(float ypr[3]);


};

#endif

