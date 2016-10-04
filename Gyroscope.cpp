// 
// 
// 

#include "Gyroscope.h"

#include "MPU6050_6Axis_MotionApps20.h"


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}




Gyroscope::Gyroscope(Stream * logger, byte interrupt_pin) {
	loggerln("Gyroscope::constructor");
	dmpReady = false;

	_logger = logger;
	_intrrupt_pin = interrupt_pin;

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

}

bool Gyroscope::initialize(int iterations, float tolerance) {
	logger("Gyroscope::initialize iterations: ");
	logger(iterations);
	logger(" tolerance: ");
	loggerln(tolerance);

	_stability_iterations = iterations;
	_stability_countdown = iterations;
	_stability_tolerance = tolerance;
	_stability_is_stable = false;

	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	mpu.setDLPFMode(MPU6050_DLPF_BW_256);
	

	//Initializing I2C devices...
	mpu.initialize();
	pinMode(_intrrupt_pin, INPUT);

	if (mpu.testConnection()) {
		loggerln("MPU6050 connection successful");
	}
	else {
		loggerln("MPU6050 connection failed");
	}


	devStatus = mpu.dmpInitialize();
	logger("dmpInitialize: ");
	loggerln(devStatus);

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	mpu.setZAccelOffset(1688); // 1688 factory default for my test chip


	if (devStatus == 0) {
		loggerln("turn on the DMP, now that it's ready");

		mpu.setDMPEnabled(true);

		attachInterrupt(digitalPinToInterrupt(_intrrupt_pin), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		//DMP ready! Waiting for first interrupt...
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		logger("packetSize: ");
		loggerln(packetSize);
		return true;
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)

		//DMP Initialization failed (code devStatus)
		loggerln("DMP Initialization failed");
		return false;
	}

}


void Gyroscope::loop() {
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		// other program behavior stuff here
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		loggerln("FIFO overflow!");

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;



		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		current_angles[0] = ypr[0] * 180 / M_PI;
		current_angles[1] = ypr[1] * 180 / M_PI;
		current_angles[2] = ypr[2] * 180 / M_PI;


		if (!_stability_is_stable) {

			if (_stability_countdown == 0) {
				if ((abs(_stability_last_values[0] - current_angles[0]) < _stability_tolerance) ||
					(abs(_stability_last_values[1] - current_angles[1]) < _stability_tolerance) ||
					(abs(_stability_last_values[2] - current_angles[2]) < _stability_tolerance)) {
					_stability_is_stable = true;
					//actuator.test_axis();
					loggerln("Gimbal Stable");

					return;
				}
				else {
					_stability_countdown = _stability_iterations;
				}

				//print the last values
				logger("ypr\t");
				logger(current_angles[0]);
				logger("\t");
				logger(current_angles[1]);
				logger("\t");
				loggerln(current_angles[2]);


				_stability_last_values[0] = current_angles[0];
				_stability_last_values[1] = current_angles[1];
				_stability_last_values[2] = current_angles[2];
			}


		}


//		if (_stability_is_stable) {
//			//find a way to reset the values
//
////#ifdef DEBUG_MESSAGES
////			Serial.print("ypr\t");
////			Serial.print(ypr[0]);
////			Serial.print("\t");
////			Serial.print(ypr[1]);
////			Serial.print("\t");
////			Serial.println(ypr[2]);
////#endif
//
//
//			//actuator.write_pitch(current_angles[1] - stability_last_values[1] + PITCH_CENTER_ANGLE);
//			//actuator.write_roll(current_angles[2] - stability_last_values[2] + ROLL_CENTER_ANGLE);
//
//
//		}

		_stability_countdown--;
	}
}

bool Gyroscope::is_stable() {
	return _stability_is_stable;
}


void Gyroscope::read_ypr(float ypr[3]) {
	ypr[0] = current_angles[0] - _stability_last_values[0];
	ypr[1] = current_angles[1] - _stability_last_values[1];
	ypr[2] = current_angles[2] - _stability_last_values[2];


}
