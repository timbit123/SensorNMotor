//#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
//#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>
#include "Kinematics.h"
#include "Kinematics_ARG.h"
#include <Servo.h>
#define armValue (10)
#define dealyVal (3000)
#define Speed (1210)
// Include this last as it contains objects from above declarations
//#include "FlightControlProcessor.h"
//#include "DataStorage.h"

#include <Device_I2C.h>

// Gyroscope declaration
#define ITG3200_ADDRESS_ALTERNATE
#include <Gyroscope_ITG3200.h>

// Accelerometer declaration
#include <Accelerometer_ADXL345.h>
// Motor declaration
//#include <Motors_PWM_Timer.h>

Servo M1;
Servo M2;
Servo M3;
Servo M4;

#include "SerialCom.h"

void initPlatform() {
	Wire.begin();
	TWBR = 12;
}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() {
	// Kenny default value, a real accel calibration is strongly recommended
	accelScaleFactor[XAXIS] = 0.0371299982;
	accelScaleFactor[YAXIS] = -0.0374319982;
	accelScaleFactor[ZAXIS] = -0.0385979986;
}

/**
* Measure critical sensors
*/
void measureCriticalSensors() {
	measureGyroSum();
	measureAccelSum();
}

void setup()
{
	Serial.begin(BAUD);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	M1.attach(5);
	M2.attach(6);
	M3.attach(9);
	M4.attach(10);

	M1.write(armValue); 
	M2.write(armValue); 
	M3.write(armValue); 
	M4.write(armValue); 
	initPlatform();
	initializePlatformSpecificAccelCalibration();

	// Initialize sensors
	// If sensors have a common initialization routine
	// insert it into the gyro class because it executes first
	initializeGyro(); // defined in Gyro.h
	while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
	initializeAccel(); // defined in Accel.h
	computeAccelBias();
	setupFourthOrder();

	initializeKinematics();
	/* add setup code here */

}
void process100HzTask() {

	G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
	hundredHZpreviousTime = currentTime;

	evaluateGyroRate();
	evaluateMetersPerSec();

	for (int axis = XAXIS; axis <= ZAXIS; axis++) {
		filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
	}

	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

	sendSerialTelemetry();
}
void process50HzTask() {

	//read serial here
	//for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
	//	Serial.print(gyroRate[axis]);
	//	Serial.print(',');
	//}
	//for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
	//	Serial.print(filteredAccel[axis]);
	//	Serial.print(',');
	//}
	//Serial.println();
	readSerialCommand();
}
void process10HzTask()
{
}

void process1HzTask()
{
}

void loop()
{

	currentTime = micros();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) {

		frameCounter++;


		process100HzTask();

		// ================================================================
		// 50Hz task loop
		// ================================================================
		if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
			process50HzTask();
		}

		// ================================================================
		// 10Hz task loop
		// ================================================================
		if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
			process10HzTask();
		}

		// ================================================================
		// 1Hz task loop
		// ================================================================
		if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
			process1HzTask();
		}

		previousTime = currentTime;
	}

	if (frameCounter >= 100) {
		frameCounter = 0;
	}

}
