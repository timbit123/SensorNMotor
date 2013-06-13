/*
AeroQuad v3.0.1 - February 2012
www.AeroQuad.com
Copyright (c) 2012 Ted Carancho.  All rights reserved.
An Open Source Arduino based multicopter.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

char queryType = 'X';

void initCommunication() {
	// do nothing here for now
}

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
//bool validateCalibrateCommand(byte command)
//{
//  if (readFloatSerial() == 123.45) {// use a specific float value to validate full throttle call is being sent
//    motorArmed = OFF;
//    calibrateESC = command;
//    return true;
//  }
//  else {
//    calibrateESC = 0;
//    testCommand = 1000;
//    return false;
//  }
//}

//void readSerialPID(unsigned char PIDid) {
//  struct PIDdata* pid = &PID[PIDid];
//  pid->P = readFloatSerial();
//  pid->I = readFloatSerial();
//  pid->D = readFloatSerial();
//  pid->lastError = 0;
//  pid->integratedError = 0;
//}

//void skipSerialValues(byte number) {
//  for(byte i=0; i<number; i++) {
//    readFloatSerial();
//  }
//}

void readSerialCommand() {
	// Check for serial message
	if (Serial.available()) {
		queryType = Serial.read();
		switch (queryType) {
		case 'X': // Stop sending messages
			break;

		case 'M': // Read Motor Values
                        if(ARMED)
                        {
			  M1.writeMicroseconds(readIntegerSerial()); //motor1
			  M2.writeMicroseconds(readIntegerSerial()); //motor2
			  M3.writeMicroseconds(readIntegerSerial()); //motor3
			  M4.writeMicroseconds(readIntegerSerial()); //motor4
                        }else
                        {
                          readIntegerSerial();
                          readIntegerSerial();
                          readIntegerSerial();
                          readIntegerSerial();
                          Serial.println("Motor NOT ARMED");
                        }
			break;

                case 'A': //ARM or disarm Motor
                        if(readIntegerSerial())// if == 1 we ARMED it
                        {
                          ARMED = true;
                          Serial.println(1);
                        }else{
                          ARMED = false;
                          M1.write(armValue); 
	                  M2.write(armValue); 
	                  M3.write(armValue); 
	                  M4.write(armValue);
                          Serial.println(0);
                        }
                        break;
/*
		case 'N': // Read Motor Values
			M1.write(readIntegerSerial()); //motor1
			readIntegerSerial(); //motor2
			readIntegerSerial(); //motor3
			readIntegerSerial(); //motor4
			break;

*/

		}
	}
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val) {
	Serial.print(val);
	comma();
}

void PrintValueComma(double val) {
	Serial.print(val);
	comma();
}

void PrintValueComma(char val) {
	Serial.print(val);
	comma();
}

void PrintValueComma(int val) {
	Serial.print(val);
	comma();
}

void PrintValueComma(unsigned long val)
{
	Serial.print(val);
	comma();
}

void PrintValueComma(byte val)
{
	Serial.print(val);
	comma();
}

void PrintValueComma(long int val)
{
	Serial.print(val);
	comma();
}

//void PrintPID(unsigned char IDPid)
//{
//	PrintValueComma(PID[IDPid].P);
//	PrintValueComma(PID[IDPid].I);
//	PrintValueComma(PID[IDPid].D);
//}

void PrintDummyValues(byte number) {
	for(byte i=0; i<number; i++) {
		PrintValueComma(0);
	}
}


void sendSerialTelemetry() {
	switch (queryType) {
	case '=': // Reserved debug command to view any variable from Serial Monitor
		break;

	case 'e': // miscellaneous config values
		PrintValueComma(aref);
		Serial.println(minArmedThrottle);
		queryType = 'X';
		break;


	case 'i': // Send sensor data (Main information to use!)
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			PrintValueComma(gyroRate[axis]);
		}
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			PrintValueComma(filteredAccel[axis]);
		}
		Serial.println();
		break;



	case 'k': // Send accelerometer cal values
		Serial.print(accelScaleFactor[XAXIS], 6);
		comma();
		Serial.print(runTimeAccelBias[XAXIS], 6);
		comma();
		Serial.print(accelScaleFactor[YAXIS], 6);
		comma();
		Serial.print(runTimeAccelBias[YAXIS], 6);
		comma();
		Serial.print(accelScaleFactor[ZAXIS], 6);
		comma();
		Serial.println(runTimeAccelBias[ZAXIS], 6);
		queryType = 'X';
		break;

	case 'l': // Send raw accel values
		measureAccelSum();
		PrintValueComma((int)(accelSample[XAXIS]/accelSampleCount));
		accelSample[XAXIS] = 0;
		PrintValueComma((int)(accelSample[YAXIS]/accelSampleCount));
		accelSample[YAXIS] = 0;
		Serial.println ((int)(accelSample[ZAXIS]/accelSampleCount));
		accelSample[ZAXIS] = 0;
		accelSampleCount = 0;
		break;

	case 'q': // Send Vehicle State Value
		Serial.println(vehicleState);
		queryType = 'X';
		break;

	case 's': // Send all flight data
		PrintValueComma(motorArmed);
		PrintValueComma(kinematicsAngle[XAXIS]);
		PrintValueComma(kinematicsAngle[YAXIS]);
		//PrintValueComma(getHeading());


		/*for (byte motor = 0; motor < LASTMOTOR; motor++) {
		PrintValueComma(motorCommand[motor]);
		}*/


		PrintValueComma(flightMode);
		Serial.println();
		break;


	case 'x': // Stop sending messages
		break;

	}
}

void readValueSerial(char *data, byte size) {
	byte index = 0;
	byte timeout = 0;
	data[0] = '\0';

	do {
		if (Serial.available() == 0) {
			delay(1);
			timeout++;
		} else {
			data[index] = Serial.read();
			timeout = 0;
			index++;
		}
	} while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

	data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial() {
	char data[15] = "";

	readValueSerial(data, sizeof(data));
	return atof(data);
}

// Used to read integer values from the serial port
long readIntegerSerial() {
	char data[16] = "";

	readValueSerial(data, sizeof(data));
	return atol(data);
}

void comma() {
	Serial.print(',');
}

#endif // _AQ_SERIAL_COMM_
