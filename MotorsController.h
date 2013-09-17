/*
Servo motors[4];

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

int motorCommand[4] = {0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way
*/
/*
void initializeMotors()
{
	motors[MOTOR1].attach(2);
	motors[MOTOR2].attach(3);
	motors[MOTOR3].attach(5);
	motors[MOTOR4].attach(6);

	motors[MOTOR1].write(MINCOMMAND); 
	motors[MOTOR2].write(MINCOMMAND); 
	motors[MOTOR3].write(MINCOMMAND); 
	motors[MOTOR4].write(MINCOMMAND);
}
*/
/*
void writeMotors()
{
	motors[MOTOR1].writeMicroseconds(motorCommand[MOTOR1]);
	motors[MOTOR2].writeMicroseconds(motorCommand[MOTOR2]);
	motors[MOTOR3].writeMicroseconds(motorCommand[MOTOR3]);
	motors[MOTOR4].writeMicroseconds(motorCommand[MOTOR4]);
}*/
/*
void commandAllMotors(int command)
{
	motors[MOTOR1].writeMicroseconds(command);
	motors[MOTOR2].writeMicroseconds(command);
	motors[MOTOR3].writeMicroseconds(command);
	motors[MOTOR4].writeMicroseconds(command);
}
*/
void pulseMotors(byte nbPulse) {
	for (byte i = 0; i < nbPulse; i++) {
		commandAllMotors(MINCOMMAND + 100);
		delay(250);
		commandAllMotors(MINCOMMAND);
		delay(250);
	}
}
