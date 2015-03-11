/*
Motor control library for MTE 380 group 4, controlling a 4WD robot
*/

#include "Arduino.h"

#ifndef MotorControl_h
#define MotorControl_h

class Motor
{
	public:
		Motor();
		Motor(int pin1, int pin2);
	 	void motorForward( double throttle );
		void motorReverse( double throttle );

	private:
		int m_pin1;
		int m_pin2;

};


class MotorControl
{
	public:
		MotorControl();
		MotorControl(Motor frontLeft, Motor frontRight, Motor rearLeft, Motor rearRight);
		MotorControl(int FA1, int FA2, int FB1, int FB2, int RA1, int RA2, int RB1, int RB2);
		void forward(double throttle);
		void reverse(double throttle);
		void forward( double leftThrottle, double rightThrottle );
		void reverse( double leftThrottle, double rightThrottle );
		void turnLeft( );
		void turnRight( );
		void fullStop();

	private:
		void throttleLeft( int motorDirection, double throttle ); //motorDirection 1 is forward, -1 is reverse
		void throttleRight( int motorDirection, double throttle );

		Motor m_frontLeft;
		Motor m_frontRight;
		Motor m_rearLeft;
		Motor m_rearRight;
};

#endif