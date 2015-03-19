#include "Arduino.h"
#include "MotorControl.h"

const int FORWARD = 1;
const int REVERSE = -1;

//BASIC MOTOR FUNCTIONS
Motor::Motor()
{
	m_pin1 = -1;
	m_pin2 = -1;
}

Motor::Motor(int pin1, int pin2)
{
	m_pin1 = pin1;
	m_pin2 = pin2;

	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
}

void Motor::motorForward( double throttle )
{
	if(throttle > 1)
	{
		throttle = 1;
	}

	if(throttle < 0)
	{
		throttle = 0;
	}

	digitalWrite(m_pin1, LOW);
	analogWrite(m_pin2, 255 * throttle);
}

void Motor::motorReverse( double throttle )
{
	if(throttle > 1)
	{
		throttle = 1;
	}

	if(throttle < 0)
	{
		throttle = 0;
	}

	digitalWrite(m_pin2, LOW);
	analogWrite(m_pin1, 255 * throttle);
}


//MOTOR CONTROLS
MotorControl::MotorControl()
{

}

MotorControl::MotorControl(Motor frontLeft, Motor frontRight, Motor rearLeft, Motor rearRight)
{
	m_frontLeft = frontLeft;
	m_frontRight = frontRight;
	m_rearLeft = rearLeft;
	m_rearRight = rearRight;
}

MotorControl::MotorControl(int FA1, int FA2, int FB1, int FB2, int RA1, int RA2, int RB1, int RB2)
{
	m_frontLeft = Motor(FA1, FA2);
	m_frontRight = Motor(FB1, FB2);
	m_rearLeft = Motor(RA1, RA2);
	m_rearRight = Motor(RB1, RB2);
}

void MotorControl::forward( double throttle )
{
	forward(throttle, throttle);
}

void MotorControl::reverse( double throttle)
{
	reverse(throttle, throttle);
}

void MotorControl::forward( double leftThrottle, double rightThrottle )
{
	throttleLeft(FORWARD, leftThrottle);
	throttleRight(FORWARD, rightThrottle);
}

void MotorControl::reverse( double leftThrottle, double rightThrottle )
{
	throttleLeft(REVERSE, leftThrottle);
	throttleRight(REVERSE, rightThrottle);
}

void MotorControl::turnLeft( )
{
	throttleLeft(REVERSE, 1);
	throttleRight(FORWARD, 1);
}

void MotorControl::turnRight( )
{
	throttleLeft(FORWARD, 1);
	throttleRight(REVERSE, 1);
}

void MotorControl::fullStop()
{
	throttleLeft(FORWARD, 0.1);
	throttleRight(FORWARD, 0.1);
	delay(10);
	throttleLeft(FORWARD, 0);
	throttleRight(FORWARD, 0);
}


void MotorControl::throttleLeft( int motorDirection, double throttle ) //motorDirection 1 is forward, -1 is reverse
{
	if(motorDirection == 1)
  	{
  		m_frontLeft.motorForward(throttle);
  		m_rearLeft.motorForward(throttle);
  	}
  	else
  	{
    	m_frontLeft.motorReverse(throttle);
    	m_rearLeft.motorReverse(throttle);
  	}
}

void MotorControl::throttleRight( int motorDirection, double throttle )
{
	if(motorDirection == 1)
  	{
  		m_frontRight.motorForward(throttle);
  		m_rearRight.motorForward(throttle);
  	}
  	else
  	{
    	m_frontRight.motorReverse(throttle);
    	m_rearRight.motorReverse(throttle);
  	}
}
