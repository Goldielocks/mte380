/*
Motor control library for MTE 380 group 4, controlling a 4WD robot
*/

#include "Arduino.h"

#ifndef Lux_h
#define Lux_h

class Lux
{
	public:
		Lux(int pin);
	 	float getLux( );

	private:
		int m_pin;

};

#endif