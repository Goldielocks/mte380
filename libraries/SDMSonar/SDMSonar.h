/*
Motor control library for MTE 380 group 4, controlling a 4WD robot
*/

#include "Arduino.h"

#ifndef SDMSonar_h
#define SDMSonar_h

class SDMSonar
{
	public:
		SDMSonar(int pin);
	 	float pulse( );

	private:
		int m_pin;
};

#endif