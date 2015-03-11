#include "Arduino.h"
#include "Lux.h"

Lux::Lux(int pin)
{
	m_pin = pin;
	pinMode(pin, INPUT);
}

float Lux::getLux( )
{
	return analogRead(m_pin);
}
