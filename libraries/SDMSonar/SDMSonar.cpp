#include "Arduino.h"
#include "SDMSonar.h"

SDMSonar::SDMSonar(int pin)
{
	m_pin = pin;

	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
}

float SDMSonar::pulse( )
{
	unsigned long time;
	unsigned long sizeofpulse;
	float distance;

	//Write pulse for 25 useconds
	digitalWrite(m_pin, LOW);
	delayMicroseconds(25);
	digitalWrite(m_pin, HIGH);

	time = micros();
	pinMode(m_pin, INPUT);
	sizeofpulse = pulseIn(m_pin, LOW, 18000);
	time = micros() - time - sizeofpulse;
	distance = (time * 340.29/2/10000) - 3;
	
	//Serial.print(" Distance: ");
	//Serial.print(distance);
	//Serial.print(" cm \n\r");

	pinMode(m_pin, OUTPUT);
	return analogRead(m_pin);
}
