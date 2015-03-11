#include <SDMSonar.h>
#include <Lux.h>
#include <MotorControl.h>
#include <Ultrasonic.h>

#define TRIGGER_PIN  43
#define ECHO_PIN     42
#define FL1 6
#define FL2 7
#define FR1 8
#define FR2 9
#define RL1 2
#define RL2 3
#define RR1 4
#define RR2 5
#define LIGHT_PIN A9
#define SDM_SONAR_PIN 32


Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
SDMSonar sdmSonar(SDM_SONAR_PIN);

Motor FL(FL1, FL2);
Motor FR(FR1, FR2);
Motor RL(RL1, RL2);
Motor RR(RR1, RR2);

MotorControl motorControl(FL, FR, RL, RR);
Lux lightSensor(LIGHT_PIN);
int state = 1;

int turnCount = 0;
int stateCount = 0;

float distance;
float lux;    

void setup()
{
}

void loop()
{
  float distance = sdmSonar.pulse();

  double throttle = 1;
  if(distance < 150)
  {
    throttle = throttle * pow((distance/150), 2);
  }

  if(throttle < 0.3)
  {
    throttle = 0.3;
  }

  if(distance < 30)
  {
    motorControl.reverse(0.6);
  }
  else if(turnCount == 0)
  {
    motorControl.forward(throttle);
  }
  else
  {
    motorControl.turnLeft();
    turnCount--;
  }
  Serial.print("throttle: ");
  Serial.println(throttle);

  if(distance < 30 && turnCount == 0)
  {
    stateCount++;
    turnCount = 18;
  }

  else
  {
    stateCount = 0;
  }

  delay(50);                  // waits for a second
}                                                                            

