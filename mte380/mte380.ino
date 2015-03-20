#include <MotorControl.h>
#include <Ultrasonic.h>
#include <Servo.h>

#define CLIFF_TRIGGER_PIN  32
#define CLIFF_ECHO_PIN     33
#define LEFT_TRIGGER_PIN  27
#define LEFT_ECHO_PIN     26
#define RIGHT_TRIGGER_PIN  29
#define RIGHT_ECHO_PIN     28
#define FRONTLEFT_TRIGGER_PIN  24
#define FRONTLEFT_ECHO_PIN     25
#define FRONTRIGHT_TRIGGER_PIN  31
#define FRONTRIGHT_ECHO_PIN     30

#define NUMBER_BUFFERS 1
#define BUFFER_SIZE 2
#define BUFFER2_SIZE 10

#define BUFFER_01      0

size_t count = 0;

Ultrasonic cliffSonic(CLIFF_TRIGGER_PIN, CLIFF_ECHO_PIN);
Ultrasonic leftSonic(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
Ultrasonic rightSonic(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
Ultrasonic frontLeftSonic(FRONTLEFT_TRIGGER_PIN, FRONTLEFT_ECHO_PIN);
Ultrasonic frontRightSonic(FRONTRIGHT_TRIGGER_PIN, FRONTRIGHT_ECHO_PIN);

#define FL1 6
#define FL2 7
#define FR1 8
#define FR2 9
#define RL1 2
#define RL2 3
#define RR1 4
#define RR2 5


Motor FL(FL1, FL2);
Motor FR(FR1, FR2);
Motor RL(RL1, RL2);
Motor RR(RR1, RR2);

MotorControl motorControl(FL, FR, RL, RR);

#define VERTICAL_IR_PIN        A15
#define HORIZONTAL_IR_PIN      A14

int TILT_PIN = 52;

int ARM_SERVO_PIN = 12;


int NINETY_DEGREE_DELAY = 340;
float CORRECTION_GAIN = 1;

int currentDirection = 0;

int state = 0;
int side = 0;

bool disableSD = false;

int turnCount = 0;
int stateCount = 0;
int pos = 0;
Servo armServo;

float leftDistance;
float rightDistance;
float cliffDistance;

float hDistance, vDistance;

void setup()
{
  pinMode(VERTICAL_IR_PIN, INPUT);
  pinMode(HORIZONTAL_IR_PIN, INPUT);

  pinMode(TILT_PIN, INPUT);  
  Serial.begin(9600);
  armServo.attach(ARM_SERVO_PIN, 544, 2600); // Min freq/max freq to achieve expected angles

  if(!(
  leftSonic.sampleCreate(NUMBER_BUFFERS, BUFFER_SIZE) &
    rightSonic.sampleCreate(NUMBER_BUFFERS, BUFFER_SIZE) &
    frontLeftSonic.sampleCreate(NUMBER_BUFFERS, BUFFER_SIZE) &
    frontRightSonic.sampleCreate(NUMBER_BUFFERS, BUFFER_SIZE) &
    cliffSonic.sampleCreate(NUMBER_BUFFERS, 3)
    ))  
  {
    disableSD = true;
    Serial.println("Could not allocate memory.");
  }  

  moveArm(180);
  delay(500);
}

void loop()
{
  Serial.println(digitalRead(TILT_PIN));  
  /*
  long microsec = rightSonic.timing();
   
   rightDistance = rightSonic.convert(microsec, Ultrasonic::CM);
   Serial.println(rightDistance);
   */
  //rightDistance = analogRead(HORIZONTAL_IR_PIN);
  //updateUltrasonics();
  // Serial.println(rightDistance); 
  //initialPosition();
  //findRamp();
  //climbRamp();
  //findBase();
  findLarry();
  getLarry();
  while(1){delay(10);}
}               

void initialPosition()
{
  switch(side)
  {
  case 0:
    while( hDistance < 100 )
    {
      updateIR();
      motorControl.forward(0.5);
      delay(10);
    }
    motorControl.reverse(1);
    delay(15);
    motorControl.fullStop();

    moveArm(180);
    //TODO: go forward
    faceLeft();
    break;

  case 1:
    currentDirection = currentDirection%4;
    switch(currentDirection)
    {
    case 1:

      faceRight();
      break;
    case 2:
      faceRight();
      faceRight();
      break;
    case 3:
      do
      {
        updateUltrasonics();
        float error = leftDistance - rightDistance;
        float percentError = CORRECTION_GAIN * 2 * error / (leftDistance + rightDistance);
        motorControl.reverse(1 + percentError, 1 - percentError);
      }
      while( leftDistance < 150 || rightDistance < 150);
      break;
    }
  }
  state++;
}

void findRamp()
{
  motorControl.fullStop();
  delay(500);
  hDistance = 10;
  do
  {
    updateUltrasonics();
    updateIR();
    //TODO: If right margin is wrong, perform S turn

    float error = rightDistance - 8.5;
    float percentError = error/10;
    if(percentError > 0.2)
    {
      percentError = 0.2;
    }
    motorControl.forward(0.7 + percentError, 0.7 - percentError);
    Serial.print(rightDistance);
    Serial.print('\t');
    Serial.println(hDistance);

    delay(20);
  }
  while(hDistance < 100);
  motorControl.reverse(0.3);
  delay(500);
  motorControl.fullStop();
  moveArm(60);
  motorControl.forward(0.4);
  delay(1000);
  moveArm(135);
  motorControl.forward(1);
  delay(2000);
  moveArm(100);

  state++;
}

void climbRamp()
{
  motorControl.forward(1);
  delay(1000);
  int pinState = 0;
  int mounted = 0;
  while(mounted == 0 || pinState == 0)
  {
    updateUltrasonics();
    int currentPin = digitalRead(TILT_PIN);
    if(currentPin != pinState)
    {
      bool change = true;
      for(int i = 0; i < 20 && change == true; i++)
      {
        change = (currentPin == digitalRead(TILT_PIN));
        delay(15);
      }
    }
    if(pinState == 1 ) //state switch 0
    {
      motorControl.forward(0.8);
      if(cliffDistance > 10 && cliffDistance != 0)
      {
        delay(10000);
        moveArm(180);
      }
      else{ 
        moveArm(180); 
      }    
    }
    /*else if(pinState == 0) //state switch 1
     {
     side = 1;
     mounted = 1;
     motorControl.reverse(0.2);
     moveArm(30);
     }*/
    delay(50);
  } 
  moveArm(150);
  motorControl.forward(1);
  delay(25);  
  faceLeft();
  state++; 
}

void findBase()
{
  Serial.println("Enter findbase");
  moveArm(180);
  for(int i = 0; i < 10; i++)
    {
      updateUltrasonics();
      updateIR();
      delay(50);
    }
  motorControl.forward(0.7);
  int baseFound = 0;
  do
  {
    
    updateUltrasonics();
    updateIR();

    
    Serial.print(leftDistance);
    Serial.print('\t');
    Serial.print(rightDistance);
    Serial.print('\t');
    Serial.print(vDistance);
    Serial.print('\t');
    Serial.println(hDistance);
    if(leftDistance < 100 || rightDistance < 100 || vDistance > 100 )
    {

      Serial.println("base found");
      motorControl.fullStop();

      if(leftDistance < 100)
      { 
        faceLeft();
      }
      if(rightDistance < 100)
      {
        faceRight();
      }
      delay(250);
      while(vDistance < 100)
      {
        updateIR();
        motorControl.forward(0.7);
        Serial.print(vDistance);
        Serial.print('\t');
        Serial.println(hDistance);
      }
      baseFound = 1;
      delay(15);

    }

  }
  while(baseFound == 0 );//TODO check vertical distance
    motorControl.fullStop();
  state++;
}

void findLarry()
{
  faceLeft();
  delay(250);
  
  currentDirection = currentDirection % 4;
  do
  {
    while(updateUltrasonics() == 0)
      {delay(10);}
    motorControl.reverse(0.2);
  }while( rightDistance < 45);
  unsigned long startTime = millis();
  motorControl.fullStop();
  do
  {
    while(updateUltrasonics() == 0)
      {delay(10);}
    motorControl.forward(0.2);
  }while(rightDistance >45);
  do
  {
    while(updateUltrasonics() == 0)
      {delay(10);}
    motorControl.forward(0.2);
  }while(rightDistance < 45);
  unsigned long averageTime = (millis() - startTime)/2;
  motorControl.fullStop();
  motorControl.reverse(0.2);
  delay(averageTime - 120);
  motorControl.forward(0.2);
  delay(100);
  motorControl.fullStop();

  faceRight();
  delay(250);  
}

void getLarry()
{
  moveArm(80);
  motorControl.forward(0.3);
  delay(2000);
  motorControl.fullStop();
  moveArm(180);
  moveArm(90);
  moveArm(180);
  moveArm(90);
  delay(50);
  state = 0;
}

void faceLeft()
{
  currentDirection++;
  motorControl.turnLeft();
  delay(NINETY_DEGREE_DELAY);
  motorControl.fullStop();

}

void faceRight()
{
  currentDirection--;
  motorControl.turnRight();
  delay(NINETY_DEGREE_DELAY);
  motorControl.fullStop();

}

void moveArm( int newAngle )
{
  int currentAngle = armServo.read();
  if( currentAngle > newAngle )
  {
    for(currentAngle; currentAngle >= newAngle; currentAngle--)
    {
      armServo.write(currentAngle);
      delay(15);
    }
  }
  else if(currentAngle < newAngle)
  {
    for(currentAngle; currentAngle <= newAngle; currentAngle++)
    {
      armServo.write(currentAngle);
      delay(15);
    }
  }
}

void updateIR()
{
  float hStDev, vStDev, h, v;
  hStDev = 0;
  vStDev = 0;  
  h = 0;
  v = 0;

  float hBuff[BUFFER2_SIZE];
  float vBuff[BUFFER2_SIZE];

  for(int i = 0; i < BUFFER2_SIZE; i++)
  {
    hBuff[i] = analogRead(HORIZONTAL_IR_PIN);
    vBuff[i] = analogRead(VERTICAL_IR_PIN); 
    //Serial.print(hBuff[i]);
    //Serial.print('\t');
    h += hBuff[i];
    v += vBuff[i];
    delay(10);
  }
  //Serial.println();
  h = h/BUFFER2_SIZE;
  v = v/BUFFER2_SIZE;


  for(int i = 0; i < BUFFER2_SIZE; i++)
  {
    hStDev += pow(hBuff[i] - h, 2);
    vStDev += pow(vBuff[i] - v, 2);
  }


  hStDev = sqrt(hStDev/BUFFER2_SIZE);
  vStDev = sqrt(vStDev/BUFFER2_SIZE);


  //Serial.print(h);
  //Serial.print('\t');
  // Serial.println(hStDev);
  //  Serial.print('\t');

  if(hStDev < 20){ 
    hDistance = h; 
  }
  if(vStDev < 20){ 
    vDistance = v; 
  }

}

int updateUltrasonics()
{
  float leftStDev, rightStDev, left, right;
  long leftusec, rightusec;

  switch(state)
  {
    //detect the base
  case 2: //using right placeholder for the cliff because... why not
    rightusec = cliffSonic.timing(); 
    right = cliffSonic.convert(rightusec, Ultrasonic::CM);
    rightStDev = cliffSonic.unbiasedStdDev(right, BUFFER_01);

    if(rightStDev < 10)
    {
      cliffDistance = right;
      return 1;
    }
    return 0;
    //    case 4:;
    //    case 5:;
    //
    //    case 0:;
    //    case 1: 
    //      /*    leftusec = frontLeftSonic.timing();
    //       left = frontLeftSonic.convert(leftusec, Ultrasonic::CM);
    //       leftStDev = frontLeftSonic.unbiasedStdDev(left, BUFFER_01);
    //       delay(10);
    //       rightusec = frontRightSonic.timing();
    //       right = frontRightSonic.convert(rightusec, Ultrasonic::CM);
    //       rightStDev = frontRightSonic.unbiasedStdDev(right, BUFFER_01);
    //       
    //       if(rightStDev < 10 && leftStDev < 10)
    //       {
    //       rightDistance = right;
    //       leftDistance = left;
    //       }
    //       break;*/
    //      //find the ramp
    //
    //      //detect the peak
    //    case 3:;
  default:  
    leftusec = leftSonic.timing();
    left = leftSonic.convert(leftusec, Ultrasonic::CM);
    leftStDev = leftSonic.unbiasedStdDev(left, BUFFER_01);
    delay(10);
    rightusec = rightSonic.timing();
    right = rightSonic.convert(rightusec, Ultrasonic::CM);
    rightStDev = rightSonic.unbiasedStdDev(right, BUFFER_01);

    if(rightStDev < 30 && leftStDev < 30)
    {
      rightDistance = right;
      leftDistance = left;
      return 1;
    }
    return 0;
  }
}






