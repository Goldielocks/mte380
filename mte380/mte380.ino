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
#define BUFFER_SIZE 10

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
  /*rightDistance = analogRead(HORIZONTAL_IR_PIN);
   Serial.println(rightDistance); */
  initialPosition();
  findRamp();
  //climbRamp();
  //findBase();
  //findLarry();
  //getLarry();
  while(1){
    delay(500);
  }

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
    float percentError = error/5;
    if(percentError > 0.2)
    {percentError = 0.2;}
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
  delay(500);  
  faceLeft();
  state++; 
}

void findBase()
{
  moveArm(180);
  int baseFound = 0;
  while(baseFound == 0)//TODO check vertical distance
  {
    updateUltrasonics();
    if(leftDistance < 130 || rightDistance < 130 )
    {
      motorControl.forward(1);
      delay(300);
      motorControl.fullStop();

      baseFound == 1;

      if(leftDistance < 130)
      { 
        faceRight();
      }
      if(rightDistance < 130)
      {
        faceRight();
      }

      float error = leftDistance - rightDistance;
      float percentError = CORRECTION_GAIN * 2 * error / (leftDistance + rightDistance);
      motorControl.forward(1 - percentError, 1 + percentError);

      motorControl.fullStop();
    }

    delay(50);
  }

  motorControl.forward(1);
  state++;
}

void findLarry()
{
  int larryFound = 0;
  while( larryFound == 0)
  {
    faceLeft();
  }
  delay(50);
  //state++;
}

void getLarry()
{
  moveArm(180);
  motorControl.reverse(0.3);
  delay(200);
  motorControl.fullStop();
  moveArm(150);
  motorControl.forward(0.3);
  delay(200);
  motorControl.fullStop();
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

  float hBuff[BUFFER_SIZE];
  float vBuff[BUFFER_SIZE];

  for(int i = 0; i < BUFFER_SIZE; i++)
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
  h = h/BUFFER_SIZE;
  v = v/BUFFER_SIZE;


  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    hStDev += pow(hBuff[i] - h, 2);
    vStDev += pow(vBuff[i] - v, 2);
  }


  hStDev = sqrt(hStDev/BUFFER_SIZE);
  vStDev = sqrt(vStDev/BUFFER_SIZE);


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

void updateUltrasonics()
{
  float leftStDev, rightStDev, left, right;
  long leftusec, rightusec;

  switch(state)
  {
  case 0:;
  case 1: 
    /*    leftusec = frontLeftSonic.timing();
     left = frontLeftSonic.convert(leftusec, Ultrasonic::CM);
     leftStDev = frontLeftSonic.unbiasedStdDev(left, BUFFER_01);
     delay(10);
     rightusec = frontRightSonic.timing();
     right = frontRightSonic.convert(rightusec, Ultrasonic::CM);
     rightStDev = frontRightSonic.unbiasedStdDev(right, BUFFER_01);
     
     if(rightStDev < 10 && leftStDev < 10)
     {
     rightDistance = right;
     leftDistance = left;
     }
     break;*/
    //find the ramp

    //detect the peak
  case 3:    
    leftusec = leftSonic.timing();
    left = leftSonic.convert(leftusec, Ultrasonic::CM);
    leftStDev = leftSonic.unbiasedStdDev(left, BUFFER_01);
    delay(10);
    rightusec = rightSonic.timing();
    right = rightSonic.convert(rightusec, Ultrasonic::CM);
    rightStDev = rightSonic.unbiasedStdDev(right, BUFFER_01);

    if(rightStDev < 10 && leftStDev < 10)
    {
      rightDistance = right;
      leftDistance = left;
    }
    break;
    //detect the base
  case 2: //using right placeholder for the cliff because... why not
    rightusec = cliffSonic.timing(); 
    right = cliffSonic.convert(rightusec, Ultrasonic::CM);
    rightStDev = cliffSonic.unbiasedStdDev(right, BUFFER_01);

    if(rightStDev < 10)
    {
      cliffDistance = right;
    }
    break;
  case 4:;
  case 5:;
    //detect the base edge      
  }
}


