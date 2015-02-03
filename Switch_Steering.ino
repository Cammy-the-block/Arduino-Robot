#include <Wire.h>
#include <Servo.h> //Include required headers
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

//Attach motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//define motors

Adafruit_DCMotor *leftFrontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftBackMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightFrontMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightBackMotor = AFMS.getMotor(4);

const int startSpeed = 80;
const int maxSpeed = 255;
//                           [0]  [1]     [2]       [3]      [4]    [5]
//                           pin, pin, lowlimit, highlimit, speed, servo pin
const int servoData [][6] = {{7,    8,        55,      117,    25,   9},  //shovel lift     [0]
                            {2,     3,         0,      180,   200,  10},  //paw open close  [1]
                            {4,     5,         0,      180,   200,  11},  //paw arm         [2]
                            {12,   13,         0,      180,   200,  6}}; //shovel tilt      [3]
                         
int servoPosition [] = {90, 90, 120, 90};

int servoState [] = {0, 0, 0, 0};

unsigned long servoLastAdjust[] = {0, 0, 0, 0};
                         
Servo servos[4];

//Give names to input pins
int leftForward = 14;
int leftBackward = 15;
int rightForward = 16;
int rightBackward = 17;

//Declare variables to hold speeds
int leftSpeed = 0;
int rightSpeed = 0;

//Variables to hold last sesen state of each side
int leftPreviousState = 3;
int rightPreviousState = 3;

int pawState = 0;

//Vars for when that state started
unsigned long leftStartTime = 0;
unsigned long rightStartTime = 0;

//Declare switch state variables
int leftForwardState = 0;
int leftBackwardState = 0;
int rightForwardState = 0;
int rightBackwardState = 0;




void setup() {  
  Serial.begin(9600);            
  // start Serial
   AFMS.begin();
  //Set pins as inputs
  pinMode(leftForward, INPUT); 
  pinMode(leftBackward, INPUT);
  pinMode(rightForward, INPUT);
  pinMode(rightBackward, INPUT);
  
  
  for(int x = 0; x < 4; x++){
    servos[x].attach(servoData[x][5]); 
  }
  
}

void loop() {
  readValues(); //read switch values
  computeMotorSpeeds(); //compute motor speeds
  adjustServos();
  adjustMotors();
  printValues();
  delay(1);
}

void readValues(){
  //read in values from input pins
  leftForwardState = digitalRead(leftForward);
  leftBackwardState = digitalRead(leftBackward);
  rightForwardState = digitalRead(rightForward);
  rightBackwardState = digitalRead(rightBackward);
  
  for(int x = 0; x < 4; x++){
    
    if(digitalRead(servoData[x][0])){
      servoState[x] = 1;
    }
    else if(digitalRead(servoData[x][1])){
      servoState[x] = -1;
    }
    else{
      servoState[x] = 0;
    }
  }
}

void computeMotorSpeeds(){
  
  if(leftForwardState){
    if(leftPreviousState == 0){
      if(millis() < leftStartTime + 1000){
        leftSpeed = startSpeed;
      }
      else if(millis() > 2000 + leftStartTime){
        leftSpeed = maxSpeed;
      }
      else {
        leftSpeed = map(millis() - leftStartTime - 1000, 0, 1000, startSpeed, maxSpeed);
      }
    }
    else{
      
      leftPreviousState = 0;
      
      if(rightPreviousState != 2){
        leftStartTime = rightStartTime;
      }
      else{
        leftStartTime = millis();
        leftSpeed = startSpeed;
      }
      
    }
    
  }
  else if(leftBackwardState){
    if(leftPreviousState == 1){
      if(millis() < leftStartTime + 1000){
        leftSpeed = startSpeed;
      }
      else if(millis() > 2000 + leftStartTime){
        leftSpeed = maxSpeed;
      }
      else {
        leftSpeed = map(millis() - leftStartTime - 1000, 0, 1000, startSpeed, maxSpeed);
      }
    }
    else{
      leftPreviousState = 1;
      if(rightPreviousState != 2){
        leftStartTime = rightStartTime;
        
      }
      else{
        leftStartTime = millis();
        leftSpeed = startSpeed;
      }
    }
  }
  else {

    leftPreviousState = 2;
  }
  
  if(rightForwardState){
    
    if(rightPreviousState == 0){
      if(millis() < 1000 + rightStartTime){
        rightSpeed = startSpeed;
      }
      else if( millis() > 2000 + rightStartTime){
        rightSpeed = maxSpeed;
      }
      else {
        rightSpeed = map(millis() - rightStartTime - 1000, 0, 1000, startSpeed, maxSpeed);
      }
    }
    else{
      rightPreviousState = 0;
      if(leftPreviousState != 2){
        rightStartTime = leftStartTime;
      }
      else{
        rightStartTime = millis();
        rightSpeed = startSpeed;
      }
    }
    
  }
  else if(rightBackwardState){
   
    if(rightPreviousState == 1){
      if(millis() < 1000 + rightStartTime){
        rightSpeed = startSpeed;
      }
      else if(millis() > 2000 + rightStartTime){
        rightSpeed = maxSpeed;
      }
      else {
        rightSpeed = map(millis() - rightStartTime - 1000, 0, 1000, startSpeed, maxSpeed);
      }
    }
    else{
      rightPreviousState = 1;
      if(leftPreviousState != 2){
        rightStartTime = leftStartTime;
      }
      else{
        rightStartTime = millis();
        rightSpeed = startSpeed;
      }
      
    }
  }
  else {
    rightPreviousState = 2;
  }
  
}

void adjustServos(){
  for(int x = 0; x < 4; x++){
    servoPosition[x] = computeServoSpeed(servoPosition[x], servoData[x][2], servoData[x][3], servoState[x], servoLastAdjust[x], servoData[x][4]);
    servos[x].write(servoPosition[x]);
    //Serial.println(servoPosition[x]);
  }
}

int computeServoSpeed(int position, const int lowerLimit, const int higherLimit, int state, unsigned long& lastAdjust, const int speed){
  if (position > lowerLimit && state == -1 && lastAdjust + (1000 / speed) < millis()){
    position -= 1;
    lastAdjust = millis();
  }
  if (position < higherLimit && state == 1 && lastAdjust + (1000 / speed)< millis()){
    position += 1;
    lastAdjust = millis();
  }
  return position;
}

void adjustMotors(){
  switch (leftPreviousState){
    case 0:
      leftFrontMotor->run(FORWARD);
      leftBackMotor->run(FORWARD);
      break;
    case 1:
      leftFrontMotor->run(BACKWARD);
      leftBackMotor->run(BACKWARD);
      break;
    default:
      leftFrontMotor->run(RELEASE);
      leftBackMotor->run(RELEASE);
  }
  
  switch (rightPreviousState){
    case 0:
      rightFrontMotor->run(FORWARD);
      rightBackMotor->run(FORWARD);
      break;
    case 1:
      rightFrontMotor->run(BACKWARD);
      rightBackMotor->run(BACKWARD);
      break;
    default:
      rightFrontMotor->run(RELEASE);
      rightBackMotor->run(RELEASE);
  }
  
  leftFrontMotor->setSpeed(leftSpeed);
  leftBackMotor->setSpeed(leftSpeed);
  rightFrontMotor->setSpeed(rightSpeed);
  rightBackMotor->setSpeed(rightSpeed);
  
}

void printValues(){
  Serial.println(servoPosition[0]);
  /*
  Serial.print(leftForwardState);
  Serial.print(", ");
  Serial.print(leftBackwardState);
  Serial.print(", ");
  Serial.print(rightForwardState);
  Serial.print(", ");
  Serial.print(rightBackwardState);
  Serial.println(", ");
  */
  
}
