#include <Wire.h>                   //Include required headers
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

//Give names to input pins
int leftForward = 3;
int leftBackward = 4;
int rightForward = 5;
int rightBackward = 6;

int pot = 2;

//Declare variables to hold speeds
int leftSpeed = 0;
int rightSpeed = 0;

//Variables to hold last sesen state of each side
int leftPreviousState = 3;
int rightPreviousState = 3;

//Vars for when that state started
unsigned long leftStartTime = 0;
unsigned long rightStartTime = 0;

//Declare switch state variables
int leftForwardState = 0;
int leftBackwardState = 0;
int rightForwardState = 0;
int rightBackwardState = 0;

int potValue = 0;



void setup() {                
  // start Serial
  Serial.begin(9600);
   AFMS.begin();
  //Set pins as inputs
  pinMode(leftForward, INPUT); 
  pinMode(leftBackward, INPUT);
  pinMode(rightForward, INPUT);
  pinMode(rightBackward, INPUT);
  
  pinMode(pot, INPUT);
  
}

void loop() {

  readValues();
  adjustMotors();
  printValues();
  delay(100);
}

void readValues(){
  //read in values from input pins
  leftForwardState = digitalRead(leftForward);
  leftBackwardState = digitalRead(leftBackward);
  rightForwardState = digitalRead(rightForward);
  rightBackwardState = digitalRead(rightBackward);
  
  potValue = analogRead(pot);
}

void adjustMotors(){
  
  if(leftForwardState){
    
    if(leftPreviousState == 0){
      if(millis() < leftStartTime + 1000){
        leftSpeed = startSpeed;
      }
      else if(millis() > 2000 + leftStartTime){
        leftSpeed = maxSpeed;
      }
      else {
        leftSpeed = map(millis() - leftStartTime - 500, 0, 1000, 0, maxSpeed - startSpeed);
      }
    }
    else{
      leftPreviousState = 0;
      leftStartTime = millis();
      leftSpeed = startSpeed;
    }
    leftFrontMotor->run(FORWARD);
    leftBackMotor->run(FORWARD);
    
    leftFrontMotor->setSpeed(leftSpeed);
    leftBackMotor->setSpeed(leftSpeed);
    
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
        leftSpeed = map(millis() - leftStartTime - 500, 0, 1000, 0, maxSpeed - startSpeed);
      }
    }
    else{
      leftPreviousState = 1;
      leftStartTime = millis();
      leftSpeed = startSpeed;
    }
    leftFrontMotor->run(BACKWARD);
    leftBackMotor->run(BACKWARD);
    
    leftFrontMotor->setSpeed(leftSpeed);
    leftBackMotor->setSpeed(leftSpeed);
  }
  else {
    leftFrontMotor->run(RELEASE);
    leftBackMotor->run(RELEASE);
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
        rightSpeed = map(millis() - rightStartTime - 500, 0, 1000, 0, maxSpeed - startSpeed);
      }
    }
    else{
      rightPreviousState = 0;
      rightStartTime = millis();
      rightSpeed = startSpeed;
    }
    rightFrontMotor->run(FORWARD);
    rightBackMotor->run(FORWARD);
    
    rightFrontMotor->setSpeed(rightSpeed);
    rightBackMotor->setSpeed(rightSpeed);
    
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
        rightSpeed = map(millis() - rightStartTime - 500, 0, 1000, 0, maxSpeed - startSpeed);
      }
    }
    else{
      rightPreviousState = 1;
      rightStartTime = millis();
      rightSpeed = startSpeed
      ;
    }
    rightFrontMotor->run(BACKWARD);
    rightBackMotor->run(BACKWARD);
    
    rightFrontMotor->setSpeed(rightSpeed);
    rightBackMotor->setSpeed(rightSpeed);
  }
  else {
    rightFrontMotor->run(RELEASE);
    rightBackMotor->run(RELEASE);
    rightPreviousState = 3;
  }
  
}

void printValues(){
  Serial.print(leftForwardState);
  Serial.print(", ");
  Serial.print(leftBackwardState);
  Serial.print(", ");
  Serial.print(rightForwardState);
  Serial.print(", ");
  Serial.print(rightBackwardState);
  Serial.println(", ");
}
