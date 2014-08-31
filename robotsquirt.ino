//  Squirt Gun Robot 2014 *PeterFarell*
// works with simple key fob 4 button transmitter. connected to pins A0-A3 
// A4 is reserved for the sharp Ir sensor for object detection
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>

int pos = 0;  // left over variable for testing
int ms = 255; // motor speed going straight
int ts = 150; // motor turn speed
int ang = 70; // servo +- angle max 90

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);  
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(2);
Servo servot;

// ir: the pin where your sensor is attached
// 15: the number of readings the library will make before calculating a mean distance
// 45: the difference between two consecutive measurements to be taken as valid
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y

int maindelay = 100;  // set for regual use
int backupdelay = 1000; // back up for one second when avoiding wall
int mainturndelay = 80;  // set for when buton is held down
int backupturndelay = 800; /// set to turn about 90 degrees
int IR_SENSOR = 0; // Sensor is connected to the analog A0
int intSensorResult = 0; //Sensor result
float fltSensorCalc = 0; //Calculated value
int led = 9;

void goForward(int mf) 
{
    digitalWrite(led, HIGH);
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);

    myMotor->setSpeed(mf-9); // -10 because this motor is faster than myMotor2
    myMotor2->setSpeed(mf); 
    delay (100);
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    
}

void goBackward(int mb, int del)
{
   digitalWrite(led, HIGH);
   Serial.print("backward");
    myMotor->run(BACKWARD);
    myMotor2->run(BACKWARD);

    myMotor->setSpeed(mb-7); // -10 because this motor is faster than myMotor2
    myMotor2->setSpeed(mb);
    delay (del);
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    
   
}

void goTurn(int gt, int td)
{
   digitalWrite(led, HIGH);
   Serial.print("tight turn");
    myMotor->run(BACKWARD);
    myMotor2->run(FORWARD);

    myMotor->setSpeed(gt);
    myMotor2->setSpeed(gt);
    delay (td);
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    
  
}

void goSquirt(int sq)
{
     digitalWrite(led, HIGH);    
    Serial.print("squirt");
    for (int i=91-sq; i <=89+sq; i++){
      myMotor3->run(FORWARD);
      delay(5);
      servot.write(i);}
      myMotor3->run(RELEASE);
      servot.write(90);
      return;
      
}


void setup() {
 // Serial.begin(9600);           // set up Serial library at 9600 bps
 // Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  servot.attach(10);
  
  myMotor->setSpeed(100);  // setting up 3 motors (1,4,2) {2 (myMotor3) is the squirt motor}
  myMotor->run(FORWARD);
  myMotor2->setSpeed(100);
  myMotor2->run(FORWARD);
  myMotor3->setSpeed(255);
  myMotor3->run(FORWARD);
  myMotor3->run(RELEASE); // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  int ms = 255; // motor speed
  int ts = 180; // turn speed
  int ang = 45; // servo +- angle max 90
  pinMode(led, OUTPUT);
 
  
  

}
// motor 1 is faster
void loop() {
  uint8_t i;
  int sensorValue = analogRead(A0);
  int sensorValue2 = analogRead(A1);
  int sensorValue3 = analogRead(A2);
  int sensorValue4 = analogRead(A3);
  
  intSensorResult = analogRead(IR_SENSOR); //Get sensor value
  fltSensorCalc = (6787.0 / (intSensorResult - 3.0)) - 4.0; //Calculate distance in cm

  
  //Serial.print(fltSensorCalc); //Send distance to computer
  //Serial.println(" cm"); //Add cm to result
  //Serial.println(sensorValue);
  //Serial.println(sensorValue2);
  //Serial.println(sensorValue3);
  //Serial.println(sensorValue4);

  
 
  if (sensorValue > 1000){
   goForward(ms);
   Serial.println("goFoward");
    digitalWrite(led, LOW);
  }
  
  if (sensorValue2 > 1000){
   goBackward(ms, maindelay);
   Serial.println("goBackward");
    digitalWrite(led, LOW);
  }
  
  if (sensorValue4 > 1000){  //  squirt in three directions... 
    goSquirt(ang);  
    Serial.println("squirtAway"); 
     digitalWrite(led, LOW);
  }
  
  if (sensorValue3 > 1000){
   goTurn(ts, mainturndelay);
   Serial.println("turning");
    digitalWrite(led, LOW);
  }

}


