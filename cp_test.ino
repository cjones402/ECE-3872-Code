#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "protothreads.h"

#define LEDA 13  // The pin the LED is connected to
#define LEDB 12 // The pin the second LED is connected to
#define INBUTTON 4 // The pin to take the button input
#define RESET 3

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

Servo myServo;
int servo_position = 0 ;

int potVal;
int outputVal;

pt ptDC;
pt ptServo;

bool startPressed = false;

// Code to run servos back and forth
int runServo(struct pt* pt) {
  PT_BEGIN(pt);
  digitalWrite(LEDA, HIGH); // Turn the LED on
  for (servo_position = 0; servo_position <=180; servo_position +=1){
       myServo.write(servo_position);
       delay(10);
  }

  for (servo_position=180; servo_position >= 0; servo_position -=1){
    myServo.write(servo_position);
    delay(10);
  }
  digitalWrite(LEDA, LOW); // Turn the LED off
  PT_END(pt);
}

// Code to run dc motors backwards and forwards
int runDcMotor(struct pt* pt) {
  PT_BEGIN(pt);
  uint8_t i;
  digitalWrite(LEDB, HIGH); // Turn the LED on
  //Serial.print("tick");

  myMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);  
    delay(10);
  }
  
  //Serial.print("tock");

  myMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);  
    delay(10);
  }
  digitalWrite(LEDB, LOW); // Turn the LED off
  PT_END(pt);
}

/* the function */
void buttonWait(int buttonPin){
  int buttonState = 0;
  while(1){
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {
      startPressed = true;
      return;
    }
  }
}

void setup() {
  pinMode(LEDA, OUTPUT); // Declare the LED as an output
  pinMode(LEDB, OUTPUT); // Declare the LED as an output
  //pinMode(11, OUTPUT);
  pinMode(INBUTTON, INPUT_PULLUP); 
  pinMode(A0, INPUT);  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  myServo.attach(10);
  PT_INIT(&ptDC);
  PT_INIT(&ptServo);
}

void loop() {
  if (!startPressed) {
     buttonWait(4); 
  }
      potVal = analogRead(A0);
      PT_SCHEDULE(runDcMotor(&ptDC));
      PT_SCHEDULE(runServo(&ptServo));
      outputVal = map(potVal, 0, 1023, 1,8); //Convert from 0-1023 proportional to the number of a number of from 1 to 8
}
