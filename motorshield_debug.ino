#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "protothreads.h"

Servo myServo;
Servo myServoTwo;
Servo myServoThree;
double servo_position = 0 ;
pt ptServo;
bool startPressed = false;
bool testPressed = false;
bool isPlaying = false;
double tempo = 0;
bool detectedSong = false;
int i_note_index = 0;
int duration;
unsigned long timeVal;
//Button State Definitions
int PlayModeButtonState = 0;
int TestModeButtonState = 0;
int lastRead = 0;
int lastLastRead = 0;
int currRead = 0;
int lastReadStart = 0;
int currReadStart = 0;
int Octive = 0;
int potPin = 0;
int potVal = 0;
// Definitions
#define LEDA 5
#define LEDB 4
#define LEDC 3
#define ANALOG_PIN A1 
//#define ENVELOPE_PIN 97 
#define AUDIO_OUT 8 
//#define Octive 6
#define potPin 1


// UI BOARD PINOUTS
// PIN 5 - Red LED
// PIN 4 - Orange LED
// PIN 3 - Yellow LED
// A0 - Potentiometer
// PIN 2 - BUTTON PLAY
// PIN 10 - BUTTON TEST
// 5V - White Jumper with 2 lines
// GND - White Jumper with 1 line

// Taken from Prof. Brother's Conductor code to play using a speaker using tone
// Music Notes based on Octive--
double C = 16.3516*pow(2,Octive);
double D = 18.35405*pow(2,Octive);
double E = 20.60172*pow(2,Octive);
double F = 21.82676*pow(2,Octive);
double G = 24.49971*pow(2,Octive);
double A = 27.5*pow(2,Octive);
double B = 30.86771*pow(2,Octive);
double high_C = 32.70320*pow(2,Octive);
#define rest 0
int newOctive = 6;
  
//Row Row Row Your Boat
int songLength = 54;  
double notes[] = {C, rest, C, rest, C, rest, D, rest, E, rest, E, rest, D, rest, E, rest, F, rest, G, rest, high_C, rest, high_C, rest, high_C, rest, G, rest, G, rest, G, rest, E, rest, E, rest, E, rest, C, rest, C, rest, C, rest, G, rest, F, rest, E, rest, D, rest, C, rest};
int beats[] = {2,1,2,1,2,1,1,1,2,1,2,1,1,1,2,1,1,1,6,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,2,1,1,1,5,1};

double getTempo() { 
  int numSamples = 8; // number of tempo samples to collect 
  boolean timerRunning = false; // is the timer running 
  boolean hasFirstSignal = false; 
  unsigned long FirstDetectionTime; 
  unsigned long startTime = 0; // start time in milliseconds 
  unsigned long stopTime = 0; // stop time in milliseconds 
  double duration = 0.0; 
  const int numReadings = 20; // number of readings to average over 
  int readings[numReadings]; // array of past readings. 
  int readIndex = 0;              // the index of the current reading 
  int total = 0;                  // the running total 
  int average = 0;                // the average 
  double scaleBy = 1.0; 
  int threshold = 40; 
  boolean active = false; 
  boolean IsReady = true; 
  int audioMax = 40; 
  double durAve; 
  double numDurs = 0; 
  int count;

  Serial.println("Temp function started!");

  for (int thisReading = 0; thisReading < numReadings; thisReading++) { 
    readings[thisReading] = 0; 
  } 

  while(numDurs < numSamples) { 
    delay(1); 
    Serial.println(analogRead(A1));
    // SMOOTHING FILTER 
    total = total - readings[readIndex]; 
    readings[readIndex] = analogRead(A1); 
    total = total + readings[readIndex]; 
    readIndex = readIndex + 1; 
    if (readIndex >= numReadings) { 
      readIndex = 0; 
    } 
    average = total / numReadings; 
    
    // check max 
    if(average > audioMax) { 
      audioMax = average; 
    } 

    if(IsReady) { // If we are ready to detect a note 
      // if a note is detected 
      if ( !active && average > threshold || active && average > 0.7 * audioMax ) { 
        if(!hasFirstSignal) { 
          hasFirstSignal = true; 
          Serial.println("First signal");
          FirstDetectionTime = millis(); 
        } 
        if(!active){ 
          startTime = millis(); 
          if(stopTime > 0.0){
            duration = (double)startTime-stopTime; 
            Serial.print("duration: "); 
            Serial.println(duration*scaleBy); 
            if(duration > 100 && duration < 650){ 
              durAve = ((durAve*numDurs + duration*scaleBy)/(numDurs+1)); 
              numDurs++;
              Serial.print("duration average: "); 
              Serial.println(durAve); 
             } 
          }
          active = true; 
          //Serial.println(average); 
        } 
      } else { 
        if(active) { 
          IsReady = false; 
          active = false; 
          stopTime = millis();
          
        } 
      } 
    } else { 
      if(average < threshold || (average < 70 && average < 0.2*audioMax)) { 
        if( 0.2*audioMax > threshold ) { 
          if (0.2*audioMax < 70){ 
             threshold = 0.2*audioMax; 
          } else { 
            threshold = 70; 
          } 
        } 
        IsReady = true; 
        audioMax = 40; 
      } 
    } 
  } 
  Serial.println("Delaying for output!");
  Serial.println((durAve*71)-(millis()-FirstDetectionTime));
  if (((durAve*71)-(millis()-FirstDetectionTime)) < 0) {
      return durAve; 
  }
  delay((durAve*71)-(millis()-FirstDetectionTime)); 
  detectedSong = true;
  return durAve; 
} 

int runServo(struct pt* pt) {
  PT_BEGIN(pt);
  for (servo_position = 0; servo_position <=120; servo_position +=1){
       myServo.write(servo_position);
       myServoTwo.write(servo_position);
       myServoThree.write(servo_position);
       delay(10);
  }

  for (servo_position=120; servo_position >= 0; servo_position -=1){
    myServo.write(servo_position);
    myServoTwo.write(servo_position);
    myServoThree.write(servo_position);
    delay(10);
  }
  PT_END(pt);
}

int buttonPressed(uint8_t button) {
  static uint16_t lastStates = 0;
  uint8_t state = digitalRead(button);
  if (state != ((lastStates >> button) & 1)) {
    lastStates ^= 1 << button;
    return state == HIGH;
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
  //AFMS.begin();  // create with the default frequency 1.6KHz
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDC, OUTPUT);
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDC, LOW);
  pinMode(2, INPUT);
  pinMode(12, INPUT);
  pinMode(ANALOG_PIN, INPUT);
  pinMode(AUDIO_OUT, OUTPUT);
  //pinMode(potPin, INPUT);
  
  myServo.attach(10);
  myServoTwo.attach(9);
  myServoThree.attach(11);
  //PT_INIT(&ptServo);
  Serial.begin(9600);
}

void loop() {
  // Code for the potentiometer
  potVal = analogRead(A0);
  //Serial.print(potVal);
  //Serial.print("\n");
  Octive = map(potVal, 0, 1023, 3, 0);
//  Serial.print(octave);
//  Serial.print("\n");
  if (Octive <= 1) {
    digitalWrite(LEDC, HIGH);
    newOctive = 5;
  } else {
    digitalWrite(LEDC, LOW);
  }
  if (Octive == 2) {
    digitalWrite(LEDB, HIGH);
    newOctive = 6;
  } else {
    digitalWrite(LEDB, LOW);
  }
  if (Octive >= 3) {
    digitalWrite(LEDA, HIGH);
    newOctive = 7;
  } else {
    digitalWrite(LEDA, LOW);
  }
  //Serial.println(digitalRead(2));
  lastReadStart = currReadStart;
  currReadStart = digitalRead(2);
  if (lastReadStart == 1|| (startPressed && !testPressed)) {
    if(currReadStart == 0 || startPressed){
      startPressed = true;
      testPressed = false;
      //digitalWrite(LEDA, HIGH);
    Serial.println("Temp Match Started..."); 
    C = 16.3516*pow(2,Octive + 4);
    D = 18.35405*pow(2,Octive + 4);
    E = 20.60172*pow(2,Octive + 4);
    F = 21.82676*pow(2,Octive + 4);
    G = 24.49971*pow(2,Octive + 4);
    A = 27.5*pow(2,Octive + 4);
    B = 30.86771*pow(2,Octive + 4);
    high_C = 32.70320*pow(2,Octive + 4);
    double notes[] = {C, rest, C, rest, C, rest, D, rest, E, rest, E, rest, D, rest, E, rest, F, rest, G, rest, high_C, rest, high_C, rest, high_C, rest, G, rest, G, rest, G, rest, E, rest, E, rest, E, rest, C, rest, C, rest, C, rest, G, rest, F, rest, E, rest, D, rest, C, rest};
    if(!detectedSong) {
      delay(1000);
      tempo = getTempo();
    }
    if(!isPlaying){
       duration = beats[i_note_index] * tempo;
       tone(AUDIO_OUT, notes[i_note_index]);
       timeVal = millis();
       isPlaying = true;
    }
    tone(AUDIO_OUT, notes[i_note_index]);
    if((millis() - timeVal) > duration){
      isPlaying = false;
      if((i_note_index + 1) > 53) {
          i_note_index = 0;
        } else {
          ++i_note_index;
        }
      duration = beats[i_note_index] * tempo;
      tone(AUDIO_OUT, notes[i_note_index]);
      timeVal = millis();
      isPlaying = true;
    }
    //delay(duration);
    //Serial.println("Returned from tempo");
    //digitalWrite(LEDA, LOW);
    for (servo_position = 0; servo_position <=120; servo_position +=(2 - (2 * tempo) / 1000)){
       myServo.write(servo_position);
       myServoTwo.write(servo_position);
       myServoThree.write(120 - servo_position);
       delay(10);
       // NEW CODE
       if(!isPlaying){
        duration = beats[i_note_index] * tempo;
        tone(AUDIO_OUT, notes[i_note_index]);
        timeVal = millis();
        isPlaying = true;
      }
      tone(AUDIO_OUT, notes[i_note_index]);
      if((millis() - timeVal) > duration){
        isPlaying = false;
        if((i_note_index + 1) > 53) {
          i_note_index = 0;
        } else {
          ++i_note_index;
        }
        duration = beats[i_note_index] * tempo;
        tone(AUDIO_OUT, notes[i_note_index]);
        timeVal = millis();
        isPlaying = true;
      }
    }

    for (servo_position=120; servo_position >= 0; servo_position -=(2 - (2 * tempo) / 1000)){
      myServo.write(servo_position);
      myServoTwo.write(servo_position);
      myServoThree.write(120 - servo_position);
      delay(10);
      // NEW CODE
       if(!isPlaying){
        duration = beats[i_note_index] * tempo;
        tone(AUDIO_OUT, notes[i_note_index]);
        timeVal = millis();
        isPlaying = true;
      }
      tone(AUDIO_OUT, notes[i_note_index]);
      if((millis() - timeVal) > duration){
        isPlaying = false;
        if((i_note_index + 1) > 53) {
          i_note_index = 0;
        } else {
          ++i_note_index;
        }
        duration = beats[i_note_index] * tempo;
        tone(AUDIO_OUT, notes[i_note_index]);
        timeVal = millis();
        isPlaying = true;
      }
    }
    }
    }
    //Serial.println(digitalRead(10));
    lastLastRead = lastRead;
    lastRead = currRead;
    currRead = digitalRead(12);
    if(lastRead == 1 && lastLastRead == 1 && currRead == 0) {
      if(true) {
        testPressed = true;
        startPressed = false;
        digitalWrite(LEDA, HIGH);
        delay(500);
        digitalWrite(LEDB, HIGH);
        delay(500);
        digitalWrite(LEDC, HIGH);
        delay(1000);
        digitalWrite(LEDC, LOW);
        delay(500);
        digitalWrite(LEDB, LOW);
        delay(500);
        digitalWrite(LEDA, LOW);
        delay(2000);
        for (servo_position = 0; servo_position <=120; servo_position +=(2 - (2 * tempo) / 1000)){
       myServo.write(servo_position);
       myServoTwo.write(servo_position);
       myServoThree.write(120 - servo_position);
       delay(10);
       // NEW CODE
      //tone(AUDIO_OUT, notes[i_note_index]);
    }

    for (servo_position=120; servo_position >= 0; servo_position -=(2 - (2 * tempo) / 1000)){
      myServo.write(servo_position);
      myServoTwo.write(servo_position);
      myServoThree.write(120 - servo_position);
      delay(10);
      // NEW CODE
      //tone(AUDIO_OUT, notes[i_note_index]);
    }
    delay(1000);
    int tempo = 250;
    int duration = 0;
    for(int i = 0; i < 8; i++) {
        tone(8, 450);
        delay(450);
        tone(8, 0);
        delay(450);
    }
    //digitalWrite(LEDA, HIGH);
    while(1){
              for (servo_position = 0; servo_position <=120; servo_position +=(2 - (2 * tempo) / 1000)){
       myServo.write(servo_position);
       myServoTwo.write(servo_position);
       myServoThree.write(120 - servo_position);
       delay(10);
       // NEW CODE
      //tone(AUDIO_OUT, notes[i_note_index]);
    }

    for (servo_position=120; servo_position >= 0; servo_position -=(2 - (2 * tempo) / 1000)){
      myServo.write(servo_position);
      myServoTwo.write(servo_position);
      myServoThree.write(120 - servo_position);
      delay(10);
      // NEW CODE
      //tone(AUDIO_OUT, notes[i_note_index]);
    }
      };
      }
    }
    delay(10);
  }
