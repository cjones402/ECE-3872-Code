#define ANALOG_PIN A0
#define ENVELOPE_PIN 2
#define AUDIO_OUT 3

int raw_value;
int audio = 0;
int audioMax = 0;
int i;
boolean timerRunning;
int startTime;
int stopTime;
double duration;
double durAve;
double numDurs;

const int numReadings = 20;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int multiplier = 1.5;
int addto = 0;


boolean active = false;

void start_timer() {
  timerRunning = true;
  startTime = millis();
  //Serial.println("start");
}

void stop_timer() {
  stopTime = millis();
  duration = (double)stopTime-startTime;
  Serial.print("duration: ");
  Serial.println(duration*multiplier+addto);
    if(duration > 90 && duration < 700){
    durAve = ((durAve*numDurs + duration*multiplier+addto)/(numDurs+1));
    numDurs++;
    Serial.print("duration average: ");
    Serial.println(durAve);
  }
  if(numDurs == 16) {
    Serial.print("duration average final: ");
    Serial.println(durAve*1.1);
  }
  //Serial.print("tempo: ");
  //Serial.println(duration/1000);
  timerRunning = false;
  //Serial.println("stopped");
}

void change() {
  Serial.println("changed");
  if(timerRunning){
    stop_timer();
  } else {
    start_timer();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  raw_value = 0; // to store raw value of mic
  pinMode(ANALOG_PIN,INPUT);
  pinMode(ENVELOPE_PIN,INPUT);
  pinMode(AUDIO_OUT,OUTPUT);
  
  //attachInterrupt(digitalPinToInterrupt(ENVELOPE_PIN), start_timer, FALLING);
  //attachInterrupt(digitalPinToInterrupt(ENVELOPE_PIN), stop_timer, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENVELOPE_PIN), change, CHANGE);
  timerRunning = false;
  startTime = 0;
  stopTime = 0;
  duration = 0.0;

   for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(ANALOG_PIN);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
   if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  //Serial.println(average);
  //delay(1);        // delay in between reads for stability
  //*
  if (average > 50 || active && average > 50 ) {
    raw_value = 1;
    if(!active){
      timerRunning = true;
      change();
    }
    active = true;
  } else {
    average = 0;
    if(active){
      timerRunning = false;
      change();
    }
    active = false;
  }//*/
  if(average>35){

    if (average>=100){
      average = 100;
    }
    //Serial.println(average>50);
  } else {
    //Serial.println(0);
  }
  
  //Serial.plot(ANALOG_PIN);
  /*
  raw_value = analogRead(ANALOG_PIN); // Reading the op voltage of mic
  audio = (raw_value-(0.5/3.3)*1023);
  if(i<1000){
  if(audio>audioMax){
    audioMax = audio;
    delay(1);
  }
  i++;
  }else{
    Serial.print("Raw value is : ");
    Serial.println(audioMax);
    audioMax = 0;
    i=0;
  }*/
}
