#include <QTRSensors.h>
#define enA 6
#define enB 5
#define in1 10
#define in2 9
#define in3 8
#define in4 7
#define KP .3
#define KD 4
#define RightBaseSpeed 100
#define LeftBaseSpeed 100
#define RightMaxSpeed 150
#define LeftMaxSpeed 150

QTRSensors qtr;

const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];
boolean mode = LOW;
boolean sensorFlag = LOW;
const byte interPin = 2;
long unsigned int lastPress;
const byte debounceTime = 40;
boolean ledState = LOW;
unsigned int ledTime = 0;
int lasterr = 0;
byte dataTurn = 0;

void sensorCal(){
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t []){A0,A1,A2,A3,A4,A5,A6,A7}, sensorCount);
  qtr.setEmitterPin(3);
  
  delay(200);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  for(int i=0; i<400; i++){
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN,LOW);

  Serial.begin(9600);

  for(byte i=0; i<sensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for(byte i=0; i<sensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  
  delay(500);
  
}

void sensorData(){
  
  uint16_t position = qtr.readLineBlack(sensorValues);
  /**/
  for(byte i=0; i<sensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println(position);

  delay(250);
  /**/
  boolean follow = true;
  if(dataTurn==5){
    follow = false;
    dataTurn = 0;
    for(byte i=0; i<sensorCount; i++){
      if(sensorValues[i]>200){
        follow = true;
        break;
      }
    }
  }
  if(follow){
    uint16_t err = position - 2000;

    uint16_t motorSpeed = KP * err + KD * (err - lasterr);
    lasterr = err;

    uint16_t leftMotorSpeed = LeftBaseSpeed + motorSpeed;
    uint16_t rightMotorSpeed = RightBaseSpeed - motorSpeed;
  
    setMotorSpeed(rightMotorSpeed,leftMotorSpeed);
    dataTurn++;
  }
  
  if(ledTime>6){
    ledState = !ledState;
    ledTime = 0;
  }
  digitalWrite(LED_BUILTIN, ledState);
  ledTime++;
}

void setMotorSpeed(uint16_t r, uint16_t l){
  if(r>RightMaxSpeed)
    r = RightMaxSpeed;
  if(l>LeftMaxSpeed)
    l = LeftMaxSpeed;
  if(r<0)
    r = 0;
  if(l<0)
    l = 0;
  analogWrite(enA,r);
  analogWrite(enB,l);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  delay(100);
}

void forward(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void back(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void right(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}

void left(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void off(){
  analogWrite(enA,0);
  analogWrite(enB,0);
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}



void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  Serial.begin(9600);
  pinMode(interPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interPin),changeMode, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(mode == LOW){
    if(Serial.available()>0){
      int inp = Serial.read();
      switch(inp){
        case '0':
        left();
        delay(100);
        break;
        
        case '1':
        right();
        delay(100);
        break;

        case '2':
        forward();
        delay(100);
        break;

        case '3':
        back();
        delay(100);
        break;
        
        case '4':
        off();
        default:
        break;
      }
    }
  }
  else if(mode==HIGH){
    if(sensorFlag==LOW){
       Serial.println("IR Test");
       sensorCal();
       sensorFlag=HIGH;
       }
    else{
       sensorData();
    }
  }
}

void changeMode(){
  if(millis()-lastPress > debounceTime){
    lastPress = millis();
    if(digitalRead(interPin)==0){
      mode = !mode;
    }
    else if(digitalRead(interPin)==1){
      mode = mode;
    }
  }
}
