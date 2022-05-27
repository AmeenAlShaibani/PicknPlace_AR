#include <Servo.h>
#include "NewPing.h"

/////////////////////define constants for the Ultrasonics////////////////////////////

#define US1  3  // Trigger and Echo both on pin 10
#define US2  4
#define US3  12
#define US4  13

#define MAX_DISTANCE 400

NewPing sonar1(US1, US1, MAX_DISTANCE);
NewPing sonar2(US2, US2, MAX_DISTANCE);
NewPing sonar3(US3, US3, MAX_DISTANCE);
NewPing sonar4(US4, US4, MAX_DISTANCE);


// Define Variables

float duration1; // Stores HC-SR04 pulse duration value
float distance1; // Stores calculated distance in cm

float duration2; // Stores HC-SR04 pulse duration value
float distance2; // Stores calculated distance in cm

float duration3; // Stores HC-SR04 pulse duration value
float distance3; // Stores calculated distance in cm

float duration4; // Stores HC-SR04 pulse duration value
float distance4; // Stores calculated distance in cm

float soundcm = 331.4 / 10000;;  // Stores calculated speed of sound in cm/ms
int iterations = 5;

//////////////////////Define Instances for Servo///////////////////////////////
// Define an instance of each servo
Servo DSSERVO25KG_grab; 
Servo DSSERVO25KG_lift; 

#define PWM_grab_Pin 6 
#define PWM_lift_Pin 7 

///////////////////////Define LED pins/////////////////////////////////////////
const int ledPin =  9;  // the number of the LED pin
const int ledPin2 =  8; //Number of LED Pin

void setup() {
  Serial.begin(57600);
  
// initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);        
  pinMode(ledPin2,OUTPUT);
  //INITIALIZE PINS TO LOW 
  digitalWrite(ledPin, LOW);   
  digitalWrite(ledPin2, LOW);

  // attach servos to arduino pins
  DSSERVO25KG_grab.attach(PWM_grab_Pin);
  DSSERVO25KG_lift.attach(PWM_lift_Pin);

  distance1 = 0;
  distance2=0;
  distance3=0;
  distance4=0;
}

void loop() {
  String data = readSerialPort();
  if (data == "") {
    String USDATA = get_USdata();
    //only send data if there is something close to robot 
    //TODO: Make the side US distance be less than the front 2. 

    if((distance1 < 15) || (distance2 < 15) || (distance3 < 15) || (distance4 < 15)){
      Serial.print(USDATA);
      Serial.print('\n');
    }
  } else {
    StatusIndicator(data);
    ServoControl(data);
  }
  delay(500);
}

String readSerialPort() {
  String data = "";
  if (Serial.available() > 0) {
    delay(10);
    data = Serial.readStringUntil('\n');
    Serial.println(data);
    Serial.flush();
  }
  return data;
}

String get_USdata() { 
  duration1 = sonar1.ping_median(iterations);
  duration2 = sonar2.ping_median(iterations);
  duration3 = sonar3.ping_median(iterations);
  duration4 = sonar4.ping_median(iterations);

  // Calculate the distance
  distance1 = (duration1 / 2) * soundcm;
  distance2 = (duration2 / 2) * soundcm;
  distance3 = (duration3 / 2) * soundcm;
  distance4 = (duration4 / 2) * soundcm;

  if((distance1 < 2) || (distance1 > 400)) {
    distance1 = 1000;
  }
  if ((distance2 < 2) || (distance2 > 400)) {
    distance2 = 1000;
  }
  if ((distance3 < 2) || (distance3 > 400)) {
    distance3 = 1000;
  }
  if ((distance4 < 2) || (distance4 > 400)) {
    distance4 = 1000;
  }
    
  
  String USDATA = "";
  USDATA += distance1 ;
  USDATA += ",";
  USDATA += distance2;
  USDATA += ",";
  USDATA += distance3 ;
  USDATA += "," ;
  USDATA += distance4;
  return USDATA;
}


void StatusIndicator(String data){
  if(data == "RC Mode") {
       // turn LED2 Off:
    digitalWrite(ledPin2,LOW);
       // turn LED1 on:    
    digitalWrite(ledPin, HIGH);
  }
  else if(data == "Confirmation Mode"){
       // turn LED1 Off:
    digitalWrite(ledPin,LOW);
       // turn LED2 on:    
    digitalWrite(ledPin2, HIGH); 
  }
  else if(data == "Scout Mode"){
    //set both pins to high
    digitalWrite(ledPin, HIGH); 
    digitalWrite(ledPin2, HIGH);
  }
}

void ServoControl(String data) {
  if(data == "Open Servo_grab") {
    DSSERVO25KG_grab.write(45);
    //Serial.println("Open Grabbing Servo Arm");
  }
  else if (data == "Close Servo_grab"){
    DSSERVO25KG_grab.write(140);
    //Serial.println("CLOSE Grabbing Servo Arm");
  }
  else if(data == "Open Servo_lift") {
    DSSERVO25KG_lift.write(0);
    //Serial.println("Open Liftting Servo Arm");
  }
  else if (data == "Close Servo_lift"){
    DSSERVO25KG_lift.write(100);
    //Serial.println("CLOSE Liftting Servo Arm");
  }
 
}
