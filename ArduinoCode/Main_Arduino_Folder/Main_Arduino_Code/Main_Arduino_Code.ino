#include <Servo.h>
#include "NewPing.h"

/////////////////////define constants for the Ultrasonics////////////////////////////

// Define Constants
#define edgeLeft  3  
#define frontLeft 2
#define frontRight  12
#define edgeRight  13
#define sideLeft 10
#define sideRight 11

#define MAX_DISTANCE 400

NewPing EL(edgeLeft, edgeLeft, MAX_DISTANCE);
NewPing FL(frontLeft, frontLeft, MAX_DISTANCE);
NewPing FR(frontRight, frontRight, MAX_DISTANCE);
NewPing ER(edgeRight, edgeRight, MAX_DISTANCE);
NewPing SR(sideRight, sideRight, MAX_DISTANCE);
NewPing SL(sideLeft, sideLeft, MAX_DISTANCE);


// Define Variables


float EL_duration; // Stores HC-SR04 pulse duration value
float EL_distance; // Stores calculated distance in cm

float FL_duration; // Stores HC-SR04 pulse duration value
float FL_distance; // Stores calculated distance in cm

float FR_duration; // Stores HC-SR04 pulse duration value
float FR_distance; // Stores calculated distance in cm

float ER_duration; // Stores HC-SR04 pulse duration value
float ER_distance; // Stores calculated distance in cm

float SR_duration; // Stores HC-SR04 pulse duration value
float SR_distance; // Stores calculated distance in cm

float SL_duration; // Stores HC-SR04 pulse duration value
float SL_distance; // Stores calculated distance in cm

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
  Serial.begin(115200);
  
// initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);        
  pinMode(ledPin2,OUTPUT);
  //INITIALIZE PINS TO LOW 
  digitalWrite(ledPin, LOW);   
  digitalWrite(ledPin2, LOW);

  // attach servos to arduino pins
  DSSERVO25KG_grab.attach(PWM_grab_Pin);
  DSSERVO25KG_lift.attach(PWM_lift_Pin);

  EL_distance=0;
  FL_distance=0;
  FR_distance=0;
  ER_distance=0;
  SR_distance=0;
  SL_distance=0;
}

void loop() {
  String data = readSerialPort();
  if (data == "") {
    String USDATA = get_USdata();
    //only send data if there is something close to robot 
    //TODO: Make the side US distance be less than the front 2. 
    Serial.print(USDATA);
    Serial.print('\n');
    
  } else {
    StatusIndicator(data);
    ServoControl(data);
  }
}

String readSerialPort() {
  String data = "";
  if (Serial.available() > 0) {
    delay(10);
    data = Serial.readStringUntil('\n');
  }
  return data;
}

String get_USdata() { 

  EL_duration = EL.ping_median(iterations);
  delay(10);
  FL_duration = FL.ping_median(iterations);
  delay(10);
  FR_duration = FR.ping_median(iterations);
  delay(10);
  ER_duration = ER.ping_median(iterations);
  delay(10);
  SR_duration = SR.ping_median(iterations);
  delay(10);
  SL_duration = SL.ping_median(iterations);

  // Calculate the distance
  EL_distance = (EL_duration / 2) * soundcm;
  FL_distance = (FL_duration / 2) * soundcm;
  FR_distance = (FR_duration / 2) * soundcm;
  ER_distance = (ER_duration / 2) * soundcm;
  SR_distance = (SR_duration / 2) * soundcm;
  SL_distance = (SL_duration / 2) * soundcm;

  if ((EL_distance < 2) || (EL_distance > 400)) {
    EL_distance = 1000;
  }
  if ((FL_distance < 2) || (FL_distance > 400)) {
    FL_distance = 1000;
  }
  if ((FR_distance < 2) || (FR_distance > 400)) {
    FR_distance = 1000;
  }
  if ((ER_distance < 2) || (ER_distance > 400)) {
    ER_distance = 1000;
  }
  if ((SR_distance < 2) || (SR_distance > 400)) {
    SR_distance = 1000;
  }
  if ((SL_distance < 2) || (SL_distance > 400)) {
    SL_distance = 1000;
  }  
  
  String USDATA = "";
  USDATA += SR_distance ;
  USDATA += ",";
  USDATA += ER_distance;
  USDATA += ",";
  USDATA += FR_distance ;
  USDATA += "," ;
  USDATA += FL_distance;
  USDATA += "," ;
  USDATA += EL_distance;
  USDATA += "," ;
  USDATA += SL_distance;
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
