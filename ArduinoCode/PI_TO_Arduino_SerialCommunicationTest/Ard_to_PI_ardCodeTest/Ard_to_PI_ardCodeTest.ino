#include <Servo.h>

// A descriptive name for the Arduino
// pins to provide PWM signal
#define PWM_grab_Pin 6 
#define PWM_lift_Pin 7 

// Define an instance of each servo
Servo DSSERVO25KG_grab; 
Servo DSSERVO25KG_lift; 

const int ledPin =  9;  // the number of the LED pin
const int ledPin2 =  8; //Number of LED Pin

void setup() {
  Serial.begin(9600);
  
// initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);        
  pinMode(ledPin2,OUTPUT);
  //INITIALIZE PINS TO LOW 
  digitalWrite(ledPin, LOW);   
  digitalWrite(ledPin2, LOW);

  // attach servos to arduino pins
  DSSERVO25KG_grab.attach(PWM_grab_Pin);
  DSSERVO25KG_lift.attach(PWM_lift_Pin);
}

void loop() {
  String data = "";
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
  
  StatusIndicator(data);
  ServoControl(data);

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
    Serial.println("Open Grabbing Servo Arm");
  }
  else if (data == "Close Servo_grab"){
    DSSERVO25KG_grab.write(140);
    Serial.println("CLOSE Grabbing Servo Arm");
  }
  else if(data == "Open Servo_lift") {
    DSSERVO25KG_lift.write(0);
    Serial.println("Open Liftting Servo Arm");
  }
  else if (data == "Close Servo_lift"){
    DSSERVO25KG_lift.write(100);
    Serial.println("CLOSE Liftting Servo Arm");
  }
 
}
