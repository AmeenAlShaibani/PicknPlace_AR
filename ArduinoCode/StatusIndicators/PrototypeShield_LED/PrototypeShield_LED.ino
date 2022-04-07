// This source code start as example code from Arduino
// Original credit goes to  DojoDave modified by Tom Igoe
// Then I did a minimal modification just to make it works 
// with DFRobot Prototype Shield

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  1;      // the number of the LED pin
const int ledPin2 =  3; //Number of LED Pin

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);        
  pinMode(ledPin2,OUTPUT);
  //INITIALIZE PINS TO LOW 
  digitalWrite(ledPin, LOW);   
  digitalWrite(ledPin2, LOW);
}

void loop(){
  // read the state of the pushbutton value:
      
    // turn LED on:    
  digitalWrite(ledPin, HIGH);  //Set to HIGH or LOW
  digitalWrite(ledPin2, HIGH);
  
}
