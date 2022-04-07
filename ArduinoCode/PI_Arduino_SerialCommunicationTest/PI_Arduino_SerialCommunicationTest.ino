//This code is meant to test connection between arduino and Pi 
//The arduino will send a Hello from Arduino! string every second 
//through a serial port, and the goal is to recieve the signal from the pi 

void setup() {
  Serial.begin(9600); //set baud rate to 9600, you can use 57600 or 115200
                     //Baud rate needs to be the same on PI and arduino
}

void loop() {
  Serial.println("Hello from Arduino!"); 
  delay(1000); // this is in millisecond

}
