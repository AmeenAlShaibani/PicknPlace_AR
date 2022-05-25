
/*
  HC-SR04 in 3-Wire Mode with Temp and Humidity Demonstration
  HC-SR04-3Wire-Temp-Humid-Demo.ino
  Demonstrates enhancements of HC-SR04 Ultrasonic Range Finder
  With DHT22 Temperature and Humidity Sensor
  Displays results on Serial Monitor

  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

// Include NewPing Library
#include "NewPing.h"

// Define Constants
#define edgeLeft  3  // Trigger and Echo both on pin 10
#define frontLeft  4
#define frontRight  12
#define edgeRight  13

//TESTING
/////////////////////////////////
#define sideLeft
#define sideRight
/////////////////////////////////


#define MAX_DISTANCE 400

//TESTING
/////////////////////////////////
#define MAX_DISTANCE_SIDE 50
#define MAX_DISTANCE_FRONT 50
#define MAX_DISTANCE_EDGE 50

/////////////////////////////////

NewPing EL(edgeLeft, edgeLeft, MAX_DISTANCE);
NewPing FL(frontLeft, frontLeft, MAX_DISTANCE);
NewPing FR(frontRight, frontRight, MAX_DISTANCE);
NewPing ER(edgeRight, edgeRight, MAX_DISTANCE);


// Define Variables

float duration1; // Stores HC-SR04 pulse duration value
float distance1; // Stores calculated distance in cm

float duration2; // Stores HC-SR04 pulse duration value
float distance2; // Stores calculated distance in cm

float duration3; // Stores HC-SR04 pulse duration value
float distance3; // Stores calculated distance in cm

float duration4; // Stores HC-SR04 pulse duration value
float distance4; // Stores calculated distance in cm

float soundcm;  // Stores calculated speed of sound in cm/ms
int iterations = 5;


void setup() {
  Serial.begin (4800);
}

void loop()
{
    
  //Sound speed in cm
    
  soundcm =  331.4 / 10000;
    
  duration1 = EL.ping_median(iterations);
  duration2 = FL.ping_median(iterations);
  duration3 = FR.ping_median(iterations);
  duration4 = ER.ping_median(iterations);

  // Calculate the distance
  distance1 = (duration1 / 2) * soundcm;
  distance2 = (duration2 / 2) * soundcm;
  distance3 = (duration3 / 2) * soundcm;
  distance4 = (duration4 / 2) * soundcm;

  // Send US 1 results to Serial Monitor
 
    Serial.print(" m/s, ");
    Serial.print("Distance of edgeLeft: ");

    if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance1 > 20) {
      
    } 
    else {
    Serial.print(distance1);
    Serial.print(" cm");
    delay(500);
    }
  
    Serial.println(" ");

    
    

    // Send US 2 results to Serial Monitor
  
    Serial.print(" m/s, ");
    Serial.print("Distance of frontLeft: ");

    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance2 > 20) {
      
    } 
    else {
    Serial.print(distance2);
    Serial.print(" cm");
    delay(500);
    }
  
    Serial.println(" ");

  
  
//
//    // Send US 3 results to Serial Monitor

 
    Serial.print(" m/s, ");
    Serial.print("Distance of frontRight: ");

    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance3 > 20) {
      
    } 
    else {
    Serial.print(distance3);
    Serial.print(" cm");
    delay(500);
    }
  
  Serial.println(" ");


  
//
//    // Send US 4 results to Serial Monitor

 

    Serial.print(" m/s, ");
    Serial.print("Distance of edgeRight: ");

    if (distance4 >= 400 || distance4 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance4 > 20) {
    
    } 
    else {
    Serial.print(distance4);
    Serial.print(" cm");
    delay(500);
    }
  
  Serial.println(" ");
 
 
}
