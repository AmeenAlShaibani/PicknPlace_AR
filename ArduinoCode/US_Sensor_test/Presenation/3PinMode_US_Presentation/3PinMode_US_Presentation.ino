
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

float soundsp;  // Stores calculated speed of sound in M/S
float soundcm;  // Stores calculated speed of sound in cm/ms
int iterations = 5;


void setup() {
  Serial.begin (4800);
}

void loop()
{

  delay(1000);  // Delay so DHT-22 sensor can stabalize
    
    // Calculate the Speed of Sound in M/S
    soundsp = 331.4 ;
    
    // Convert to cm/ms
    
    soundcm = soundsp / 10000;
    
  duration1 = sonar1.ping_median(iterations);
  duration2 = sonar2.ping_median(iterations);
  duration3 = sonar3.ping_median(iterations);
  duration4 = sonar4.ping_median(iterations);

  
  // Calculate the distance
  distance1 = (duration1 / 2) * soundcm;
  distance2 = (duration2 / 2) * soundcm;
  distance3 = (duration3 / 2) * soundcm;
  distance4 = (duration4 / 2) * soundcm;

  // Send US 1 results to Serial Monitor
 
//
//
//    if (distance1 >= 400 || distance1 <= 2) {
//    Serial.print("Out of range");
//    }
//   else if (distance1 > 40) {
//      
//    } 
//    else {
//    Serial.print(" m/s, ");
//    Serial.print("Distance of US1: ");
//    Serial.print(distance1);
//    Serial.print(" cm");
//    delay(200);
//    }
//  
//    Serial.println(" ");

    
    

    // Send US 2 results to Serial Monitor
  
    

    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance2 > 40) {
      
    } 
    else {
     Serial.print(" m/s, ");
    Serial.print("Distance of US2: ");
    Serial.print(distance2);
    Serial.print(" cm");
    delay(200);
    }
  
    Serial.println(" ");

  
  
//
//    // Send US 3 results to Serial Monitor

    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    }
    else if (distance3 > 40) {
      
    } 
    else {
    Serial.print(" m/s, ");
    Serial.print("Distance of US3: ");
    Serial.print(distance3);
    Serial.print(" cm");
    delay(200);
    }
  
  Serial.println(" ");


  
//
//    // Send US 4 results to Serial Monitor
//
//    if (distance4 >= 400 || distance4 <= 2) {
//    Serial.print("Out of range");
//    }
//    else if (distance4 > 40) {
//    
//    } 
//    else {
//      
//    Serial.print(" m/s, ");
//    Serial.print("Distance of US4: ");
//    Serial.print(distance4);
//    Serial.print(" cm");
//    delay(200);
//    }
//  
//  Serial.println(" ");
 
 
}
