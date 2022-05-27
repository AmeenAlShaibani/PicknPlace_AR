
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
#define sideLeft 11
#define sideRight 10
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

float soundcm;  // Stores calculated speed of sound in cm/ms
int iterations = 5;


void setup() {
  Serial.begin (4800);
}

void loop()
{
    
  //Sound speed in cm
    
  soundcm =  331.4 / 10000;
    
  EL_duration = EL.ping_median(iterations);
  FL_duration = FL.ping_median(iterations);
  FR_duration = FR.ping_median(iterations);
  ER_duration = ER.ping_median(iterations);
  SR_duration = SR.ping_median(iterations);
  SL_duration = SL.ping_median(iterations);

  // Calculate the distance
  EL_distance = (EL_duration / 2) * soundcm;
  FL_distance = (FL_duration / 2) * soundcm;
  FR_distance = (FR_duration / 2) * soundcm;
  ER_distance = (ER_duration / 2) * soundcm;
  SR_distance = (SR_duration / 2) * soundcm;
  SL_distance = (SL_duration / 2) * soundcm;


  // Send US 1 results to Serial Monitor
 
    Serial.print(" m/s, ");
    Serial.print("Distance of edgeLeft: ");

    if (EL_distance >= 400 || EL_distance <= 2) {
    Serial.print("Out of range");
    }
    else if (EL_distance > 20) {
      
    } 
    else {
    Serial.print(EL_distance);
    Serial.print(" cm");
    delay(500);
    }
  
    Serial.println(" ");

    
    

    // Send US 2 results to Serial Monitor
  
    Serial.print(" m/s, ");
    Serial.print("Distance of frontLeft: ");

    if (FL_distance >= 400 || FL_distance <= 2) {
    Serial.print("Out of range");
    }
    else if (FL_distance > 20) {
      
    } 
    else {
    Serial.print(FL_distance);
    Serial.print(" cm");
    delay(500);
    }
  
    Serial.println(" ");

  
  
//
//    // Send US 3 results to Serial Monitor

 
    Serial.print(" m/s, ");
    Serial.print("Distance of frontRight: ");

    if (FR_distance >= 400 || FR_distance <= 2) {
    Serial.print("Out of range");
    }
    else if (FR_distance > 20) {
      
    } 
    else {
    Serial.print(FR_distance);
    Serial.print(" cm");
    delay(500);
    }
  
  Serial.println(" ");


  
//
//    // Send US 4 results to Serial Monitor

 

    Serial.print(" m/s, ");
    Serial.print("Distance of edgeRight: ");

    if (ER_distance >= 400 || ER_distance <= 2) {
    Serial.print("Out of range");
    }
    else if (ER_distance > 20) {
    
    } 
    else {
    Serial.print(ER_distance);
    Serial.print(" cm");
    delay(500);
    }
  
  Serial.println(" ");
 
 
}
