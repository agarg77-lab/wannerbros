#include "Arduino.h"                      // Core Arduino functions (pinMode, digitalWrite, etc.)
#include "Enes100.h"                      // ENES100 library for WiFi communication with the vision system
#include "Tank.h"                         // Library for controlling the OSV's tank-drive motors

// I have no idea if this is right, haven't tested it yet. 

void setup() {                            // Runs once when the Arduino starts
  Enes100.begin("WannerBros", MATERIAL, 169, 1116, 14, 50);  // Connect to the vision system: team name, mission type, marker ID, room number, WiFi TX pin, WiFi RX pin
  Tank.begin();                           // Initialize the motor driver

  Enes100.println("Starting driving");    // Send a status message to the vision system's display
  delay(3000);                            // Wait 3 seconds before starting the loop
}

void loop() {                             // Runs over and over after setup() finishes
  Enes100.println(Enes100.getX());        // Send the OSV's X position (from the vision system) to the display
  Enes100.println(Enes100.getY());        // Send the OSV's Y position to the display
  Enes100.println(Enes100.getTheta());    // Send the OSV's heading angle to the display

  float dist = Tank.readDistanceSensor(1); //The parameter is the sensorID but I do not know if its 1 or 7
  Enes100.print(dist*1000); //Put * 1000 in order to convert it from meters to centimeters
  
  //Basically if the sensor sees something closer than 15cm it displays a warning and turns off the motors 
  if(dist > 0 && dist < 15) {
    Enes100.println("Obstacle ahead!");
    Tank.turnOffMotors();
    //In this if statement should be turning right or left based on the course
  }
  

  delay(500);                             // Wait half a second before repeating the loop
}
