#include "Arduino.h"                      // Core Arduino functions (pinMode, digitalWrite, etc.)
#include "Enes100.h"                      // ENES100 library for WiFi communication with the vision system
#include "Tank.h"                         // Library for controlling the OSV's tank-drive motors

// I have no idea if this is right, haven't tested it yet. 


// ---- Ultrasonic sensor pins (CHANGE to match your wiring) ----
const int FRONT_TRIG = 22;                // Pin that sends the trigger pulse for the front sensor
const int FRONT_ECHO = 23;                // Pin that reads the echo pulse for the front sensor
const int SIDE_TRIG  = 24;                // Pin that sends the trigger pulse for the side sensor
const int SIDE_ECHO  = 25;                // Pin that reads the echo pulse for the side sensor

const unsigned long ECHO_TIMEOUT_US = 25000;  // Max time (microseconds) to wait for an echo, about 4 m range

// Returns distance in cm, or -1 if no echo was received
float readDistanceCm(int trigPin, int echoPin) {   // Function that measures distance for one sensor
  digitalWrite(trigPin, LOW);             // Make sure the trigger pin starts LOW
  delayMicroseconds(2);                   // Wait 2 microseconds so the LOW signal settles
  digitalWrite(trigPin, HIGH);            // Start the trigger pulse
  delayMicroseconds(10);                  // Hold the pulse HIGH for 10 microseconds (required by the sensor)
  digitalWrite(trigPin, LOW);             // End the trigger pulse, which makes the sensor send out sound

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);  // Measure how long the echo pin stays HIGH (microseconds)
  if (duration == 0) {                    // pulseIn returns 0 if no echo came back before the timeout
    return -1;                            // Return -1 to signal "nothing detected"
  }
  return duration * 0.0343 / 2.0;         // Convert time to cm: speed of sound is 0.0343 cm/us, divide by 2 for round trip
}

// Averages a few readings to reduce noise; ignores failed readings
float readDistanceAvg(int trigPin, int echoPin, int samples = 3) {  // Function that averages several readings (default 3)
  float total = 0;                        // Running sum of the good readings
  int good = 0;                           // Counts how many readings were valid
  for (int i = 0; i < samples; i++) {     // Repeat once for each sample
    float d = readDistanceCm(trigPin, echoPin);  // Take one distance reading
    if (d > 0) {                          // Only use readings that actually detected something
      total += d;                         // Add the valid reading to the sum
      good++;                             // Count it as a valid reading
    }
    delay(20);                            // Short pause so old echoes die down before the next ping
  }
  return (good > 0) ? total / good : -1;  // Return the average, or -1 if none of the readings were valid
}

void setup() {                            // Runs once when the Arduino starts
  Enes100.begin("WannerBros", MATERIAL, 169, 1116, 52, 50);  // Connect to the vision system: team name, mission type, marker ID, room number, WiFi TX pin, WiFi RX pin
  Tank.begin();                           // Initialize the motor driver

  pinMode(FRONT_TRIG, OUTPUT);            // Front trigger pin sends signals out
  pinMode(FRONT_ECHO, INPUT);             // Front echo pin receives signals in
  pinMode(SIDE_TRIG, OUTPUT);             // Side trigger pin sends signals out
  pinMode(SIDE_ECHO, INPUT);              // Side echo pin receives signals in

  Enes100.println("Starting driving");    // Send a status message to the vision system's display
  delay(3000);                            // Wait 3 seconds before starting the loop
}

void loop() {                             // Runs over and over after setup() finishes
  Enes100.println(Enes100.getX());        // Send the OSV's X position (from the vision system) to the display
  Enes100.println(Enes100.getY());        // Send the OSV's Y position to the display
  Enes100.println(Enes100.getTheta());    // Send the OSV's heading angle to the display

  float frontCm = readDistanceAvg(FRONT_TRIG, FRONT_ECHO);  // Get the averaged distance from the front sensor
  float sideCm  = readDistanceAvg(SIDE_TRIG, SIDE_ECHO);    // Get the averaged distance from the side sensor

  Enes100.print("Front (cm): ");          // Print a label without starting a new line
  Enes100.println(frontCm);               // Print the front distance, then a new line
  Enes100.print("Side (cm): ");           // Print a label without starting a new line
  Enes100.println(sideCm);                // Print the side distance, then a new line

  if (frontCm > 0 && frontCm < 15) {      // If the front sensor sees something valid closer than 15 cm
    Enes100.println("Obstacle ahead!");   // Warn on the display that something is in the way
    // Add your avoid / stop logic with Tank here   // Placeholder: put your motor commands here
  }

  delay(500);                             // Wait half a second before repeating the loop
}
