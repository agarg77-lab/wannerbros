
#include <Servo.h>

Servo ds3218Heavy;
const int servoPin = 6;
int targetAngle = 270; // Desired angle (0 to 270)

void setup() {
  // Attach without overriding default min/max, we will handle it in loop
  ds3218Heavy.attach(servoPin); 
}

void loop() {
  // Map 0-270 degrees to 500-2500 microseconds
  int pulseWidth = map(targetAngle, 0, 270, 500, 2500);
  
  // Constrain to prevent sending out-of-bounds pulses
  pulseWidth = constrain(pulseWidth, 500, 2500);
  
  ds3218Heavy.writeMicroseconds(pulseWidth); // tells it to rotate

  delay(2000);
  // alternates every 2 seconds
  
  // Toggle for demonstration
  targetAngle = (targetAngle == 270) ? 120 : 270; 
} 
// 120 deg is flat
