#include <Servo.h>
#include "Adafruit_HX711.h"

// Define the pins for the HX711 communication
const uint8_t DATA_PIN = 9;  
const uint8_t CLOCK_PIN = 8; 

Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

Servo ds3218Heavy;
const int servoPin = 6;
int targetAngle = 170; // Desired angle (0 to 270)

void setup() {
  // Attach without overriding default min/max, we will handle it in loop
  ds3218Heavy.attach(servoPin); 

  //Setup for Load Cell
  Serial.begin(115200);

  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {
    delay(10);
  }

  Serial.println("Adafruit HX711 Test!");

  // Initialize the HX711
  hx711.begin();

  // read and toss 3 values each
  Serial.println("Tareing....");
  for (uint8_t t=0; t<3; t++) {
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
  }
}

void loop() {
  // Map 0-270 degrees to 500-2500 microseconds
  int pulseWidth = map(targetAngle, 0, 270, 500, 2500);
  
  // Constrain to prevent sending out-of-bounds pulses
  pulseWidth = constrain(pulseWidth, 500, 2500);
  
  ds3218Heavy.writeMicroseconds(pulseWidth); // tells it to rotate

  delay(2000);
  // alternates every 5 seconds
  
  // Toggle for demonstration
  targetAngle = (targetAngle == 170) ? 130 : 170; 

  //Code for Load Cell
  // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
  // since the read is blocking this will not be more than 10 or 80 SPS (L or H switch)

  int32_t weightA128 = (hx711.readChannelBlocking(CHAN_A_GAIN_128))/1000;
  if (weightA128 > 110 && weightA128 < 145)
  {
    Serial.println("Channel A (Gain 128): LIGHT");
  }
  else if (weightA128 > 165 && weightA128 < 210)
  {
    Serial.println("Channel A (Gain 128): MEDIUM");
  }
  else if (weightA128 > 250 && weightA128 < 290)
  {
    Serial.println("Channel A (Gain 128): HEAVY");
  }
  delay(2000);
  Serial.println(weightA128);
} 
// 120 deg is flat

/*
All data for solid cube raised position (Channel A/1000)
No Cube:
  35-45
Light Cube:
  115-135
Medium Cube:
  165-210
Heavy Cube:
  250-290
*/