#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

const double HEADING_TOLERANCE = 0.0524;

boolean FixHeading(double currentTheta, double targetTheta) {
  double error = targetTheta - currentTheta;

  while (error > PI)  error -= 2 * PI;
  while (error < -PI) error += 2 * PI;
  if (fabs(error) < HEADING_TOLERANCE) {
    return true;
  } else if (targetTheta - currentTheta > 0) {
  // turn left
  } else {
  // turn right
  }


}


void setup() {
  Enes100.begin("WannerBros", MATERIAL, 169, 1116, 52, 50);
  Tank.begin();
  Enes100.println("Starting driving");

  delay(3000);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  Enes100.println(Enes100.getX());
  Enes100.println(Enes100.getY());
  Enes100.println(Enes100.getTheta());
  delay(2000);
}