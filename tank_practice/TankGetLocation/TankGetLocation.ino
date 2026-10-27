#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"


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
