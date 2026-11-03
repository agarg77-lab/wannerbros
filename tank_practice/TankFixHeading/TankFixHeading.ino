#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"
#include "math.h"

const double AIM_TOLERANCE = 2*PI * 5/360; // 10 degrees in radians
const int TURN_SPEED = 120;               // motor PWM
const int SLOW_TURN_SPEED = 70;           // slower speed near the target
const double SLOW_ZONE = 0.35;   // within 20 degrees: switch to nudging
const int PULSE_MS = 10;         // how long each nudge turns
const int SETTLE_MS = 400;       // wait for a fresh vision reading

const int DRIVE_SPEED = 150;
const int SLOW_DRIVE_SPEED = 90;
const double SLOW_DIST = 0.30; // slow down within 0.2 meters
const double STOP_DIST = 0.05; // close enough to target

bool startedAtA = false;
double siteX = 0.5;
double siteY = 1.5;

// ---------- latest trusted position (filled by updatePose) ----------
double poseX = 0, poseY = 0, poseTheta = 0;
bool done = false;

// ---------- motor functions: ONLY these talk to the tank, rewrite for our robot ----------
void driveForward(int speed) {
  Tank.setLeftMotorPWM(speed);
  Tank.setRightMotorPWM(speed);
}

void turnLeft(int speed) {
  Tank.setLeftMotorPWM(-speed);
  Tank.setRightMotorPWM(speed);
}

void turnRight(int speed) {
  Tank.setLeftMotorPWM(speed);
  Tank.setRightMotorPWM(-speed);
}

void stopMotors() {
  Tank.turnOffMotors();
}

// ---------- helpers ----------
double wrapAngle(double a) {          // keep an angle between -PI and PI
  while (a > PI)  a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

// true only if the vision system sees us AND the numbers are inside the arena
bool markerVisible() {
  if (!Enes100.isVisible()) return false;
  double x = Enes100.getX();
  double y = Enes100.getY();
  return x >= 0.0 && x <= 4.0 && y >= 0.0 && y <= 2.0;
}

// The ONLY place we read position. If the marker is lost: stop, wait, then continue.
void updatePose() {
  bool wasLost = false;
  while (!markerVisible()) {
    if (!wasLost) {
      stopMotors();                                  // dont drive blind
      Enes100.println("Marker lost: stopped, waiting");
      wasLost = true;
    }
    delay(100);
  }
  if (wasLost) Enes100.println("Marker found again");

  poseX = Enes100.getX();
  poseY = Enes100.getY();
  poseTheta = Enes100.getTheta();
}

// ---------- turning ----------
// One step of turning toward target. Returns true once we're facing it.
bool fixHeading(double currentTheta, double target) {
  double error = wrapAngle(target - currentTheta);

  if (fabs(error) < AIM_TOLERANCE) {
    stopMotors();
    return true;
  }

  int speed = (fabs(error) > SLOW_ZONE) ? TURN_SPEED : SLOW_TURN_SPEED;
  if (error > 0) turnLeft(speed);
  else turnRight(speed);

  if (fabs(error) <= SLOW_ZONE) {     // close: nudge, stop, wait for a fresh reading
    delay(PULSE_MS);
    stopMotors();
    delay(SETTLE_MS);
  }
  return false;
}

// Keeps turning (with fresh readings every time) until we face target
void turnToHeading(double targetTheta) {
  while (true) {
    updatePose();
    if (fixHeading(poseTheta, targetTheta)) return;
    delay(20);
  }
}

// ---------- driving ----------
void driveToPoint(double targetX, double targetY) {
  updatePose();
  double dx = targetX - poseX;
  double dy = targetY - poseY;
  double targetAngle = atan2(dy, dx);
  turnToHeading(targetAngle);
  while (true) {
    updatePose();
    dx = targetX - poseX;
    dy = targetY - poseY;
    targetAngle = atan2(dy, dx);
    double dist = sqrt(sq(dx) + sq(dy));
    fixHeading(poseTheta, targetAngle);
    int speed = (dist > SLOW_DIST) ? DRIVE_SPEED : SLOW_DRIVE_SPEED;
    driveForward(speed);
    delay(200);
    stopMotors();
    delay(SETTLE_MS);
    updatePose();
    dx = targetX - poseX;
    dy = targetY - poseY;
    dist = sqrt(sq(dx) + sq(dy));
    if (dist < STOP_DIST) {
      return true;
    }
  }
}



void setup() {
  Enes100.begin("WannerBros", MATERIAL, 169, 1116, 52, 50);
  Tank.begin();
  Enes100.println("Starting driving");

  delay(3000);

  turnToHeading(PI);
  driveToPoint(0.7, 1.5);


  
}

void loop() {
  // put your main code here, to run repeatedly:

}