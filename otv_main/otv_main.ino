#include "Arduino.h"
#include "Enes100.h"
#include "Arduino_LED_Matrix.h"

// =====================================================================
// SETUP NUMBERS - check these before uploading
// =====================================================================
const char* TEAM_NAME = "WannerBros";
const int MARKER_ID = 169;        // check: our robot's Aruco marker ID
const int ROOM = 1116;

const bool MOTOR_TEST  = false;   // true = only the wheels test (wheels off the table!)
const bool SENSOR_TEST = false;   // true = only print the distance sensors
const bool DEBUG = true;          // true = print debug messages

// 1 = full run, 2 = start at the obstacles, 3 = start at the limbo
const int START_AT_PART = 1;

// ---------- motor pins ----------
// Tested: left forward = pin 3 HIGH + pin 2 LOW, right forward = pin 4 HIGH + pin 5 LOW.
// STILL TO CHECK: turnLeft must turn the robot counter-clockwise (seen from above).
// If it turns clockwise, use LEFT_FWD = 4, LEFT_BACK = 5, RIGHT_FWD = 3, RIGHT_BACK = 2.
const int LEFT_FWD   = 3;
const int LEFT_BACK  = 2;
const int RIGHT_FWD  = 4;
const int RIGHT_BACK = 5;

// ---------- speed control (not wired yet) ----------
// UNTIL THEN: the ENA and ENB jumpers on the L298N must be in place.
const bool HAS_PWM = false;
const int LEFT_EN  = -1;          // must be a PWM pin: 6, 9, 10 or 11
const int RIGHT_EN = -1;

// ---------- front distance sensors (HC-SR04) ----------
// Both sensors share trigger pin 13, so every ping fires both.
// Pin 13 is also the board's "L" LED, so it flickers while pinging. That's normal.
const int LEFT_TRIG  = 13;
const int LEFT_ECHO  = 12;        // check with SENSOR_TEST which side this really is
const int RIGHT_TRIG = 13;
const int RIGHT_ECHO = A0;

// =====================================================================
// ARENA (meters). Check these against the real arena.
// =====================================================================
const double ROW_Y[3] = {0.5, 1.0, 1.5};   // centres of the three obstacle slots
const double CHECK_X[2] = {1.0, 1.87};     // where we stop to check: before column 1, between the columns
const double LINEUP_X = 2.75;              // past column 2: line up for the limbo here
const double LIMBO_ROW_Y = 1.5;            // the limbo is in the top half
const double GOAL_X = 3.7;                 // stop once past this

// =====================================================================
// TUNING
// =====================================================================
const double DEG = PI / 180.0;

// turning in place
const double AIM_TOLERANCE = 5 * DEG;    // done turning when within 5 degrees
const double SLOW_ZONE     = 20 * DEG;   // within 20 degrees: switch to short pulses
const int TURN_PULSE_MS    = 30;         // length of one turning pulse
const int SETTLE_MS        = 400;        // wait for a fresh vision reading after a pulse

// driving
const double STEER_DEADBAND  = 4 * DEG;  // no-PWM steering: ignore errors smaller than this
const double BIG_ERROR       = 30 * DEG; // more than 30 degrees off: stop and turn in place
const double SLOW_DIST       = 0.30;     // last 30 cm are done in short pulses
const double STOP_DIST       = 0.05;     // close enough to the target
const int CREEP_MS           = 80;       // length of one driving pulse in the last 30 cm
const double CUBE_STOP_DIST  = 0.15;     // stop when the front sensors see the cube this close
const double CUBE_CHECK_DIST = 0.60;     // only look for the cube when this close to the site

// obstacles and limbo
const double SLOT_BLOCKED_DIST = 0.50;   // at a check point: closer than this = slot blocked
const double MIN_VALID_DIST    = 0.03;   // readings closer than this are ignored (raise it if
                                         // the sensors see the cube we're carrying)
const unsigned long LIMBO_BLIND_MAX_MS = 8000;  // max time driving without seeing the marker

// speeds: only used once HAS_PWM is true (0-255)
const int DRIVE_SPEED      = 200;
const int SLOW_DRIVE_SPEED = 120;
const int TURN_SPEED       = 150;
const int SLOW_TURN_SPEED  = 110;
const double STEER_GAIN    = 400;        // extra PWM per radian of heading error
const int MAX_STEER        = 90;

const double NO_READING = 9.99;          // distance returned when a sensor sees nothing

// =====================================================================
// STATE
// =====================================================================
bool startedAtA = false;
double siteX = 0.176;     // mission site we drive to (set by determineStartSite)
double siteY = 1.5;
int currentRow = 2;       // which obstacle row we're driving in (0, 1 or 2)

double poseX = 0, poseY = 0, poseTheta = 0;   // latest trusted position (from updatePose)

bool visionConnected = false;
char currentStage = ' ';

// =====================================================================
// DEBUG HELPERS: LED matrix + messages
// =====================================================================
unsigned long lastStatus = 0;   // used by statusReport() and turnToHeading()

// Print to the USB Serial Monitor and the vision console.
void say(const String& msg) {
  if (!DEBUG) return;
  String line = "[" + String(millis() / 1000.0, 1) + "s] " + msg;
  Serial.println(line);
  if (visionConnected) Enes100.println(line.c_str());
}

// Print which stage the code is in
void stage(char c, const String& msg) {
  currentStage = c;
  say(String("STAGE ") + c + ": " + msg);
}

// =====================================================================
// MOTOR FUNCTIONS - the only code that touches the motor pins
// =====================================================================

// speed: -255 (full backward) ... 0 (coast) ... 255 (full forward).
// Without PWM, any positive number = full forward any negative = full backward.
void setSide(int fwdPin, int backPin, int enPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(backPin, LOW);
  } else if (speed < 0) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(backPin, HIGH);
  } else {
    digitalWrite(fwdPin, LOW);    // coast
    digitalWrite(backPin, LOW);
  }
  if (HAS_PWM) analogWrite(enPin, abs(speed));
}

void setLeft(int speed)  { setSide(LEFT_FWD, LEFT_BACK, LEFT_EN, speed); }
void setRight(int speed) { setSide(RIGHT_FWD, RIGHT_BACK, RIGHT_EN, speed); }

void driveForward(int speed)  { setLeft(speed);  setRight(speed); }
void driveBackward(int speed) { setLeft(-speed); setRight(-speed); }
void turnLeft(int speed)      { setLeft(-speed); setRight(speed); }
void turnRight(int speed)     { setLeft(speed);  setRight(-speed); }

// Brake, both inputs HIGH stops the wheels fast
void stopMotors() {
  digitalWrite(LEFT_FWD, HIGH);
  digitalWrite(LEFT_BACK, HIGH);
  digitalWrite(RIGHT_FWD, HIGH);
  digitalWrite(RIGHT_BACK, HIGH);
  if (HAS_PWM) {
    analogWrite(LEFT_EN, 255);
    analogWrite(RIGHT_EN, 255);
  }
}

// Drive forward while curving toward the target. error > 0 = target is to the left.
void driveSteer(int speed, double error) {
  if (HAS_PWM) {
    int correction = constrain((int)(STEER_GAIN * error), -MAX_STEER, MAX_STEER);
    setLeft(speed - correction);
    setRight(speed + correction);
  } else {
    // No speed control yet: to curve, let the wheels on the inside of the turn coast.
    if (error > STEER_DEADBAND) {          // target to the left
      setLeft(0);
      setRight(speed);
    } else if (error < -STEER_DEADBAND) {  // target to the right
      setLeft(speed);
      setRight(0);
    } else {
      driveForward(speed);
    }
  }
}

// =====================================================================
// DISTANCE SENSORS
// =====================================================================

// One HC-SR04 reading in meters. NO_READING if not wired, no echo, or too close to trust.
double readUltrasonic(int trigPin, int echoPin) {
  if (trigPin < 0 || echoPin < 0) return NO_READING;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 25000UL);   // give up after 25 ms (about 4 m)
  if (us == 0) return NO_READING;
  double m = us * 0.000343 / 2.0;                       // speed of sound, there and back
  if (m < MIN_VALID_DIST) return NO_READING;
  return m;
}

double readLeft()  { return readUltrasonic(LEFT_TRIG, LEFT_ECHO); }
double readRight() { return readUltrasonic(RIGHT_TRIG, RIGHT_ECHO); }

// Closest thing seen by either front sensor, in meters.
double frontDistance() {
  double left = readLeft();
  delay(30);                       // shared trigger: let every echo die out first
  double right = readRight();
  return (left < right) ? left : right;
}

// Middle of three frontDistance() readings
double frontDistanceMedian() {
  double a = frontDistance(); delay(30);
  double b = frontDistance(); delay(30);
  double c = frontDistance();
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// =====================================================================
// VISION
// =====================================================================
double wrapAngle(double a) {          // keep an angle between -PI and PI
  while (a > PI)  a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

bool readPose() {
  if (!Enes100.isVisible()) return false;
  double x = Enes100.getX();
  double y = Enes100.getY();
  double t = Enes100.getTheta();
  if (x < 0.0 || x > 4.0 || y < 0.0 || y > 2.0) return false;   // junk reading
  poseX = x;
  poseY = y;
  poseTheta = t;
  return true;
}

// The ONLY place we read position (except under the limbo). If the marker is lost:
// stop, show L, wait until it's seen again, then carry on.
void updatePose() {
  if (readPose()) return;

  stopMotors();
  say("Marker NOT visible: stopped, waiting (marker " + String(MARKER_ID) + ")");
  unsigned long lostAt = millis();
  unsigned long lastHint = millis();

  while (!readPose()) {
    if (millis() - lastHint > 5000) {
      lastHint = millis();
      say("Still can't see marker " + String(MARKER_ID) + " after " +
          String((millis() - lostAt) / 1000) + " s. Check MARKER_ID, that the marker is "
          "uncovered, the robot is in the arena, and the vision system is running.");
    }
    delay(5);
  }
  say("Marker visible again after " + String((millis() - lostAt) / 1000.0, 1) + " s");
}

// Prints where we are once per second (so prints don't slow the driving loop).
void statusReport(const String& what, double targetX, double targetY) {
  if (!DEBUG || millis() - lastStatus < 1000) return;
  lastStatus = millis();
  double dx = targetX - poseX;
  double dy = targetY - poseY;
  say(what + " | x=" + String(poseX, 3) + " y=" + String(poseY, 3) +
      " heading=" + String(poseTheta / DEG, 1) + " deg | to target " +
      String(sqrt(dx * dx + dy * dy), 3) + " m | front " + String(frontDistance(), 2) + " m");
}

// =====================================================================
// TURNING
// =====================================================================

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
  if (fabs(error) <= SLOW_ZONE) {     // close: short pulse, stop, wait for a fresh reading
    delay(TURN_PULSE_MS);
    stopMotors();
    delay(SETTLE_MS);
  }
  return false;
}

// Keeps turning (with a fresh reading every time) until we face target
void turnToHeading(double target) {
  char stageBefore = currentStage;
  stage('6', "Turning to face " + String(target / DEG, 1) + " deg");
  unsigned long start = millis();
  unsigned long lastWarn = millis();
  while (true) {
    updatePose();
    if (DEBUG && millis() - lastStatus > 1000) {
      lastStatus = millis();
      say("Turning | heading " + String(poseTheta / DEG, 1) + " deg, error " +
          String(wrapAngle(target - poseTheta) / DEG, 1) + " deg");
    }
    if (millis() - lastWarn > 10000) {
      lastWarn = millis();
      say("Turning for " + String((millis() - start) / 1000) + " s. If the heading isn't "
          "changing: TURN_PULSE_MS too short, or no motor power. If the error keeps GROWING: "
          "turnLeft/turnRight are reversed (see the motor pin note at the top).");
    }
    if (fixHeading(poseTheta, target)) {
      say("Facing target (" + String((millis() - start) / 1000.0, 1) + " s)");
      if (stageBefore == '7' || stageBefore == '8') stage(stageBefore, "Back to driving");
      return;
    }
    delay(20);
  }
}

// =====================================================================
// DRIVING
// =====================================================================

// Drive to (targetX, targetY). If stopForCube is true, also stop as soon as the
// front sensors see something closer than CUBE_STOP_DIST near the target.
void driveToPoint(double targetX, double targetY, bool stopForCube) {
  say("Driving to (" + String(targetX, 2) + ", " + String(targetY, 2) + ")");
  updatePose();
  turnToHeading(atan2(targetY - poseY, targetX - poseX));   // aim once at the start
  stage('7', "Driving");

  bool movingFast = false;
  while (true) {
    updatePose();
    double dx = targetX - poseX;
    double dy = targetY - poseY;
    double dist = sqrt(dx * dx + dy * dy);
    double ahead = dx * cos(poseTheta) + dy * sin(poseTheta);   // target distance straight ahead
    double error = wrapAngle(atan2(dy, dx) - poseTheta);

    statusReport(movingFast ? "Driving" : "Creeping", targetX, targetY);

    if (stopForCube && dist < CUBE_CHECK_DIST) {
      double front = frontDistance();
      if (front < CUBE_STOP_DIST) {
        stopMotors();
        say("Cube ahead at " + String(front, 3) + " m: stopped (" + String(dist, 3) +
            " m from the site centre)");
        return;
      }
    }

    // arrived, or level with / past the target: stop, never turn around
    if (dist < STOP_DIST || ahead < STOP_DIST) {
      stopMotors();
      say("Reached (" + String(targetX, 2) + ", " + String(targetY, 2) + "): now at x=" +
          String(poseX, 3) + " y=" + String(poseY, 3));
      return;
    }

    if (dist > SLOW_DIST) {
      if (fabs(error) > BIG_ERROR) {        // way off course: stop and turn in place
        say("Off course by " + String(error / DEG, 1) + " deg: re-aiming");
        stopMotors();
        delay(SETTLE_MS);
        turnToHeading(atan2(dy, dx));
        movingFast = false;
        continue;
      }
      driveSteer(DRIVE_SPEED, error);       // far: keep moving, steer as we go
      movingFast = true;
      delay(20);
    } else {
      if (movingFast) {                     // just entered the last 30 cm: stop,
        stage('8', "Last 30 cm: re-aim, then creep");
        stopMotors();                       // let the vision system catch up,
        delay(SETTLE_MS);                   // and re-aim from close up
        updatePose();
        turnToHeading(atan2(targetY - poseY, targetX - poseX));
        stage('8', "Creeping");
        movingFast = false;
        continue;
      }
      driveForward(SLOW_DRIVE_SPEED);       // close: short pulse, stop, fresh reading
      delay(CREEP_MS);
      stopMotors();
      delay(SETTLE_MS);
    }
  }
}

// =====================================================================
// PART 1: START SITE + MISSION
// =====================================================================
void determineStartSite() {
  stage('5', "Working out the start site");
  updatePose();
  say("Start: x=" + String(poseX, 3) + " y=" + String(poseY, 3) +
      " heading=" + String(poseTheta / DEG, 1) + " deg");
  if (poseY > 1.0) {
    startedAtA = true;
    siteX = 0.28;
    siteY = 0.5;
    say("Started at A, going to B");
  } else {
    startedAtA = false;
    siteX = 0.176;
    siteY = 1.5;
    say("Started at B, going to A");
  }
}

void doMission() {
  stage('M', "Mission");
  // TODO: lower the scoop, push the cube into the wall, light sensor -> FOAM/PLASTIC,
  //       back off, lift with the servo, load cell -> weight class. Send both with
  //       Enes100.mission(...). Leave the scoop stowed and the cube on it at the end.
  say("Mission code not written yet: skipping");
}

// =====================================================================
// PART 2: OBSTACLES
// =====================================================================

// Which row (0, 1, 2) is closest to this y?
int nearestRow(double y) {
  int best = 0;
  for (int i = 1; i < 3; i++)
    if (fabs(y - ROW_Y[i]) < fabs(y - ROW_Y[best])) best = i;
  return best;
}

// Next row to try: the closest row we haven't tried yet. On a tie, the higher row
// (toward the limbo side). Returns -1 if every row has been tried.
int pickNextRow(int row, const bool tried[3]) {
  int best = -1;
  for (int i = 2; i >= 0; i--) {             // high rows first, so they win ties
    if (tried[i]) continue;
    if (best < 0 || abs(i - row) < abs(best - row)) best = i;
  }
  return best;
}

// Stopped and facing east at a check point: is the slot ahead blocked?
bool slotBlocked(double &reading) {
  stage('C', "Checking the slot ahead");
  stopMotors();
  delay(200);
  reading = frontDistanceMedian();
  bool blocked = reading < SLOT_BLOCKED_DIST;
  say("Slot check at x=" + String(poseX, 2) + ", row y=" + String(ROW_Y[currentRow], 1) +
      ": left " + String(readLeft(), 2) + " m, right " + String(readRight(), 2) +
      " m, using " + String(reading, 2) + " m -> " + (blocked ? "BLOCKED" : "clear"));
  return blocked;
}

// Get to check point `col` (0 = before column 1, 1 = between the columns), then find
// an open slot, shifting rows until one is clear. Leaves us facing east in that row.
void passColumn(int col) {
  bool tried[3] = {false, false, false};
  double reading[3] = {0, 0, 0};

  say("Column " + String(col + 1) + ": going to check point x=" + String(CHECK_X[col], 2));
  driveToPoint(CHECK_X[col], ROW_Y[currentRow], false);
  turnToHeading(0);

  while (true) {
    tried[currentRow] = true;
    if (!slotBlocked(reading[currentRow])) {
      say("Column " + String(col + 1) + ": going through row y=" + String(ROW_Y[currentRow], 1));
      return;
    }

    int next = pickNextRow(currentRow, tried);
    if (next < 0) {
      // Every slot looked blocked: a sensor must be wrong. Take the row that looked most open.
      int best = 0;
      for (int i = 1; i < 3; i++) if (reading[i] > reading[best]) best = i;
      say("Every slot looked blocked! Going through the most open one (row y=" +
          String(ROW_Y[best], 1) + "). Check the sensors.");
      if (best != currentRow) {
        currentRow = best;
        driveToPoint(CHECK_X[col], ROW_Y[currentRow], false);
        turnToHeading(0);
      }
      return;
    }

    say("Shifting from row y=" + String(ROW_Y[currentRow], 1) + " to y=" + String(ROW_Y[next], 1));
    currentRow = next;
    driveToPoint(CHECK_X[col], ROW_Y[currentRow], false);
    turnToHeading(0);
  }
}

void drivePastObstacles() {
  updatePose();
  currentRow = nearestRow(poseY);
  say("PART 2: obstacles. Starting in row y=" + String(ROW_Y[currentRow], 1));
  passColumn(0);
  passColumn(1);
  driveToPoint(LINEUP_X, ROW_Y[currentRow], false);   // through the gap, clear of column 2
  say("Past both obstacle columns");
}

// =====================================================================
// PART 3: LIMBO
// =====================================================================

// The marker may be hidden under the limbo, so this loop does NOT use updatePose()
// (which would stop and wait). While the marker is visible it steers east; while it's
// hidden it drives straight; it stops past GOAL_X or if hidden for too long.
void goUnderLimbo() {
  say("PART 3: limbo");
  driveToPoint(LINEUP_X, LIMBO_ROW_Y, false);
  turnToHeading(0);
  stage('U', "Driving under the limbo");

  unsigned long lastSeen = millis();
  bool blind = false;
  while (true) {
    if (markerVisible()) {
      poseX = Enes100.getX();
      poseY = Enes100.getY();
      poseTheta = Enes100.getTheta();
      if (blind) {
        say("Marker visible again at x=" + String(poseX, 2));
        blind = false;
      }
      lastSeen = millis();
      if (poseX > GOAL_X) break;
      driveSteer(DRIVE_SPEED, wrapAngle(0 - poseTheta));
      statusReport("Limbo", GOAL_X, LIMBO_ROW_Y);
    } else {
      if (!blind) {
        say("Marker hidden (under the limbo?): driving straight");
        blind = true;
      }
      driveForward(DRIVE_SPEED);
      if (millis() - lastSeen > LIMBO_BLIND_MAX_MS) {
        say("Marker hidden for too long: stopping");
        break;
      }
    }
    delay(20);
  }
  stopMotors();
  say("Limbo done at x=" + String(poseX, 2));
}

// =====================================================================
// TEST MODES
// =====================================================================
void motorTest() {
  stage('2', "Motor test (wheels off the table)");
  say("ALL wheels FORWARD");
  driveForward(255);  delay(1500); stopMotors(); delay(1000);
  say("ALL wheels BACKWARD");
  driveBackward(255); delay(1500); stopMotors(); delay(1000);
  say("TURN LEFT: left wheels back, right wheels forward");
  turnLeft(255);      delay(1500); stopMotors(); delay(1000);
  say("TURN RIGHT: left wheels forward, right wheels back");
  turnRight(255);     delay(1500); stopMotors(); delay(1000);
  setLeft(0);
  setRight(0);
  say("Motor test done");
}

void sensorTest() {
  stage('T', "Sensor test: hold your hand in front of ONE sensor at a time");
  while (true) {
    double l = readLeft();
    delay(30);
    double r = readRight();
    say("left (pin " + String(LEFT_ECHO) + "): " + String(l, 3) + " m   right (pin A0): " +
        String(r, 3) + " m");
    delay(500);
  }
}

// =====================================================================
// MAIN
// =====================================================================
void setup() {
  Serial.begin(9600);             // USB Serial Monitor (doesn't wait for a laptop)
  matrix.begin();
  stage('1', "Setup started");

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  if (HAS_PWM) {
    pinMode(LEFT_EN, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
  }
  setLeft(0);
  setRight(0);

  pinMode(LEFT_TRIG, OUTPUT);     // shared trigger
  digitalWrite(LEFT_TRIG, LOW);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_ECHO, INPUT);

  say("Settings: marker " + String(MARKER_ID) + ", room " + String(ROOM) +
      ", start at part " + String(START_AT_PART));
  delay(2000);

  if (MOTOR_TEST)  { motorTest();  return; }
  if (SENSOR_TEST) { sensorTest(); return; }   // never returns

  stage('3', "Connecting to the vision system");
  Enes100.begin(TEAM_NAME, MATERIAL, MARKER_ID, ROOM);
  visionConnected = true;
  say("Connected");

  stage('4', "Waiting to see marker " + String(MARKER_ID));
  updatePose();
  say("Marker seen");

  if (START_AT_PART <= 1) {
    say("PART 1: mission site");
    determineStartSite();
    driveToPoint(siteX, siteY, true);
    doMission();
  }
  if (START_AT_PART <= 2) drivePastObstacles();
  if (START_AT_PART <= 3) goUnderLimbo();

  stage('9', "Finished");
}

void loop() {

}
