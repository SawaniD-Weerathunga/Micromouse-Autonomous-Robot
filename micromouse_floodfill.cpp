#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <queue>
#include <string.h>

/* ========================= PIN MAP (unchanged) ========================= */
#define AIN1 19
#define AIN2 18
#define PWMA 5
#define BIN1 17
#define BIN2 16
#define PWMB 4
#define STBY 23

#define ENCODER_R_A 26
#define ENCODER_L_A 14

// Sensor physical layout (left -> right on one straight bar):
// [ 90° Left ] [ Front-Left ] [ Front-Right ] [ 90° Right ]
#define SENSOR_90_LEFT   32
#define SENSOR_FRONT_L   33
#define SENSOR_FRONT_R   34
#define SENSOR_90_RIGHT  35

/* ===================== QUICK SAFETY TOGGLES (unchanged) ===================== */
#define TURN_SIGN (-1)   // set to (-1) if right/left are reversed
const bool SWAP_SIDE_SENSORS = false;

/* ===================== CONSTANTS & THRESHOLDS (unchanged) ===================== */
#define COUNTS_PER_CELL 369
const int   MAX_PWM = 255;
int baseSpeed = 200;

const float FRONT_WALL_THRESHOLD = 10.0f;
const float SIDE_WALL_THRESHOLD  = 12.0f;

const float SIDE_DIFF_CM       = 1.5f;
const float FRONT_DIFF_TURN_CM = 1.0f;

float steeringKp = 3.5f;
float steeringKd = 0.6f;

float distanceKp = 1.5f;
float distanceKd = 1.0f;

float turnKp = 2.8f;
float turnKd = 0.7f;

const int MAX_SPEED = 180;
const int MIN_SPEED = 50;

/* =========================== STATE (unchanged) =========================== */
Adafruit_MPU6050 mpu;
float gyro_z_offset = 0.0f;
float current_angle = 0.0f;
unsigned long last_gyro_time = 0;

float prev_steering_error = 0.0f;
float prev_distance_error = 0.0f;

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

/* ===================== PROTOTYPES (your originals) ===================== */
float getFrontLeftDistanceCm();
float getFrontRightDistanceCm();
float get90LeftDistanceCm();
float get90RightDistanceCm();

void setMotorSpeed(int leftSpeed, int rightSpeed);
void driveStop();
void moveOneCell();
void turnRight90();
void turnLeft90();
void turn180();
void updateAngle();
void turnToAngle(float targetAngle);
int   smoothAnalogRead(int pin);

// (kept for reference; not used by flood-fill loop)
void checkWallsAndDecide();

/* ===================== INTERRUPTS (unchanged) ===================== */
void IRAM_ATTR countLeft()  { encoderLeft++; }
void IRAM_ATTR countRight() { encoderRight++; }

/* =================================================================
   ===============  FLOOD-FILL ALGORITHM (16×16)  ==================
   ================================================================= */

// Robot states
enum RobotState { EXPLORING, RETURN_TO_START, FINISHED };
RobotState currentState = EXPLORING;

// Heading
#define FORWARD   0
#define RIGHT     1
#define BACKWARD  2
#define LEFT      3

int current_x = 0, current_y = 0;        // start at (0,0), facing FORWARD
int previous_x = 0, previous_y = 0;
int orient = FORWARD;

// 16×16 maps
int cellsArray[16][16] = {0};  // encodes walls (your code scheme)
bool visited[16][16] = {false};

// Flood map: goal = center four cells (7,7) (8,7) (7,8) (8,8) => 0
int floodArray[16][16];

// Return map: Manhattan distance to start (0,0)
int returnFloodArray[16][16];

// Build the initial 16×16 goal-centered flood map
void initFloodCenterMap() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      int dx = min(abs(x - 7), abs(x - 8));  // distance to nearest center column
      int dy = min(abs(y - 7), abs(y - 8));  // distance to nearest center row
      floodArray[x][y] = dx + dy;            // L1 distance to 4-center region
    }
  }
}

// Encode cell walls based on robot-relative walls at this cell
// (same style you used, extended to 16×16)
void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
  if (left && right && forward) { if (orient == FORWARD) cellsArray[x][y] = 13; else if (orient == RIGHT) cellsArray[x][y] = 12; else if (orient == BACKWARD) cellsArray[x][y] = 11; else if (orient == LEFT) cellsArray[x][y] = 14; }
  else if (left && right && !forward) { if (orient == FORWARD || orient == BACKWARD) cellsArray[x][y] = 9; else if (orient == RIGHT || orient == LEFT) cellsArray[x][y] = 10; }
  else if (left && !right && forward) { if (orient == FORWARD) cellsArray[x][y] = 8; else if (orient == RIGHT) cellsArray[x][y] = 7; else if (orient == BACKWARD) cellsArray[x][y] = 6; else if (orient == LEFT) cellsArray[x][y] = 5; }
  else if (!left && right && forward) { if (orient == FORWARD) cellsArray[x][y] = 7; else if (orient == RIGHT) cellsArray[x][y] = 6; else if (orient == BACKWARD) cellsArray[x][y] = 5; else if (orient == LEFT) cellsArray[x][y] = 8; }
  else if (forward) { if (orient == FORWARD) cellsArray[x][y] = 2; else if (orient == RIGHT) cellsArray[x][y] = 3; else if (orient == BACKWARD) cellsArray[x][y] = 4; else if (orient == LEFT) cellsArray[x][y] = 1; }
  else if (left) { if (orient == FORWARD) cellsArray[x][y] = 1; else if (orient == RIGHT) cellsArray[x][y] = 2; else if (orient == BACKWARD) cellsArray[x][y] = 3; else if (orient == LEFT) cellsArray[x][y] = 4; }
  else if (right) { if (orient == FORWARD) cellsArray[x][y] = 3; else if (orient == RIGHT) cellsArray[x][y] = 4; else if (orient == BACKWARD) cellsArray[x][y] = 1; else if (orient == LEFT) cellsArray[x][y] = 2; }
}

// Bounds + wall check using cellsArray codes (same logic style)
bool isAccessible(int cx, int cy, int tx, int ty) {
  if (tx < 0 || tx >= 16 || ty < 0 || ty >= 16) return false;
  if (cx == tx) {
    if (cy > ty) { // moving SOUTH (negative Y)
      if ((cellsArray[cx][cy] == 4) || (cellsArray[cx][cy] == 5) || (cellsArray[cx][cy] == 6) ||
          (cellsArray[cx][cy] == 10) || (cellsArray[cx][cy] == 11) || (cellsArray[cx][cy] == 12) || (cellsArray[cx][cy] == 14)) return false;
      else return true;
    } else { // moving NORTH (positive Y)
      if ((cellsArray[cx][cy] == 2) || (cellsArray[cx][cy] == 7) || (cellsArray[cx][cy] == 8) ||
          (cellsArray[cx][cy] == 10) || (cellsArray[cx][cy] == 12) || (cellsArray[cx][cy] == 13) || (cellsArray[cx][cy] == 14)) return false;
      else return true;
    }
  } else if (cy == ty) {
    if (cx > tx) { // moving WEST (negative X)
      if ((cellsArray[cx][cy] == 1) || (cellsArray[cx][cy] == 5) || (cellsArray[cx][cy] == 8) ||
          (cellsArray[cx][cy] == 9) || (cellsArray[cx][cy] == 11) || (cellsArray[cx][cy] == 13) || (cellsArray[cx][cy] == 14)) return false;
      else return true;
    } else { // moving EAST (positive X)
      if ((cellsArray[cx][cy] == 3) || (cellsArray[cx][cy] == 6) || (cellsArray[cx][cy] == 7) ||
          (cellsArray[cx][cy] == 9) || (cellsArray[cx][cy] == 11) || (cellsArray[cx][cy] == 12) || (cellsArray[cx][cy] == 13)) return false;
      else return true;
    }
  }
  return false;
}

void getSurroungings(int cx, int cy, int *nX, int *nY, int *eX, int *eY, int *sX, int *sY, int *wX, int *wY) {
  *nX = cx; *nY = (cy + 1) >= 16 ? -1 : cy + 1;
  *eX = (cx + 1) >= 16 ? -1 : cx + 1; *eY = cy;
  *sX = cx; *sY = (cy - 1) < 0 ? -1 : cy - 1;
  *wX = (cx - 1) < 0 ? -1 : cx - 1; *wY = cy;
}

bool isIncrementConsistent(int cx, int cy, int map[16][16]) {
  int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
  int cVal = map[cx][cy]; int minCnt = 0;
  if ((nX != -1) && isAccessible(cx, cy, nX, nY) && (map[nX][nY] == cVal - 1)) minCnt++;
  if ((eX != -1) && isAccessible(cx, cy, eX, eY) && (map[eX][eY] == cVal - 1)) minCnt++;
  if ((sX != -1) && isAccessible(cx, cy, sX, sY) && (map[sX][sY] == cVal - 1)) minCnt++;
  if ((wX != -1) && isAccessible(cx, cy, wX, wY) && (map[wX][wY] == cVal - 1)) minCnt++;
  return minCnt > 0;
}

void makeCellConsistent(int cx, int cy, int map[16][16]) {
  int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
  int minVal = 1000;
  if ((nX != -1) && isAccessible(cx, cy, nX, nY)) minVal = min(minVal, map[nX][nY]);
  if ((eX != -1) && isAccessible(cx, cy, eX, eY)) minVal = min(minVal, map[eX][eY]);
  if ((sX != -1) && isAccessible(cx, cy, sX, sY)) minVal = min(minVal, map[sX][sY]);
  if ((wX != -1) && isAccessible(cx, cy, wX, wY)) minVal = min(minVal, map[wX][wY]);
  if (minVal != 1000) map[cx][cy] = minVal + 1;
}

void floodFillUsingQueue(int sx, int sy, int map[16][16]) {
  memset(visited, false, sizeof(visited));
  std::queue<int> q;
  if (!isIncrementConsistent(sx, sy, map)) makeCellConsistent(sx, sy, map);
  q.push(sx); q.push(sy); visited[sy][sx] = true;
  while (!q.empty()) {
    int cx = q.front(); q.pop();
    int cy = q.front(); q.pop();
    if (!isIncrementConsistent(cx, cy, map)) {
      makeCellConsistent(cx, cy, map);
      int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
      int nx[4] = {nX, eX, sX, wX}; int ny[4] = {nY, eY, sY, wY};
      for (int i=0;i<4;i++) {
        if (nx[i] != -1 && isAccessible(cx, cy, nx[i], ny[i]) && !visited[ny[i]][nx[i]]) {
          q.push(nx[i]); q.push(ny[i]); visited[ny[i]][nx[i]] = true;
        }
      }
    }
  }
}

char whereToMove(int cx, int cy, int orient, int map[16][16]) {
  int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
  int values[4] = {1000,1000,1000,1000};
  if (nX != -1 && isAccessible(cx, cy, nX, nY)) values[FORWARD]  = map[nX][nY];
  if (eX != -1 && isAccessible(cx, cy, eX, eY)) values[RIGHT]    = map[eX][eY];
  if (sX != -1 && isAccessible(cx, cy, sX, sY)) values[BACKWARD] = map[sX][sY];
  if (wX != -1 && isAccessible(cx, cy, wX, wY)) values[LEFT]     = map[wX][wY];

  int minValue = 1000;
  for (int i=0;i<4;i++) if (values[i] < minValue) minValue = values[i];

  int f_dir = orient;
  int l_dir = (orient+3)%4;
  int r_dir = (orient+1)%4;
  int b_dir = (orient+2)%4;

  if (values[f_dir] == minValue) return 'F';
  if (values[l_dir] == minValue) return 'L';
  if (values[r_dir] == minValue) return 'R';
  if (values[b_dir] == minValue) return 'B';
  return 'B';
}

int orientationUpdate(int orient, char t) {
  if (t=='L') return (orient+3)%4;
  if (t=='R') return (orient+1)%4;
  if (t=='B') return (orient+2)%4;
  return orient;
}

void advanceCoordinates(int orient, int &x, int &y) {
  if (orient == FORWARD) y++;
  else if (orient == RIGHT) x++;
  else if (orient == BACKWARD) y--;
  else if (orient == LEFT) x--;
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(150);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(SENSOR_FRONT_L, INPUT);
  pinMode(SENSOR_FRONT_R, INPUT);
  pinMode(SENSOR_90_LEFT, INPUT);
  pinMode(SENSOR_90_RIGHT, INPUT);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), countRight, RISING);

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(100);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  Serial.println("Calibrating gyro...");
  for (int i=0;i<500;i++) {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    gyro_z_offset += g.gyro.z;
    delay(2);
  }
  gyro_z_offset /= 500.0f;
  last_gyro_time = micros();

  // Build initial flood maps
  initFloodCenterMap();                       // goal-centered flood
  for (int x=0;x<16;x++) for (int y=0;y<16;y++) returnFloodArray[x][y] = x + y;  // to start

  Serial.print("Gyro offset: "); Serial.println(gyro_z_offset, 5);
  Serial.println("\n*** MICROMOUSE READY (16x16 flood fill + your sensing/motion) ***");
  delay(400);
}

/* ===================== MAIN LOOP ===================== */
void loop() {
  switch (currentState) {

    case EXPLORING: {
      // 1) Read walls using YOUR sensing code/thresholds (unchanged)
      float FL=0, FR=0, SL=0, SR=0;
      for (int i=0;i<3;i++){ FL+=getFrontLeftDistanceCm(); FR+=getFrontRightDistanceCm(); SL+=get90LeftDistanceCm(); SR+=get90RightDistanceCm(); delay(5); }
      FL/=3.0f; FR/=3.0f; SL/=3.0f; SR/=3.0f;
      if (SWAP_SIDE_SENSORS){ float t=SL; SL=SR; SR=t; }

      bool wallFront = (min(FL,FR) < FRONT_WALL_THRESHOLD);
      bool wallLeft  = (SL < SIDE_WALL_THRESHOLD);
      bool wallRight = (SR < SIDE_WALL_THRESHOLD);

      // 2) Update cell wall encoding at current pose
      updateCells(current_x, current_y, orient, wallLeft, wallRight, wallFront);

      // 3) If not at goal cell (0), re-flood to keep map consistent
      if (floodArray[current_x][current_y] != 0) {
        floodFillUsingQueue(current_x, current_y, floodArray);
      } else {
        Serial.println(">>> Center reached! Switching to RETURN_TO_START.");
        currentState = RETURN_TO_START;
        turn180();
        orient = orientationUpdate(orient, 'B');
        delay(150);
        return;
      }

      // 4) Choose next move by flood map (orientation-biased)
      char decision = whereToMove(current_x, current_y, orient, floodArray);
      Serial.print("[EXPLORE] @("); Serial.print(current_x); Serial.print(","); Serial.print(current_y);
      Serial.print(") ori="); Serial.print(orient);
      Serial.print(" -> "); Serial.println(decision);

      if (decision=='L'){ turnLeft90();  orient = orientationUpdate(orient, 'L'); delay(120); }
      else if (decision=='R'){ turnRight90(); orient = orientationUpdate(orient, 'R'); delay(120); }
      else if (decision=='B'){ turn180(); orient = orientationUpdate(orient, 'B'); delay(200); }

      // 5) Move forward one cell with your motion code
      moveOneCell();
      previous_x = current_x; previous_y = current_y;
      advanceCoordinates(orient, current_x, current_y);
      delay(120);
      break;
    }

    case RETURN_TO_START: {
      // Stop when we return to (0,0)
      if (current_x == 0 && current_y == 0) {
        Serial.println(">>> Returned to START. Finished.");
        currentState = FINISHED;
        setMotorSpeed(0,0);
        return;
      }

      // Use the simple return flood map (x+y) to bias toward start
      char decision = whereToMove(current_x, current_y, orient, returnFloodArray);
      Serial.print("[RETURN] @("); Serial.print(current_x); Serial.print(","); Serial.print(current_y);
      Serial.print(") ori="); Serial.print(orient);
      Serial.print(" -> "); Serial.println(decision);

      if (decision=='L'){ turnLeft90();  orient = orientationUpdate(orient, 'L'); delay(120); }
      else if (decision=='R'){ turnRight90(); orient = orientationUpdate(orient, 'R'); delay(120); }
      else if (decision=='B'){ turn180(); orient = orientationUpdate(orient, 'B'); delay(200); }

      moveOneCell();
      previous_x = current_x; previous_y = current_y;
      advanceCoordinates(orient, current_x, current_y);
      delay(100);
      break;
    }

    case FINISHED: {
      setMotorSpeed(0,0);
      while(true) { delay(1000); }
      break;
    }
  }
}

/* ===================== YOUR ORIGINAL SENSORS (UNCHANGED) ===================== */
int smoothAnalogRead(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(pin); delayMicroseconds(200); }
  return (int)(sum / 10);
}

// FRONT sensors
float getFrontLeftDistanceCm() {
  float x = (float)smoothAnalogRead(SENSOR_FRONT_L);
  return (1.947e-12*pow(x,4)) - (1.369e-8*pow(x,3)) + (3.513e-5*pow(x,2)) - (0.04132*x) + 22.63;
}
float getFrontRightDistanceCm() {
  float x = (float)smoothAnalogRead(SENSOR_FRONT_R);
  return (1.609e-12*pow(x,4)) - (1.18e-8*pow(x,3)) + (3.166e-5*pow(x,2)) - (0.0392*x) + 22.95;
}

// SIDE sensors (90°)
float get90LeftDistanceCm() {
  float x = (float)smoothAnalogRead(SENSOR_90_LEFT);
  return (1.181e-12*pow(x,4)) - (9.183e-9*pow(x,3)) + (2.674e-5*pow(x,2)) - (0.0366*x) + 22.88;
}
float get90RightDistanceCm() {
  float x = (float)smoothAnalogRead(SENSOR_90_RIGHT);
  return (1.608e-12*pow(x,4)) - (1.129e-8*pow(x,3)) + (2.927e-5*pow(x,2)) - (0.03573*x) + 20.85;
}

/* ===================== YOUR ORIGINAL MOTORS / TURNS (UNCHANGED) ===================== */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); leftSpeed = -leftSpeed; }
  if (rightSpeed >= 0){ digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); rightSpeed = -rightSpeed; }

  analogWrite(PWMA, constrain(leftSpeed,  0, 255));
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

void driveStop() {
  setMotorSpeed(-60, -60); delay(25);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void updateAngle() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  unsigned long now = micros();
  float dt = (now - last_gyro_time) / 1000000.0f;
  last_gyro_time = now;
  float rateZ = (g.gyro.z - gyro_z_offset);
  current_angle += rateZ * dt * (180.0f / PI);
}

/* --- Your moveOneCell() (unchanged) --- */
void moveOneCell() {
  encoderLeft = 0;
  encoderRight = 0;
  prev_steering_error = 0.0f;
  prev_distance_error = (float)COUNTS_PER_CELL;

  while ((encoderLeft + encoderRight) / 2 < COUNTS_PER_CELL) {
    long avg = (encoderLeft + encoderRight) / 2;
    float distErr = COUNTS_PER_CELL - avg;
    float distDer = distErr - prev_distance_error;
    prev_distance_error = distErr;

    int currentSpeed = constrain((int)(distanceKp * distErr + distanceKd * distDer), MIN_SPEED, baseSpeed);

    float SL=0, SR=0;
    for (int i=0;i<2;i++){ SL += get90LeftDistanceCm(); SR += get90RightDistanceCm(); delay(2); }
    SL*=0.5f; SR*=0.5f;
    if (SWAP_SIDE_SENSORS){ float t=SL; SL=SR; SR=t; }

    bool leftWall  = (SL < SIDE_WALL_THRESHOLD);
    bool rightWall = (SR < SIDE_WALL_THRESHOLD);

    float steeringError = 0.0f;
    if (leftWall && rightWall)        steeringError = (SR - SL);
    else if (leftWall && !rightWall)  steeringError = -(SL - 8.0f);
    else if (!leftWall && rightWall)  steeringError =  (SR - 8.0f);

    float steerDer = (steeringError - prev_steering_error);
    prev_steering_error = steeringError;
    float corr = (steeringKp * steeringError) + (steeringKd * steerDer);

    const int MIN_FWD = 60;
    float maxCorr = (float)baseSpeed - MIN_FWD;
    corr = constrain(corr, -maxCorr, maxCorr);

    int L = (int)(baseSpeed - corr);
    int R = (int)(baseSpeed + corr);
    L = constrain(L, MIN_FWD, MAX_SPEED);
    R = constrain(R, MIN_FWD, MAX_SPEED);
    setMotorSpeed(L, R);

    float frontMin = min(getFrontLeftDistanceCm(), getFrontRightDistanceCm());
    if (frontMin < 5.5f) { Serial.println("EMERGENCY FRONT STOP"); break; }
    delay(10);
  }

  driveStop();
}

/* --- Your turn helpers (unchanged) --- */
void turnToAngle(float targetAngleRaw) {
  float targetAngle = TURN_SIGN * targetAngleRaw;
  current_angle = 0.0f;
  last_gyro_time = micros();
  float prev_error = targetAngle;
  unsigned long start = millis();

  while (fabsf(targetAngle - current_angle) > 1.2f) {
    updateAngle();
    if (millis() - start > 3000) { Serial.println("Turn timeout!"); break; }
    float err = targetAngle - current_angle;
    float der = err - prev_error;
    prev_error = err;

    float u = turnKp * err + turnKd * der;
    int turnSpeed = constrain((int)fabsf(u), 75, 160);

    if (targetAngle > 0) { // RIGHT
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    } else {               // LEFT
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMA, turnSpeed);
    analogWrite(PWMB, turnSpeed);
    delay(8);
  }

  setMotorSpeed((targetAngle>0)?-70:70, (targetAngle>0)?70:-70);
  delay(40);
  driveStop();
  delay(120);
}

void turnRight90() { Serial.println("RIGHT 90");  turnToAngle(+90.0f); }
void turnLeft90()  { Serial.println("LEFT 90");   turnToAngle(-90.0f); }
void turn180()     { Serial.println("TURN 180");  turnToAngle(180.0f); }

/* ===================== (Optional) Old rule-based decision ===================== */
void checkWallsAndDecide() {
  // Not used in the flood-fill loop; kept for testing.
  float FL=0, FR=0, SL=0, SR=0;
  for (int i=0;i<3;i++){
    FL += getFrontLeftDistanceCm();
    FR += getFrontRightDistanceCm();
    SL += get90LeftDistanceCm();
    SR += get90RightDistanceCm();
    delay(5);
  }
  FL/=3.0f; FR/=3.0f; SL/=3.0f; SR/=3.0f;
  if (SWAP_SIDE_SENSORS) { float tmp=SL; SL=SR; SR=tmp; }
  bool wallFront = (min(FL,FR) < FRONT_WALL_THRESHOLD);
  bool wallLeft  = (SL < SIDE_WALL_THRESHOLD);
  bool wallRight = (SR < SIDE_WALL_THRESHOLD);
  if (wallFront && wallLeft && wallRight) { turn180(); return; }
  if (wallFront && wallLeft && !wallRight) { turnRight90(); return; }
  if (wallFront && wallRight && !wallLeft) { turnLeft90();  return; }
  if (wallFront){
    float sideDiff = SR - SL;
    if (fabsf(sideDiff) > SIDE_DIFF_CM) { if (sideDiff>0) turnRight90(); else turnLeft90(); return; }
    float df = FR - FL;
    if (fabsf(df) > FRONT_DIFF_TURN_CM) { if (df>0) turnRight90(); else turnLeft90(); return; }
    turnRight90(); return;
  }
}