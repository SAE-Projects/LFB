// ================= MOTOR PINS =================
#define LIN1 2
#define LIN2 4
#define RIN1 6
#define RIN2 7
#define STBY 13
#define PWMA 3
#define PWMB 5

// ================= SENSOR PINS =================
#define S0 14
#define S1 15
#define S2 18
#define S3 17
#define OUT A2

#define NUM_SENSORS 16

// ================= PARAMETERS =================
float Kp = 0.12;
float Kd = 0.06;

int baseSpeed = 55;
int maxTurn   = 60;

// EARLIER TURN DETECTION
#define LEFT_POS_TH   -2600
#define RIGHT_POS_TH   2600

#define TURN_COMMIT_TIME 130

// ================= GLOBALS =================
int sensorRaw[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorNorm[NUM_SENSORS];

long position = 0;
long lastPosition = 0;
long lastError = 0;

unsigned long turnStart = 0;
bool turningLeft  = false;
bool turningRight = false;

// ================= SETUP =================
void setup() {
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  pinMode(LIN1, OUTPUT); pinMode(LIN2, OUTPUT);
  pinMode(RIN1, OUTPUT); pinMode(RIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  calibrateSensors(600);
}

// ================= LOOP =================
void loop() {

  readSensors();
  normalizeSensors();
  computePosition();

  unsigned long now = millis();

  // ================= SHARP TURN COMMIT =================
  if (turningLeft) {
    unsigned long t = now - turnStart;

    if (t < TURN_COMMIT_TIME) {

      // HARD ENTRY + SMOOTH EXIT
      if (t < 70)
        driveMotors(25, baseSpeed + 45);   // SNAP LEFT
      else
        driveMotors(40, baseSpeed + 25);   // STABILIZE

      return;
    }
    else turningLeft = false;
  }

  if (turningRight) {
    unsigned long t = now - turnStart;

    if (t < TURN_COMMIT_TIME) {

      if (t < 70)
        driveMotors(baseSpeed + 45, 25);   // SNAP RIGHT
      else
        driveMotors(baseSpeed + 25, 40);   // STABILIZE

      return;
    }
    else turningRight = false;
  }

  // ================= TURN DETECTION =================
  if (position < LEFT_POS_TH) {
    turningLeft = true;
    turnStart = now;
    return;
  }

  if (position > RIGHT_POS_TH) {
    turningRight = true;
    turnStart = now;
    return;
  }

  // ================= LINE FOLLOW =================
  long error = -position;

  if (abs(error) < 200) error = 0;

  long P = error * Kp;
  long D = (error - lastError) * Kd;
  lastError = error;

  long turn = constrain(P + D, -maxTurn, maxTurn);

  int leftPWM  = baseSpeed - turn;
  int rightPWM = baseSpeed + turn;

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveMotors(leftPWM, rightPWM);
}

// ================= POSITION =================
void computePosition() {
  long sum = 0, weighted = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int w = (i * 1000) - 7500;
    weighted += (long)sensorNorm[i] * w;
    sum += sensorNorm[i];
  }

  if (sum > 300) position = weighted / sum;
  else position = lastPosition;

  lastPosition = position;
}

// ================= SENSOR =================
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(S0, i & 1);
    digitalWrite(S1, i & 2);
    digitalWrite(S2, i & 4);
    digitalWrite(S3, i & 8);
    delayMicroseconds(5);
    sensorRaw[i] = analogRead(OUT);
  }
}

void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = constrain(sensorRaw[i], sensorMin[i], sensorMax[i]);
    sensorNorm[i] = (long)(v - sensorMin[i]) * 1000 /
                    (sensorMax[i] - sensorMin[i] + 1);
  }
}

// ================= MOTORS =================
void driveMotors(int l, int r) {
  digitalWrite(LIN1, HIGH);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, HIGH);
  digitalWrite(RIN2, LOW);

  analogWrite(PWMA, l);
  analogWrite(PWMB, r);
}

// ================= CALIBRATION =================
void calibrateSensors(int loops) {
  for (int j = 0; j < loops; j++) {
    readSensors();
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = min(sensorMin[i], sensorRaw[i]);
      sensorMax[i] = max(sensorMax[i], sensorRaw[i]);
    }
    delay(5);
  }
}
