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

int baseSpeed = 150;
int maxTurn   = 200;

// EARLIER TURN DETECTION
#define LEFT_POS_TH   -3000
#define RIGHT_POS_TH   3000

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
  Serial.begin(9600);
}

// ================= LOOP =================
void loop() {

  readSensors();
  normalizeSensors();
  computePosition();

  unsigned long now = millis();

  // ================= SHARP TURN COMMIT =================
  if (turningLeft) {
    Serial.println("Entered Left");
    if((sensorNorm[8] > 890 &&
      sensorNorm[9] > 890) ||
      (sensorNorm[10] > 890 &&
       sensorNorm[11] > 890)){
        turningLeft = false;
        return;
       }
    else{
      driveMotors(-100, 100);
    }
     // delay(1200);
    Serial.println("Exiting Left");
  }

  if (turningRight) {
     Serial.println("Entered Right");
    if((sensorNorm[8] > 890 &&
      sensorNorm[9] > 890) ||
      (sensorNorm[10] > 890 &&
       sensorNorm[11] > 890)){
        turningRight = false;
        return;}
   else{
    driveMotors(100, -100);}
    // delay(1200);
    Serial.println("Exiting Right");
    
  }

  // ================= TURN DETECTION =================
  if (sensorNorm[3] > 890 &&
      sensorNorm[4] > 890 &&
      sensorNorm[5] > 890) {
    turningLeft = true;
    Serial.println("Entering Left");
    driveMotors(70,70);
    delay(150);
    return;
  }

  if (sensorNorm[9] > 890 &&
      sensorNorm[10] > 890 &&
      sensorNorm[11] > 890) {
    turningRight = true;
    Serial.println("Entering Right");
    return;
  }
  if (sensorNorm[])


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

  Serial.println(position);
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
  // clamp
  l = constrain((int)l, -200, 200);
  r = constrain((int)r, -200, 200);

  // set direction pins once per side
  if (l >= 0) { digitalWrite(LIN1, HIGH); digitalWrite(LIN2, LOW);  }
  else         { digitalWrite(LIN1, LOW);  digitalWrite(LIN2, HIGH); }

  if (r >= 0) { digitalWrite(RIN1, HIGH); digitalWrite(RIN2, LOW);  }
  else         { digitalWrite(RIN1, LOW);  digitalWrite(RIN2, HIGH); }

  analogWrite(PWMA, abs((int)l));
  analogWrite(PWMB, abs((int)r));
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
