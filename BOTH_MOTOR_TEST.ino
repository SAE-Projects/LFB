// ===== MOTOR PINS =====
#define LIN1 2
#define LIN2 4
#define RIN1 6
#define RIN2 7
#define STBY 13
#define PWMA 3
#define PWMB 5

void setup() {

  pinMode(LIN1, OUTPUT);
  pinMode(LIN2, OUTPUT);
  pinMode(RIN1, OUTPUT);
  pinMode(RIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);   // Enable motor driver
}

void loop() {

  // ===== BOTH MOTORS FORWARD =====
  digitalWrite(LIN1, HIGH);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, HIGH);
  digitalWrite(RIN2, LOW);

  analogWrite(PWMA, 120);
  analogWrite(PWMB, 120);

  delay(3000);   // run 3 seconds


  // ===== STOP =====
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(2000);   // wait 2 seconds


  // ===== BOTH MOTORS BACKWARD =====
  digitalWrite(LIN1, LOW);
  digitalWrite(LIN2, HIGH);
  digitalWrite(RIN1, LOW);
  digitalWrite(RIN2, HIGH);

  analogWrite(PWMA, 120);
  analogWrite(PWMB, 120);

  delay(3000);


  // ===== STOP =====
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(3000);
}
