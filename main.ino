// setting the pin connections
// === Motor Pins ===
const int enL = 9;
const int in1L = 7;
const int in2L = 6;

const int enR = 3;
const int in1R = 4;
const int in2R = 5;

// === Ultrasonic Sensor ===
const int trigPin = 10;
const int echoPin = 11;

// === setting Constants ===
const int BASE_SPEED = 100;
const int MIN_SPEED = 65;
const int SPIN_SPEED = 110;

const float DESIRED_DISTANCE = 10.0;
const float ERROR_MARGIN = 2.0;
const float WALL_LOST_THRESHOLD = 50.0;

const unsigned long SPIN_DURATION = 260;
const unsigned long ADVANCE_DURATION = 210;
const unsigned long STOP_AFTER_TIME = 500;
const unsigned long STARTUP_DELAY = 400;
const unsigned long STRAIGHT_TIME = 220;  // 0.1s straight after turn

// === PID Constants ===
float kp = 3.8; //3.6
float ki = 0.4;
float kd = 1.6;

const unsigned long SAMPLE_TIME = 50;
const float DERIVATIVE_FILTER_ALPHA = 0.6; //0.8

// === State Variables ===
float error = 0, previousError = 0, integral = 0, derivative = 0, filteredDerivative = 0;
unsigned long lastTime = 0;
unsigned long spinStartTime = 0;
unsigned long postSpinStartTime = 0;
unsigned long startupTime = 0;
unsigned long postTurnStraightStartTime = 0;

int spinStage = 0;  // 0 = idle, 1 = spinning, 2 = advancing
bool firstSpinCompleted = false;
bool hasStoppedAfterFirstSpin = false;
bool programEnded = false;
bool inPostTurnStraight = false;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(enL, OUTPUT); pinMode(in1L, OUTPUT); pinMode(in2L, OUTPUT);
  pinMode(enR, OUTPUT); pinMode(in1R, OUTPUT); pinMode(in2R, OUTPUT);

  lastTime = millis();
  startupTime = millis();
}
//== calculating distance from sensors and sampling out reading to ignore noise readings==
float readDistance() {
  float sum = 0;
  int samples = 5;
  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 20000);
    if (duration == 0) duration = 3000;
    float d = duration * 0.0343 / 2.0;
    sum += d;
    delay(5);
  }
  return sum / samples;
}

void setLeftMotor(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  analogWrite(enL, speed);
  digitalWrite(in1L, forward ? HIGH : LOW);
  digitalWrite(in2L, forward ? LOW : HIGH);
}

void setRightMotor(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  analogWrite(enR, speed);
  digitalWrite(in1R, forward ? HIGH : LOW);
  digitalWrite(in2R, forward ? LOW : HIGH);
}

void spinRight() {
  setLeftMotor(SPIN_SPEED, true);
  setRightMotor(SPIN_SPEED, false);
}

void moveForward() {
  setLeftMotor(BASE_SPEED, true);
  setRightMotor(BASE_SPEED, true);
}

void stopMotors() {
  setLeftMotor(0, true);
  setRightMotor(0, true);
}
// == we set 3 different stages for the bot spin state to take a right turn when no wall is detected 
//== PID follower when the wall is present before taking a turn 
//== Straight follower after taking a turn it goes straight and stops at 20cms
void loop() {
  if (programEnded) return;

  float distance = readDistance();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  // === Stop after time post-spin ===
  if (firstSpinCompleted && !hasStoppedAfterFirstSpin &&
      currentTime - postSpinStartTime >= STOP_AFTER_TIME) {
    stopMotors();
    hasStoppedAfterFirstSpin = true;
    programEnded = true;
    Serial.println("✅ Traveled ~21cm after turn → Stopping permanently.");
    return;
  }

  // === Spin handling ===
  if (spinStage == 1) {
    if (currentTime - spinStartTime < SPIN_DURATION) {
      spinRight();
      Serial.println("🔄 Spinning 90° right...");
      return;
    } else {
      spinStage = 2;
      spinStartTime = currentTime;
      Serial.println("➡ Spin done. Moving forward briefly...");
      return;
    }
  }

  if (spinStage == 2) {
    if (currentTime - spinStartTime < ADVANCE_DURATION) {
      moveForward();
      return;
    } else {
      spinStage = 0;
      if (!firstSpinCompleted) {
        firstSpinCompleted = true;
        postSpinStartTime = millis();

        // Begin straight motion before PID
        inPostTurnStraight = true;
        postTurnStraightStartTime = currentTime;
        Serial.println("➡ Advance complete. Moving straight for 0.1s...");
        return;
      }
    }
  }

  // === Straight move after turn ===
  if (inPostTurnStraight) {
    if (currentTime - postTurnStraightStartTime < STRAIGHT_TIME) {
      moveForward();
      return;
    } else {
      inPostTurnStraight = false;
      Serial.println("➡ Straight motion done. Resuming PID wall following...");
    }
  }

  // === Initiate spin if wall lost ===
  if (spinStage == 0 && !firstSpinCompleted &&
      (millis() - startupTime > STARTUP_DELAY) &&
      distance > WALL_LOST_THRESHOLD && distance < 300.0) {
    spinStage = 1;
    spinStartTime = currentTime;
    Serial.println("🚫 Wall lost. Starting spin...");
    return;
  }

  // === PID Wall Following ===
  if (currentTime - lastTime >= SAMPLE_TIME && !hasStoppedAfterFirstSpin) {
    error = distance - DESIRED_DISTANCE;

    integral += error * deltaTime;
    integral = constrain(integral, -50, 50);

    derivative = (error - previousError) / deltaTime;
    filteredDerivative = DERIVATIVE_FILTER_ALPHA * filteredDerivative +
                         (1 - DERIVATIVE_FILTER_ALPHA) * derivative;

    float output = kp * error + ki * integral + kd * filteredDerivative;
    output = constrain(output, -20, 20);
//== making sure that the bot doesn’t speed up when its moving away 
    float absError = abs(error);
    float speedScale = constrain(1.0 - (absError / 10.0), 0.3, 1.0);
    float baseSpeed = BASE_SPEED * speedScale;

    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;

    leftSpeed = max(leftSpeed, MIN_SPEED);
    rightSpeed = max(rightSpeed, MIN_SPEED);

    setLeftMotor(leftSpeed, true);
    setRightMotor(rightSpeed, true);

    Serial.print("Distance: "); Serial.print(distance);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" | R: "); Serial.println(rightSpeed);

    previousError = error;
    lastTime = currentTime;
  }
}
