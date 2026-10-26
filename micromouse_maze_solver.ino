/*
  ESP32 Maze Robot - Higher Speed Test Code

  Driver:
    HW-095 / L298N

  Sensors:
    3 x HC-SR04

  TEST_MODE:
    0 = Motor test
    1 = Ultrasonic sensor test
    2 = Higher-speed straight-drive test
    3 = Left-wall-following test
    4 = Turn test

  IMPORTANT:
    Each HC-SR04 ECHO pin must use a voltage divider
    before connecting to an ESP32 GPIO.

  FIXES APPLIED:
    - runTurnTest() referenced TURN_LEFT_90_MS / TURN_RIGHT_90_MS,
      which didn't exist (declared names are LEFT_TURN_90_MS /
      RIGHT_TURN_90_MS) - this would not compile. Fixed to use the
      correct declared names.
    - followLeftWall() had the steering correction signs inverted:
      it sped up the wheel on the side it needed to slow down,
      which steers AWAY from the wall on any drift instead of
      correcting back toward it (positive feedback). Swapped to
      the arrangement that was already noted as "alternative" in
      a comment - that one was actually correct.
    - Added a front slowdown zone (FRONT_SLOWDOWN_DISTANCE) since
      base speeds nearly doubled vs. the previous slow-speed
      version but the stop distance and reaction margin did not
      change - at these speeds the robot needs more room to react.
    - Turn timings (LEFT_TURN_90_MS / RIGHT_TURN_90_MS) are UNCHANGED
      from the old slow-speed values even though TURN_FAST_SPEED
      increased - these will almost certainly overshoot 90 degrees
      now. You MUST recalibrate with TEST_MODE 4 before trusting
      turns at this speed. Left as-is here since only real testing
      on your hardware can find the correct value.
*/

// =====================================================
// SELECT TEST MODE
// =====================================================

// 0 = motor test
// 1 = sensor test
// 2 = straight drive
// 3 = left-wall following
// 4 = turn test

#define TEST_MODE 2

// =====================================================
// PIN DEFINITIONS
// =====================================================

// Left HC-SR04
const int trigLeft = 22;
const int echoLeft = 23;

// Front HC-SR04
const int trigFront = 19;
const int echoFront = 21;

// Right HC-SR04
const int trigRight = 5;
const int echoRight = 18;

// Left motor
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 25;
const int LEFT_ENA = 33;

// Right motor
const int RIGHT_IN1 = 14;
const int RIGHT_IN2 = 27;
const int RIGHT_ENA = 13;

// Motor direction configuration
// Change only if a wheel rotates in the wrong physical direction.
const bool LEFT_REVERSED = false;
const bool RIGHT_REVERSED = true;

// =====================================================
// HIGHER SPEED SETTINGS
// =====================================================

// These are intentionally higher than the previous values.
const int LEFT_BASE_SPEED = 150;
const int RIGHT_BASE_SPEED = 145;

// Used for direct motor tests
const int TEST_LEFT_SPEED = 150;
const int TEST_RIGHT_SPEED = 150;

const int MAX_SPEED = 200;

// Turn speeds
const int TURN_FAST_SPEED = 155;
const int TURN_SLOW_SPEED = 90;

// Calibrate these on the actual maze floor.
// NOTE: these were tuned for a slower TURN_FAST_SPEED previously.
// Re-run TEST_MODE 4 and adjust before trusting these values -
// they will very likely need to increase now that turn speed is higher.
const unsigned long LEFT_TURN_90_MS = 500;
const unsigned long RIGHT_TURN_90_MS = 500;
const unsigned long TURN_180_MS = 1000;

// =====================================================
// ULTRASONIC SETTINGS
// =====================================================

const float SOUND_SPEED = 0.0343f;
const float MAX_DISTANCE_CM = 120.0f;

const unsigned long ECHO_TIMEOUT_US = 25000UL;
const int SENSOR_GAP_MS = 50;

// =====================================================
// WALL FOLLOWING SETTINGS
// =====================================================

const float WALL_SETPOINT = 8.4f;
const float WALL_THRESHOLD = 14.0f;
const float OPEN_THRESHOLD = 22.0f;
const float FRONT_BLOCKED_DISTANCE = 8.0f;

// NEW: start easing off speed before the hard-stop distance so the
// robot has more reaction room at these higher base speeds.
const float FRONT_SLOWDOWN_DISTANCE = 18.0f;
const int FRONT_SLOWDOWN_REDUCTION = 30;

const float KP = 2.5f;
const int MAX_CORRECTION = 30;

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void setMotor(
  int in1Pin,
  int in2Pin,
  int enablePin,
  int speed,
  bool reversed
) {
  bool forward = speed >= 0;

  if (reversed) {
    forward = !forward;
  }

  int pwm = constrain(abs(speed), 0, 255);

  if (pwm == 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, 0);
    return;
  }

  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  analogWrite(enablePin, pwm);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  setMotor(
    LEFT_IN1,
    LEFT_IN2,
    LEFT_ENA,
    leftSpeed,
    LEFT_REVERSED
  );

  setMotor(
    RIGHT_IN1,
    RIGHT_IN2,
    RIGHT_ENA,
    rightSpeed,
    RIGHT_REVERSED
  );
}

void stopMotors() {
  driveMotors(0, 0);
}

void pivotLeft(int speed) {
  setMotor(
    LEFT_IN1,
    LEFT_IN2,
    LEFT_ENA,
    -speed,
    LEFT_REVERSED
  );

  setMotor(
    RIGHT_IN1,
    RIGHT_IN2,
    RIGHT_ENA,
    speed,
    RIGHT_REVERSED
  );
}

void pivotRight(int speed) {
  setMotor(
    LEFT_IN1,
    LEFT_IN2,
    LEFT_ENA,
    speed,
    LEFT_REVERSED
  );

  setMotor(
    RIGHT_IN1,
    RIGHT_IN2,
    RIGHT_ENA,
    -speed,
    RIGHT_REVERSED
  );
}

// =====================================================
// ULTRASONIC FUNCTIONS
// =====================================================

float pingOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration =
    pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return MAX_DISTANCE_CM;
  }

  float distanceCm =
    duration * SOUND_SPEED / 2.0f;

  if (distanceCm < 2.0f ||
      distanceCm > MAX_DISTANCE_CM) {
    return MAX_DISTANCE_CM;
  }

  return distanceCm;
}

float median3(float a, float b, float c) {
  if (a > b) {
    float temp = a;
    a = b;
    b = temp;
  }

  if (b > c) {
    float temp = b;
    b = c;
    c = temp;
  }

  if (a > b) {
    float temp = a;
    a = b;
    b = temp;
  }

  return b;
}

float filteredDistance(int trigPin, int echoPin) {
  float a = pingOnce(trigPin, echoPin);
  delay(SENSOR_GAP_MS);

  float b = pingOnce(trigPin, echoPin);
  delay(SENSOR_GAP_MS);

  float c = pingOnce(trigPin, echoPin);

  return median3(a, b, c);
}

void readAllSensors(
  float &leftDistance,
  float &frontDistance,
  float &rightDistance
) {
  leftDistance =
    filteredDistance(trigLeft, echoLeft);

  delay(SENSOR_GAP_MS);

  frontDistance =
    filteredDistance(trigFront, echoFront);

  delay(SENSOR_GAP_MS);

  rightDistance =
    filteredDistance(trigRight, echoRight);
}

// =====================================================
// WALL FOLLOWING
// =====================================================

void followLeftWall(
  float leftDistance,
  float frontDistance
) {
  if (frontDistance < FRONT_BLOCKED_DISTANCE) {
    stopMotors();

    Serial.println("FRONT BLOCKED");
    return;
  }

  // If no left wall is detected, drive straight.
  if (leftDistance > WALL_THRESHOLD) {
    driveMotors(
      LEFT_BASE_SPEED,
      RIGHT_BASE_SPEED
    );

    return;
  }

  float error =
    leftDistance - WALL_SETPOINT;

  int correction =
    (int)(KP * error);

  correction =
    constrain(
      correction,
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

  /*
    Corrected arrangement.

    error > 0 means the robot is FARTHER than setpoint from the
    left wall (drifting away) -> right wheel must speed up /
    left wheel must slow down to steer back toward the wall.
  */

  int leftSpeed =
    LEFT_BASE_SPEED - correction;

  int rightSpeed =
    RIGHT_BASE_SPEED + correction;

  // Ease off speed as we approach a front wall, on top of the
  // wall-following correction above.
  if (frontDistance < FRONT_SLOWDOWN_DISTANCE) {
    leftSpeed -= FRONT_SLOWDOWN_REDUCTION;
    rightSpeed -= FRONT_SLOWDOWN_REDUCTION;
  }

  leftSpeed =
    constrain(leftSpeed, 0, MAX_SPEED);

  rightSpeed =
    constrain(rightSpeed, 0, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("L=");
  Serial.print(leftDistance, 1);

  Serial.print(" F=");
  Serial.print(frontDistance, 1);

  Serial.print(" Error=");
  Serial.print(error, 1);

  Serial.print(" PWM_L=");
  Serial.print(leftSpeed);

  Serial.print(" PWM_R=");
  Serial.println(rightSpeed);
}

// =====================================================
// TEST MODE 0: MOTOR TEST
// =====================================================

void runMotorTest() {
  static bool started = false;

  if (started) {
    return;
  }

  started = true;

  Serial.println("MOTOR TEST START");
  delay(1000);

  Serial.println("Left motor forward");
  setMotor(
    LEFT_IN1,
    LEFT_IN2,
    LEFT_ENA,
    TEST_LEFT_SPEED,
    LEFT_REVERSED
  );
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("Right motor forward");
  setMotor(
    RIGHT_IN1,
    RIGHT_IN2,
    RIGHT_ENA,
    TEST_RIGHT_SPEED,
    RIGHT_REVERSED
  );
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("Both motors forward");
  driveMotors(
    TEST_LEFT_SPEED,
    TEST_RIGHT_SPEED
  );
  delay(5000);
  stopMotors();
  delay(1000);

  Serial.println("Pivot left");
  pivotLeft(TURN_FAST_SPEED);
  delay(1000);
  stopMotors();
  delay(1000);

  Serial.println("Pivot right");
  pivotRight(TURN_FAST_SPEED);
  delay(1000);
  stopMotors();

  Serial.println("MOTOR TEST COMPLETE");
}

// =====================================================
// TEST MODE 1: SENSOR TEST
// =====================================================

void runSensorTest() {
  stopMotors();

  float leftDistance;
  float frontDistance;
  float rightDistance;

  readAllSensors(
    leftDistance,
    frontDistance,
    rightDistance
  );

  Serial.print("Left: ");
  Serial.print(leftDistance, 1);

  Serial.print(" cm | Front: ");
  Serial.print(frontDistance, 1);

  Serial.print(" cm | Right: ");
  Serial.print(rightDistance, 1);

  Serial.println(" cm");

  delay(300);
}

// =====================================================
// TEST MODE 2: STRAIGHT DRIVE
// =====================================================

void runStraightDriveTest() {
  static unsigned long startTime = 0;
  static bool started = false;

  if (!started) {
    started = true;
    startTime = millis();

    Serial.println("STRAIGHT DRIVE TEST");
    Serial.println("Robot should move forward for 10 seconds.");
  }

  driveMotors(
    TEST_LEFT_SPEED,
    TEST_RIGHT_SPEED
  );

  if (millis() - startTime >= 10000UL) {
    stopMotors();

    Serial.println("STRAIGHT DRIVE TEST COMPLETE");
    Serial.println("Robot is stopped.");

    while (true) {
      stopMotors();
      delay(1000);
    }
  }
}

// =====================================================
// TEST MODE 3: LEFT-WALL FOLLOWING
// =====================================================

void runLeftWallTest() {
  float leftDistance;
  float frontDistance;
  float rightDistance;

  readAllSensors(
    leftDistance,
    frontDistance,
    rightDistance
  );

  followLeftWall(
    leftDistance,
    frontDistance
  );
}

// =====================================================
// TEST MODE 4: TURN TEST
// =====================================================

void runTurnTest() {
  static bool completed = false;

  if (completed) {
    stopMotors();
    return;
  }

  completed = true;

  Serial.println("LEFT 90-DEGREE TURN");

  pivotLeft(TURN_FAST_SPEED);
  delay(LEFT_TURN_90_MS - 100);

  pivotLeft(TURN_SLOW_SPEED);
  delay(100);

  stopMotors();
  delay(1000);

  Serial.println("RIGHT 90-DEGREE TURN");

  pivotRight(TURN_FAST_SPEED);
  delay(RIGHT_TURN_90_MS - 100);

  pivotRight(TURN_SLOW_SPEED);
  delay(100);

  stopMotors();

  Serial.println("TURN TEST COMPLETE");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32 MAZE ROBOT TEST");
  Serial.println("==============================");
  Serial.print("TEST_MODE: ");
  Serial.println(TEST_MODE);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENA, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENA, OUTPUT);

  digitalWrite(trigLeft, LOW);
  digitalWrite(trigFront, LOW);
  digitalWrite(trigRight, LOW);

  stopMotors();

  Serial.println("Setup complete.");
  delay(2000);
}

// =====================================================
// LOOP
// =====================================================

void loop() {
#if TEST_MODE == 0

  runMotorTest();

#elif TEST_MODE == 1

  runSensorTest();

#elif TEST_MODE == 2

  runStraightDriveTest();

#elif TEST_MODE == 3

  runLeftWallTest();

#elif TEST_MODE == 4

  runTurnTest();

#else

  stopMotors();

  Serial.println("Invalid TEST_MODE.");
  Serial.println("Choose 0, 1, 2, 3, or 4.");

  while (true) {
    delay(1000);
  }

#endif
}
