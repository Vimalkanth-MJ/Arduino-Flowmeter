
const int motorPin1 = A1;   // Motor control pin 1
const int motorPin2 = A2;   // Motor control pin 2
const unsigned long motorRunTime = 5000;  // Time to run the motor in milliseconds (5 seconds)

unsigned long motorStartTime = 0;  // Variable to store motor start time
bool motorRunning = false;  // Flag to indicate if motor is running


void openValve();
void closeValve();
void runMotor(int pin1, int pin2, unsigned long duration);
void stopMotor();
void updateMotor();


void openValve() {
  runMotor(motorPin1, motorPin2, motorRunTime);
}

void closeValve() {
  runMotor(motorPin2, motorPin1, motorRunTime);
}

void runMotor(int pin1, int pin2, unsigned long duration) {
  stopMotor();  // Stop the motor if it's currently running
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  motorStartTime = millis(); // Record motor start time
  motorRunning = true;
}

void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  motorRunning = false;
}

void updateMotor() {
  if (motorRunning && (millis() - motorStartTime >= motorRunTime)) {
    stopMotor();
  }
}
