#include <Arduino.h>

// N20 Motor Configuration
const uint8_t MOTOR_IN1_PIN = 4;  // Motor driver IN1 (PWM-capable)
const uint8_t MOTOR_IN2_PIN = 5;  // Motor driver IN2 (PWM-capable)
const uint8_t ENCODER_A_PIN = 6;  // Encoder channel A
const uint8_t ENCODER_B_PIN = 7;  // Encoder channel B

// N20 Motor specs
const float MOTOR_RPM_MAX = 200.0;
const float MOTOR_REDUCTION_RATIO = 150.0;
const float MOTOR_BASE_PPR = 7.0;
const float MOTOR_PPR = MOTOR_BASE_PPR * MOTOR_REDUCTION_RATIO; // 7 * 150 = 1050 PPR

// PWM Configuration
const uint8_t PWM_CHANNEL = 0;
const uint32_t PWM_FREQ = 20000; // 20 kHz
const uint8_t PWM_BITS = 10;     // 10-bit resolution (0-1023)

// track which pin currently has PWM attached (-1 = none)
static int current_pwm_pin = -1;

static void detachCurrentPWM() {
  if (current_pwm_pin < 0) return;
  ledcDetachPin(current_pwm_pin);
  current_pwm_pin = -1;
}

static void setHardBrake() {
  // both HIGH = hard brake
  detachCurrentPWM();
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

static void setSoftBrake() {
  // both LOW = soft brake (coast/passive)
  detachCurrentPWM();
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

// Encoder variables
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500; // Print every 500ms

// Encoder ISR functions
void IRAM_ATTR encoderAISR() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  encoderCount += (a != b) ? 1 : -1;
}

void IRAM_ATTR encoderBISR() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  encoderCount += (a == b) ? 1 : -1;
}

void setMotorSpeed(int pwm_value) {
  // pwm_value: -((1<<PWM_BITS)-1) .. +((1<<PWM_BITS)-1)
  // Positive = forward, Negative = reverse
  // Special: pwm_value == 0 => hard brake (both HIGH)

  if (pwm_value == 0) {
    setHardBrake();
    return;
  }

  int pwm_magnitude = abs(pwm_value);
  const uint32_t max_duty = (1u << PWM_BITS) - 1;
  pwm_magnitude = constrain(pwm_magnitude, 0, (int)max_duty);

  // Invert duty (like "100 - PWM"): use max_duty - requested
  pwm_magnitude = (int)(max_duty - pwm_magnitude);

  // staticPin: driven HIGH (direction); pwmPin: receives PWM waveform
  uint8_t staticPin = (pwm_value > 0) ? MOTOR_IN1_PIN : MOTOR_IN2_PIN;
  uint8_t pwmPin    = (pwm_value > 0) ? MOTOR_IN2_PIN : MOTOR_IN1_PIN;

  if (current_pwm_pin != pwmPin) {
    detachCurrentPWM();
    // attach PWM channel to selected pin
    ledcAttachPin(pwmPin, PWM_CHANNEL);
    current_pwm_pin = pwmPin;
  }

  // Static pin is output HIGH (direction selection)
  pinMode(staticPin, OUTPUT);
  digitalWrite(staticPin, HIGH);

  // Write duty
  ledcWrite(PWM_CHANNEL, pwm_magnitude);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("N20 Motor Encoder Test");

  // Setup encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderBISR, CHANGE);

  // Setup motor driver pins as GPIO initially
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  // Setup PWM (channel configured once)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);
  // do not attach yet; setHardBrake will detach and set pins

  // Stop motor initially with hard brake
  setHardBrake();

  Serial.println("Motor Configuration:");
  Serial.print("  Max RPM: "); Serial.println(MOTOR_RPM_MAX);
  Serial.print("  Reduction ratio: "); Serial.println(MOTOR_REDUCTION_RATIO);
  Serial.print("  Base PPR: "); Serial.println(MOTOR_BASE_PPR);
  Serial.print("  Effective PPR: "); Serial.println(MOTOR_PPR);
  Serial.println("....................................");
}

void loop() {
  unsigned long currentTime = millis();

  // state machine: 0 = forward(5s), 1 = stop(1s), 2 = backward(5s), 3 = stop(1s)
  static unsigned long lastChange = 0;
  static uint8_t state = 0;
  const unsigned long durations[] = {5000, 1000, 5000, 1000}; // ms
  const int dirSigns[] = { 1, 0, -1, 0 };
  const int pwmVal = 100; // magnitude 0-1023

  if (lastChange == 0) {
    lastChange = currentTime;
    setMotorSpeed(dirSigns[state] * pwmVal);
    Serial.println("Starting: forward");
  }

  if (currentTime - lastChange >= durations[state]) {
    state = (state + 1) % 4;
    lastChange = currentTime;

    int out = dirSigns[state] * pwmVal;
    setMotorSpeed(out);

    if (out > 0) {
      Serial.println("Forward for 5s");
    } else if (out < 0) {
      Serial.println("Backward for 5s");
    } else {
      Serial.println("Stop for 1s");
    }
  }

  // encoder periodic print (unchanged)
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;

    long currentCount = encoderCount;
    long deltaCount = currentCount - lastEncoderCount;
    lastEncoderCount = currentCount;

    // Calculate RPM
    float revolutions = deltaCount / (4.0 * MOTOR_PPR); // 4x counting (quadrature)
    float timeSeconds = PRINT_INTERVAL / 1000.0;
    float rpm = (revolutions / timeSeconds) * 60.0;

    // Calculate angle in degrees
    float angleDegrees = (currentCount / (4.0 * MOTOR_PPR)) * 360.0;

    Serial.print("Encoder: ");
    Serial.print(currentCount);
    Serial.print(" | Delta: ");
    Serial.print(deltaCount);
    Serial.print(" | RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Angle: ");
    Serial.print(angleDegrees, 2);
    Serial.println("°");
  }

  delay(10);
}
