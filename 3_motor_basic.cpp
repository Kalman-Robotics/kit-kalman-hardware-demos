#include <Arduino.h>

// N20 Motor Configuration
const uint8_t MOTOR_IN1_PIN = 4;  // Motor driver IN1 (PWM-capable)
const uint8_t MOTOR_IN2_PIN = 5;  // Motor driver IN2 (PWM-capable)

// N20 Motor specs
const float MOTOR_RPM_MAX = 200.0;
const float MOTOR_REDUCTION_RATIO = 150.0;

// PWM Configuration
const uint8_t PWM_CHANNEL = 0;
const uint32_t PWM_FREQ = 20000; // 20 kHz
const uint8_t PWM_BITS = 10;     // 10-bit resolution (0-1023)

// track which pin currently has PWM attached (-1 = none)
static int current_pwm_pin = -1;

static void detachCurrentPWM() {
  if (current_pwm_pin < 0) return;
  // detach PWM from the pin so it can be used as a GPIO
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
  // both LOW = soft brake (allows motor to coast / passive braking)
  detachCurrentPWM();
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void setMotorSpeed(int pwm_value) {
  // pwm_value: -1023 to +1023
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

  // Determine pins: staticPin will be driven HIGH, pwmPin will receive PWM.
  uint8_t staticPin = (pwm_value > 0) ? MOTOR_IN1_PIN : MOTOR_IN2_PIN;
  uint8_t pwmPin    = (pwm_value > 0) ? MOTOR_IN2_PIN : MOTOR_IN1_PIN;

  // If PWM is currently attached to a different pin, detach it first
  if (current_pwm_pin != pwmPin) {
    detachCurrentPWM();
    // Attach PWM to the selected pin
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
  Serial.println("....................................");
  Serial.println("N20 Motor Test");

  // Setup motor driver pins as GPIO outputs initially
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  // Setup PWM (channel configured once)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);

  // Stop motor initially with hard brake
  setHardBrake();

  Serial.println("Motor Configuration:");
  Serial.print("  Max RPM: "); Serial.println(MOTOR_RPM_MAX);
  Serial.print("  Reduction ratio: "); Serial.println(MOTOR_REDUCTION_RATIO);
  Serial.println("....................................");
}

void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastChange = 0;
  static uint8_t state = 0; // 0 = forward, 1 = stop, 2 = backward, 3 = stop
  const unsigned long durations[] = {5000, 1000, 5000, 1000}; // ms
  const int dirSigns[] = { 1, 0, -1, 0 };
  const int pwmVal = 250; // magnitude 0-1023

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

  delay(10);
}
