#include <Arduino.h>

// Encoder Configuration
const uint8_t ENCODER_A_PIN = 6;  // Encoder channel A
const uint8_t ENCODER_B_PIN = 7;  // Encoder channel B

// N20 Motor specs
const float MOTOR_REDUCTION_RATIO = 150.0;
const float MOTOR_BASE_PPR = 7.0;
const float MOTOR_PPR = MOTOR_BASE_PPR * MOTOR_REDUCTION_RATIO; // 7 * 150 = 1050 PPR

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

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("....................................");
  Serial.println("Encoder Test");

  // Setup encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderBISR, CHANGE);

  Serial.println("Encoder Configuration:");
  Serial.print("  Reduction ratio: "); Serial.println(MOTOR_REDUCTION_RATIO);
  Serial.print("  Base PPR: "); Serial.println(MOTOR_BASE_PPR);
  Serial.print("  Effective PPR: "); Serial.println(MOTOR_PPR);
  Serial.println("....................................");
}

void loop() {
  unsigned long currentTime = millis();

  // Encoder periodic print
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