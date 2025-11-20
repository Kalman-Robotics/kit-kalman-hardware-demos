#include <Arduino.h>
#include <motor_ctl_kaia.h>

// Motor pins
#define MOTOR_IN1_PIN 4  // Motor control
#define MOTOR_IN2_PIN 5  // Motor control
#define ENCODER_A_PIN 6  // Encoder A
#define ENCODER_B_PIN 7  // Encoder B

// PWM settings
#define MOT_PWM_CHANNEL 0
#define MOT_PWM_FREQ 20000  // 20kHz
#define MOT_PWM_BITS 10

MotorController motor;
float targetPWM = 0.0f;

void setMotorPWM(MotorController *motor_controller, float pwm) {
  int max_pwm = (1 << MOT_PWM_BITS) - 1;
  pwm = constrain(pwm, -1.0f, 1.0f);
  int pwm_magnitude = round(max_pwm * (1 - abs(pwm)));

  if (pwm == 0) {
    // Hard brake
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    return;
  }

  uint8_t in1 = pwm > 0 ? MOTOR_IN1_PIN : MOTOR_IN2_PIN;
  uint8_t in2 = pwm > 0 ? MOTOR_IN2_PIN : MOTOR_IN1_PIN;

  #if ESP_IDF_VERSION_MAJOR >= 5
  ledcAttachChannel(in2, MOT_PWM_FREQ, MOT_PWM_BITS, MOT_PWM_CHANNEL);
  #else
  ledcAttachPin(in2, MOT_PWM_CHANNEL);
  #endif

  ledcWrite(MOT_PWM_CHANNEL, pwm_magnitude);
  pinMode(in1, OUTPUT);
  digitalWrite(in1, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== N20 Motor PWM Test ===");
  Serial.println("Commands:");
  Serial.println("  w/s: increase/decrease PWM");
  Serial.println("  a: reverse direction");
  Serial.println("  space: stop");
  Serial.println("  i: print info");

  // Setup motor pins
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  #if ESP_IDF_VERSION_MAJOR >= 5
  ledcAttachChannel(MOTOR_IN2_PIN, MOT_PWM_FREQ, MOT_PWM_BITS, MOT_PWM_CHANNEL);
  #else
  ledcSetup(MOT_PWM_CHANNEL, MOT_PWM_FREQ, MOT_PWM_BITS);
  #endif

  motor.setPWMCallback(setMotorPWM);

  setMotorPWM(&motor, 0);
  Serial.println("Ready!");
}

void loop() {
  // Directly set PWM
  setMotorPWM(&motor, targetPWM);

  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'w':  // Increase PWM
        targetPWM += 0.1f;
        targetPWM = constrain(targetPWM, -1.0f, 1.0f);
        Serial.print("Target PWM: ");
        Serial.println(targetPWM, 2);
        break;
        
      case 's':  // Decrease PWM
        targetPWM -= 0.1f;
        targetPWM = constrain(targetPWM, -1.0f, 1.0f);
        Serial.print("Target PWM: ");
        Serial.println(targetPWM, 2);
        break;
        
      case 'a':  // Reverse direction
        targetPWM = -targetPWM;
        Serial.print("Reversed PWM: ");
        Serial.println(targetPWM, 2);
        break;
        
      case ' ':  // Stop
        targetPWM = 0.001f;
        Serial.println("Stopped");
        break;
        
      case 'i':  // Print info
        Serial.println("\n--- Motor Info ---");
        Serial.print("Current PWM: ");
        Serial.println(targetPWM, 2);
        Serial.println("------------------\n");
        break;
    }
  }

  delay(10);
}