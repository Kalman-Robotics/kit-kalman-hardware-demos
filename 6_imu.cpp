#include <Arduino.h>
#include "IMU6500.h"

// IMU Configuration
const int IMU_SDA_PIN = 48;
const int IMU_SCL_PIN = 47;
const uint32_t IMU_I2C_FREQ = 400000; // 400 kHz

// Create IMU instance
IMU6500 imu;

// Conversion factors
const float ACCEL_SCALE = 16384.0;  // LSB/g for ±2g range (default)
const float GYRO_SCALE = 131.0;     // LSB/(°/s) for ±250°/s range (default)

// Print interval
const unsigned long PRINT_INTERVAL = 500; // ms
unsigned long lastPrintTime = 0;
unsigned long lastGyroUpdateTime = 0;

// Euler angles (in degrees)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Complementary filter coefficient (0.0 to 1.0)
// Higher value = trust gyro more, lower value = trust accel more
const float ALPHA = 0.96;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("====================================");
  Serial.println("MPU6500 IMU - Euler Angles Demo");
  Serial.println("====================================");
  
  // Initialize IMU
  Serial.print("Initializing IMU on SDA:");
  Serial.print(IMU_SDA_PIN);
  Serial.print(", SCL:");
  Serial.print(IMU_SCL_PIN);
  Serial.print(", Freq:");
  Serial.print(IMU_I2C_FREQ);
  Serial.println(" Hz");
  
  if (!imu.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ)) {
    Serial.println("❌ Failed to initialize IMU!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✅ IMU initialized successfully!");
  Serial.println("====================================");
  Serial.println("Calculating Euler angles...");
  Serial.println();
  
  delay(1000);
  lastGyroUpdateTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensor data
  if (imu.read()) {
    // Get raw data
    int16_t ax, ay, az, gx, gy, gz;
    imu.getData(ax, ay, az, gx, gy, gz);
    
    // Convert to physical units
    float accel_x_g = ax / ACCEL_SCALE;
    float accel_y_g = ay / ACCEL_SCALE;
    float accel_z_g = az / ACCEL_SCALE;
    
    float gyro_x_dps = gx / GYRO_SCALE;
    float gyro_y_dps = gy / GYRO_SCALE;
    float gyro_z_dps = gz / GYRO_SCALE;
    
    // Calculate time delta for gyroscope integration
    float dt = (currentTime - lastGyroUpdateTime) / 1000.0; // seconds
    lastGyroUpdateTime = currentTime;
    
    // ========================================
    // STEP 1: Calculate angles from accelerometer
    // ========================================
    // Roll: rotation around X-axis
    float accel_roll = atan2(accel_y_g, accel_z_g) * 180.0 / PI;
    
    // Pitch: rotation around Y-axis
    float accel_pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180.0 / PI;
    
    // Note: Yaw cannot be determined from accelerometer alone (needs magnetometer)
    
    // ========================================
    // STEP 2: Integrate gyroscope rates
    // ========================================
    float gyro_roll_delta = gyro_x_dps * dt;
    float gyro_pitch_delta = gyro_y_dps * dt;
    float gyro_yaw_delta = gyro_z_dps * dt;
    
    // ========================================
    // STEP 3: Complementary filter (fuse accel + gyro)
    // ========================================
    // Roll and Pitch: combine accelerometer (slow, stable) with gyroscope (fast, drifts)
    roll = ALPHA * (roll + gyro_roll_delta) + (1.0 - ALPHA) * accel_roll;
    pitch = ALPHA * (pitch + gyro_pitch_delta) + (1.0 - ALPHA) * accel_pitch;
    
    // Yaw: only from gyroscope (will drift over time without magnetometer)
    yaw += gyro_yaw_delta;
    
    // Normalize yaw to -180 to +180 degrees
    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
    
    // ========================================
    // STEP 4: Print results at specified interval (without ASCII box)
    // ========================================
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      lastPrintTime = currentTime;
      
      // Raw sensor data
      Serial.println("Accelerometer (g):");
      Serial.print("  X: ");
      Serial.print(accel_x_g, 3);
      Serial.print("  Y: ");
      Serial.print(accel_y_g, 3);
      Serial.print("  Z: ");
      Serial.print(accel_z_g, 3);
      Serial.println();
      
      Serial.println("Gyroscope (°/s):");
      Serial.print("  X: ");
      Serial.print(gyro_x_dps, 2);
      Serial.print("  Y: ");
      Serial.print(gyro_y_dps, 2);
      Serial.print("  Z: ");
      Serial.print(gyro_z_dps, 2);
      Serial.println();
      
      // Euler angles
      Serial.println("Euler Angles (degrees):");
      Serial.print("  Roll:  ");
      Serial.print(roll, 2);
      Serial.println("°");
      Serial.print("  Pitch: ");
      Serial.print(pitch, 2);
      Serial.println("°");
      Serial.print("  Yaw:   ");
      Serial.print(yaw, 2);
      Serial.println("°");
      Serial.println();
    }
  } else {
    Serial.println("❌ Failed to read IMU data!");
  }
  
  delay(10);
}
