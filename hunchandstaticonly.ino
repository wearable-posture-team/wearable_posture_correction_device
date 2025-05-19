#include <Wire.h>
#include "BluetoothSerial.h"

// MPU6050 addresses
const int MPU1_ADDRESS = 0x68;
const int MPU2_ADDRESS = 0x69;

// Variables for the first MPU6050
float RateRoll1, RatePitch1, RateYaw1;
float AccX1, AccY1, AccZ1;
float AngleRoll1, AnglePitch1;
float KalmanAngleRoll1 = 0, KalmanUncertAngleRoll1 = 2 * 2;
float KalmanAnglePitch1 = 0, KalmanUncertAnglePitch1 = 2 * 2;
float PrevKalmanAngleRoll1 = 0, PrevKalmanAnglePitch1 = 0;

// Variables for the second MPU6050
float RateRoll2, RatePitch2, RateYaw2;
float AccX2, AccY2, AccZ2;
float AngleRoll2, AnglePitch2;
float KalmanAngleRoll2 = 0, KalmanUncertAngleRoll2 = 2 * 2;
float KalmanAnglePitch2 = 0, KalmanUncertAnglePitch2 = 2 * 2;

uint32_t LoopTimer;
uint32_t LastCheckTime = 0;
const uint32_t CheckInterval = 10000; // 10 seconds interval in milliseconds

float RateCalRoll1, RateCalPitch1, RateCalYaw1;
float RateCalRoll2, RateCalPitch2, RateCalYaw2;
int RateCalNum;
float Kalman1DOutput[] = {0, 0};

// Calibration offsets for the accelerometers
float AccXOffset1 = 0.0, AccYOffset1 = 0.0, AccZOffset1 = 0.0;
float AccXOffset2 = 0.0, AccYOffset2 = 0.0, AccZOffset2 = 0.0;

// Vibration motor and buzzer pins
const int VibrationMotorPin = 33;
const int BuzzerPin = 32;

const float HUNCH_THRESHOLD = 65; // Set a threshold for hunch detection (in degrees)

BluetoothSerial SerialBT;
String detectionMode = "None";

// Function to initialize MPU6050
void initMPU6050(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake the sensor up
  Wire.endTransmission();
  delay(100);

  // Set accelerometer configuration (optional)
  Wire.beginTransmission(address);
  Wire.write(0x1C);  // Accelerometer configuration register
  Wire.write(0x10);  // Set full-scale range to ±8g
  Wire.endTransmission();
}

// Function to read raw accelerometer values from the MPU6050
void read_accelerometer_raw(int address, int16_t &AccXRaw, int16_t &AccYRaw, int16_t &AccZRaw) {
  Wire.beginTransmission(address);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission();
  Wire.requestFrom(address, 6);

  if (Wire.available() == 6) {
    AccXRaw = Wire.read() << 8 | Wire.read();
    AccYRaw = Wire.read() << 8 | Wire.read();
    AccZRaw = Wire.read() << 8 | Wire.read();
  }
}

// Function to calibrate accelerometer for a given MPU6050
void calibrate_accelerometer(int address, float &AccXOffset, float &AccYOffset, float &AccZOffset) {
  const int num_samples = 1000;
  long AccXSum = 0, AccYSum = 0, AccZSum = 0;
  int16_t AccXRaw, AccYRaw, AccZRaw;

  for (int i = 0; i < num_samples; i++) {
    read_accelerometer_raw(address, AccXRaw, AccYRaw, AccZRaw);
    AccXSum += AccXRaw;
    AccYSum += AccYRaw;
    AccZSum += AccZRaw;
    delay(3);  // Wait a little between readings
  }

  // Calculate the average offsets
  AccXOffset = (float)AccXSum / num_samples / 4096.0;
  AccYOffset = (float)AccYSum / num_samples / 4096.0;
  AccZOffset = (float)AccZSum / num_samples / 4096.0 - 1.0;  // Subtract 1g for Z-axis
}

// Function to read gyro signals from the MPU6050
void gyro_signals(int address, float &RateRoll, float &RatePitch, float &RateYaw, float &AccX, float &AccY, float &AccZ, float &AngleRoll, float &AnglePitch, float AccXOffset, float AccYOffset, float AccZOffset) {
  int16_t AccXLSB, AccYLSB, AccZLSB;
  read_accelerometer_raw(address, AccXLSB, AccYLSB, AccZLSB);

  Wire.beginTransmission(address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096.0 - AccXOffset;
  AccY = (float)AccYLSB / 4096.0 - AccYOffset;
  AccZ = (float)AccZLSB / 4096.0 - AccZOffset;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.142);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.142);
}

// Function for Kalman filter
void kalman_1d(float &KalmanState, float &KalmanUncert, float KalmanInput, float KalmanMeas) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncert = KalmanUncert + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncert / (KalmanUncert + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeas - KalmanState);
  KalmanUncert = (1 - KalmanGain) * KalmanUncert;
}

void setup() {
  Serial.begin(57600);
  SerialBT.begin("ESP32Posture"); // Bluetooth device name
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(VibrationMotorPin, OUTPUT); // Set vibration motor pin as output
  pinMode(BuzzerPin, OUTPUT); // Set buzzer pin as output
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  initMPU6050(MPU1_ADDRESS);
  initMPU6050(MPU2_ADDRESS);

  calibrate_accelerometer(MPU1_ADDRESS, AccXOffset1, AccYOffset1, AccZOffset1);
  calibrate_accelerometer(MPU2_ADDRESS, AccXOffset2, AccYOffset2, AccZOffset2);

  for (RateCalNum = 0; RateCalNum < 2000; RateCalNum++) {
    gyro_signals(MPU1_ADDRESS, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1, AccXOffset1, AccYOffset1, AccZOffset1);
    RateCalRoll1 += RateRoll1;
    RateCalPitch1 += RatePitch1;
    RateCalYaw1 += RateYaw1;

    gyro_signals(MPU2_ADDRESS, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2, AccXOffset2, AccYOffset2, AccZOffset2);
    RateCalRoll2 += RateRoll2;
    RateCalPitch2 += RatePitch2;
    RateCalYaw2 += RateYaw2;
    delay(1);
  }
  RateCalRoll1 = RateCalRoll1 / 2000;
  RateCalPitch1 = RateCalPitch1 / 2000;
  RateCalYaw1 = RateCalYaw1 / 2000;
  RateCalRoll2 = RateCalRoll2 / 2000;
  RateCalPitch2 = RateCalPitch2 / 2000;
  RateCalYaw2 = RateCalYaw2 / 2000;
  LoopTimer = micros();
  LastCheckTime = millis(); // Initialize the last check time
}

void loop() {
  if (SerialBT.available()) {
    String incoming = SerialBT.readString();
    Serial.print("Received: ");
    Serial.println(incoming); // Debug message
    if (incoming == "Static") {
      detectionMode = "Static";
      Serial.println("Mode set to Static"); // Debug message
    } else if (incoming == "Hunch") {
      detectionMode = "Hunch";
      Serial.println("Mode set to Hunch"); // Debug message
    }
  }

  gyro_signals(MPU1_ADDRESS, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1, AccXOffset1, AccYOffset1, AccZOffset1);
  RateRoll1 -= RateCalRoll1;
  RatePitch1 -= RateCalPitch1;
  RateYaw1 -= RateCalYaw1;

  kalman_1d(KalmanAngleRoll1, KalmanUncertAngleRoll1, RateRoll1, AngleRoll1);
  kalman_1d(KalmanAnglePitch1, KalmanUncertAnglePitch1, RatePitch1, AnglePitch1);

  gyro_signals(MPU2_ADDRESS, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2, AccXOffset2, AccYOffset2, AccZOffset2);
  RateRoll2 -= RateCalRoll2;
  RatePitch2 -= RateCalPitch2;
  RateYaw2 -= RateCalYaw2;

  kalman_1d(KalmanAngleRoll2, KalmanUncertAngleRoll2, RateRoll2, AngleRoll2);
  kalman_1d(KalmanAnglePitch2, KalmanUncertAnglePitch2, RatePitch2, AnglePitch2);

  Serial.print("MPU1 Roll Angle= ");
  Serial.print(KalmanAngleRoll1);
  Serial.print(" Pitch Angle= ");
  Serial.print(KalmanAnglePitch1);
  Serial.print(" | MPU2 Roll Angle= ");
  Serial.print(KalmanAngleRoll2);
  Serial.print(" Pitch Angle= ");
  Serial.println(KalmanAnglePitch2);

  // Detect static or hunch based on detection mode
  if (detectionMode == "Static") {
    // Check if the difference is less than 5 degrees for 15 seconds
    if (abs(KalmanAngleRoll1 - PrevKalmanAngleRoll1) < 5 && abs(KalmanAnglePitch1 - PrevKalmanAnglePitch1) < 5) {
      if (millis() - LastCheckTime >= CheckInterval) {
        Serial.println("Triggering alert");
        digitalWrite(VibrationMotorPin, HIGH); // Turn on the vibration motor
        digitalWrite(BuzzerPin, HIGH); // Turn on the buzzer
        delay(250); // Vibrate and buzz for 2 seconds
        digitalWrite(VibrationMotorPin, LOW); // Turn off the vibration motor
        digitalWrite(BuzzerPin, LOW); // Turn off the buzzer

        PrevKalmanAngleRoll1 = KalmanAngleRoll1;
        PrevKalmanAnglePitch1 = KalmanAnglePitch1;
        LastCheckTime = millis(); // Reset the timer
      }
    } else {
      PrevKalmanAngleRoll1 = KalmanAngleRoll1;
      PrevKalmanAnglePitch1 = KalmanAnglePitch1;
      LastCheckTime = millis(); // Reset the timer
    }
  } else if (detectionMode == "Hunch") {
    // Check if the pitch angle of MPU1 is less than the threshold
    if (KalmanAnglePitch1 < HUNCH_THRESHOLD) {
      Serial.println("Triggering alert");
      digitalWrite(VibrationMotorPin, HIGH); // Turn on the vibration motor
      digitalWrite(BuzzerPin, HIGH); // Turn on the buzzer
      delay(100); // Vibrate and buzz for 2 seconds
      digitalWrite(VibrationMotorPin, LOW); // Turn off the vibration motor
      digitalWrite(BuzzerPin, LOW); // Turn off the buzzer
    }
  }

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}