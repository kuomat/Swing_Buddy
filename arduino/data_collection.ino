#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// const int MPU_ADDR = 0x68;   // MPU6050 I2C address
const int buttonPin = 2;     // Button pin
const int sdCardPin = 3;     // SD card chip select pin
int fileCounter = 1;         // Start at file1.txt
int lastButtonState = HIGH;  // For button debounce
bool isRecording = false;    // Recording state
Adafruit_MPU6050 mpu;

// Variables to store accelerometer and gyroscope data
// float AccX, AccY, AccZ;
// float GyroX, GyroY, GyroZ;
// float accAngleX, accAngleY, gyroAngleX = 0, gyroAngleY = 0, yaw = 0;
// float roll, pitch;

// Timing variables
// float elapsedTime, currentTime, previousTime = 0;

// Calibration offsets
// float AccErrorX = 0.58, AccErrorY = -1.58;
// float GyroErrorX = -0.56, GyroErrorY = 2, GyroErrorZ = -0.8;

File dataFile;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  // initializeMPU6050();

  // Initialize the SD card
  if (!SD.begin(sdCardPin)) {
    Serial.println("SD Card failed or not present");
    while (1);
  }
  Serial.println("SD Card initialized successfully!");

  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  // Check if button was pressed (active low) and debounce
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50); // Debounce delay
    if (digitalRead(buttonPin) == LOW) {
      if (!isRecording) {
        startRecording();
      } else {
        stopRecording();
      }
    }
  }

  lastButtonState = buttonState;  // Update button state

  // Record IMU data if recording is active
  if (isRecording) {
    readIMUData();
    // computeAngles();
    // writeDataToFile();
    // writeDataToSerial();
    delay(50); // Adjust sample rate as needed
  }
}

// // Function to read IMU data
// void readIMUData() {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   // Print the 6 DOF data
//   Serial.print("Acceleration X: ");
//   Serial.print(a.acceleration.x);
//   Serial.print(", Y: ");
//   Serial.print(a.acceleration.y);
//   Serial.print(", Z: ");
//   Serial.print(a.acceleration.z);
//   Serial.println(" m/s^2");

//   Serial.print("Rotation X: ");
//   Serial.print(g.gyro.x);
//   Serial.print(", Y: ");
//   Serial.print(g.gyro.y);
//   Serial.print(", Z: ");
//   Serial.print(g.gyro.z);
//   Serial.println(" rad/s");

//   Serial.print("Temperature: ");
//   Serial.print(temp.temperature);
//   Serial.println(" degC");

//   Serial.println("");
// }
void readIMUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check if the file is open before writing
  if (dataFile) {
    // Write acceleration data
    dataFile.print("Acceleration X: ");
    dataFile.print(a.acceleration.x);
    dataFile.print(", Y: ");
    dataFile.print(a.acceleration.y);
    dataFile.print(", Z: ");
    dataFile.print(a.acceleration.z);
    dataFile.println(" m/s^2");

    // Write gyroscope data
    dataFile.print("Rotation X: ");
    dataFile.print(g.gyro.x);
    dataFile.print(", Y: ");
    dataFile.print(g.gyro.y);
    dataFile.print(", Z: ");
    dataFile.print(g.gyro.z);
    dataFile.println(" rad/s");

    // Add a blank line to separate data points
    dataFile.println();

    // Ensure data is written to the file immediately
    dataFile.flush();
  }
}

void startRecording() {
  String fileName = "/imu_data" + String(fileCounter) + ".txt";
  dataFile = SD.open(fileName.c_str(), FILE_WRITE);

  if (dataFile) {
    Serial.println("Recording started. Saving to file: " + fileName);
    fileCounter++;
    isRecording = true;
  } else {
    Serial.println("Error: Could not open file for writing.");
  }
}

void stopRecording() {
  if (dataFile) {
    dataFile.close();
    Serial.println("Recording stopped. File saved.");
  }
  isRecording = false;
}
