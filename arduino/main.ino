#include <TensorFlowLite_ESP32.h>
#include <Wire.h>
#include <MPU6050.h>
#include "model_data.h"

MPU6050 mpu;
const int buttonPin = 2;
bool buttonPressed = false;

// Data arrays and variables
const int MAX_DATA_POINTS = 500;
float accX[MAX_DATA_POINTS], accY[MAX_DATA_POINTS], accZ[MAX_DATA_POINTS], accMagnitude[MAX_DATA_POINTS];
unsigned long timestamps[MAX_DATA_POINTS];
int dataCount = 0;

// Results
float maxAcceleration = 0;
float meanAcceleration = 0;
float stdAcceleration = 0;
float xDominance = 0, yDominance = 0, zDominance = 0;
int numPeaks = 0;

// TFLite Variables
tflite::MicroInterpreter* interpreter = nullptr;
tflite::ErrorReporter* error_reporter = nullptr;
tflite::MicroErrorReporter micro_error_reporter;
tflite::MicroAllocator* allocator = nullptr;
uint8_t tensor_arena[16 * 1024];
tflite::Model* model = nullptr;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonPress, FALLING);

  // Initialize LED pins
  pinMode(D10, OUTPUT);
  pinMode(D9, OUTPUT);
  turnOffLED();

  // Initialize TensorFlow Lite Micro
  error_reporter = &micro_error_reporter;
  model = tflite::GetModel(model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch.");
    while (1);
  }
  allocator = tflite::MicroAllocator::Create(tensor_arena, sizeof(tensor_arena), error_reporter);
  if (!allocator) {
    Serial.println("Failed to create allocator.");
    while (1);
  }
  interpreter = new tflite::MicroInterpreter(model, allocator, error_reporter);
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Tensor allocation failed.");
    while (1);
  }

  Serial.println("Ready to start motion tracking...");
}


void loop() {
  // Collect data if button is pressed
  if (buttonPressed) {
    collectData();
    calculateFeatures();
    // printResults();
    int prediction = predictClass();
    handlePrediction(prediction);
    buttonPressed = false;
  }

}

void turnOffLED() {
  digitalWrite(D10, LOW);
  digitalWrite(D9, LOW);
}

void onButtonPress() {
  buttonPressed = true;
}

void collectData() {
  Serial.println("Collecting data...");
  dataCount = 0;
  unsigned long startTime = millis();
  digitalWrite(D10, LOW);
  digitalWrite(D9, LOW);


  while (millis() - startTime < 5000) { // Collect data for 5 seconds
    if (dataCount < MAX_DATA_POINTS) {
      timestamps[dataCount] = millis();
      accX[dataCount] = mpu.getAccelerationX();
      accY[dataCount] = mpu.getAccelerationY();
      accZ[dataCount] = mpu.getAccelerationZ();
      accMagnitude[dataCount] = sqrt(accX[dataCount] * accX[dataCount] +
                                     accY[dataCount] * accY[dataCount] +
                                     accZ[dataCount] * accZ[dataCount]);
      dataCount++;
    }
  }
}

int predictClass() {
  // Prepare input tensor
  float* input = interpreter->input(0)->data.f;
  for (int i = 0; i < MAX_DATA_POINTS; i++) {
    input[i] = accMagnitude[i];
  }

  // Run inference
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Inference failed.");
    return -1;
  }

  // Read output
  TfLiteTensor* output = interpreter->output(0);
  float maxValue = output->data.f[0];
  int maxIndex = 0;
  for (int i = 1; i < 3; i++) { // Assume 3 classes
    if (output->data.f[i] > maxValue) {
      maxValue = output->data.f[i];
      maxIndex = i;
    }
  }

  Serial.print("Prediction: "); Serial.println(maxIndex);
  return maxIndex;
}

void calculateFeatures() {
  // Initialize variables
  maxAcceleration = 0;
  meanAcceleration = 0;
  stdAcceleration = 0;
  // totalEnergy = 0;
  numPeaks = 0;
  xDominance = 0;
  yDominance = 0;
  zDominance = 0;

  // Calculate features
  for (int i = 0; i < dataCount; i++) {
    float acc = accMagnitude[i];
    meanAcceleration += acc;

    if (acc > maxAcceleration) {
      maxAcceleration = acc;
    }

    // Count peaks (simple threshold-based approach)
    if (i > 0 && i < dataCount - 1 && acc > accMagnitude[i - 1] && acc > accMagnitude[i + 1]) {
      numPeaks++;
    }

    // Dominance ratios
    xDominance += abs(accX[i]);
    yDominance += abs(accY[i]);
    zDominance += abs(accZ[i]);
  }

  meanAcceleration /= dataCount;
  xDominance /= (meanAcceleration * dataCount);
  yDominance /= (meanAcceleration * dataCount);
  zDominance /= (meanAcceleration * dataCount);

  // Calculate standard deviation
  for (int i = 0; i < dataCount; i++) {
    stdAcceleration += pow(accMagnitude[i] - meanAcceleration, 2);
  }
  stdAcceleration = sqrt(stdAcceleration / dataCount);
}

void handlePrediction(int prediction) {
  turnOffLED();
  if (prediction == 0) {
    // Red for slow
    Serial.println("Lighting red");
    digitalWrite(D10, HIGH);
  } else if (prediction == 1) {
    // Green for fast
    Serial.println("Lighting green");
    digitalWrite(D9, HIGH);
  } else if (prediction == 2) {
    // Both on for just right
    Serial.println("Lighting both");
    digitalWrite(D10, HIGH);
    digitalWrite(D9, HIGH);
  }
  delay(2000);
}


//// For testing if the lights work or not
// void printResults() {
//   Serial.println("\n--- Results ---");
//   Serial.print("Max Acceleration: "); Serial.println(maxAcceleration);
//   Serial.print("Mean Acceleration: "); Serial.println(meanAcceleration);
//   Serial.print("Standard Deviation of Acceleration: "); Serial.println(stdAcceleration);
//   Serial.print("X Dominance: "); Serial.println(xDominance);
//   Serial.print("Y Dominance: "); Serial.println(yDominance);
//   Serial.print("Z Dominance: "); Serial.println(zDominance);
//   Serial.print("Number of Peaks: "); Serial.println(numPeaks);

//   turnOffLED();

//   Serial.println("Lighting red");
//   digitalWrite(D10, HIGH);
//   delay(2000);