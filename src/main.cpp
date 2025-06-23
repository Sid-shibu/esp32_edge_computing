#include <Wire.h> // Hardware I2C library
#include <ESP32Servo.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_MPU6050.h> // MPU6050 library
#include <Adafruit_Sensor.h> // Dependency for Adafruit_MPU6050
#include <math.h>

#ifndef PI
#define PI 3.14159265359
#endif

// --- HARDWARE CONFIGURATION ---
#define MPU_ADDR 0x68    // Default MPU6050 I2C address (AD0 low)
#define TCA_ADDR 0x70    // Default TCA9548A I2C address
#define EN_BUTTON 0      // GPIO pin for calibration/enable button
#define NUM_SENSORS 3    // Number of MPU6050 sensors connected via multiplexer

// Define servo pins for each sensor (adjust these GPIOs based on your wiring)
const uint8_t SERVO_PINS[NUM_SENSORS] = {25, 26, 27}; 

// --- TREMOR DETECTION PARAMETERS ---
#define SAMPLES 256
#define SAMPLING_FREQ 250
#define TREMOR_MAGNITUDE_THRESHOLD 50.0
#define TREMOR_CONSISTENCY_THRESHOLD 0.7
#define MIN_TREMOR_DURATION 2000
#define CALIBRATION_SAMPLES 100

// --- TREMOR CLASSIFICATION THRESHOLDS ---
#define REST_TREMOR_MIN 3.0
#define REST_TREMOR_MAX 6.0
#define ACTION_TREMOR_MIN 4.0
#define ACTION_TREMOR_MAX 12.0
#define PATHOLOGICAL_TREMOR_MIN 2.0
#define PATHOLOGICAL_TREMOR_MAX 25.0

// --- FILTERING PARAMETERS ---
#define BUTTERWORTH_CUTOFF 30.0
#define MEDIAN_FILTER_SIZE 5 // Added this global define

// --- OUTPUT PINS ---
#define STATUS_LED 2     // GPIO for status LED
#define BUZZER_PIN 4     // GPIO for buzzer

// === MEDIAN FILTER GLOBALS ===
double medianBuffer[NUM_SENSORS][MEDIAN_FILTER_SIZE] = {0};
int medianIndex[NUM_SENSORS] = {0};

// === SYSTEM STATE FLAGS ===
bool isCalibrated = false;
bool allSensorsOk = true;

// === WIFI CREDENTIALS ===
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// === WEB SERVER OBJECT ===
WebServer server(80);

// --- TCA9548A Multiplexer OBJECT ---
QWIICMUX tca; // No argument for constructor

// --- MPU6050 Sensor OBJECT (only one instance needed with multiplexer) ---
Adafruit_MPU6050 mpu; 

// --- SERVO OBJECT (single servo for all strings) ---
Servo servo;

// === STRUCTURES FOR TREMOR ANALYSIS ===
struct TremorData {
  double magnitude;
  double frequency;
  double amplitude;
  double consistency;
  double rmsValue;
  bool isTremor;
  String classification;
  unsigned long duration;
  double confidence;
};

struct SensorReading {
  float accelX, accelY, accelZ; // Changed to float as Adafruit MPU returns float
  float gyroX, gyroY, gyroZ;   // Changed to float
  double magnitude;
  unsigned long timestamp;
};

// === GLOBAL VARIABLES ===
arduinoFFT FFT; // Use correct class name and no template

// FFT Arrays
double* vReal = nullptr;
double* vImag = nullptr;

// Sensor data buffers
SensorReading* sensorBuffer[NUM_SENSORS];
TremorData tremorResults[NUM_SENSORS];
double baselineNoise[NUM_SENSORS] = {0};
unsigned long lastTremorTime[NUM_SENSORS] = {0};
unsigned long tremorStartTime[NUM_SENSORS] = {0};

// --- IMPROVEMENT 1: Butterworth Filter Coefficient Optimization ---
struct ButterworthCoeffs {
  double a0, a1, a2, b1, b2;
  bool initialized = false;
} butterCoeffs[NUM_SENSORS];

double filterHistory[NUM_SENSORS][4] = {0};

void initButterworth(uint8_t sensorIndex, double cutoff, double sampleRate) {
  double omega = 2.0 * PI * cutoff / sampleRate;
  double cosOmega = cos(omega);
  double sinOmega = sin(omega);
  double alpha = sinOmega / sqrt(2.0);
  double norm = 1.0 + alpha;
  butterCoeffs[sensorIndex].a0 = (1.0 - cosOmega) / 2.0 / norm;
  butterCoeffs[sensorIndex].a1 = (1.0 - cosOmega) / norm;
  butterCoeffs[sensorIndex].a2 = (1.0 - cosOmega) / 2.0 / norm;
  butterCoeffs[sensorIndex].b1 = -2.0 * cosOmega / norm;
  butterCoeffs[sensorIndex].b2 = (1.0 - alpha) / norm;
  butterCoeffs[sensorIndex].initialized = true;
}

double butterworthFilter(double input, uint8_t sensorIndex, double cutoff, double sampleRate) {
  if (!butterCoeffs[sensorIndex].initialized) {
    initButterworth(sensorIndex, cutoff, sampleRate);
  }
  double output = butterCoeffs[sensorIndex].a0 * input
                + butterCoeffs[sensorIndex].a1 * filterHistory[sensorIndex][0]
                + butterCoeffs[sensorIndex].a2 * filterHistory[sensorIndex][1]
                - butterCoeffs[sensorIndex].b1 * filterHistory[sensorIndex][2]
                - butterCoeffs[sensorIndex].b2 * filterHistory[sensorIndex][3];
  filterHistory[sensorIndex][1] = filterHistory[sensorIndex][0];
  filterHistory[sensorIndex][0] = input;
  filterHistory[sensorIndex][3] = filterHistory[sensorIndex][2];
  filterHistory[sensorIndex][2] = output;
  return output;
}

// === MEDIAN FILTER FOR SPIKE REMOVAL ===
double applyMedianFilter(double value, uint8_t sensorIndex) {
  medianBuffer[sensorIndex][medianIndex[sensorIndex]] = value;
  medianIndex[sensorIndex] = (medianIndex[sensorIndex] + 1) % MEDIAN_FILTER_SIZE;
  double sorted[MEDIAN_FILTER_SIZE];
  memcpy(sorted, medianBuffer[sensorIndex], sizeof(sorted));
  // --- IMPROVEMENT 2: Use insertion sort for median filter (faster for small arrays) ---
  for (int i = 1; i < MEDIAN_FILTER_SIZE; i++) {
    double key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[MEDIAN_FILTER_SIZE / 2];
}

// === MPU6050 INITIALIZATION (using hardware I2C with TCA9548A) ===
bool initMPU(uint8_t sensorChannel) {
  Serial.printf("Sensor %d: Activating TCA channel %d...\n", sensorChannel + 1, sensorChannel);
  tca.enablePort(sensorChannel); // Use correct SparkFun method

  Serial.printf("Sensor %d: Attempting to initialize MPU on TCA channel %d...\n", sensorChannel + 1, sensorChannel);
  
  // The mpu.begin() function will use the currently selected TCA channel
  if (!mpu.begin(MPU_ADDR)) { // Use MPU_ADDR (0x68) or 0x69 if AD0 is high
    Serial.printf("Sensor %d: Failed to find MPU6050 chip on channel %d. Check wiring and address.\n", sensorChannel + 1, sensorChannel);
    return false;
  }
  Serial.printf("Sensor %d: MPU6050 Found on channel %d!\n", sensorChannel + 1, sensorChannel);

  // Set MPU6050 ranges and filters
  // These settings are applied to the MPU currently active on the TCA channel
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.printf("Sensor %d: Accel range set to: ", sensorChannel + 1);
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G: Serial.println("+-2G"); break;
    case MPU6050_RANGE_4_G: Serial.println("+-4G"); break;
    case MPU6050_RANGE_8_G: Serial.println("+-8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.printf("Sensor %d: Gyro range set to: ", sensorChannel + 1);
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG: Serial.println("+-250 deg/s"); break;
    case MPU6050_RANGE_500_DEG: Serial.println("+-500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+-1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+-2000 deg/s"); break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // As per PDF suggestion for 5Hz, but 21Hz is a common setting.
                                            // The PDF mentions 5Hz bandwidth, but MPU6050 has discrete options.
                                            // MPU6050_BAND_5_HZ is also an option if stricter filtering desired.
  Serial.printf("Sensor %d: Filter bandwidth set to: ", sensorChannel + 1);
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ: Serial.println("94 Hz"); break;
    case MPU6050_BAND_44_HZ: Serial.println("44 Hz"); break;
    case MPU6050_BAND_21_HZ: Serial.println("21 Hz"); break;
    case MPU6050_BAND_10_HZ: Serial.println("10 Hz"); break;
    case MPU6050_BAND_5_HZ: Serial.println("5 Hz"); break;
  }

  delay(100); // Give sensor time to settle after configuration
  Serial.printf("✅ Sensor %d initialized successfully on TCA channel %d.\n", sensorChannel + 1, sensorChannel);
  return true;
}

// === READ SENSOR DATA (using hardware I2C with TCA9548A) ===
SensorReading readSensorData(uint8_t sensorChannel) {
  SensorReading reading;
  reading.timestamp = micros();
  tca.enablePort(sensorChannel); // Use correct SparkFun method
  
  sensors_event_t a, g, temp;
  // Get new sensor events from the MPU currently active on the TCA channel
  mpu.getEvent(&a, &g, &temp);

  reading.accelX = a.acceleration.x;
  reading.accelY = a.acceleration.y;
  reading.accelZ = a.acceleration.z;
  
  reading.gyroX = g.gyro.x;
  reading.gyroY = g.gyro.y;
  reading.gyroZ = g.gyro.z;
  
  // Calculate magnitude in m/s^2
  // Note: Adafruit library directly gives m/s^2, no need for 16384.0 scaling for raw values.
  // The magnitude calculation is for the vector magnitude of acceleration.
  reading.magnitude = sqrt(pow(reading.accelX, 2) + pow(reading.accelY, 2) + pow(reading.accelZ, 2));
  
  // Check for obviously bad readings (e.g., all zeros or very large numbers if disconnected)
  // A magnitude of 0 usually means a read failure.
  if (reading.magnitude == 0.0f && (reading.accelX == 0.0f && reading.accelY == 0.0f && reading.accelZ == 0.0f)) {
      // This might indicate a communication failure or disconnected sensor
      // Serial.printf("Sensor %d: Read data all zeros. Possible issue.\n", sensorChannel + 1);
      // We already handle this check in detectTremor's warning.
  }
  
  return reading;
}


// === TREMOR DETECTION ALGORITHM ===
// This function needs to process data from a specific sensor, previously handled by software I2C.
// Now it gets data via readSensorData which handles the TCA channel selection.
TremorData detectTremor(uint8_t sensorIndex) {
  TremorData result;
  
  unsigned long sampleInterval = 1000000 / SAMPLING_FREQ;
  
  // Collect samples
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long startTime = micros();
    
    SensorReading reading = readSensorData(sensorIndex); // This now uses hardware I2C via TCA

    // If readSensorData failed (magnitude is 0), warn and continue.
    // The filtering and FFT might get impacted, but we proceed for demonstration.
    if (reading.magnitude == 0 && allSensorsOk) {
      Serial.printf("Warning: Sensor %d returned zero magnitude during sampling. Check connection.\n", sensorIndex + 1);
    }

    double filteredMagnitude = applyMedianFilter(reading.magnitude, sensorIndex);
    double smoothedMagnitude = butterworthFilter(filteredMagnitude, sensorIndex, 
                                                 BUTTERWORTH_CUTOFF, SAMPLING_FREQ);
    
    vReal[i] = smoothedMagnitude; // Store raw smoothed magnitude for DC removal
    vImag[i] = 0.0;
    
    sensorBuffer[sensorIndex][i] = reading; // Store raw readings (optional, for debugging)
    
    // Maintain sampling rate
    while (micros() - startTime < sampleInterval) {
      delayMicroseconds(1);
    }
  }
  
  // Remove DC component (mean subtraction)
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] -= mean;
    // Apply Hann window
    vReal[i] *= 0.5 * (1.0 - cos(2.0 * PI * i / (SAMPLES - 1)));
  }
  // Perform FFT (use correct arduinoFFT API)
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  
  // Find dominant frequency
  double maxMagnitude = 0;
  int peakIndex = 0;
  double freqResolution = (double)SAMPLING_FREQ / SAMPLES;
  
  // Search within the pathological tremor frequency range
  int minBin = max(1, (int)(PATHOLOGICAL_TREMOR_MIN / freqResolution)); // Avoid DC component (bin 0)
  int maxBin = min(SAMPLES/2, (int)(PATHOLOGICAL_TREMOR_MAX / freqResolution)); // Max bin is N/2 (Nyquist)
  
  for (int i = minBin; i < maxBin; i++) {
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
      peakIndex = i;
    }
  }
  
  result.frequency = peakIndex * freqResolution;
  result.amplitude = maxMagnitude; // Amplitude of the dominant frequency component
  
  // Calculate RMS of the AC component (tremor signal)
  double rmsSumSq = 0;
  for (int i = 0; i < SAMPLES; i++) {
    rmsSumSq += vReal[i] * vReal[i]; // vReal here contains the windowed, DC-removed signal
  }
  result.rmsValue = sqrt(rmsSumSq / SAMPLES);
  
  // Calculate total magnitude (average absolute value of the AC component)
  result.magnitude = 0;
  for (int i = 0; i < SAMPLES; i++) {
    result.magnitude += abs(vReal[i]);
  }
  result.magnitude /= SAMPLES;
  
  // Calculate consistency (fundamental vs harmonics)
  double fundamentalMag = vReal[peakIndex];
  double harmonicMag = 0;
  if (peakIndex * 2 < SAMPLES/2) harmonicMag += vReal[peakIndex * 2];
  if (peakIndex * 3 < SAMPLES/2) harmonicMag += vReal[peakIndex * 3];
  
  result.consistency = (fundamentalMag + 1e-6) / (fundamentalMag + harmonicMag + 1e-6); // Avoid division by zero
  
  // Tremor detection criteria
  bool magnitudeCheck = result.rmsValue > TREMOR_MAGNITUDE_THRESHOLD;
  bool frequencyCheck = result.frequency >= PATHOLOGICAL_TREMOR_MIN && 
                        result.frequency <= PATHOLOGICAL_TREMOR_MAX;
  bool consistencyCheck = result.consistency > TREMOR_CONSISTENCY_THRESHOLD;
  
  // Amplitude check against baseline noise (noise floor of the FFT)
  bool amplitudeCheck = result.amplitude > baselineNoise[sensorIndex] * 3.0; // 3x noise floor
  
  result.isTremor = magnitudeCheck && frequencyCheck && consistencyCheck && amplitudeCheck;
  
  // Calculate confidence based on how many criteria are met
  result.confidence = 0.0;
  if (magnitudeCheck) result.confidence += 0.25;
  if (frequencyCheck) result.confidence += 0.25;
  if (consistencyCheck) result.confidence += 0.25;
  if (amplitudeCheck) result.confidence += 0.25;
  
  // Classify tremor type
  if (result.isTremor) {
    if (result.frequency >= REST_TREMOR_MIN && result.frequency <= REST_TREMOR_MAX) {
      result.classification = "Rest Tremor";
    } else if (result.frequency >= ACTION_TREMOR_MIN && result.frequency <= ACTION_TREMOR_MAX) {
      result.classification = "Action Tremor";
    } else if (result.frequency > ACTION_TREMOR_MAX) { // Beyond action tremor range, but still within pathological
      result.classification = "High-Frequency Tremor";
    } else { // Below action tremor range, but still within pathological
      result.classification = "Low-Frequency Tremor";
    }
    
    if (tremorStartTime[sensorIndex] == 0) {
      tremorStartTime[sensorIndex] = millis();
    }
    result.duration = millis() - tremorStartTime[sensorIndex];
    lastTremorTime[sensorIndex] = millis();
  } else {
    result.classification = "Normal";
    // Reset tremor duration if no tremor for a certain period
    if (millis() - lastTremorTime[sensorIndex] > MIN_TREMOR_DURATION) {
      tremorStartTime[sensorIndex] = 0;
      result.duration = 0;
    }
  }
  
  return result;
}

// === SERVO CONTROL WITH TREMOR COMPENSATION ===
void controlTremorFeedback(uint8_t sensorIndex, TremorData tremor) {
  // Only activate compensation if tremor is detected with high confidence
  if (!tremor.isTremor || tremor.confidence < 0.5) {
    servo.write(90); // Neutral position
    return;
  }
  
  // Map tremor magnitude to an intensity (0-100)
  double intensity = map(tremor.rmsValue, 0, TREMOR_MAGNITUDE_THRESHOLD * 2, 0, 100); // Scale based on RMS
  intensity = constrain(intensity, 0, 100);
  
  int baseAngle = 90;
  // Amplitude of servo movement based on intensity
  int amplitude = map(intensity, 0, 100, 0, 45); // Max 45 degrees swing (90 +/- 45 = 45 to 135)
  
  // Calculate phase for a continuous sinusoidal motion based on tremor frequency
  // millis() gives milliseconds, tremor.frequency is Hz.
  // (millis() / 1000.0) converts millis to seconds
  double phase = (millis() / 1000.0) * tremor.frequency * 2 * PI;
  
  // Apply a 180° phase shift (PI radians) for counter-phase compensation
  int angle = baseAngle + amplitude * sin(phase + PI); 
  
  angle = constrain(angle, 45, 135); // Ensure angle stays within safe limits for typical servos
  servo.write(angle);
}

// === SERVO CONTROL WITH TREMOR COMPENSATION (SINGLE SERVO) ===
void controlTensionServo() {
  // Find the maximum tremor RMS value and confidence among all sensors
  double maxRMS = 0;
  double maxConfidence = 0;
  double maxFrequency = 0;
  bool tremorDetected = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tremorResults[i].isTremor && tremorResults[i].confidence > maxConfidence) {
      maxConfidence = tremorResults[i].confidence;
      maxRMS = tremorResults[i].rmsValue;
      maxFrequency = tremorResults[i].frequency;
      tremorDetected = true;
    }
  }
  if (!tremorDetected || maxConfidence < 0.5) {
    servo.write(90); // Neutral position
    return;
  }
  // Map tremor RMS to intensity (0-100)
  double intensity = map(maxRMS, 0, TREMOR_MAGNITUDE_THRESHOLD * 2, 0, 100);
  intensity = constrain(intensity, 0, 100);
  int baseAngle = 90;
  int amplitude = map(intensity, 0, 100, 0, 45); // Max 45 degrees swing
  double phase = (millis() / 1000.0) * maxFrequency * 2 * PI;
  int angle = baseAngle + amplitude * sin(phase + PI);
  angle = constrain(angle, 45, 135);
  servo.write(angle);
}

// === CALIBRATION PROCEDURE ===
void calibrateBaseline() {
  Serial.println("🔧 Starting baseline calibration...");
  digitalWrite(STATUS_LED, HIGH);
  
  // Re-initialize allSensorsOk to true before starting calibration for a clean check
  allSensorsOk = true; 

  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    Serial.printf("Calibrating sensor %d...\n", sensor + 1);
    
    double totalMagnitude = 0;
    int validReadings = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      SensorReading reading = readSensorData(sensor); // This reads from the specific sensor via TCA
      // Only count readings if magnitude is not zero (indicating a successful read)
      if (reading.magnitude > 0) {  
        // Apply median and Butterworth filter during calibration to get a clean baseline
        double filteredMagnitude = applyMedianFilter(reading.magnitude, sensor);
        double smoothedMagnitude = butterworthFilter(filteredMagnitude, sensor, 
                                                     BUTTERWORTH_CUTOFF, SAMPLING_FREQ);
        totalMagnitude += smoothedMagnitude;
        validReadings++;
      }
      delay(10); // Small delay between calibration samples
    }
    
    if (validReadings > 0) {
      baselineNoise[sensor] = totalMagnitude / validReadings;
      Serial.printf("Sensor %d baseline: %.2f (from %d readings)\n", 
                    sensor + 1, baselineNoise[sensor], validReadings);
    } else {
      Serial.printf("❌ Sensor %d: No valid readings during calibration! Setting default baseline.\n", sensor + 1);
      baselineNoise[sensor] = 10.0; // Default value if no valid readings obtained
      allSensorsOk = false; // Mark system as not fully okay if a sensor fails calibration
    }
  }
  
  isCalibrated = true;
  digitalWrite(STATUS_LED, LOW);
  Serial.println("✅ Calibration complete!");
  
  // Beep confirmation
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// --- IMPROVEMENT 3: Increase JSON buffer size for future expansion ---
void sendTremorDataJSON() {
  StaticJsonDocument<2048> doc;
  doc["timestamp"] = millis();
  doc["calibrated"] = isCalibrated;
  doc["all_sensors_ok"] = allSensorsOk;
  JsonArray sensors = doc.createNestedArray("sensors");
  for (int i = 0; i < NUM_SENSORS; i++) {
    JsonObject sensor = sensors.createNestedObject();
    sensor["id"] = i + 1;
    sensor["tremor_detected"] = tremorResults[i].isTremor;
    sensor["frequency"] = round(tremorResults[i].frequency * 100) / 100.0;
    sensor["amplitude"] = round(tremorResults[i].amplitude * 100) / 100.0;
    sensor["magnitude"] = round(tremorResults[i].magnitude * 100) / 100.0;
    sensor["rms"] = round(tremorResults[i].rmsValue * 100) / 100.0;
    sensor["consistency"] = round(tremorResults[i].consistency * 1000) / 1000.0;
    sensor["confidence"] = round(tremorResults[i].confidence * 1000) / 1000.0;
    sensor["classification"] = tremorResults[i].classification;
    sensor["duration"] = tremorResults[i].duration;
    sensor["baseline"] = round(baselineNoise[i] * 100) / 100.0;
  }
  bool overallTremor = false;
  double avgFrequency = 0;
  int tremorCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tremorResults[i].isTremor && tremorResults[i].confidence > 0.6) {
      overallTremor = true;
      avgFrequency += tremorResults[i].frequency;
      tremorCount++;
    }
  }
  if (tremorCount > 0) avgFrequency /= tremorCount;
  doc["overall_tremor"] = overallTremor;
  doc["average_frequency"] = round(avgFrequency * 100) / 100.0;
  doc["tremor_severity"] = "None";
  if (tremorCount > 2) doc["tremor_severity"] = "High";
  else if (tremorCount > 1) doc["tremor_severity"] = "Medium";
  else if (tremorCount > 0) doc["tremor_severity"] = "Low";
  String output;
  serializeJson(doc, output);
  Serial.println(output);
}

// --- IMPROVEMENT 4: Add /status endpoint for health checks ---
void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["status"] = "ok";
  doc["calibrated"] = isCalibrated;
  doc["all_sensors_ok"] = allSensorsOk;
  doc["wifi"] = (WiFi.status() == WL_CONNECTED);
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// === WEB SERVER HANDLERS ===
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html><head><title>Tremor Monitor</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
      body {font-family:Arial, sans-serif;margin:20px;background:#f0f0f0;color:#333;}
      h1 {color:#0056b3;}
      .card {
        background:white;
        padding:20px;
        margin-bottom:15px; 
        border-radius:10px;
        box-shadow:0 2px 10px rgba(0,0,0,0.1);
        transition: all 0.3s ease-in-out; 
      }
      .card h3 {margin-top:0; color:#0056b3;}
      .tremor {background:#ffebee;border-left:5px solid #f44336;}
      .normal {background:#e8f5e8;border-left:5px solid #4caf50;}
      .error {background:#fff3e0;border-left:5px solid #ff9800;}
      .button {
        display: inline-block;
        background-color: #007bff;
        color: white;
        padding: 10px 20px;
        border: none;
        border-radius: 5px;
        text-decoration: none;
        margin-top: 10px;
        cursor: pointer;
      }
      .button:hover {background-color: #0056b3;}
    </style>
    </head><body>
    <h1>Tremor Detection Monitor</h1>
    <div id='data'></div>
    <div class="card">
      <h3>Actions</h3>
      <button class="button" onclick="location.reload()">Refresh Data</button>
      <p>Press the EN button on the ESP32 to re-calibrate.</p>
    </div>

    <script>
    setInterval(()=>{
      fetch('/data').then(r=>r.json()).then(d=>{
        let html = '<div class="card"><h3>System Status</h3>';
        html += '<p>Calibrated: ' + (d.calibrated ? 'OK Yes' : 'ERROR No') + '</p>'; // Replaced ✅ with OK, ❌ with ERROR
        html += '<p>All Sensors OK: ' + (d.all_sensors_ok ? 'OK Yes' : 'ERROR No - Check Connections!') + '</p>'; // Replaced ✅ with OK, ❌ with ERROR
        html += '<p>Overall Tremor: ' + (d.overall_tremor ? 'Warning DETECTED' : 'Normal') + '</p>'; // Replaced ⚠ with Warning, ✅ with nothing
        html += '<p>Severity: <strong>' + d.tremor_severity + '</strong></p>';
        if (d.overall_tremor) {
          html += '<p>Average Freq: ' + d.average_frequency + ' Hz</p>';
        }
        html += '</div>';
        
        d.sensors.forEach(s=>{
          let statusClass = 'normal';
          if (s.baseline === 0 || !d.all_sensors_ok) statusClass = 'error'; // Error if baseline is 0 or overall sensors are not ok
          else if (s.tremor_detected) statusClass = 'tremor';
          
          html += '<div class="card ' + statusClass + '">';
          html += '<h3>Finger ' + s.id + ' Sensor</h3>';
          
          if (s.baseline === 0 || !d.all_sensors_ok) { 
            html += '<p>Warning: Sensor Error or Uncalibrated - Check Connection/Calibration</p>'; // Replaced ⚠
          } else {
            html += '<p>Status: <strong>' + (s.tremor_detected ? 'Warning: Tremor Detected' : 'Normal') + '</strong></p>'; // Replaced ⚠ and ✅
            html += '<p>Classification: ' + s.classification + '</p>';
            html += '<p>Frequency: ' + s.frequency + ' Hz</p>';
            html += '<p>RMS Magnitude: ' + s.rms + '</p>';
            html += '<p>Amplitude (FFT Peak): ' + s.amplitude + '</p>';
            html += '<p>Consistency: ' + (s.consistency*100).toFixed(1) + '%</p>';
            html += '<p>Confidence: ' + (s.confidence*100).toFixed(1) + '%</p>';
            html += '<p>Duration: ' + (s.duration / 1000).toFixed(1) + ' s</p>';
            html += '<p>Baseline Noise: ' + s.baseline + '</p>';
          }
          html += '</div>';
        });
        document.getElementById('data').innerHTML = html;
      }).catch(e=>{
        console.error("Error fetching data:", e);
        document.getElementById('data').innerHTML = '<div class="card error"><h3>Connection Error</h3><p>Unable to fetch data from ESP32. Ensure WiFi is connected and ESP32 is running.</p></div>';
      });
    }, 1000); // Update every 1 second
    </script></body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  StaticJsonDocument<1024> doc; 
  doc["timestamp"] = millis();
  doc["calibrated"] = isCalibrated;
  doc["all_sensors_ok"] = allSensorsOk;
  
  JsonArray sensors = doc.createNestedArray("sensors");
  for (int i = 0; i < NUM_SENSORS; i++) {
    JsonObject sensor = sensors.createNestedObject();
    sensor["id"] = i + 1;
    sensor["tremor_detected"] = tremorResults[i].isTremor;
    sensor["frequency"] = tremorResults[i].frequency;
    sensor["amplitude"] = tremorResults[i].amplitude;
    sensor["magnitude"] = tremorResults[i].magnitude;
    sensor["rms"] = tremorResults[i].rmsValue;
    sensor["consistency"] = tremorResults[i].consistency;
    sensor["confidence"] = tremorResults[i].confidence;
    sensor["classification"] = tremorResults[i].classification;
    sensor["duration"] = tremorResults[i].duration;
    sensor["baseline"] = baselineNoise[i];
  }
  
  // Overall tremor analysis
  bool overallTremor = false;
  double avgFrequency = 0;
  int tremorCount = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tremorResults[i].isTremor && tremorResults[i].confidence > 0.6) {
      overallTremor = true;
      avgFrequency += tremorResults[i].frequency;
      tremorCount++;
    }
  }
  
  if (tremorCount > 0) avgFrequency /= tremorCount;
  
  doc["overall_tremor"] = overallTremor;
  doc["average_frequency"] = avgFrequency;
  doc["tremor_severity"] = "None";
  if (tremorCount > 2) doc["tremor_severity"] = "High";
  else if (tremorCount > 1) doc["tremor_severity"] = "Medium";
  else if (tremorCount > 0) doc["tremor_severity"] = "Low";
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// === MAIN SETUP ===
void setup() {
  Serial.begin(115200);
  Serial.println("🚀 Starting ESP32 Tremor Detection System");
  
  // Allocate memory for FFT arrays
  vReal = (double*)malloc(SAMPLES * sizeof(double));
  vImag = (double*)malloc(SAMPLES * sizeof(double));
  
  // Check memory allocation for FFT arrays
  if (!vReal || !vImag) {
    Serial.println("❌ Failed to allocate FFT memory! Halting.");
    while(true); // Halt the program
  }
  
  // Allocate memory for sensor buffers
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorBuffer[i] = (SensorReading*)malloc(SAMPLES * sizeof(SensorReading));
    if (!sensorBuffer[i]) {
      Serial.printf("❌ Failed to allocate sensor buffer memory for sensor %d! Halting.\n", i + 1);
      while(true); // Halt the program
    }
  }
  
  // Setup pins
  pinMode(EN_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Setup ESP32 servo timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // --- Initialize Hardware I2C and TCA9548A ---
  Wire.begin(); // Initialize default hardware I2C pins (GPIO 21 SDA, GPIO 22 SCL)
  Serial.println("Hardware I2C (Wire) initialized.");

  if (!tca.begin()) {
    Serial.println("❌ Failed to find TCA9548A multiplexer! Check wiring.");
    while(true); // Halt if multiplexer not found
  }
  Serial.println("TCA9548A Multiplexer found and initialized.");
  
  // --- Initialize MPU6050 sensors via TCA9548A ---
  Serial.println("🔧 Initializing MPU6050 sensors through TCA9548A...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!initMPU(i)) { // Pass the channel index (0 to NUM_SENSORS-1)
      Serial.printf("❌ Failed to initialize MPU6050 on TCA channel %d! This sensor might be problematic.\n", i);
      allSensorsOk = false; // Set overall status to false if any sensor fails
    } else {
      Serial.printf("✅ MPU6050 on TCA channel %d initialized successfully.\n", i);
    }
  }
  // Attach the single servo to the first pin (adjust as needed)
  servo.attach(SERVO_PINS[0]);
  servo.write(90); // Center position for servo
  
  // Start WiFi connection
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) { 
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n❌ Failed to connect to WiFi!");
  }
  
  // Start web server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/status", handleStatus); // New endpoint
  server.begin();
  Serial.println("Web server started.");
  
  // Wait for calibration (or force if button not pressed)
  Serial.println("Press the EN button to calibrate (or wait 5 seconds for auto-calibration if button not pressed)...");
  unsigned long calibration_wait_start = millis();
  while (digitalRead(EN_BUTTON) == HIGH && (millis() - calibration_wait_start < 5000)) {
    delay(100);
  }
  calibrateBaseline();
}

// === MAIN LOOP ===
void loop() {
  // Check for recalibration request
  if (digitalRead(EN_BUTTON) == LOW) {
    delay(50); // Debounce
    if (digitalRead(EN_BUTTON) == LOW) {
      calibrateBaseline();
      // Wait for button release
      while (digitalRead(EN_BUTTON) == LOW) delay(10);
    }
  }
  // Perform tremor detection on all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    tremorResults[i] = detectTremor(i); // Pass channel index
  }
  // Control the single servo based on combined tremor
  controlTensionServo();
  // Send data output to serial and web server
  sendTremorDataJSON();
  // Handle web server requests
  server.handleClient();
  // Visual feedback (status LED)
  bool anyTremor = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tremorResults[i].isTremor && tremorResults[i].confidence > 0.6) {
      anyTremor = true;
      break;
    }
  }
  if (anyTremor) {
    digitalWrite(STATUS_LED, (millis() / 250) % 2); // Blink LED faster for active tremor
    if (allSensorsOk) { // Only beep if all sensors are operational
      digitalWrite(BUZZER_PIN, HIGH); // Constant tone during tremor
    }
  } else {
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
  delay(50); // Small delay to prevent overwhelming output and allow other tasks
}