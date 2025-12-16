//Server Code
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include <deque>
#include <numeric>

// --- Custom UUID Definitions ---
#define SERVICE_UUID "3091094d-6d61-44b3-9ed3-f84515e7e623"

// Define unique UUIDs for each characteristic
#define CHAR_UUID_CURRENT "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c0"
#define CHAR_UUID_10SEC_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c1"
#define CHAR_UUID_1MIN_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c2"
#define CHAR_UUID_5MIN_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c3"
#define CHAR_UUID_10MIN_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c4"
#define CHAR_UUID_15MIN_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c5"
#define CHAR_UUID_30MIN_AVG "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c6"
#define CHAR_UUID_ALERT_STATE "7c3fd1f0-7343-4d83-b341-2a95bc4ce3c7"

// --- Alert State Codes (Higher number = Higher Priority) ---
#define ALERT_NONE 0
#define ALERT_30_MIN 1
#define ALERT_15_MIN 2
#define ALERT_10_MIN 3
#define ALERT_5_MIN 4
#define ALERT_1_MIN 5
#define ALERT_10_SEC 6

// Sensor Pin (Assuming GPIO 1 based on original loop call)
const int UV_SENSOR_PIN = 1;

// --- ALERTING MECHANISM DEFINITIONS (MODIFIED) ---
const int ALERT_LED_PIN = LED_BUILTIN;
const int ALERT_VIBRATOR_PIN = 2;  // *** NEW: Pin for the Vibrator Motor (e.g., GPIO 2) ***

bool alert_on = false;
int current_alert_code = ALERT_NONE;

unsigned long lastBlinkTime = 0;
const long blinkInterval = 200;
// *** NEW: Vibrator Pulse Timing (a quick buzz) ***
const long vibrateDuration = 100;  // ms to keep the motor ON

// Define the buffer sizes for each time window (assuming a 1-second delay/sample rate)
const int SAMPLES_10_SEC = 10;
const int SAMPLES_1_MIN = 60;
const int SAMPLES_5_MIN = 300;
const int SAMPLES_10_MIN = 600;
const int SAMPLES_15_MIN = 900;
const int SAMPLES_30_MIN = 1800;  // 30 minutes

// Define the **ALERT** thresholds for each time window
const float THRESHOLD_10_SEC = 12.0f;
const float THRESHOLD_1_MIN = 10.0f;
const float THRESHOLD_5_MIN = 7.0f;
const float THRESHOLD_10_MIN = 5.0f;
const float THRESHOLD_15_MIN = 3.0f;
const float THRESHOLD_30_MIN = 1.0f;

// Circular buffer to store samples
std::deque<float> sampleBuffer;

// BLE Characteristic Pointers
BLECharacteristic *pCharCurrent;
BLECharacteristic *pChar10SecAvg;
BLECharacteristic *pChar1MinAvg;
BLECharacteristic *pChar5MinAvg;
BLECharacteristic *pChar10MinAvg;
BLECharacteristic *pChar15MinAvg;
BLECharacteristic *pChar30MinAvg;
BLECharacteristic *pCharAlertState;

// Function to add a new sample and maintain buffer size
void addSample(float newSample) {
  sampleBuffer.push_back(newSample);

  // Keep the buffer size to the maximum needed (30 minutes)
  if (sampleBuffer.size() > SAMPLES_30_MIN) {
    sampleBuffer.pop_front();
  }
}

// Function to calculate the average for a given number of samples
float calculateAverage(int numSamples) {
  if (sampleBuffer.empty() || numSamples <= 0) {
    return 0.0f;
  }

  // Only calculate and return the average if the buffer contains
  // the full number of required samples for this time window.
  if (sampleBuffer.size() < numSamples) {
    // Return 0.0f or any desired default value until the window is full
    return 0.0f;
  }

  // Since we've confirmed the buffer is full, we average exactly `numSamples`
  float sum = 0.0f;
  // Iterate over the last 'numSamples' elements
  for (int i = 0; i < numSamples; ++i) {
    sum += sampleBuffer[sampleBuffer.size() - numSamples + i];
  }
  return sum / numSamples;
}

// Utility function to convert float to a printable/transmittable string (BLE handles bytes/strings well)
String floatToString(float val) {
  char buf[20];
  sprintf(buf, "%.2f", val);
  return String(buf);
}

// *** NEW: Function to execute a single short vibration pulse ***
void pulseVibrator() {
  // Note: Since we are in the main loop and don't want to use delay() here,
  // we'll rely on the main `handleBlinking` and `loop` flow to keep it simple.
  // For a simple pulse, we can momentarily block, as the main loop already has a 1000ms delay.
  // However, it's safer to integrate it into the non-blocking `handleBlinking`.
  // For now, let's stick to the non-blocking approach in `handleBlinking`
  // and just ensure the pin is set HIGH/LOW when the alert state changes.
}


// --- Alerting Function (MODIFIED) ---
void checkAndHandleAlerts(float avg_10_sec, float avg_1_min, float avg_5_min, float avg_10_min, float avg_15_min, float avg_30_min) {
  // Determine the highest priority alert code
  current_alert_code = ALERT_NONE;

  // Check from shortest (highest priority) to longest
  if (avg_10_sec > THRESHOLD_10_SEC) {
    current_alert_code = ALERT_10_SEC;
  } else if (avg_1_min > THRESHOLD_1_MIN) {
    current_alert_code = ALERT_1_MIN;
  } else if (avg_5_min > THRESHOLD_5_MIN) {
    current_alert_code = ALERT_5_MIN;
  } else if (avg_10_min > THRESHOLD_10_MIN) {
    current_alert_code = ALERT_10_MIN;
  } else if (avg_15_min > THRESHOLD_15_MIN) {
    current_alert_code = ALERT_15_MIN;
  } else if (avg_30_min > THRESHOLD_30_MIN) {
    current_alert_code = ALERT_30_MIN;
  }

  // --- Update Alert Characteristic ---
  uint8_t alert_value = (uint8_t)current_alert_code;
  pCharAlertState->setValue(&alert_value, 1);
  pCharAlertState->notify();

  if (current_alert_code != ALERT_NONE) {
    // --- Alert Triggered (Physical/Serial) ---
    if (!alert_on) {
      Serial.println("\n*** ALERT TRIGGERED: At least one average UV Index is above threshold! ***");
      // *** NEW: Turn on the vibrator when the alert FIRST triggers ***
      digitalWrite(ALERT_VIBRATOR_PIN, HIGH);
      // Reset the blink time to start the LED and VIBRATOR pattern
      lastBlinkTime = millis();
      alert_on = true;
    }

    // Print specific alert type
    Serial.print("Highest Priority Alert: ");
    switch (current_alert_code) {
      case ALERT_10_SEC: Serial.println("10-Second Avg (Code 6)."); break;
      case ALERT_1_MIN: Serial.println("1-Minute Avg (Code 5)."); break;
      case ALERT_5_MIN: Serial.println("5-Minute Avg (Code 4)."); break;
      case ALERT_10_MIN: Serial.println("10-Minute Avg (Code 3)."); break;
      case ALERT_15_MIN: Serial.println("15-Minute Avg (Code 2)."); break;
      case ALERT_30_MIN: Serial.println("30-Minute Avg (Code 1)."); break;
    }

  } else {
    // --- Alert Cleared ---
    if (alert_on) {
      Serial.println("\n*** ALERT CLEARED: All average UV Indices are now below their thresholds. ***");
      // *** MODIFIED: Turn off the LED and VIBRATOR when the alert clears ***
      digitalWrite(ALERT_LED_PIN, LOW);
      digitalWrite(ALERT_VIBRATOR_PIN, LOW);
      alert_on = false;
    }
  }
}

// Function to handle the LED and VIBRATOR pulsing logic (MODIFIED)
void handleBlinking() {
  if (alert_on) {
    digitalWrite(ALERT_LED_PIN, LOW);
    digitalWrite(ALERT_VIBRATOR_PIN, LOW);
    for (int i = 0; i <= current_alert_code; ++i) {
      digitalWrite(ALERT_LED_PIN, HIGH);
      digitalWrite(ALERT_VIBRATOR_PIN, HIGH);
      delay(100);
      digitalWrite(ALERT_LED_PIN, LOW);
      digitalWrite(ALERT_VIBRATOR_PIN, LOW);
      delay(100);
    }
    digitalWrite(ALERT_LED_PIN, LOW);
    digitalWrite(ALERT_VIBRATOR_PIN, LOW);
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE UV Tracker!");
  analogReadResolution(12);

  // --- ALERTING: Initialize LED and VIBRATOR Pins (MODIFIED) ---
  pinMode(ALERT_LED_PIN, OUTPUT);
  digitalWrite(ALERT_LED_PIN, LOW);

  // *** NEW: Initialize Vibrator Pin ***
  pinMode(ALERT_VIBRATOR_PIN, OUTPUT);
  digitalWrite(ALERT_VIBRATOR_PIN, LOW);

  BLEDevice::init("UV Tracker");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // --- Initialize all 7 Characteristics (No Change) ---
  pCharCurrent = pService->createCharacteristic(
    CHAR_UUID_CURRENT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharCurrent->addDescriptor(new BLE2902());

  pChar10SecAvg = pService->createCharacteristic(
    CHAR_UUID_10SEC_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar10SecAvg->addDescriptor(new BLE2902());

  pChar1MinAvg = pService->createCharacteristic(
    CHAR_UUID_1MIN_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar1MinAvg->addDescriptor(new BLE2902());

  pChar5MinAvg = pService->createCharacteristic(
    CHAR_UUID_5MIN_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar5MinAvg->addDescriptor(new BLE2902());

  pChar10MinAvg = pService->createCharacteristic(
    CHAR_UUID_10MIN_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar10MinAvg->addDescriptor(new BLE2902());

  pChar15MinAvg = pService->createCharacteristic(
    CHAR_UUID_15MIN_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar15MinAvg->addDescriptor(new BLE2902());

  pChar30MinAvg = pService->createCharacteristic(
    CHAR_UUID_30MIN_AVG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar30MinAvg->addDescriptor(new BLE2902());

  // New Alert State Characteristic
  pCharAlertState = pService->createCharacteristic(
    CHAR_UUID_ALERT_STATE,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharAlertState->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Service and Characteristics defined. Advertising started.");
}

float read_uv_index(int pin) {
  int sensor_millivolts = analogReadMilliVolts(pin);
  float voltage = sensor_millivolts / 1000.0;
  // This mapping (voltage * 10) is a simplification, tune based on your sensor's datasheet
  float uv_index = voltage * 10.0;
  return uv_index;
}

void loop() {
  // --- Data Acquisition & Calculation ---
  float current_uv_index = read_uv_index(UV_SENSOR_PIN);
  addSample(current_uv_index);

  // Calculate averages (will return 0.0f until buffer is full for that time window)
  float avg_10_sec = calculateAverage(SAMPLES_10_SEC);
  float avg_1_min = calculateAverage(SAMPLES_1_MIN);
  float avg_5_min = calculateAverage(SAMPLES_5_MIN);
  float avg_10_min = calculateAverage(SAMPLES_10_MIN);
  float avg_15_min = calculateAverage(SAMPLES_15_MIN);
  float avg_30_min = calculateAverage(SAMPLES_30_MIN);

  // Print to Serial for debugging
  Serial.println("--- New Sample Cycle ---");
  Serial.println("Current UVI: " + floatToString(current_uv_index));
  Serial.println("Avg 10 sec: " + floatToString(avg_10_sec));
  Serial.println("Avg 1 min: " + floatToString(avg_1_min));
  Serial.println("Avg 5 min: " + floatToString(avg_5_min));
  Serial.println("Avg 10 min: " + floatToString(avg_10_min));
  Serial.println("Avg 15 min: " + floatToString(avg_15_min));
  Serial.println("Avg 30 min: " + floatToString(avg_30_min));

  // --- ALERTING: Check and Trigger/Clear Alerts, Update Alert Characteristic ---
  checkAndHandleAlerts(avg_10_sec, avg_1_min, avg_5_min, avg_10_min, avg_15_min, avg_30_min);

  // --- Update BLE Average Characteristics and Notify Clients (No Change) ---
  pCharCurrent->setValue(floatToString(current_uv_index).c_str());
  pCharCurrent->notify();

  pChar10SecAvg->setValue(floatToString(avg_10_sec).c_str());
  pChar10SecAvg->notify();

  pChar1MinAvg->setValue(floatToString(avg_1_min).c_str());
  pChar1MinAvg->notify();

  pChar5MinAvg->setValue(floatToString(avg_5_min).c_str());
  pChar5MinAvg->notify();

  pChar10MinAvg->setValue(floatToString(avg_10_min).c_str());
  pChar10MinAvg->notify();

  pChar15MinAvg->setValue(floatToString(avg_15_min).c_str());
  pChar15MinAvg->notify();

  pChar30MinAvg->setValue(floatToString(avg_30_min).c_str());
  pChar30MinAvg->notify();

  // --- ALERTING: Handle Blinking/Pulsing ---
  handleBlinking();

  delay(1000);  // wake up after 1 second
}
