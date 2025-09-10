
// **Source Code for Virtual Prototype on Wokwi**

// Include necessary libraries
// Adafruit MPU6050 for accelerometer/gyroscope
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// SoftwareSerial for communication with the GPS module
#include <SoftwareSerial.h>
// TinyGPS++ for parsing GPS data
#include <TinyGPS++.h>

// Wire for I2C communication (used by the MPU6050)
#include <Wire.h>

// Adafruit NeoPixel for the status LED
#include <Adafruit_NeoPixel.h>

// === HARDWARE SETUP ===
// MPU6050 sensor object
Adafruit_MPU6050 mpu;

// GPS module setup
// Rx pin 2, Tx pin 3
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// NeoPixel status LED setup
#define LED_PIN 6
Adafruit_NeoPixel pixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// === CONFIGURATION VARIABLES ===
// A simple threshold for detecting a "crash"
// 5G is a good starting point for a severe impact
const float CRASH_THRESHOLD_G = 5.0;

// Duration the G-force must be above the threshold to trigger an alert (in milliseconds)
const int CRASH_DURATION_MS = 100;

// Variables to track crash timing
unsigned long crashStartTime = 0;

// Variables to hold sensor data
float accelX, accelY, accelZ;

// System states for the status LED and logic flow
enum SystemState { IDLE, CRASH_DETECTED, ALERT_SENT };
SystemState currentState = IDLE;

// Flag to prevent sending multiple alerts for a single crash
bool alertSent = false;

// === SETUP FUNCTION: Runs once when the program starts ===
void setup() {
  Serial.begin(9600); // Start serial communication for debugging
  ss.begin(GPSBaud); // Start serial communication with the GPS module

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // Stay in this loop if the sensor isn't found
    while (1) {
      delay(10);
    }
  }

  // Configure the MPU6050's range for better sensitivity to a crash
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Initialize the NeoPixel LED
  pixel.begin();
  setLEDColor(0, 255, 0); // Set initial color to green (IDLE)
  
  Serial.println("System Initialized. Awaiting a crash...");
}

// === MAIN LOOP: Runs continuously ===
void loop() {
  readSensors();
  updateStatusLED();

  // === CRASH DETECTION ALGORITHM ===
  // Calculate the magnitude (total force) of the acceleration vector
  // This is a more reliable way to detect impact from any direction
  float accelerationMagnitude = sqrt(
      accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Check if the acceleration exceeds the crash threshold
  if (accelerationMagnitude > CRASH_THRESHOLD_G) {
    if (crashStartTime == 0) {
      // If this is the first time the threshold is exceeded, start the timer
      crashStartTime = millis();
    }
    // Check if the high G-force has been sustained for the required duration
    if ((millis() - crashStartTime) > CRASH_DURATION_MS && !alertSent) {
      triggerAlert();
    }
  } else {
    // If G-force drops below the threshold, reset the timer
    crashStartTime = 0;
  }
}

// === HELPER FUNCTIONS ===

// Reads data from the MPU6050 sensor
void readSensors() {
  sensors_event_t a; // Object to store acceleration data
  mpu.getEvent(&a); // Get the latest data from the sensor

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  // Print data to the serial monitor for debugging and live demo
  Serial.print("Accel X: ");
  Serial.print(accelX);
  Serial.print(" G | Y: ");
  Serial.print(accelY);
  Serial.print(" G | Z: ");
  Serial.println(accelZ);
}

// Function to handle the alert sequence after a crash is detected
void triggerAlert() {
  currentState = CRASH_DETECTED;

  // Simulate a text message alert
  Serial.println("\n----------------------------------");
  Serial.println("ALERT! CRASH DETECTED!");
  Serial.println("Sending SMS alert to emergency services...");

  // Get and print the GPS location
  getGPSLocation();

  // This is a placeholder for the real GSM communication
  // In a real device, you would use AT commands to send an SMS
  Serial.println("\nSMS Sent Successfully!");
  Serial.println("----------------------------------\n");

  alertSent = true;
  currentState = ALERT_SENT;
}

// Function to get GPS coordinates from the module
void getGPSLocation() {
  // This loop tries to get a valid GPS fix
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        Serial.print("Location: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
        return;
      }
    }
  }
  
  // This is the fallback for the virtual demo
  // It prints a hardcoded location if a live fix isn't available
  Serial.println("Simulating GPS location...");
  Serial.print("Latitude: 11.016844, Longitude: 76.955833 (Coimbatore, India)");
  Serial.println();
}

// Function to set the color of the NeoPixel LED
void setLEDColor(int r, int g, int b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// Function to update the LED color based on the current system state
void updateStatusLED() {
  switch (currentState) {
    case IDLE:
      setLEDColor(0, 255, 0); // Green for idle
      break;
    case CRASH_DETECTED:
      setLEDColor(255, 165, 0); // Orange for crash detected
      break;
    case ALERT_SENT:
      setLEDColor(255, 0, 0); // Red for alert sent
      break;
  }
}
