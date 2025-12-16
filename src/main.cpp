#include <Arduino.h>
#include <WiFi.h>
#include <OSCMessage.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

void connect_tufts_wireless();
void get_network_info();
void sendOSC();
void calibrateGyro();
void sensor_readings();

volatile float pitch_rad;
volatile float roll_rad;
volatile float yaw_rad;

volatile float pitch_deg = 0.0f;
volatile float roll_deg = 0.0f;
volatile float yaw_deg = 0.0f;

// Gyro bias (deg/s) estimated at startup and updated slowly when stationary
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

const float TIMESTEP = 0.5f; // 50 ms
const float alpha = 0.98f;    // Complementary filter coefficient

// state counter (button-driven mode index)
float state_counter = 0;

// Button debouncing state
float lastButtonReading = HIGH; // last raw reading from pin
float lastButtonStableState = HIGH; // debounced stable state
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelayMs = 50; // ms


//PIN DECLARATIONS FOR PCB (IO)

const int sda_pin = 3;
const int scl_pin = 2;

const int ir_led = 1;

const int stat_button_pin = 4; // Pin for status button
const int led_pin1 = 5;         // Pin for status LED
const int led_pin2 = 6;         // Pin for status LED
const int led_pin3 = 7;         // Pin for status LED
const int led_pin4 = 8;         // Pin for status LED

float pot_value = 0.0;          // Variable to store potentiometer value

const char* ssid = "Tufts_Wireless";       // e.g., "Tufts_IoT" or "Tufts_Wireless"
const char* password = "";   // leave "" if it's an open network
unsigned int localPort = 7773; // local port to listen for OSC packets
unsigned int sendPort = 7771; // Port to send to Max 

WiFiUDP Udp;
const IPAddress outIp(10,243,86,184); // IP to Send to

Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println();

  pinMode(ir_led, OUTPUT);
  digitalWrite(ir_led, HIGH); // Turn on IR LED

  pinMode(stat_button_pin, INPUT_PULLUP); // Status button
  pinMode(led_pin1, OUTPUT);         // Status LED
  pinMode(led_pin2, OUTPUT);         // Status LED
  pinMode(led_pin3, OUTPUT);         // Status LED
  pinMode(led_pin4, OUTPUT);         // Status LED
  pinMode(0, INPUT);               // Potentiometer pin
  /* Initialize I2C on pins 21 (SDA) and 22 (SCL) */
  Wire.begin(sda_pin, scl_pin);

  Wire.setClock(400000); // 400 kHz fast mode
  
  connect_tufts_wireless();

  Udp.begin(localPort);

  // Try to initialize MPU6050
  // Scan I2C bus to find devices
  Serial.println("\nScanning I2C bus for devices...");
  int deviceCount = 0;
  for (int addr = 1; addr < 127; addr++) {
    Serial.println(addr);
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      Serial.println(addr, HEX);
      deviceCount++;
    }
  }
  Serial.print("Total devices found: ");
  Serial.println(deviceCount);
  // Serial.println();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
    // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // // Calibrate gyro biases while stationary (do this after MPU init)
  calibrateGyro();

  sensor_readings();
  sendOSC();
  delay(1000);

}

void loop() {
  pot_value = analogRead(0); // Read potentiometer value
  sendOSC();

  // Debounce the status button (active LOW)
  int reading = digitalRead(stat_button_pin);
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelayMs) {
    // If the button state has been stable for longer than debounce delay
    if (reading != lastButtonStableState) {
      lastButtonStableState = reading;
      // Detect falling edge: HIGH -> LOW (button pressed)
      if (lastButtonStableState == LOW) {
        state_counter++;
      }
    }
  }
  lastButtonReading = reading;

  digitalWrite(led_pin1, LOW);
  digitalWrite(led_pin2, LOW);
  digitalWrite(led_pin3, LOW);
  digitalWrite(led_pin4, LOW);
  if (state_counter > 3){
    state_counter = 0;
  }
  if (state_counter == 0) {
    digitalWrite(led_pin1, HIGH);
  }else if (state_counter == 1) {
    digitalWrite(led_pin2, HIGH);
  }
  else if (state_counter == 2) {
    digitalWrite(led_pin3, HIGH);
  }
  else if (state_counter == 3) {
    digitalWrite(led_pin4, HIGH);
  }
}

void sendOSC() {
  OSCMessage msg("/datafromesp");

  // Do not attempt to send if Wi-Fi is disconnected
  if (WiFi.status() != WL_CONNECTED) return;
  // /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate dt
  static unsigned long last = micros();
  unsigned long now = micros();
  float dt = (now - last) / 1e6f;
  last = now;

  // Clamp dt to reasonable bounds to avoid huge integration jumps
  if (!isfinite(dt) || dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  // Convert accelerometer (m/s^2 → g)
  float ax = a.acceleration.x / 9.8f;
  float ay = a.acceleration.y / 9.8f;
  float az = a.acceleration.z / 9.8f;

  // Compute accelerometer angles
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  float accel_roll  = atan2f(ay, az);

  // Gyro is already rad/s from Adafruit → convert to deg/s
  float raw_gx = g.gyro.x * 180.0f / M_PI;
  float raw_gy = g.gyro.y * 180.0f / M_PI;
  float raw_gz = g.gyro.z * 180.0f / M_PI;

  // Subtract calibrated biases to get corrected rates
  float gx = raw_gx - gyroBiasX;
  float gy = raw_gy - gyroBiasY;
  float gz = raw_gz - gyroBiasZ;

  // Integrate gyro (deg)
  pitch_deg += gy * dt;
  roll_deg  += gx * dt;
  yaw_deg   += gz * dt;

  float gyro_pitch = pitch_deg * M_PI / 180.0f;
  float gyro_roll  = roll_deg  * M_PI / 180.0f;
  float gyro_yaw   = yaw_deg   * M_PI / 180.0f;

  // Complementary filter
  pitch_rad = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
  roll_rad  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
  yaw_rad   = gyro_yaw;

  pitch_deg = pitch_rad * 180.0f / M_PI;
  roll_deg  = roll_rad  * 180.0f / M_PI;
  yaw_deg   = yaw_rad   * 180.0f / M_PI;

  
  msg.add(state_counter);
  msg.add(roll_deg);
  msg.add(yaw_deg);
  msg.add(pot_value);

  // Online bias adaptation: if device appears stationary (raw angular rates near bias),
  // slowly update bias estimates to compensate drift/temperature changes.
  const float stationaryThreshold = 1.0f; // deg/s
  const float biasAlpha = 0.001f; // slow update rate
  if (fabsf(raw_gx - gyroBiasX) < stationaryThreshold && fabsf(raw_gy - gyroBiasY) < stationaryThreshold && fabsf(raw_gz - gyroBiasZ) < stationaryThreshold) {
    // move bias slightly toward measured raw value
    gyroBiasX = (1.0f - biasAlpha) * gyroBiasX + biasAlpha * raw_gx;
    gyroBiasY = (1.0f - biasAlpha) * gyroBiasY + biasAlpha * raw_gy;
    gyroBiasZ = (1.0f - biasAlpha) * gyroBiasZ + biasAlpha * raw_gz;
  }

  Udp.beginPacket(outIp, sendPort);
  msg.send(Udp); // Send the bytes to the SLIP stream
  Udp.endPacket();  // Mark the end of the OSC Packet
  msg.empty();   // Free space occupied by message
  delay(10);
}

// Simple gyro calibration: average N samples while device is stationary
void calibrateGyro() {
  const int samples = 300;
  const int delayMs = 5;
  Serial.println("Calibrating gyro; keep device still...");
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    // convert to deg/s (Adafruit gives rad/s for gyro)
    float gx = g.gyro.x * 180.0f / M_PI;
    float gy = g.gyro.y * 180.0f / M_PI;
    float gz = g.gyro.z * 180.0f / M_PI;
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(delayMs);
  }
  gyroBiasX = (float)(sumX / samples);
  gyroBiasY = (float)(sumY / samples);
  gyroBiasZ = (float)(sumZ / samples);
  Serial.print("Gyro biases (deg/s): ");
  Serial.print(gyroBiasX); Serial.print(", ");
  Serial.print(gyroBiasY); Serial.print(", ");
  Serial.println(gyroBiasZ);
}


void connect_tufts_wireless(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Start Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retry++;

    if (retry > 40) {    // ~20 seconds
      Serial.println("\nFailed to connect!");
      return;
    }
  }
  Serial.println("Connected!");
}

void get_network_info(){
    if(WiFi.status() == WL_CONNECTED) {
        Serial.print("[*] Network information for ");
        Serial.println(ssid);

        Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
        Serial.print("[+] Gateway IP : ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("[+] Subnet Mask : ");
        Serial.println(WiFi.subnetMask());
        Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
        Serial.print("[+] ESP32 IP : ");
        Serial.println(WiFi.localIP());
    }
}

void sensor_readings() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate dt
  static unsigned long last = micros();
  unsigned long now = micros();
  float dt = (now - last) / 1e6f;
  last = now;

  // Clamp dt to reasonable bounds to avoid huge integration jumps
  if (!isfinite(dt) || dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  // Convert accelerometer (m/s^2 → g)
  float ax = a.acceleration.x / 9.8f;
  float ay = a.acceleration.y / 9.8f;
  float az = a.acceleration.z / 9.8f;

  // Compute accelerometer angles
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  float accel_roll  = atan2f(ay, az);

  // Gyro is already rad/s from Adafruit → convert to deg/s
  float raw_gx = g.gyro.x * 180.0f / M_PI;
  float raw_gy = g.gyro.y * 180.0f / M_PI;
  float raw_gz = g.gyro.z * 180.0f / M_PI;

  // Subtract calibrated biases to get corrected rates
  float gx = raw_gx - gyroBiasX;
  float gy = raw_gy - gyroBiasY;
  float gz = raw_gz - gyroBiasZ;

  // Integrate gyro (deg)
  pitch_deg += gy * dt;
  roll_deg  += gx * dt;
  yaw_deg   += gz * dt;

  float gyro_pitch = pitch_deg * M_PI / 180.0f;
  float gyro_roll  = roll_deg  * M_PI / 180.0f;
  float gyro_yaw   = yaw_deg   * M_PI / 180.0f;

  // Complementary filter
  pitch_rad = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
  roll_rad  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
  yaw_rad   = gyro_yaw;

  pitch_deg = pitch_rad * 180.0f / M_PI;
  roll_deg  = roll_rad  * 180.0f / M_PI;
  yaw_deg   = yaw_rad   * 180.0f / M_PI;

  Serial.println(pitch_deg);
  Serial.println(roll_deg);
  Serial.println(yaw_deg);

  // Online bias adaptation: if device appears stationary (raw angular rates near bias),
  // slowly update bias estimates to compensate drift/temperature changes.
  const float stationaryThreshold = 1.0f; // deg/s
  const float biasAlpha = 0.001f; // slow update rate
  if (fabsf(raw_gx - gyroBiasX) < stationaryThreshold && fabsf(raw_gy - gyroBiasY) < stationaryThreshold && fabsf(raw_gz - gyroBiasZ) < stationaryThreshold) {
    // move bias slightly toward measured raw value
    gyroBiasX = (1.0f - biasAlpha) * gyroBiasX + biasAlpha * raw_gx;
    gyroBiasY = (1.0f - biasAlpha) * gyroBiasY + biasAlpha * raw_gy;
    gyroBiasZ = (1.0f - biasAlpha) * gyroBiasZ + biasAlpha * raw_gz;
  }
}