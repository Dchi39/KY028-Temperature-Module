#include <Arduino.h>
#include <math.h>

// ===== PIN CONFIG =====
#define TEMP_ADC_PIN 34
#define HEATER_PWM_PIN 25

// ===== NTC PARAMETERS =====
#define R_FIXED 10000.0
#define R0 10000.0
#define BETA 3435.0
#define T0 298.15     // 25°C in Kelvin

// ===== PID PARAMETERS (TUNE THESE) =====
float Kp = 12.0;
float Ki = 0.5;
float Kd = 30.0;

// ===== CONTROL SETTINGS =====
float setPoint = 40.0;   // Target temperature (°C)
float pwmOutput = 0;

// ===== PID VARIABLES =====
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// ===== PWM SETTINGS =====
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RES 8     // 0–255

// ===== FUNCTION =====
float readTemperature();

void setup() {
    Serial.begin(115200);

    // ADC setup
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // PWM setup
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(HEATER_PWM_PIN, PWM_CHANNEL);

    lastTime = millis();
    Serial.println("ESP32 PID Temperature Controller Started");
}

void loop() {
    float currentTemp = readTemperature();
    unsigned long now = millis();
    float deltaTime = (now - lastTime) / 1000.0; // seconds

    // PID calculation
    float error = setPoint - currentTemp;
    integral += error * deltaTime;
    float derivative = (error - lastError) / deltaTime;

    pwmOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Clamp PWM
    pwmOutput = constrain(pwmOutput, 0, 255);

    ledcWrite(PWM_CHANNEL, (int)pwmOutput);

    // Debug output
    Serial.print("Temp: ");
    Serial.print(currentTemp, 2);
    Serial.print(" °C | PWM: ");
    Serial.println((int)pwmOutput);

    lastError = error;
    lastTime = now;

    delay(200); // control loop speed
}

// ===== NTC TEMPERATURE FUNCTION =====
float readTemperature() {
    int adc = analogRead(TEMP_ADC_PIN);

    if (adc <= 0 || adc >= 4095) return -100;

    double rNTC = R_FIXED * ((double)adc / (4095.0 - adc));
    double tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rNTC / R0));
    return tempK - 273.15;
}
