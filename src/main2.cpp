#include <Arduino.h>
#include <math.h>

// ================= PIN CONFIG =================
#define TEMP_ADC_PIN_c1 4
#define TEMP_ADC_PIN_c2 5
#define TEMP_ADC_PIN_c3 6
#define TEMP_ADC_PIN_c4 7

#define HEATER_PWM_PIN_c1 15
#define HEATER_PWM_PIN_c2 16
#define HEATER_PWM_PIN_c3 17
#define HEATER_PWM_PIN_c4 18

// ================= PWM CONFIG =================
#define PWM_FREQ 1000
#define PWM_RES  8      // 0–255

#define PWM_CH_c1 0
#define PWM_CH_c2 1
#define PWM_CH_c3 2
#define PWM_CH_c4 3

// ================= NTC PARAMETERS =================
#define R_FIXED 100000.0
#define R0      100000.0
#define BETA    3950.0
#define T0      298.15   // 25°C in Kelvin

// ================= PID PARAMETERS =================
float Kp = 12.0;
float Ki = 0.5;
float Kd = 30.0;

// ================= CONTROL =================
// Independent setpoints for each heater
float setPoint[4] = {40.0, 40.0, 30.0, 30.0};

// ================= PID VARIABLES =================
float integral[4]  = {0, 0, 0, 0};
float lastError[4] = {0, 0, 0, 0};
unsigned long lastTime[4] = {0, 0, 0, 0};

// ================= FUNCTION =================
float readTemperature(uint8_t pin);
void pidControl(uint8_t index, float currentTemp, uint8_t pwmChannel);

void setup() {
    Serial.begin(115200);

    // ADC setup
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // PWM setup
    ledcSetup(PWM_CH_c1, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_c2, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_c3, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_c4, PWM_FREQ, PWM_RES);

    ledcAttachPin(HEATER_PWM_PIN_c1, PWM_CH_c1);
    ledcAttachPin(HEATER_PWM_PIN_c2, PWM_CH_c2);
    ledcAttachPin(HEATER_PWM_PIN_c3, PWM_CH_c3);
    ledcAttachPin(HEATER_PWM_PIN_c4, PWM_CH_c4);

    unsigned long now = millis();
    for (int i = 0; i < 4; i++) lastTime[i] = now;

    Serial.println("ESP32 4-Heater PID Controller with Independent Setpoints Started");
}

void loop() {
    float temp[4];

    temp[0] = readTemperature(TEMP_ADC_PIN_c1);
    temp[1] = readTemperature(TEMP_ADC_PIN_c2);
    temp[2] = readTemperature(TEMP_ADC_PIN_c3);
    temp[3] = readTemperature(TEMP_ADC_PIN_c4);

    pidControl(0, temp[0], PWM_CH_c1);
    pidControl(1, temp[1], PWM_CH_c2);
    pidControl(2, temp[2], PWM_CH_c3);
    pidControl(3, temp[3], PWM_CH_c4);

    // Debug output
    Serial.print("T1: "); Serial.print(temp[0],1); Serial.print(" | SP: "); Serial.print(setPoint[0]);
    Serial.print(" | T2: "); Serial.print(temp[1],1); Serial.print(" | SP: "); Serial.print(setPoint[1]);
    Serial.print(" | T3: "); Serial.print(temp[2],1); Serial.print(" | SP: "); Serial.print(setPoint[2]);
    Serial.print(" | T4: "); Serial.print(temp[3],1); Serial.print(" | SP: "); Serial.println(setPoint[3]);

    delay(200);
}

// ================= PID CONTROL FUNCTION =================
void pidControl(uint8_t i, float currentTemp, uint8_t pwmChannel) {
    unsigned long now = millis();
    float dt = (now - lastTime[i]) / 1000.0;
    if (dt <= 0) return;

    float error = setPoint[i] - currentTemp;
    integral[i] += error * dt;
    float derivative = (error - lastError[i]) / dt;

    float output = (Kp * error) + (Ki * integral[i]) + (Kd * derivative);
    output = constrain(output, 0, 255);

    ledcWrite(pwmChannel, (int)output);

    lastError[i] = error;
    lastTime[i] = now;
}

// ================= NTC READ FUNCTION =================
float readTemperature(uint8_t pin) {
    int adc = analogRead(pin);
    if (adc <= 0 || adc >= 4095) return -100;

    double rNTC = R_FIXED * ((double)adc / (4095.0 - adc));
    double tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rNTC / R0));
    return tempK - 273.15;
}
