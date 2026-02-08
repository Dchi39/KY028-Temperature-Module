#include <Arduino.h>
#include <math.h>

// ================= PIN CONFIG =================
#define TEMP_ADC_PIN      4    // main sensor for PID
#define TEMP_ADC_PIN_2    5    // monitoring sensor

#define HEATER_PWM_PIN_1  15
#define HEATER_PWM_PIN_2  16
#define HEATER_PWM_PIN_3  17
#define HEATER_PWM_PIN_4  12

#define IDLE_BUTTON_PIN   10   // HIGH = IDLE mode

// ================= PWM CONFIG =================
#define PWM_FREQ 1000
#define PWM_RES  8            // 0–255

#define PWM_CH_1 0
#define PWM_CH_2 1
#define PWM_CH_3 2
#define PWM_CH_4 3

#define PWM_MAX 255

// ================= NTC PARAMETERS =================
#define R_FIXED 100000.0
#define R0      100000.0
#define BETA    3950.0
#define T0      298.15

// ================= PID PARAMETERS =================
float Kp = 4.0;
float Ki = 0.05;
float Kd = 1.5;

// ================= SETPOINTS =================
#define SETPOINT_NORMAL 170.0
#define SETPOINT_IDLE   100.0
#define HYST            0.5
#define BOOST_TEMP      169.0
#define TEMP_MAX_SAFE   200.0

float setPoint = SETPOINT_NORMAL;

// ================= PID VARIABLES =================
float integral  = 0;
float lastError = 0;
unsigned long lastTime = 0;

// ================= MODE TRACKING =================
bool lastIdleMode = false;

// ================= DEBUG =================
uint8_t pidPwm   = 0;
uint8_t coil3Pwm = 0;
uint8_t coil4Pwm = 0;

// ================= FUNCTION DECLARATIONS =================
float readTemperature(uint8_t pin);
void pidControl(float currentTemp);

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(IDLE_BUTTON_PIN, INPUT);

    // PWM setup
    ledcSetup(PWM_CH_1, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_2, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_3, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_4, PWM_FREQ, PWM_RES);

    ledcAttachPin(HEATER_PWM_PIN_1, PWM_CH_1);
    ledcAttachPin(HEATER_PWM_PIN_2, PWM_CH_2);
    ledcAttachPin(HEATER_PWM_PIN_3, PWM_CH_3);
    ledcAttachPin(HEATER_PWM_PIN_4, PWM_CH_4);

    // All heaters OFF
    ledcWrite(PWM_CH_1, 0);
    ledcWrite(PWM_CH_2, 0);
    ledcWrite(PWM_CH_3, 0);
    ledcWrite(PWM_CH_4, 0);

    lastTime = millis();

    Serial.println("Mode | MainTemp | SP | PID | Coil3 | Coil4 | Temp2");
}

void loop() {
    // -------- MODE SELECTION --------
    bool idleMode = digitalRead(IDLE_BUTTON_PIN);

    if (idleMode != lastIdleMode) {
        integral = 0;
        lastError = 0;
        lastTime = millis();
    }

    setPoint = idleMode ? SETPOINT_IDLE : SETPOINT_NORMAL;
    lastIdleMode = idleMode;

    // -------- TEMPERATURE READ --------
    float temperature  = readTemperature(TEMP_ADC_PIN);
    float temperature2 = readTemperature(TEMP_ADC_PIN_2);

    // -------- CONTROL --------
    pidControl(temperature);

    // -------- DEBUG PRINT --------
    Serial.print(idleMode ? "IDLE" : "NORMAL");
    Serial.print(" | ");
    Serial.print(temperature, 1);
    Serial.print(" | ");
    Serial.print(setPoint, 1);
    Serial.print(" | ");
    Serial.print(pidPwm);
    Serial.print(" | ");
    Serial.print(coil3Pwm);
    Serial.print(" | ");
    Serial.print(coil4Pwm);
    Serial.print(" | ");
    Serial.println(temperature2, 1);

    delay(200);
}

void pidControl(float currentTemp) {
    // -------- SAFETY --------
    if (currentTemp < 0 || currentTemp > TEMP_MAX_SAFE) {
        pidPwm = coil3Pwm = coil4Pwm = 0;
        ledcWrite(PWM_CH_1, 0);
        ledcWrite(PWM_CH_2, 0);
        ledcWrite(PWM_CH_3, 0);
        ledcWrite(PWM_CH_4, 0);
        return;
    }

    // -------- BOOST MODE --------
    if (currentTemp <= BOOST_TEMP) {
        pidPwm = PWM_MAX;
        lastTime = millis();
    }
    else {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        if (dt <= 0) return;

        float error = setPoint - currentTemp;

        if (currentTemp >= setPoint) {
            pidPwm = 0;
            integral = 0;
        } else {
            integral += error * dt;
            integral = constrain(integral, 0, 300);

            float derivative = (error - lastError) / dt;
            float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            pidPwm = constrain((int)output, 0, PWM_MAX);
        }

        lastError = error;
        lastTime = now;
    }

    // -------- PWM OUTPUT --------
    ledcWrite(PWM_CH_1, pidPwm);
    ledcWrite(PWM_CH_2, pidPwm);

    coil3Pwm = map(pidPwm, 0, 255, 0, 200);
    coil4Pwm = map(pidPwm, 0, 255, 0, 180);

    ledcWrite(PWM_CH_3, coil3Pwm);
    ledcWrite(PWM_CH_4, coil4Pwm);
}

// ================= TEMPERATURE READ =================
float readTemperature(uint8_t pin) {
    int adc = analogRead(pin);
    if (adc <= 0 || adc >= 4095) return -100;

    double rNTC = R_FIXED * ((double)adc / (4095.0 - adc));
    double tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rNTC / R0));
    return tempK - 273.15;
}