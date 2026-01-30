#include <Arduino.h>
#include <math.h>

// ===== Hardware Configuration =====
#define ANALOG_PIN 7        // ESP32 ADC pin
#define R_FIXED 100000.0      // 10kΩ resistor
#define BETA 3950.0          // Beta value
#define T0 298.15            // 25°C in Kelvin
#define R0 100000.0           // Resistance at 25°C

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);       // 0–4095
    analogSetAttenuation(ADC_11db); // 0–3.3V

    pinMode(ANALOG_PIN, INPUT);

    Serial.println("ESP32 NTC Temperature Measurement Started");
}

void loop() {
    int adcValue = analogRead(ANALOG_PIN);

    // Avoid division by zero
    if (adcValue <= 0 || adcValue >= 4095) {
        Serial.println("ADC out of range");
        delay(1000);
        return;
    }

    // Calculate NTC resistance
    double rNTC = R_FIXED * ((double)adcValue / (4095.0 - adcValue));

    // Beta equation
    double tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rNTC / R0));
    double tempC = tempK - 273.15;
    double tempF = (tempC * 9.0 / 5.0) + 32.0;

    // Output
    Serial.print("ADC: ");
    Serial.print(adcValue);

    Serial.print(" | R_NTC: ");
    Serial.print(rNTC, 0);
    Serial.print(" ohm");

    Serial.print(" | Temp: ");
    Serial.print(tempC, 2);
    Serial.print(" °C  /  ");
    Serial.print(tempF, 2);
    Serial.println(" °F");

    delay(1000);
}
