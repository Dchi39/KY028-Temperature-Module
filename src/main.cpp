#include <Arduino.h>
#include <math.h>

// ===== ESP32 Analog Pin =====
#define ANALOG_INPUT 7   

// ===== Variables =====
int analog_output;
int revised_output;
float temp_C;
float temp_F;

// ===== Function Prototype =====
double Thermistor(int RawADC);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // ESP32 ADC setup
    analogReadResolution(12);           // 12-bit ADC (0–4095)
    analogSetAttenuation(ADC_11db);     // Full range ~0–3.3V

    pinMode(ANALOG_INPUT, INPUT);

    Serial.println("ESP32 Analog Temperature Sensor Started");
}

void loop()
{
    // Read analog value
    analog_output = analogRead(ANALOG_INPUT);

    // Reverse output (KY-028 thermistor wiring is inverted)
    revised_output = map(analog_output, 0, 4095, 4095, 0);

    // Calculate temperature
    temp_C = Thermistor(revised_output);
    temp_F = (temp_C * 9.0 / 5.0) + 32.0;

    // Print values
    Serial.print("ADC Value: ");
    Serial.println(analog_output);

    Serial.print("Temperature: ");
    Serial.print(temp_C, 1);
    Serial.print(" °C | ");
    Serial.print(temp_F, 1);
    Serial.println(" °F");

    Serial.println("---------------------------");

    delay(1000);
}

// ===== Thermistor Calculation =====
double Thermistor(int RawADC)
{
    if (RawADC <= 0) return -273.15; // safety check

    double Temp;
    Temp = log((40950000.0 / RawADC) - 10000.0);
    Temp = 1.0 / (0.001129148 +
                  (0.000234125 * Temp) +
                  (0.0000000876741 * Temp * Temp * Temp));
    Temp = Temp - 273.15; // Kelvin → Celsius
    return Temp;
}
