#include <Arduino.h>
#include <SPI.h>
#include <MS5607.h>
#include <HardwareSerial.h>

// NOTE: edit SPI pins based on wiring
#define CLK 10
#define MISO 10
#define MOSI 10
#define CS 10

SPIClass spi_bus;
MS5607 ms5607(&spi_bus, CS, MS5607::OSR_Rate::OSR512);

void setup() {
    Serial.begin(115200);
    delay(1000);

    spi_bus.begin(CLK, MISO, MOSI, -1);
    bool ms5607_initialization_result = ms5607.initialize();
    
    // put in indefinite spin loop if initialization does not work
    while (!ms5607_initialization_result) {}
}

void loop() {
    bool reading_is_valid = false;

    uint32_t raw_temp = ms5607.read_raw_temperature(reading_is_valid);
    if (!reading_is_valid) {
        Serial.println("Invalid raw temperature read, restarting loop...");
        return;
    }

    int32_t temp_value = ms5607.calculate_temperature(raw_temp);
    if (temp_value == -1) {
        Serial.println("Invalid temperature calculated, restarting loop...");
        return;
    }

    reading_is_valid = false;
    uint32_t raw_pressure = ms5607.read_raw_pressure(reading_is_valid);
    if (!reading_is_valid) {
         Serial.println("Invalid raw pressure read, restarting loop...");
        return;
    }

    int32_t pressure_value = ms5607.calculate_pressure(raw_pressure);
    if (pressure_value == -1) {
        Serial.println("Invalid pressure calculated, restarting loop...");
        return;
    }

    // temp_value is in 0.01 °C, pressure_value is in 0.01 mbar
    float temperature_c = temp_value / 100.0;
    float pressure_mbar = pressure_value / 100.0;

    // Format into Arduino string
    String output = "Temperature: " + String(temperature_c, 2) + " °C, "
                    "Pressure: " + String(pressure_mbar, 2) + " mbar";

    Serial.println(output);

    delay(10);
}