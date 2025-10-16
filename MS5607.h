#ifndef MS5607_SPI_H_
#define MS5607_SPI_H_

#include <Arduino.h>
#include <SPI.h>

/* MS5607 Command Set */
#define MS5607_CMD_RESET              0x1E  /* Reset command */

/* PROM Read Commands (Calibration data) */
#define MS5607_CMD_READ_PROM_BASE     0xA0  /* Base address for PROM read */
#define MS5607_CMD_READ_PROM_C1       0xA2  /* Pressure sensitivity */
#define MS5607_CMD_READ_PROM_C2       0xA4  /* Pressure offset */
#define MS5607_CMD_READ_PROM_C3       0xA6  /* Temperature coefficient of pressure sensitivity */
#define MS5607_CMD_READ_PROM_C4       0xA8  /* Temperature coefficient of pressure offset */
#define MS5607_CMD_READ_PROM_C5       0xAA  /* Reference temperature */
#define MS5607_CMD_READ_PROM_C6       0xAC  /* Temperature coefficient of the temperature */
#define MS5607_CMD_READ_PROM_CRC      0xAE  /* CRC check */
#define NUM_COEFFS                    6

/* ADC Read */
#define MS5607_CMD_ADC_READ           0x00  /* Read ADC result */

/* Pressure Conversion Commands (D1) */
#define MS5607_CMD_CONVERT_D1_OSR256  0x40
#define MS5607_CMD_CONVERT_D1_OSR512  0x42
#define MS5607_CMD_CONVERT_D1_OSR1024 0x44
#define MS5607_CMD_CONVERT_D1_OSR2048 0x46
#define MS5607_CMD_CONVERT_D1_OSR4096 0x48

/* Temperature Conversion Commands (D2) */
#define MS5607_CMD_CONVERT_D2_OSR256  0x50
#define MS5607_CMD_CONVERT_D2_OSR512  0x52
#define MS5607_CMD_CONVERT_D2_OSR1024 0x54
#define MS5607_CMD_CONVERT_D2_OSR2048 0x56
#define MS5607_CMD_CONVERT_D2_OSR4096 0x58


class MS5607 
{
public:
    typedef enum OSR_Rate {
        OSR256,
        OSR512,
        OSR1024,
        OSR2048,
        OSR4096,
    } OSR_t;

    MS5607(SPIClass *spi_bus, uint8_t cs_pin, OSR_t osr_rate = OSR256);

    bool initialize();
    void set_osr_rate(OSR_t osr_rate);
    uint32_t read_raw_temperature(bool &reading_is_valid);
    int32_t calculate_temperature(uint32_t raw_temperature);
    uint32_t read_raw_pressure(bool &reading_is_valid);
    int32_t calculate_pressure(uint32_t raw_pressure);
    float get_altitude(uint32_t t, uint32_t p);
    float get_altitude_2(uint32_t p_pa);
    void dump_calibration_coeffs();

private:
    SPIClass *_spi;
    SPISettings _spi_settings;

    uint8_t _CS_pin;

    // calibration coeffs
    float _c1 {-1};
    float _c2 {-1};
    float _c3 {-1};
    float _c4 {-1};
    float _c5 {-1};
    float _c6 {-1};

    // OSR commands -- set based on the OSR rate
    uint16_t _pressure_command;
    uint16_t _temperature_command;
    long _adc_conversion_time_micro;

    // used to record the last recorded temp and pressure, for calculation purposes
    float _last_calculated_temperature;
    float _dT; // diff between actual and reference temperature
    
    // allow for pressure calculations with cached temperature readings
    float _last_calculated_offset;
    float _last_calculated_actual_sensitivity;

    void _read_calibration_coefficients();
    bool _validate_crc4(uint16_t coeffs[NUM_COEFFS + 2]);
    uint32_t _read_adc();
    void _setup_pressure_calculation();
    bool _test_spi();

    uint16_t _read_prom(uint8_t command);
    void _send_command(uint8_t command);
};

#endif // MS5607_SPI_H_