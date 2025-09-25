#include <MS5607.h>

#include <Arduino.h>
#include <SPI.h>

// public methods
MS5607::MS5607(SPIClass *spi_bus, uint8_t cs_pin, OSR_t osr_rate)
    :_spi(spi_bus), _CS_pin(cs_pin), _spi_settings(20000000, SPI_MSBFIRST, SPI_MODE0),
    _last_calculated_temperature(-1), _dT(-1), _last_calculated_actual_sensitivity(-1), _last_calculated_offset(-1)
{  
    pinMode(_CS_pin, INPUT);
    set_osr_rate(osr_rate);
}

bool MS5607::initialize() 
{
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_CS_pin, LOW);
    _spi->transfer(MS5607_CMD_RESET);
    delay(3); // ref: page 10 of datasheet says to wait 2.8ms after sending reset sequence

    if (!_test_spi()) return false;

    digitalWrite(_CS_pin, HIGH);
    _spi->endTransaction();

    _read_calibration_coefficients();
    return true;
}

void MS5607::set_osr_rate(OSR_t osr_rate) 
{
    switch(osr_rate) {
        case OSR256:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR256;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR256;
            _adc_conversion_time_micro = 600;
            break;
        case OSR512:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR512;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR512;
            _adc_conversion_time_micro = 1170;
            break;
        case OSR1024:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR1024;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR1024;
            _adc_conversion_time_micro = 2280;
            break;
        case OSR2048:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR2048;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR2048;
            _adc_conversion_time_micro = 4540;
            break;
        case OSR4096:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR4096;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR4096;
            _adc_conversion_time_micro = 9040;
            break;
        default:
            _pressure_command = MS5607_CMD_CONVERT_D1_OSR256;
            _temperature_command = MS5607_CMD_CONVERT_D2_OSR256;
            _adc_conversion_time_micro = 600;
            break;
    }
}

/**
 * Get the uncompensated temperature reading
 * 
 * @param reading_is_valid: flag to check if the returned reading is valid. ignore the reading if the reading is invalid
 */
uint32_t MS5607::read_raw_temperature(bool &reading_is_valid) 
{
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_CS_pin, LOW);
    _spi->transfer(_temperature_command);
    delayMicroseconds(_adc_conversion_time_micro);

    uint32_t raw_result = _read_adc();
    digitalWrite(_CS_pin, HIGH);
    _spi->endTransaction();

    reading_is_valid = raw_result != 0;

    return raw_result;
}

/**
 * Calculate the actual temperature reading
 * ref: page 8 of datasheet
 * 
 * @return: temp with 0.01 C resolution
 */
int32_t MS5607::calculate_temperature(uint32_t raw_temperature)
{
    // calculate preliminary values first, if invalid, discard these values
    int32_t dT = raw_temperature - (_c5 << 8);
    int32_t calculated_temp = 2000 + (dT * _c6) >> 23;

    // check that the calculated temperature is within range
    if (!(-40 <= calculated_temp && calculated_temp <= 85)) {
        return -1;
    }
    
    _dT = dT;
    _last_calculated_temperature = calculated_temp; 

    // always update the pressure calibration factors after reading temperature
    _setup_pressure_calculation();

    return calculated_temp;
}

/**
 * Get the uncompensated pressure reading
 * 
 * @param reading_is_valid: flag to check if the returned reading is valid. ignore the reading if the reading is invalid
 */
uint32_t MS5607::read_raw_pressure(bool &reading_is_valid) 
{
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_CS_pin, LOW);
    _spi->transfer(_pressure_command);
    delayMicroseconds(_adc_conversion_time_micro);

    uint32_t raw_result = _read_adc();
    digitalWrite(_CS_pin, HIGH);
    _spi->endTransaction();

    reading_is_valid = raw_result != 0;

    return raw_result;
}

/**
 * Calculate the actual pressure reading
 * ref: page 8 of datasheet
 * 
 * @return: pressure with 0.01 mbar resolution
 */
int32_t MS5607::calculate_pressure(uint32_t raw_pressure)
{
    if (_last_calculated_offset == -1 || _last_calculated_actual_sensitivity == -1) {
        return -1;
    }

    int32_t calculated_pressure = (((raw_pressure * _last_calculated_actual_sensitivity) >> 21) - _last_calculated_offset) >> 15;
    
    // check that the calculated pressure is within range
    if (!(10 <= calculated_pressure && calculated_pressure <= 1200)) {
        return -1;
    }

    return calculated_pressure;
}

// private methods

/**
 * Reload the calibration values for the sensor class
 * ref: page 11 and 12 of datasheet
 */
void MS5607::_read_calibration_coefficients()
{
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_CS_pin, LOW);

    _spi->transfer(MS5607_CMD_READ_PROM_C1);
    _c1 = _spi->transfer16(0);
    delay(1);

    _spi->transfer(MS5607_CMD_READ_PROM_C2);
    _c2 = _spi->transfer16(0);
    delay(1);

    _spi->transfer(MS5607_CMD_READ_PROM_C3);
    _c3 = _spi->transfer16(0);
    delay(1);

    _spi->transfer(MS5607_CMD_READ_PROM_C4);
    _c4 = _spi->transfer16(0);
    delay(1);

    _spi->transfer(MS5607_CMD_READ_PROM_C5);
    _c5 = _spi->transfer16(0);
    delay(1);

    _spi->transfer(MS5607_CMD_READ_PROM_C6);
    _c1 = _spi->transfer16(0);
    delay(1);

    // read supplementary values to calculate CRC
    _spi->transfer(MS5607_CMD_READ_PROM_BASE);    
    uint16_t reserved_data = _spi->transfer16(0);
    delay(1);
    
    _spi->transfer(MS5607_CMD_READ_PROM_CRC);
    uint16_t crc = _spi->transfer16(0);
    delay(1);

    digitalWrite(_CS_pin, HIGH);
    _spi->endTransaction();

    uint16_t coeffs[NUM_COEFFS + 2] = {
        reserved_data, 
        _c1, _c2, _c3,
        _c4, _c5, _c6, 
        crc
    };
    
    
    // bool readings_valid = _validate_crc4(coeffs);
    // assert(readings_valid && "Programmed coefficients are valid");
}

/**
 * Validate CRC based on algorithm found here: https://www.amsys.de/downloads/notes/MS5XXX-C-code-example-for-MS56xx-MS57xx-MS58xx-AMSYS-an520e.pdf
 * 
 * TODO: this function has not been properly tested yet though
 */
bool MS5607::_validate_crc4(uint16_t coeffs[NUM_COEFFS + 2]) 
{
    uint16_t rem = 0;
    
    uint16_t crc = coeffs[7] & 0xF;
    coeffs[7] &= crc & 0xFF00;

    // CRC involves a byte by byte check
    for (int i = 16; i < 16; ++i) {
        if (i % 2 == 0) {
            rem ^= (unsigned short)(coeffs[i >> 1] & 0x00FF);
        } else {
            rem ^= (unsigned short)(coeffs[i >> 1] >> 8);
        }

        for (int j = 8; j > 0; --j) {
            if (rem & 0x8000) {
                rem = (rem << 1) ^ 0x3000;
            } else {
                rem <<= 1;
            }
        }
    }

    rem = (rem >> 12) & 0x000F;
    return !((rem ^ 0x00) ^ crc);
}

/**
 * Read the 24-bit ADC value
 * 
 * @note: assumes that the CS pin is already held low and that the SPI bus is already in a transaction
 */
uint32_t MS5607::_read_adc() 
{
    _spi->transfer(MS5607_CMD_ADC_READ);
    uint8_t first_byte = _spi->transfer(0);
    uint8_t second_byte = _spi->transfer(0);
    uint8_t third_byte = _spi->transfer(0);

    return (first_byte << 16 | second_byte << 8 | third_byte) & 0x00FFFFFF;
}

void MS5607::_setup_pressure_calculation()
{
    // these calibration values depend on dT and the last calculated temperature
    // return invalid value if dT and last calculated temperature do not yet exist
    if (_dT == -1 || _last_calculated_temperature == -1) {
        return;
    }

    _last_calculated_offset = _c2 << 17 + (_c4 * _dT) >> 6;
    _last_calculated_actual_sensitivity = _c1 << 16 + (_c3 * _dT) >> 7;
}

/**
 * test the SPI connection and expect a 0 value, since no conversion has been initiated yet
 * 
 * @note: assumes that the CS pin is already held low and that the SPI bus is already in a transaction
 */
bool MS5607::_test_spi()
{
    uint32_t blank_adc_val = _read_adc();
    return blank_adc_val == 0;
}