/**
 ******************************************************************************
 * @file    ms5607.c
 * @author  Gemini
 * @brief   Source file for the MS5607 pressure sensor driver using STM32 HAL.
 ******************************************************************************
 * @attention
 *
 * This driver is a rewrite of a ZephyrOS-based driver to work with the
 * STM32 HAL library, specifically for STM32G0xx series MCUs.
 *
 ******************************************************************************
 */

#include "ms5607.h"
#include <string.h>
#include <math.h>

// I2C timeout in milliseconds
#define MS5607_I2C_TIMEOUT 100

// --- Static Function Prototypes ---

// Low-level I2C communication functions
static HAL_StatusTypeDef ms5607_write_cmd(ms5607_t *dev, uint8_t cmd);
static HAL_StatusTypeDef ms5607_read_prom(ms5607_t *dev, uint8_t prom_addr, uint16_t *value);
static HAL_StatusTypeDef ms5607_read_adc(ms5607_t *dev, uint32_t *value);

// Data processing function
static void ms5607_calculate_compensation(ms5607_t *dev, uint32_t raw_pressure, uint32_t raw_temp);


// --- Public Function Implementations ---

/**
 * @brief Initializes the MS5607 sensor.
 */
HAL_StatusTypeDef ms5607_init(ms5607_t *dev, I2C_HandleTypeDef *i2c_handle, uint8_t device_address)
{
    if (dev == NULL || i2c_handle == NULL) {
        return HAL_ERROR;
    }

    dev->i2c_handle = i2c_handle;
    dev->device_address = device_address;

    // Reset the sensor
    if (ms5607_write_cmd(dev, MS5607_CMD_RESET) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for the reset sequence to complete (datasheet says 2.8 ms)
    HAL_Delay(3);

    // Read all calibration coefficients from the PROM
    if (ms5607_read_prom(dev, 0, &dev->sens_t1) != HAL_OK) return HAL_ERROR;    // C1
    if (ms5607_read_prom(dev, 1, &dev->off_t1) != HAL_OK) return HAL_ERROR;     // C2
    if (ms5607_read_prom(dev, 2, &dev->tcs) != HAL_OK) return HAL_ERROR;        // C3
    if (ms5607_read_prom(dev, 3, &dev->tco) != HAL_OK) return HAL_ERROR;        // C4
    if (ms5607_read_prom(dev, 4, &dev->t_ref) != HAL_OK) return HAL_ERROR;      // C5
    if (ms5607_read_prom(dev, 5, &dev->tempsens) != HAL_OK) return HAL_ERROR;   // C6

    return HAL_OK;
}

/**
 * @brief Reads the pressure and temperature from the sensor.
 */
HAL_StatusTypeDef ms5607_read_values(ms5607_t *dev, ms5607_osr_t osr)
{
    uint32_t raw_pressure = 0;
    uint32_t raw_temp = 0;

    // Commands and delays based on selected OSR
    uint8_t cmd_d1 = MS5607_CMD_CONV_D1_OSR_4096;
    uint8_t cmd_d2 = MS5607_CMD_CONV_D2_OSR_4096;
    uint32_t conv_delay = 9;

    switch (osr) {
        case MS5607_OSR_256:
            cmd_d1 = MS5607_CMD_CONV_D1_OSR_256;
            cmd_d2 = MS5607_CMD_CONV_D2_OSR_256;
            conv_delay = 1;
            break;
        case MS5607_OSR_512:
            cmd_d1 = MS5607_CMD_CONV_D1_OSR_512;
            cmd_d2 = MS5607_CMD_CONV_D2_OSR_512;
            conv_delay = 2;
            break;
        case MS5607_OSR_1024:
            cmd_d1 = MS5607_CMD_CONV_D1_OSR_1024;
            cmd_d2 = MS5607_CMD_CONV_D2_OSR_1024;
            conv_delay = 3;
            break;
        case MS5607_OSR_2048:
            cmd_d1 = MS5607_CMD_CONV_D1_OSR_2048;
            cmd_d2 = MS5607_CMD_CONV_D2_OSR_2048;
            conv_delay = 5;
            break;
        case MS5607_OSR_4096:
            // Already set by default
            break;
    }

    // --- Start Pressure Conversion (D1) ---
    if (ms5607_write_cmd(dev, cmd_d1) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(conv_delay);
    if (ms5607_read_adc(dev, &raw_pressure) != HAL_OK) {
        return HAL_ERROR;
    }

    // --- Start Temperature Conversion (D2) ---
    if (ms5607_write_cmd(dev, cmd_d2) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(conv_delay);
    if (ms5607_read_adc(dev, &raw_temp) != HAL_OK) {
        return HAL_ERROR;
    }

    // --- Calculate Compensated Values ---
    ms5607_calculate_compensation(dev, raw_pressure, raw_temp);

    return HAL_OK;
}

// --- Static Function Implementations ---

/**
 * @brief Writes a single command byte to the sensor.
 */
static HAL_StatusTypeDef ms5607_write_cmd(ms5607_t *dev, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(dev->i2c_handle, dev->device_address, &cmd, 1, MS5607_I2C_TIMEOUT);
}

/**
 * @brief Reads a 16-bit value from the sensor's PROM.
 * @param prom_addr The PROM address index (0-5).
 */
static HAL_StatusTypeDef ms5607_read_prom(ms5607_t *dev, uint8_t prom_addr, uint16_t *value)
{
    uint8_t cmd = MS5607_CMD_PROM_READ_BASE + (prom_addr * 2);
    uint8_t rx_buffer[2];

    if (ms5607_write_cmd(dev, cmd) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(dev->i2c_handle, dev->device_address, rx_buffer, 2, MS5607_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    *value = (uint16_t)(rx_buffer[0] << 8) | rx_buffer[1];
    return HAL_OK;
}

/**
 * @brief Reads the 24-bit raw ADC conversion result.
 */
static HAL_StatusTypeDef ms5607_read_adc(ms5607_t *dev, uint32_t *value)
{
    uint8_t cmd = MS5607_CMD_ADC_READ;
    uint8_t rx_buffer[3];

    if (ms5607_write_cmd(dev, cmd) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(dev->i2c_handle, dev->device_address, rx_buffer, 3, MS5607_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    *value = (uint32_t)(rx_buffer[0] << 16) | (rx_buffer[1] << 8) | rx_buffer[2];
    return HAL_OK;
}

/**
 * @brief Calculates compensated pressure and temperature values.
 * @note This function is a direct translation of the calculation logic
 * from the datasheet and the original Zephyr driver.
 */
static void ms5607_calculate_compensation(ms5607_t *dev, uint32_t raw_pressure, uint32_t raw_temp)
{
    // Intermediate values for calculation
    int64_t dT, OFF, SENS;
    int32_t TEMP;

    // --- First order temperature compensation ---
    dT = (int64_t)raw_temp - ((int64_t)dev->t_ref << 8);
    TEMP = 2000 + (dT * dev->tempsens) / (1LL << 23);

    // --- First order pressure compensation ---
    OFF = ((int64_t)dev->off_t1 << 17) + (dT * dev->tco) / (1LL << 6);
    SENS = ((int64_t)dev->sens_t1 << 16) + (dT * dev->tcs) / (1LL << 7);

    // --- Second order temperature compensation ---
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2 = (dT * dT) / (1LL << 31);
        OFF2 = 61LL * (TEMP - 2000) * (TEMP - 2000) / (1LL << 4);
        SENS2 = 2LL * (TEMP - 2000) * (TEMP - 2000);
        if (TEMP < -1500) {
            int64_t temp_sq = (TEMP + 1500) * (TEMP + 1500);
            OFF2 += 15LL * temp_sq;
            SENS2 += 8LL * temp_sq;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    // --- Final compensated pressure ---
    dev->temperature = TEMP;
    dev->pressure = ((SENS * (int64_t)raw_pressure / (1LL << 21) - OFF) / (1LL << 15));
}

/**
 * @brief Converts pressure reading to altitude in meters.
 * @param pressure Pressure in Pascals (Pa).
 * @return Altitude in meters.
 */
float toAltitude(float pressure) {
    // Using the International Barometric Formula
    // Ref. https://www.te.com/commerce/DocumentDelivery/DDEController?Action=show&DocId=Data+Sheet%7FMS5607-02BA03%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5607-02BA03_B1.pdf%7FCAT-BLPS0036
    const float p0 = 101325.0f; // Standard pressure at sea level in Pa

    // Formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure / p0, 1.0f / 5.255f));
}
