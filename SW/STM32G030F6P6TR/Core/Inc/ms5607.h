/**
 ******************************************************************************
 * @file    ms5607.h
 * @author  Gemini
 * @brief   Header file for the MS5607 pressure sensor driver using STM32 HAL.
 ******************************************************************************
 * @attention
 *
 * This driver is a rewrite of a ZephyrOS-based driver to work with the
 * STM32 HAL library, specifically for STM32G0xx series MCUs.
 *
 ******************************************************************************
 */

#ifndef MS5607_H
#define MS5607_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h" // Change this to your specific STM32 series header if needed

// Default I2C address for MS5607. Can be 0x76 or 0x77 depending on CSB pin.
#define MS5607_I2C_ADDR_PRIMARY   (0x77 << 1) // CSB pin is high
#define MS5607_I2C_ADDR_SECONDARY (0x76 << 1) // CSB pin is low

// MS5607 Commands
#define MS5607_CMD_RESET                0x1E
#define MS5607_CMD_CONV_D1_OSR_256      0x40
#define MS5607_CMD_CONV_D1_OSR_512      0x42
#define MS5607_CMD_CONV_D1_OSR_1024     0x44
#define MS5607_CMD_CONV_D1_OSR_2048     0x46
#define MS5607_CMD_CONV_D1_OSR_4096     0x48
#define MS5607_CMD_CONV_D2_OSR_256      0x50
#define MS5607_CMD_CONV_D2_OSR_512      0x52
#define MS5607_CMD_CONV_D2_OSR_1024     0x54
#define MS5607_CMD_CONV_D2_OSR_2048     0x56
#define MS5607_CMD_CONV_D2_OSR_4096     0x58
#define MS5607_CMD_ADC_READ             0x00
#define MS5607_CMD_PROM_READ_BASE       0xA0 // Base address for PROM read commands

/**
 * @brief Enum for selecting the oversampling ratio (OSR).
 * Higher OSR gives better resolution but takes more time.
 */
typedef enum {
    MS5607_OSR_256  = 0, // Conversion time: 1 ms
    MS5607_OSR_512  = 1, // Conversion time: 2 ms
    MS5607_OSR_1024 = 2, // Conversion time: 3 ms
    MS5607_OSR_2048 = 3, // Conversion time: 5 ms
    MS5607_OSR_4096 = 4, // Conversion time: 9 ms
} ms5607_osr_t;

/**
 * @brief Main device handle structure for the MS5607 sensor.
 */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;      // Pointer to the I2C handle
    uint8_t            device_address;  // I2C address of the sensor

    // Calibration coefficients read from the sensor's PROM
    uint16_t           sens_t1;         // C1: Pressure sensitivity
    uint16_t           off_t1;          // C2: Pressure offset
    uint16_t           tcs;             // C3: Temperature coefficient of pressure sensitivity
    uint16_t           tco;             // C4: Temperature coefficient of pressure offset
    uint16_t           t_ref;           // C5: Reference temperature
    uint16_t           tempsens;        // C6: Temperature coefficient of the temperature

    // Compensated values
    int32_t            temperature;     // Temperature in degrees C * 100
    int32_t            pressure;        // Pressure in mbar * 100
} ms5607_t;

/**
 * @brief Initializes the MS5607 sensor.
 * @param dev Pointer to the MS5607 device structure.
 * @param i2c_handle Pointer to the I2C peripheral handle.
 * @param device_address The I2C address of the sensor.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef ms5607_init(ms5607_t *dev, I2C_HandleTypeDef *i2c_handle, uint8_t device_address);

/**
 * @brief Reads the pressure and temperature from the sensor.
 * @param dev Pointer to the MS5607 device structure.
 * @param osr The oversampling ratio to use for the measurement.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 * @note The results are stored in dev->pressure and dev->temperature.
 */
HAL_StatusTypeDef ms5607_read_values(ms5607_t *dev, ms5607_osr_t osr);


/**
 * @brief Converts pressure reading to altitude in meters.
 * @param pressure Pressure in Pascals (Pa).
 * @return Altitude in meters.
 */
float toAltitude(float pressure);

#ifdef __cplusplus
}
#endif

#endif // MS5607_H
