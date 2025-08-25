/**
 ******************************************************************************
 * @file    AltimeterStateMachine.c
 * @author  pauli
 * @brief   Main program for a dual-deployment altimeter.
 ******************************************************************************
 * @attention
 *
 * This file contains the complete flight logic from pre-flight to recovery.
 * Ensure all GPIOs (continuity inputs, pyro outputs) are configured in CubeMX.
 *
 ******************************************************************************
 */

#include "main.h"
#include "ms5607.h"
#include "buzzer.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

// --- Add this to your main.h or here ---
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;

// --- Altimeter Configuration ---
#define LAUNCH_DETECTION_ALTITUDE_FT    200.0f
#define MAIN_DEPLOYMENT_ALTITUDE_FT     750.0f
#define BOOST_PHASE_LOCKOUT_MS          8000 // 8 seconds
#define APOGEE_WINDOW_SIZE              10   // 1-second window at 10Hz
#define APOGEE_TIP_OVER_THRESHOLD_M     15.0f // Must drop 15m below max to fire
#define SENSOR_OUTLIER_THRESHOLD_M      500.0f // Ignore readings >500m above max
#define PYRO_FIRE_DURATION_MS           1000 // Fire pyro channels for 1 second
#define LANDED_BEEP_PAUSE_MS            15000 // 15 seconds between recovery beeps
#define FEET_TO_METERS(feet) ((feet) * 0.3048f)


// --- Code Size Optimization ---
// To reduce code size, printf statements have been wrapped in this macro.
// To enable serial debugging, change this to 1. To disable, change to 0.

#if ALTIMETER_DEBUG
#define LOG_INFO(...) printf(__VA_ARGS__)
#else
#define LOG_INFO(...)
#endif


// --- Global variables ---
extern ms5607_t ms5607_sensor;
typedef enum {
    STATE_PREFLIGHT,
    STATE_LAUNCH_DETECT,
    STATE_BOOST,
    STATE_FLIGHT,          // Coasting, looking for apogee
    STATE_DROGUE_DESCENT,  // Under drogue, looking for main deployment altitude
    STATE_LANDED,          // On the ground, beeping out max altitude
    STATE_ERROR
} AltimeterState;

AltimeterState current_state = STATE_PREFLIGHT;

// --- Flash Storage Configuration ---
#define FLASH_STORAGE_ADDRESS   0x08007000 // For 32KB STM32G0

// --- Function Prototypes ---
float toAltitude(float pressure);
HAL_StatusTypeDef run_preflight_and_launch_detection(void);
void fire_pyro_channel(int channel);


// For redirecting printf to UART
#if ALTIMETER_DEBUG
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif


/**
 * @brief Writes a float value to a specified flash memory address.
 * @param address The address in flash memory to write to.
 * @param value The float value to be written.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef flash_write_float(uint32_t address, float value) {
    HAL_StatusTypeDef status;
    uint32_t data_to_write;
    // Use memcpy to avoid strict-aliasing warnings. This is the safe way to type-pun.
    memcpy(&data_to_write, &value, sizeof(data_to_write));

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return status;

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages = 1;
    uint32_t PageError = 0;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    uint64_t double_word_to_write = 0xFFFFFFFF00000000 | data_to_write;
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, double_word_to_write);

    HAL_FLASH_Lock();
    return status;
}

/**
 * @brief Reads a float value from a specified flash memory address.
 * @param address The address in flash memory to read from.
 * @return The float value read from flash.
 */
float flash_read_float(uint32_t address) {
    uint32_t read_data = *(__IO uint32_t *)address;
    float read_value;
    // Use memcpy to safely convert the read integer bits back to a float.
    memcpy(&read_value, &read_data, sizeof(read_value));
    return read_value;
}


// --- Pyro Control ---
void fire_pyro_channel(int channel) {
    if (channel == 1) {
    	LOG_INFO("!!! FIRING PYRO CHANNEL 1 (DROGUE) !!!\r\n");
        HAL_GPIO_WritePin(PYRO1_FIRE_GPIO_Port, PYRO1_FIRE_Pin, GPIO_PIN_SET);
        HAL_Delay(PYRO_FIRE_DURATION_MS);
        HAL_GPIO_WritePin(PYRO1_FIRE_GPIO_Port, PYRO1_FIRE_Pin, GPIO_PIN_RESET);
    } else if (channel == 2) {
    	LOG_INFO("!!! FIRING PYRO CHANNEL 2 (MAIN) !!!\r\n");
        HAL_GPIO_WritePin(PYRO2_FIRE_GPIO_Port, PYRO2_FIRE_Pin, GPIO_PIN_SET);
        HAL_Delay(PYRO_FIRE_DURATION_MS);
        HAL_GPIO_WritePin(PYRO2_FIRE_GPIO_Port, PYRO2_FIRE_Pin, GPIO_PIN_RESET);
    }
}

HAL_StatusTypeDef run_preflight_and_launch_detection(void) {
	LOG_INFO("--- PRE-FLIGHT CHECKS ---\r\n");
	bool pyro1_fail = (HAL_GPIO_ReadPin(PYRO1_CONT_GPIO_Port, PYRO1_CONT_Pin) == GPIO_PIN_RESET);
	bool pyro2_fail = (HAL_GPIO_ReadPin(PYRO2_CONT_GPIO_Port, PYRO2_CONT_Pin) == GPIO_PIN_RESET);

	// Because of limited Flash space, run Single buzzer sequence for continuity issues
	if (pyro1_fail || pyro2_fail) {
		LOG_INFO("ERROR: Pyro Channel 1 or 2 has continuity.\r\n");

		// Checking the macro here since during test mode, I don't really care if the pyros are actually connected
		if (!ALTIMETER_DEBUG) {
			while (1) {
				// Three short beeps for both channels failing
				Buzzer_Start(150); HAL_Delay(300); Buzzer_Update();
				Buzzer_Start(150); HAL_Delay(300); Buzzer_Update();
				Buzzer_Start(150); HAL_Delay(300); Buzzer_Update();
				HAL_Delay(1000);
			}
		}
	}

	LOG_INFO("Continuity PASSED. Armed in 30s.\r\n");
    HAL_Delay(30000);
    Buzzer_Start(1000); HAL_Delay(1000); Buzzer_Update();

    LOG_INFO("ARMED. Waiting for launch.\r\n");
    current_state = STATE_LAUNCH_DETECT;
    float ground_altitude_m = 0.0f;
    const int num_samples = 50;
    for (int i = 0; i < num_samples; i++) {
        if (ms5607_read_values(&ms5607_sensor, MS5607_OSR_4096) != HAL_OK) return HAL_ERROR;
        ground_altitude_m += toAltitude((float)ms5607_sensor.pressure);
        HAL_Delay(50);
    }
    ground_altitude_m /= num_samples;

	#if ALTIMETER_DEBUG
    	int ground_alt_int = (int)ground_altitude_m;
    	int ground_alt_frac = (int)((ground_altitude_m - ground_alt_int) * 100);
    	if(ground_alt_frac < 0) ground_alt_frac = -ground_alt_frac;
    	LOG_INFO("Ground altitude: %d.%02d m\r\n", ground_alt_int, ground_alt_frac);
	#endif

    const float launch_threshold_m = FEET_TO_METERS(LAUNCH_DETECTION_ALTITUDE_FT);
    while(1) {
        Buzzer_Update();
        if (ms5607_read_values(&ms5607_sensor, MS5607_OSR_4096) == HAL_OK) {
            float current_altitude_m = toAltitude((float)ms5607_sensor.pressure);
            if (current_altitude_m > (ground_altitude_m + launch_threshold_m)) {
            	LOG_INFO("\r\n--- LAUNCH DETECTED! ---\r\n");
            	break;
            }
            // Continue Chirping while waiting for launch detect (Pad Awareness)
            Buzzer_Start(20); HAL_Delay(20); Buzzer_Update();
        }
    }
    return HAL_OK;
}


// --- Main application ---

int AltimeterStateMachine(void)
{
  /* System initialization */
	LOG_INFO("AttoDD V1.0\r\n");


  Buzzer_Start(3000);
  HAL_Delay(500);
  Buzzer_Update(); // Ensure buzzer stops
  HAL_Delay(200);

  // Read the last saved altitude from flash on startup
  float last_saved_altitude = flash_read_float(FLASH_STORAGE_ADDRESS);
  buzz_out_altitude(last_saved_altitude, 500);


  if (ms5607_init(&ms5607_sensor, &hi2c1, MS5607_I2C_ADDR_PRIMARY) != HAL_OK) {
    current_state = STATE_ERROR; while(1);
  }


  if (run_preflight_and_launch_detection() != HAL_OK) {
	  current_state = STATE_ERROR; while(1);
  }

  /* --- Main Flight State Machine --- */
  LOG_INFO("\r\n--- ENTERING FLIGHT LOOP ---\r\n");

  current_state = STATE_BOOST;

  uint32_t boost_start_time = HAL_GetTick();
  uint32_t last_beep_time = 0;
  float max_altitude = 0.0f;
  bool drogue_fired = false;
  bool main_fired = false;

  float altitude_buffer[APOGEE_WINDOW_SIZE] = {0};
  int buffer_index = 0;
  bool buffer_full = false;

  while (1)
  {
    Buzzer_Update();

    float current_altitude_m = 0.0f;

    // Okay I know It's cringe to put everything into one line, but fight me.
    if (ms5607_read_values(&ms5607_sensor, MS5607_OSR_4096) == HAL_OK) {current_altitude_m = toAltitude((float)ms5607_sensor.pressure);}
    else {LOG_INFO("Sensor read error in flight loop!\r\n");}

    // --- Fill and Update Circular Buffer (used in multiple states) ---
    altitude_buffer[buffer_index] = current_altitude_m;
    buffer_index++;
    if (buffer_index >= APOGEE_WINDOW_SIZE) {
        buffer_index = 0;
        buffer_full = true;
    }

    // --- Calculate Moving Average (if buffer is full) ---
    float moving_avg = 0.0f;
    if (buffer_full) {
        float sum = 0;
        for (int i = 0; i < APOGEE_WINDOW_SIZE; i++) {
            sum += altitude_buffer[i];
        }
        moving_avg = sum / APOGEE_WINDOW_SIZE;
    }

	#if ALTIMETER_DEBUG
		// Pre-calculate integer parts for logging to avoid repetition
		int alt_int = (int)current_altitude_m;
		int alt_frac = (int)((current_altitude_m - alt_int) * 100);
		if(alt_frac < 0) alt_frac = -alt_frac;

		int avg_int = (int)moving_avg;
		int avg_frac = (int)((moving_avg - avg_int) * 100);
		if(avg_frac < 0) avg_frac = -avg_frac;

		int max_int = (int)max_altitude;
		int max_frac = (int)((max_altitude - max_int) * 100);
		if(max_frac < 0) max_frac = -max_frac;
	#endif

    switch (current_state)
    {
        case STATE_BOOST:
        	 LOG_INFO("STATE: BOOST | Alt: %d.%02d m\r\n", alt_int, alt_frac);
        	// Mach-lockout prevents Pressure drops from Triggering deployment
            if (HAL_GetTick() - boost_start_time > BOOST_PHASE_LOCKOUT_MS) {
            	LOG_INFO("--- Motor burnout assumed, transitioning to coast. ---\r\n");
                current_state = STATE_FLIGHT;
            }
            break;

        case STATE_FLIGHT:
        	// Another Mach-lockout defense: break the loop a value is >500m below the max
            if (current_altitude_m < max_altitude - SENSOR_OUTLIER_THRESHOLD_M) {
            	LOG_INFO("WARN: Outlier detected and ignored (%d.%02d m)\r\n", alt_int, alt_frac);
                break;
            }

            // If a new Maximum altitude is reached, set that as the max
            if (current_altitude_m > max_altitude) max_altitude = current_altitude_m;

            // Now we check a 1s moving average. If all of the values are below the maximum, fire the Drogue!
            if (buffer_full && !drogue_fired) {
                LOG_INFO("STATE: COAST | Alt: %d.%02d m | Avg: %d.%02d m | Max: %d.%02d m\r\n",
                       alt_int, alt_frac, avg_int, avg_frac, max_int, max_frac);
                if (moving_avg < (max_altitude - APOGEE_TIP_OVER_THRESHOLD_M)) {
                    fire_pyro_channel(1);
                    drogue_fired = true;
                    current_state = STATE_DROGUE_DESCENT;
                    LOG_INFO("--- APOGEE DETECTED, transitioning to drogue descent. ---\r\n");
                }
            else {
                    LOG_INFO("STATE: COAST | Alt: %d.%02d m | Max: %d.%02d m (Filling buffer)\r\n", alt_int, alt_frac, max_int, max_frac);
            	}
            }
            break;

        case STATE_DROGUE_DESCENT:
        	LOG_INFO("STATE: DROGUE | Alt: %d.%02d m | Avg: %d.%02d m\r\n", alt_int, alt_frac, avg_int, avg_frac);

        	// On the descent, we need to fire the main. Do this here
        	// Now we check a 1s moving average. If all of the values are below the Main Deployment Altitude, fire the Main!
             if (buffer_full && !main_fired) {
                 if (moving_avg < FEET_TO_METERS(MAIN_DEPLOYMENT_ALTITUDE_FT)) {
                     fire_pyro_channel(2);
                     main_fired = true;
                     current_state = STATE_LANDED;
                     last_beep_time = HAL_GetTick(); // Prime the timer for the first beep
                     LOG_INFO("--- MAIN DEPLOYMENT ALTITUDE REACHED, transitioning to landed. ---\r\n");
                 }
             }
             break;

        case STATE_LANDED:
        	LOG_INFO("STATE: LANDED | Max Alt: %d.%02d m | Waiting for beep timer...\r\n", max_int, max_frac);
            if (HAL_GetTick() - last_beep_time > LANDED_BEEP_PAUSE_MS) {
            	LOG_INFO("Beeping out max altitude: %d m\r\n", (int)max_altitude);
                buzz_out_value((int)max_altitude, 150); // Use 150ms buzzes
                last_beep_time = HAL_GetTick();
            }
            break;

        case STATE_ERROR:
        	Error_Handler();
            break;

        default:
            break;
    }
    HAL_Delay(100); // Main loop runs at ~10Hz
  }
}
