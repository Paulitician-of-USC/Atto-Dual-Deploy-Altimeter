/*
 * buzzer.c
 *
 *  Created on: Aug 24, 2025
 *      Author: pauli
 */


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h" // Change this to your specific STM32 series header if needed
#include "buzzer.h"
#

extern TIM_HandleTypeDef htim3;


// --- Buzzer Control Variables ---
volatile uint8_t buzzer_active = 0;
volatile uint32_t buzzer_start_time = 0;
volatile uint32_t buzzer_duration_ms = 0;

// --- Buzzer Functions ---
void Buzzer_Start(uint32_t duration_ms) {
    if (!buzzer_active) {
        buzzer_active = 1;
        buzzer_start_time = HAL_GetTick();
        buzzer_duration_ms = duration_ms;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    }
}

void Buzzer_Update() {
    if (buzzer_active) {
        if ((HAL_GetTick() - buzzer_start_time) >= buzzer_duration_ms) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            buzzer_active = 0;
        }
    }
}


void buzz_out_value(int value, uint32_t digit_buzz_duration_ms) {
    // This is a blocking function used for recovery, so HAL_Delay is acceptable.
    const uint32_t INTER_BUZZ_PAUSE_MS = digit_buzz_duration_ms * 2;
    const uint32_t INTER_DIGIT_PAUSE_MS = digit_buzz_duration_ms * 4;
    const uint32_t ZERO_BUZZ_DURATION_MS = digit_buzz_duration_ms * 3;

    if (value < 0) value = 0; // Don't buzz negative numbers
    if (value == 0) {
        Buzzer_Start(ZERO_BUZZ_DURATION_MS); HAL_Delay(ZERO_BUZZ_DURATION_MS);
        return;
    }

    int power_of_10 = 1;
    while (power_of_10 * 10 <= value) {
        power_of_10 *= 10;
    }

    while (power_of_10 > 0) {
        int digit = value / power_of_10;
        if (digit == 0) {
            Buzzer_Start(ZERO_BUZZ_DURATION_MS); HAL_Delay(ZERO_BUZZ_DURATION_MS);
        } else {
            for (int i = 0; i < digit; i++) {
                Buzzer_Start(digit_buzz_duration_ms); HAL_Delay(digit_buzz_duration_ms);
                Buzzer_Update();
                HAL_Delay(INTER_BUZZ_PAUSE_MS);
            }
        }
        value %= power_of_10;
        power_of_10 /= 10;
        HAL_Delay(INTER_DIGIT_PAUSE_MS);
    }
}

/**
 * @brief Outputs an altitude value by buzzing out each digit.
 * @param altitude The altitude value to output.
 * @param digit_buzz_duration_ms The duration for a single short buzz.
 */
void buzz_out_altitude(float altitude, uint32_t digit_buzz_duration_ms)
{
    const uint32_t INTER_BUZZ_PAUSE_MS = digit_buzz_duration_ms * 2;
    const uint32_t INTER_DIGIT_PAUSE_MS = digit_buzz_duration_ms * 4;
    const uint32_t ZERO_BUZZ_DURATION_MS = digit_buzz_duration_ms * 3;

    // Handle negative altitudes with a long preliminary buzz
    if (altitude < 0) {
        altitude = -altitude;
    }

    int alt_int = (int) altitude;

    // Handle the case of zero altitude
    if (alt_int == 0) {
        Buzzer_Start(ZERO_BUZZ_DURATION_MS);
        HAL_Delay(ZERO_BUZZ_DURATION_MS);
        return;
    }

    // Find the highest power of 10 less than or equal to the altitude
    int power_of_10 = 1;
    while (power_of_10 * 10 <= alt_int) {
        power_of_10 *= 10;
    }

    // Buzz out each digit from left to right
    while (power_of_10 > 0) {
        int digit = alt_int / power_of_10;

        if (digit == 0) {
            Buzzer_Start(ZERO_BUZZ_DURATION_MS);
            HAL_Delay(ZERO_BUZZ_DURATION_MS);
        } else {
            for (int i = 0; i < digit; i++) {
                Buzzer_Start(digit_buzz_duration_ms);
                HAL_Delay(digit_buzz_duration_ms);
                Buzzer_Update(); // Ensure buzzer stops
                HAL_Delay(INTER_BUZZ_PAUSE_MS);
            }
        }

        alt_int %= power_of_10;
        power_of_10 /= 10;

        // Pause between digits
        HAL_Delay(INTER_DIGIT_PAUSE_MS);
    }
}
