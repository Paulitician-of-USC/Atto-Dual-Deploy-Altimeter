/*
 * buzzer.h
 *
 *  Created on: Aug 24, 2025
 *      Author: pauli
 */

#ifndef SRC_BUZZER_H_
#define SRC_BUZZER_H_

void Buzzer_Start(uint32_t duration_ms);
void Buzzer_Update(void);
void buzz_out_value(int value, uint32_t digit_buzz_duration_ms);
void buzz_out_altitude(float altitude, uint32_t digit_buzz_duration_ms);

#endif /* SRC_BUZZER_H_ */
