#ifndef MAIN_H
#define MAIN_H

#include <driver/ledc.h>
#include <esp_err.h>

#define CKP_POS_GPIO GPIO_NUM_25
#define CMP_GPIO GPIO_NUM_13

#define OUTPUT_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define OUTPUT_LEDC_TIMER LEDC_TIMER_0
#define OUTPUT_LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define OUTPUT_LEDC_FREQUENCY_HZ 312500
#define CKP_POS_LEDC_CHANNEL LEDC_CHANNEL_0
#define CMP_LEDC_CHANNEL LEDC_CHANNEL_1
#define OUTPUT_DUTY_MID 127
#define OUTPUT_DUTY_OFF 0
#define OUTPUT_DUTY_ON 255

#define SET_OUTPUT_DUTY(channel, duty) \
	do { \
		ESP_ERROR_CHECK(ledc_set_duty(OUTPUT_LEDC_MODE, channel, duty)); \
		ESP_ERROR_CHECK(ledc_update_duty(OUTPUT_LEDC_MODE, channel)); \
	} while (0)

typedef struct {
	char syncName[15];
	int totalTeeth;
	int totalMissingTeeth;
	int cmpTeeth[10];
	int cmpCount;
} synchronism;

#endif // MAIN_H
