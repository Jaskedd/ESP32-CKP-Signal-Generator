#ifndef MAIN_H
#define MAIN_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_err.h>

#define CKP_POS_LEDC_CHANNEL LEDC_CHANNEL_0
#define CKP_POS_GPIO GPIO_NUM_25
#define OUTPUT_DUTY_MID 127
#define OUTPUT_DUTY_OFF 0

#define SET_OUTPUT_DUTY(channel, duty) \
	do { \
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty)); \
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel)); \
	} while (0)

typedef struct {
	char syncName[15];
	int totalTeeth;
	int totalMissingTeeth;
} synchronism;

void signalGeneratorStart(const synchronism* sync, int rpm);
void setRpm(int rpm);
void signalGeneratorStop(void);

#endif // MAIN_H