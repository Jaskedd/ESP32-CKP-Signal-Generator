#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>
#include <stdio.h>

#include "main.h"
#include "menu.h"

#define CKP_RAMP_STEPS 16

static const int ckpRamp[CKP_RAMP_STEPS] = {
	0, 17, 34, 51,
	68, 85, 102, 119,
	136, 153, 170, 187,
	204, 221, 238, 255
};

static void generateSignal(void* pvParameter);

void app_main(void) {
	const ledc_timer_config_t ledcTimer = {
		.speed_mode = OUTPUT_LEDC_MODE,
		.timer_num = OUTPUT_LEDC_TIMER,
		.duty_resolution = OUTPUT_LEDC_RESOLUTION,
		.freq_hz = OUTPUT_LEDC_FREQUENCY_HZ,
		.clk_cfg = LEDC_USE_APB_CLK,
	};

	const ledc_channel_config_t ledcChannels[] = {
		{
			.gpio_num = CKP_POS_GPIO,
			.speed_mode = OUTPUT_LEDC_MODE,
			.channel = CKP_POS_LEDC_CHANNEL,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = OUTPUT_LEDC_TIMER,
			.duty = OUTPUT_DUTY_OFF,
			.hpoint = 0,
		},
		{
			.gpio_num = CMP_GPIO,
			.speed_mode = OUTPUT_LEDC_MODE,
			.channel = CMP_LEDC_CHANNEL,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = OUTPUT_LEDC_TIMER,
			.duty = OUTPUT_DUTY_OFF,
			.hpoint = 0,
		},
	};

	ESP_ERROR_CHECK(ledc_timer_config(&ledcTimer));

	for (int i = 0; i < sizeof(ledcChannels) / sizeof(ledcChannels[0]); i++) {
		ESP_ERROR_CHECK(ledc_channel_config(&ledcChannels[i]));
	}

	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);
	SET_OUTPUT_DUTY(CMP_LEDC_CHANNEL, OUTPUT_DUTY_OFF);

	menuStart();

	xTaskCreatePinnedToCore(generateSignal, "generateSignal", 2048, NULL, 5, NULL, 0);
}

static void generateSignal(void* pvParameter) {
	const synchronism sync = *menuGetSelectedSynchronism();
	const int realTeeth = sync.totalTeeth - sync.totalMissingTeeth;
	int currentTooth = 0;
	int cmpState = 0;

	printf("Started signal\n");

	while (menuIsGeneratingSignal()) {
		const int rpm = menuGetRPM();
		const int period = 60000000 / (rpm * sync.totalTeeth);

		for (int toothIndex = 0; toothIndex < sync.totalTeeth; toothIndex++) {
			const int totalToothTime = (toothIndex < realTeeth) ? period : period * 2;
			const int stepTime = totalToothTime / CKP_RAMP_STEPS;
			int remainingTime = totalToothTime;

			currentTooth++;

			for (int i = 0; i < sync.cmpCount; i++) {
				if (currentTooth == sync.cmpTeeth[i]) {
					cmpState = !cmpState;
					SET_OUTPUT_DUTY(CMP_LEDC_CHANNEL, cmpState ? OUTPUT_DUTY_ON : OUTPUT_DUTY_OFF);
				}
			}

			for (int step = 0; step < CKP_RAMP_STEPS; step++) {
				const int currentStepTime = (step == CKP_RAMP_STEPS - 1) ? remainingTime : stepTime;
				SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, ckpRamp[step]);
				ets_delay_us(currentStepTime);
				remainingTime -= currentStepTime;
			}

			if (currentTooth >= sync.totalTeeth * 2) {
				currentTooth = 0;
			}
		}
	}

	printf("generateSignal task ending\n");
	vTaskDelete(NULL);
}
