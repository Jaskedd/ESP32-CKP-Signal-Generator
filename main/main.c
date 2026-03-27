#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <hal/ledc_ll.h>

#include "main.h"
#include "menu.h"

#define CKP_RAMP_STEPS 16
#define TIMER_RESOLUTION_HZ 1000000

enum {
	NORMAL_TOOTH,
	MISSING_TOOTH,
};

typedef struct {
	uint32_t stepUs;
	uint32_t lastStepUs;
} tooth_timing_t;

typedef struct {
	gptimer_handle_t timer;
	gptimer_alarm_config_t alarmConfig;
	synchronism sync;
	portMUX_TYPE lock;
	volatile bool running;
	volatile bool timingDirty;
	tooth_timing_t activeTiming[2];
	tooth_timing_t pendingTiming[2];
	int realTeeth;
	int toothIndex;
	int stepIndex;
	int currentTooth;
	int cmpState;
} signal_generator_t;

static const uint32_t ckpRamp[CKP_RAMP_STEPS] = {
	0, 17, 34, 51,
	68, 85, 102, 119,
	136, 153, 170, 187,
	204, 221, 238, 255
};

static signal_generator_t signalGenerator = {
	.lock = portMUX_INITIALIZER_UNLOCKED,
};

static void configureOutputSignal(void);
static void configureSignalTimer(void);
static void resetOutputs(void);
static void setTimingForRPM(signal_generator_t* generator, int rpm);
static void setToothTiming(tooth_timing_t* timing, uint32_t toothTimeUs);
static void IRAM_ATTR setOutputDutyRaw(ledc_channel_t channel, uint32_t duty);
static bool IRAM_ATTR onSignalTimerAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

void app_main(void) {
	configureOutputSignal();
	menuStart();
	configureSignalTimer();
	signalGeneratorStart(menuGetSelectedSynchronism(), menuGetRPM());
}

void signalGeneratorStart(const synchronism* sync, int rpm) {
	portENTER_CRITICAL(&signalGenerator.lock);
	signalGenerator.sync = *sync;
	signalGenerator.realTeeth = sync->totalTeeth - sync->totalMissingTeeth;
	signalGenerator.toothIndex = 0;
	signalGenerator.stepIndex = 0;
	signalGenerator.currentTooth = 0;
	signalGenerator.cmpState = 0;
	signalGenerator.running = true;
	setTimingForRPM(&signalGenerator, rpm);
	signalGenerator.activeTiming[NORMAL_TOOTH] = signalGenerator.pendingTiming[NORMAL_TOOTH];
	signalGenerator.activeTiming[MISSING_TOOTH] = signalGenerator.pendingTiming[MISSING_TOOTH];
	signalGenerator.timingDirty = false;
	portEXIT_CRITICAL(&signalGenerator.lock);

	resetOutputs();

	ESP_ERROR_CHECK(gptimer_set_raw_count(signalGenerator.timer, 0));
	signalGenerator.alarmConfig.alarm_count = 1;
	ESP_ERROR_CHECK(gptimer_set_alarm_action(signalGenerator.timer, &signalGenerator.alarmConfig));
	ESP_ERROR_CHECK(gptimer_start(signalGenerator.timer));
}

void signalGeneratorSetRPM(int rpm) {
	portENTER_CRITICAL(&signalGenerator.lock);
	if (signalGenerator.sync.totalTeeth != 0) {
		setTimingForRPM(&signalGenerator, rpm);
		signalGenerator.timingDirty = true;
	}
	portEXIT_CRITICAL(&signalGenerator.lock);
}

void signalGeneratorStop(void) {
	bool wasRunning;

	portENTER_CRITICAL(&signalGenerator.lock);
	wasRunning = signalGenerator.running;
	signalGenerator.running = false;
	portEXIT_CRITICAL(&signalGenerator.lock);

	if (signalGenerator.timer != NULL && wasRunning) {
		gptimer_stop(signalGenerator.timer);
	}

	resetOutputs();
}

static void configureOutputSignal(void) {
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

	resetOutputs();
}

static void configureSignalTimer(void) {
	const gptimer_config_t timerConfig = {
		.clk_src = GPTIMER_CLK_SRC_APB,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = TIMER_RESOLUTION_HZ,
		.intr_priority = 1,
	};
	const gptimer_event_callbacks_t timerCallbacks = {
		.on_alarm = onSignalTimerAlarm,
	};

	ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &signalGenerator.timer));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(signalGenerator.timer, &timerCallbacks, &signalGenerator));

	signalGenerator.alarmConfig.reload_count = 0;
	signalGenerator.alarmConfig.flags.auto_reload_on_alarm = false;

	ESP_ERROR_CHECK(gptimer_enable(signalGenerator.timer));
}

static void resetOutputs(void) {
	SET_OUTPUT_DUTY(CMP_LEDC_CHANNEL, OUTPUT_DUTY_OFF);
	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);
}

static void setTimingForRPM(signal_generator_t* generator, int rpm) {
	const uint32_t normalToothTimeUs = 60000000 / ((rpm < 1 ? 1 : rpm) * generator->sync.totalTeeth);
	const uint32_t missingToothTimeUs = normalToothTimeUs * generator->sync.totalMissingTeeth;

	setToothTiming(&generator->pendingTiming[NORMAL_TOOTH], normalToothTimeUs);
	setToothTiming(&generator->pendingTiming[MISSING_TOOTH], missingToothTimeUs);
}

static void setToothTiming(tooth_timing_t* timing, uint32_t toothTimeUs) {
	timing->stepUs = toothTimeUs / CKP_RAMP_STEPS;
	timing->lastStepUs = toothTimeUs - (timing->stepUs * (CKP_RAMP_STEPS - 1));

	if (timing->stepUs == 0) {
		timing->stepUs = 1;
	}

	if (timing->lastStepUs == 0) {
		timing->lastStepUs = 1;
	}
}

static void IRAM_ATTR setOutputDutyRaw(ledc_channel_t channel, uint32_t duty) {
	ledc_ll_set_duty_int_part(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, duty);
	ledc_ll_set_duty_direction(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_num(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_cycle(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_scale(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 0);
	ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, true);
	ledc_ll_set_duty_start(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, true);
}

static bool IRAM_ATTR onSignalTimerAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
	signal_generator_t* generator = (signal_generator_t*)user_ctx;
	const int toothType = (generator->toothIndex < generator->realTeeth) ? NORMAL_TOOTH : MISSING_TOOTH;
	const bool isLastStep = generator->stepIndex == CKP_RAMP_STEPS - 1;
	uint32_t stepDelayUs;

	if (!generator->running) {
		return false;
	}

	if (generator->stepIndex == 0) {
		if (generator->timingDirty) {
			portENTER_CRITICAL_ISR(&generator->lock);
			generator->activeTiming[NORMAL_TOOTH] = generator->pendingTiming[NORMAL_TOOTH];
			generator->activeTiming[MISSING_TOOTH] = generator->pendingTiming[MISSING_TOOTH];
			generator->timingDirty = false;
			portEXIT_CRITICAL_ISR(&generator->lock);
		}

		if (toothType == NORMAL_TOOTH) {
			generator->currentTooth++;
		} else {
			generator->currentTooth += generator->sync.totalMissingTeeth;
		}

		for (int i = 0; i < generator->sync.cmpCount; i++) {
			if (generator->currentTooth == generator->sync.cmpTeeth[i]) {
				generator->cmpState = !generator->cmpState;
				setOutputDutyRaw(CMP_LEDC_CHANNEL, generator->cmpState ? OUTPUT_DUTY_ON : OUTPUT_DUTY_OFF);
			}
		}
	}

	setOutputDutyRaw(CKP_POS_LEDC_CHANNEL, ckpRamp[generator->stepIndex]);
	stepDelayUs = isLastStep ? generator->activeTiming[toothType].lastStepUs : generator->activeTiming[toothType].stepUs;

	generator->stepIndex++;

	if (generator->stepIndex >= CKP_RAMP_STEPS) {
		generator->stepIndex = 0;
		if (toothType == NORMAL_TOOTH) {
			generator->toothIndex++;
		} else {
			generator->toothIndex = 0;
		}

		if (generator->currentTooth >= generator->sync.totalTeeth * 2) {
			generator->currentTooth = 0;
		}
	}

	generator->alarmConfig.alarm_count = edata->alarm_value + stepDelayUs;
	gptimer_set_alarm_action(timer, &generator->alarmConfig);
	return false;
}
