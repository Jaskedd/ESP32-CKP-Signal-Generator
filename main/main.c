#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <hal/ledc_ll.h>

#include "main.h"
#include "menu.h"

enum { NORMAL_TOOTH, MISSING_TOOTH };

typedef struct {
	gptimer_handle_t timer;
	gptimer_alarm_config_t alarmConfig;
	synchronism sync;
	portMUX_TYPE lock;
	volatile bool newRpmAvaiable;
	uint32_t activeStepUs[2];
	uint32_t activeLastStepUs[2];
	uint32_t pendingStepUs[2];
	uint32_t pendingLastStepUs[2];
	int toothIndex;
	int stepIndex;
	int realTeeth;
} signal_generator_t;

static const uint32_t ckpRamp[16] = {
	0, 17, 34, 51, 68, 85, 102, 119,
	136, 153, 170, 187, 204, 221, 238, 255
};

static signal_generator_t sg = {
	.lock = portMUX_INITIALIZER_UNLOCKED,
};

static void setDuty(ledc_channel_t ch, uint32_t duty);
static void setTiming(signal_generator_t* g, int rpm);
void signalGeneratorStart(const synchronism* sync, int rpm);

static bool IRAM_ATTR onSignalTimerAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* ctx) {
	signal_generator_t* g = (signal_generator_t*)ctx;

	const int type = (g->toothIndex < g->realTeeth) ? NORMAL_TOOTH : MISSING_TOOTH;
	const bool last = g->stepIndex == 15;

	if (g->stepIndex == 0 && g->newRpmAvaiable) {
		portENTER_CRITICAL_ISR(&g->lock);
		for (int i = 0; i < 2; i++) {
			g->activeStepUs[i] = g->pendingStepUs[i];
			g->activeLastStepUs[i] = g->pendingLastStepUs[i];
		}
		g->newRpmAvaiable = false;
		portEXIT_CRITICAL_ISR(&g->lock);
	}

	setDuty(CKP_POS_LEDC_CHANNEL, ckpRamp[g->stepIndex]);

	g->stepIndex++;

	if (g->stepIndex >= 16) {
		g->stepIndex = 0;
		g->toothIndex = (type == NORMAL_TOOTH) ? g->toothIndex + 1 : 0;
	}

	const uint32_t delay = last ? g->activeLastStepUs[type] : g->activeStepUs[type];
	g->alarmConfig.alarm_count = edata->alarm_value + delay;
	gptimer_set_alarm_action(timer, &g->alarmConfig);
	return false;
}

void signalGeneratorStart(const synchronism* sync, int rpm) {
	portENTER_CRITICAL(&sg.lock);
	sg.sync = *sync;
	sg.realTeeth = sg.sync.totalTeeth - sg.sync.totalMissingTeeth;
	sg.sync.totalMissingTeeth = sg.sync.totalMissingTeeth + 1;
	sg.toothIndex = 0;
	sg.stepIndex = 0;
	setTiming(&sg, rpm);
	for (int i = 0; i < 2; i++) {
		sg.activeStepUs[i] = sg.pendingStepUs[i];
		sg.activeLastStepUs[i] = sg.pendingLastStepUs[i];
	}
	sg.newRpmAvaiable = false;
	portEXIT_CRITICAL(&sg.lock);

	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);

	ESP_ERROR_CHECK(gptimer_set_raw_count(sg.timer, 0));
	sg.alarmConfig.alarm_count = 1;
	ESP_ERROR_CHECK(gptimer_set_alarm_action(sg.timer, &sg.alarmConfig));
	ESP_ERROR_CHECK(gptimer_start(sg.timer));
}

//Main app
void app_main(void) {
	const ledc_timer_config_t ledcTimer = {
		.speed_mode = LEDC_TIMER_0,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = LEDC_TIMER_8_BIT,
		.freq_hz = 312500,
		.clk_cfg = LEDC_USE_APB_CLK,
	};
	const ledc_channel_config_t ledcChannel = {
		.gpio_num = CKP_POS_GPIO,
		.speed_mode = LEDC_TIMER_0,
		.channel = CKP_POS_LEDC_CHANNEL,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledcTimer));
	ESP_ERROR_CHECK(ledc_channel_config(&ledcChannel));
	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);

	menuStart();

	const gptimer_config_t timerConfig = {
		.clk_src = GPTIMER_CLK_SRC_APB,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1000000,
		.intr_priority = 1,
	};
	const gptimer_event_callbacks_t timerCallbacks = {
		.on_alarm = onSignalTimerAlarm,
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &sg.timer));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(sg.timer, &timerCallbacks, &sg));
	sg.alarmConfig.reload_count = 0;
	sg.alarmConfig.flags.auto_reload_on_alarm = false;
	ESP_ERROR_CHECK(gptimer_enable(sg.timer));

	signalGeneratorStart(menuGetSelectedSynchronism(), menuGetRPM());
}

//Auxiliares
static void IRAM_ATTR setDuty(ledc_channel_t ch, uint32_t duty) {
	ledc_ll_set_duty_int_part(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, duty);
	ledc_ll_set_duty_direction(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, 1);
	ledc_ll_set_duty_num(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, 1);
	ledc_ll_set_duty_cycle(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, 1);
	ledc_ll_set_duty_scale(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, 0);
	ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, true);
	ledc_ll_set_duty_start(LEDC_LL_GET_HW(), LEDC_HIGH_SPEED_MODE, ch, true);
}

static void setTiming(signal_generator_t* g, int rpm) {
	const uint32_t normalUs = 60000000 / ((rpm < 1 ? 1 : rpm) * g->sync.totalTeeth);
	const uint32_t times[2] = { normalUs, normalUs * g->sync.totalMissingTeeth };
	for (int i = 0; i < 2; i++) {
		g->pendingStepUs[i] = times[i] / 16;
		g->pendingLastStepUs[i] = times[i] - (g->pendingStepUs[i] * 15);
		if (g->pendingStepUs[i] == 0) g->pendingStepUs[i] = 1;
		if (g->pendingLastStepUs[i] == 0) g->pendingLastStepUs[i] = 1;
	}
}

void setRpm(int rpm) {
	portENTER_CRITICAL(&sg.lock);
	if (sg.sync.totalTeeth != 0) {
		setTiming(&sg, rpm);
		sg.newRpmAvaiable = true;
	}
	portEXIT_CRITICAL(&sg.lock);
}
