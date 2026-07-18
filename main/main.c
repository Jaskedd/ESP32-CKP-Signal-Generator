#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <hal/ledc_ll.h>

#include "main.h"
#include "menu.h"

enum {
	NORMAL_TOOTH,  /* Diente presente en la rueda fonica */
	MISSING_TOOTH, /* Espacio sin diente (gap de sincronismo) */
};

/* Tiempos de un diente: cada micro-paso tiene una duracion fija y
 * lastStepUs absorbe el resto de redondeo para que la suma total sea
 * exactamente la duracion del diente. */
typedef struct {
	uint32_t stepUs;
	uint32_t lastStepUs;
} tooth_timing_t;

/* Estado global del generador de senial CKP.
 * Se usa un esquema de doble buffer (activeTiming / pendingTiming) para que
 * los cambios de RPM se apliquen siempre en el limite entre dientes y nunca
 * a mitad de un diente, evitando glitches en la forma de onda. */
typedef struct {
	gptimer_handle_t timer;          /* Temporizador hardware que gobierna la ISR */
	gptimer_alarm_config_t alarmConfig;
	synchronism sync;                /* Patron de sincronismo (dientes totales y faltantes) */
	portMUX_TYPE lock;               /* Proteccion de seccion critica entre tarea e ISR */
	volatile bool running;           /* true mientras el generador esta activo */

	/* La tarea de menu escribe en pendingTiming y marca newRpmAvaiable;
	 * la ISR copia pendingTiming -> activeTiming al comenzar un diente nuevo. */
	volatile bool newRpmAvaiable;
	tooth_timing_t activeTiming[2];  /* Tiempos activos en la ISR */
	tooth_timing_t pendingTiming[2]; /* Tiempos pendientes de aplicar */

	int realTeeth;   /* Dientes reales por revolucion (totales - faltantes) */
	int toothIndex;  /* Diente actual (0..realTeeth-1 = normales, luego MISSING_TOOTH) */
	int microStepIndex;   /* Micro-paso actual dentro del diente (0..CKP_RAMP_STEPS-1) */

	uint32_t stepUs;
	uint32_t lastStepUs;
} signal_generator_t;

#define CKP_RAMP_STEPS 16
/* Tabla de valores PWM para la rampa de subida. Simula inductivo/Hall */
static const uint32_t ckpRamp[CKP_RAMP_STEPS] = {
	0, 17, 34, 51,
	68, 85, 102, 119,
	136, 153, 170, 187,
	204, 221, 238, 255
};

static signal_generator_t signalGenerator = {
	.lock = portMUX_INITIALIZER_UNLOCKED,
};

static void initHW(void);
static void calculateStep(signal_generator_t* generator, int rpm);
static void calculateMicrostep(tooth_timing_t* timing, uint32_t toothTimeUs);
static void setOutputDutyRaw(ledc_channel_t channel, uint32_t duty);
static bool microStepAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

void app_main(void) {
	initHW();
	menuBegin();
	signalGeneratorStart(getTypeFromMenu(), getRpmFromMenu());

	while (1) {

	}
}

/* Inicia la generacion de la señal CKP */
void signalGeneratorStart(const synchronism* sync, int rpm) {
	portENTER_CRITICAL(&signalGenerator.lock);
	signalGenerator.sync = *sync;
	signalGenerator.sync.totalMissingTeeth = signalGenerator.sync.totalMissingTeeth + 1;
	signalGenerator.realTeeth = signalGenerator.sync.totalTeeth - signalGenerator.sync.totalMissingTeeth;
	signalGenerator.toothIndex = 0;
	signalGenerator.microStepIndex = 0;
	signalGenerator.running = true;

	calculateStep(&signalGenerator, rpm);
	signalGenerator.activeTiming[NORMAL_TOOTH] = signalGenerator.pendingTiming[NORMAL_TOOTH];
	signalGenerator.activeTiming[MISSING_TOOTH] = signalGenerator.pendingTiming[MISSING_TOOTH];
	signalGenerator.newRpmAvaiable = false;
	portEXIT_CRITICAL(&signalGenerator.lock);

	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);

	ESP_ERROR_CHECK(gptimer_set_raw_count(signalGenerator.timer, 0));
	signalGenerator.alarmConfig.alarm_count = 1;
	ESP_ERROR_CHECK(gptimer_set_alarm_action(signalGenerator.timer, &signalGenerator.alarmConfig));
	ESP_ERROR_CHECK(gptimer_start(signalGenerator.timer));
}

/* ISR del temporizador: se dispara una vez por micro-paso de rampa
 * (CKP_RAMP_STEPS veces por diente). En cada llamada:
 * 1. Copia pendingTiming -> activeTiming si hay un cambio de RPM pendiente
 *    y estamos al inicio de un diente.
 * 2. Escribe el valor PWM de la rampa en el registro del LEDC.
 * 3. Se rearma a si misma con el retardo del siguiente micro-paso.*/
static bool IRAM_ATTR microStepAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
	signal_generator_t* generator = (signal_generator_t*)user_ctx;
	const bool toothType = (generator->toothIndex < generator->realTeeth) ? NORMAL_TOOTH : MISSING_TOOTH;

	/* Actualización de rpm. Solo al inicio de un diente para evitar distorsionar la forma de onda. */
	if (generator->microStepIndex == 0 && generator->newRpmAvaiable) {
		portENTER_CRITICAL_ISR(&generator->lock);
		generator->activeTiming[NORMAL_TOOTH] = generator->pendingTiming[NORMAL_TOOTH];
		generator->activeTiming[MISSING_TOOTH] = generator->pendingTiming[MISSING_TOOTH];
		generator->newRpmAvaiable = false;
		portEXIT_CRITICAL_ISR(&generator->lock);
	}

	/* Escribe el valor PWM correspondiente a este micro-paso de la rampa */
	setOutputDutyRaw(CKP_POS_LEDC_CHANNEL, ckpRamp[generator->microStepIndex]);

	/* Avanza al siguiente micro-paso / al siguiente diente / reinicia el ciclo */
	generator->microStepIndex++;
	if (generator->microStepIndex >= CKP_RAMP_STEPS) {
		generator->microStepIndex = 0;
		generator->toothIndex = (toothType == NORMAL_TOOTH) ? generator->toothIndex + 1 : 0;
	}

	/* Rearma el temporizador para el proximo micro-paso */
	const bool isLastStep = generator->microStepIndex == CKP_RAMP_STEPS - 1;
	generator->alarmConfig.alarm_count = edata->alarm_value + isLastStep ? generator->activeTiming[toothType].lastStepUs : generator->activeTiming[toothType].stepUs;
	gptimer_set_alarm_action(timer, &generator->alarmConfig);
	return false;
}

/* Config LEDC y GPTimer*/
static void initHW(void) {
	const ledc_timer_config_t ledcTimer = {
		.speed_mode = OUTPUT_LEDC_MODE,
		.timer_num = OUTPUT_LEDC_TIMER,
		.duty_resolution = OUTPUT_LEDC_RESOLUTION,
		.freq_hz = OUTPUT_LEDC_FREQUENCY_HZ,
		.clk_cfg = LEDC_USE_APB_CLK,
	};
	const ledc_channel_config_t ledcChannel = {
		.gpio_num = CKP_POS_GPIO,
		.speed_mode = OUTPUT_LEDC_MODE,
		.channel = CKP_POS_LEDC_CHANNEL,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = OUTPUT_LEDC_TIMER,
		.duty = OUTPUT_DUTY_OFF,
		.hpoint = 0,
	};

	ESP_ERROR_CHECK(ledc_timer_config(&ledcTimer));
	ESP_ERROR_CHECK(ledc_channel_config(&ledcChannel));
	SET_OUTPUT_DUTY(CKP_POS_LEDC_CHANNEL, OUTPUT_DUTY_MID);

	const gptimer_config_t timerConfig = {
		.clk_src = GPTIMER_CLK_SRC_APB,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1000000,
		.intr_priority = 1,
	};
	const gptimer_event_callbacks_t timerCallbacks = {
		.on_alarm = microStepAlarm,
	};

	ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &signalGenerator.timer));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(signalGenerator.timer, &timerCallbacks, &signalGenerator));

	signalGenerator.alarmConfig.reload_count = 0;
	signalGenerator.alarmConfig.flags.auto_reload_on_alarm = false;

	ESP_ERROR_CHECK(gptimer_enable(signalGenerator.timer));
}

/* Funciones auxiliares */
static void calculateStep(signal_generator_t* generator, int rpm) {
	/* Tiempo por diente en microsegundos = 60 s / (RPM x dientes totales) */
	const uint32_t normalToothTimeUs = 60000000 / ((rpm < 1 ? 1 : rpm) * generator->sync.totalTeeth);

	calculateMicrostep(&generator->pendingTiming[NORMAL_TOOTH], 60000000 / ((rpm < 1 ? 1 : rpm) * generator->sync.totalTeeth));
	calculateMicrostep(&generator->pendingTiming[MISSING_TOOTH], normalToothTimeUs * generator->sync.totalMissingTeeth);
}

static void calculateMicrostep(tooth_timing_t* timing, uint32_t toothTimeUs) {
	timing->stepUs = toothTimeUs / CKP_RAMP_STEPS;
	timing->lastStepUs = toothTimeUs - (timing->stepUs * (CKP_RAMP_STEPS - 1));

	if (timing->stepUs == 0) {
		timing->stepUs = 1;
	}

	if (timing->lastStepUs == 0) {
		timing->lastStepUs = 1;
	}
}

static void setOutputDutyRaw(ledc_channel_t channel, uint32_t duty) {
	ledc_ll_set_duty_int_part(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, duty);
	ledc_ll_set_duty_direction(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_num(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_cycle(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 1);
	ledc_ll_set_duty_scale(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, 0);
	ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, true);
	ledc_ll_set_duty_start(LEDC_LL_GET_HW(), OUTPUT_LEDC_MODE, channel, true);
}

void signalGeneratorSetRPM(int rpm) {
	portENTER_CRITICAL(&signalGenerator.lock);
	if (signalGenerator.sync.totalTeeth != 0) {
		calculateStep(&signalGenerator, rpm);
		signalGenerator.newRpmAvaiable = true;
	}
	portEXIT_CRITICAL(&signalGenerator.lock);
}
