#include "menu.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "HD44780.h"

static const gpio_num_t lcdSdaGpio = GPIO_NUM_21;
static const gpio_num_t lcdSclGpio = GPIO_NUM_22;
static const gpio_num_t buttonUpGpio = GPIO_NUM_34;
static const gpio_num_t buttonDownGpio = GPIO_NUM_36;
static const gpio_num_t buttonConfirmGpio = GPIO_NUM_39;
static const adc_channel_t rpmAdcChannel = ADC_CHANNEL_0;

static const synchronism syncTable[] = {
	{"VW 60-2", 60, 2, {14, 19, 27, 49, 57, 79, 104, 110}, 8},
	{"Fire 60-2", 60, 2, {8, 30, 38, 59, 68, 99}, 6},
	// Zetec 1.8 16V early/non-VCT: 36-1 crank with a single cam sync pulse.
	{"Zetec 36-1", 36, 1, {34, 35}, 2},
	{"Sync Test", 100, 4, {50, 51}, 2},
	{"", 0, 0, {0}, 0},
};

static int selectedSyncIndex = 2;
static int currentRPM = 600;
static bool generatingSignal = false;

static void updateRPM(void* pvParameter);
static void displayRPM(void* pvParameter);
static void checkForRestart(void* pvParameter);

const synchronism* menuGetSelectedSynchronism(void) {
	return &syncTable[selectedSyncIndex];
}

int menuGetRPM(void) {
	return currentRPM;
}

bool menuIsGeneratingSignal(void) {
	return generatingSignal;
}

void menuStart(void) {
	int selectedIndex = 0;
	int syncCount = 0;
	bool menuActive = true;

	gpio_set_direction(buttonUpGpio, GPIO_MODE_INPUT);
	gpio_set_direction(buttonDownGpio, GPIO_MODE_INPUT);
	gpio_set_direction(buttonConfirmGpio, GPIO_MODE_INPUT);

	LCD_init(0x3F, lcdSdaGpio, lcdSclGpio, 16, 2);
	LCD_clearScreen();

	while (syncTable[syncCount].totalTeeth != 0) {
		syncCount++;
	}

	LCD_setCursor(0, 0);
	LCD_writeStr("Select:");
	LCD_setCursor(0, 1);
	LCD_writeChar('>');

	while (menuActive) {
		char paddedText[16];

		LCD_setCursor(1, 1);
		snprintf(paddedText, sizeof(paddedText), "%-15s", syncTable[selectedIndex].syncName);
		LCD_writeStr(paddedText);

		vTaskDelay(pdMS_TO_TICKS(100));

		if (!gpio_get_level(buttonUpGpio)) {
			printf("Up button pressed\n");
			selectedIndex = (selectedIndex - 1 + syncCount) % syncCount;
			vTaskDelay(pdMS_TO_TICKS(250));
		}

		if (!gpio_get_level(buttonDownGpio)) {
			printf("Down button pressed\n");
			selectedIndex = (selectedIndex + 1) % syncCount;
			vTaskDelay(pdMS_TO_TICKS(250));
		}

		if (!gpio_get_level(buttonConfirmGpio)) {
			printf("Confirm button pressed\n");
			selectedSyncIndex = selectedIndex;
			menuActive = false;
			vTaskDelay(pdMS_TO_TICKS(250));
		}
	}

	LCD_clearScreen();
	LCD_setCursor(0, 1);
	LCD_writeStr(syncTable[selectedSyncIndex].syncName);
	LCD_setCursor(0, 0);
	LCD_writeStr("Change made");

	for (int i = 0; i < 3; i++) {
		vTaskDelay(pdMS_TO_TICKS(500));
		LCD_writeChar('.');
	}

	vTaskDelay(pdMS_TO_TICKS(2000));
	LCD_clearScreen();

	generatingSignal = true;

	xTaskCreatePinnedToCore(updateRPM, "updateRPM", 1024, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(displayRPM, "displayRPM", 2048, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(checkForRestart, "checkForRestart", 2048, NULL, 5, NULL, 1);
}

static void updateRPM(void* pvParameter) {
	int rpmPotValue = 0;
	adc_oneshot_unit_handle_t rpmPotHandle;
	const adc_oneshot_unit_init_cfg_t rpmPotInitConfig = {.unit_id = ADC_UNIT_2};
	const adc_oneshot_chan_cfg_t rpmPotChanConfig = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_12
	};

	printf("UpdateRPM started\n");
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&rpmPotInitConfig, &rpmPotHandle));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(rpmPotHandle, rpmAdcChannel, &rpmPotChanConfig));

	while (generatingSignal) {
		ESP_ERROR_CHECK(adc_oneshot_read(rpmPotHandle, rpmAdcChannel, &rpmPotValue));
		currentRPM = (rpmPotValue * (10000 - 600) / 4095) + 600;
		vTaskDelay(pdMS_TO_TICKS(300));
	}

	printf("updateRPM task ending\n");
	vTaskDelete(NULL);
}

static void displayRPM(void* pvParameter) {
	char displayMessage[16];

	printf("display RPM Started\n");

	while (generatingSignal) {
		LCD_home();
		snprintf(displayMessage, sizeof(displayMessage), "RPM: %d       ", currentRPM);
		LCD_writeStr(displayMessage);
		vTaskDelay(pdMS_TO_TICKS(300));
	}

	printf("displayRPM task ending\n");
	vTaskDelete(NULL);
}

static void checkForRestart(void* pvParameter) {
	printf("Restart check task started\n");

	while (generatingSignal) {
		if (!gpio_get_level(buttonConfirmGpio) || !gpio_get_level(buttonUpGpio) || !gpio_get_level(buttonDownGpio)) {
			printf("Button pressed, restarting device...\n");
			generatingSignal = false;
			vTaskDelay(pdMS_TO_TICKS(250));
			printf("Restarting device...\n");
			esp_restart();
		}

		vTaskDelay(pdMS_TO_TICKS(250));
	}
}
