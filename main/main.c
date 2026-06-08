// THIS VERSION OF CODE IS SPECIFICALLY FOR UF PLANT IN VIZAG, GO THROUGH THE OPERATIONS COMMENTS AND PRIORITY TASKS COMMENTS TO UNDERSTAND
// POSTING ON OUR SERVER CURRENTLY
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_console.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "BLE.h"
#include "sensor_config.h"
#include "logs_handler.h"
#include "GSM_handler.h"  
#include "UART_HANDLER.h"
// #include "MODBUS_HANDLER.h"
#include "modbus_esp.h"
#include "RTC_handler.h"
#include "ds18x20_handler.h"
#include "GSM_handler.h"
#include "ota_handler.h"
#include "main.h"
#include "support.h"
#include <time.h>

#include "driver/periph_ctrl.h"
#include "esp_private/periph_ctrl.h"
#include "soc/soc.h"

#include "wifi_handler.h"

/*	Select anyone of the following */
/*	DEFAULT    LOG_AT_UART1		LOG_AT_UART2		DISABLE_LOGS */

#define DEBUG LOG_AT_UART2

/*								  */

/*
#include "uart.h"
#include "config_store.h"
#include "log_manager.h"
#include "gpio_manager.h"
#include "gsm_handler.h"
#include "rtc_sync.h"
*/

// 5 hours * 3600s + 30 minutes * 60s = 19800 seconds
#define IST_OFFSET_SEC (19800LL)
/*
SPIFFS Size
Flash end = 0x400000

Storage start = 0x110000

Size = 0x400000 – 0x110000 = 0x2F0000 (~2.94 MB).
*/

char all_sensor_info[4096] = {0};  // Adjust size as needed
char device_config_str[512] = {0}; // Adjust size as needed
char device_status_str[512] = {0}; // Declare this globally or adjust as needed

/* DigitalInputs Variables */
uint8_t raw_tank_high = 0;
uint8_t raw_tank_low = 0;
uint8_t treated_tank_high = 0;
uint8_t treated_tank_low = 0;
uint8_t ddps_switch = 0;

void initiate_app_sequence();
void process_rxd_ble_data(const char *data);

void get_sensor_data();
void get_device_config();
void get_device_status();
void get_all_configs();

static DeviceStatus status;
static DeviceConfig config;

#define MIN_SPACE_FOR_LOG 300 // Minimum space required for a log entry

void write_SR_all(uint8_t value);
void shiftOut(uint8_t val);

void sensor_modbus_requests();

void process_data_modbus(void);
esp_err_t gsm_command_exe();
void time_sync(const char *str);
void send_all_logs();

float SOLARPercentage(float voltage);
float BatteryPercentage(float voltage);

void buzzer_beep(uint32_t on_ms, uint32_t off_ms, uint32_t repeat);
void ota_func();
void post_data(void);
void backlog_post_task(void *pvParameters);
static TaskHandle_t backlog_post_task_handle = NULL;
static uint8_t backlog_post_fail_streak = 0;
// static int64_t last_ntp_sync_unix = 0;   /* tracks last GSM NTP sync epoch */
static int64_t last_rtc_sync_unix = 0;

bool is_bat_15V = true;
static time_t scheduled_log_unix = 0;
static uint32_t scheduled_interval_sec = 0;
bool ota_in_progress = false;

bool BLE_INTERRUPT = false;
SemaphoreHandle_t BLE_INTERRUPTxMutex;

void print_binary(uint8_t value)
{
	printf("Binary: ");
	for (int i = 7; i >= 0; i--)
	{
		printf("%d", (value >> i) & 1);
	}
	printf("\n");
}

void init_shift_register()
{
	gpio_set_direction(DATA_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);

	// Initialize with all LOW
	write_SR_all(shift_reg_state);
}

char *fake_data_logs[] = {
	"{\"DT\":1759061732,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":90.5,\"Sol\":85.0,\"UFM1F\":1.2,\"UFM1V\":3.3,\"UFM2F\":1.5,\"UFM2V\":3.4,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759061832,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":89.0,\"Sol\":86.5,\"UFM1F\":1.3,\"UFM1V\":3.4,\"UFM2F\":1.6,\"UFM2V\":3.5,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759061932,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":87.0,\"Sol\":84.0,\"UFM1F\":1.4,\"UFM1V\":3.3,\"UFM2F\":1.7,\"UFM2V\":3.6,\"SW1\":0.0,\"SW2\":1.0}",
	"{\"DT\":1759062032,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":85.5,\"Sol\":83.0,\"UFM1F\":1.5,\"UFM1V\":3.2,\"UFM2F\":1.8,\"UFM2V\":3.7,\"SW1\":1.0,\"SW2\":1.0}",
	"{\"DT\":1759062132,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":84.0,\"Sol\":82.5,\"UFM1F\":1.6,\"UFM1V\":3.4,\"UFM2F\":1.9,\"UFM2V\":3.8,\"SW1\":0.0,\"SW2\":0.0}",
	"{\"DT\":1759062232,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":82.5,\"Sol\":81.0,\"UFM1F\":1.7,\"UFM1V\":3.5,\"UFM2F\":2.0,\"UFM2V\":3.9,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759062332,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":81.0,\"Sol\":80.5,\"UFM1F\":1.8,\"UFM1V\":3.6,\"UFM2F\":2.1,\"UFM2V\":4.0,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759062432,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":79.5,\"Sol\":80.0,\"UFM1F\":1.9,\"UFM1V\":3.7,\"UFM2F\":2.2,\"UFM2V\":4.1,\"SW1\":0.0,\"SW2\":1.0}",
	"{\"DT\":1759062532,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":78.0,\"Sol\":79.0,\"UFM1F\":2.0,\"UFM1V\":3.8,\"UFM2F\":2.3,\"UFM2V\":4.2,\"SW1\":1.0,\"SW2\":1.0}",
	"{\"DT\":1759062632,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":76.5,\"Sol\":78.0,\"UFM1F\":2.1,\"UFM1V\":3.9,\"UFM2F\":2.4,\"UFM2V\":4.3,\"SW1\":0.0,\"SW2\":0.0}",
	"{\"DT\":1759062732,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":75.0,\"Sol\":77.5,\"UFM1F\":2.2,\"UFM1V\":4.0,\"UFM2F\":2.5,\"UFM2V\":4.4,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759062832,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":73.5,\"Sol\":76.0,\"UFM1F\":2.3,\"UFM1V\":4.1,\"UFM2F\":2.6,\"UFM2V\":4.5,\"SW1\":1.0,\"SW2\":0.0}",
	"{\"DT\":1759062932,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":72.0,\"Sol\":75.0,\"UFM1F\":2.4,\"UFM1V\":4.2,\"UFM2F\":2.7,\"UFM2V\":4.6,\"SW1\":0.0,\"SW2\":1.0}",
	"{\"DT\":1759063032,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":70.5,\"Sol\":74.0,\"UFM1F\":2.5,\"UFM1V\":4.3,\"UFM2F\":2.8,\"UFM2V\":4.7,\"SW1\":1.0,\"SW2\":1.0}",
	"{\"DT\":1759063132,\"Device\":\"xyz\",\"IMEI\":\"866082073481955\",\"FR_v\":\"1.0.0\",\"Lat\":28.6139,\"Lon\":77.2090,\"Bat\":69.0,\"Sol\":73.0,\"UFM1F\":2.6,\"UFM1V\":4.4,\"UFM2F\":2.9,\"UFM2V\":4.8,\"SW1\":0.0,\"SW2\":0.0}"};

void temp_device_status(DeviceStatus *status)
{
	strcpy(status->current_date, "2023-12-10");
	strcpy(status->current_time, "01:10:12");
	status->gsm_signal_strength = 50;
	status->battery_level = 50.0f;
	status->solar_level = 50.0f;
	strcpy(status->gsm_post_status, "FAIL");
	status->log_count = 10;
}

void extra_func_change_data()
{
	temp_device_status(&status);
	save_device_status_to_nvs(&status);

	// temp_device_config(&config);
	save_device_config_to_nvs(&config);
}

static void ble_no_conn_timeout_cb(TimerHandle_t xTimer)
{
    if (!is_ble_connected_()) {
        ESP_LOGW("BLE", "No connection for 10 min — shutting down BLE");
        write_SR(LED, 0);
        stop_ble();
    }
}

const char *sample_status_string =
	"----- Device Status ----- Date: 2025-07-16 Time: 20:32:10 GSM Signal Strength: -79 dBm"
	"GSM Post Status: SUCCESS"
	"Battery Level: 87.45%"
	"Solar Charging Level: 55.20%"
	"GPIO Extender Enabled: Yes"
	"Log Count Since Last Clear: 147";

/* Initiates BLE sequence and handles data on press */
void btn_press_handler(void *param)
{
	int64_t ble_start_time = 0;
	volatile int64_t m = esp_timer_get_time();
	volatile int64_t x = esp_timer_get_time();
	volatile int64_t connected_time = esp_timer_get_time();
	volatile int64_t buzz_per = esp_timer_get_time();
	bool last_clicked = false;

	// if( is_ble_active() == false)	initiate_app_sequence();
	last_clicked = true;
	int count = 1; // Counter for status notify chunks

	while (1)
	{
		if( ota_in_progress == true)
		{
			buzzer_beep(500, 500, 2);
			continue;
		}

		if (!gpio_get_level(SW2))
		{
			int64_t z = esp_timer_get_time();
			if ((z - x) / 1000 >= 2000)
			{
				ESP_LOGI("SW1 PRESSED", "TRYING TO START BLE");

				if (is_ble_active() == false){
					initiate_app_sequence();
					ble_start_time = esp_timer_get_time();
				}
				write_SR(LED, 1);
				buzzer_beep(500, 250, 2);

				last_clicked = true;
			}
		}
		else
		{
			x = esp_timer_get_time();
		}

		if (!gpio_get_level(GPIO_NUM_0))
		{
			int64_t n = esp_timer_get_time();
			if ((n - m) / 1000 >= 2000)
			{
				ESP_LOGI("BOOT BUTTON:", "Button Pressed FOR 3 SECOND, Erasing logs\n");
				m = esp_timer_get_time();

				// format_spiffs();
				erase_logs();
				clear_posted_logs();
				erase_post_state_blob();
				clear_sensor_nvs_namespace();
				clear_device_status_nvs();
				clear_device_config_nvs(); // CLEAR the sensor data if required.

				ESP_LOGI("BOOT BUTTON", "Device configuration & SPIFF cleared");
			}
		}
		else
		{
			m = esp_timer_get_time();
		}

		vTaskDelay(10);

		if (is_ble_connected_())
		{
			if (ble_no_conn_timer != NULL) {
				xTimerStop(ble_no_conn_timer, 0);
			}
			int64_t blink_time = esp_timer_get_time();
			if ((blink_time - connected_time) / 1000 >= 500)
			{
				// write_SR(LED, !gpio_get_level(LED));
				connected_time = esp_timer_get_time();
			}

			if (last_clicked == true)
			{
				get_all_configs();
				// set_ble_data(all_sensor_info);
				get_device_status();

				last_clicked = false;
			}
		}
		else if (ble_cmd_processing == true)
		{
			ble_cmd_processing = false;
		}
		// timeout check must be here — outside the ble_cmd_processing block
		if (ble_start_time > 0 && is_ble_active() &&
			(esp_timer_get_time() - ble_start_time) / 1000 >= (5 * 60 * 1000))
		{
			ESP_LOGW("BLE", "No connection for 5 min — shutting down BLE");
			write_SR(LED, 0);
			stop_ble();
			ble_start_time = 0;
		}

		if (is_ble_rx_done()) //&& !ble_cmd_processing)
		{
			ESP_LOGE("BLE", "Received BLE data, processing...");

			int msg = 1;
			xQueueOverwrite(BLE_INT_QUEUE, &msg); // request interrupt
			ESP_LOGB("BLE", "Interrupt request sent");

			// Wait until POLL confirms it has paused
			xSemaphoreTake(BLE_INT_POLL_PAUSED, portMAX_DELAY);
			ESP_LOGW("BLE", "POLL paused, starting urgent work");

			const char *rxd_buff = read_ble_rx_buffer();
			// printf("rxd_buff ptr address: %p\n", (void *)rxd_buff);

			if (rxd_buff == NULL)
			{
				ESP_LOGW("BLE", "Received NULL data from BLE RX buffer");
				reset_ble_rx_buffer();
				// Handle NULL case here (return, skip, etc.)

				// Signal POLL that urgent work is finished
				xSemaphoreGive(BLE_INT_BLE_DONE);
				ESP_LOGI("BLE", "Urgent work done, POLL may resume");
			}
			else
			{
				ESP_LOGI("DEBUG", "First 20 bytes:'%.*s'", 20, rxd_buff);
				// Process rxd_buff as usual
				process_rxd_ble_data(rxd_buff);
				// Signal POLL that urgent work is finished
				xSemaphoreGive(BLE_INT_BLE_DONE);
				ESP_LOGB("BLE", "Urgent work done, POLL may resume");
			}
		}
	}
}

void get_all_configs()
{
	get_sensor_data();
	get_device_config();
	get_device_status();
}

void process_rxd_ble_data(const char *data)
{
	ESP_LOGI("BLE", "Processing received BLE data: %s", data);
	// const char *prefix = "{$sensor_config$";
	// printf("Data ptr address: %p\n", (void *)data);

	if (!data)
	{
		ESP_LOGW("DEBUG", "Data pointer is NULL");
	}
	else if (strlen(data) == 0)
	{
		ESP_LOGW("DEBUG", "Data is an empty string");
		// ESP_LOGI("DEBUG", "First 20 bytes:'%.*s'", 20, data);
	}
	else
	{
		ESP_LOGI("DEBUG", "First 20 bytes:'%.*s'", 20, data);
	}

	if (data && strcmp(data, "{$send_sensor_data}") == 0)
	{
		printf("Received command to notify send_config1 data\n");
		get_sensor_data();
		status_notify_chunks(all_sensor_info);
	}

	else if (data && strcmp(data, "{$send_device_config}") == 0)
	{
		printf("Received command to notify send_config2 data\n");
		get_device_config();
		status_notify_chunks(device_config_str);
	}

	else if (data && strcmp(data, "{$send_device_status}") == 0)
	{
		printf("Received command to notify send_config3 data\n");
		get_device_status();
		status_notify_chunks(device_status_str);
	}

	else if (data && strcmp(data, "{$send_all_data_config}") == 0)
	{
		printf("Received command to notify send_all_data_config data\n");
		get_all_configs();
		status_notify_chunks(all_sensor_info);
		status_notify_chunks(device_status_str);
		status_notify_chunks(device_config_str);
	}

	else if (data && strncmp(data, "{$device_config$", 16) == 0)
	{
		process_device_config_data(data, &config);
		save_device_config_to_nvs(&config);
		load_device_config_from_nvs(&config);

		get_device_config();
		status_notify_chunks(device_config_str);
		ble_interrupted = true;
	}

	else if (data && strncmp(data, "{$sensor_config$", 16) == 0)
	{
		process_sensor_config_data(data);
		save_sensors_to_nvs();
		load_sensors_from_nvs();

		get_sensor_data();
		status_notify_chunks(all_sensor_info);
		ble_interrupted = true;
	}

	// else if( data && strncmp(data, "{$device_config$", 16) == 0)
	// {
	// 	process_device_config_data(data, &config);
	// }

	else if (data && strncmp(data, "{$send_logs}", 12) == 0)
	{
		send_all_logs();
		// for (int i = 0; i < (sizeof(fake_data_logs) / sizeof(fake_data_logs[0])); i++)
		// // for (int i = 0; i < 2; i++)
		// {
		// 	status_notify_chunks(fake_data_logs[i]);
		// 	// vTaskDelay(pdMS_TO_TICKS(100));
		// }
	}
	else if (data && strncmp(data, "{$delete_logs}", 12) == 0)
	{
		ESP_LOGW("BLE", "Deleting Logs...");
		erase_logs();
		// Delete the logs files
	}
	else if (data && strncmp(data, "{$time_sync$", 12) == 0)
	{
		ESP_LOGW("BLE", "Syncing Time...");
		// Time sync
		time_sync(data);
		get_device_config();
		status_notify_chunks(device_config_str);
		ble_interrupted = true;
	}
	else
	{
		ESP_LOGW("BLE", "Unknown command received: %s", data);
	}

	reset_ble_rx_buffer();
}

void CHECK_BLE_INTERRUPT()
{
	int msg = 0;
	if (xQueueReceive(BLE_INT_QUEUE, &msg, 0) == pdPASS && msg == 1)
	{
		ESP_LOGW("POLL", "Interrupt received, pausing");

		// tell BLE: I have paused
		xSemaphoreGive(BLE_INT_POLL_PAUSED);

		// wait until BLE finishes
		xSemaphoreTake(BLE_INT_BLE_DONE, portMAX_DELAY);

		ESP_LOGW("POLL", "BLE done, resuming");
		return; // exit poll and restart fresh
	}
}

void initiate_app_sequence()
{
	ESP_LOGI("INITIATE", "Initiating application sequence...");
	// set_ble_name("ESP_TEST1_device");
	// start_BLE(DEVICE_ID);
	start_BLE(config.server_name);
	sleep(1); // Let BLE init
}

void get_sensor_data()
{
	// Use pointer tracking to write directly to the GLOBAL buffer.
	// This saves ~800 bytes of stack memory, preventing the crash.
	char *ptr = all_sensor_info;
	size_t remaining = sizeof(all_sensor_info);
	int len = 0;

	memset(all_sensor_info, 0, sizeof(all_sensor_info));

	// 1. Start JSON
	len = snprintf(ptr, remaining, "{");
	if (len > 0)
	{
		ptr += len;
		remaining -= len;
	}

	int sensor_count = get_sensor_count();
	printf("Sensor Count : %d\n", sensor_count);

	// 2. Digital Inputs (Direct write, no temp buffer)
	len = snprintf(ptr, remaining,
				   "S_Name:Raw Water HIGH,Key:RW_H,Val:%d,is_en:1,Offset:0.0,Scale:1.0,RS485:Yes,Response:%s,"
				   "S_Name:Raw Water LOW,Key:RW_L,Val:%d,is_en:1,Offset:0.0,Scale:1.0,RS485:Yes,Response:%s,"
				   "S_Name:Treated Water HIGH,Key:TW_H,Val:%d,is_en:1,Offset:0.0,Scale:1.0,RS485:Yes,Response:%s,"
				   "S_Name:Treated Water LOW,Key:TW_L,Val:%d,is_en:1,Offset:0.0,Scale:1.0,RS485:Yes,Response:%s,"
				   "S_Name:Digital Diff Press Sw,Key:DDPS,Val:%d,is_en:1,Offset:0.0,Scale:1.0,RS485:Yes,Response:%s,",
				   (int)raw_tank_high, sensors[DigitalInputs].response_ok ? "OK" : "Fail",
				   (int)raw_tank_low, sensors[DigitalInputs].response_ok ? "OK" : "Fail",
				   (int)treated_tank_high, sensors[DigitalInputs].response_ok ? "OK" : "Fail",
				   (int)treated_tank_low, sensors[DigitalInputs].response_ok ? "OK" : "Fail",
				   (int)ddps_switch, sensors[DigitalInputs].response_ok ? "OK" : "Fail");
	if (len > 0 && len < remaining)
	{
		ptr += len;
		remaining -= len;
	}

	// 3. Other Sensors
	for (int i = 1; i < sensor_count; i++)
	{
		// Write directly to ptr, removing the need for 'char sensor_info[128]'
		len = snprintf(ptr, remaining,
					   "S_Name:%s,Key:%s,Val:%.2f,is_en:%s,Offset:%.2f,Scale:%.2f,RS485:%s,Response:%s,",
					   sensors[i].name,
					   sensors[i].key,
					   sensors[i].value,
					   sensors[i].is_enabled ? "1" : "0",
					   sensors[i].offset,
					   sensors[i].scale,
					   sensors[i].is_rs485 ? "Yes" : "No",
					   sensors[i].response_ok ? "OK" : "Fail");

		if (len > 0 && len < remaining)
		{
			ptr += len;
			remaining -= len;
		}
	}

	// 4. Close JSON
	strlcat(all_sensor_info, "}", sizeof(all_sensor_info));

	// printf("ALL SENSOR INFO: %s\n", all_sensor_info);
}

void get_device_config()
{
	memset(device_config_str, 0, sizeof(device_config_str));
	snprintf(device_config_str, sizeof(device_config_str),
		"{$device_config$"
		"Device_type:%s,"
		"IMEI_num:%s,"
		"Servr_name:%s,"
		"Fr_vr:%s,"
		"data_freq:%ld,"
		"lat:%.6f,"
		"lon:%.6f,"
		"PS_enabled_DDPS_disabled:%s,"
		"wifi_enabled_gsm_disabled:%s,"
		"wifi_ssid:%s,"
		"wifi_password:%s,"
		"num_level_sensor:%d,"
		"PS_delta:%ld,"
		"Backwash_frequency_min:%ld,"
		"Backwash_time_sec:%ld,"
		"gsm_sim_name:%s"
		"}",
		config.device_type,
		config.IMEI_num,
		config.server_name,
		config.firmware_version,
		config.data_frequency_sec,
		config.latitude,
		config.longitude,
		config.PS_enabled_DDPS_disabled ? "1" : "0",
		config.wifi_enabled_gsm_disabled ? "1" : "0",
		config.wifi_ssid,
		config.wifi_password,
		config.num_level_sensor,
		config.PS_delta,
		config.Backwash_frequency_min,
		config.Backwash_time_sec,
		config.gsm_sim_name
	);		
	ESP_LOGI("CONFIG", "%s", device_config_str);

}

void get_device_status()
{
	memset(device_status_str, 0, sizeof(device_status_str));
	update_and_print_time();

	// if (load_device_status_from_nvs(&status) == ESP_OK)
	{
		snprintf(device_status_str, sizeof(device_status_str),
				 "{$device_status$"
				 "date:%s,"
				 "time:%s,"
				 "gsm_sig:%d,"
				 "gsm_post_status:%s,"
				 "battery_lvl:%.2f%%,"
				 "solar_lvl:%.2f%%,"
				 "log_count:%ld}",
				 status.current_date,
				 time_app_hms,
				 status.gsm_signal_strength,
				 status.gsm_post_status,
				 status.battery_level,
				 status.solar_level,
				 status.log_count);

		ESP_LOGI("STATUS", "%s", device_status_str);
		// You can now use `device_status_str` in BLE notify or elsewhere
	}
}

#define IST_OFFSET_SEC (5*3600 + 30*60)

void time_sync(const char *str)
{
	const char *key = "$time_sync$";
	char *ptr = strstr(str, key);

	if (ptr != NULL)
	{
		// Move pointer right after $time_sync$
		ptr += strlen(key);
	}
	else
		return;

	// Step 3: Extract the number until space or end of string
	char temp[20]; // buffer for the number
	int i = 0;
	while (*ptr && *ptr != '}' && i < (int)(sizeof(temp) - 1))
	{
		temp[i++] = *ptr++;
	}
	temp[i] = '\0'; // null terminate the number string

	// Step 4: Convert to integer
	uint32_t timestamp = (uint32_t)strtoul(temp, NULL, 10);

	// Print result
	printf("Unix timestamp: %lu\n", timestamp);

	// Step 5: Convert to date/time structure
	time_t rawtime = (time_t)timestamp;
	rawtime += IST_OFFSET_SEC;
	struct tm ts;
	gmtime_r(&rawtime, &ts); // or use gmtime_r for UTC

	// Add IST offset manually (5 hours 30 min)
	// ts.tm_hour += 5;
	// ts.tm_min += 30;

	// Normalize in case minutes/hours overflow
	mktime(&ts);

	// Step 6: Store in separate variables
	int year = ts.tm_year + 1900;
	int month = ts.tm_mon;
	int day = ts.tm_mday;
	int hour = ts.tm_hour;
	int minute = ts.tm_min;
	int second = ts.tm_sec;
	int weekday = ts.tm_wday;

	// Print results
	printf("Date: %02d-%02d-%04d\n", day, month, year);
	printf("Time: %02d:%02d:%02d\n", hour, minute, second);
	printf("Weekday:%d\n", weekday);

	set_rtc_time(day, weekday, month, year, hour, minute, second);
	/* RTC changed via BLE, re-anchor scheduler on next cycle. */
	scheduled_log_unix = 0;
	scheduled_interval_sec = 0;
}

char compact_summary_str[1024]; // Adjust size as needed
void generate_compact_summary_string()
{
	// Use a pointer to track position in the buffer.
	// This PREVENTS buffer scanning errors that cause \u0001 characters.
	char *ptr = compact_summary_str;
	size_t remaining = sizeof(compact_summary_str);
	int len = 0;

	// Clear the buffer completely to remove old garbage
	memset(compact_summary_str, 0, sizeof(compact_summary_str));

	// 1. Header Information
	len = snprintf(ptr, remaining,
				   "{"
				   "\"DT\":%lld,"
				   "\"Device\":\"%s\","
				   "\"D_type\":\"%s\","
				   "\"IMEI\":\"%s\","
				   "\"FR_v\":\"%s\","
				   "\"Lat\":%.6f,"
				   "\"Lon\":%.6f,",
				   unix_timestamp,
				   config.server_name,
				   config.device_type,
				   config.IMEI_num,
				   config.firmware_version,
				   config.latitude,
				   config.longitude);

	// Advance pointer safely
	if (len > 0 && len < remaining)
	{
		ptr += len;
		remaining -= len;
	}

	// 2. Sensor Data Loop
	int sensor_count = get_sensor_count();

	for (int i = 0; i < sensor_count; i++)
	{

		// Skip disabled sensors
		if (!sensors[i].is_enabled)
			continue;

		// Skip DigitalInputs (handled manually later)
		if (i == DigitalInputs)
			continue;

		// Skip specific excluded sensors
		if (i == HMI_TDS_IN || i == HMI_TDS_OUT ||
			i == HMI_TEMPW_IN || i == HMI_TEMPW_OUT ||
			i == DDPS_en || i == BACKW_FREQ || i == PS_DELTA)
		{
			continue;
		}

		// --- CORRUPTION FIX ---
		// We use the pointer 'ptr' to write exactly where we need to.
		// We also force a clean format string.
		const char *key_to_use = sensors[i].key;

		// HARD FIX for the specific corrupt key if needed
		if (i == HMI_BACKW_BTN)
		{
			key_to_use = "HMI_BW_B";
		}

		len = snprintf(ptr, remaining, "\"%s\":%.2f,", key_to_use, sensors[i].value);
		if (len > 0 && len < remaining)
		{
			ptr += len;
			remaining -= len;
		}
	}

	// 3. Digital Input Data
	// Casting to double to match %.2f format specifier
	len = snprintf(ptr, remaining,
				   "\"RW_H\":%.2f,"
				   "\"RW_L\":%.2f,"
				   "\"TW_H\":%.2f,"
				   "\"TW_L\":%.2f,"
				   "\"DDPS\":%.2f", // No trailing comma here
				   (double)raw_tank_high,
				   (double)raw_tank_low,
				   (double)treated_tank_high,
				   (double)treated_tank_low,
				   (double)ddps_switch);

	if (len > 0 && len < remaining)
	{
		ptr += len;
		remaining -= len;
	}

	// 4. Close JSON
	strlcat(compact_summary_str, "}", sizeof(compact_summary_str));

	// DEBUG: Print the final string to confirm it is clean
	// printf("FINAL JSON: %s\n", compact_summary_str);
}

FILE *log_file = NULL;

void send_all_logs()
{
    FILE *f = fopen(log_file_path, "r");
    if (!f)
    {
        ESP_LOGE("send_all_logs", "Unable to open log file for reading");
        status_notify_fast("{$log_start$}");
        status_notify_fast("{$log_end$}");
        ble_notify_drain();
        return;
    }

    ESP_LOGW("send_all_logs", "Sending all logs (fast mode)");

    char line[512];
    uint32_t log_count = 0;

    status_notify_chunks("{$log_start$}");

    while (fgets(line, sizeof(line), f))
    {
        status_notify_fast(line);
        log_count++;

        /* Every 20 logs, let the BLE controller drain to prevent queue overflow.
         * This spacing is critical on ESP32-NimBLE to avoid VHCI watchdog crashes. */
        if (log_count % 20 == 0) {
            ble_notify_drain();
            ESP_LOGI("send_all_logs", "Sent %lu logs so far...", (unsigned long)log_count);
        }
    }
    fclose(f);

    /* Send end marker and wait for BLE controller to flush all pending notifies */
    status_notify_fast("{$log_end$}");
    ble_notify_drain();

    ESP_LOGI("send_all_logs", "Done — sent %lu logs total", (unsigned long)log_count);
}


int uart_vprintf(const char *fmt, va_list args)
{
	char buf[256];
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	if (len > 0)
	{
		uart_write_bytes(UART_NUM_2, buf, len);
	}
	return len;
}

// void shiftOut(uint8_t val) {
//   uint8_t i;
//         // gpio_set_level(CLOCK_PIN, 0);
//         // gpio_set_level(DATA_PIN, (value >> i) & 0x01);
//         // gpio_set_level(CLOCK_PIN, 1);
// 	gpio_set_level(LATCH_PIN, 0);  // Begin
//   for (i = 0; i < 8; i++) {
//     //   digitalWrite(dataPin, !!(val & (1 << (7 - i))));
// 	  gpio_set_level(DATA_PIN, !!(val & (1 << (7 - i))));

//     // digitalWrite(clockPin, HIGH);
// 	gpio_set_level(CLOCK_PIN, 1);
//     // digitalWrite(clockPin, LOW);
// 	gpio_set_level(CLOCK_PIN, 0);
//   }
//   gpio_set_level(LATCH_PIN, 1);  // Stop
// }

void write_sensor_RS485(uint8_t *data, int len)
{
	// printf("REDE Tx\n");
	// RE DE Pin TOGGLE

	write_SR(SEN_RS485_REDE, HIGH);
	//  shiftOut(0b10101010);  // Enable TX
	// write_SR_all(0b10101010);
	// print_binary(0b10001010);

	// vTaskDelay(1 / portTICK_PERIOD_MS);

	write_UART1(data, len);
	uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100)); // Wait max 100 ms
	// ESP_LOGI("RS485","Sent !");

	// vTaskDelay(10 / portTICK_PERIOD_MS);

	// write_SR_all(0b10001010);
	// print_binary(0b10001010);
	//  shiftOut(0b00001010);  // Enable RX
	// RE DE Pin TOGGLE
	// printf("REDE Rx\n");
	write_SR(SEN_RS485_REDE, LOW);
}

void temp_print_arr(uint8_t *arr)
{
	printf("Sent:");

	for (int i = 0; i < 8; i++)
	{
		printf(" 0x%02X", arr[i]);
	}

	printf("\n");
}

void response_print(uint8_t *arr, uint8_t len)
{
	printf("Response Arr:");

	for (int i = 0; i < len; i++)
	{
		printf(" 0x%02X", arr[i]);
	}

	printf("\n");
}

/**
 * @brief Convert date/time strings into Unix timestamp
 * @param date_str e.g. "2025-01-01"
 * @param time_str e.g. "10:05:06"
 * @return time_t (Unix timestamp), or -1 on error
 */
time_t convert_to_unix(const char *date_str, const char *time_str)
{
	struct tm t = {0}; // Zero-initialize

	// Parse date (DD-MM-YYYY)
	sscanf(date_str, "%2d-%2d-%4d", &t.tm_mday, &t.tm_mon, &t.tm_year);
	t.tm_mon -= 1;	   // struct tm months are 0–11
	t.tm_year -= 1900; // struct tm years are since 1900

	// Parse time (HH:MM:SS)
	sscanf(time_str, "%2d:%2d:%2d", &t.tm_hour, &t.tm_min, &t.tm_sec);

	// ── CRITICAL FIX ──────────────────────────────────────────────────────────
	// The physical RTC chip always stores IST time as raw H:M:S numbers.
	// Our "unix_timestamp" (DT field) is defined as IST epoch (UTC+5:30 epoch),
	// NOT standard UTC epoch.  mktime() is TZ-sensitive: if TZ=IST-5:30 is
	// active (set by sync_time() after a WiFi NTP sync), mktime correctly
	// converts IST→UTC, returning a value 19800s too low.
	// Fix: temporarily force TZ=UTC0 so mktime treats the IST H:M:S as-is,
	// returning the IST epoch numerically — exactly what the server expects.
	// This matches ACOM's get_realtime_rtc_ts() approach.
	// ──────────────────────────────────────────────────────────────────────────
	char old_tz[32] = {0};
	const char *tz_env = getenv("TZ");
	if (tz_env) {
		strncpy(old_tz, tz_env, sizeof(old_tz) - 1);
	}

	setenv("TZ", "UTC0", 1);
	tzset();

	time_t result = mktime(&t);

	// Restore previous TZ
	if (old_tz[0]) {
		setenv("TZ", old_tz, 1);
	} else {
		unsetenv("TZ");
	}
	tzset();

	return result;
}

bool wait_for_flags(int timeout_sec)
{
	while (timeout_sec > 0)
	{
		// if( flow_rate_lpm != 0 && flow_counting == false)
		// if( temp_checking == false)
		if (adc_sampling == false)
			return true;

		vTaskDelay(100 / portTICK_PERIOD_MS);
		timeout_sec -= 100;
		CHECK_BLE_INTERRUPT();
	}
	return false;
}

bool wait_for_gsm(int timeout_sec)
{
	while (timeout_sec > 0)
	{
		if (gsm_cmd_checking == false)
			return true;

		vTaskDelay(100 / portTICK_PERIOD_MS);
		timeout_sec -= 100;
	}
	return false;
}

void daily_restart_check(const struct tm *timeinfo)
{
	if (!timeinfo)
		return;

	int today = timeinfo->tm_mday;

	nvs_handle_t nvs;
	int32_t last_day = -1; // default invalid value

	if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
	{
		esp_err_t err = nvs_get_i32(nvs, "last_day", &last_day);
		if (err != ESP_OK)
		{
			ESP_LOGW(TAG, "No previous day stored, first run");
		}

		if (today != last_day)
		{
			ESP_LOGI(TAG, "New day detected! (was %ld, now %d)", last_day, today);

			// Save new day before restart
			nvs_set_i32(nvs, "last_day", today);
			nvs_commit(nvs);
			nvs_close(nvs);

			ota_func();
			// esp_restart(); // restart once per day
		}

		nvs_close(nvs);
	}
	else
	{
		ESP_LOGE(TAG, "Failed to open NVS!");
	}
}

// const char *post_body =
// 						"{\"DT\":1755113963,"
// 						"\"Device\":\"Test\","
// 						"\"IMEI\":\"123IEMEI457\","
// 						"\"FR_v\":\"1.0.0\","
// 						"\"Lat\":0.100000,"
// 						"\"Lon\":0.200000,"
// 						"\"Bat\":100.00,"
// 						"\"Sol\":100.00,"
// 						"\"PSR\":0.00,"
// 						"\"PH\":0.00,"
// 						"\"TPA\":0.00,"
// 						"\"FLW\":0.00,"
// 						"\"SW1\":0.00,"
// 						"\"SW2\":0.00}";
// 	// gsm_command_exe();
// 	// gsm_post_data(post_body);

// 	/*
// 		if gsm INIT or POST fails
// 		check
// 		if data is already marked as failed
// 			yes - do nothing
// 			no  - mark the start of failed data (prev_log_offset)
// 		else
// 			try post failed data first
// 				success - clear failed offset
// 				fail    - do nothing, try next time
// 				atleast 5 success - update/store failed offset
// 		then try post new data
// 			success - do nothing
// 			fail    - mark the start of failed data (prev_log_offset)
// 	*/

static void sync_rtc_from_wifi(void)
{
    if (last_rtc_sync_unix != 0 &&
        (unix_timestamp - last_rtc_sync_unix) < 86400LL) {   /* once per day */
        return;
    }

    /* sync_time() writes UTC to physical RTC */
    if (sync_time() == ESP_OK) {
        scheduled_log_unix     = 0;
        scheduled_interval_sec = 0;

        // 1. Get the fresh UTC time that sync_time() just fetched
        time_t utc_now = time(NULL); 

        // 2. Force your tracking variables to represent IST Epoch
        unix_timestamp          = utc_now + IST_OFFSET_SEC; 
        last_rtc_sync_unix = unix_timestamp;

        ESP_LOGI("WIFI_NTP", "RTC synced via SNTP and offset to IST");
    } else {
        ESP_LOGW("WIFI_NTP", "SNTP sync failed during WiFi post");
    }
}


/* Sync RTC from GSM NTP once per day.
 * Call only after a confirmed successful POST (PDP context is active).
 * Does NOT touch unix_timestamp (DT) — only writes to physical RTC. */
static void try_gsm_ntp_sync(void)
{
    /* Once per 6 hours */
    if (last_rtc_sync_unix != 0 &&
        (unix_timestamp - last_rtc_sync_unix) < 21600LL) {   // ← was 86400LL
        return;
    }

    int year, mon, day, hour, min, sec, wday;
    if (gsm_get_network_time(&year, &mon, &day, &hour, &min, &sec, &wday) != ESP_OK) {
        ESP_LOGW("GSM_NTP", "RTC sync skipped — NTP fetch failed");
        return;
    }

    /* gsm_get_network_time returns UTC. Convert to IST before writing to RTC
     * (RTC stores IST, consistent with BLE time_sync behaviour). */
    struct tm utc_t = {
        .tm_year = year - 1900,
        .tm_mon  = mon  - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min  = min,
        .tm_sec  = sec,
        .tm_isdst = 0
    };
    /* mktime with TZ=UTC (ESP-IDF default) treats struct as UTC → correct epoch */
    time_t utc_epoch = mktime(&utc_t);
    time_t ist_epoch = utc_epoch + IST_OFFSET_SEC;

    struct tm ist_t;
    gmtime_r(&ist_epoch, &ist_t);   /* break IST epoch into IST H:M:S */

    set_rtc_time(ist_t.tm_mday, ist_t.tm_wday,
                 ist_t.tm_mon, ist_t.tm_year,
                 ist_t.tm_hour, ist_t.tm_min, ist_t.tm_sec);


	/* keep unix_timestamp aligned with the corrected RTC */
    unix_timestamp = (int64_t)ist_epoch;
    /* Re-anchor scheduler so next DT slot aligns to corrected RTC */
    scheduled_log_unix    = 0;
    scheduled_interval_sec = 0;

    last_rtc_sync_unix = unix_timestamp;   /* mark — unix_timestamp is current DT, not modified */

    ESP_LOGI("GSM_NTP", "RTC synced from NTP → IST %04d/%02d/%02d %02d:%02d:%02d",
             ist_t.tm_year + 1900, ist_t.tm_mon + 1, ist_t.tm_mday,
             ist_t.tm_hour, ist_t.tm_min, ist_t.tm_sec);
}

esp_err_t handle_gsm_post()
{
	gpio_set_level(GSM_RST_GPIO, 0);
	write_SR(GSM_en, HIGH);
	delay_ms(15000); // Let GSM settle

	size_t curr_offset = get_curr_log_offset(); // Updated in write_log_entry
	// prev_log_offset curr_offset
	// | |
	// v V

	// size_t prev_log_offset = get_prev_log_offset(); // Start of current offset { [100] + LOG_x = [200] }

	if (gsm_at() == ESP_FAIL)
	{
		ESP_LOGE("POLL", "RESTARTING MODEM");
		gpio_set_level(GSM_RST_GPIO, 1);
		write_SR(GSM_en, LOW);
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		write_SR(GSM_en, HIGH);
		gpio_set_level(GSM_RST_GPIO, 0);
		if (get_is_failed() == false)
		{
			set_failed_offset(get_prev_log_offset()); // Failed log address is start of current log address
			ESP_LOGE("POLL", "Set failed offset %d", get_prev_log_offset());
		}
		else
		{
			ESP_LOGE("POLL", "Failed offset ALREADY SET %d", get_failed_offset());
		}
		strcpy(status.gsm_post_status, "FAIL"); // Reset GSM post status
		gsm_requires_init = true;
		return ESP_FAIL;
	}
	bool init_failed = false;

	if(gsm_requires_init == true)
	{
		static int gsm_init_fail_count = 0;

		gsm_uart_busy = true;
		esp_err_t ret = gsm_command_exe();
		gsm_uart_busy = false;

		if( ret == ESP_FAIL )
		{	 
			strcpy(status.gsm_post_status, "FAIL");
			ESP_LOGE("POLL", "GSM init failed");

			gsm_init_fail_count++;
			ESP_LOGW("POLL", "GSM init fail count: %d/3", gsm_init_fail_count);

			/* After 3 consecutive failures, power-cycle the modem to force
			   SIM re-detection. Recovers without needing a full device reboot. */
			if (gsm_init_fail_count >= 3) {
				ESP_LOGE("POLL", "3 consecutive failures — power-cycling modem");
				write_SR(GSM_en, LOW);
				vTaskDelay(pdMS_TO_TICKS(3000));
				write_SR(GSM_en, HIGH);
				vTaskDelay(pdMS_TO_TICKS(10000));
				gsm_init_fail_count = 0;
				ESP_LOGW("POLL", "Modem power-cycled — SIM re-detection next cycle");
			}

			if( get_is_failed() == false)
			{
				set_failed_offset(get_prev_log_offset()); // Failed log address is start of current log address
				ESP_LOGE("POLL", "Set failed offset %d", get_prev_log_offset());
			}
			else
			{
				ESP_LOGE("POLL", "Failed offset ALREADY SET %d", get_failed_offset());
			}
			init_failed = true;
		}
		else
		{
			gsm_init_fail_count = 0;   // reset on success
			gsm_requires_init = false;
		}
	}
	if (init_failed == false)
	{
		post_status_t checked = check_and_try_failed(false);
		// post_status_t checked = no_failed_data; // testing
		if (checked == no_failed_data)
		{
			// Post the new data data
			if (gsm_post_data(compact_summary_str) == ESP_FAIL)
			{
				// size_t failed_offset = get_failed_offset();
				if (get_is_failed() == false)
				{
					set_failed_offset(get_prev_log_offset()); // Failed log address is start of current log address
					ESP_LOGE("POLL", "Set failed offset %d", get_prev_log_offset());
				}
				else
				{
					ESP_LOGE("POLL", "Failed offset ALREADY SET %d", get_failed_offset());
				}
				strcpy(status.gsm_post_status, "FAIL"); // Reset GSM post status
				gsm_requires_init = true;
				return ESP_FAIL;
			}
			else
			{
				backlog_post_fail_streak = 0;
				strcpy(status.gsm_post_status, "OK"); // Reset GSM post status
				ESP_LOGI("POLL", "Posted !!");
				try_gsm_ntp_sync();   /* once/day RTC correction via NTP */
				return ESP_OK;
			}
		}
		else if (checked == backlog_drain_in_progress)
		{
			backlog_post_fail_streak = 0;
			ESP_LOGW("POLL", "Backlog batch done — yielding for next cycle, drain continues");
			strcpy(status.gsm_post_status, "OK");
			try_gsm_ntp_sync();
			return ESP_OK;
		}
		else if (checked == posted_failed_data)
		{
			backlog_post_fail_streak = 0;
			ESP_LOGI("POLL", "Posted all failed data successfully. Now posting current record.");

			if (get_prev_log_offset() == get_curr_log_offset())
			{
				ESP_LOGW("POLL", "Current record was last in backlog drain — skipping duplicate post.");
				strcpy(status.gsm_post_status, "OK");
				return ESP_OK;
			}

			if (gsm_post_data(compact_summary_str) == ESP_FAIL)
			{
				if (get_is_failed() == false)
				{
					set_failed_offset(get_prev_log_offset());
					ESP_LOGE("POLL", "Current record post failed after backlog drain, set failed offset %d", get_prev_log_offset());
				}
				strcpy(status.gsm_post_status, "FAIL");
				gsm_requires_init = true;
				return ESP_FAIL;
			}
			strcpy(status.gsm_post_status, "OK");
			ESP_LOGI("POLL", "Current record posted after backlog drain.");
			try_gsm_ntp_sync();   /* once/day RTC correction via NTP */
			return ESP_OK;
		}
		else if (checked == failed_posting_failed_data)
		{
			ESP_LOGE("POLL", "Failed to post failed data");
			strcpy(status.gsm_post_status, "FAIL");
			backlog_post_fail_streak++;
			ESP_LOGW("POLL", "Backlog post fail streak: %u/10", backlog_post_fail_streak);
			if (backlog_post_fail_streak >= 10) {
				gsm_requires_init = true;
				backlog_post_fail_streak = 0;
				ESP_LOGW("POLL", "Backlog failed 10 times — forcing GSM re-init");
			}
			return ESP_FAIL;
		}
	}
	return ESP_FAIL;
}

esp_err_t handle_wifi_post()
{

	size_t curr_offset = get_curr_log_offset();		// Updated in write_log_entry
													//	 prev_log_offset      curr_offset
													//		   |				   |
													//         v                   V
	size_t prev_log_offset = get_prev_log_offset(); // Start of current offset { [100] +  LOG_x   =  [200] }

	esp_err_t wifi_success = ESP_OK;

	if (wifi_is_connected() == false)
	{
		ESP_LOGW("MAIN", " Connecting to WiFi...");
		if (wifi_handler_init((uint8_t *)config.wifi_ssid, (uint8_t *)config.wifi_password) == ESP_OK)
		{
			ESP_LOGW("MAIN", " WiFi Connected Successfully");
			wifi_success = ESP_OK;
		}
		else
		{
			ESP_LOGE("MAIN", " WiFi Connection Failed");
			wifi_success = ESP_FAIL;
		}
	}

	if (wifi_success == ESP_FAIL)
	{
		if (get_is_failed() == false)
		{
			set_failed_offset(get_prev_log_offset()); // Failed log address is start of current log address
			ESP_LOGE("POLL", "Set failed offset %d", get_prev_log_offset());
		}
		else
		{
			ESP_LOGE("POLL", "Failed offset ALREADY SET %d", get_failed_offset());
		}
		strcpy(status.gsm_post_status, "FAIL"); // Reset GSM post status
		return ESP_FAIL;
	}

	post_status_t checked = check_and_try_failed(true);
	// post_status_t checked = no_failed_data;				// testing

	if (checked == no_failed_data)
	{
		esp_err_t ret = http_send_json(POST_URL, compact_summary_str, HTTP_METHOD_POST);
		if (ret == ESP_OK)
        {
            strcpy(status.gsm_post_status, "OK");
            ESP_LOGI("POLL", "Posted !!");
            set_post_status(get_curr_log_offset(), false, false);
            sync_rtc_from_wifi();   /* once/day RTC correction via SNTP */
        }
		else
		{
			/* Catches ESP_FAIL, ESP_ERR_HTTP_CONNECT, and any other error.
			 * Old code checked == ESP_FAIL only, so ESP_ERR_HTTP_CONNECT (0x7006)
			 * fell into the silent "else" branch with no return and no NVS save. */
			ESP_LOGE("POLL", "WiFi post failed (err=0x%x)", (unsigned)ret);
			if (get_is_failed() == false)
			{
				set_failed_offset(get_prev_log_offset());
				ESP_LOGE("POLL", "Set failed_offset to %zu", (size_t)get_prev_log_offset());
			}
			else
			{
				ESP_LOGE("POLL", "Failed offset ALREADY SET %zu", (size_t)get_failed_offset());
			}
			strcpy(status.gsm_post_status, "FAIL");
			return ESP_FAIL;
		}
	}
	else if (checked == backlog_drain_in_progress)
	{
		ESP_LOGW("POLL", "Backlog batch done — yielding for next cycle, drain continues");
		strcpy(status.gsm_post_status, "OK");
		sync_rtc_from_wifi();   /* first successful post of the day — sync RTC even mid-drain */
		return ESP_OK;
	}
	else if (checked == posted_failed_data)
	{
		ESP_LOGI("POLL", "Posted all backlog successfully. Now posting current record.");
		if (get_prev_log_offset() == get_curr_log_offset()) {
			ESP_LOGW("POLL", "Current record was last in backlog drain — skipping duplicate post.");
			set_post_status(get_curr_log_offset(), false, false);
			strcpy(status.gsm_post_status, "OK");
			return ESP_OK;
		}
		if (http_send_json(POST_URL, compact_summary_str, HTTP_METHOD_POST) != ESP_OK) {
			if (get_is_failed() == false) set_failed_offset(get_prev_log_offset());
			strcpy(status.gsm_post_status, "FAIL");
			return ESP_FAIL;
		}
		strcpy(status.gsm_post_status, "OK");
		set_post_status(get_curr_log_offset(), false, false);
		sync_rtc_from_wifi();
	}
	else if (checked == failed_posting_failed_data)
	{
		ESP_LOGE("POLL", "Failed to post backlog data");
		strcpy(status.gsm_post_status, "FAIL");
	}

	return ESP_OK;
}

uint8_t SensorSelect = 0;

static uint8_t modbus_arr[8] = {0};

static volatile bool modbus_response_status[SENSOR_TYPE_COUNT] = {false};

uint16_t data_t = 0;
//  float pressureVal, phVal, ufFlowVal1, ufVolumeVal1, tdsVal, chlorine_PCVal, chlorine_NCVal, SW1_mod_val, SW2_mod_val,  ufFlowVal2, ufVolumeVal2, weight_val, turbidityVal;
//  float tempWVal, tempAVal, FlowVal, Sw1Val, Sw2Val, tempVal;

// void poll_sensor()
// {
// 	// strcpy(status.gsm_post_status, "FAIL");

// 	ESP_LOGI("POLL", "Polling sensors...");
// 	sensor_data_colleccted = false;

// 	// set_UART2_rx_frame_len(0);

// 	if( gsm_requires_init == false)
// 	{
// 		GSM_wake();
// 	}

// 	// write_SR(GSM_en, HIGH);

// 	// write_SR(IO_12V,HIGH);
// 	// write_SR(SEN_RS485_en,HIGH);

// 	// vTaskDelay(10000 / portTICK_PERIOD_MS);	// Stabilize Sensors

// 	/* Sensor Data */

// 	// write_SR(SEN_RS485_en,HIGH);
// 	// vTaskDelay(50 / portTICK_PERIOD_MS);

// 	for( SensorSelect = 0; SensorSelect < MODBUS_Sensor_Count(); SensorSelect++)
// 	{
// 		if(SensorSelect == HMI_TDS_IN) SensorSelect = SYS_en;

// 		sensor_modbus_requests();
// 		vTaskDelay(5 / portTICK_PERIOD_MS);
// 		CHECK_BLE_INTERRUPT();
// 	}
// 	CHECK_BLE_INTERRUPT();
// 	process_data_modbus();
// 	ESP_LOGI("POLL", "Polling Done.");

// 	// write_SR(IO_12V,LOW);
// 	// write_SR(SEN_RS485_en,LOW);
// }

void poll_sensor()
{
	// ESP_LOGI("POLL", "Polling sensors...");
	sensor_data_colleccted = false;

	// REMOVED: GSM_wake() logic.
	// We do not need to wake the modem here because we will Power it ON later.

	for (SensorSelect = 0; SensorSelect < MODBUS_Sensor_Count(); SensorSelect++)
	{
		if (SensorSelect == HMI_TDS_IN)
			SensorSelect = SYS_en;

		sensor_modbus_requests();
		vTaskDelay(5 / portTICK_PERIOD_MS);
		CHECK_BLE_INTERRUPT();
	}
	CHECK_BLE_INTERRUPT();
	process_data_modbus();
	// ESP_LOGI("POLL", "Polling Done.");
}

void data_post_sequence()
{

	/* RTC Data  */
	struct tm current_time;
	readClock(&current_time);

	// status.current_date,
	strftime(status.current_date, sizeof(status.current_date), "%d-%m-%Y", &current_time);
	strftime(status.current_time, sizeof(status.current_time), "%H:%M:%S", &current_time);
	strftime(time_app_hms, sizeof(time_app_hms), "%H-%M-%S", &current_time);

	ESP_LOGI("POLL", "Date: %s, Time: %s", status.current_date, status.current_time);
	unix_timestamp = convert_to_unix(status.current_date, status.current_time);

	if (unix_timestamp != -1)
	{
		printf("Unix Timestamp: %lld\n", unix_timestamp);
	}
	else
	{
		printf("Invalid date/time format!\n");
	}

	ESP_LOGE("POLL", "Waiting for Flags !");

	/* Wait for Operations to be completed or for timeout , with CHECK_BLE_INTERRUPT*/
	wait_for_flags(5000);

	/* Flow Sensor Data */
	// while( flow_rate_lpm == 0 && flow_counting);
	// sensors[FLW].value = flow_rate_lpm; // Flow
	// // if( sensors[12].value > 1)	sensors[12].value = 0;
	// // sensors[12].value = 25.66;
	// ESP_LOGE("POLL", "Flow Rate: %f", flow_rate_lpm);

	/*  Temperature Sensor Data */
	// while (temp_checking);
	// sensors[TPA].value = temperature; // Ambient Temperature
	// ESP_LOGE("POLL", "A-Temperature: %f", temperature);

	/* ADC Data */
	// while(adc_sampling);
	/* Switch Data */
	// Already in sensors list
	// sensors[13].value = ! gpio_get_level(SW1); // SW1
	// sensors[14].value = ! gpio_get_level(SW2); // SW2

	// ESP_LOGE("POLL", "SW1_mod: %d | SW2_mod: %d\n", (int) sensors[SW1_mod].value, (int) sensors[SW2_mod].value);
	// sensor_data_colleccted = true;

	// CHECK_BLE_INTERRUPT();

	// ESP_LOGW("POLL","Waiting for GSM ");

	// wait_for_gsm(30000);

	// ESP_LOGW("POLL","Done waiting for GSM ");

	/* GPS Data */
	if (check_GPS_data || config.latitude == 0.0 || config.longitude == 0.0)
	{
		if (gsm_uart_busy)
		{
			ESP_LOGW("GPS", "GSM busy — skipping GPS poll, reusing last coords (%.6f, %.6f)", latitude, longitude);
		}
		else if (get_gps_location(&latitude, &longitude, CHECK_BLE_INTERRUPT) == ESP_OK)
		{
			ESP_LOGI("GPS data", "Fetch Success");
			check_GPS_data = false;
		}
		else
		{
			ESP_LOGE("GPS data", "Fetch Fail");
		}
	}
	
	// CHECK_BLE_INTERRUPT();

	/* Lat Long */
	if (latitude != config.latitude && latitude != 0.0)
	{
		config.latitude = latitude;
		config.longitude = longitude;
		save_device_config_to_nvs(&config);
		load_device_config_from_nvs(&config);
		ESP_LOGI("POLL", "SAVING LATITUDE");
	}

	if (longitude != config.longitude && longitude != 0.0)
	{
		config.latitude = latitude;
		config.longitude = longitude;
		save_device_config_to_nvs(&config);
		load_device_config_from_nvs(&config);
		ESP_LOGI("POLL", "SAVING LONGITUDE");
	}

	if (longitude != 0.0 || latitude != 0.0)
	{
		config.latitude = latitude;
		config.longitude = longitude;

		ESP_LOGI("POLL", "LAT: %f | LONG: %f", latitude, longitude);
	}

	// print_sensor_data();
	generate_compact_summary_string();
	printf("Compact Summary: %s\n", compact_summary_str);

	/* GSM Data Post */

	write_log_entry(compact_summary_str); // Write LOG to SPIFFS

	/* HTTP/GSM posting is handled by backlog_post_task (own 6144-byte stack).
	   Calling it directly from app_main (3584-byte stack) caused the stack overflow. */
	ESP_LOGW("MAIN", "Log written — notifying backlog_post_task");
	if (backlog_post_task_handle) {
		xTaskNotifyGive(backlog_post_task_handle);
	}

	// if(init_failed == false)
	// {
	// 	check_fw_update();
	// }

	// if( ! check_GPS_data)
	// {

	// 	if( gsm_requires_init == false)
	// 	{
	// 		vTaskDelay(1000 / portTICK_PERIOD_MS);
	// 		// GSM_sleep();
	// 	}
	// 	else
	// 	{
	// 		write_SR(GSM_en, LOW);
	// 	}
	// }

	// // Restart once in a day		|    Also refresh the GPS location once !
	// daily_restart_check(&current_time);

	// set_UART2_rx_frame_len(10);

	// if( prev_data_post_fail )
	// {
	// 	// gsm_post_data(compact_summary_str);
	// 	write_log_entry(compact_summary_str);

	// 	if( get_file_size() != 0)
	// 	{
	// 		if( try_post_all_logs() )
	// 		{
	// 			erase_logs();
	// 		}
	// 		else{
	// 			ESP_LOGE("POLL", "Failed Posting failed data");
	// 		}
	// 	}
	// 	else
	// 	{
	// 		ESP_LOGE("POLL", "Prev data file empty, Failed Posting !");
	// 	}
	// }
	// else
	// {
	// 	if( gsm_post_data(compact_summary_str) == ESP_FAIL)
	// 	{
	// 		// log_failed_data();
	// 		write_log_entry(compact_summary_str);
	// 		prev_data_post_fail = true;
	// 	}
	// 	else{
	// 		prev_data_post_fail = false;
	// 	}
	// }
}
// 0 to 5 times fail
// 5 to 10 times pass
// reset count

esp_err_t fake_gsm_post_data(uint8_t count)
{
	static uint8_t fail_count = 0;

	// if fail_count > 5 , pass the data, so 5 times fail
	if (fail_count++ >= count)
	{
		return ESP_OK;
	}
	else if (fail_count >= count * 2) // at fail_count = 10, i.e 5 times failed and 5 times passed, reset
	{
		fail_count = 0;
		return ESP_OK;
	}
	else
	{
		ESP_LOGE("FAKE GSM POST", "Failed to post data");
		return ESP_FAIL;
	}
}

void fake_logs_test()
{
	/* RTC Data  */
	struct tm current_time;
	readClock(&current_time);
	printf("Time: %02d:%02d:%02d %02d/%02d/%04d\n",
		   current_time.tm_hour, current_time.tm_min, current_time.tm_sec,
		   current_time.tm_mday, current_time.tm_mon + 1, current_time.tm_year + 1900);

	// strcpy(config->date_format, "DD-MM-YYYY");
	// strcpy(config->time_format, "HH:MM:SS");
	// strcpy(status->current_date, "2025-01-01");
	// strcpy(status->current_time, "10:05:06");

	// status.current_date,
	strftime(status.current_date, sizeof(status.current_date), "%d-%m-%Y", &current_time);
	strftime(status.current_time, sizeof(status.current_time), "%H:%M:%S", &current_time);
	strftime(time_app_hms, sizeof(time_app_hms), "%H-%M-%S", &current_time);

	// ESP_LOGI("fake_logs_test", "Date: %s, Time: %s", status.current_date, status.current_time);
	unix_timestamp = convert_to_unix(status.current_date, status.current_time);

	if (unix_timestamp != -1)
	{
		// printf("Unix Timestamp: %lld\n", unix_timestamp);
	}
	else
	{
		printf("Invalid date/time format!\n");
	}

	generate_compact_summary_string();
	printf("Compact Summary: %.*s\n", 17, compact_summary_str);

	/* GSM Data Post */

	write_log_entry(compact_summary_str);
	// size_t curr_offset = get_curr_log_offset();

	size_t curr_offset = get_curr_log_offset();		// Updated in write_log_entry
													//	 prev_log_offset      curr_offset
													//		   |				   |
													//         v                   V
	size_t prev_log_offset = get_prev_log_offset(); // Start of current offset { [100] +  LOG_x   =  [200] }

	// if ( gsm_at() == ESP_FAIL )
	// {
	// 	ESP_LOGE("POLL","RESTARTING ESP");

	// 	// vTaskDelay(3000 / portTICK_PERIOD_MS);

	// 	write_SR(GSM_en, LOW);
	// 	vTaskDelay(3000 / portTICK_PERIOD_MS);
	// 	write_SR(GSM_en, HIGH);
	// }

	static uint8_t fail_count = 0;
	bool init_failed = false;

	if (gsm_requires_init == true)
	{
		// esp_err_t ret = gsm_command_exe();

		// Logs testing
		esp_err_t ret = fake_gsm_post_data(3);

		// if( fail_count++ > 3)
		// {
		// 	ret = ESP_OK;
		// 	fail_count = 0;
		// }
		// else
		// {
		// 	ret = ESP_FAIL;
		// }

		if (ret == ESP_FAIL)
		{
			ESP_LOGE("fake_logs_test", "GSM init failed");
			if (get_is_failed() == false)
			{
				set_failed_offset(prev_log_offset); // Failed log address is start of current log address
				ESP_LOGE("fake_logs_test", "Set failed offset %d", prev_log_offset);
			}
			else
			{
				ESP_LOGE("fake_logs_test", "Failed offset ALREADY SET %d", get_failed_offset());
			}
			init_failed = true;
		}
		else
		{
			gsm_requires_init = false;
		}
	}

	if (init_failed == false)
	{
		post_status_t checked = check_and_try_failed(false);
		// post_status_t checked = no_failed_data;				// testing

		if (checked == no_failed_data)
		{
			// Post the new data data
			// if( gsm_post_data(compact_summary_str) == ESP_FAIL )
			// if( fake_gsm_post_data(3) == ESP_FAIL )
			if (fake_post_fail(compact_summary_str) == ESP_FAIL)
			{

				if (get_is_failed() == false)
				{
					set_failed_offset(prev_log_offset); // Failed log address is start of current log address
					ESP_LOGE("fake_logs_test", "Set failed offset %d", prev_log_offset);
				}
				else
				{
					ESP_LOGE("fake_logs_test", "Failed offset ALREADY SET %d", get_failed_offset());
				}
			}
			else
			{
				ESP_LOGI("fake_logs_test", "Posted !!");
			}
		}
		else if (checked == posted_failed_data)
		{
			ESP_LOGI("fake_logs_test", "Posted all failed data successfully");
		}
		else if (checked == failed_posting_failed_data)
		{
			ESP_LOGE("fake_logs_test", "Failed to post failed data");
		}
	}

	size_t last_log_off = get_curr_log_offset();
	size_t failed_offset = get_failed_offset();
	bool failed_flag = get_is_failed();

	// if(fail_offf != 0)
	{
		ESP_LOGE("fake_logs_test", "last log offset: %d, failed offset: %d, failed flag: %d", last_log_off, failed_offset, failed_flag);
	}

	if (gsm_requires_init == false)
	{
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		// GSM_sleep();
	}
}

void print_sensor_name(uint8_t sensor)
{
	switch (sensor)
	{

	case DigitalInputs:
		printf("DIGITAL INPUTS\n");
		break;

	case Pressure1:
		printf("PRESSURE 1\n");
		break;

	case Pressure2:
		printf("PRESSURE 2\n");
		break;

	case Flow1:
		printf("Flow 1\n");
		break;

	case Flow2:
		printf("Flow 2\n");
		break;

	case Rly_SV1:
		printf("Rly_SV1\n");
		break;

	case Rly_SV2:
		printf("Rly_SV2\n");
		break;

	case Rly_SV3:
		printf("Rly_SV3\n");
		break;

	case Rly_SV4:
		printf("Rly_SV4\n");
		break;

	case Rly_HP:
		printf("Relay High Pressure Pump\n");
		break;

	case Rly_UV:
		printf("Relay UV\n");
		break;

	case Rly_BackW:
		printf("Relay Backwash\n");
		break;

	case Rly_RW:
		printf("Relay RW\n");
		break;

	case SYS_en:
		printf("System Enable\n");
		break;

	case DDPS_en:
		printf("DDPS Enable\n");
		break;

	case BACKW_FREQ:
		printf("Backwash Frequency\n");
		break;

	case PS_DELTA:
		printf("Pressure Delta\n");
		break;

	case HMI_BACKW_BTN:
		printf("HMI Backwash Button\n");
		break;

	case TDS_IN:
		printf("TDS IN\n");
		break;

	case TEMPW_IN:
		printf("TEMPW IN\n");
		break;

	case TDS_OUT:
		printf("TDS OUT\n");
		break;

	case TEMPW_OUT:
		printf("TEMPW OUT\n");
		break;
	default:
		printf("UNKNOWN SENSOR\n");
		break;
	}
}

esp_err_t modbus_write_and_confirm(uint8_t SensorSelect, uint16_t value)
{
	uint8_t tx_frame[8] = {0};
	uint8_t rx_frame[8] = {0};

	/* Build write request (FC06) */
	MODBUS_handler(SensorSelect, tx_frame, 1, value); // read_write = 1 → FC06

	// for (int i = 0; i < len_of_modbus_response(); i++)
	// {
	// 	printf("%02X ", tx_frame[i]);
	// 	printf("\n");
	// }

	/* RS485 TX enable */
	write_SR(SEN_RS485_en, HIGH);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	clear_contentuart1_buff();
	write_sensor_RS485(tx_frame, sizeof(tx_frame));

	/* Expect 8-byte echo */
	set_UART1_rx_frame_len(len_of_modbus_response());

	if (get_UART1_data(rx_frame, 100) == ESP_FAIL)
	{
		ESP_LOGE("MODBUS", "Write timeout");
		return ESP_FAIL;
	}

	/* CRC validation */
	uint16_t crc_calc = modbus_crc(rx_frame, 6);
	uint16_t crc_recv = rx_frame[6] | (rx_frame[7] << 8);

	if (crc_calc != crc_recv)
	{
		ESP_LOGE("MODBUS", "CRC mismatch on write ACK");
		return ESP_FAIL;
	}

	/* ACK must exactly match TX frame */
	if (memcmp(tx_frame, rx_frame, 8) != 0)
	{
		ESP_LOGE("MODBUS", "Invalid write ACK");
		return ESP_FAIL;
	}

	return ESP_OK;
}

float temp_chl = 0.0;
uint8_t count_chl_fail = 0;

/*DigitalInputs Bit masks */
#define RAW_TANK_HIGH_MASK (1 << 0)
#define RAW_TANK_LOW_MASK (1 << 1)
#define TREATED_TANK_HIGH_MASK (1 << 2)
#define TREATED_TANK_LOW_MASK (1 << 3)
#define RMS_PS_DDPS_MASK (1 << 4)

// /* DigitalInputs Variables */
// uint8_t raw_tank_high;
// uint8_t raw_tank_low;
// uint8_t treated_tank_high;
// uint8_t treated_tank_low;
// uint8_t ddps_switch;

void sensor_modbus_requests()
{
	if (SensorSelect == 0)
	{
		for (int i = 0; i < SENSOR_TYPE_COUNT; i++)
			modbus_response_status[i] = false;
	}

	MODBUS_handler(SensorSelect, modbus_arr, 0, 0); // read_write = 0 -> FC03;

	uint8_t resp_len = len_of_modbus_response();

	if (resp_len == 0)
	{
		ESP_LOGW("RETURN", " Expected Response Len is 0");
		return;
	}

	// set_UART2_rx_frame_len(resp_len);
	set_UART1_rx_frame_len(resp_len);

	// print_sensor_name(SensorSelect);
	// temp_print_arr(modbus_arr);
	// ESP_LOGI("LEN","Expected Response Length: %d\n",resp_len);

	uint8_t resp[resp_len];
	memset(resp, 0, resp_len);

	clear_contentuart1_buff();
	write_sensor_RS485(modbus_arr, sizeof(modbus_arr));

	if (get_UART1_data(resp, 100) == ESP_FAIL)
	{
		modbus_response_status[SensorSelect] = false;
		// printf("Timeout from slave: ");
		// print_sensor_name(SensorSelect);
		return;
	}

	// CRC validation
	uint16_t crc_calc = modbus_crc(resp, resp_len - 2);
	uint16_t crc_recv = resp[resp_len - 2] | (resp[resp_len - 1] << 8);

	// if( SensorSelect != TURBIDITY)  // skip CRC for TURBIDITY
	if (crc_calc != crc_recv)
	{
		print_sensor_name(SensorSelect);
		modbus_response_status[SensorSelect] = false;
		printf("CRC mismatch! Expected: 0x%04X, Got: 0x%04X\n", crc_calc, crc_recv);
		return;
	}

	uint8_t byte_count = resp[2];
	if (byte_count != (resp_len - 5))
	{
		printf("Invalid byte count\n");
		modbus_response_status[SensorSelect] = false;
		return;
	}

	modbus_response_status[SensorSelect] = true;
	// printf("Data rxd for %d\n", SensorSelect);

	switch (SensorSelect)
	{
	case DigitalInputs:
		data_t = (resp[3] << 8) | resp[4];
		raw_tank_high = (data_t & RAW_TANK_HIGH_MASK) ? 1 : 0;
		raw_tank_low = (data_t & RAW_TANK_LOW_MASK) ? 1 : 0;
		treated_tank_high = (data_t & TREATED_TANK_HIGH_MASK) ? 1 : 0;
		treated_tank_low = (data_t & TREATED_TANK_LOW_MASK) ? 1 : 0;
		ddps_switch = (data_t & RMS_PS_DDPS_MASK) ? 1 : 0;

		sensors[DigitalInputs].value = (float)data_t; // No acutal use.
		printf("DigitalInputs: %f | raw: %d \n", sensors[DigitalInputs].value, data_t);
		printf("  Raw Tank High: %d\n", raw_tank_high);
		printf("  Raw Tank Low: %d\n", raw_tank_low);
		printf("  Treated Tank High: %d\n", treated_tank_high);
		printf("  Treated Tank Low: %d\n", treated_tank_low);
		printf("  DDPS sw: %d\n", ddps_switch);
		break;

	case Pressure1:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Pressure1].value = (float)data_t / 100;
		// printf("Pressure1: %f | raw: %d \n", sensors[Pressure1].value, data_t );
		break;

	case Pressure2:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Pressure2].value = (float)data_t / 100;
		// printf("Pressure2: %f | raw: %d \n", sensors[Pressure2].value, data_t );
		break;

	case Flow1:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Flow1].value = (float)data_t;
		// sensors[Flow1].value = (float)data_t / 10.0; //Earlier scaling,now changed to direct value from sensor
		// printf("Flow1: %f | raw: %d\n", sensors[Flow1].value, data_t);
		break;

	case Flow2:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Flow2].value = (float)data_t;
		// printf("Flow2: %f | raw: %d\n", sensors[Flow2].value, data_t);
		break;

	case Rly_SV1:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_SV1].value = (float)data_t;
		// printf("Rly_SV1: %f | raw: %d\n", sensors[Rly_SV1].value, data_t);
		break;

	case Rly_SV2:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_SV2].value = (float)data_t;
		// printf("Rly_SV2: %f | raw: %d\n", sensors[Rly_SV2].value, data_t);
		break;

	case Rly_SV3:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_SV3].value = (float)data_t;
		// printf("Rly_SV3: %f | raw: %d\n", sensors[Rly_SV3].value, data_t);
		break;

	case Rly_SV4:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_SV4].value = (float)data_t;
		// printf("Rly_SV4: %f | raw: %d\n", sensors[Rly_SV4].value, data_t);
		break;

	case Rly_HP:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_HP].value = (float)data_t;
		// printf("Rly_HP: %f | raw: %d\n", sensors[Rly_HP].value, data_t);
		break;

	case Rly_UV:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_UV].value = (float)data_t;
		// printf("Rly_UV: %f | raw: %d\n", sensors[Rly_UV].value, data_t);
		break;

	case Rly_BackW:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_BackW].value = (float)data_t;
		// printf("Rly_BackW: %f | raw: %d\n", sensors[Rly_BackW].value, data_t);
		break;

	case Rly_RW:
		data_t = (resp[3] << 8) | resp[4];
		sensors[Rly_RW].value = (float)data_t;
		// printf("Rly_RW: %f | raw: %d\n", sensors[Rly_RW].value, data_t);
		break;

	case SYS_en:
		data_t = (resp[3] << 8) | resp[4];
		sensors[SYS_en].value = (float)data_t;
		// printf("SYS_en: %f | raw: %d\n", sensors[SYS_en].value, data_t);
		break;

	case DDPS_en:
		data_t = (resp[3] << 8) | resp[4];
		sensors[DDPS_en].value = (float)data_t;
		// printf("DDPS_en: %f | raw: %d\n", sensors[DDPS_en].value, data_t);
		break;

	case BACKW_FREQ:
		data_t = (resp[3] << 8) | resp[4];
		sensors[BACKW_FREQ].value = (float)data_t;
		// printf("BACKW_FREQ: %f | raw: %d\n", sensors[BACKW_FREQ].value, data_t);
		break;

	case PS_DELTA:
		data_t = (resp[3] << 8) | resp[4];
		sensors[PS_DELTA].value = (float)data_t / 100;
		// printf("PS_DELTA: %f | raw: %d \n", sensors[PS_DELTA].value, data_t );
		break;

	case HMI_BACKW_BTN:
		data_t = (resp[3] << 8) | resp[4];
		sensors[HMI_BACKW_BTN].value = (float)data_t;
		// printf("HMI_BACKW_BTN: %f | raw: %d\n", sensors[HMI_BACKW_BTN].value, data_t);
		break;
		
	case RMS_SENSOR_CFG:
		data_t = (resp[3] << 8) | resp[4];
		sensors[RMS_SENSOR_CFG].value = (float)data_t;
		// printf("RMS_SENSOR_CFG: %f | raw: %d\n", sensors[RMS_SENSOR_CFG].value, data_t);
		break;

	case TDS_IN:
		sensors[TDS_IN].value = decodeModbusResponse_TDS(resp);
		data_t = (resp[3] << 8) | resp[4];
		// printf("TDS IN: %f | Raw:%d \n", sensors[TDS_IN].value, data_t);
		break;

	case TEMPW_IN:
		data_t = (resp[3] << 8) | resp[4];
		sensors[TEMPW_IN].value = (float)data_t;
		// printf("Temp W IN: %f | raw: %d \n", sensors[TEMPW_IN].value, data_t);
		break;

	case TDS_OUT:
		sensors[TDS_OUT].value = decodeModbusResponse_TDS(resp);
		data_t = (resp[3] << 8) | resp[4];
		// printf("TDS OUT: %f | Raw:%d \n", sensors[TDS_OUT].value, data_t);
		break;

	case TEMPW_OUT:
		data_t = (resp[3] << 8) | resp[4];
		sensors[TEMPW_OUT].value = (float)data_t;
		// printf("Temp W OUT: %f | raw: %d \n", sensors[TEMPW_OUT].value, data_t);
		break;

	default:
		printf("Unknown sensor select\n");
	}
}

// extern sensor_config_t sensors[MAX_SENSORS];

void process_data_modbus(void)
{

	for (int i = 0; i < SENSOR_TYPE_COUNT; i++)
	{
		if (modbus_response_status[i] && sensors[i].is_enabled)
		{
			sensors[i].response_ok = true;
			// sensors[i].value is already set in sensor_modbus_requests
		}
		else
		{
			sensors[i].response_ok = false;
			sensors[i].value = 0.0f;
		}

		// printf("[MODBUS] Sensor[%d] | %s <- %.2f [%s]  | en: [%s]\n",
		//    i,
		//    sensors[i].key,
		//    sensors[i].value,
		//    (sensors[i].response_ok ? "OK" : "FAIL"),
		//    (sensors[i].is_enabled ? "YES" : "NO"));
	}
}

static void IRAM_ATTR flow_sensor_isr_handler(void *args)
{
	flow_pulse_count++;
}

void init_flow_buzz_gpio()
{
	// gpio_pad_select_gpio(FLOW_SENSOR);
	gpio_set_direction(FLOW_SENSOR, GPIO_MODE_INPUT);
	// gpio_pulldown_en(INPUT_PIN);
	// gpio_pullup_dis(INPUT_PIN);
	gpio_set_intr_type(FLOW_SENSOR, GPIO_INTR_POSEDGE);
	//  Install ISR service first!
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // Once per application
													// gpio_install_isr_service();

	gpio_isr_handler_add(FLOW_SENSOR, flow_sensor_isr_handler, (void *)FLOW_SENSOR);

	/* init Switch GPIO */
	gpio_set_direction(FLOW_SENSOR, GPIO_MODE_INPUT);
	gpio_set_direction(on_board_sw1, GPIO_MODE_INPUT);
	gpio_set_direction(on_board_sw2, GPIO_MODE_INPUT);

	gpio_set_direction(SW1, GPIO_MODE_INPUT);
	gpio_set_direction(SW2, GPIO_MODE_INPUT);

	/* Buzzer GPIO init*/
	gpio_reset_pin(BUZZER_GPIO);
	gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

	/* GSM RST PIN */
	gpio_reset_pin(GSM_RST_GPIO);
	gpio_set_direction(GSM_RST_GPIO, GPIO_MODE_OUTPUT);
}

void flow_task(void *pvParameters)
{
	uint32_t notificationValue;

	while (1)
	{
		// Wait for trigger from timer task
		xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);

		flow_pulse_count = 0;
		flow_rate_lpm = 0;
		flow_counting = true;

		// Enable interrupt
		gpio_isr_handler_add(FLOW_SENSOR, flow_sensor_isr_handler, NULL);

		vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds

		// Disable interrupt
		gpio_isr_handler_remove(FLOW_SENSOR);

		uint32_t pulses;
		portENTER_CRITICAL(&flowMux);
		pulses = flow_pulse_count;
		portEXIT_CRITICAL(&flowMux);

		// YF-S401 formula (typical):
		// flow (L/min) = pulses ( Hz) / 7.5      ,  As pulses are counted for 5 sec divide pulse by 5
		flow_rate_lpm = ((float)pulses / (5.0f * 7.5f));
		flow_counting = false;
		ESP_LOGI("FLOW", "Pulses: %ld, Flow Rate: %.2f L/min", pulses, flow_rate_lpm);

		// You can send this value to another task or store it
	}
}

void tmper_task(void *pvParameters)
{
	uint32_t notificationValue;

	while (1)
	{
		// Wait for trigger from timer task
		xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);
		temperature = 0.0;
		temp_checking = true;
		temperature = get_temp_ds18x20();
		temp_checking = false;

		ESP_LOGI("Tmeperature", "Temperature: %f", temperature);

		// You can send this value to another task or store it
	}
}

// esp_err_t gsm_command_exe()
// {

// 		gsm_err =

// 		return ESP_OK;
// }

esp_err_t gsm_command_exe()
{
	// void gsm_task(void* pvParameters) {
	//     uint32_t notificationValue;
	//     while (1) {
	// Wait for trigger from timer task
	// xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);
	// ESP_LOGI("GSM", "Stack high watermark: %d", uxTaskGetStackHighWaterMark(NULL));

	// Turn on the load switch of GSM

	// Wait for GSM to become stable
	// delay(5000);

	/* GSM command execution start */
	gsm_cmd_checking = true;

	// for(int i = 0; i < 20; i++)
	// {
	// 	gsm_at();
	// 	vTaskDelay(pdMS_TO_TICKS(1000));
	// }

	char imei[20];
	esp_err_t gsm_err = ESP_OK;

	gsm_err = gsm_init(imei);

	if (gsm_err == ESP_OK)
	{
		if (imei[0] != '\0')
		{
			if (strcmp(imei, config.IMEI_num))
			{
				strcpy(config.IMEI_num, imei);
				printf("New IMEI :%s\n", config.IMEI_num);
				save_device_config_to_nvs(&config);
				load_device_config_from_nvs(&config);
				print_device_config(&config);
			}
		}

		// ESP_LOGI("GSM", "Stack high watermark: %d", uxTaskGetStackHighWaterMark(NULL));

		int sig_strength = gsm_signal_strength();
		ESP_LOGI("GSM", "Signal Strength: %d", sig_strength);
		// if( sig_strength != 0)
		{
			status.gsm_signal_strength = sig_strength;
		}

		if (strstr(config.gsm_sim_name, "JIO"))
		{
			if (start_internet(0) == ESP_FAIL)
			{
                gsm_err = ESP_FAIL;
				printf("Failed to start internet on 0 SIM\n");
			}
				
		}
		else
		{
			if (start_internet(1) == ESP_FAIL)
			{
                gsm_err = ESP_FAIL;
				printf("Failed to start internet on 1 SIM\n");
			}
		}
		// After start_internet() succeeds, verify context is truly up
		// Only verify context if start_internet() succeeded
		if (gsm_err == ESP_OK)
		{
			if( uart_write_modem("AT+QIACT?\r\n", "+QIACT: 1,1,1", 5000) == ESP_FAIL )
			{
				ESP_LOGE("GSM", "PDP context not active after init, retrying...");
				vTaskDelay(5000 / portTICK_PERIOD_MS);
				if( uart_write_modem("AT+QIACT?\r\n", "+QIACT: 1,1,1", 5000) == ESP_FAIL )
				{
					ESP_LOGE("GSM", "PDP context failed after retry");
					gsm_err = ESP_FAIL;   // ← set gsm_err instead of returning directly
				}
			}
		}

		if (gsm_err == ESP_OK)
		{
			// ESP_LOGI("GSM", "Stack high watermark: %d", uxTaskGetStackHighWaterMark(NULL));
			gsm_err = configurl();
			if( gsm_err == ESP_OK)
			{
				ESP_LOGI("GSM", "Configured URL");
			}
			else
			{
				ESP_LOGE("GSM", "Failed to configure URL");
			}
			// // gsm_post_data("Hello !");
		}
	}
	else
	{
		ESP_LOGE("GSM", "Aborting further commands");
	}

	/* GSM command execution end */
	if (gsm_err == ESP_OK)
	{
		ESP_LOGI("GSM", "Executed init ");
	}
	else
	{
		ESP_LOGI("GSM", "Init Execution failed");
	}

	/* GPS Check */
	// if ( check_GPS_data || config.latitude == 0.0 || config.longitude == 0.0)
	// if(  get_gps_location(&latitude, &longitude) == ESP_OK  )
	// {
	// 	ESP_LOGI("GPS data","Fetch Success");
	// }

	gsm_cmd_checking = false;

	return gsm_err;
	// }
}

static esp_adc_cal_characteristics_t adc_chars;

void adc_init_custom()
{
	adc2_config_channel_atten(ADC_CH_1, ADC_ATTEN_DB);
	adc2_config_channel_atten(ADC_CH_2, ADC_ATTEN_DB);

	esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB, ADC_WIDTH_BIT, ADC_VREF, &adc_chars);
}

esp_err_t read_two_adc_voltages(float *voltage1, float *voltage2)
{
	int raw1 = 0, raw2 = 0;

	if (adc2_get_raw(ADC_CH_1, ADC_WIDTH_BIT, &raw1) != ESP_OK)
		return ESP_FAIL;
	if (adc2_get_raw(ADC_CH_2, ADC_WIDTH_BIT, &raw2) != ESP_OK)
		return ESP_FAIL;

	uint32_t mv1 = esp_adc_cal_raw_to_voltage(raw1, &adc_chars);
	uint32_t mv2 = esp_adc_cal_raw_to_voltage(raw2, &adc_chars);

	*voltage1 = mv1 / 1000.0f;
	*voltage2 = mv2 / 1000.0f;
	return ESP_OK;
}

void adc_sampling_task(void *pvParameters)
{
	uint32_t notificationValue;

	while (1)
	{
		xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);
		ESP_LOGI("ADC_samp", "Sampling started...");
		adc_sampling = true;

		TickType_t start_tick = xTaskGetTickCount();
		float sum1 = 0.0f, sum2 = 0.0f;
		int samples = 0;

		while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(5000))
		{
			float v1, v2;
			if (read_two_adc_voltages(&v1, &v2) == ESP_OK)
			{
				sum1 += v1;
				sum2 += v2;
				samples++;
			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}

		if (samples > 0)
		{
			sampled_voltage1 = sum1 / samples;
			sampled_voltage2 = sum2 / samples;
		}

		adc_sampling = false;
		ESP_LOGI("ADC_samp", "Sampling done: V1=%.3f V, V2=%.3f V", sampled_voltage1, sampled_voltage2);
	}
}

// Float version of map function
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float BatteryPercentage(float voltage)
{
	if (is_bat_15V)
	{
		// 15V
		float actualVolt;
		actualVolt = (36 * voltage) / 3;
		float percentage = mapFloat(actualVolt, 10, 16.8, 0, 100);
		// Clamp percentage
		if (percentage < 0)
		{
			// percentage = 0;
			return 0;
		}
		else if (percentage > 100)
		{
			// percentage = 100;
			return 100;
		}
		else
		{
			return percentage;
		}
	}
	else
	{
		// 12v
		float actualVolt;
		actualVolt = (36 * voltage) / 3;
		float percentage = mapFloat(actualVolt, 10, 13.6, 0, 100);
		// Clamp percentage
		if (percentage < 0)
		{
			// percentage = 0;
			return 0;
		}
		else if (percentage > 100)
		{
			// percentage = 100;
			return 100;
		}
		else
		{
			return percentage;
		}
	}
}

float SOLARPercentage(float voltage)
{
	float actualVolt;
	actualVolt = (54 * voltage) / 3;
	float percentage = mapFloat(actualVolt, 10, 24, 0, 100);
	// Clamp percentage
	if (percentage < 0)
	{
		// percentage = 0;
		return 0;
	}
	else if (percentage > 100)
	{
		// percentage = 100;
		return 100;
	}
	else
	{
		return percentage;
	}
}

uint64_t millis()
{
	return esp_timer_get_time() / 1000; // convert microseconds → ms
}

void update_and_print_time()
{
	struct tm current_time;
	readClock(&current_time);
	printf("Time: %02d:%02d:%02d %02d/%02d/%04d\n",
		   current_time.tm_hour, current_time.tm_min, current_time.tm_sec,
		   current_time.tm_mday, current_time.tm_mon + 1, current_time.tm_year + 1900);

	// status.current_date,
	strftime(status.current_date, sizeof(status.current_date), "%d-%m-%Y", &current_time);
	strftime(status.current_time, sizeof(status.current_time), "%H:%M:%S", &current_time);

	strftime(time_app_hms, sizeof(time_app_hms), "%H-%M-%S", &current_time);

	ESP_LOGI("POLL", "Date: %s, Time: %s", status.current_date, status.current_time);
}

void buzzer_beep(uint32_t on_ms, uint32_t off_ms, uint32_t repeat)
{
	printf("Buzzer Beep: ON for %ld ms, OFF for %ld ms, Repeat %ld times\n", on_ms, off_ms, repeat);

	for (uint32_t i = 0; i < repeat; i++)
	{
		gpio_set_level(BUZZER_GPIO, 1); // buzzer ON
		vTaskDelay(pdMS_TO_TICKS(on_ms));

		gpio_set_level(BUZZER_GPIO, 0); // buzzer OFF
		vTaskDelay(pdMS_TO_TICKS(off_ms));
	}
}
typedef enum
{
	FULL = 1,
	MEDIUM,
	EMPTY
} lvl_state_t;

lvl_state_t RW_tank = EMPTY, TW_tank = EMPTY;
lvl_state_t RW_tank_prev = EMPTY, TW_tank_prev = EMPTY;

void check_tank_lvls_1SENSOR()
{
	ESP_LOGW("TANK_LVL", "1 SENSOR | RW: H=%d L=%d | TW: H=%d L=%d",
             raw_tank_high, raw_tank_low, treated_tank_high, treated_tank_low);
    if      (raw_tank_high == 1 && raw_tank_low == 0) 	RW_tank = FULL;
    else if (raw_tank_high == 0 && raw_tank_low == 0)	RW_tank = MEDIUM;
    else if (raw_tank_high == 0 && raw_tank_low == 1)	RW_tank = EMPTY;     

    if 		(treated_tank_high == 1 && treated_tank_low == 0) TW_tank = FULL;
    else if (treated_tank_high == 0 && treated_tank_low == 0) TW_tank = MEDIUM;
    else if (treated_tank_high == 0 && treated_tank_low == 1) TW_tank = EMPTY;

}

void check_tank_lvls_2SENSOR()
{
	ESP_LOGW("TANK_LVL", "2 SENSOR | RW: H=%d L=%d | TW: H=%d L=%d",
             raw_tank_high, raw_tank_low, treated_tank_high, treated_tank_low);
    if      (raw_tank_high == 1 && raw_tank_low == 1) 	RW_tank = FULL;
    else if (raw_tank_high == 0 && raw_tank_low == 1)	RW_tank = MEDIUM;
    else if (raw_tank_high == 0 && raw_tank_low == 0)	RW_tank = EMPTY;     

    if 		(treated_tank_high == 1 && treated_tank_low == 1) TW_tank = FULL;
    else if (treated_tank_high == 0 && treated_tank_low == 1) TW_tank = MEDIUM;
    else if (treated_tank_high == 0 && treated_tank_low == 0) TW_tank = EMPTY;
}

typedef enum
{
	NONE,
	RAW_WATER_EMPTY,
	RAW_WATER_FULL,
	NORMAL_OPERATION,
	BACKWASH_OPERATION,
	TURN_OFF_SYSTEM
} rms_state_t;

rms_state_t CUR_RMS_STATE = NONE;
rms_state_t CUR_RW_tank_STATE = NONE;

void fill_RW_tank(lvl_state_t state)
{
	if(state == FULL && CUR_RW_tank_STATE != RAW_WATER_FULL)
	{
		modbus_write_and_confirm(Rly_RW, 0); // off motor
		CUR_RW_tank_STATE = RAW_WATER_FULL;
		
		printf(" ");
		ESP_LOGU("RMS", "Raw water Motor Turned OFF");
		printf(" ");
	}

	else if (state == EMPTY && CUR_RW_tank_STATE != RAW_WATER_EMPTY)
	{
		modbus_write_and_confirm(Rly_RW, 1); //on motor
		CUR_RW_tank_STATE = RAW_WATER_EMPTY;
		
		printf(" ");
		ESP_LOGU("RMS", "Raw water Motor Turned ON");
		printf(" ");
	}

}

void normal_operation()
{
	if (CUR_RMS_STATE == NORMAL_OPERATION)
		return;
	// Turn ON all relays
	modbus_write_and_confirm(Rly_SV1, 0);
	modbus_write_and_confirm(Rly_SV2, 1);
	modbus_write_and_confirm(Rly_SV3, 1);
	modbus_write_and_confirm(Rly_HP, 1);
	modbus_write_and_confirm(Rly_UV, 1);
	modbus_write_and_confirm(Rly_BackW, 0);
	modbus_write_and_confirm(HMI_BACKW_BTN, 0);

	CUR_RMS_STATE = NORMAL_OPERATION;
	printf(" ");
	ESP_LOGU("SYSTEM", "NORMAL_OPERATION");
	printf(" ");
}

void backwash_operation()
{
	if (CUR_RMS_STATE == BACKWASH_OPERATION)
		return;
	// Turn ON Backwash relay only
	modbus_write_and_confirm(Rly_SV1, 1);
	modbus_write_and_confirm(Rly_SV2, 0);
	modbus_write_and_confirm(Rly_SV3, 0);
	modbus_write_and_confirm(Rly_HP, 1);
	modbus_write_and_confirm(Rly_UV, 0);
	modbus_write_and_confirm(Rly_BackW, 1);
	modbus_write_and_confirm(HMI_BACKW_BTN, 1);

	CUR_RMS_STATE = BACKWASH_OPERATION;
	printf(" ");
	ESP_LOGU("SYSTEM", "Backwash relay turned ON");
	printf(" ");
}

void turn_off_system()
{
	if (CUR_RMS_STATE == TURN_OFF_SYSTEM)
		return;
	// Turn OFF all relays
	modbus_write_and_confirm(Rly_SV1, 0);
	modbus_write_and_confirm(Rly_SV2, 0);
	modbus_write_and_confirm(Rly_SV3, 0);
	modbus_write_and_confirm(Rly_HP, 0);
	modbus_write_and_confirm(Rly_UV, 0);
	modbus_write_and_confirm(Rly_BackW, 0);
	// modbus_write_and_confirm(Rly_RW, 0);
	modbus_write_and_confirm(HMI_BACKW_BTN, 0);

	CUR_RMS_STATE = TURN_OFF_SYSTEM;
	printf(" ");
	ESP_LOGU("SYSTEM", "All relays turned OFF");
	printf(" ");
}

// // UF VIZAG operations AS PER MALIK SIR 

// /*
//  * RMS Relay Control Functions
//  *
//  * SV1, SV2  — NO (Inlet SVs):    energised (1) = CLOSED | de-energised (0) = OPEN
//  * SV3       — NC (Backwash SV):  energised (1) = OPEN   | de-energised (0) = CLOSED
//  * SV4       — NC (Reject SV):    energised (1) = OPEN   | de-energised (0) = CLOSED
//  * Rly_HP    — Chlorine Dosing Pump: 1 = ON, 0 = OFF
//  * Rly_UV    — UV Lamp:              1 = ON, 0 = OFF
//  * Rly_BackW — Backwash Motor:       1 = ON, 0 = OFF
//  * Rly_RW    — Raw Water / Borewell: 1 = ON, 0 = OFF (controlled by fill_RW_tank only)
//  * 
//  * 
//  * RMS Relay Control Functions — Updated Mapping
//  *
//  * Rly_RW    — Borewell Pump:          fills raw water tank. Managed by fill_RW_tank() only.
//  * Rly_HP    — Raw Water Pump:         pumps water from raw tank exit into the system.
//  * Rly_BackW — Chlorine Dosing Pump:   doses during backwash only.
//  * Rly_UV    — UV Lamp:                ON during normal operation only.
//  *
//  * SV1, SV2  — NO (Inlet SVs):         energised (1) = CLOSED | de-energised (0) = OPEN
//  * SV3       — NC (Backwash SV):       energised (1) = OPEN   | de-energised (0) = CLOSED
//  * SV4       — NC (Reject SV):         energised (1) = OPEN   | de-energised (0) = CLOSED
//  */
 
  
// void normal_operation()
// {
//     if (CUR_RMS_STATE == NORMAL_OPERATION)
//         return;
 
//     /*
//      * Normal Operation:
//      *   SV1 (NO, Inlet) → 0 = de-energised = OPEN  → raw water flows in
//      *   SV2 (NO, Inlet) → 0 = de-energised = OPEN  → raw water flows in
//      *   SV3 (NC, BW SV) → 0 = de-energised = CLOSED → backwash path sealed
//      *   SV4 (NC, Reject)→ 0 = de-energised = CLOSED → reject path sealed
//      *   Rly_HP  (RW Pump)   → 1 = ON  (pumps water from raw tank into system)
//      *   Rly_UV              → 1 = ON
//      *   Rly_BackW (Dosing)  → 0 = OFF
//      */
//     modbus_write_and_confirm(Rly_SV1,  0); // NO de-energised → OPEN  (inlet flows)
//     modbus_write_and_confirm(Rly_SV2,  0); // NO de-energised → OPEN  (inlet flows)
//     modbus_write_and_confirm(Rly_SV3,  0); // NC de-energised → CLOSED (BW path sealed)
//     modbus_write_and_confirm(Rly_SV4,  0); // NC de-energised → CLOSED (reject path sealed)
//     modbus_write_and_confirm(Rly_HP,   1); // Raw water pump ON
//     modbus_write_and_confirm(Rly_UV,   1); // UV ON
//     modbus_write_and_confirm(Rly_BackW,0); // Dosing pump OFF
//     modbus_write_and_confirm(HMI_BACKW_BTN, 0);
 
//     CUR_RMS_STATE = NORMAL_OPERATION;
//     printf(" ");
//     ESP_LOGU("SYSTEM", "NORMAL_OPERATION");
//     printf(" ");
// }
 
// void backwash_operation()
// {
//     if (CUR_RMS_STATE == BACKWASH_OPERATION)
//         return;
 
//     /*
//      * Backwash Operation:
//      *   SV1 (NO, Inlet) → 1 = energised = CLOSED → inlet blocked
//      *   SV2 (NO, Inlet) → 1 = energised = CLOSED → inlet blocked
//      *   SV3 (NC, BW SV) → 1 = energised = OPEN   → backwash path open
//      *   SV4 (NC, Reject)→ 1 = energised = OPEN   → reject/drain path open
//      *   Rly_HP  (RW Pump)   → 1 = ON  (pumps water from raw tank for backwash)
//      *   Rly_UV              → 0 = OFF
//      *   Rly_BackW (Dosing)  → 1 = ON  (doses during backwash)
//      */
//     modbus_write_and_confirm(Rly_SV1,  1); // NO energised   → CLOSED (inlet blocked)
//     modbus_write_and_confirm(Rly_SV2,  1); // NO energised   → CLOSED (inlet blocked)
//     modbus_write_and_confirm(Rly_SV3,  1); // NC energised   → OPEN   (BW path open)
//     modbus_write_and_confirm(Rly_SV4,  1); // NC energised   → OPEN   (reject path open)
//     modbus_write_and_confirm(Rly_HP,   1); // Raw water pump ON
//     modbus_write_and_confirm(Rly_UV,   0); // UV OFF
//     modbus_write_and_confirm(Rly_BackW,1); // Dosing pump ON
//     modbus_write_and_confirm(HMI_BACKW_BTN, 1);
 
//     CUR_RMS_STATE = BACKWASH_OPERATION;
//     printf(" ");
//     ESP_LOGU("SYSTEM", "Backwash relay turned ON");
//     printf(" ");
// }
 
// void turn_off_system()
// {
//     if (CUR_RMS_STATE == TURN_OFF_SYSTEM)
//         return;
 
//     /*
//      * Turn Off System — everything off, inlet physically closed:
//      *   SV1 (NO, Inlet) → 1 = energised = CLOSED → inlet physically shut
//      *   SV2 (NO, Inlet) → 1 = energised = CLOSED → inlet physically shut
//      *   SV3 (NC, BW SV) → 0 = de-energised = CLOSED → backwash path sealed
//      *   SV4 (NC, Reject)→ 0 = de-energised = CLOSED → reject path sealed
//      *   Rly_HP (Dosing)  → 0 = OFF
//      *   Rly_UV           → 0 = OFF
//      *   Rly_BackW        → 0 = OFF
//      *   Rly_RW (Borewell)→ managed by fill_RW_tank()
// 	 *   
//      */
//     modbus_write_and_confirm(Rly_SV1,  1); // NO energised   → CLOSED (inlet shut)
//     modbus_write_and_confirm(Rly_SV2,  1); // NO energised   → CLOSED (inlet shut)
//     modbus_write_and_confirm(Rly_SV3,  0); // NC de-energised → CLOSED (BW sealed)
//     modbus_write_and_confirm(Rly_SV4,  0); // NC de-energised → CLOSED (reject sealed)
//     modbus_write_and_confirm(Rly_HP,   0); // Raw water pump OFF
//     modbus_write_and_confirm(Rly_UV,   0); // UV OFF
//     modbus_write_and_confirm(Rly_BackW,0); // Dosing pump OFF
//     // Rly_RW intentionally not controlled here — fill_RW_tank() manages borewell pump
//     modbus_write_and_confirm(HMI_BACKW_BTN, 0);
 
//     CUR_RMS_STATE = TURN_OFF_SYSTEM;
//     printf(" ");
//     ESP_LOGU("SYSTEM", "All relays turned OFF");
//     printf(" ");
// }


bool frst_run_rms = true;
// config.num_level_sensor
void rms_control()
{
	// ESP_LOGI("POLL", "RMS Control Check");
	static uint64_t periodic_last_ms = 0;
	static uint64_t PERIODIC_MINUTES = 0; // <-- set X minutes here
	static uint64_t periodic_interval_ms = 0;

	volatile uint64_t now_ms = millis();
	if (periodic_last_ms == 0)
	{
		PERIODIC_MINUTES = config.Backwash_frequency_min; // <-- set X minutes here
		periodic_interval_ms = PERIODIC_MINUTES * 60ULL * 1000ULL;
		periodic_last_ms = now_ms; // initialize on first run
		printf("BW Freqy %llu mins\nInterval ms: %llu\n", PERIODIC_MINUTES, periodic_interval_ms);
	}

	// 2. DYNAMIC UPDATE FIX: Check if config changed at runtime
	if (PERIODIC_MINUTES != config.Backwash_frequency_min)
	{
		ESP_LOGW("RMS_CONTROL", "Backwash Freq changed from %llu to %ld mins", PERIODIC_MINUTES, config.Backwash_frequency_min);

		// Update local static variables to match new config
		PERIODIC_MINUTES = config.Backwash_frequency_min;
		periodic_interval_ms = PERIODIC_MINUTES * 60ULL * 1000ULL;

		// Optional: Reset the timer so the new interval starts counting from NOW.
		// If you don't reset this, switching from 60min -> 1min might trigger an immediate backwash.
		// periodic_last_ms = now_ms;
	}

	if (CUR_RMS_STATE == TURN_OFF_SYSTEM)
		ESP_LOGU("SYSTEM", "---TURN_OFF_SYSTEM---");
	else if (CUR_RMS_STATE == NORMAL_OPERATION)
		ESP_LOGU("SYSTEM", "---NORMAL_OPERATION---");
	else if (CUR_RMS_STATE == BACKWASH_OPERATION)
		ESP_LOGU("SYSTEM", "---BACKWASH_OPERATION---");

	if (CUR_RW_tank_STATE == RAW_WATER_EMPTY)
		ESP_LOGU("SYSTEM", "---RAW_WATER_EMPTY---");
	else if (CUR_RW_tank_STATE == RAW_WATER_FULL)
		ESP_LOGU("SYSTEM", "---RAW_WATER_FULL---");

	if (sensors[SYS_en].value == 1.0)
	{
		if( config.num_level_sensor == 1)		check_tank_lvls_1SENSOR();
		else if( config.num_level_sensor == 2)	check_tank_lvls_2SENSOR();
		
		/* Priority 1 : Shut down system if PRS > Delta */
		if (config.PS_enabled_DDPS_disabled == 0 && ddps_switch == 1)
		{
			turn_off_system();
			ESP_LOGU("TURN_OFF_SYSTEM", "PS_DDPS_1_ENABLED& DDPS SWITCH 1 - System Turned OFF");
			return;
		}
		else if (config.PS_enabled_DDPS_disabled == 1 && config.PS_delta > 0)
		{
			if ((sensors[Pressure1].value - sensors[Pressure2].value) > config.PS_delta)
			{
				turn_off_system();
				ESP_LOGU("TURN_OFF_SYSTEM", "PS_DDPS_1_DELTA_EXCEEDED - System Turned OFF");
				return;
			}
		}

		/* Priority 2 : Periodic Backwash check */
		now_ms = millis();
		if (RW_tank != EMPTY && ( (PERIODIC_MINUTES > 0 && ((now_ms - periodic_last_ms) >= periodic_interval_ms)) || frst_run_rms)) // Tank is not empty and time has crossed limit
		{	
			if (RW_tank == FULL)
			{
				fill_RW_tank(FULL);
			}
			frst_run_rms = false;
			// advance by interval (use += to avoid drift on small delays)
			periodic_last_ms = millis();
			backwash_operation();
			printf(" ");
			ESP_LOGU("RMS_CONTROL", "Performing periodic BackWash operation");
			printf(" ");
			return;
		}
		else
		{
			// printf("perodic_last_ms: %llu | now_ms: %llu | diff: %llu | interval_ms: %llu\n", periodic_last_ms, now_ms, (now_ms - periodic_last_ms), periodic_interval_ms);
		}

		if (CUR_RMS_STATE == BACKWASH_OPERATION && (now_ms - periodic_last_ms) >= (config.Backwash_time_sec * 1000ULL))
		{ // perform backwash for config.Backwash_time_sec seconds

			printf("perodic_last_ms: %llu | now_ms: %llu | diff: %llu | interval_ms: %llu\n", periodic_last_ms, now_ms, (now_ms - periodic_last_ms), periodic_interval_ms);
			turn_off_system();
			printf(" ");
			ESP_LOGU("RMS_CONTROL", "BackWash duration over. Turning off system");
			printf(" ");
		}
		else if (sensors[HMI_BACKW_BTN].value == 1.0 && CUR_RMS_STATE != BACKWASH_OPERATION)
		{ // If sysem is on and HMI backwash button is pressed, reset it.
			modbus_write_and_confirm(HMI_BACKW_BTN, 0);
		}
		else if (RW_tank != EMPTY && CUR_RMS_STATE == BACKWASH_OPERATION)
		{
			// In backwash operation, skip other checks.
			return;
		}

		/* Priority 3 : Normal operation checks */
		if (RW_tank == EMPTY) // RW_tank EMPTY		-   RW_pump ON
		{
			turn_off_system();
			delay_ms(500);
			fill_RW_tank(EMPTY);
			return;
		}
		else if(RW_tank == FULL) // RW_tank Medium  - RW_pump ON
		{
			fill_RW_tank(FULL);
		}

		if ( (RW_tank == FULL || RW_tank == MEDIUM) && TW_tank == EMPTY )// RW_level (HIGH or Not Empty) & TW_level (not full) - Normal Operation
		{
			normal_operation();
		}
		else if ((RW_tank == FULL || RW_tank == MEDIUM) && TW_tank == MEDIUM )
		{
			if( TW_tank_prev == FULL)	turn_off_system();
			else if (TW_tank_prev == EMPTY)	normal_operation();
		}
		else if (TW_tank == FULL) // TW_level HIGH  -   Sys OFF
		{
			turn_off_system();
			ESP_LOGU("TURN_OFF_SYSTEM", "TREATED TANK HIGH - System Turned OFF");
		}

		// system ON conditions checked above.

		if		( RW_tank == FULL && RW_tank_prev != FULL)		RW_tank_prev = FULL;
		else if ( RW_tank == EMPTY && RW_tank_prev != EMPTY)	RW_tank_prev = EMPTY;
  
		if		( TW_tank == FULL && TW_tank_prev != FULL)		TW_tank_prev = FULL;
		else if ( TW_tank == EMPTY && TW_tank_prev != EMPTY)	TW_tank_prev = EMPTY;

	}
	// else if (sensors[HMI_BACKW_BTN].value == 1.0)
	// { // If sysem is off and HMI backwash button is pressed, do backwash.
	// 	backwash_operation();
	// }

	// FOR UF VIZAG ONLY 
	//START
	else if (sensors[HMI_BACKW_BTN].value == 1.0 && RW_tank == EMPTY)
	{ // Turn off backwsh HMI btn when system OFF and btn is pressed ON.
		// Tank is empty — cannot perform backwash, reset button
		modbus_write_and_confirm(HMI_BACKW_BTN, 0);
		ESP_LOGW("RMS_CONTROL", "BW Button pressed but RW Tank EMPTY — resetting button");
		// BW button ON + tank EMPTY, Reset button to 0 immediately, do not enter backwash state
	}

	else if (sensors[HMI_BACKW_BTN].value == 1.0 && RW_tank != EMPTY)
	{
		backwash_operation(); //BW button ON + tank NOT empty, then Run backwash
	}
	//END
	
	else
	{		
		fill_RW_tank(FULL);
		delay_ms(500);
		turn_off_system();
		ESP_LOGU("TURN_OFF_SYSTEM", "SYSTEM_EN OFF SYSTEM TURNED OFF");
	}
}

void update_rms_config_register()
{
	static uint8_t last_sent_mode = 0;
	uint8_t current_mode = config.num_level_sensor; // Assuming this is the config variable that determines the mode

	if (last_sent_mode != current_mode)
	{
		ESP_LOGW("RMS_UPDATE", "Level sensors 1/2 Mode changed from %d to %d. Updating RMS_SENSOR_CFG register.", last_sent_mode, current_mode);

		if (modbus_write_and_confirm(RMS_SENSOR_CFG, (uint16_t)current_mode) == ESP_OK)
		{
            ESP_LOGW("RMS_UPDATE", "PCB register 0x0015 written: %d — PCB ACK OK", current_mode);
            ESP_LOGW("RMS_UPDATE", "Current sensor bits from PCB: RW_H=%d RW_L=%d TW_H=%d TW_L=%d",
                     raw_tank_high, raw_tank_low, treated_tank_high, treated_tank_low);
            ESP_LOGI("RMS_UPDATE", "Update Success");
            last_sent_mode = current_mode; // Update tracker only on success
			/* Reset state machine so stale CUR_RMS_STATE and prev tank values
               from old mode don't block rms_control() from acting on new sensor readings */
            CUR_RMS_STATE  = NONE;
            RW_tank_prev   = MEDIUM;
            TW_tank_prev   = MEDIUM;
        }
		else
        {
            ESP_LOGE("RMS_UPDATE", "PCB write FAILED for reg 0x0015 — PCB may not support RMS_SENSOR_CFG. Retry next cycle.");
            ESP_LOGW("RMS_UPDATE", "ESP32 will still use num_level_sensor=%d for local logic.", current_mode);
        	ESP_LOGE("RMS_UPDATE", "Update Failed - Will retry next cycle");
        }
		
	}
}

void hmi_tds_update()
{
	// Update TDS values on HMI
	modbus_write_and_confirm(HMI_TDS_IN, (uint16_t)(sensors[TDS_IN].value));
	modbus_write_and_confirm(HMI_TDS_OUT, (uint16_t)(sensors[TDS_OUT].value));
	modbus_write_and_confirm(HMI_TEMPW_IN, (uint16_t)(sensors[TEMPW_IN].value) * 10);
	modbus_write_and_confirm(HMI_TEMPW_OUT, (uint16_t)(sensors[TEMPW_OUT].value) * 10);
	// ESP_LOGI("SYS", "HMI update Done.");
}

void rms_task(void *pvParameters)
{
	while (1)
	{
		poll_sensor();
		update_rms_config_register();
		rms_control();
		hmi_tds_update();
		vTaskDelay(pdMS_TO_TICKS(50)); // Check every 500 ms
	}
}

void ota_func()
{
	ota_in_progress = true;
	bool sim;
	if (strstr(config.gsm_sim_name, "JIO"))
	{
		sim = 0;
	}
	else
	{
		sim = 1;
	}

	// --- MODIFIED SECTION START ---
	ESP_LOGW("MAIN", "Checking for OTA updates...");
	if (config.wifi_enabled_gsm_disabled == true)
	{
		ESP_LOGW("MAIN", "WiFi Enabled & GSM Disabled mode active");

		bool wifi_connected = false;

		// Check if already connected
		if (wifi_is_connected())
		{
			wifi_connected = true;
		}
		else
		{
			ESP_LOGW("MAIN", "Connecting to WiFi for OTA...");

			// Retry Loop: Try to connect 3 times
			for (int i = 0; i < 3; i++)
			{
				if (wifi_handler_init((uint8_t *)config.wifi_ssid, (uint8_t *)config.wifi_password) == ESP_OK)
				{
					// Wait briefly to ensure IP is assigned
					vTaskDelay(pdMS_TO_TICKS(2000));

					if (wifi_is_connected())
					{
						ESP_LOGW("MAIN", "WiFi Connected Successfully (Attempt %d)", i + 1);
						wifi_connected = true;
						break;
					}
				}
				else
				{
					ESP_LOGE("MAIN", "WiFi Connection Attempt %d Failed, Retrying...", i + 1);
					vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds before retrying
				}
			}
		}

		if (wifi_connected)
		{
			start_ota_wifi();
		}
		else
		{
			ESP_LOGE("MAIN", "Could not connect to WiFi for OTA after multiple attempts.");
		}
	}
	else
	{
		ESP_LOGW("MAIN", "GSM Enabled mode active");
		gpio_set_level(GSM_RST_GPIO, 0);
		write_SR(GSM_en, HIGH);
		// Ensure GSM has enough time to stabilize
		delay_ms(8000);
		check_fw_update(sim);
	}

	// Set status to true so we don't loop OTA checks forever if they fail
	// set_ota_checked_status(true);
	esp_restart(); // Restart to apply any updates
}

/* post_data() — runs inside backlog_post_task (6144-byte stack), never from app_main */
void post_data(void)
{
    bool backlog_pending = get_is_failed();

    if (gsm_requires_init) {
        write_SR(GSM_en, HIGH);
        ESP_LOGW("GSM_PWR", "GSM modem ON — waiting for boot...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    if (config.wifi_enabled_gsm_disabled == true) {
        ESP_LOGW("MAIN", " WIFI Mode Active");
        handle_wifi_post();
    } else {
        ESP_LOGW("MAIN", " GSM Mode Active");
        handle_gsm_post();
    }

    if (!check_GPS_data && !backlog_pending) {
        ESP_LOGW("GSM_PWR", "Powering OFF GSM modem...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(GSM_RST_GPIO, 1);
        write_SR(GSM_en, LOW);
        gsm_requires_init = true;
        ESP_LOGW("GSM_PWR", "GSM modem OFF. Will cold-start next cycle.");
    } else if (!check_GPS_data && backlog_pending) {
        ESP_LOGW("GSM_PWR", "Backlog active — keeping GSM modem ON for faster drain");
    }
}

/* backlog_post_task — dedicated 6144-byte stack for all HTTP/GSM posting.
   Wakes on TaskNotify from main loop (every log cycle) or self-triggers
   while backlog remains. Stack is sized to hold esp_http_client_perform + TLS. */
void backlog_post_task(void *pvParameters)
{
    while (1) {
        /* Block until main loop writes a log or a batch completes */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!ble_cmd_processing) {
            ESP_LOGW("POST_WORKER", "Stack free before post: %d bytes",
                     uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));

            /* Print pending backlog count */
            if (get_is_failed()) {
                size_t f_off  = get_failed_offset();
                size_t c_off  = get_curr_log_offset();
                size_t f_size = get_file_size();
                size_t pending_bytes = (c_off >= f_off)
                                       ? (c_off - f_off)
                                       : (f_size - f_off + c_off);   /* wrapped */
                size_t pending_logs  = pending_bytes / MAX_LINE_LEN;
                ESP_LOGW("BACKLOG", "Pending backlog: ~%d entries (%d bytes) | failed_off=%d curr_off=%d",
                         pending_logs, pending_bytes, f_off, c_off);
            } else {
                ESP_LOGI("BACKLOG", "No pending backlog.");
            }

            post_data();
            ESP_LOGW("POST_WORKER", "Stack free after post:  %d bytes",
                     uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
        }

        /* If backlog still has entries, self-trigger for next batch */
        if (get_is_failed()) {
            vTaskDelay(pdMS_TO_TICKS(1000));   /* 1 s gap between backlog batches */
            xTaskNotifyGive(backlog_post_task_handle);
        }
    }
}

void app_main(void)
{

	// vTaskDelay(2000 / portTICK_PERIOD_MS);

	set_UART1_rx_frame_len(10);
	// set_UART2_rx_frame_len(10);

	/* UART Init */

#ifdef DEBUG == DEFAULT
	UART1_init();
	start_UART1_task();
	UART2_init();
	start_UART2_task();

#elif DEBUG == LOG_AT_UART2
	UART0_init();
	UART1_init();
	start_UART0_task();
	start_UART1_task();

#elif DEBUG == LOG_AT_UART1
	UART0_init();
	UART2_init();
	start_UART0_task();
	start_UART2_task();

#elif DEBUG == DISABLE_LOGS
	UART0_init();
	UART1_init();
	UART2_init();
	start_UART0_task();
	start_UART1_task();
	start_UART2_task();
#endif

	/*NVS Read Section */
	esp_err_t err = nvs_flash_init();

	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}

	ESP_ERROR_CHECK(err);

	// clear_sensor_nvs_namespace();
	// clear_device_status_nvs();
	// clear_device_config_nvs(); // CLEAR the sensor data if required.

	// Try to load saved sensors, else set defaults
	if (load_sensors_from_nvs() != ESP_OK)
	{
		// Set default values manually
		ESP_LOGW(TAG_NVS, "Using default sensor config");
		init_default_sensors(); // You define this function to populate sensors[]
		save_sensors_to_nvs();	// Save them for future
	}
	else
	{
		printf("NVS load Success \n");
	}

	/*NVS Read Section Ends*/

	// delete_spiffs_file("/spiffs/data_logs.txt");
	// delete_log_file();
	// log_handler_init();

	gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);

	printf("Hello from app_main!\n");

	if (load_sensors_from_nvs() == ESP_OK)
	{
		// print_sensor_data();
	}

	if (load_device_status_from_nvs(&status) != ESP_OK)
	{
		init_default_device_status(&status);
		save_device_status_to_nvs(&status);
	}

	if (load_device_config_from_nvs(&config) != ESP_OK)
	{
		init_default_device_config(&config);
		save_device_config_to_nvs(&config);
	}

	if (load_device_status_from_nvs(&status) == ESP_OK)
	{
		print_device_status(&status);
	}

	if (load_device_config_from_nvs(&config) == ESP_OK)
	{
		print_device_config(&config);
	}

	// #ifdef DEBUG == DEFAULT

	// #elif DEBUG == LOG_AT_UART2
	// // esp_log_set_vprintf(uart_vprintf);

	// #elif DEBUG == LOG_AT_UART1
	// // esp_log_set_vprintf(uart_vprintf);

	// #elif DEBUG == DISABLE_LOGS
	// esp_log_level_set("*", ESP_LOG_NONE); // Disable all logs
	// #endif

	// ESP_LOGI("UART2", "UART2 initialized");

	rtc_init_();
	init_flow_buzz_gpio();
	init_shift_register();
	init_temperature_sensor();
	adc_init_custom();
	init_log_system();

	// format_spiffs();
	printf("FILE SIZE: %d bytes\n", get_file_size());
	if (get_file_size() <= MIN_SPACE_FOR_LOG)
	{
		ESP_LOGI("MAIN", "Enough space available for logs file size %d", get_file_size());
		check_last_log_ind();
	}
	else
	{
		check_last_log_ind();
		size_t curr_offset = get_curr_log_offset();
		ESP_LOGW("MAIN", "Wrap around took place");
		ESP_LOGW("MAIN", "Current Log offset: %zu ", curr_offset);
	}

	ESP_LOGI("MAIN", "Log system initialized");
	size_t last_log_off = get_curr_log_offset();
	size_t failed_offset = get_failed_offset();
	bool failed_flag = get_is_failed();
	bool post_wrap = get_post_wrap();

	if (!failed_flag)
	{
		set_post_status(last_log_off, post_wrap, failed_flag);
	}

	ESP_LOGE("MAIN", "last log offset: %d, failed offset: %d, failed flag: %d", last_log_off, failed_offset, failed_flag);

	BLE_INT_QUEUE = xQueueCreate(1, sizeof(int)); // queue length = 1
	BLE_INT_POLL_PAUSED = xSemaphoreCreateBinary();
	BLE_INT_BLE_DONE = xSemaphoreCreateBinary();


	update_and_print_time();
	unix_timestamp = convert_to_unix(status.current_date, status.current_time);
	// if( is_ble_active() == false)
	// initiate_app_sequence();
	if (unix_timestamp != -1)
	{
		printf("Unix Timestamp: %lld\n", unix_timestamp);
	}
	else
	{
		printf("Invalid date/time format!\n");
		unix_timestamp = 1754894220;
	}

	generate_compact_summary_string();
	printf("Compact Summary: %s\n", compact_summary_str);

	gpio_set_level(GSM_RST_GPIO, 1);
	write_SR(GSM_en, LOW);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	write_SR(GSM_en, HIGH);
	gpio_set_level(GSM_RST_GPIO, 0);
	write_SR(IO_12V, HIGH);
	write_SR(LED, LOW);

	// ESP_LOGW("APP", "Waiting delay for GSM stability");
	// vTaskDelay(5000 / portTICK_PERIOD_MS);
	// while(1)
	// {
	// 	ESP_LOGW("LOOP", "GSM RESET HIGH");
	// 	gpio_set_level(GSM_RST_GPIO, 1);

	// 	vTaskDelay(20000 / portTICK_PERIOD_MS);

	// 	ESP_LOGW("LOOP", "GSM RESET LOW");
	// 	gpio_set_level(GSM_RST_GPIO, 0);

	// 	vTaskDelay(20000 / portTICK_PERIOD_MS);
	// }

	bool sim;
	if (strstr(config.gsm_sim_name, "JIO"))
	{
		sim = 0;
	}
	else
	{
		sim = 1;
	}

	int ota_status = false;
	bool skip_ota = true;
	get_ota_checked_status(&ota_status);

	if (strcmp(config.server_name, DEVICE_ID) == 0)
	{
		printf("The Device name is not set by USER, skipping OTA check!\n");
		skip_ota = false;
	}

	// ... inside app_main ...
		/* RTC Data  */
	struct tm current_time;
	readClock(&current_time);

	xTaskCreatePinnedToCore(btn_press_handler, "btn_press_handler", 8192, NULL, 5, NULL, 1);
	delay_ms(100);

	// // TEMP TEST — force OTA check on next boot, remove after testing
	// nvs_handle_t test_nvs;
	// if (nvs_open("storage", NVS_READWRITE, &test_nvs) == ESP_OK) {
	// 	nvs_set_i32(test_nvs, "last_day", -1);  // invalid day, always triggers
	// 	nvs_commit(test_nvs);
	// 	nvs_close(test_nvs);
	// 	ESP_LOGW("TEST", "Forced last_day = -1, OTA will trigger on next boot");
	// }

		// Restart once in a day		|    Also refresh the GPS location once !
	if (strcmp(config.server_name, DEVICE_ID) != 0)
		daily_restart_check(&current_time);

		// write_SR(IO_12V,HIGH);
	write_SR(SEN_RS485_en, HIGH);
	write_SR(GSM_en, HIGH);
	gpio_set_level(GSM_RST_GPIO, 0);

	// --- MODIFIED SECTION END ---
	delay_ms(500); // Wait for GSM to stabilize
	xTaskCreate(adc_sampling_task, "adc_sampling_task", 4096, NULL, 5, &adc_sampling_handle);
	xTaskCreate(rms_task, "rms_task", 4096, NULL, 5, &rms_task_handle);
	xTaskCreate(backlog_post_task, "backlog_post_task", 6144, NULL, 5, &backlog_post_task_handle);

	write_SR(Lamp, HIGH);
	buzzer_beep(1000, 0, 1);

	/* Kick post worker once at boot — drains any pre-existing backlog from last session */
	if (backlog_post_task_handle) {
		xTaskNotifyGive(backlog_post_task_handle);
	}

	uint8_t state = 0;

	// start_delay();
	uint64_t start = millis();
	uint32_t delay_ms = config.data_frequency_sec * 1000; // 60 sec
	if (delay_ms == 0)
		delay_ms = 5000;

	// delay_ms = 10000;
	ESP_LOGW("MAIN", " Data Frequency set to: %ld", delay_ms);

	uint32_t pre_trigger_ms = 1000; // 5 sec before end
	bool pre_done = false;
	bool first_run = true;


	// vTaskDelay(3000 / portTICK_PERIOD_MS);	// Stabilize Sensors

	// xTaskNotify(gsm_task_handle, 0, eNoAction);

	// get_sensor_data();
	// get_device_config();
	// get_device_status();

	static int logss_count = 0;
	// print_all_logs_timestamp();

	while (1)
	{
		// check_btn_press();

		uint64_t now = millis();
		uint64_t elapsed = now - start;

		if ((!pre_done && elapsed >= (delay_ms - pre_trigger_ms)) || first_run)
		{
			// printf("⚡ Pre-trigger event at %llu ms\n", elapsed);
			pre_done = true;

			// xTaskNotify(gsm_task_handle, 0, eNoAction);
		}

		if ((elapsed >= delay_ms || first_run) && pre_done && !ble_cmd_processing)
		{
			ESP_LOGW("MAIN", "\n✅ Main delay done at %llu ms\n", elapsed);

			xTaskNotify(adc_sampling_handle, 0, eNoAction);
			// xTaskNotify(flow_task_handle, 0, eNoAction);	// Start counting the flow
			// xTaskNotify(tempr_task_handle, 0, eNoAction);	// Start checking the temperature
			printf(" \n");

			// CHECK_BLE_INTERRUPT();
			if (strcmp(config.server_name, DEVICE_ID) == 0)
			{
				printf("The Device name is not set by USER, skippiing post sequence!\n");
			}
			else
			{
				data_post_sequence();
			}

			if (is_ble_connected_())
			{
				CHECK_BLE_INTERRUPT();
				get_sensor_data();
				status_notify_chunks(all_sensor_info);
				ESP_LOGB("MAIN-BLE", "NOTIFIED SENSOR DATA");
				get_device_status();
				status_notify_chunks(device_status_str);
				ESP_LOGB("MAIN-BLE", "NOTIFIED DEVICE STATUS");
				get_device_config();
				status_notify_chunks(device_config_str);
				ESP_LOGB("MAIN-BLE", "NOTIFIED DEVICE CONFIG");
			}

			start = now; // restart delay
			pre_done = false;
			first_run = false;
		}

		vTaskDelay(pdMS_TO_TICKS(50));

		// If BLE is connected update every 5 second
		if (is_ble_connected_() && delay_ms > 5000)
		{
			delay_ms = 5000;
		}
		// state = ! state;
		// write_SR(LED, !state);
	}

	ESP_LOGI("MAIN", "Exiting main loop");
	print_posted_logs();
}