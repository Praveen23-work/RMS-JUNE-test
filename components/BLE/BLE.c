#include <stdio.h>
#include "BLE.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "esp_system.h"


static const char *TAG1 = "BLE_STOP";

void esp__reset() {
    // Do something
    esp_restart();  // Reboots the ESP32
}


#define MAX_BLE_READ_DATA_LEN 512
char ble_data_buffer[MAX_BLE_READ_DATA_LEN] = {0};

#define MAX_BLE_NAME_LEN 32
char ble_device_name[MAX_BLE_NAME_LEN] = "MyESP32";  // default name


uint8_t ble_addr_type;
struct ble_gap_adv_params adv_params;
bool status = false;
bool ble_ready = false;
volatile bool ble_connected = false;
bool ble_stop_requested = false;

static uint16_t current_conn_handle = 0;
uint16_t status_char_handle;

static int ble_data_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_config_write(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int dummy_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
                                                      
void ble_app_advertise(void);


static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xFFF1),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = ble_config_write
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFFF2),
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = ble_data_read
            },
			{
			    .uuid = BLE_UUID16_DECLARE(0xFFF3),
			    .access_cb = dummy_access_cb,  // ✅ must exist
			    .flags = BLE_GATT_CHR_F_NOTIFY,
			    .val_handle = &status_char_handle
			},
            { 0 }
        }
    },
    { 0 }
};


void set_ble_data(const char *data) {
    strncpy(ble_data_buffer, data, MAX_BLE_READ_DATA_LEN - 1);
    ble_data_buffer[MAX_BLE_READ_DATA_LEN - 1] = '\0'; // Ensure null termination
}

static int ble_data_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    printf("BLE Read Requested: %s\n", ble_data_buffer);
    os_mbuf_append(ctxt->om, ble_data_buffer, strlen(ble_data_buffer));
    return 0;
}


char latest_ble_command[100] = {0};
char latest_ble_command_temp[100] = {0};

// static int ble_config_write(uint16_t conn_handle, uint16_t attr_handle,
//                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
//     memset(latest_ble_command, 0, sizeof(latest_ble_command));
//     memcpy(latest_ble_command, ctxt->om->om_data, ctxt->om->om_len);
//     latest_ble_command[ctxt->om->om_len] = '\0';  // Null-terminate
//     // ESP_LOGI("BLE_CONFIG", "Received: %s", latest_ble_command);
//     ESP_LOGI("BLE_CONFIG", "Received %d bytes: %s", ctxt->om->om_len, latest_ble_command);
//     return 0;
// }

#define MAX_COMMAND_BUFFER 4096

static char ble_rx_buffer[MAX_COMMAND_BUFFER];
static size_t ble_command_offset = 0;
static bool ble_rx_done = false;

static int ble_config_write(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    size_t len = ctxt->om->om_len;

    // Check buffer overflow
    if ((ble_command_offset + len) >= MAX_COMMAND_BUFFER) {
        ESP_LOGE("BLE_CONFIG", "Buffer overflow! Resetting buffer.");
        ble_command_offset = 0;
        memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // Append received chunk
    memcpy(&ble_rx_buffer[ble_command_offset], ctxt->om->om_data, len);
    ble_command_offset += len;

    if (ble_rx_buffer[0] == '{' && ble_rx_buffer[1] == '$') 
    {
    // Valid command, continue processing
    ESP_LOGI("BLE_CONFIG", "Valid command received: %s", ble_rx_buffer);
    } 
    else {
        // Invalid command, clear buffer
        ESP_LOGW("BLE_CONFIG", "Invalid command format - %s, clearing buffer.",ble_rx_buffer);
        ble_command_offset = 0;
        memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
        ble_rx_done = false;
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // Check if message is complete (e.g., ends with '\n')
    if (ble_rx_buffer[ble_command_offset - 1] == '}') {
        ble_rx_buffer[ble_command_offset] = '\0';  // Null-terminate
        ESP_LOGI("BLE_CONFIG", "Full command received: %s", ble_rx_buffer);
        ESP_LOGI("BLE_CONFIG", "First 20 bytes:'%.*s'", 20, ble_rx_buffer);

        // TODO: Process the complete command here...
        ble_rx_done = true;

        // Reset for next command
        // ble_command_offset = 0;
        // memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
    } else {
        // ESP_LOGI("BLE_CONFIG", "Chunk received (%d bytes), waiting for more...", (int)len);
        // ESP_LOGI("BLE_CONFIG", "Chunk rxd: %s", ble_rx_buffer);
    }

   

    return 0;
}

const char *read_ble_rx_buffer(void) {
    if (ble_command_offset == 0) {
        ESP_LOGW("BLE_CONFIG", "No data received yet");
        return NULL;
    }
    ble_command_offset = 0;
    // Ensure null-termination
    // ble_rx_buffer[ble_command_offset] = '\0';

    ESP_LOGI("BLE_CONFIG", "Received: %s", ble_rx_buffer);
    printf("ble_rx_buffer ptr address: %p\n", (void *)ble_rx_buffer);
    // Return the full command
    return ble_rx_buffer;
}

void reset_ble_rx_buffer(void) {
    ble_command_offset = 0;
    ble_rx_buffer[ble_command_offset] = '\0';
    memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
    ble_rx_done = false;  // Reset the done flag
    ESP_LOGI("BLE_CONFIG", "BLE RX buffer reset");
}

const char *read_ble(void) 
{
	
    if (strlen(latest_ble_command) == 0) 
    {
	    /*ESP_LOGW("BLE_CONFIG", "Received empty command");
	   */ return "-1";
    }
    memset(latest_ble_command_temp, 0, sizeof(latest_ble_command_temp));
    memcpy(latest_ble_command_temp, ble_rx_buffer, strlen(ble_rx_buffer));
    memset(latest_ble_command, 0, sizeof(latest_ble_command));

    ESP_LOGI("BLE_CONFIG", "Received: %s", latest_ble_command_temp);
    return latest_ble_command_temp;
}

bool is_ble_rx_done(void) 
{
    if (ble_rx_done) {
        ble_rx_done = false;  // Reset for next command
        return true;
    }
    return false;
}


static int dummy_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int mtu_exchange_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t mtu, void *arg)
{
    if (error->status == 0) {
        ESP_LOGI("GAP", "MTU exchange complete: %d", mtu);
        struct ble_gap_upd_params params;
        params.itvl_min = 6;    /* 7.5 ms */
        params.itvl_max = 12;   /* 15 ms  */
        params.latency  = 0;
        params.supervision_timeout = 500;
        params.min_ce_len = 0;
        params.max_ce_len = 0;
        int rc = ble_gap_update_params(conn_handle, &params);
        if (rc != 0) ESP_LOGW("GAP", "Conn param update failed: %d", rc);
        else         ESP_LOGI("GAP", "Conn params updated: 7.5-15ms interval");
    } else {
        ESP_LOGW("GAP", "MTU exchange failed: status=%d", error->status);
    }
    return 0;
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) 
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
            if (event->connect.status == 0) {
                current_conn_handle = event->connect.conn_handle;
				ESP_LOGI("GAP", "Conn handle received: %d", current_conn_handle);
                ble_connected = true;
                ble_gattc_exchange_mtu(current_conn_handle, NULL, mtu_exchange_cb);
            } else {
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
                esp_restart();  // Reboots the ESP32
            	nimble_port_stop();
		        nimble_port_deinit();
                ble_connected = false;
    
            
            if (ble_stop_requested) 
			{
		        ble_stop_requested = false;
		
		        ESP_LOGI(TAG1, "Finalizing BLE shutdown after disconnect...");
		
		        nimble_port_stop();
		        nimble_port_deinit();
		        ESP_LOGI(TAG1, "BLE fully stopped and memory released.");
		       
		    }
		    else 
		    {
                ESP_LOGI(TAG1, "BLE - Waiting for connection");
		        ble_app_advertise();  // resume advertising if not shutting down
		    }
		    
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI("GAP", "BLE GAP EVENT ADV COMPLETE");
            ble_app_advertise();
            break;
            case BLE_GAP_EVENT_CONN_UPDATE:
            if (event->conn_update.status == 0) {
                struct ble_gap_conn_desc desc;
                ble_gap_conn_find(event->conn_update.conn_handle, &desc);
                ESP_LOGI("GAP", "Conn params updated: itvl=%d latency=%d timeout=%d",
                         desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
            } else {
                ESP_LOGW("GAP", "Connection update failed: %d", event->conn_update.status);
            }
            break;
        default:
            break;
    }
    return 0;
}


void ble_app_advertise(void) 
{
    if (!ble_ready) return;

    struct ble_hs_adv_fields fields;
    const char *device_name;

    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}


void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_ready = true;
    ble_app_advertise();
}

void host_task(void *param) {
    nimble_port_run();
    printf("Killing RTOS tasks for BLE\n");
    nimble_port_freertos_deinit();     // Frees NimBLE-related resources
    vTaskDelete(NULL);                 // Deletes this FreeRTOS task
}

void connect_ble(const char *device_name) {

    // printf("0\n");
    nvs_flash_init();
    //  printf("1\n");
    nimble_port_init();
    // printf("2\n");
    // ble_svc_gap_device_name_set("BLE-Server");
    ble_svc_gap_device_name_set(device_name);
    // printf("3\n");
    ble_svc_gap_init();
    // printf("4\n");
    ble_svc_gatt_init();
    // printf("5\n");
    ble_gatts_count_cfg(gatt_svcs);
    // printf("6\n");
    ble_gatts_add_svcs(gatt_svcs);
    // printf("7\n");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    // printf("8\n");
    ble_att_set_preferred_mtu(512);
    nimble_port_freertos_init(host_task);
}

#define MAX_NOTIFY_SIZE 20

void status_notify(const char *message) 
{

    printf("Notify call received.\n");
        
    if (!ble_ready || !ble_connected) {
        printf("Not notifying: BLE not ready or not connected\n");
        return;
    }
    // current_conn_handle = 0;
    printf("conn_handle: %d, char_handle: %d\n", current_conn_handle, status_char_handle);
	
    if (current_conn_handle == BLE_HS_CONN_HANDLE_NONE || status_char_handle == 0) {
        printf("Not notifying: Condition failed (conn_handle == 0 or char_handle == 0)\n");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(message, strlen(message));
    // ble_gattc_notify_custom(current_conn_handle, status_char_handle, om);

    int rc = ble_gattc_notify_custom(current_conn_handle, status_char_handle, om);
    ESP_LOGI("NOTIFY", "Notify result: %d", rc);

    printf("Notified: %s\n", message);
}

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MAX_NOTIFY_PAYLOAD  514
#define NOTIFY_DELAY_MS      50   /* ms between notifies — safe for NimBLE HCI queue */
#define NOTIFY_RETRY_DELAY  200   /* ms to wait when mbuf pool is exhausted          */


void status_notify_chunks(const char *message)
{
    size_t msg_len = strlen(message);
    size_t offset  = 0;

    if (!ble_ready || !ble_connected) {
        ESP_LOGW("BLE Notify", "Not notifying: BLE not ready or not connected");
        return;
    }

    if (current_conn_handle == BLE_HS_CONN_HANDLE_NONE || status_char_handle == 0) {
        ESP_LOGW("BLE Notify", "Invalid conn/char handle");
        return;
    }

    if (msg_len == 0) {
        ESP_LOGI("BLE Notify", "No data to send");
        return;
    }

    /* Use negotiated MTU so each notify carries as much as the link allows.
     * MTU=23 (BLE default) → 20 bytes/notify.
     * MTU=517 (after exchange) → 514 bytes/notify = far fewer notifies, no pool exhaustion. */
    uint16_t mtu = ble_att_mtu(current_conn_handle);
    if (mtu < 23) mtu = 23;
    size_t chunk_size = (mtu - 3);
    if (chunk_size > MAX_NOTIFY_PAYLOAD) chunk_size = MAX_NOTIFY_PAYLOAD;

    while (offset < msg_len) {
        size_t chunk_len = (msg_len - offset < chunk_size) ? (msg_len - offset) : chunk_size;

        struct os_mbuf *om = ble_hs_mbuf_from_flat(message + offset, chunk_len);
        if (!om) {
            /* Pool temporarily exhausted — wait and retry, do NOT drop the chunk */
            ESP_LOGW("BLE", "mbuf alloc failed, waiting %dms...", NOTIFY_RETRY_DELAY);
            vTaskDelay(pdMS_TO_TICKS(NOTIFY_RETRY_DELAY));
            continue;
        }

        int rc = ble_gattc_notify_custom(current_conn_handle, status_char_handle, om);
        if (rc != 0) {
            ESP_LOGW("BLE", "Notify rc=%d at offset %d, retrying...", rc, (int)offset);
            os_mbuf_free_chain(om);
            vTaskDelay(pdMS_TO_TICKS(NOTIFY_RETRY_DELAY));
            continue;
        }

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(NOTIFY_DELAY_MS));
    }
    ESP_LOGI("BLE Notify", "Sent %d bytes in chunks of %d", (int)msg_len, (int)chunk_size);
}


void set_ble_name(const char *name) {
    strncpy(ble_device_name, name, MAX_BLE_NAME_LEN - 1);
    ble_device_name[MAX_BLE_NAME_LEN - 1] = '\0';  // Ensure null-termination

    // Set GAP device name (used by central devices)
    int rc = ble_svc_gap_device_name_set(ble_device_name);
    if (rc != 0) {
        ESP_LOGE("BLE", "Failed to set BLE name: %d", rc);
    } else {
        ESP_LOGI("BLE", "BLE Name set to: %s", ble_device_name);
    }
}



void stop_ble(void) 
{
    ESP_LOGI(TAG1, "Stopping BLE activities...");

    ble_gap_adv_stop();
    ble_gap_disc_cancel();

    if (ble_connected) {
        ESP_LOGI(TAG1, "Disconnecting BLE connection handle=%d", current_conn_handle);
        int rc = ble_gap_terminate(current_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGE(TAG1, "Failed to disconnect, err=%d", rc);
        } else {
            ble_stop_requested = true; // Wait for disconnect event to finalize shutdown
            return;
        }
    }

    // If not connected, shutdown now
    ESP_LOGI(TAG1, "No active connection. Finalizing BLE shutdown...");

    nimble_port_stop();
    
    // Wait for stack to stop (optional, can also check a flag)
	vTaskDelay(pdMS_TO_TICKS(100));
    
    nimble_port_deinit();
	// esp_nimble_hci_and_controller_deinit();
    ble_connected = false;
    ESP_LOGI(TAG1, "BLE fully stopped and memory released.");
}

bool is_ble_connected_()
{
	return ble_connected;
}

bool is_ble_active(void) {
    // BLE is considered active if advertising is ongoing or a connection exists
    if (ble_gap_adv_active() || ble_connected) {
        return true;
    }
    return false;
}

#define NOTIFY_DELAY_FAST     50
#define NOTIFY_RETRY_MAX       2
#define NOTIFY_RETRY_DELAY   200
#define NOTIFY_DRAIN_MS      200

void status_notify_fast(const char *message)
{
    static uint32_t notify_count = 0;
    size_t msg_len = strlen(message);
    size_t offset = 0;

    if (!ble_ready || !ble_connected) return;
    if (msg_len == 0) return;
    if (current_conn_handle == BLE_HS_CONN_HANDLE_NONE || status_char_handle == 0) return;

    /* Wait for MTU exchange — NimBLE starts at 23, exchange is async */
    uint16_t mtu = ble_att_mtu(current_conn_handle);
    int wait_cycles = 0;
    while (mtu <= 23 && wait_cycles < 10) {
        ESP_LOGW("BLE", "Waiting for MTU (mtu=%d, cycle %d)", mtu, wait_cycles);
        vTaskDelay(pdMS_TO_TICKS(50));
        mtu = ble_att_mtu(current_conn_handle);
        wait_cycles++;
    }
    if (mtu <= 23) {
        ESP_LOGE("BLE", "MTU still %d after wait, aborting", mtu);
        return;
    }

    size_t chunk_size = (mtu - 3);
    if (chunk_size > MAX_NOTIFY_PAYLOAD) chunk_size = MAX_NOTIFY_PAYLOAD;

    while (offset < msg_len) {
        size_t chunk_len = MIN(chunk_size, msg_len - offset);
        int retries = 0;

        while (retries < NOTIFY_RETRY_MAX) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(message + offset, chunk_len);
            if (!om) {
                vTaskDelay(pdMS_TO_TICKS(NOTIFY_RETRY_DELAY));
                retries++;
                continue;
            }
            int rc = ble_gattc_notify_custom(current_conn_handle, status_char_handle, om);
            if (rc == 0) {
                notify_count++;
                break;
            }
            if (rc == 5) { ESP_LOGE("BLE", "Conn lost, aborting"); return; }
            ESP_LOGW("BLE", "Notify rc=%d, retry %d", rc, retries + 1);
            os_mbuf_free_chain(om);
            vTaskDelay(pdMS_TO_TICKS(NOTIFY_RETRY_DELAY));
            retries++;
        }
        if (retries >= NOTIFY_RETRY_MAX) {
            ESP_LOGE("BLE", "Notify FAILED after retries at %zu, aborting", offset);
            return;
        }
        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(NOTIFY_DELAY_FAST));
    }
}

void ble_notify_drain(void)
{
    vTaskDelay(pdMS_TO_TICKS(NOTIFY_DRAIN_MS));
}

void start_BLE(const char *device_name)
{
	connect_ble(device_name);
/*	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);*/
        
}

void notify()
{
	
}

