#include <stdio.h>
#include "sensor_config.h"
#include "support.h"

sensor_config_t sensors[MAX_SENSORS];
int sensor_count;

// sensor_config_t default_sensors[MAX_SENSORS] = {
//        0 {"Pressure Sensor", "PSR", true, true, 0.0, 1.00, 0.0, false},
//        1 {"pH", "PH", true, true, 0.0, 1.00, 0.0, false},
//        2 {"TDS", "TDS", true, true, 0.0, 1.00, 0.0, false},
//        3 {"Chlorine Sensor", "CLO", true, true, 0.0, 1.00, 0.0, false},
//        4 {"TempW", "TPW", true, false, 0.0, 1.00, 0.0, false},
//        5 {"Weight Sensor", "WGT", true, true, 0.0, 1.00, 0.0, false},
//        6 {"Turbidity", "TBD", true, false, 0.0, 1.00, 0.0, false},
//        7 {"Ultrasonic Flow", "UFM1_F", true, true, 0.0, 1.00, 0.0, false},
//        8 {"Ultrasonic Volume", "UFM1_V", true, true, 0.0, 2.00, 0.0, false},
//        9 {"Ultrasonic Flow", "UFM2_F", true, true, 0.0, 1.00, 0.0, false},
//        10 {"Ultrasonic Flow2", "UFM2_V", true, true, 0.0, 2.00, 0.0, false},

//        11 {"TempA", "TPA", true, false, 0.0, 1.00, 0.0, false},
//        12 {"Flow Sensor", "FLW", true, false, 0.0, 1.00, 0.0, false},
//        13 {"Switch 1", "SW1", true, false, 0.0, 1.00, 0.0, false},
//        14 {"Switch 2", "SW2", true, false, 0.0, 1.00, 0.0, false}
//     };


// void init_default_sensors()
// {
//     sensor_config_t default_sensors[MAX_SENSORS] = {
//        0 {"Pressure Sensor", "PSR", true, true, 0.0, 1.00, 0.0, false},
//        1{"pH", "PH", true, true, 0.0, 1.00, 0.0, false},
//        2 {"TDS", "TDS", true, true, 0.0, 1.00, 0.0, false},
//        3 {"Chlorine Sensor", "CLO", true, true, 0.0, 1.00, 0.0, false},
//        4 {"TempW", "TPW", true, false, 0.0, 1.00, 0.0, false},
//        5 {"Weight Sensor", "WGT", true, true, 0.0, 1.00, 0.0, false},
//        6 {"Turbidity", "TBD", true, false, 0.0, 1.00, 0.0, false},
//        7 {"Ultrasonic Flow", "UFM1F", true, true, 0.0, 1.00, 0.0, false},
//        8 {"Ultrasonic Volume", "UFM1V", true, true, 0.0, 2.00, 0.0, false},
//        9 {"Ultrasonic Flow2", "UFM2F", true, true, 0.0, 1.00, 0.0, false},
//        10 {"Ultrasonic Flow2", "UFM2V", true, true, 0.0, 2.00, 0.0, false},
//        11{"Chlorine NC", "CLO_NC", true, true, 0.0, 1.00, 0.0, false},
//        
//        12 {"TempA", "TPA", true, false, 0.0, 1.00, 0.0, false},
//        13 {"Flow Sensor", "FLW", true, false, 0.0, 1.00, 0.0, false},
//        14 {"Switch 1", "SW1", true, false, 0.0, 1.00, 0.0, false},
//        15 {"Switch 2", "SW2", true, false, 0.0, 1.00, 0.0, false}
//     };
//     memcpy(sensors, default_sensors, sizeof(default_sensors));
// }

// sensor_map_t sensor_map[] = {
//     {TPW, 4, &tempWVal},
//     {TPA, 12, &tempAVal},
//     {FLW, 16, &FlowVal},
//     {SW1, 2, &Sw1Val},
//     {SW2, 3, &Sw2Val},
// };

// sensor_map_t sensor_map[] = {
//     {TPW, 4, &tempWVal},
//     {TPA, 12, &tempAVal},
//     {FLW, 16, &FlowVal},
//     // {SW1, 14, &Sw1Val},
//     // {SW2, 15, &Sw2Val},
// };

esp_err_t save_sensors_to_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SENSOR_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    size_t data_size = sizeof(sensor_config_t) * MAX_SENSORS;
    
    err = nvs_set_blob(nvs_handle, SENSOR_KEY, sensors, data_size);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "Failed to write blob: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG_NVS, "Sensor config saved (%d bytes)", data_size);
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t load_sensors_from_nvs(void) {
    nvs_handle_t handle;
    size_t required_size;
    esp_err_t err = nvs_open(SENSOR_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("NVS", "No saved sensor config found.");
        return err;
    }

    // Get size of saved blob
    err = nvs_get_blob(handle, SENSOR_KEY, NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW("NVS", "Sensor blob not found.");
        nvs_close(handle);
        return err;
    }

    if (required_size > sizeof(sensor_config_t) * MAX_SENSORS) {
        ESP_LOGE("NVS", "Sensor blob too large!");
        nvs_close(handle);
        return ESP_ERR_NVS_VALUE_TOO_LONG;
    }

    // Read blob
    err = nvs_get_blob(handle, SENSOR_KEY, sensors, &required_size);
    if (err == ESP_OK) {
        sensor_count = required_size / sizeof(sensor_config_t);
        ESP_LOGI("NVS", "Loaded %d sensor(s) from NVS.", sensor_count);
    } else {
        ESP_LOGE("NVS", "Failed to read sensors: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    return err;
}



esp_err_t clear_sensor_nvs_namespace(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SENSOR_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS namespace (%s)", esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_all(handle);  // Clears all keys in this namespace
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to erase NVS (%s)", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to commit NVS erase (%s)", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "Sensor NVS namespace cleared successfully.");
    }

    nvs_close(handle);
    return err;
}

int get_sensor_count(void) 
{
    return sensor_count;
}

/*esp_err_t load_sensors_from_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SENSOR_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_NVS, "NVS open failed (READONLY): %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(sensor_config_t) * MAX_SENSORS;
    err = nvs_get_blob(nvs_handle, SENSOR_KEY, sensors, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_NVS, "Sensor config loaded (%d bytes)", required_size);
    } else {
        ESP_LOGW(TAG_NVS, "No sensor config found in NVS");
    }

    nvs_close(nvs_handle);
    return err;
}*/



// void init_default_sensors()
// {
//     sensor_config_t default_sensors[MAX_SENSORS] = {
//         {"Pressure Sensor", "PSR", true, true, 0.0, 1.00, 0.0, false},
//         {"pH", "PH", true, true, 0.0, 1.00, 0.0, false},
//         {"TDS", "TDS", true, true, 0.0, 1.00, 0.0, false},
//         {"Chlorine PC", "CLO_PC", true, true, 0.0, 1.00, 0.0, false},
//         {"TempW", "TPW", true, false, 0.0, 1.00, 0.0, false},
//         {"Weight Sensor", "WGT", true, true, 0.0, 1.00, 0.0, false},
//         {"Turbidity", "TBD", true, false, 0.0, 1.00, 0.0, false},
//         {"Ultrasonic Flow", "UFM1F", true, true, 0.0, 1.00, 0.0, false},
//         {"Ultrasonic Volume", "UFM1V", true, true, 0.0, 2.00, 0.0, false},
//         {"Ultrasonic Flow2", "UFM2F", true, true, 0.0, 1.00, 0.0, false},
//         {"Ultrasonic Flow2", "UFM2V", true, true, 0.0, 2.00, 0.0, false},
//         {"Chlorine NC", "CLO_NC", true, true, 0.0, 1.00, 0.0, false},

//         {"TempA", "TPA", true, false, 0.0, 1.00, 0.0, false},
//         {"Flow Sensor", "FLW", true, false, 0.0, 1.00, 0.0, false},
//         {"Switch 1", "SW1_mod", true, false, 0.0, 1.00, 0.0, false},
//         {"Switch 2", "SW2_mod", true, false, 0.0, 1.00, 0.0, false}
//     };
//     memcpy(sensors, default_sensors, sizeof(default_sensors));
// }

void init_default_sensors()
{
    sensor_config_t default_sensors[MAX_SENSORS] =
    {
        /* --- MODBUS SENSORS (ORDERED BY modbus_sensor_e) --- */
        [DigitalInputs] = {"Digital Inputs",   "DIN",   true, false, 0.0, 1.00, 0.0, false},

        [Pressure1]      = {"Pressure 1",       "PSR1",  true, true,  0.0, 1.00, 0.0, false},
        [Pressure2]      = {"Pressure 2",       "PSR2",  true, true,  0.0, 1.00, 0.0, false},

        [Flow1]          = {"Flow 1",            "FL1",   true, false, 0.0, 1.00, 0.0, false},
        [Flow2]          = {"Flow 2",            "FL2",   true, false, 0.0, 1.00, 0.0, false},

        [Rly_SV1]         = {"Relay SV1",           "RY_SV1",  true, false, 0.0, 1.00, 0.0, false},
        [Rly_SV2]         = {"Relay SV2",           "RY_SV2",  true, false, 0.0, 1.00, 0.0, false},
        [Rly_SV3]         = {"Relay SV3",           "RY_SV3",  true, false, 0.0, 1.00, 0.0, false},
        [Rly_SV4]         = {"Relay SV4",           "RY_SV4",  true, false, 0.0, 1.00, 0.0, false},
        [Rly_HP]          = {"Relay HP",            "RY_HP",   true, false, 0.0, 1.00, 0.0, false},
        [Rly_UV]          = {"Relay UV",            "RY_UV",   true, false, 0.0, 1.00, 0.0, false},
        [Rly_BackW]       = {"Relay BackWash",      "RY_BW",   true, false, 0.0, 1.00, 0.0, false},
        [Rly_RW]          = {"Relay RW",            "RY_RW",   true, false, 0.0, 1.00, 0.0, false},

        [HMI_TDS_IN]      = {"HMI TDS IN",        "HMI_TDS_IN",   true, false, 0.0, 1.00, 0.0, false},
        [HMI_TEMPW_IN]    = {"HMI TEMPW IN",      "HMI_TEMPW_IN", true, false, 0.0, 1.00, 0.0, false},
        [HMI_TDS_OUT]     = {"HMI TDS OUT",       "HMI_TDS_OUT",  true, false, 0.0, 1.00, 0.0, false},
        [HMI_TEMPW_OUT]   = {"HMI TEMPW OUT",     "HMI_TEMPW_OUT",true, false, 0.0, 1.00, 0.0, false},

        [SYS_en]          = {"System Enable",         "SYS_en",    true, false, 0.0, 1.00, 0.0, false},
        [DDPS_en]         = {"DDPS Enable",           "DDPS_en",   true, false, 0.0, 1.00, 0.0, false},
        [BACKW_FREQ]      = {"BackWash Freq",         "BACKW_FREQ",true, false, 0.0, 1.00, 0.0, false},
        [PS_DELTA]        = {"Pressure Delta",        "PS_DELTA",  true, false, 0.0, 1.00, 0.0, false},
        [HMI_BACKW_BTN]   = {"HMI BackW Btn",         "HMI_BW_BTN",true, false, 0.0, 1.00, 0.0, false},

        /* From TDS Sensor Module */
        [TDS_IN]            = {"TDS RW",              "TDS_RW",   true, true,  0.0, 1.00, 0.0, false},
        [TEMPW_IN]          = {"Water Temp RW",       "TPW_RW",   true, false, 0.0, 1.00, 0.0, false},
        [TDS_OUT]           = {"TDS TW",              "TDS_TW",  true, true,  0.0, 1.00, 0.0, false},
        [TEMPW_OUT]         = {"Water Temp TW",       "TPW_TW",  true, true,  0.0, 1.00, 0.0, false},

    };
    memcpy(sensors, default_sensors, sizeof(default_sensors));
}
                                  

void print_sensor_data(void) {
    ESP_LOGI("SENSOR", "---- Loaded Sensor Info ----");
    for (int i = 0; i < sensor_count; i++) {
        sensors[i].value = 0.0;
        printf("SENSOR" "[%d] Name: %s | Value: %.2f | Offset: %.2f | Scale: %.2f | RS485: %s | Response: %s | is_en: %s\n",
                 i + 1,
                 sensors[i].name,
                 sensors[i].value,
                 sensors[i].offset,
                 sensors[i].scale,
                 sensors[i].is_rs485 ? "Yes" : "No",
                 sensors[i].response_ok ? "OK" : "Fail",
                 sensors[i].is_enabled ? "Yes" : "No");
    }
}



// void process_sensor_config_data(const char *data) 
// {
//     ESP_LOGI("SENSOR_CONFIG", "process_sensor_config_data Running !!");
//     char buffer[512];
//     strncpy(buffer, data, sizeof(buffer));
//     buffer[sizeof(buffer) - 1] = '\0';

//     // Remove outer braces
//     char *start = strchr(buffer, '{');
//     char *end = strrchr(buffer, '}');
//     if (!start || !end || end <= start) {
//         ESP_LOGE("SENSOR_CONFIG", "Invalid input format");
//         ESP_LOGE("SENSOR_CONFIG", "Data - %s", data);
//         return;
//     }
//     *end = '\0';
//     start++;

//     // Skip optional prefix if present
//     const char *prefix = "$sensor_config$";
//     if (strncmp(start, prefix, strlen(prefix)) == 0) {
//         start += strlen(prefix);
//     }

//     // Process chunks starting with "S_Name:"
//     char *entry = strstr(start, "S_Name:");
//     while (entry) {
//         entry += strlen("S_Name:");
//         char key[8] = {0};
//         float offset = 0.0f, scale = 0.0f;

//         int parsed = sscanf(entry, "%7[^,],Offset: %f,Scale: %f", key, &offset, &scale);
//         if (parsed == 3) {
//             key[strcspn(key, "\r\n")] = '\0';

//             bool found = false;
//             for (int i = 0; i < MAX_SENSORS; i++) {
//                 if (strcmp(sensors[i].key, key) == 0) {
//                     sensors[i].offset = offset;
//                     sensors[i].scale = scale;
//                     found = true;
//                     ESP_LOGI("SENSOR_CONFIG", "Updated [%s]: Offset=%.2f, Scale=%.2f",
//                              sensors[i].key, offset, scale);
//                     break;
//                 }
//             }
//             if (!found) {
//                 ESP_LOGW("SENSOR_CONFIG", "Sensor key '%s' not found in config", key);
//             }
//         } else {
//             ESP_LOGW("SENSOR_CONFIG", "Failed to parse sensor config at: %s", entry);
//         }

//         // Look for next entry
//         entry = strstr(entry, "S_Name:");
//     }
// }





void process_sensor_config_data(const char *data) 
{
    ESP_LOGI("SENSOR_CONFIG", "process_sensor_config_data Running !!");

    // Allocate buffer dynamically based on input length
    size_t len = strlen(data) + 1;  // +1 for '\0'
    char *buffer = malloc(len);
    if (!buffer) {
        ESP_LOGE("SENSOR_CONFIG", "Memory allocation failed!");
        return;
    }

    strcpy(buffer, data);  // safe because we allocated exact size

    // Remove outer braces
    char *start = strchr(buffer, '{');
    char *end = strrchr(buffer, '}');
    if (!start || !end || end <= start) {
        ESP_LOGE("SENSOR_CONFIG", "Invalid input format");
        ESP_LOGE("SENSOR_CONFIG", "Data - %s", data);
        free(buffer);
        return;
    }
    *end = '\0';
    start++;

    // Skip optional prefix if present
    const char *prefix = "$sensor_config$";
    if (strncmp(start, prefix, strlen(prefix)) == 0) {
        start += strlen(prefix);
    }

    // Process chunks starting with "S_Name:"
    char *entry = strstr(start, "Key:");
    while (entry) {
        entry += strlen("Key:");
        char key[8] = {0};
        float offset = 0.0f, scale = 1.0f;
        char is_en_str[8] = {0};
        bool is_enabled = true;  // Default if not present
        uint8_t is_enabled_int = 1; // Default to enabled

        // Try to parse with is_en first
        int parsed = sscanf(entry, "%7[^,],Offset:%f,Scale:%f,is_en:%hhd", key, &offset, &scale, &is_enabled_int);

        if (parsed >= 3) {
            key[strcspn(key, "\r\n")] = '\0';

            // If is_en parsed, handle it
            if (parsed == 4) {
                if (is_enabled_int == 0) {
                    is_enabled = false;
                    ESP_LOGW("PROCESS_SENSOR_CONFIG", "Sensor [%s] disabled", key);
                } else if (is_enabled_int == 1) {
                    is_enabled = true;
                    ESP_LOGW("PROCESS_SENSOR_CONFIG", "Sensor [%s] enabled", key);
                } else {
                    ESP_LOGW("SENSOR_CONFIG", "Unknown is_en value '%d' (defaulting to true)", is_enabled_int);
                }
            }

            bool found = false;
            for (int i = 0; i < MAX_SENSORS; i++) {
                if (strcmp(sensors[i].key, key) == 0) {
                    sensors[i].offset = offset;
                    sensors[i].scale = scale;
                    sensors[i].is_enabled = is_enabled;

                    found = true;
                    ESP_LOGI("SENSOR_CONFIG", "Updated [%s]: Offset=%.2f, Scale=%.2f, is_enabled=%d",
                             sensors[i].key, offset, scale, is_enabled ? 1 : 0);
                    break;
                }
            }
            if (!found) {
                ESP_LOGW("SENSOR_CONFIG", "Sensor key '%s' not found in config", key);
            }
        } else {
            ESP_LOGW("SENSOR_CONFIG", "Failed to parse sensor config at: %s", entry);
        }

        // Look for next entry
        entry = strstr(entry, "Key:");
    }
    free(buffer);  // Free allocated memory
}





void process_device_config_data(const char *input, DeviceConfig *device_config) {
    ESP_LOGW("DEBUG_INCOMING", "Raw Config String: %s", input);
    if (!input || !strstr(input, "{$device_config$") || !device_config) {
        printf("Invalid input.\n");
        return;
    }

    const char *start = strstr(input, "{$device_config$");
    start += strlen("{$device_config$");

    while (*start && *start != '}') {
        char key[32] = {0};
        char value[64] = {0};

        // Find key
        const char *colon = strchr(start, ':');
        if (!colon) break;
        int key_len = colon - start;
        if (key_len >= sizeof(key)) key_len = sizeof(key) - 1;
        strncpy(key, start, key_len);
        key[key_len] = '\0';

        // Find value
        const char *comma = strchr(colon + 1, ',');
        const char *end = strchr(colon + 1, '}');
        const char *val_end = (comma && (!end || comma < end)) ? comma : end;
        if (!val_end) break;

        int val_len = val_end - (colon + 1);
        if (val_len >= sizeof(value)) val_len = sizeof(value) - 1;
        strncpy(value, colon + 1, val_len);
        value[val_len] = '\0';

        // Map key-value to struct
        if (strcmp(key, "Servr_name") == 0) {
            strncpy(device_config->server_name, value, sizeof(device_config->server_name));
        } else if (strcmp(key, "data_freq") == 0) {
            device_config->data_frequency_sec = (uint32_t)atoi(value);
        } else if (strcmp(key, "PS_enabled_DDPS_disabled") == 0) {
            device_config->PS_enabled_DDPS_disabled = atoi(value) ? true : false;
        } else if (strcmp(key, "wifi_enabled_gsm_disabled") == 0) {
            device_config->wifi_enabled_gsm_disabled = atoi(value) ? true : false;
        } else if (strcmp(key, "wifi_ssid") == 0) {
            strncpy(device_config->wifi_ssid, value, sizeof(device_config->wifi_ssid));
        } else if (strcmp(key, "wifi_password") == 0) {
            strncpy(device_config->wifi_password, value, sizeof(device_config->wifi_password));
        }  else if (strcmp(key, "num_level_sensor") == 0) {
            device_config->num_level_sensor = (uint8_t)atoi(value);
        } else if (strcmp(key, "PS_delta") == 0) {
           float tmp;
            if (sscanf(value, "%f", &tmp) == 1) {
                device_config->PS_delta = tmp;
            } else {
                ESP_LOGW("CONF", "sscanf failed for PS_delta '%s'", value);
            }
        } else if (strcmp(key, "Backwash_frequency_min") == 0) {
            device_config->Backwash_frequency_min =  (uint32_t)atoi(value);
        } else if (strcmp(key, "Backwash_time_sec") == 0) {
            device_config->Backwash_time_sec =  (uint32_t)atoi(value);
        } else if (strcmp(key, "gsm_sim_name") == 0) {
            strncpy(device_config->gsm_sim_name, value, sizeof(device_config->gsm_sim_name));
        }

        // Move to next key-pair
        start = val_end + 1;
    }

    // Debug print
    printf("Parsed config:\n");
    printf("  Server: %s\n", device_config->server_name);
    printf("  Freq: %lu\n", device_config->data_frequency_sec);
    printf("  PS Enabled DDPS Disabled: %s\n", device_config->PS_enabled_DDPS_disabled ? "True" : "False");
    printf("  Wifi Enabled GSM Disabled: %s\n", device_config->wifi_enabled_gsm_disabled ? "True" : "False");
    printf("  WiFi SSID: %s\n", device_config->wifi_ssid);
    printf("  WiFi Password: %s\n", device_config->wifi_password);
    printf("  Number of Level Sensors: %d\n", device_config->num_level_sensor);
    printf("  PS Delta: %lu\n", device_config->PS_delta);
    printf("  Backwash Frequency (min): %lu\n", device_config->Backwash_frequency_min);
    printf("  Backwash Time (sec): %lu\n", device_config->Backwash_time_sec);
    printf("  SIM Name: %s\n", device_config->gsm_sim_name);
}


/* Device and Data COnfigurations */



esp_err_t save_device_status_to_nvs(const DeviceStatus *status) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_STATUS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, DEVICE_STATUS_KEY, status, sizeof(DeviceStatus));
    if (err == ESP_OK) err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

esp_err_t load_device_status_from_nvs(DeviceStatus *status) {
    nvs_handle_t handle;
    size_t required_size = sizeof(DeviceStatus);
    esp_err_t err = nvs_open(DEVICE_STATUS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_blob(handle, DEVICE_STATUS_KEY, status, &required_size);
    nvs_close(handle);
    return err;
}

esp_err_t save_device_config_to_nvs(const DeviceConfig *config) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, DEVICE_CONFIG_KEY, config, sizeof(DeviceConfig));
    if (err == ESP_OK) err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

esp_err_t load_device_config_from_nvs(DeviceConfig *config) {
    nvs_handle_t handle;
    size_t required_size = sizeof(DeviceConfig);
    esp_err_t err = nvs_open(DEVICE_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_blob(handle, DEVICE_CONFIG_KEY, config, &required_size);
    nvs_close(handle);
    return err;
}

esp_err_t clear_device_status_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_STATUS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_erase_key(handle, DEVICE_STATUS_KEY);
    if (err == ESP_OK) err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

esp_err_t clear_device_config_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_erase_key(handle, DEVICE_CONFIG_KEY);
    if (err == ESP_OK) err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

/* Print the Config and Status */

void print_device_status(const DeviceStatus *status) {
    printf("\n----- Device Status -----\n");
    printf("Date: %s\n", status->current_date);
    printf("Time: %s\n", status->current_time);
    printf("GSM Signal Strength: %d dBm\n", status->gsm_signal_strength);
    printf("GSM Post Status: %s\n", status->gsm_post_status);
    printf("Battery Level: %.2f%%\n", status->battery_level);
    printf("Solar Charging Level: %.2f%%\n", status->solar_level);
    printf("Log Count Since Last Clear: %ld\n", status->log_count);
}

void print_device_config(const DeviceConfig *config) {
    printf("\n----- Device Config -----\n");
    printf("Device Type: %s\n", config->device_type);
    printf("IMEI_num : %s\n", config->IMEI_num);
    printf("Server Name: %s\n", config->server_name);
    printf("Firmware Version: %s\n", config->firmware_version);
    printf("Data Frequency (sec): %ld\n", config->data_frequency_sec);
    printf("Latitude: %.6f\n", config->latitude);
    printf("Longitude: %.6f\n", config->longitude);
    printf("PS_enabled_DDPS_disabled: %s\n", config->PS_enabled_DDPS_disabled ? "Yes" : "No");
        config->PS_enabled_DDPS_disabled ? printf("PS Enabled\n") : printf("DDPS Enabled\n");
    printf("wifi_enabled_gsm_disabled: %s\n", config->wifi_enabled_gsm_disabled ? "Yes" : "No");
        config->wifi_enabled_gsm_disabled ? printf("Wifi Enabled\n") : printf("GSM Enabled\n");
    printf("Number of Level Sensors: %d\n", config->num_level_sensor);
    printf("WiFi SSID: %s\n", config->wifi_ssid);
    printf("WiFi Password: %s\n", config->wifi_password);
    printf("PS Delta: %ld\n", config->PS_delta);
    printf("Backwash Frequency (min): %ld\n", config->Backwash_frequency_min);
    printf("Backwash Time (sec): %ld\n", config->Backwash_time_sec);
    printf("SIM: %s\n", config->gsm_sim_name);
}


void init_default_device_status(DeviceStatus *status) {
    strcpy(status->current_date, "2026-01-01");
    strcpy(status->current_time, "10-05-06");
    status->gsm_signal_strength = 0;
    status->battery_level = 0.0f;
    status->solar_level = 0.0f;
    strcpy(status->gsm_post_status, "OK");
    status->log_count = 0;
}





void init_default_device_config(DeviceConfig *config) {
    strcpy(config->device_type, DEVICE_TYPE);
    strcpy(config->IMEI_num, "000000000");
    strcpy(config->server_name, DEVICE_ID);
    strcpy(config->firmware_version, "FR_1.0.0");
    config->data_frequency_sec = 60;
    config->latitude = 0.0;
    config->longitude = 0.0;
    config->wifi_enabled_gsm_disabled = false;
    strcpy(config->wifi_ssid, "WIFI");
    strcpy(config->wifi_password, "WIFI_PASS");
    config->num_level_sensor = 1;
    config->PS_enabled_DDPS_disabled = false;
    config->PS_delta = 0;
    config->Backwash_frequency_min = 0;
    config->Backwash_time_sec = 15;
    strcpy(config->gsm_sim_name, "AIRTEL");
}