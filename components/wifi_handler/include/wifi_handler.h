#ifndef WIFI_INIT
#define WIFI_INIT
#include "esp_http_client.h"

#define POST_URL    "https://gkcsq4xz1k.execute-api.ap-south-1.amazonaws.com/default/sendTelemetryData_v2"
// #define POST_URL    "http://safewaternetwork.in:8096/v1/schools/siteData"   // production — uncomment when going live

#define POST_HOST    "gkcsq4xz1k.execute-api.ap-south-1.amazonaws.com"
#define POST_PATH    "/default/sendTelemetryData_v2"
// #define POST_HOST "safewaternetwork.in:8096"                               // production
// #define POST_PATH "/v1/schools/siteData"                                   // production
#define OTA_CONFIG_URL "http://13.201.57.30:8080/RMS_OTA/ConfigFiles/"
#define OTA_FILE_URL   "http://13.201.57.30:8080/RMS_OTA/FirmwareFiles/"
// #define POST_URL    "http://safewaternetwork.in/chlorinationv2/acom.jsp"

esp_err_t wifi_handler_init(uint8_t *wifi_id, uint8_t *wifi_pass);
bool wifi_is_connected(void);
esp_err_t sync_time(void);
esp_err_t http_send_json(const char * url, const char *json_payload, esp_http_client_method_t method_type);
esp_err_t start_ota_wifi();

#endif