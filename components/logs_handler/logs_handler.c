#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "logs_handler.h"
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "GSM_handler.h"

#define LOG_TAG "LOG_MANAGER"

static FILE *log_file = NULL;

// static size_t max_log_file_size = 0xC40000; // 12 MB safe limit
// static size_t max_log_file_size = 0x2C0000; // ~2.75 MB — leaves 190KB headroom on 2.94MB SPIFFS
static size_t max_log_file_size = 0xB00000; // 11 MB — safe on 12.75 MB partition
/*RMS: ~20,200 logs → 14.1 days offline at 1 log/min*/

esp_err_t init_log_system()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;

    esp_spiffs_info("storage", &total, &used);
    ESP_LOGI(LOG_TAG, "SPIFFS: total=%d, used=%d", total, used);

    // Create file if not exists
    log_file = fopen(log_file_path, "a+");
    if (!log_file)
    {
        ESP_LOGE(LOG_TAG, "Failed to open log file");
        return ESP_FAIL;
    }
    fclose(log_file);
    return ESP_OK;
}

size_t get_file_size()
{
    struct stat st;
    if (stat(log_file_path, &st) == 0)
    {
        return st.st_size;
    }
    return 0;
}

// esp_err_t write_log_entry(const char *entry)
// {
//     size_t entry_len = strlen(entry) + 1; // Include newline later
//     size_t file_size = get_file_size();

//     if (file_size + entry_len >= max_log_file_size)
//     {
//         FILE *src = fopen(log_file_path, "r");
//         if (!src)
//             return ESP_FAIL;

//         char *buffer = malloc(max_log_file_size);
//         if (!buffer)
//         {
//             fclose(src);
//             return ESP_ERR_NO_MEM;
//         }

//         size_t new_pos = 0;
//         int found = 0;
//         while (fgets(buffer + new_pos, max_log_file_size - new_pos, src))
//         {
//             size_t line_len = strlen(buffer + new_pos);
//             if (!found && file_size - (new_pos + line_len) >= entry_len)
//             {
//                 found = 1;
//                 break;
//             }
//             new_pos += line_len;
//         }
//         fclose(src);

//         if (!found)
//         {
//             free(buffer);
//             ESP_LOGW(LOG_TAG, "No room for new entry, consider erasing logs");
//             return ESP_ERR_NO_MEM;
//         }

//         FILE *dst = fopen(log_file_path, "w");
//         if (!dst)
//         {
//             free(buffer);
//             return ESP_FAIL;
//         }

//         fwrite(buffer + new_pos, 1, strlen(buffer + new_pos), dst);
//         free(buffer);
//         fclose(dst);
//     }

//     log_file = fopen(log_file_path, "a");
//     if (!log_file)
//         return ESP_FAIL;

//     fprintf(log_file, "%s\n", entry);
//     fclose(log_file);
//     return ESP_OK;
// }


size_t curr_log_offset = 0;

time_t extract_timestamp(const char *log_line);

void check_last_log_ind() {
    size_t file_size = get_file_size();
    if (file_size == 0) {
        ESP_LOGI(LOG_TAG, "Log file is empty, resetting offset");
        curr_log_offset = 0;
        return;
    }

    /* File is append-only (until full erase), so the write cursor is always EOF.
       Do NOT infer position from DT ordering — RTC corrections can make DT go
       backward and incorrectly move curr_log_offset to an old location,
       causing already-posted entries to be treated as backlog. */
    curr_log_offset = file_size;
    ESP_LOGW(LOG_TAG, "Recovered log write offset at EOF: %zu", curr_log_offset);
}

static const char *TAG_LOG_DELETE = "LOG_DELETE";

esp_err_t delete_log_lines_and_check_space(size_t offset, size_t len) {
    FILE *fp = fopen(log_file_path, "r+");
    if (!fp) {
        ESP_LOGE(TAG_LOG_DELETE, "Failed to open log file");
        return ESP_FAIL;
    }

    // Seek to given offset
    fseek(fp, offset, SEEK_SET);

    char line[MAX_LINE_LEN];
    size_t delete_start = offset;
    size_t delete_end = offset;
    int lines_deleted = 0;
    size_t deleted_bytes = 0;

    while (fgets(line, sizeof(line), fp)) {

        delete_end = ftell(fp);  // Mark end after \n
        deleted_bytes = delete_end - delete_start;

        lines_deleted++;

        ESP_LOGI(TAG_LOG_DELETE, "Line %d deleted from %zu to %zu (len: %zu)", lines_deleted, delete_start, delete_end, deleted_bytes);


        if (len <= deleted_bytes ) {
            break;  // New data fits in deleted space
        }
    }

    // Truncate file
    fflush(fp);
    fclose(fp);

    ESP_LOGI(TAG_LOG_DELETE, "Deleted %d lines, total %zu bytes, New OFFSET: %zu", lines_deleted, deleted_bytes, curr_log_offset);
    return ESP_OK;
}


// // Returns timestamp as time_t, or -1 on failure
// time_t extract_timestamp(const char *log_line) {
//     struct tm tm_time = {0};

//     const char *date_ptr = strstr(log_line, "D:");
//     const char *time_ptr = strstr(log_line, "T:");

//     if (!date_ptr || !time_ptr) {
//         return -1;
//     }

//     // Extract date
//     if (sscanf(date_ptr, "D:%4d-%2d-%2d", &tm_time.tm_year, &tm_time.tm_mon, &tm_time.tm_mday) != 3) {
//         return -1;
//     }

//     // Extract time
//     if (sscanf(time_ptr, "T:%2d:%2d:%2d", &tm_time.tm_hour, &tm_time.tm_min, &tm_time.tm_sec) != 3) {
//         return -1;
//     }

//     tm_time.tm_year -= 1900; // tm_year is years since 1900
//     tm_time.tm_mon -= 1;     // tm_mon is 0-based

//     time_t timestamp = mktime(&tm_time);
//     return timestamp;
// }

time_t extract_timestamp(const char *log_line) {
    const char *dt_ptr = strstr(log_line, "\"DT\":");
    if (!dt_ptr) {
        return -1;
    }

    time_t timestamp;
    if (sscanf(dt_ptr, "\"DT\":%lld", &timestamp) != 1) {
        return -1;
    }

    return timestamp; // Already a UNIX timestamp
}

/*
   	    1 [0]   LOG_6
        2 [100] LOG_7
        3 [200] LOG_3
        4 [300] LOG_4
        5 [400] LOG_5
                        //	 prev_log_offset      curr_offset
                        //		   |				   |
                        //         v                   V			
                               { [100] +  LOG_7   =  [200] }

        Full wrap around of failed data -
        failed_data = [110] or [200] LOG_2   now overwritten by LOG_7
        update failed offset to next avail valid log
*/

void verify_failed_offset(size_t current_data_len)
{
    size_t failed_offset = get_failed_offset();
	bool failed_flag = get_is_failed();
	bool post_wrap = get_post_wrap();

	if( failed_flag && (failed_offset == curr_log_offset) )
	{
        ESP_LOGW("LOGS_HANDLER","Failed Offset had a wrap, incrementing failed offset");
		set_post_status(curr_log_offset + current_data_len, post_wrap, failed_flag);
	}

}

size_t prev_log_offset = 0;

size_t get_prev_log_offset()
{
    return prev_log_offset;
}


esp_err_t write_log_entry(const char *new_log)
{
    // ESP_LOGI(LOG_TAG, "Starting write_log_entry");

    size_t new_log_len = strlen(new_log) + 1; // Include newline
    size_t file_size = get_file_size();

    ESP_LOGI(LOG_TAG, "Log entry length: %d, current file size: %d", new_log_len, file_size);

    // Enters condition once the file is FULL
    if (file_size + new_log_len >= max_log_file_size)
    {
        size_t space_left = max_log_file_size - curr_log_offset;

        // Check if there is space to delete some logs and make space for new entry
        if (space_left < new_log_len)
        {
            curr_log_offset = 0; // Reset offset if file size exceeds limit
            ESP_LOGW(LOG_TAG, "Log file full. Attempting to remove oldest entries.");
        }

        ESP_LOGW(LOG_TAG, "Cannot write log entry, file size exceeds limit");
        // delete_log_lines_and_check_space(curr_log_offset, new_log_len);
        
        // log_file = fopen(log_file_path, "r+");
        // fseek(log_file, curr_log_offset, SEEK_SET); // Start writing from the last known position
        fclose(log_file);
        erase_logs();
        set_post_status(0,false,false); // Reset failed offset on full wrap
        ESP_LOGE(LOG_TAG, "ERASED , RESTARTING ...............");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    else
    {
        log_file = fopen(log_file_path, "a");
    }

   

    if (!log_file)
    {
        ESP_LOGE(LOG_TAG, "Failed to open log file in append mode");
        return ESP_FAIL;
    }


    fprintf(log_file, "%s\n", new_log);      // ESP_LOGI(LOG_TAG, "Log entry written: %s", entry);
    fclose(log_file);

    prev_log_offset = curr_log_offset;  // Store previous offset
    // verify_failed_offset(new_log_len);  // Check if failed offset overwritten
    curr_log_offset += new_log_len;     // Update current log offset
    ESP_LOGI(LOG_TAG, "Log Entry Written successfully");
    ESP_LOGI(LOG_TAG, "prev_log_offset: %zu | curr_log_offset: %zu", prev_log_offset, curr_log_offset);
    return ESP_OK;
}



void print_all_logs()
{
    log_file = fopen(log_file_path, "r");
    if (!log_file)
    {
        ESP_LOGE(LOG_TAG, "Unable to open log file for reading");
        return;
    }

    ESP_LOGW("print_all_logs","Printing all logs");
    char line[256];
    while (fgets(line, sizeof(line), log_file))
    {
        printf("%s", line);
    }
    fclose(log_file);
}


void print_all_logs_timestamp()
{
    log_file = fopen(log_file_path, "r");
    if (!log_file)
    {
        ESP_LOGE(LOG_TAG, "Unable to open log file for reading");
        return;
    }

    size_t off = get_failed_offset();
    bool failed = get_is_failed();

    ESP_LOGE("print_all_logs", "failed_offset = %d | is_failed %d",off,failed);

    ESP_LOGW("print_all_logs", "Printing all logs");
    char line[400];
    uint32_t count = 0;

    while (1) {
        long offset = ftell(log_file);   // <-- position before reading
        if (!fgets(line, sizeof(line), log_file)) {
            break; // EOF
        }

        // Look for "DT": field
        char *dt_ptr = strstr(line, "\"DT\":");
        if (dt_ptr) {
            long timestamp = strtol(dt_ptr + 5, NULL, 10);
            printf("[%lu] [%ld] %ld\n", count, offset, timestamp);
        } else {
            printf("[%lu] [%ld] (no timestamp)\n", count, offset);
        }

        count++;
    }

    fclose(log_file);
}



void erase_logs()
{
    log_file = fopen(log_file_path, "w");
    if (log_file) {
        fclose(log_file); // Opening in "w" mode truncates the file
        ESP_LOGI("SPIFFS", "File %s cleared", log_file_path);
    } else {
        ESP_LOGE("SPIFFS", "Failed to open file %s for clearing", log_file_path);
    }
}


void format_spiffs(void) {
    esp_err_t ret = esp_spiffs_format("storage"); // NULL for default partition
    if (ret == ESP_OK) {
        ESP_LOGI("SPIFFS", "SPIFFS partition formatted successfully");
    } else {
        ESP_LOGE("SPIFFS", "Failed to format SPIFFS partition: %s", esp_err_to_name(ret));
    }
}

size_t get_curr_log_offset()
{
    return curr_log_offset;
} 

void set_max_log_size(size_t max_size)
{
    max_log_file_size = max_size;
} 

size_t get_max_log_size()
{
    return max_log_file_size;
}

void update_last_log_offset()
{
    curr_log_offset = get_file_size();
}