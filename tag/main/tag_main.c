// =============================================================================
// tag_main.c  —  ESP32  RSSI Trilateration Tag  (CSV serial output)
// =============================================================================
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// ─────────────────────────────────────────────────────────────────────────────
//  CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
#define TAG_ID          "tag_3"
#define DEVICE_ID       "esp32_tag_3"
#define LOCATION_ID     "lab_1"
#define FLOOR_ID        "floor_1"
#define ROOM_ID         "room_1"

#define AP_CHANNEL      6
#define MAX_ANCHORS     6
#define RSSI_SAMPLES    5
#define PATH_LOSS_N     2.7f
#define RSSI_AT_1M     -40.0f

// Anchor positions in cm
static const struct {
    int   id;
    float x;
    float y;
} ANCHOR_POSITIONS[] = {
    { 0,    0,    0 },
    { 1,  300,    0 },
    { 2,  150,  300 },
    { 3,    0,  300 },
    { 4,  300,  300 },
    { 5,  150,    0 },
};
#define NUM_ANCHOR_POSITIONS 6

// ─────────────────────────────────────────────────────────────────────────────

static const char *TAG = "TAG";
static int s_scan_number = 0;

typedef struct {
    char    ssid[32];
    uint8_t bssid[6];
    int     anchor_id;
    float   x_cm;
    float   y_cm;
    int8_t  rssi_samples[RSSI_SAMPLES];
    int     sample_count;
    float   avg_rssi;
    float   distance_cm;
    bool    valid;
} anchor_reading_t;

static anchor_reading_t s_anchors[MAX_ANCHORS];
static int              s_anchor_count = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi Init
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ─────────────────────────────────────────────────────────────────────────────
//  RSSI to distance
// ─────────────────────────────────────────────────────────────────────────────
static float rssi_to_distance(float rssi)
{
    return powf(10.0f, (RSSI_AT_1M - rssi) / (10.0f * PATH_LOSS_N)) * 100.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Get anchor position
// ─────────────────────────────────────────────────────────────────────────────
static bool get_anchor_position(int anchor_id, float *x, float *y)
{
    for (int i = 0; i < NUM_ANCHOR_POSITIONS; i++) {
        if (ANCHOR_POSITIONS[i].id == anchor_id) {
            *x = ANCHOR_POSITIONS[i].x;
            *y = ANCHOR_POSITIONS[i].y;
            return true;
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Scan once
// ─────────────────────────────────────────────────────────────────────────────
static void scan_once(void)
{
    wifi_scan_config_t scan_cfg = {
        .ssid        = NULL,
        .bssid       = NULL,
        .channel     = AP_CHANNEL,
        .show_hidden = false,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) return;

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) return;

    wifi_ap_record_t *ap_list = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_list) return;

    esp_wifi_scan_get_ap_records(&ap_count, ap_list);

    for (int i = 0; i < ap_count; i++) {
        if (strncmp((char *)ap_list[i].ssid, "TOF_ANCHOR_", 11) != 0) continue;

        int anchor_id = atoi((char *)ap_list[i].ssid + 11);

        int idx = -1;
        for (int j = 0; j < s_anchor_count; j++) {
            if (s_anchors[j].anchor_id == anchor_id) { idx = j; break; }
        }

        if (idx == -1 && s_anchor_count < MAX_ANCHORS) {
            idx = s_anchor_count++;
            memset(&s_anchors[idx], 0, sizeof(anchor_reading_t));
            strlcpy(s_anchors[idx].ssid, (char *)ap_list[i].ssid,
                    sizeof(s_anchors[idx].ssid));
            memcpy(s_anchors[idx].bssid, ap_list[i].bssid, 6);
            s_anchors[idx].anchor_id = anchor_id;
            get_anchor_position(anchor_id,
                                &s_anchors[idx].x_cm,
                                &s_anchors[idx].y_cm);
            s_anchors[idx].valid = true;
        }

        if (idx >= 0) {
            int slot = s_anchors[idx].sample_count % RSSI_SAMPLES;
            s_anchors[idx].rssi_samples[slot] = ap_list[i].rssi;
            s_anchors[idx].sample_count++;
        }
    }

    free(ap_list);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Compute distances
// ─────────────────────────────────────────────────────────────────────────────
static void compute_distances(void)
{
    for (int i = 0; i < s_anchor_count; i++) {
        if (!s_anchors[i].valid) continue;

        int count = s_anchors[i].sample_count < RSSI_SAMPLES
                    ? s_anchors[i].sample_count : RSSI_SAMPLES;
        if (count == 0) continue;

        float sum = 0;
        for (int j = 0; j < count; j++) sum += s_anchors[i].rssi_samples[j];
        s_anchors[i].avg_rssi    = sum / count;
        s_anchors[i].distance_cm = rssi_to_distance(s_anchors[i].avg_rssi);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trilateration
// ─────────────────────────────────────────────────────────────────────────────
static bool trilaterate(float *est_x, float *est_y)
{
    float ax[MAX_ANCHORS], ay[MAX_ANCHORS], ad[MAX_ANCHORS];
    int   n = 0;

    for (int i = 0; i < s_anchor_count; i++) {
        if (!s_anchors[i].valid || s_anchors[i].sample_count == 0) continue;
        ax[n] = s_anchors[i].x_cm;
        ay[n] = s_anchors[i].y_cm;
        ad[n] = s_anchors[i].distance_cm;
        n++;
    }

    if (n < 2) return false;

    if (n == 2) {
        float w0 = 1.0f / (ad[0] + 1.0f);
        float w1 = 1.0f / (ad[1] + 1.0f);
        *est_x = (ax[0] * w0 + ax[1] * w1) / (w0 + w1);
        *est_y = (ay[0] * w0 + ay[1] * w1) / (w0 + w1);
        return true;
    }

    float A[MAX_ANCHORS][2], b[MAX_ANCHORS];
    int   rows = n - 1;

    for (int i = 1; i < n; i++) {
        A[i-1][0] = 2.0f * (ax[i] - ax[0]);
        A[i-1][1] = 2.0f * (ay[i] - ay[0]);
        b[i-1]    = (ad[0]*ad[0]) - (ad[i]*ad[i])
                  - (ax[0]*ax[0]) + (ax[i]*ax[i])
                  - (ay[0]*ay[0]) + (ay[i]*ay[i]);
    }

    float ATA[2][2] = {0}, ATb[2] = {0};
    for (int i = 0; i < rows; i++) {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];
        ATb[0]    += A[i][0] * b[i];
        ATb[1]    += A[i][1] * b[i];
    }

    float det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabsf(det) < 1e-6f) return false;

    *est_x = (ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det;
    *est_y = (ATA[0][0] * ATb[1] - ATA[1][0] * ATb[0]) / det;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build rssi_vector string
//  Format: {TOF_ANCHOR_0:-65,TOF_ANCHOR_1:-72,TOF_ANCHOR_2:-80}
// ─────────────────────────────────────────────────────────────────────────────
static void build_rssi_vector(char *buf, size_t buf_size)
{
    int offset = 0;
    offset += snprintf(buf + offset, buf_size - offset, "{");
    bool first = true;
    for (int i = 0; i < s_anchor_count; i++) {
        if (!s_anchors[i].valid || s_anchors[i].sample_count == 0) continue;
        offset += snprintf(buf + offset, buf_size - offset,
                           "%s%s:%.0f",
                           first ? "" : ";",
                           s_anchors[i].ssid,
                           s_anchors[i].avg_rssi);
        first = false;
    }
    offset += snprintf(buf + offset, buf_size - offset, "}");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print CSV header once
// ─────────────────────────────────────────────────────────────────────────────
static void print_csv_header(void)
{
    printf("_id,device_id,location_id,floor_id,room_id,timestamp,"
           "confidence,rssi_vector,x,y,scan_number\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print one CSV row
// ─────────────────────────────────────────────────────────────────────────────
static void print_csv_row(float x, float y)
{
    char rssi_vec[256];
    build_rssi_vector(rssi_vec, sizeof(rssi_vec));

    // Timestamp in milliseconds since boot
    int64_t timestamp_ms = esp_timer_get_time() / 1000;

    // Confidence: simple metric based on number of anchors seen
    // 0.0 - 1.0, more anchors = higher confidence
    float confidence = (float)s_anchor_count / (float)NUM_ANCHOR_POSITIONS;

    // _id: device + scan number combined
    char id[32];
    snprintf(id, sizeof(id), "%s_%d", DEVICE_ID, s_scan_number);

    printf("%s,%s,%s,%s,%s,%lld,%.2f,\"%s\",%.1f,%.1f,%d\n",
           id,
           DEVICE_ID,
           LOCATION_ID,
           FLOOR_ID,
           ROOM_ID,
           (long long)timestamp_ms,
           confidence,
           rssi_vec,
           x,
           y,
           s_scan_number);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main localization task
// ─────────────────────────────────────────────────────────────────────────────
static void localization_task(void *pvParam)
{
    print_csv_header();
    int scan_count = 0;

    while (true) {
        scan_once();
        scan_count++;

        if (scan_count < RSSI_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        scan_count = 0;
        compute_distances();

        float est_x = 0, est_y = 0;
        if (trilaterate(&est_x, &est_y)) {
            s_scan_number++;
            print_csv_row(est_x, est_y);
        } else {
            ESP_LOGW(TAG, "Not enough anchors for position fix");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "=== RSSI Trilateration Tag starting ===");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(localization_task, "loc_task", 8192, NULL, 5, NULL);
}