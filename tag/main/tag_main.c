// =============================================================================
// tag_main.c  —  ESP32-S3  FTM Trilateration Tag  (HTTP POST to backend)
// =============================================================================
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "lwip/ip4_addr.h"

// ─────────────────────────────────────────────────────────────────────────────
//  PER-TAG CONFIGURATION  ← Edit before flashing each tag
// ─────────────────────────────────────────────────────────────────────────────
#define TAG_NUM         0               // Change per tag: 0, 1, 2...
#define DEVICE_ID       "esp32s3_tag_0" // Change per tag
#define LOCATION_ID     "lab_1"
#define FLOOR_ID        "floor_1"
#define ROOM_ID         "room_1"

// Network — same for all tags
#define WIFI_STA_SSID   "EAP1250"
#define WIFI_STA_PASS   ""              // open network

// Backend server
#define SERVER_HOST     "192.168.1.157"
#define SERVER_PORT     8080
#define SERVER_ENDPOINT "/engine/status"

// Tag SoftAP settings
#define TAG_AP_CHANNEL  1
#define TAG_AP_MAX_CONN 4
#define TAG_AP_IP_LAST  100             // 10.0.0.100 — change per tag if needed

// FTM settings
#define MAX_ANCHORS         6
#define SCAN_INTERVAL_MS    2000
#define FTM_FRAME_COUNT     16
#define FTM_BURST_PERIOD    2
#define WIFI_RETRY_MAX      10

// Anchor positions in cm — update to match physical layout
static const struct {
    int   id;
    float x;
    float y;
} ANCHOR_POSITIONS[] = {
    { 61,    0,    0 },
    { 62,  300,    0 },
    { 63,  150,  300 },
    { 64,    0,  300 },
    { 65,  300,  300 },
    { 66,  150,    0 },
};
#define NUM_ANCHOR_POSITIONS 6

// ─────────────────────────────────────────────────────────────────────────────

static const char *TAG = "TAG";
static char s_tag_ap_ssid[32];
static int  s_scan_number = 0;
static int  s_retry_num   = 0;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define FTM_DONE_BIT        BIT2
#define FTM_FAILED_BIT      BIT3

typedef struct {
    char     ssid[32];
    uint8_t  bssid[6];
    int      anchor_id;
    uint8_t  channel;
    float    x_cm;
    float    y_cm;
    uint32_t rtt_ps;
    float    distance_cm;
    bool     valid;
    bool     ftm_done;
} anchor_info_t;

static anchor_info_t s_anchors[MAX_ANCHORS];
static int           s_anchor_count = 0;
static uint32_t      s_rtt_ps       = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi + FTM Event Handler
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {

            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                ESP_LOGI(TAG, "STA started, connecting to %s...", WIFI_STA_SSID);
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < WIFI_RETRY_MAX) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGW(TAG, "STA disconnected, retry %d/%d",
                             s_retry_num, WIFI_RETRY_MAX);
                } else {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    ESP_LOGE(TAG, "STA failed after %d retries", WIFI_RETRY_MAX);
                }
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *e = event_data;
                ESP_LOGI(TAG, "AP: device " MACSTR " connected", MAC2STR(e->mac));
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *e = event_data;
                ESP_LOGI(TAG, "AP: device " MACSTR " disconnected", MAC2STR(e->mac));
                break;
            }

            case WIFI_EVENT_FTM_REPORT: {
                wifi_event_ftm_report_t *report = event_data;

                if (report->status == FTM_STATUS_SUCCESS) {
                    uint8_t num = report->ftm_report_num_entries;
                    wifi_ftm_report_entry_t *entries = malloc(
                        sizeof(wifi_ftm_report_entry_t) * num);

                    if (entries) {
                        esp_err_t err = esp_wifi_ftm_get_report(entries, num);
                        if (err == ESP_OK && num > 0) {
                            uint64_t total = 0;
                            for (int i = 0; i < num; i++) {
                                total += entries[i].rtt;
                            }
                            s_rtt_ps = (uint32_t)(total / num);
                            ESP_LOGI(TAG, "FTM OK — RTT: %lu ps  entries: %d",
                                     s_rtt_ps, num);
                        } else {
                            s_rtt_ps = 0;
                        }
                        free(entries);
                    }
                    xEventGroupSetBits(s_wifi_event_group, FTM_DONE_BIT);
                } else {
                    ESP_LOGW(TAG, "FTM failed — status: %d", report->status);
                    xEventGroupSetBits(s_wifi_event_group, FTM_FAILED_BIT);
                }
                break;
            }

            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = event_data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi Init (AP+STA)
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_init_apsta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // ── SoftAP config ─────────────────────────────────────────────────────────
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid_len       = (uint8_t)strlen(s_tag_ap_ssid),
            .channel        = TAG_AP_CHANNEL,
            .max_connection = TAG_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,
        }
    };
    memcpy(ap_cfg.ap.ssid, s_tag_ap_ssid, strlen(s_tag_ap_ssid));

    // ── STA config ────────────────────────────────────────────────────────────
    wifi_config_t sta_cfg = { 0 };
    strlcpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid));
    memset(sta_cfg.sta.password, 0, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    // ── Static IP for AP interface ────────────────────────────────────────────
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip,      10, 0, 0, (uint8_t)TAG_AP_IP_LAST);
    IP4_ADDR(&ip_info.gw,      10, 0, 0, (uint8_t)TAG_AP_IP_LAST);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "Tag AP: SSID=%s  IP=10.0.0.%d", s_tag_ap_ssid, TAG_AP_IP_LAST);
    ESP_LOGI(TAG, "Tag MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Wait for STA to connect
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA connected to \"%s\"", WIFI_STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA connection failed — will retry in background");
    }
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
//  Scan for TOF_ANCHOR_x SSIDs
// ─────────────────────────────────────────────────────────────────────────────
static void scan_for_anchors(void)
{
    s_anchor_count = 0;
    memset(s_anchors, 0, sizeof(s_anchors));

    wifi_scan_config_t scan_cfg = {
        .ssid        = NULL,
        .bssid       = NULL,
        .channel     = 0,
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

    for (int i = 0; i < ap_count && s_anchor_count < MAX_ANCHORS; i++) {
        if (strncmp((char *)ap_list[i].ssid, "TOF_ANCHOR_", 11) != 0) continue;

        int anchor_id = atoi((char *)ap_list[i].ssid + 11);
        int idx       = s_anchor_count++;

        strlcpy(s_anchors[idx].ssid, (char *)ap_list[i].ssid,
                sizeof(s_anchors[idx].ssid));
        memcpy(s_anchors[idx].bssid, ap_list[i].bssid, 6);
        s_anchors[idx].channel   = ap_list[i].primary;
        s_anchors[idx].anchor_id = anchor_id;
        s_anchors[idx].valid     = true;
        s_anchors[idx].ftm_done  = false;
        get_anchor_position(anchor_id,
                            &s_anchors[idx].x_cm,
                            &s_anchors[idx].y_cm);

        ESP_LOGI(TAG, "Found: %s  BSSID: " MACSTR "  CH: %d",
                 s_anchors[idx].ssid,
                 MAC2STR(s_anchors[idx].bssid),
                 ap_list[i].primary);
    }

    free(ap_list);
    ESP_LOGI(TAG, "Total anchors found: %d", s_anchor_count);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Run FTM session
// ─────────────────────────────────────────────────────────────────────────────
static bool run_ftm(anchor_info_t *anchor)
{
    xEventGroupClearBits(s_wifi_event_group, FTM_DONE_BIT | FTM_FAILED_BIT);

    wifi_ftm_initiator_cfg_t ftm_cfg = {
        .frm_count    = FTM_FRAME_COUNT,
        .burst_period = FTM_BURST_PERIOD,
        .channel      = anchor->channel,
    };
    memcpy(ftm_cfg.resp_mac, anchor->bssid, 6);

    esp_err_t err = esp_wifi_ftm_initiate_session(&ftm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FTM initiate failed: %s", esp_err_to_name(err));
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           FTM_DONE_BIT | FTM_FAILED_BIT,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(5000));

    if (bits & FTM_DONE_BIT && s_rtt_ps > 0) {
        anchor->rtt_ps      = s_rtt_ps;
        anchor->distance_cm = (s_rtt_ps * 0.03f) / 2.0f;
        anchor->ftm_done    = true;
        ESP_LOGI(TAG, "  %s → RTT: %lu ps  Dist: %.1f cm",
                 anchor->ssid, anchor->rtt_ps, anchor->distance_cm);
        return true;
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trilateration
// ─────────────────────────────────────────────────────────────────────────────
static bool trilaterate(float *est_x, float *est_y)
{
    float ax[MAX_ANCHORS], ay[MAX_ANCHORS], ad[MAX_ANCHORS];
    int   n = 0;

    for (int i = 0; i < s_anchor_count; i++) {
        if (!s_anchors[i].valid || !s_anchors[i].ftm_done) continue;
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
//  Send data to backend via HTTP POST
// ─────────────────────────────────────────────────────────────────────────────
static void send_to_server(float x, float y)
{
    char payload[512];
    int  offset = 0;

    offset += snprintf(payload + offset, sizeof(payload) - offset,
                       "{\"tag_id\":\"%s\","
                       "\"x_cm\":%.1f,"
                       "\"y_cm\":%.1f,"
                       "\"timestamp\":%lld,"
                       "\"scan_number\":%d,"
                       "\"anchors\":[",
                       DEVICE_ID, x, y,
                       (long long)(esp_timer_get_time() / 1000),
                       s_scan_number);

    bool first = true;
    for (int i = 0; i < s_anchor_count; i++) {
        if (!s_anchors[i].valid || !s_anchors[i].ftm_done) continue;
        offset += snprintf(payload + offset, sizeof(payload) - offset,
                           "%s{\"anchor\":\"%s\","
                           "\"rtt_ps\":%lu,"
                           "\"distance_cm\":%.1f}",
                           first ? "" : ",",
                           s_anchors[i].ssid,
                           s_anchors[i].rtt_ps,
                           s_anchors[i].distance_cm);
        first = false;
    }

    offset += snprintf(payload + offset, sizeof(payload) - offset, "]}");

    char url[64];
    snprintf(url, sizeof(url), "http://%s:%d%s",
             SERVER_HOST, SERVER_PORT, SERVER_ENDPOINT);

    esp_http_client_config_t config = {
        .url    = url,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, payload, strlen(payload));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "POST OK — X=%.1f Y=%.1f  status=%d",
                 x, y, esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main FTM task
// ─────────────────────────────────────────────────────────────────────────────
static void ftm_task(void *pvParam)
{
    while (true) {
        scan_for_anchors();

        if (s_anchor_count == 0) {
            ESP_LOGW(TAG, "No anchors found, retrying...");
            vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
            continue;
        }

        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "FTM ranging — %s", DEVICE_ID);
        ESP_LOGI(TAG, "========================================");

        for (int i = 0; i < s_anchor_count; i++) {
            run_ftm(&s_anchors[i]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        float est_x = 0, est_y = 0;
        if (trilaterate(&est_x, &est_y)) {
            s_scan_number++;
            ESP_LOGI(TAG, "Position: X=%.1f cm  Y=%.1f cm", est_x, est_y);
            send_to_server(est_x, est_y);
        } else {
            ESP_LOGW(TAG, "Not enough FTM measurements for position fix");
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    snprintf(s_tag_ap_ssid, sizeof(s_tag_ap_ssid), "TOF_TAG_%d", TAG_NUM);

    ESP_LOGI(TAG, "=== FTM Tag %d starting ===", TAG_NUM);

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_apsta();
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(ftm_task, "ftm_task", 8192, NULL, 5, NULL);
}