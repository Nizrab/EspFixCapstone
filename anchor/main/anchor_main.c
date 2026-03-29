// =============================================================================
// anchor_main.c  —  ESP32-S3  Indoor Localization  |  AP+STA Anchor
// =============================================================================
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_timer.h"
#include "lwip/ip4_addr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"

// ─────────────────────────────────────────────────────────────────────────────
//  PER-ANCHOR CONFIGURATION  ← Edit before flashing each unit
// ─────────────────────────────────────────────────────────────────────────────
#define ANCHOR_ID           62       // Change to 0, 1, 2, 3, 4, or 5
#define ANCHOR_X_CM         0       // X position in room (cm)
#define ANCHOR_Y_CM         0       // Y position in room (cm)

// Your network credentials — same for all anchors
#define WIFI_STA_SSID       "EAP1250"
//#define WIFI_STA_PASS       ""

// SoftAP settings
#define AP_CHANNEL          6       // All anchors MUST use the same channel
#define AP_MAX_CONN         4
#define WIFI_RETRY_MAX      5

// ─────────────────────────────────────────────────────────────────────────────

static const char *TAG = "ANCHOR";
static char s_ap_ssid[32];

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static int s_retry_num = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi Event Handler
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
//  Wi-Fi Initialisation  (AP + STA concurrent mode)
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
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,
            .ftm_responder  = true,
        }
    };
    memcpy(ap_cfg.ap.ssid, s_ap_ssid, strlen(s_ap_ssid));
    ap_cfg.ap.ssid_len = (uint8_t)strlen(s_ap_ssid);

    // ── STA config ────────────────────────────────────────────────────────────
    wifi_config_t sta_cfg = { 0 };
    strlcpy((char *)sta_cfg.sta.ssid,     WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid));
    memset(sta_cfg.sta.password, 0, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    // ── Unique static IP per anchor ───────────────────────────────────────────
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip_info;
    uint8_t subnet = (uint8_t)(ANCHOR_ID);
    IP4_ADDR(&ip_info.ip,      192, 168, 1, subnet);
    IP4_ADDR(&ip_info.gw,      192, 168, 1, 2);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255,    0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_start());
    // Set static IP on STA interface
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    esp_netif_dhcpc_stop(sta_netif);

    esp_netif_ip_info_t sta_ip_info;
    IP4_ADDR(&sta_ip_info.ip,      192, 168, 1, (uint8_t)ANCHOR_ID);
    IP4_ADDR(&sta_ip_info.gw,      192, 168, 1, 2);
    IP4_ADDR(&sta_ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(sta_netif, &sta_ip_info);

    ESP_LOGI(TAG, "STA static IP: 192.168.1.%d  GW: 192.168.1.2", (int)ANCHOR_ID);
    // Log AP info
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "SoftAP started — SSID: %s  CH: %d", s_ap_ssid, AP_CHANNEL);
    ESP_LOGI(TAG, "AP IP:  192.168.1.%d",(int)ANCHOR_ID);
    ESP_LOGI(TAG, "AP MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Wait for STA connection (non-fatal if it fails — AP still works)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA connected to \"%s\"", WIFI_STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA connection failed — AP still active");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "TOF_ANCHOR_%d", (int)ANCHOR_ID);

    ESP_LOGI(TAG, "=== ToF Anchor %d  [x=%d cm, y=%d cm] ===",
             (int)ANCHOR_ID, ANCHOR_X_CM, ANCHOR_Y_CM);

    // NVS init
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_apsta();

    ESP_LOGI(TAG, "Anchor %d running — SSID \"%s\" active",
             (int)ANCHOR_ID, s_ap_ssid);

    // Keep main task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}