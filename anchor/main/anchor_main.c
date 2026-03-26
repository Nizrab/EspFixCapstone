// =============================================================================
// anchor_main.c  —  ESP32-S3  ToF Indoor Localization  |  FTM Responder Anchor
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
#define ANCHOR_ID           4.1           // Change to 0, 1, 2, 3, 4, or 5
#define ANCHOR_X_CM         0           // X position in the room (cm)
#define ANCHOR_Y_CM         0           // Y position in the room (cm)

// FTM SoftAP settings
#define AP_CHANNEL          6           // All anchors MUST use the same channel
#define AP_MAX_CONN         4

// ─────────────────────────────────────────────────────────────────────────────

static const char *TAG = "ANCHOR";
static char s_ap_ssid[32];

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi Event Handler
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *e = event_data;
                ESP_LOGI(TAG, "AP: station " MACSTR " joined", MAC2STR(e->mac));
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *e = event_data;
                ESP_LOGI(TAG, "AP: station " MACSTR " left", MAC2STR(e->mac));
                break;
            }
            default:
                break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Wi-Fi Initialisation  (AP only, unique IP per anchor)
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // ── SoftAP config (FTM Responder enabled) ────────────────────────────────
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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

    // ── Unique static IP per anchor ───────────────────────────────────────────
    // Anchor 0 → 192.168.1.1
    // Anchor 1 → 192.168.2.1
    // Anchor 2 → 192.168.3.1
    // Anchor 3 → 192.168.4.1
    // Anchor 4 → 192.168.5.1
    // Anchor 5 → 192.168.6.1
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip,      192, 168, ANCHOR_ID + 1, 1);
    IP4_ADDR(&ip_info.gw,      192, 168, ANCHOR_ID + 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255,            0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));

    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_start());

    // Log MAC and IP
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "SoftAP started — SSID: %s  CH: %d  FTM-Responder: ON",
             s_ap_ssid, AP_CHANNEL);
    ESP_LOGI(TAG, "AP IP:   192.168.%d.1", ANCHOR_ID + 1);
    ESP_LOGI(TAG, "AP MAC:  %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ─────────────────────────────────────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "TOF_ANCHOR_%d", ANCHOR_ID);

    ESP_LOGI(TAG, "=== ToF Anchor %d  [x=%d cm, y=%d cm] ===",
             ANCHOR_ID, ANCHOR_X_CM, ANCHOR_Y_CM);

    // NVS init
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_ap();

    ESP_LOGI(TAG, "Anchor %d running — FTM Responder ACTIVE on SSID \"%s\"",
             ANCHOR_ID, s_ap_ssid);

    // Keep main task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}