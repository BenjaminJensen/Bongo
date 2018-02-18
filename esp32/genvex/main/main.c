#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <nvs_flash.h>


#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "genvex_sensors.h"
#include "genvex_control.h"
#include "aws_client.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

void initialize_wifi(void)
{
    // Disable wifi driver logging
    esp_log_level_set("wifi", ESP_LOG_NONE);
    wifi_event_group = xEventGroupCreate();
	nvs_flash_init();
    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

/* ************************************************************************* *
 * Connect to the wireless network defined via menuconfig, using the supplied
 * passphrase.
 * ************************************************************************* */
void wifi_connect(void)
{
    wifi_config_t cfg = {
        .sta = {
            .ssid     = "Boing",
            .password = "benster101",
        },
    };

    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }

    return ESP_OK;
}
void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    initialize_wifi();
    init_aws_client(wifi_event_group, CONNECTED_BIT);

	//init_genvex_sensor();
	init_genvex_control();
}
