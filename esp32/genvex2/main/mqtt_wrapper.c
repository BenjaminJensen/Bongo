#include "mqtt_wrapper.h"
#include "mqtt_client.h"
#include <freertos/semphr.h>

#include "esp_log.h"

static esp_mqtt_client_handle_t local_client;
static SemaphoreHandle_t pubMutex;
static const char* TAG = "MQTT_WRAPPER";

void mqttw_init(void* client) {
  local_client = (esp_mqtt_client_handle_t)client;

  // Create Mutex for publishing clients
	pubMutex = xSemaphoreCreateMutex();
}


bool mqttw_publish(const char* topic, const char* data, int qos) {
  bool ret = false;

  /** START WRAP **/
  if(xSemaphoreTake( pubMutex, 100 / portTICK_PERIOD_MS ) == pdTRUE) {
    int msg_id;
    if(local_client != NULL) {
      msg_id = esp_mqtt_client_publish(local_client, topic, data, 0, qos, 0);
      ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
      ret = true;
    }
    else {
      ESP_LOGI(TAG, "Local client NULL: %d", (int)local_client);
    }
   	// Release Mutex
     xSemaphoreGive( pubMutex );
	}
  /** End WRAP **/

  return ret;
}

bool mqttw_subscribe(const char* topic, int qos, mqttw_sub_handle handle) {
  bool ret = false;


  return ret;
}
