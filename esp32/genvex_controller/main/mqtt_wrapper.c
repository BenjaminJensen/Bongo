#include "mqtt_wrapper.h"
#include "mqtt_client.h"
#include <freertos/semphr.h>

#include "esp_log.h"

#define MAX_SUB (4)

typedef struct {
  char topic[128];
  mqttw_sub_handle_t handle;
} sub_table_entry_t;

static sub_table_entry_t sub_table[MAX_SUB];
static int sub_table_cnt;
static SemaphoreHandle_t subMutex;

static esp_mqtt_client_handle_t local_client;
static SemaphoreHandle_t pubMutex;
static const char* TAG = "MQTT_WRAPPER";

void mqttw_init(void* client) {
  local_client = (esp_mqtt_client_handle_t)client;

  // Create Mutex for publishing clients
	pubMutex = xSemaphoreCreateMutex();
  
  // Create Mutex for publishing clients
	subMutex = xSemaphoreCreateMutex();
  sub_table_cnt = -1;
}

bool mqttw_publish(const char* topic, const char* data, int qos, int retain) {
  bool ret = false;

  /** START WRAP **/
  if(xSemaphoreTake( pubMutex, 100 / portTICK_PERIOD_MS ) == pdTRUE) {
    int msg_id;
    if(local_client != NULL) {
      msg_id = esp_mqtt_client_publish(local_client, topic, data, 0, qos, retain);
      //ESP_LOGI(TAG, "sent publish successful [%s], msg_id=%d", topic,  msg_id);
      ret = true;
    }
    else {
      ESP_LOGW(TAG, "Local client NULL: %d", (int)local_client);
    }
   	// Release Mutex
     xSemaphoreGive( pubMutex );
	}
  else {
    ESP_LOGW( TAG, "Unable to take sem in \"mqttw_publish\" [%s]", topic);
  }
  /** End WRAP **/

  return ret;
}

bool mqttw_subscribe(const char* topic, int qos, mqttw_sub_handle_t handle) {
  bool ret = false;

  if(xSemaphoreTake( subMutex, 100 / portTICK_PERIOD_MS ) == pdTRUE) {
    bool found = false;

    // Evaluate free subscription slots
    if(sub_table_cnt < (MAX_SUB - 1) ) {
      for(int index = 0; index <= sub_table_cnt; index++) {
        ESP_LOGD(TAG, "(mqttw_handle_data) index: %d {sub_table_cnt = %d}", index, sub_table_cnt);
        if(strncmp(topic, sub_table[index].topic, strlen(topic)) == 0) {
          if(sub_table[index].handle != NULL) {
            found = true;
          }
        }
      }

      if(found == false) {
        if(esp_mqtt_client_subscribe(local_client, topic, qos) > -1) {
          sub_table_cnt++;
          sub_table[sub_table_cnt].handle = handle;
          strcpy(sub_table[sub_table_cnt].topic, topic);
          ret = true;
        }
        else {
          ESP_LOGW(TAG, "Error subscribing");
        }
      }
      else {
        ESP_LOGI(TAG, "mqttw_subscribe: topic already subscribed, re-subscribing.");
        if(esp_mqtt_client_subscribe(local_client, topic, qos) < 0) {
          ESP_LOGW(TAG, "Error re-subscribing");
        }
        else {
          ret = true;
        }
      }

    } 
    else {
      ESP_LOGW(TAG, "No free sub slots, MAX: %d", sub_table_cnt);
    }
    // Release Mutex
    xSemaphoreGive( subMutex );
  }
  else {
    ESP_LOGW(TAG, "mqttw_subscribe: Unable to get submux");
  }
  return ret;
}

void mqttw_handle_data(int topic_len, const char* topic, int data_len, const char* data) {
  char tmp[128];

  strncpy(tmp, topic, topic_len);
  tmp[topic_len] = '\0';

  if(xSemaphoreTake( subMutex, 100 / portTICK_PERIOD_MS ) == pdTRUE) {
    bool found = false;

    for(int index = 0; index <= sub_table_cnt; index++) {
      ESP_LOGI(TAG, "(mqttw_handle_data) index: %d {sub_table_cnt = %d}", index, sub_table_cnt);
      if(strncmp(topic, sub_table[index].topic, topic_len) == 0) {
        if(sub_table[index].handle != NULL) {
          (*(sub_table[index].handle))(data, data_len);
          found = true;
        }
        else {
          ESP_LOGW(TAG, "Topic handler NULL: %.*s", topic_len, topic);
        }
      }
    }

    if(found == false) {
      ESP_LOGW(TAG, "Topic not found: %.*s", topic_len, topic);
    }
    // Release Mutex
    xSemaphoreGive( subMutex );
  }
  else {
    ESP_LOGW(TAG, "mqttw_handle_data: Unable to get submux");
  }
}
