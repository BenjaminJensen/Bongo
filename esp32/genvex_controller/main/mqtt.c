#include "mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "mqtt_wrapper.h"
#include "com.h"
#include "control.h"
#include "mqtt_hass.h"

/************************************
* Module definitions
************************************/
static const char *TAG = "GENVEX_MQTT";
static uint8_t last_speed;
esp_mqtt_client_handle_t client;

/************************************
* Global MQTT topics
************************************/
static const char* rotor_fan_on_state_topic = "homeassistant/fan/genvex/rotor/on/state";
static const char* rotor_fan_on_set_topic = "homeassistant/fan/genvex/rotor/on/set";
static const char* rotor_fan_speed_state_topic = "homeassistant/fan/genvex/rotor/speed/state";
static const char* rotor_fan_command_topic = "homeassistant/fan/genvex/rotor/speed/set";
static const char* genvex_status_topic = "homeassistant/sensor/genvex/state";
static const char* genvex_ambient_topic = "homeassistant/sensor/genvex/ambi/state";

// New
const char* speed_set_topic = "homeassistant/sensor/genvex/speed/set";

/************************************
* Forvard declarations
************************************/
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);
static void mqtt_app_start(void);
static void set_rotor_speed(const char* data, int len);
static void set_rotor_on(const char* data, int len);
static void set_rotor_state(uint8_t s);
static void set_genvex(const char* data, int len);
static void mqtt_sub(void);

/************************************
* Public Functions
************************************/

int mqtt_init(void) {
    last_speed = 2;
    mqtt_app_start();
    mqttw_init(client);

    return 1;
}

/**
 * Updates ambient BME280 status over MQTT
 */
int mqtt_update_bme280_status(float t, float p, float h) {

    int ret = -1;
    char buf[64];
    int len = 0;
    len = sprintf(buf, "{\"t\": %.2f, \"h\": %.2f, \"p\": %.2f}", t, h, p);
    buf[len] = 0;
    mqttw_publish(genvex_ambient_topic, buf, 0, 0);

    ESP_LOGI(TAG, "%s",buf);
    return ret;
}
/**
 * Updates the rotor status over MQTT
 */
int mqtt_update_rotor_status(rotor_status_t* status) {
    int ret = -1;
    char buf[128];
    int len = 0;
    //ESP_LOGI(TAG, "update rorot status: t: %.2f, h: %.2f, p: %.2f, mt: %.2f, RPM: %d, AVG RPM: %d, state: %d, Temp Fault: %d, Rotor Fault: %d", status->temp, status->humi, status->pres, status->motor_temp, status->rotor_rpm, status->rotor_rpm_avg, status->state, status->temp_fault, status->rotor_fault);

    len = sprintf(buf, "{\"t\": %.2f, \"h\": %.2f, \"p\": %.2f, \"mt\": %.2f, \"rrpm\": %d, \"arpm\": %d, \"s\": %d, \"rf\": %d, \"tf\": %d}",
    status->temp, status->humi, status->pres, status->motor_temp, status->rotor_rpm, status->rotor_rpm_avg,status->state, status->rotor_fault, status->temp_fault);
    if(len < 128) {
        buf[len] = '\0'; // Terminate string
        mqttw_publish("homeassistant/sensor/genvex/rotor/state", buf, 0, 0);
        ret  = 0;
        
        ESP_LOGI(TAG,"%s", buf);
    } 
    else {
        ESP_LOGW(TAG, "Buffer overflow (%d >= 128)! (mqtt_update_rotor_status)", len);
    }
    buf[0] = status->state + '0';
    buf[1] = '\0';

    mqttw_publish(rotor_fan_speed_state_topic, buf, 0, 0);

    if(status->state > 0)
        buf[0] = '1';
    else {
        buf[0] = '0';
    }
    
    
    mqttw_publish(rotor_fan_on_state_topic, buf, 0, 0);
    
    return ret;
}

/**
 * Updates control status over MQTT
 */
int mqtt_update_control_status(control_status_t* status) {

    int ret = -1;
    char buf[32];
    int len = 0;
    len = sprintf(buf, "{\"s\": %d}", status->speed);
    buf[len] = 0;
    mqttw_publish(genvex_status_topic, buf, 0, 0);

    ESP_LOGI(TAG, "%s",buf);
    return ret;
}

/************************************
* Private Functions
************************************/
static void mqtt_sub(void) {
    
    mqttw_subscribe(rotor_fan_on_set_topic, 0, set_rotor_on);
    mqttw_subscribe(rotor_fan_command_topic, 0, set_rotor_speed);
    mqttw_subscribe(speed_set_topic, 0, set_genvex);
}


/************************************
* Private MQTT EVENT Functions
************************************/

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    //esp_mqtt_client_handle_t client = event->client;
    //int msg_id;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
           
            // Register all components with Home Assistant 
            mqtt_hass_reqister();

            // Subscribe
            mqtt_sub();

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
           
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA");
            mqttw_handle_data(event->topic_len, event->topic, event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        .client_id ="Genvex Controller",
        .task_stack = 16384,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}


/************************************
* Private MQTT Subscribe handler Functions
************************************/

static void set_rotor_speed(const char* data, int len)
{
    char buf[2];
    uint8_t speed;
    
    speed = *data;

    if(speed == 'o') {
        mqttw_publish(rotor_fan_on_state_topic,"0" , 0, 0);
        speed = 0;
        com_set_rotor_speed(speed);
    }
    else {
        if(len > 1)
            ESP_LOGW(TAG, "Speed len: %d", len);
        speed -= '0';
        
    }
    last_speed = speed;

    com_set_rotor_speed(speed);
    buf[0] = speed + '0';
    buf[1] = 0; // String termination
    //mqttw_publish(rotor_fan_speed_state_topic, buf, 0);

    // TODO: send command to rotor controller
    ESP_LOGI(TAG, "Change speed: %d", speed);
}

static void set_rotor_on(const char* data, int len)
{
    if(len > 1)
        ESP_LOGW(TAG, "state len: %d", len);

    uint8_t state = *data - '0';
    ESP_LOGI(TAG, "Change State : %d", state);

    if(state == 1) { // Turn on
        com_set_rotor_speed(last_speed);
    } 
    else { // Turn off
        com_set_rotor_speed(0);
    }

    //set_rotor_state(state);
}

/**
 * Updates the ON / OFF state of the rotor
 * 
 */
static void set_rotor_state(uint8_t s) {
    char buf[2];

    buf[0] = s + '0';
    buf[1] = 0; 
    //mqttw_publish(rotor_fan_on_state_topic, buf, 0);

    // TODO: Send command to rotor controller
    set_rotor_state(s);
}

static void set_genvex(const char* data, int len) {
    uint8_t tmp;
    tmp = *data;
    if( tmp >= 48 && tmp <= 51) // ASCII '0' and '3'
        tmp -= '0';
    control_update_speed(tmp);
}