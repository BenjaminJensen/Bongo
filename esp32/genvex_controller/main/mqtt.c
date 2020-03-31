#include "mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "mqtt_wrapper.h"
#include "com.h"

/************************************
* Declarations
************************************/
static const char *TAG = "GENVEX_MQTT";
static uint8_t last_speed;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);
static void mqtt_app_start(void);
static void mqtt_hass_reqister(void);
static void set_rotor_speed(const char* data, int len);
static void set_rotor_on(const char* data, int len);
static void set_rotor_state(uint8_t s);

esp_mqtt_client_handle_t client;

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
 * Updates the rotor status over MQTT
 */

int mqtt_update_rotor_status(rotor_status_t* status) {
    int ret = -1;
    char buf[128];
    int len = 0;
    ESP_LOGI(TAG, "update rorot status: t: %.2f, h: %.2f, p: %.2f, mt: %.2f, RPM: %d, state: %d", status->temp, status->humi, status->pres, status->motor_temp, status->rotor_rpm, status->state);

    len = sprintf(buf, "{\"t\": %.2f, \"h\": %.2f, \"p\": %.2f, \"mt\": %.2f, \"rrpm\": %d, \"s\": %d}",status->temp, status->humi, status->pres, status->motor_temp, status->rotor_rpm, status->state);
    if(len < 128) {
        buf[len] = '\0'; // Terminate string
        mqttw_publish("homeassistant/sensor/genvex/rotor/state", buf, 0);
        ret  = 0;
    } 
    else {
        ESP_LOGW(TAG, "Buffer overflow (%d >= 128)! (mqtt_update_rotor_status)", len);
    }

    return ret;
}

/************************************
* Private Functions
************************************/

static const char* rotor_fan_on_state_topic = "homeassistant/fan/genvex/rotor/on/state";
static const char* rotor_fan_on_set_topic = "homeassistant/fan/genvex/rotor/on/set";
static const char* rotor_fan_speed_state_topic = "homeassistant/fan/genvex/rotor/speed/state";
static const char* rotor_fan_command_topic = "homeassistant/fan/genvex/rotor/speed/set";
static const char* rotor_fan_config_topic = "homeassistant/fan/genvex/rotor/config";

static void mqtt_hass_reqister(void) {

    //-----------------------------
    // in-pre
    //-----------------------------
    // in-preT
    const char* in_pre_t_config_topic = "homeassistant/sensor/genvex/in-preT/config";
    const char* in_pre_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"In-pre Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-in-pre-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_t_config_topic, in_pre_t_config_payload, 0);
    
    // in-preH
    const char* in_pre_h_config_topic = "homeassistant/sensor/genvex/in-preH/config";
    const char* in_pre_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"In-pre Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-in-pre-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_h_config_topic, in_pre_h_config_payload, 0);
    
    // in-preP
    const char* in_pre_p_config_topic = "homeassistant/sensor/genvex/in-preP/config";
    const char* in_pre_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"In-pre Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-in-pre-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_p_config_topic, in_pre_p_config_payload, 0);

    //-----------------------------
    // in-post
    //-----------------------------
    // in-postT
    const char* in_post_t_config_topic = "homeassistant/sensor/genvex/in-postT/config";
    const char* in_post_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"in-post Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-in-post-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_t_config_topic, in_post_t_config_payload, 0);
    
    // in-postH
    const char* in_post_h_config_topic = "homeassistant/sensor/genvex/in-postH/config";
    const char* in_post_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"in-post Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-in-post-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_h_config_topic, in_post_h_config_payload, 0);
    
    // in-postP
    const char* in_post_p_config_topic = "homeassistant/sensor/genvex/in-postP/config";
    const char* in_post_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"in-post Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-in-post-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_p_config_topic, in_post_p_config_payload, 0);

    //-----------------------------
    // out-pre
    //-----------------------------
    // out-preT
    const char* out_pre_t_config_topic = "homeassistant/sensor/genvex/out-preT/config";
    const char* out_pre_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"out-pre Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-out-pre-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_t_config_topic, out_pre_t_config_payload, 0);
    
    // out-preH
    const char* out_pre_h_config_topic = "homeassistant/sensor/genvex/out-preH/config";
    const char* out_pre_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"out-pre Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-out-pre-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_h_config_topic, out_pre_h_config_payload, 0);
    
    // out-preP
    const char* out_pre_p_config_topic = "homeassistant/sensor/genvex/out-preP/config";
    const char* out_pre_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"out-pre Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-out-pre-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_p_config_topic, out_pre_p_config_payload, 0);

    //-----------------------------
    // out-post
    //-----------------------------
    // out-postT
    const char* out_post_t_config_topic = "homeassistant/sensor/genvex/out-postT/config";
    const char* out_post_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"out-post Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-out-post-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_t_config_topic, out_post_t_config_payload, 0);
    
    // out-postH
    const char* out_post_h_config_topic = "homeassistant/sensor/genvex/out-postH/config";
    const char* out_post_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"out-post Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-out-post-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_h_config_topic, out_post_h_config_payload, 0);
    
    // out-postP
    const char* out_post_p_config_topic = "homeassistant/sensor/genvex/out-postP/config";
    const char* out_post_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"out-post Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-out-post-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_p_config_topic, out_post_p_config_payload, 0);
    
    //-----------------------------
    // Rotor fan
    //-----------------------------

    const char* rotor_fan_config_payload = "{\"payload_on\": 1, \"payload_off\": 0, \"payload_low_speed\": 1, \"payload_medium_speed\": 2, \"payload_high_speed\": 3, \"name\": \"Genvex Rotor\",\"state_topic\": \"homeassistant/fan/genvex/rotor/on/state\",\"command_topic\": \"homeassistant/fan/genvex/rotor/on/set\",\"speed_state_topic\": \"homeassistant/fan/genvex/rotor/speed/state\",\"speed_command_topic\": \"homeassistant/fan/genvex/rotor/speed/set\",\"unique_id\": \"genvex-rotor-fan\",\"speeds\":[\"off\", \"low\", \"medium\",\"high\"],\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_fan_config_topic, rotor_fan_config_payload, 0);

    mqttw_subscribe(rotor_fan_on_set_topic, 0, set_rotor_on);
    mqttw_subscribe(rotor_fan_command_topic, 0, set_rotor_speed);

    //-----------------------------
    // Rotor sensors
    //-----------------------------
    // rotorT
    const char* rotor_t_config_topic = "homeassistant/sensor/genvex/rotorT/config";
    const char* rotor_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"Rotor Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-rotor-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_t_config_topic, rotor_t_config_payload, 0);
    
    // rotorH
    const char* rotor_h_config_topic = "homeassistant/sensor/genvex/rotorH/config";
    const char* rotor_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"Rotor Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-rotor-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_h_config_topic, rotor_h_config_payload, 0);
    
    // rotorP
    const char* rotor_p_config_topic = "homeassistant/sensor/genvex/rotorP/config";
    const char* rotor_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"Rotor Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-rotor-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_p_config_topic, rotor_p_config_payload, 0);

    // rotorMT
    const char* rotor_mt_config_topic = "homeassistant/sensor/genvex/rotorMT/config";
    const char* rotor_mt_config_payload = "{\"device_class\": \"temperature\",\"name\": \"Rotor Motor Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.mt}}\",\"unique_id\": \"genvex-rotor-mt\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_mt_config_topic, rotor_mt_config_payload, 0);
    
    // rotor rpm
    const char* rotor_rpm_config_topic = "homeassistant/sensor/genvex/rotorRPM/config";
    const char* rotor_rpm_config_payload = "{\"name\": \"Rotor RPM\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"RPM\", \"value_template\": \"{{ value_json.rrpm}}\",\"unique_id\": \"genvex-rotor-rrpm\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_rpm_config_topic, rotor_rpm_config_payload, 0);
    
    // rotor state
    const char* rotor_speed_config_topic = "homeassistant/sensor/genvex/rotorS/config";
    const char* rotor_speed_config_payload = "{\"name\": \"Rotor Speed\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"value_template\": \"{{ value_json.s}}\",\"unique_id\": \"genvex-rotor-speed\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_speed_config_topic, rotor_speed_config_payload, 0);
    
    //-----------------------------
    // Publish
    //-----------------------------

    // in-pre
    const char* in_pre_state_topic = "homeassistant/sensor/genvex/in-pre/state";
    mqttw_publish(in_pre_state_topic,"{\"t\":11,\"h\":21,\"p\":321.4}" , 0);

    // in-post
    const char* in_post_state_topic = "homeassistant/sensor/genvex/in-post/state";
    mqttw_publish(in_post_state_topic,"{\"t\":12,\"h\":22,\"p\":322.4}" , 0);

    // out-pre
    const char* out_pre_state_topic = "homeassistant/sensor/genvex/out-pre/state";
    mqttw_publish(out_pre_state_topic,"{\"t\":13,\"h\":23,\"p\":323.4}" , 0);

    // out-post
    const char* out_post_state_topic = "homeassistant/sensor/genvex/out-post/state";
    mqttw_publish(out_post_state_topic,"{\"t\":14,\"h\":24,\"p\":324.4}" , 0);
    
    // Rotor fan
    //const char* rotor_fan_on_state_topic = "homeassistant/fan/genvex/rotor/on/state";
    mqttw_publish(rotor_fan_on_state_topic,"1" , 0);

    //const char* rotor_fan_speed_state_topic = "homeassistant/fan/genvex/rotor/speed/state";
    mqttw_publish(rotor_fan_speed_state_topic,"2" , 0);

    ESP_LOGI(TAG,"Publish");
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
        .client_id ="Genvex Controller"
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
        mqttw_publish(rotor_fan_on_state_topic,"0" , 0);
        speed = 0;
        set_rotor_state(speed);
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
    mqttw_publish(rotor_fan_speed_state_topic, buf, 0);

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

    set_rotor_state(state);
}

/**
 * Updates the ON / OFF state of the rotor
 * 
 */
static void set_rotor_state(uint8_t s) {
    char buf[2];

    buf[0] = s + '0';
    buf[1] = 0; 
    mqttw_publish(rotor_fan_on_state_topic, buf, 0);

    // TODO: Send command to rotor controller
}