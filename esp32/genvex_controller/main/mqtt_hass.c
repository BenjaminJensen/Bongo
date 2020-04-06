#include "mqtt_hass.h"
#include "mqtt_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "MQTT_HASS";
static int register_in_pre(void);
static int register_in_post(void);
static int register_out_pre(void);
static int register_out_post(void);

static int register_rotor_fan(void);
static int register_rotor_sensors(void);

static int register_genvex(void);


void mqtt_hass_reqister(void) {


    register_rotor_fan();
    register_rotor_sensors();

    register_genvex();

    register_in_pre();
    register_in_post();
    register_out_pre();
    register_out_post();


    //-----------------------------
    // Publish
    //-----------------------------

    // in-pre
    const char* in_pre_state_topic = "homeassistant/sensor/genvex/in-pre/state";
    mqttw_publish(in_pre_state_topic,"{\"t\":11,\"h\":21,\"p\":321.4}" , 0, 0);

    vTaskDelay(50 / portTICK_PERIOD_MS );
    
    // in-post
    const char* in_post_state_topic = "homeassistant/sensor/genvex/in-post/state";
    mqttw_publish(in_post_state_topic,"{\"t\":12,\"h\":22,\"p\":322.4}" , 0, 0);

    vTaskDelay(50 / portTICK_PERIOD_MS );
    
    // out-pre
    const char* out_pre_state_topic = "homeassistant/sensor/genvex/out-pre/state";
    mqttw_publish(out_pre_state_topic,"{\"t\":13,\"h\":23,\"p\":323.4}" , 0, 0);

    vTaskDelay(50 / portTICK_PERIOD_MS );

    
    // out-post
    const char* out_post_state_topic = "homeassistant/sensor/genvex/out-post/state";
    mqttw_publish(out_post_state_topic,"{\"t\":14,\"h\":24,\"p\":324.4}" , 0, 0);
   
    vTaskDelay(50 / portTICK_PERIOD_MS );
    
    ESP_LOGI(TAG,"Publish");
}

/**
 * Register BME280 In flow pre rotor
 */
static int register_in_pre(void) {
    //-----------------------------
    // in-pre
    //-----------------------------
    // in-preT
    const char* in_pre_t_config_topic = "homeassistant/sensor/genvex/in-preT/config";
    const char* in_pre_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"In-pre Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-in-pre-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_t_config_topic, in_pre_t_config_payload, 0, 1);
    
    // in-preH
    const char* in_pre_h_config_topic = "homeassistant/sensor/genvex/in-preH/config";
    const char* in_pre_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"In-pre Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-in-pre-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_h_config_topic, in_pre_h_config_payload, 0, 1);
    
    // in-preP
    const char* in_pre_p_config_topic = "homeassistant/sensor/genvex/in-preP/config";
    const char* in_pre_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"In-pre Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/in-pre/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-in-pre-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_pre_p_config_topic, in_pre_p_config_payload, 0, 1);    

    return 0;
}

/**
 * Register BME280 In flow post rotor
 */
static int register_in_post(void) {
    //-----------------------------
    // in-post
    //-----------------------------
    // in-postT
    const char* in_post_t_config_topic = "homeassistant/sensor/genvex/in-postT/config";
    const char* in_post_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"in-post Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-in-post-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_t_config_topic, in_post_t_config_payload, 0, 1);
        
    // in-postH
    const char* in_post_h_config_topic = "homeassistant/sensor/genvex/in-postH/config";
    const char* in_post_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"in-post Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-in-post-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_h_config_topic, in_post_h_config_payload, 0, 1);
    
    // in-postP
    const char* in_post_p_config_topic = "homeassistant/sensor/genvex/in-postP/config";
    const char* in_post_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"in-post Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/in-post/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-in-post-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(in_post_p_config_topic, in_post_p_config_payload, 0, 1);

    return 0;
}

/**
 * Register BME280 out flow pre rotor
 */
static int register_out_pre(void) {
    //-----------------------------
    // out-pre
    //-----------------------------
    // out-preT
    const char* out_pre_t_config_topic = "homeassistant/sensor/genvex/out-preT/config";
    const char* out_pre_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"out-pre Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-out-pre-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_t_config_topic, out_pre_t_config_payload, 0, 1);
    
    // out-preH
    const char* out_pre_h_config_topic = "homeassistant/sensor/genvex/out-preH/config";
    const char* out_pre_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"out-pre Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-out-pre-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_h_config_topic, out_pre_h_config_payload, 0, 1);
    
    // out-preP
    const char* out_pre_p_config_topic = "homeassistant/sensor/genvex/out-preP/config";
    const char* out_pre_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"out-pre Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/out-pre/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-out-pre-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_pre_p_config_topic, out_pre_p_config_payload, 0, 1);

    return 0;
}

/**
 * Register BME280 out flow post rotor
 */
static int register_out_post(void) {
    //-----------------------------
    // out-post
    //-----------------------------
    // out-postT
    const char* out_post_t_config_topic = "homeassistant/sensor/genvex/out-postT/config";
    const char* out_post_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"out-post Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-out-post-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_t_config_topic, out_post_t_config_payload, 0, 1);
    
    // out-postH
    const char* out_post_h_config_topic = "homeassistant/sensor/genvex/out-postH/config";
    const char* out_post_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"out-post Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-out-post-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_h_config_topic, out_post_h_config_payload, 0, 1);
    
    // out-postP
    const char* out_post_p_config_topic = "homeassistant/sensor/genvex/out-postP/config";
    const char* out_post_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"out-post Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/out-post/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-out-post-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(out_post_p_config_topic, out_post_p_config_payload, 0, 1);
    
    return 0;
}

/**
 * Register rotor fan component
 */
static int register_rotor_fan(void) {
    //-----------------------------
    // Rotor fan
    //-----------------------------
    
    static const char* rotor_fan_config_topic = "homeassistant/fan/genvex/rotor/config";
    const char* rotor_fan_config_payload = "{\"payload_on\": 1, \"payload_off\": 0, \"payload_low_speed\": 1, \"payload_medium_speed\": 2, \"payload_high_speed\": 3, \"name\": \"Genvex Rotor\",\"state_topic\": \"homeassistant/fan/genvex/rotor/on/state\",\"command_topic\": \"homeassistant/fan/genvex/rotor/on/set\",\"speed_state_topic\": \"homeassistant/fan/genvex/rotor/speed/state\",\"speed_command_topic\": \"homeassistant/fan/genvex/rotor/speed/set\",\"unique_id\": \"genvex-rotor-fan\",\"speeds\":[\"off\", \"low\", \"medium\",\"high\"],\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_fan_config_topic, rotor_fan_config_payload, 0, 1);

    return 0;
}

/**
 * Register rotor sensors
 */
static int register_rotor_sensors(void) {

    //-----------------------------
    // Rotor sensors
    //-----------------------------
    // rotorT
    const char* rotor_t_config_topic = "homeassistant/sensor/genvex/rotorT/config";
    const char* rotor_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"Rotor Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-rotor-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_t_config_topic, rotor_t_config_payload, 0, 1);
    
    // rotorH
    const char* rotor_h_config_topic = "homeassistant/sensor/genvex/rotorH/config";
    const char* rotor_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"Rotor Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-rotor-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_h_config_topic, rotor_h_config_payload, 0, 1);
    
    // rotorP
    const char* rotor_p_config_topic = "homeassistant/sensor/genvex/rotorP/config";
    const char* rotor_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"Rotor Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-rotor-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_p_config_topic, rotor_p_config_payload, 0, 1);

    // rotorMT
    const char* rotor_mt_config_topic = "homeassistant/sensor/genvex/rotorMT/config";
    const char* rotor_mt_config_payload = "{\"device_class\": \"temperature\",\"name\": \"Rotor Motor Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.mt}}\",\"unique_id\": \"genvex-rotor-mt\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_mt_config_topic, rotor_mt_config_payload, 0, 1);
    
    // rotor rpm
    const char* rotor_rpm_config_topic = "homeassistant/sensor/genvex/rotorRPM/config";
    const char* rotor_rpm_config_payload = "{\"name\": \"Rotor RPM\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"RPM\", \"value_template\": \"{{ value_json.rrpm}}\",\"unique_id\": \"genvex-rotor-rrpm\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_rpm_config_topic, rotor_rpm_config_payload, 0, 1);
    
     // average rotor rpm
    const char* rotor_arpm_config_topic = "homeassistant/sensor/genvex/rotorARPM/config";
    const char* rotor_arpm_config_payload = "{\"name\": \"Rotor RPM(avg)\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"unit_of_measurement\": \"RPM\", \"value_template\": \"{{ value_json.arpm}}\",\"unique_id\": \"genvex-rotor-arpm\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_arpm_config_topic, rotor_arpm_config_payload, 0, 1);
    
    // rotor state
    const char* rotor_speed_config_topic = "homeassistant/sensor/genvex/rotorS/config";
    const char* rotor_speed_config_payload = "{\"name\": \"Rotor Speed\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"value_template\": \"{{ value_json.s}}\",\"unique_id\": \"genvex-rotor-speed\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_speed_config_topic, rotor_speed_config_payload, 0, 1);
    
     // rotor fault
    const char* rotor_rotor_fault_config_topic = "homeassistant/sensor/genvex/rotorRF/config";
    const char* rotor_rotor_fault_config_payload = "{\"name\": \"Rotor Fault\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"value_template\": \"{{ value_json.rf}}\",\"unique_id\": \"genvex-rotor-rotor-fault\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_rotor_fault_config_topic, rotor_rotor_fault_config_payload, 0, 1);
    
     // temperature fault
    const char* rotor_temp_fault_config_topic = "homeassistant/sensor/genvex/rotorTF/config";
    const char* rotor_temp_fault_config_payload = "{\"name\": \"Rotor Temperature fault\",\"state_topic\": \"homeassistant/sensor/genvex/rotor/state\", \"value_template\": \"{{ value_json.tf}}\",\"unique_id\": \"genvex-rotor-temp-fault\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(rotor_temp_fault_config_topic, rotor_temp_fault_config_payload, 0, 1);

    return 0;
}
/**
 * Register genvex
 */

static int register_genvex(void) {
    //-----------------------------
    // Genvex
    //-----------------------------
     // Speed {0-3}
    const char* speed_config_topic = "homeassistant/sensor/genvex/speed/config";
    const char* speed_config_payload = "{\"name\": \"Genvex speed\",\"state_topic\": \"homeassistant/sensor/genvex/state\", \"value_template\": \"{{ value_json.s}}\",\"unique_id\": \"genvex-speed\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(speed_config_topic, speed_config_payload, 0, 1);

    //-----------------------------
    // Genvex BME280 ambient
    //-----------------------------
    // out-postT
    const char* ambi_t_config_topic = "homeassistant/sensor/genvex/ambiT/config";
    const char* ambi_t_config_payload = "{\"device_class\": \"temperature\",\"name\": \"Ambient Temperature\",\"state_topic\": \"homeassistant/sensor/genvex/ambi/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\",\"unique_id\": \"genvex-ambi-t\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(ambi_t_config_topic, ambi_t_config_payload, 0, 1);
    
    // out-postH
    const char* ambi_h_config_topic = "homeassistant/sensor/genvex/ambiH/config";
    const char* ambi_h_config_payload = "{\"device_class\": \"humidity\",\"name\": \"Ambient Humidity\",\"state_topic\": \"homeassistant/sensor/genvex/ambi/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\",\"unique_id\": \"genvex-ambi-h\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(ambi_h_config_topic, ambi_h_config_payload, 0, 1);
    
    // out-postP
    const char* ambi_p_config_topic = "homeassistant/sensor/genvex/ambiP/config";
    const char* ambi_p_config_payload = "{\"device_class\": \"pressure\",\"name\": \"Ambient Pressure\",\"state_topic\": \"homeassistant/sensor/genvex/ambi/state\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\",\"unique_id\": \"genvex-ambi-p\",\"device\": {\"name\": \"Genvex Controller\", \"identifiers\": [\"genvexcontrolleresp32\"],\"model\": \"Genvex Controller MK1\",\"manufacturer\": \"BJ Inc.\"}}";
    mqttw_publish(ambi_p_config_topic, ambi_p_config_payload, 0, 1);
    
    return 0;    
}