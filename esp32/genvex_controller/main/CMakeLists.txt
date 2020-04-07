set(COMPONENT_SRCS "tcp_logger.c" "mqtt_hass.c" "bme280_wrapper.c" "com.c" "app_main.c" "mqtt.c" "mqtt_wrapper.c" "BME280_driver/bme280.c" "control.c" "sensor_helper.c" "tcp_logger.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "BME280_driver/")

register_component()
