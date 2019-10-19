set(COMPONENT_SRCS "app_main.c"
                    "genvex_control.c"
                    "genvex_sensors.c"
                    "mqtt_wrapper.c"
                    "temperature_controller.c"

)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()