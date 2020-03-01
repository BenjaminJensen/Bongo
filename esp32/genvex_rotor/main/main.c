/* 
* Lindab LVAR 75 Rotor controller
*/

#include "bme280_wrapper.h"
#include "com.h"
#include "esp_log.h"
#include "motor.h"

static const char *TAG = "ROTOR_CTRL";

/*
* Application setup
*/
void app_main()
{
    setup_bme280();
    com_init();
    motor_init();
    ESP_LOGI(TAG, "LVAR 75 Rotor Controller online");
}
