#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mqtt.h"

/************************************
* Defines
************************************/

#define INPUT_SPEED_LED0 	GPIO_NUM_15
#define INPUT_SPEED_LED1	GPIO_NUM_4
#define INPUT_SPEED_LED2	GPIO_NUM_0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INPUT_SPEED_LED0) | (1ULL<<INPUT_SPEED_LED1) | (1ULL<<INPUT_SPEED_LED2))

#define OUTPUT_SPEED		GPIO_NUM_22
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<OUTPUT_SPEED) 

static const char *TAG = "CTRL"; 
static uint8_t set_speed = 0;
SemaphoreHandle_t sem_set_speed;

/************************************
* Forward declarations
************************************/
static uint8_t get_speed(void);
static void control_task(void* data);
static uint8_t ctrl_set_speed(uint8_t s);

/************************************
* Public functions
************************************/
void control_init(void) {

    // Setup IO
    gpio_config_t io_conf;

	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	// Setup INPUT
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	// Setup OUTPUT
	gpio_set_direction(OUTPUT_SPEED, GPIO_MODE_OUTPUT);

    // Create mutex for shared data
    sem_set_speed = xSemaphoreCreateMutex();

    // Creat task
    xTaskCreate(control_task, "control_task", 4096, NULL, 12, NULL);
}

void control_update_speed(uint8_t s) {
    if(s >= 0 && s <= 3) {
        if( sem_set_speed != NULL ) {
            if( xSemaphoreTake( sem_set_speed, ( TickType_t ) 10 ) == pdTRUE ) {
                set_speed = s;
                xSemaphoreGive( sem_set_speed );
                ESP_LOGI(TAG, "Genvex Speed updated: %d", s);
            }
            else {
                ESP_LOGW(TAG, "Unable to take Mutex in \"update_speed\"");
            }
        }
        else {
            ESP_LOGW(TAG, "Mutex is NULL \"update_speed\"");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Genvex Speed out of bound: %d", s);
    }
    
}
/************************************
* Private functions
************************************/
static void control_task(void* data) {
    TickType_t xLastWakeTime;
    uint8_t last_speed = 0;
    uint8_t speed;
    uint8_t state = 0;
    uint8_t cur_speed = 0;

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        
        switch(state) {
            case 0:
                speed = get_speed();
                
                if(speed != cur_speed) {
                    last_speed = speed;
                    state = 1;
                } else if(speed != set_speed) {
                    last_speed = speed;
                    state = 2;
                }
                break;
            case 1:
                // update speed based on manual intervention
                if(speed == last_speed) {
                    // update speed
                    cur_speed = speed;
                    if( sem_set_speed != NULL ) {
                        if( xSemaphoreTake( sem_set_speed, ( TickType_t ) 10 ) == pdTRUE ) {
                            set_speed = speed;
                            xSemaphoreGive( sem_set_speed );
                        }
                        else {
                            ESP_LOGW(TAG, "Unable to take Mutex in \"control_task\" case: %d", state);
                        }
                    }
                    else {
                        ESP_LOGW(TAG, "Mutex is NULL \"control_task\"");
                    }
                    // Do speed update
                    
                    ESP_LOGI(TAG, "New speed found: %d", speed);
                    // MQTT
                    control_status_t s;
                    s.speed = cur_speed;
                    mqtt_update_control_status(&s);
                }
                state = 0;
                break;
            case 2:
                // Setpoint have changed remote
                ESP_LOGI(TAG, "Update speed to: %d", set_speed);
                ctrl_set_speed(set_speed);
                if( sem_set_speed != NULL ) {
                    if( xSemaphoreTake( sem_set_speed, ( TickType_t ) 10 ) == pdTRUE ) {
                        set_speed = get_speed(); // Reset set value in case of error
                        xSemaphoreGive( sem_set_speed );
                    }
                    else {
                        ESP_LOGW(TAG, "Unable to take Mutex in \"control_task\" case: %d", state);
                    }
                }
                else {
                    ESP_LOGW(TAG, "Mutex is NULL \"control_task\"");
                }
                state = 0;
                break;
            default:
                state = 0;
                break;
        }
        // Wait for the next cycle. 50ms
        vTaskDelayUntil( &xLastWakeTime, 50 / portTICK_PERIOD_MS );
    }
}

static uint8_t get_speed() {
	uint8_t speed = 0;

	if(gpio_get_level(INPUT_SPEED_LED0) == 0) {
		speed = 1;
	}

	if(gpio_get_level(INPUT_SPEED_LED1) == 0) {
		speed = 2;
	}

	if(gpio_get_level(INPUT_SPEED_LED2) == 0) {
		speed = 3;
	}

	return speed;
}

static uint8_t ctrl_set_speed(uint8_t s) {
    uint8_t ret = 0;
    uint8_t retry = 0;

    while(get_speed() != s && retry < 8) {
        gpio_set_level(OUTPUT_SPEED, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(OUTPUT_SPEED, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    return ret;
}