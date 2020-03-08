/* 
* Lindab LVAR 75 Rotor controller
* Motor control:
* - Spped
* - Temperature
* - Status
*/
#include "motor.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"

#include "driver/pcnt.h"

static const char *TAG = "MOTOR"; 

/************************************
* PWM
************************************/
#define SPEED_TASK_STACK_SIZE    (8192)
#define SPEED_TASK_PRIO          (7)

#define GPIO_PWM0A_OUT UART_PIN_NO_CHANGE   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B

#define SPEED_STEP_MS (100)
static uint8_t set_speed;
static uint8_t cur_speed = 0;

/************************************
* Pulse Counting
************************************/
#define PCNT_UNIT0           PCNT_UNIT_0
#define PCNT_UNIT1           PCNT_UNIT_1

#define PCNT_INPUT_SIG0_IO  2  // Pulse Input GPIO
#define PCNT_INPUT_SIG1_IO  4  // Pulse Input GPIO

static void setup_pcnt(void);
static void pcnt_task(void* data);
/************************************
* DS18B20
************************************/
#define GPIO_DS18B20_0       (5)
#define MAX_DEVICES          (1)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_9_BIT)
#define SAMPLE_PERIOD        (100)   // milliseconds

#define DS18B20_TASK_STACK_SIZE    (8192)
#define DS18B20_TASK_PRIO          (10)

static const OneWireBus_ROMCode rom_code = {.bytes = {0x46,0x04,0x17,0x63,0x02,0x58,0xff,0x28}};
OneWireBus * owb;
DS18B20_Info device;
owb_rmt_driver_info rmt_driver_info;

static uint16_t cur_motor_temp = 0;
SemaphoreHandle_t sem_motor_data;

static void setup_ds18b20(void);
static void ds18b20_read_temp_task(void* );
static void setup_pwm(void);
static void speed_task(void*);
static void mcpwm_example_gpio_initialize(void);
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);

/************************************
* Public Funtions
************************************/

void motor_init() {
    esp_log_level_set("*", ESP_LOG_INFO);

    // Create mutex for shared data
    sem_motor_data = xSemaphoreCreateMutex();

    if( sem_motor_data == NULL ) {
        ESP_LOGE(TAG, "Unable to create motor data Mutex!");
    }

    // Set up pulse counters for speed measurement
    setup_pcnt();

    // Setup DS18B20
    setup_ds18b20();

    // Setup PWM
    setup_pwm();
}

void motor_get_data(motor_data_t* data) {
     if( xSemaphoreTake( sem_motor_data, ( TickType_t ) 10 ) == pdTRUE ) {
        data->temperature = cur_motor_temp;
        data->set_point = set_speed;
        data->speed = cur_speed;
        xSemaphoreGive(sem_motor_data);
    }
    else {
        ESP_LOGW(TAG, "Unable to take motor data Mutex in \"ds18b20_read_temp_task\"");
    }
}
uint16_t motor_get_temp(void) {
    uint16_t temp = 0xffff;
    if( xSemaphoreTake( sem_motor_data, ( TickType_t ) 10 ) == pdTRUE ) {
        temp = cur_motor_temp;
        xSemaphoreGive(sem_motor_data);
    }
    else {
        ESP_LOGW(TAG, "Unable to take motor data Mutex in \"ds18b20_read_temp_task\"");
    }
    return temp;
}

typedef struct  {
    uint8_t step;
    uint8_t max;
} speed_step_t;

speed_step_t speed_curve[4] = {
    {.step = 10, .max = 0},
    {.step = 10, .max = 50},
    {.step = 10, .max = 70},
    {.step = 10, .max = 100}
};

void speed_task(void* p) {
    uint8_t cur_step = 0;

    while(1) {
        if(set_speed > cur_speed){
            ESP_LOGI(TAG, "Ramp-up current: %d to %d", cur_speed, set_speed);
            ESP_LOGI(TAG, "Step current: %d, max: %d", cur_step, speed_curve[cur_speed+1].max);
            if(cur_step < speed_curve[cur_speed + 1].max) {
                
                // Increment step
                cur_step += speed_curve[cur_speed].step;
                
                // set pwm
                brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, cur_step);
                
                // Clamp
                if(cur_step >= speed_curve[cur_speed+1].max) {
                    cur_step = speed_curve[cur_speed+1].max;
                    cur_speed++;
                }
                ESP_LOGI(TAG, "New step : %d, max: %d. Speed: %d", cur_step, speed_curve[cur_speed].max, cur_speed);
            }
        }
        else if( set_speed < cur_speed) {
            ESP_LOGI(TAG, "Ramp-down current: %d to %d", cur_speed, set_speed);
            ESP_LOGI(TAG, "Step current: %d, max: %d", cur_step, speed_curve[cur_speed-1].max);
            if(cur_step > speed_curve[cur_speed - 1].max) {
                // Increment step
                cur_step -= speed_curve[cur_speed].step;
                
                // set pwm
                brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, cur_step);

                // Clamp
                if(cur_step <= speed_curve[cur_speed - 1].max) {
                    cur_step = speed_curve[cur_speed - 1].max;
                    if(cur_speed > 0)
                        cur_speed--;
                }
                ESP_LOGI(TAG, "New step : %d, max: %d. Speed: %d", cur_step, speed_curve[cur_speed].max, cur_speed);
            }
        }
        vTaskDelay(SPEED_STEP_MS / portTICK_PERIOD_MS);
    } // while
}
void motor_set_speed(uint8_t speed) {
    set_speed = speed;
}

/************************************
* Private Funtions - Speed monitoring
************************************/

static void setup_pcnt(void) {
    // Prepare configuration for the PCNT unit
    pcnt_config_t pcnt_config0 = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG0_IO,
        .ctrl_gpio_num = -1,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT0,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 0,
        .counter_l_lim = 0,
    };
    // Initialize PCNT unit 
    pcnt_unit_config(&pcnt_config0);

    // Initialize PCNT's counter
    pcnt_counter_pause(PCNT_UNIT0);
    pcnt_counter_clear(PCNT_UNIT0);

    // Everything is set up, now go to counting
    pcnt_counter_resume(PCNT_UNIT0);

    // Prepare configuration for the PCNT unit
    pcnt_config_t pcnt_config1 = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG1_IO,
        .ctrl_gpio_num = -1,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT1,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 0,
        .counter_l_lim = 0,
    };
    // Initialize PCNT unit 
    pcnt_unit_config(&pcnt_config1);

    // Initialize PCNT's counter
    pcnt_counter_pause(PCNT_UNIT1);
    pcnt_counter_clear(PCNT_UNIT1);

    // Everything is set up, now go to counting
    pcnt_counter_resume(PCNT_UNIT1);
    xTaskCreate(pcnt_task, "pcnt_task", SPEED_TASK_STACK_SIZE, NULL, SPEED_TASK_PRIO, NULL);
}

static void pcnt_task(void* data) {
    int16_t count = 0;
    float rpm;
    float sampling_period_ms = 5000.0f;

    float filter0[12] = {0};
    int filter_index0 = 0;
    
    float filter1[12] = {0};
    int filter_index1 = 0;

    while(1) {
        // Unit 0
        pcnt_get_counter_value(PCNT_UNIT0, &count);
        pcnt_counter_clear(PCNT_UNIT0);
        float tmp = count;
        tmp = tmp / 4.0f;
        rpm = (float)(60.0f*1000.0f/(float)sampling_period_ms) * tmp;

        filter0[filter_index0++] = rpm;
        if(filter_index0 >= 12)
            filter_index0 = 0;
        float avg_rpm = 0;
        for(int i = 0; i < 12; i++)
            avg_rpm += filter0[i];

        avg_rpm /= 12;

        printf("Speed0 %.2f RPM (%.2f avg), cnt: %d\n", rpm, avg_rpm, count);

        // Unit 1
        pcnt_get_counter_value(PCNT_UNIT1, &count);
        pcnt_counter_clear(PCNT_UNIT1);
        tmp = count;
        tmp = tmp / 4.0f;
        rpm = (float)(60.0f*1000.0f/(float)sampling_period_ms) * tmp;

        filter1[filter_index1++] = rpm;
        if(filter_index1 >= 12)
            filter_index1 = 0;
        avg_rpm = 0;
        for(int i = 0; i < 12; i++)
            avg_rpm += filter1[i];

        avg_rpm /= 12;

        printf("Speed1 %.2f RPM (%.2f avg), cnt: %d\n", rpm, avg_rpm, count);


        vTaskDelay(sampling_period_ms / portTICK_PERIOD_MS);
    }
}
/************************************
* Private Funtions - Motor Control
************************************/

static void setup_pwm() {
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    ESP_LOGI(TAG, "Configuring Initial Parameters of mcpwm...");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings


    xTaskCreate(speed_task, "motor_task", SPEED_TASK_STACK_SIZE, NULL, SPEED_TASK_PRIO, NULL);
    //brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
    //brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70.0);
}

static void mcpwm_example_gpio_initialize(void)
{
    ESP_LOGI(TAG, "initializing mcpwm gpio...");
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/************************************
* Private Funtions - DS18b20
************************************/

static void setup_ds18b20(void) {
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

     // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    ESP_LOGI(TAG,"Find devices:\n");
    
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s\n", num_devices, rom_code_s);
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(TAG, "Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

    // In this example, if a single device is present, then the ROM code is probably
    // not very interesting, so just print it out. If there are multiple devices,
    // then it may be useful to check that a specific device is present.
    if (num_devices == 1)
    {
        // For a single device only:
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(owb, &rom_code);
        if (status == OWB_STATUS_OK)
        {
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            ESP_LOGI(TAG, "Single device %s present\n", rom_code_s);
        }
        else
        {
            ESP_LOGW(TAG, "An error occurred reading ROM code: %d", status);
        }
    }
    else {
        ESP_LOGW(TAG, "Unable to locate DS18B20");
    }

    // TODO: Crap code, needs re-write
    // --------------------------------
    // Create DS18B20 devices on the 1-Wire bus
    
    ds18b20_init_solo(&device, owb);          // only one device on bus
    ds18b20_use_crc(&device, true);           // enable CRC check for temperature readings
    ds18b20_set_resolution(&device, DS18B20_RESOLUTION);

    // Create temperature reading task
    xTaskCreate(ds18b20_read_temp_task, "ds18b20_task", DS18B20_TASK_STACK_SIZE, NULL, DS18B20_TASK_PRIO, NULL);
}

static void ds18b20_read_temp_task(void* x) {
    // Read temperatures more efficiently by starting conversions on all devices at the same time
    int errors_count = 0;
    int sample_count = 0;
    
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "DS18B20 task started");
    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGV(TAG, "Convert device");
        ds18b20_convert(&device);

        ESP_LOGV(TAG, "Wait for conversion");
        // In this application all devices use the same resolution,
        // so use the first device to determine the delay
        ds18b20_wait_for_conversion(&device);

        // Read the results immediately after conversion otherwise it may fail
        // (using printf before reading may take too long)
        uint16_t reading = 0 ;
        DS18B20_ERROR error = 0 ;

        ESP_LOGV(TAG, "Read temperature");
        error = ds18b20_read_temp_raw(&device, &reading);
        
        // Print results in a separate loop, after all have been read
        ESP_LOGV(TAG, "\nTemperature readings (degrees C): sample %d\n", ++sample_count);
        
        if (error == DS18B20_OK) {
            if( sem_motor_data != NULL ) {
                if( xSemaphoreTake( sem_motor_data, ( TickType_t ) 10 ) == pdTRUE ) {
                    cur_motor_temp = reading;
                    xSemaphoreGive(sem_motor_data);
                }
                else {
                    ESP_LOGW(TAG, "Unable to take motor data Mutex in \"ds18b20_read_temp_task\"");
                }
            }
            else {
                ESP_LOGW(TAG, "Motor data Mutex is NULL \"ds18b20_read_temp_task\"");
            }
        }
        else {
            errors_count++;
        }

        ESP_LOGV(TAG, "Motor driver temp: %d    %d errors\n", reading, errors_count);

        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
    
}