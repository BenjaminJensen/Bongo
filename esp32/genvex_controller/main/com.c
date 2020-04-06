#include <stdint.h>
#include "com.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280_wrapper.h"
#include "sensor_helper.h"
#include "mqtt.h"

#define SLAVE_ID 10

/************************************
* UART
************************************/

// Note: UART2 default pins IO16, IO17 do not work on ESP32-WROVER module 
// because these pins connected to PSRAM
#define TXD_PIN   (17)
#define RXD_PIN   (16)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN   (18)

// CTS is not used in RS485 Half-Duplex Mode
#define CTS_PIN  UART_PIN_NO_CHANGE

#define BUF_SIZE        (127)
#define BAUD_RATE       (56000)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define COM_TASK_STACK_SIZE    (8192)
#define COM_TASK_PRIO          (10)
#define UART_PORT          (UART_NUM_2)

static const char *TAG = "UART"; 
SemaphoreHandle_t sem_rs485;

/************************************
* Data
************************************/

static void com_task();
static void get_rotor_state(void);
static void parse_rotor_state(uint8_t *);

/************************************
* Function definitions
************************************/
void com_init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);
    
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: IO19)
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN);

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Set RS485 half duplex mode
    uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    
    // Create mutex for shared data
    sem_rs485 = xSemaphoreCreateMutex();

    // Start com task
    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIO, NULL);
}

void com_set_rotor_speed(uint8_t speed) {
    if( sem_rs485 != NULL ) {
		if( xSemaphoreTake( sem_rs485, ( TickType_t ) 10 ) == pdTRUE ) {
            // Rotor set speed command
            char buf[5] = {0xAA, 0x00, 0xFF, 0xA1, speed};
            uart_write_bytes(UART_PORT, buf, 5);
            xSemaphoreGive( sem_rs485 );
		}
		else {
			ESP_LOGW(TAG, "Unable to take Mutex in \"get_rotor_state\"");
		}
	}
	else {
		ESP_LOGW(TAG, "Mutex is NULL \"get_rotor_state\"");
	}
}

void com_task() {
    TickType_t xLastWakeTime;

    ESP_LOGI(TAG, "COM task start.");

    xLastWakeTime = xTaskGetTickCount();
    while(1) {
        // Get and publish rotor state
        get_rotor_state();
        
        // Get and publish in-pre state
        //TODO
        
        // Get and publish in-pre state
        //TODO

        // Get and publish in-pre state
        //TODO

        // Get and publish in-pre state
        //TODO

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
    }
}

/*
* Parse RS485 communication
*
* Protocol format:
* {0xAA, 0x00, 0xFF, slave_id(4) + command(4), data(8) 
*
*/

static void get_rotor_state() {
    char buf[5] = {0xAA, 0x00, 0xFF, 0xA0, 0x00};
    uint8_t data[20];
    int len = 0;

    if( sem_rs485 != NULL ) {
		if( xSemaphoreTake( sem_rs485, ( TickType_t ) 10 ) == pdTRUE ) {
            //\xaa\x00\xff\xa0\x00
            // Rotor get status command
            uart_write_bytes(UART_PORT, buf, 5);

			 // read responce from rotor, within 50ms
            len = uart_read_bytes(UART_PORT, data, 23, 50/portTICK_RATE_MS);
			xSemaphoreGive(sem_rs485);
		}
		else {
			ESP_LOGW(TAG, "Unable to take Mutex in \"get_rotor_state\"");
		}
	}
	else {
		ESP_LOGW(TAG, "Mutex is NULL \"get_rotor_state\"");
	}

    if(len == 23) { // all bytes read
        //ESP_LOGI(TAG, "Data from rotor successfully read");

        if(data[0] == 0xAA && data[1] == 0x00 && data[2] == 0xFF && data[3] == 10) {
            parse_rotor_state(&data[4]);
        }
        else {
            ESP_LOGW(TAG, "Rotor Status: Wrong header %x %x %x %x", data[0], data[1], data[2], data[3]);
        }
    }
    else {
        ESP_LOGW(TAG, "Data read from Rotor failed. %d of 20 bytes read", len);
    }
}

/**
 * Parses the raw byte stream crom the RS485 rotor responce
 */
static void parse_rotor_state(uint8_t *data) {
    int32_t temp = 0;
    uint32_t pres = 0;
    uint32_t humi = 0;
    int16_t mtemp = 0;
    rotor_status_t status;

    //-------------------------
    // BME280 values
    //-------------------------

    // De-serialize temperature
    temp = data[0];
    temp += data[1] << 8;
    temp += data[2] << 16;
    temp += data[3] << 24;

    // De-serialize pressure
    pres = data[4];
    pres += data[5] << 8;
    pres += data[6] << 16;
    pres += data[7] << 24;

    // De-serialize humidity
    humi = data[8];
    humi += data[9] << 8;
    humi += data[10] << 16;
    humi += data[11] << 24;

    status.temp = convert_bme280_t_raw_to_float(temp);
    status.pres = convert_bme280_p_raw_to_float(pres);
    status.humi = convert_bme280_h_raw_to_float(humi);

    //-------------------------
    // Other values
    //-------------------------
    
    // De-serialize motor temperature
    mtemp = data[12];
    mtemp += data[13] << 8;
    status.motor_temp = convert_ds18b20(mtemp); 

    // De-serialize rotor rpm (rmp0)
    status.rotor_rpm = data[14];

    // De-serialize rotor rpm (rmp0)
    status.rotor_rpm_avg = data[15];
    
    // De-serialize setpoint {0-3}    
    status.state = data[16];

    // Rotor fault
    status.rotor_fault = data[17];

    // Temperature fault
    status.temp_fault = data[18];

    mqtt_update_rotor_status(&status);
}
