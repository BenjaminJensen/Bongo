#include <stdint.h>
#include "com.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SLAVE_ID 10

/************************************
* UART
************************************/

// Note: UART2 default pins IO16, IO17 do not work on ESP32-WROVER module 
// because these pins connected to PSRAM
#define TXD_PIN   (23)
#define RXD_PIN   (22)

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

/************************************
* Data
************************************/
static struct  {
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
} status_data;

SemaphoreHandle_t sem_tx_data;

static void com_eval_cmd(uint8_t cmd, uint8_t data);
static void com_task();
static void send_status();
static void com_parse_com(uint8_t c);

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
    sem_tx_data = xSemaphoreCreateMutex();

    if( sem_tx_data == NULL ) {
        ESP_LOGE(TAG, "Unable to create UART Mutex!");
    }
    
    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIO, NULL);
}

void com_set_bme280_data(int32_t temp, uint32_t pres, uint32_t humi) {
    if( sem_tx_data != NULL ) {
        if( xSemaphoreTake( sem_tx_data, ( TickType_t ) 10 ) == pdTRUE ) {
            status_data.temp = temp;
            status_data.pres = pres;
            status_data.humi = humi;
            xSemaphoreGive( sem_tx_data );
        }
        else {
            ESP_LOGW(TAG, "Unable to take data Mutex in \"com_set_bme280_data\"");
        }
    }
}

void com_set_ds18b20_data(int16_t temp) {
    if( sem_tx_data != NULL ) {
        if( xSemaphoreTake( sem_tx_data, ( TickType_t ) 10 ) == pdTRUE ) {
            status_data.mtemp = temp;
            xSemaphoreGive(sem_tx_data);
        }
        else {
            ESP_LOGW(TAG, "Unable to take data Mutex in \"com_set_ds18b20_data\"");
        }
    }
}

void com_task() {
	uint8_t data[BUF_SIZE];
    int i;
    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    while(1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, PACKET_READ_TICS);
        if(len <= 0) {
            vTaskDelay( 10 / portTICK_PERIOD_MS);
        }
        else {
            ESP_LOGI(TAG, "UART data received(%d).\r\n", len);
            for(i = 0; i < len; i++) {
                com_parse_com(data[i]);
            }
        }
    }
}

/*
* Parse RS485 communication
*
* Protocol format:
* {0xAA, 0x00, 0xFF, slave_id(4) + command(4), data(8) 
*
*/

static void com_parse_com(uint8_t c) {
	static uint8_t state = 0;
	static uint8_t cmd = 0xFF;
	static uint8_t slave_id = 0xFF;

    switch(state) {
        case 0:
            if(c == 0xAA)
                state = 1;
            break;
        case 1:
            if(c == 0x00)
                state = 2;
            else
                state = 0;
            break;
        case 2:
            if(c == 0xFF)
                state = 3;
            else
                state = 0;
            break;
        case 3:
            // Getting slave_id and cmd
            cmd = c & 0x0F;
            slave_id = (c >> 4) & 0x0F;
            state = 4;
            break;
        case 4:
            // Getting cmd data
            if(slave_id == SLAVE_ID) {
                com_eval_cmd(cmd, c);
            }
            state = 0;
            break;
        default:
            ESP_LOGW(TAG, "Wrong state(%d) in \"com_parse_com\"!", state);
            state = 0;
            break;
    }
}

static void com_eval_cmd(uint8_t cmd, uint8_t data) {
	switch(cmd) {
		case 0:
			send_status();
			break;
		default:
            ESP_LOGW(TAG, "Unknown command(%d) in \"com_eval_cmd\"!", cmd);
			break;
	}
}

void send_status() {
	uint8_t id; // 0
	uint8_t speed; // 15
	uint8_t setpoint; // 16
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
	uint8_t i = 0; // Transmission buffer index
	uint8_t k;
	uint8_t tmp;
	char buf[21];
    
    temp = 0;
    pres = 0;
    humi = 0;
    mtemp = 0;

	if( sem_tx_data != NULL ) {
        if( xSemaphoreTake( sem_tx_data, ( TickType_t ) 10 ) == pdTRUE ) {    
            temp = status_data.temp;
            pres = status_data.pres;
            humi = status_data.humi;
            mtemp = status_data.mtemp;
            xSemaphoreGive(sem_tx_data);
        }
        else {
            ESP_LOGW(TAG, "Unable to take data Mutex in \"send_status\"");
        }
    }
	
	// Test data
	id = 10;
	speed = 80; // 80%
	setpoint = 90; // 90%
	
	buf[0] = 0xAA;
	buf[1] = 0x00;
	buf[2] = 0xFF;
	
	buf[3] = id;
	i = 4;
	
	// send 4 bytes BME280-temp
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((temp >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	// send 4 bytes BME280-pres
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((pres >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	// send 4 bytes BME280-humi
	for(k = 0; k < 4; k++) {
		tmp = (uint8_t) ((humi >> i*8)& 0xFF);
		buf[i++] = tmp;
	}
	
	// Motor temperature
	tmp = (uint8_t)(mtemp & 0xFF);
	buf[i++] = tmp;
	tmp = (uint8_t)((mtemp >> 8) & 0xFF);
	buf[i++] = tmp;
	
	// Current speed
	buf[i++] = speed;
	
	// Current setpoint
	buf[i++] = setpoint;
	
    uart_write_bytes(UART_PORT, buf, 19);
}
