#include <stdint.h>
#include "com.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280_wrapper.h"

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

/************************************
* Data
************************************/
static struct  {
	int32_t temp;
	uint32_t pres;
	uint32_t humi;
	int16_t mtemp;
    uint8_t rpm0;

} status_data;

static void com_eval_cmd(uint8_t cmd, uint8_t data);
static void com_task();
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
    
    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIO, NULL);
}

void com_set_bme280_data(int32_t temp, uint32_t pres, uint32_t humi) {
    status_data.temp = temp;
    status_data.pres = pres;
    status_data.humi = humi;
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
            ESP_LOGD(TAG, "UART data received(%d).\r\n", len);
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
            // Not implemented in the 
			//send_status();
			break;
        case 1:
            ESP_LOGI(TAG, "Change speed to (%d)", data);
            //motor_set_speed(data);
            break;
		default:
            ESP_LOGW(TAG, "Unknown command(%d) in \"com_eval_cmd\"!", cmd);
			break;
	}
}


