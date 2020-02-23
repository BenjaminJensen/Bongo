#include "bme280_wrapper.h"
#include <stdint.h>
#include <string.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "com.h"

/************************************
* SPI
************************************/
#define BME280_HOST    HSPI_HOST

#define PIN_NUM_MISO 27
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

static spi_bus_config_t buscfg;
static spi_device_interface_config_t devcfg;

/************************************
* BME280
************************************/
#define BME280_TASK_STACK_SIZE 4096
#define BME280_TASK_PRIO 9
static const char *TAG = "BME280";

struct bme280_dev dev;
struct bme280_data comp_data;

/************************************
* SPI
************************************/
static spi_device_handle_t spi;

// Forward declarations
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);
void setup_bme280_spi();

/************************************
* BME280 Public
************************************/

int8_t setup_bme280() {
	
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;
	
	setup_bme280_spi();

	dev.intf = BME280_SPI_INTF;
	dev.read = user_spi_read;
	dev.write = user_spi_write;
	dev.delay_ms = user_delay_ms;
	
	rslt = bme280_init(&dev);
	ESP_LOGI(TAG, "BME280 chip id: %d", dev.chip_id);
	// TODO: Handle BME280 config error
	if(rslt != 0) {
		ESP_LOGW(TAG, "Unable to initialize\r\n");
	}
	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	
	// TODO: Start BME280 task
	if(rslt == 0) {
		xTaskCreate(bme280_task, "com_task", BME280_TASK_STACK_SIZE, NULL, BME280_TASK_PRIO, NULL);
	}
	else {
		ESP_LOGW(TAG, "Unable to configure(%d)", rslt);
	}

	return rslt;
}

void bme280_task() {
	static uint8_t state = 0;
	int8_t rslt = BME280_OK;
	
	// Setup FreeRTOS task

	while(1) {
		switch(state) {
			case 0:
				// 840 uS
				rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
				state++;
				break;
			case 1:// 2.4 mS
				// Wait for conversion
				vTaskDelay( 100 / portTICK_PERIOD_MS);
				rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
				com_set_bme280_data(comp_data.temperature, comp_data.pressure, comp_data.humidity);
				float temp = comp_data.temperature / 100.0f;
            	float pres = comp_data.pressure / 1000.0f;
            	float humi = comp_data.humidity / 1024.0f;

				ESP_LOGV(TAG,"BME280: t: %f, p: %f, h: %f",temp,pres , humi);
				// 840 uS
				rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
				break;	
			default:
				state = 0;
		}
		if(rslt != BME280_OK) {
			ESP_LOGI(TAG,"task error: state(%d), BME280(%d) \"bme280_task\"", state, rslt);
		}
	}
}
/*
*
* Internal functions
*
*/


int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	esp_err_t ret; // Return 0 for Success, non-zero for failure
    spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = reg_addr & 0x7F;
    t.length = len*8;
    t.tx_buffer = reg_data;

	//ESP_LOGI(TAG, "Start SPI Write: addr(%d) len(%d) data(%d)\r\n", reg_addr, len, *reg_data);
	ret = spi_device_polling_transmit(spi, &t);  //Transmit
	//ESP_LOGI(TAG, "End SPI Write [%d]", *reg_data);
	vTaskDelay( 10 / portTICK_PERIOD_MS);
	if(ret!=ESP_OK) {
		ESP_LOGW(TAG, "SPI Write failed: addr(%d) len(%d)\r\n", reg_addr, len);	
	}

	return ret;
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	esp_err_t ret; // Return 0 for Success, non-zero for failure
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = reg_addr;
    t.rxlength = len*8;
    t.rx_buffer = reg_data;

    //ESP_LOGI(TAG, "Start SPI Read: addr(%d) len(%d)\r\n", reg_addr, len);
	ret = spi_device_polling_transmit(spi, &t);  //Transmit
	//ESP_LOGI(TAG, "End SPI Read [%d]", *reg_data);
	if(ret!=ESP_OK) {
		ESP_LOGW(TAG, "SPI Read failed: addr(%d) len(%d)\r\n", reg_addr, len);
	}

	return ret;
}

/*
* WARNING: This function assumes it is only called with the arguments {1,2}
*/
void user_delay_ms(uint32_t period) {
	vTaskDelay( period / portTICK_PERIOD_MS);
}

void setup_bme280_spi() {

	esp_err_t ret;
	buscfg.miso_io_num = PIN_NUM_MISO;
	buscfg.mosi_io_num = PIN_NUM_MOSI;
	buscfg.sclk_io_num = PIN_NUM_CLK;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 64;
    
    devcfg.clock_speed_hz = 1000*1000;//Clock out at 1 MHz
	devcfg.mode = 3;					//SPI mode 0
	devcfg.spics_io_num = PIN_NUM_CS;	//CS pin
	devcfg.queue_size = 1;				//We want to be able to queue 7 transactions at a time
	devcfg.address_bits = 8;
	devcfg.command_bits = 0;
	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	//devcfg.input_delay_ns = 50;

    //Initialize the SPI bus
    ret=spi_bus_initialize(BME280_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    //Attach the BME280 to the SPI bus
    ret=spi_bus_add_device(BME280_HOST, &devcfg, &spi);

    ESP_ERROR_CHECK(ret);
	if(ret == ESP_OK) {
		ESP_LOGI(TAG,"BME280 SPI setup ok.");
	}
}
