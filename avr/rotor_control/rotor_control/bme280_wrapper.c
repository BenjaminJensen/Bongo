#include "include/bme280_wrapper.h"

#include <avr/io.h>
#include <util/delay.h>
#include "include/timer.h"
#include "include/bme280_wrapper.h"
#include "include/com.h"
#include <util/twi.h>

/************************************
* BME280
************************************/

#define BME_I2C

#define SS_HIGH (PORTB |= (1 << 2))
#define SS_LOW (PORTB &= ~(1 << 2))

static uint8_t timer_handle = 255;
struct bme280_dev dev;
struct bme280_data comp_data;

// Forward declarations
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

void user_delay_ms(uint32_t period);
void setup_bme280_spi();
void setup_bme_i2c();

/************************************
* BME280 Public
************************************/

int8_t setup_bme280() {
	
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;
	
	#ifdef BME_I2C
		setup_bme_i2c();
		dev.dev_id = BME280_I2C_ADDR_PRIM;
		dev.intf = BME280_I2C_INTF;
		dev.read = user_i2c_read;
		dev.write = user_i2c_write;
		dev.delay_ms = user_delay_ms;
	
	#else
		
		setup_bme280_spi();
		dev.dev_id = 0;	
		dev.intf = BME280_SPI_INTF;
		dev.read = user_spi_read;
		dev.write = user_spi_write;
		dev.delay_ms = user_delay_ms;
	#endif
	
	rslt = bme280_init(&dev);
	
	if(rslt != 0) {
		while(1) {
			//uart_putchar(rslt);
			_delay_ms(500 / 5);
		}
	}
	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	
	if(rslt == 0) {
		timer_handle = timer_register(50);
	}
	
	return rslt;
}

void bme280_task() {
	static uint8_t state = 0;
	int8_t rslt = BME280_OK;
	
	if(timer_ovf(timer_handle) != 0) {
		switch(state) {
			case 0:
				// 840 uS
				rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
				state++;
				break;
			case 1:// 2.4 mS
				rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
				com_set_bme280_data(comp_data.temperature, comp_data.pressure, comp_data.humidity);
				// 840 uS
				rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
				break;	
			default:
				state = 0;
		}
	}
}
/*
*
* Internal functions
*
*/


/*
* WARNING: This function assumes it is only called with the arguments {1,2}
*/
void user_delay_ms(uint32_t period) {
	
	if(period)
	_delay_ms(1);
	else
	_delay_ms(2);
}

#ifdef BME_I2C
void setup_bme_i2c() {
	
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

	TWIStart();
	if (TWIGetStatus() != 0x08)
		return -1;
	//select devise and send A2 A1 A0 address bits
	TWIWrite(reg_addr);
	if (TWIGetStatus() != 0x18)
		return -1;
	
	TWIStop();
	
    return rslt;
}
#else
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	/* Start transmission */
	
	uint16_t i;
	
	SS_LOW;
	// Start by writing register address
	SPDR = reg_addr & 0x7F;
	
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF))) ;

	for(i = 0; i < len; i++) {
		SPDR = reg_data[i];
		/* Wait for transmission complete */
		while(!(SPSR & (1<<SPIF))) ;
	}
	SS_HIGH;
	return rslt;
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	uint16_t i;
	
	SS_LOW;
	// Start by writing register address
	SPDR = reg_addr;
	
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF))) ;

	for(i = 0; i < len; i++) {
		/* Wait for transmission complete */
		SPDR = 0x00;
		while(!(SPSR & (1<<SPIF))) ;
		reg_data[i] = SPDR;
	}
	SS_HIGH;
	return rslt;
}

void setup_bme280_spi() {
	// Set MOSI, SCK as Output
	DDRB = (1<<5) | (1<<3) | (1<<2);
	PORTB |= (1 << 2);
	// Enable SPI, Set as Master Mode: 11
	//Prescaler: Fosc/16, Enable Interrupts
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPR1) | (1<<CPHA) | (1<<CPOL);
}

#endif