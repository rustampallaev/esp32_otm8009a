#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "mcp23s17.h"


#define MCP_HOST        HSPI_HOST
#define DMA_CHAN        1

#define SPI_PIN_NUM_RST	 GPIO_NUM_5
#define SPI_PIN_NUM_MISO GPIO_NUM_12
#define SPI_PIN_NUM_MOSI GPIO_NUM_13
#define SPI_PIN_NUM_CLK  GPIO_NUM_14
#define SPI_PIN_NUM_CS   GPIO_NUM_15


spi_bus_config_t buscfg;
spi_bus_config_t buscfg_mcp23S17;
spi_device_interface_config_t devcfg_mcp23S17;
spi_device_handle_t handle_spi_mcp23S17;
spi_transaction_t trans_mcp23S17;


uint16_t _modeCache   = 0xFFFF; 
uint16_t _outputCache = 0x0000;
uint16_t _pullupCache = 0x0000;
uint16_t _invertCache = 0x0000;


void init_spi(void);

//********************************************************************
//	Function Name:  MCP23S17_Initalize(uint8_t address)
// 	Description:	
//
//	Example:		MCP23S17_Initalize(param 1)
//
//					Param #1) address of IC, I.E. A0,A1,A2 pins on Chip.
//
//	Returns: none		
//
//  Notes:	none
//********************************************************************
void MCP23S17_Initalize(uint8_t address)
{

    gpio_set_direction(SPI_PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_PIN_NUM_RST, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(SPI_PIN_NUM_RST, HIGH);


    init_spi();

	_modeCache   = 0xFFFF; 
	_outputCache = 0x0000;
	_pullupCache = 0x0000;
	_invertCache = 0x0000;
	
	 mcp23S17_WriteByte(address, IOCON, 0x08);    					// Set up ICON A,B to auto increment
}
//***************************************
// mcp23S17_WriteByte
//***************************************
void mcp23S17_WriteByte(uint8_t address, uint8_t reg, uint8_t value)
{
	uint8_t tx_data[3];

	tx_data[0] = MCP23S17_MANUF_CHIP_ADDRESS | (address << 1);
	tx_data[1] = reg;
	tx_data[2] = value;
	 
	trans_mcp23S17.tx_buffer = tx_data;	
	trans_mcp23S17.length = 24;
	
	ESP_ERROR_CHECK(spi_device_transmit(handle_spi_mcp23S17, &trans_mcp23S17)); 
	
}
//***************************************
// mcp23S17_WriteWord
//***************************************
void mcp23S17_WriteWord(uint8_t address, uint8_t reg, uint16_t data)
{
	uint8_t tx_data[4];
	
	tx_data[0] = MCP23S17_MANUF_CHIP_ADDRESS | (address << 1);
	tx_data[1] = reg;
	tx_data[2] = (uint8_t)(data);
	tx_data[3] = (uint8_t)(data >> 8);
	 
	trans_mcp23S17.tx_buffer = tx_data;	
	trans_mcp23S17.length = 32;

	ESP_ERROR_CHECK(spi_device_transmit(handle_spi_mcp23S17, &trans_mcp23S17)); 
}
/**********************************************************************
 *  mcp23S17_ReadWord(uint8_t address, uint8_t reg)
 *
 *	returns PORTB as HB, and PORTA as LB
 *	
 **********************************************************************/
uint16_t mcp23S17_ReadWord(uint8_t address, uint8_t reg)
{
	
	uint8_t tx_data[4];
	uint8_t rx_data[4];	
	
	tx_data[0] = MCP23S17_MANUF_CHIP_ADDRESS | (address << 1) | 0x01;
	tx_data[1] = reg;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	 
	trans_mcp23S17.tx_buffer = tx_data;	
	trans_mcp23S17.rx_buffer = rx_data;
	trans_mcp23S17.length = 32;
	
	ESP_ERROR_CHECK(spi_device_transmit(handle_spi_mcp23S17, &trans_mcp23S17)); 
	
	
	return ( (rx_data[3] << 8) | rx_data[2] );	
}
//*********************************************************************
//*******************  INITALIZE THE I/O PORT PINS  *******************
//*********************************************************************
void mcp23S17_GpioPinMode(uint8_t address, uint8_t pin, uint8_t mode)
{
	// Determine the mode before changing the bit state in the mode cache
	// Since input = "HIGH", OR in a 1 in the appropriate place
	if(mode == HIGH) 
	{
		_modeCache |= 1 << pin;                
	} 
	// If not, the mode must be output, so and in a 0 in the appropriate place
	else 
	{
		_modeCache &= ~(1 << pin);             
	}
	
	mcp23S17_WriteWord(address, IODIRA, _modeCache);
}
/**********************************************************************
 *  mcp23S17_GpioMode(uint8_t address, uint16_t mode)
 *
 *	PORTB(HB) | PORTA(LB)
 *	
 **********************************************************************/
void mcp23S17_GpioMode(uint8_t address, uint16_t mode)
{
	_modeCache = mode;
	
	mcp23S17_WriteWord(address, IODIRA, mode);	
}
//*********************************************************************
//*******************  INITALIZE THE I/O PORT PINS  *******************
//*********************************************************************
void mcp23S17_PullupPinMode(uint8_t address, uint8_t pin, uint8_t mode)
{
	// Determine the mode before changing the bit state in the mode cache
	// Since input = "HIGH", OR in a 1 in the appropriate place
	if(mode == ON) 
	{
		_pullupCache |= 1 << pin;                
	} 
	// If not, the mode must be output, so and in a 0 in the appropriate place
	else 
	{
		_pullupCache &= ~(1 << pin);             
	}
	
	mcp23S17_WriteWord(address, GPPUA, _pullupCache);
}
/**********************************************************************
 *
 *  mcp23S17_PullupMode(uint8_t address, uint16_t mode)
 *  
 *	PORTB(HB) | PORTA(LB)
 *	
 **********************************************************************/
void mcp23S17_PullupMode(uint8_t address, uint16_t mode)
{
	_pullupCache = mode;
	
	mcp23S17_WriteWord(address, GPPUA, mode);	
}
//*********************************************************************
// mcp23S17_GpioInvertPinMode
//*********************************************************************
void mcp23S17_GpioInvertPinMode(uint8_t address, uint8_t pin, uint8_t mode)
{
	// Determine the mode before changing the bit state in the mode cache
	// Since input = "HIGH", OR in a 1 in the appropriate place
	if(mode == true) 
	{
		_invertCache |= 1 << pin;                
	} 
	// If not, the mode must be output, so and in a 0 in the appropriate place
	else 
	{
		_invertCache &= ~(1 << pin);             
	}
	
	mcp23S17_WriteWord(address, IPOLA, _invertCache);
}
/**********************************************************************
 *
 *  mcp23S17_GpioInvertMode(uint8_t address, uint8_t mode)
 *  
 *	PORTB(HB) | PORTA(LB)
 *	
 **********************************************************************/
void mcp23S17_GpioInvertMode(uint8_t address, uint16_t mode)
{
	_invertCache = mode;
	
	mcp23S17_WriteWord(address, IPOLA, _invertCache);
}
//*********************************************************************
// mcp23S17_SetPin
//*********************************************************************
void mcp23S17_SetPin(uint8_t address, uint8_t pin)
{
	_outputCache |= (1 << pin);
	mcp23S17_WriteWord(address, GPIOA, _outputCache);	
}
//*********************************************************************
// mcp23S17_ClrPin
//*********************************************************************
void mcp23S17_ClrPin(uint8_t address, uint8_t pin)
{
	_outputCache &= ~(1 << pin);
	mcp23S17_WriteWord(address, GPIOA, _outputCache);	
}
//*********************************************************************
// mcp23S17_WritePorts
//*********************************************************************
void mcp23S17_WritePorts(uint8_t address, uint16_t value)
{
	_outputCache = value;
	mcp23S17_WriteWord(address, GPIOA, _outputCache);	
}
/**********************************************************************
 *  mcp23S17_ReadWord(uint8_t address, uint8_t reg)
 *
 *	returns PORTB as HB, and PORTA as LB
 *	
 **********************************************************************/
uint16_t mcp23S17_ReadPorts(uint8_t address)
{
	return mcp23S17_ReadWord(address, GPIOA);
}
//*********************************************************************
// mcp23S17_ReadPin
//*********************************************************************
bool mcp23S17_ReadPin(uint8_t address, uint8_t gpio)
{
	
	uint16_t value = 0; 
	
	value = mcp23S17_ReadWord(address, GPIOA);
	
	return value & (1 << gpio) ? HIGH : LOW; 
	
}


void init_spi(void)
{
	
	// spi_bus_config_t
	buscfg.sclk_io_num = SPI_PIN_NUM_CLK;   		// GPIO pin for Spi CLocK signal, or -1 if not used.
	buscfg.mosi_io_num = SPI_PIN_NUM_MOSI;   		// GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
	buscfg.miso_io_num = SPI_PIN_NUM_MISO;  	    // GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.O
	buscfg.quadwp_io_num = -1;  					// GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used.
	buscfg.quadhd_io_num = -1;  					// GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used.
	buscfg.max_transfer_sz = 0;  					// Maximum transfer size, in bytes. Defaults to 4094 if 0.
	ESP_ERROR_CHECK(spi_bus_initialize(MCP_HOST, &buscfg, DMA_CHAN)); 		// Use dma_chan 1
 
	 /****************************************************************************
     * 
     * 
     * Set Up SPI for MCP23S17 (IO Expander)
     * 
     * 
     ****************************************************************************/
	devcfg_mcp23S17.address_bits = 0;
	devcfg_mcp23S17.command_bits = 0;
	devcfg_mcp23S17.dummy_bits = 0;
	devcfg_mcp23S17.mode = 0;
	devcfg_mcp23S17.duty_cycle_pos = 0;
	devcfg_mcp23S17.cs_ena_posttrans = 0;
	devcfg_mcp23S17.cs_ena_pretrans = 0;
	devcfg_mcp23S17.clock_speed_hz = SPI_MASTER_FREQ_40M;   
	devcfg_mcp23S17.spics_io_num = SPI_PIN_NUM_CS;
	devcfg_mcp23S17.flags = 0;
	devcfg_mcp23S17.queue_size = 4;
	devcfg_mcp23S17.pre_cb = NULL;
	devcfg_mcp23S17.post_cb = NULL;
	ESP_ERROR_CHECK(spi_bus_add_device(MCP_HOST, &devcfg_mcp23S17, &handle_spi_mcp23S17));

	// spi_transaction_t
	trans_mcp23S17.flags = 0;
	trans_mcp23S17.addr = 0;
	trans_mcp23S17.cmd = 0;
	trans_mcp23S17.length = 32;   		// 4 bytes
 	trans_mcp23S17.rxlength = 24;
	trans_mcp23S17.tx_buffer = NULL;
	trans_mcp23S17.rx_buffer = NULL;
	
}