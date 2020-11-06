#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "nrf24l01.h"


#define NRF_HOST            VSPI_HOST
#define DMA_CHAN            2

#define NRF_CE_PIN          GPIO_NUM_5
#define NRF_SPI_PIN_MISO    GPIO_NUM_12
#define NRF_SPI_PIN_MOSI    GPIO_NUM_13
#define NRF_SPI_PIN_CLK     GPIO_NUM_14
#define NRF_SPI_PIN_CS      GPIO_NUM_15
#define NRF_IRQ_PIN         GPIO_Pin_10

#define NRF_CSN_L           gpio_set_level(NRF_SPI_PIN_CS, 0);
#define NRF_CSN_H           gpio_set_level(NRF_SPI_PIN_CS, 1);

#define NRF_CE_L          	gpio_set_level(NRF_CE_PIN, 0);
#define NRF_CE_H            gpio_set_level(NRF_CE_PIN, 1);

#define NRF_RX_ON           NRF_CE_H
#define NRF_RX_OFF          NRF_CE_L


// The address used to test presence of the transceiver,
// note: should not exceed 5 bytes
#define NRF_TEST_ADDR            "nRF24"

static const uint8_t NRF_ADDR_REGS[7] = {
		NRF_REG_RX_ADDR_P0,
		NRF_REG_RX_ADDR_P1,
		NRF_REG_RX_ADDR_P2,
		NRF_REG_RX_ADDR_P3,
		NRF_REG_RX_ADDR_P4,
		NRF_REG_RX_ADDR_P5,
		NRF_REG_TX_ADDR
};

spi_bus_config_t buscfg;
spi_bus_config_t buscfg_mcp23S17;
spi_device_interface_config_t devcfg_mcp23S17;
spi_device_handle_t handle_spi_mcp23S17;
spi_transaction_t trans_mcp23S17;

static void nrf_ll_init(void);
static uint8_t nrf_read_reg(uint8_t reg);
static void nrf_write_reg(uint8_t reg, uint8_t value) ;
static uint8_t nrf_ll_rw(uint8_t data);


static void nrf_ll_init(void)
{
	
    gpio_set_direction(NRF_CE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NRF_CE_PIN, 1);


	buscfg.sclk_io_num = NRF_SPI_PIN_CLK;   		// GPIO pin for Spi CLocK signal, or -1 if not used.
	buscfg.mosi_io_num = NRF_SPI_PIN_MOSI;   		// GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
	buscfg.miso_io_num = NRF_SPI_PIN_MISO;  	    // GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.O
	buscfg.quadwp_io_num = -1;  					// GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used.
	buscfg.quadhd_io_num = -1;  					// GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used.
	buscfg.max_transfer_sz = 0;  					// Maximum transfer size, in bytes. Defaults to 4094 if 0.
	ESP_ERROR_CHECK(spi_bus_initialize(NRF_HOST, &buscfg, DMA_CHAN)); 		// Use dma_chan 1
 
	devcfg_mcp23S17.address_bits = 0;
	devcfg_mcp23S17.command_bits = 0;
	devcfg_mcp23S17.dummy_bits = 0;
	devcfg_mcp23S17.mode = 0;
	devcfg_mcp23S17.duty_cycle_pos = 0;
	devcfg_mcp23S17.cs_ena_posttrans = 0;
	devcfg_mcp23S17.cs_ena_pretrans = 0;
	devcfg_mcp23S17.clock_speed_hz = SPI_MASTER_FREQ_40M;   
	devcfg_mcp23S17.spics_io_num = NRF_SPI_PIN_CS;
	devcfg_mcp23S17.flags = 0;
	devcfg_mcp23S17.queue_size = 4;
	devcfg_mcp23S17.pre_cb = NULL;
	devcfg_mcp23S17.post_cb = NULL;
	ESP_ERROR_CHECK(spi_bus_add_device(NRF_HOST, &devcfg_mcp23S17, &handle_spi_mcp23S17));

	trans_mcp23S17.flags = 0;
	trans_mcp23S17.addr = 0;
	trans_mcp23S17.cmd = 0;
	trans_mcp23S17.length = 32;   		// 4 bytes
 	trans_mcp23S17.rxlength = 24;
	trans_mcp23S17.tx_buffer = NULL;
	trans_mcp23S17.rx_buffer = NULL;
	
}

static uint8_t nrf_ll_rw(uint8_t data)
{
    return 0;
}

static uint8_t nrf_read_reg(uint8_t reg) {
	uint8_t value;

	NRF_CSN_L;
	nrf_ll_rw(reg & NRF_MASK_REG_MAP);
	value = nrf_ll_rw(NRF_CMD_NOP);
	NRF_CSN_H;

	return value;
}

static void nrf_write_reg(uint8_t reg, uint8_t value) {
	NRF_CSN_L;
	if (reg < NRF_CMD_W_REGISTER) {
		nrf_ll_rw(NRF_CMD_W_REGISTER | (reg & NRF_MASK_REG_MAP));
		nrf_ll_rw(value);
	} else {
		nrf_ll_rw(reg);
		if ((reg != NRF_CMD_FLUSH_TX) && (reg != NRF_CMD_FLUSH_RX) && \
				(reg != NRF_CMD_REUSE_TX_PL) && (reg != NRF_CMD_NOP)) {
			nrf_ll_rw(value);
		}
	}
	NRF_CSN_H;
}

static void nrf_read_mb_reg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	NRF_CSN_L;
	nrf_ll_rw(reg);
	while (count--) { *pBuf++ = nrf_ll_rw(NRF_CMD_NOP); }
	NRF_CSN_H;
}

static void nrf_write_mb_reg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	NRF_CSN_L;
	nrf_ll_rw(reg);
	while (count--) {
		nrf_ll_rw(*pBuf++);
	}
	NRF_CSN_H;
}

uint8_t nrf_check(void) {
	uint8_t rxbuf[sizeof(NRF_TEST_ADDR) - 1U];
	uint8_t *ptr = (uint8_t *)NRF_TEST_ADDR;
	uint8_t idx;

	nrf_write_mb_reg(
		NRF_CMD_W_REGISTER | NRF_REG_TX_ADDR,
		ptr,
		sizeof(NRF_TEST_ADDR) - 1U
	);

	nrf_read_mb_reg(
		NRF_CMD_R_REGISTER | NRF_REG_TX_ADDR,
		rxbuf,
		sizeof(NRF_TEST_ADDR) - 1U
	);

	for (idx = 0U; idx < sizeof(NRF_TEST_ADDR) - 1U; idx++) {
		if (rxbuf[idx] != *ptr++) {
			return 0U;
		}
	}
	return !0U;
}

void nrf_set_power_mode(uint8_t mode) {
	uint8_t reg;

	reg = nrf_read_reg(NRF_REG_CONFIG);
	if (mode == NRF_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Standby-I mode with consumption about 26uA
		reg |= NRF_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// It goes into Power Down mode with consumption about 900nA
		reg &= ~NRF_CONFIG_PWR_UP;
	}
	nrf_write_reg(NRF_REG_CONFIG, reg);
}

void nrf_set_operational_mode(uint8_t mode) {
	uint8_t reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg = nrf_read_reg(NRF_REG_CONFIG);
	reg &= ~NRF_CONFIG_PRIM_RX;
	reg |= (mode & NRF_CONFIG_PRIM_RX);
	nrf_write_reg(NRF_REG_CONFIG, reg);
}

void nrf_set_crc_scheme(uint8_t scheme) {
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg = nrf_read_reg(NRF_REG_CONFIG);
	reg &= ~NRF_MASK_CRC;
	reg |= (scheme & NRF_MASK_CRC);
	nrf_write_reg(NRF_REG_CONFIG, reg);
}

void nrf_set_rf_channel(uint8_t channel) {
	nrf_write_reg(NRF_REG_RF_CH, channel);
}

void nrf_set_auto_retry(uint8_t ard, uint8_t arc) {
	nrf_write_reg(
		NRF_REG_SETUP_RETR,
		(uint8_t)((ard << 4) | (arc & NRF_MASK_RETR_ARC))
	);
}

void nrf_set_addr_width(uint8_t addr_width) {
	nrf_write_reg(NRF_REG_SETUP_AW, addr_width - 2U);
}

void nrf_set_addr(uint8_t pipe, const uint8_t *addr) {
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case NRF_PIPETX:
		case NRF_PIPE0:
		case NRF_PIPE1:
			// Get address width
			addr_width = nrf_get_addr_width();
#if (!NRF_ADDR_REVERSE)
			nrf_write_mb_reg(
				NRF_CMD_W_REGISTER | NRF_ADDR_REGS[pipe],
				(uint8_t *)addr,
				addr_width
			);
#else
			// Write address in reverse order
			NRF_CSN_L;
			NRF_LL_RW(NRF_CMD_W_REGISTER | NRF_ADDR_REGS[pipe]);
			while (addr_width--) { NRF_LL_RW(*(addr + addr_width)); }
			NRF_CSN_H;
#endif // NRF_ADDR_REVERSE
			break;
		case NRF_PIPE2:
		case NRF_PIPE3:
		case NRF_PIPE4:
		case NRF_PIPE5:
			// Write first byte from the addr buffer,
			// it will be the LSByte of the pipe address
			nrf_write_reg(NRF_ADDR_REGS[pipe], *addr);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}

void nrf_set_tx_power(uint8_t tx_pwr) {
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg = nrf_read_reg(NRF_REG_RF_SETUP);
	reg &= ~NRF_MASK_RF_PWR;
	reg |= tx_pwr;
	nrf_write_reg(NRF_REG_RF_SETUP, reg);
}

void nrf_set_data_rate(uint8_t data_rate) {
	uint8_t reg;

	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg = nrf_read_reg(NRF_REG_RF_SETUP);
	reg &= ~NRF_MASK_DATARATE;
	reg |= data_rate;
	nrf_write_reg(NRF_REG_RF_SETUP, reg);
}

void nrf_set_rx_pipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (nrf_read_reg(NRF_REG_EN_RXADDR) | (1U << pipe)) & NRF_MASK_EN_RX;
	nrf_write_reg(NRF_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nrf_write_reg(NRF_REG_RX_PW_P0 + pipe, payload_len & NRF_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = nrf_read_reg(NRF_REG_EN_AA);
	if (aa_state == NRF_AA_ON) {
		reg |= (1U << pipe);
	} else {
		reg &= ~(1U << pipe);
	}
	nrf_write_reg(NRF_REG_EN_AA, reg);
}

void nrf_close_pipe(uint8_t pipe) {
	uint8_t reg;

	reg = nrf_read_reg(NRF_REG_EN_RXADDR);
	reg &= ~(1U << pipe);
	reg &= NRF_MASK_EN_RX;
	nrf_write_reg(NRF_REG_EN_RXADDR, reg);
}

void nrf_enable_aa(uint8_t pipe) {
	uint8_t reg;

	// Set bit in EN_AA register
	reg = nrf_read_reg(NRF_REG_EN_AA);
	reg |= (1U << pipe);
	nrf_write_reg(NRF_REG_EN_AA, reg);
}

void nrf_disable_aa(uint8_t pipe) {
	if (pipe > 5U) {
		// Disable Auto-ACK for ALL pipes
		nrf_write_reg(NRF_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		uint8_t reg;
		reg = nrf_read_reg(NRF_REG_EN_AA);
		reg &= ~(1U << pipe);
		nrf_write_reg(NRF_REG_EN_AA, reg);
	}
}

uint8_t nrf_get_addr_width(void) {
	return nrf_read_reg(NRF_REG_SETUP_AW) + 2U;
}

uint8_t nrf_get_status(void) {
	return nrf_read_reg(NRF_REG_STATUS);
}

uint8_t nrf_get_irq_flags(void) {
	return (nrf_read_reg(NRF_REG_STATUS) & NRF_MASK_STATUS_IRQ);
}

uint8_t nrf_get_status_rx_fifo(void) {
	return (nrf_read_reg(NRF_REG_FIFO_STATUS) & NRF_MASK_RXFIFO);
}

uint8_t nrf_get_status_tx_fifo(void) {
	return ((nrf_read_reg(NRF_REG_FIFO_STATUS) & NRF_MASK_TXFIFO) >> 4);
}

uint8_t nrf_get_rx_source(void) {
	return ((nrf_read_reg(NRF_REG_STATUS) & NRF_MASK_RX_P_NO) >> 1);
}

uint8_t nrf_get_retransmit_couters(void) {
	return (nrf_read_reg(NRF_REG_OBSERVE_TX));
}

void nrf_reset_plos(void) {
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = nrf_read_reg(NRF_REG_RF_CH);
	nrf_write_reg(NRF_REG_RF_CH, reg);
}

void nrf_flush_tx(void) {
	nrf_write_reg(NRF_CMD_FLUSH_TX, NRF_CMD_NOP);
}

void nrf_flush_rx(void) {
	nrf_write_reg(NRF_CMD_FLUSH_RX, NRF_CMD_NOP);
}

void nrf_clear_irq_flags(void) {
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg = nrf_read_reg(NRF_REG_STATUS);
	reg |= NRF_MASK_STATUS_IRQ;
	nrf_write_reg(NRF_REG_STATUS, reg);
}

void nrf_write_payload(uint8_t *pBuf, uint8_t length) {
	nrf_write_mb_reg(NRF_CMD_W_TX_PAYLOAD, pBuf, length);
}

nrf_rx_result nrf_read_payload(uint8_t *pBuf, uint8_t *length) {
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (nrf_read_reg(NRF_REG_STATUS) & NRF_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6U) {
		// Get payload length
		*length = nrf_read_reg(NRF_REG_RX_PW_P0 + pipe);

		// Read a payload from the RX FIFO
		if (*length) {
			nrf_read_mb_reg(NRF_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nrf_rx_result)pipe);
	}

	// The RX FIFO is empty
	*length = 0U;

	return NRF_RX_EMPTY;
}


void nrf_init(void) {
    nrf_ll_init();

	nrf_write_reg(NRF_REG_CONFIG,     0x08);
	nrf_write_reg(NRF_REG_EN_AA,      0x3F);
	nrf_write_reg(NRF_REG_EN_RXADDR,  0x03);
	nrf_write_reg(NRF_REG_SETUP_AW,   0x03);
	nrf_write_reg(NRF_REG_SETUP_RETR, 0x03);
	nrf_write_reg(NRF_REG_RF_CH,      0x02);
	nrf_write_reg(NRF_REG_RF_SETUP,   0x0E);
	nrf_write_reg(NRF_REG_STATUS,     0x00);
	nrf_write_reg(NRF_REG_RX_PW_P0,   0x00);
	nrf_write_reg(NRF_REG_RX_PW_P1,   0x00);
	nrf_write_reg(NRF_REG_RX_PW_P2,   0x00);
	nrf_write_reg(NRF_REG_RX_PW_P3,   0x00);
	nrf_write_reg(NRF_REG_RX_PW_P4,   0x00);
	nrf_write_reg(NRF_REG_RX_PW_P5,   0x00);
	nrf_write_reg(NRF_REG_DYNPD,      0x00);
	nrf_write_reg(NRF_REG_FEATURE,    0x00);

	uint8_t addr[5];
	uint8_t idx;
	for (idx = 0U; idx < sizeof(addr); idx++) {
		addr[idx] = 0xE7;
	}
	nrf_set_addr(NRF_PIPETX, addr);
	nrf_set_addr(NRF_PIPE0, addr);
	for (idx = 0U; idx < sizeof(addr); idx++) {
		addr[idx] = 0xC2;
	}
	nrf_set_addr(NRF_PIPE1, addr);
	for (idx = 2U; idx < 6U; idx++) {
		addr[0] = idx + 0xC1;
		nrf_set_addr(idx, addr);
	}
	nrf_flush_rx();
	nrf_flush_tx();
	nrf_clear_irq_flags();

	NRF_CSN_H;

    nrf_disable_aa(0xFF);
    nrf_set_rf_channel(115);
    nrf_set_data_rate(NRF_DR_250kbps);
    nrf_set_crc_scheme(NRF_CRC_2byte);
    nrf_set_addr_width(3);
    static const uint8_t nrf_addr[] = { 0xE7, 0x1C, 0xE3 };
    nrf_set_addr(NRF_PIPE1, nrf_addr);
    nrf_set_rx_pipe(NRF_PIPE1, NRF_AA_OFF, 5);
    nrf_set_operational_mode(NRF_MODE_RX);
    nrf_set_power_mode(NRF_PWR_UP);
    NRF_CE_H;
}