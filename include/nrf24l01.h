

#ifndef NRF_ADDR_REVERSE
// How the TX/RX address should be transmitted to the transceiver
//   0 - straight - the first byte of the address transmitted first
//   1 - reverse - the last byte of the address transmitted first
#define NRF_ADDR_REVERSE         0
#endif


// nRF24L0 instruction definitions
#define NRF_CMD_R_REGISTER       (uint8_t)0x00 // Register read
#define NRF_CMD_W_REGISTER       (uint8_t)0x20 // Register write
#define NRF_CMD_R_RX_PAYLOAD     (uint8_t)0x61 // Read RX payload
#define NRF_CMD_W_TX_PAYLOAD     (uint8_t)0xA0 // Write TX payload
#define NRF_CMD_FLUSH_TX         (uint8_t)0xE1 // Flush TX FIFO
#define NRF_CMD_FLUSH_RX         (uint8_t)0xE2 // Flush RX FIFO
#define NRF_CMD_REUSE_TX_PL      (uint8_t)0xE3 // Reuse TX payload
#define NRF_CMD_LOCK_UNLOCK      (uint8_t)0x50 // Lock/unlock exclusive features
#define NRF_CMD_NOP              (uint8_t)0xFF // No operation (used for reading status register)

// nRF24L0 register definitions
#define NRF_REG_CONFIG           (uint8_t)0x00 // Configuration register
#define NRF_REG_EN_AA            (uint8_t)0x01 // Enable "Auto acknowledgment"
#define NRF_REG_EN_RXADDR        (uint8_t)0x02 // Enable RX addresses
#define NRF_REG_SETUP_AW         (uint8_t)0x03 // Setup of address widths
#define NRF_REG_SETUP_RETR       (uint8_t)0x04 // Setup of automatic retransmit
#define NRF_REG_RF_CH            (uint8_t)0x05 // RF channel
#define NRF_REG_RF_SETUP         (uint8_t)0x06 // RF setup register
#define NRF_REG_STATUS           (uint8_t)0x07 // Status register
#define NRF_REG_OBSERVE_TX       (uint8_t)0x08 // Transmit observe register
#define NRF_REG_RPD              (uint8_t)0x09 // Received power detector
#define NRF_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
#define NRF_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
#define NRF_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
#define NRF_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
#define NRF_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
#define NRF_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
#define NRF_REG_TX_ADDR          (uint8_t)0x10 // Transmit address
#define NRF_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
#define NRF_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
#define NRF_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
#define NRF_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
#define NRF_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
#define NRF_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5
#define NRF_REG_FIFO_STATUS      (uint8_t)0x17 // FIFO status register
#define NRF_REG_DYNPD            (uint8_t)0x1C // Enable dynamic payload length
#define NRF_REG_FEATURE          (uint8_t)0x1D // Feature register

// Register bits definitions
#define NRF_CONFIG_PRIM_RX       (uint8_t)0x01 // PRIM_RX bit in CONFIG register
#define NRF_CONFIG_PWR_UP        (uint8_t)0x02 // PWR_UP bit in CONFIG register
#define NRF_FLAG_RX_DR           (uint8_t)0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define NRF_FLAG_TX_DS           (uint8_t)0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define NRF_FLAG_MAX_RT          (uint8_t)0x10 // MAX_RT bit (maximum number of TX retransmits interrupt)

// Register masks definitions
#define NRF_MASK_REG_MAP         (uint8_t)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define NRF_MASK_CRC             (uint8_t)0x0C // Mask for CRC bits [3:2] in CONFIG register
#define NRF_MASK_STATUS_IRQ      (uint8_t)0x70 // Mask for all IRQ bits in STATUS register
#define NRF_MASK_RF_PWR          (uint8_t)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define NRF_MASK_RX_P_NO         (uint8_t)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define NRF_MASK_DATARATE        (uint8_t)0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define NRF_MASK_EN_RX           (uint8_t)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define NRF_MASK_RX_PW           (uint8_t)0x3F // Mask [5:0] bits in RX_PW_Px register
#define NRF_MASK_RETR_ARD        (uint8_t)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define NRF_MASK_RETR_ARC        (uint8_t)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define NRF_MASK_RXFIFO          (uint8_t)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define NRF_MASK_TXFIFO          (uint8_t)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define NRF_MASK_PLOS_CNT        (uint8_t)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define NRF_MASK_ARC_CNT         (uint8_t)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register
#define NRF_MASK_SETUP_AW        (uint8_t)0x03 // Mask for AW[1:0] bits in SETUP_AW register


// Retransmit delay
enum {
	NRF_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	NRF_ARD_250us  = (uint8_t)0x00,
	NRF_ARD_500us  = (uint8_t)0x01,
	NRF_ARD_750us  = (uint8_t)0x02,
	NRF_ARD_1000us = (uint8_t)0x03,
	NRF_ARD_1250us = (uint8_t)0x04,
	NRF_ARD_1500us = (uint8_t)0x05,
	NRF_ARD_1750us = (uint8_t)0x06,
	NRF_ARD_2000us = (uint8_t)0x07,
	NRF_ARD_2250us = (uint8_t)0x08,
	NRF_ARD_2500us = (uint8_t)0x09,
	NRF_ARD_2750us = (uint8_t)0x0A,
	NRF_ARD_3000us = (uint8_t)0x0B,
	NRF_ARD_3250us = (uint8_t)0x0C,
	NRF_ARD_3500us = (uint8_t)0x0D,
	NRF_ARD_3750us = (uint8_t)0x0E,
	NRF_ARD_4000us = (uint8_t)0x0F
};

// Data rate
enum {
	NRF_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	NRF_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	NRF_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
};

// RF output power in TX mode
enum {
	NRF_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	NRF_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	NRF_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	NRF_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
};

// CRC encoding scheme
enum {
	NRF_CRC_off   = (uint8_t)0x00, // CRC disabled
	NRF_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	NRF_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
};

// nRF24L01 power control
enum {
	NRF_PWR_UP   = (uint8_t)0x02, // Power up
	NRF_PWR_DOWN = (uint8_t)0x00  // Power down
};

// Transceiver mode
enum {
	NRF_MODE_RX = (uint8_t)0x01, // PRX
	NRF_MODE_TX = (uint8_t)0x00  // PTX
};

// Enumeration of RX pipe addresses and TX address
enum {
	NRF_PIPE0  = (uint8_t)0x00, // pipe0
	NRF_PIPE1  = (uint8_t)0x01, // pipe1
	NRF_PIPE2  = (uint8_t)0x02, // pipe2
	NRF_PIPE3  = (uint8_t)0x03, // pipe3
	NRF_PIPE4  = (uint8_t)0x04, // pipe4
	NRF_PIPE5  = (uint8_t)0x05, // pipe5
	NRF_PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
};

// State of auto acknowledgment for specified pipe
enum {
	NRF_AA_OFF = (uint8_t)0x00,
	NRF_AA_ON  = (uint8_t)0x01
};

// Status of the RX FIFO
enum {
	NRF_STATUS_RXFIFO_DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
	NRF_STATUS_RXFIFO_EMPTY = (uint8_t)0x01, // The RX FIFO is empty
	NRF_STATUS_RXFIFO_FULL  = (uint8_t)0x02, // The RX FIFO is full
	NRF_STATUS_RXFIFO_ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
};

// Status of the TX FIFO
enum {
	NRF_STATUS_TXFIFO_DATA  = (uint8_t)0x00, // The TX FIFO contains data and available locations
	NRF_STATUS_TXFIFO_EMPTY = (uint8_t)0x01, // The TX FIFO is empty
	NRF_STATUS_TXFIFO_FULL  = (uint8_t)0x02, // The TX FIFO is full
	NRF_STATUS_TXFIFO_ERROR = (uint8_t)0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
};

// Result of RX FIFO reading
typedef enum {
	NRF_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	NRF_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	NRF_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	NRF_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	NRF_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	NRF_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	NRF_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} nrf_rx_result;



void nrf_init(void);
uint8_t nrf_check(void);

void nrf_set_power_mode(uint8_t mode);
void nrf_set_Operational_mode(uint8_t mode);
void nrf_set_rf_channel(uint8_t channel);
void nrf_set_auto_retry(uint8_t ard, uint8_t arc);
void nrf_set_addr_width(uint8_t addr_width);
void nrf_set_addr(uint8_t pipe, const uint8_t *addr);
void nrf_set_tx_power(uint8_t tx_pwr);
void nrf_set_rata_rate(uint8_t data_rate);
void nrf_set_crc_scheme(uint8_t scheme);
void nrf_set_rx_pipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len);
void nrf_close_pipe(uint8_t pipe);
void nrf_enable_aa(uint8_t pipe);
void nrf_disable_aa(uint8_t pipe);

uint8_t nrf_get_addr_width(void);
uint8_t nrf_get_status(void);
uint8_t nrf_get_irq_flags(void);
uint8_t nrf_get_status_rx_fifo(void);
uint8_t nrf_get_status_tx_fifo(void);
uint8_t nrf_get_rx_source(void);
uint8_t nrf_get_retransmit_counters(void);

void nrf_reset_plos(void);
void nrf_flush_tx(void);
void nrf_flush_rx(void);
void nrf_clear_irq_flags(void);

void nrf_write_payload(uint8_t *pBuf, uint8_t length);
nrf_rx_result nrf_read_payload(uint8_t *pBuf, uint8_t *length);