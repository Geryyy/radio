/* @file: radio.h
 * @author: Gerald Ebmer
 * @date: 13.06.2018
 * @brief: header file for RFM98W lora module support class
 */

#include "mbed.h"
#include "Radio.h"
#include "globals.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_TEMP				 0x3C

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

typedef struct
{
	void (*receivecallback)(void);
	void (*transmitCompleteClb)(void);
	uint32_t frequency;
	uint8_t txPower;
	uint8_t spreadingFactor;
	uint32_t signalBandwith;
	uint8_t codingRateDenominator;
	uint16_t preambleLength;
	uint8_t syncword;
	uint8_t crc;
	uint8_t messageSize;
}loraSettings_t;

typedef enum
{
		idle,
		send,
		receive,
		sleep_mode,
		disconnected
}loraStatus_e;


class RFM98W : public Radio
{
public:


private:
    // SPI
    SPI spi;
    // Chip Select
    DigitalOut cs;
    // Reset
    DigitalOut reset;
    // Interrupt
    DigitalIn dio0;


    uint32_t _frequency;
    int _packetIndex;
    // void (*_onReceive)(void);
    // void (*_transmitCompleteClb)(void);
    uint8_t messageSize;
    volatile uint8_t loraLocked;

/* Methods */
public:
    RFM98W(PinName MOSI, PinName MISO, PinName SCK, PinName CS, PinName RESET, PinName INTERRUPT, uint32_t timeout = 200, SMPcallback_t frameReadyCallback = NULL, SMPcallback_t rogueFrameCallback = NULL, bool debug = false);
    int serviceRadio();
    
    int startreceive();
    int stopreceive();

    void lora_init(loraSettings_t* settings);
    void lora_reset();
    void lora_deinit();
    uint8_t lora_readVersion();
    uint8_t lora_ready();

private:
    // service radio 
    uint32_t blockSendTimeout;

    uint32_t beginnToWaitTimestamp;




    int sendBytes(unsigned char *data, int len);

    void spi_init(void);
    
    uint8_t lora_readRegister(uint8_t address);
    void lora_writeRegister(uint8_t address, uint8_t value);
    void lora_writeRegisterSafe(uint8_t address, uint8_t value);
    uint8_t lora_singleTransfer(uint8_t address, uint8_t value);
    void lora_fifoTransfer(uint8_t address, const uint8_t* values, uint8_t length);
    void PIOINT1_IRQHandler(void);
    void lora_poll(void);

    loraStatus_e lora_getStatus();
    void lora_setMessageSize(uint8_t size);
    uint8_t lora_getMessageSize();
    void lora_onReceive(void (*callback)(void));
    int lora_setReceive();
    void lora_setIdle();
    void lora_setSleep();
    void lora_setTxPower(uint8_t level);
    void lora_setFrequency(uint32_t frequency);
    void lora_setSpreadingFactor(uint8_t sf);
    void lora_setSignalBandwidth(uint32_t sbw);
    void lora_setCodingRate4(uint8_t denominator);
    void lora_setPreambleLength(uint16_t length);
    void lora_setSyncWord(uint8_t sw);
    void lora_crc();
    void lora_noCrc();
    void lora_implicitHeaderMode();
    void lora_explicitHeaderMode();
    uint8_t lora_getMode();

    uint8_t lora_parsePacket();
    uint8_t lora_packetRssi();
    float lora_packetSnr();
    uint8_t lora_available();
    int16_t lora_read();
    uint16_t lora_readBytes(uint8_t* buffer, uint16_t length);
    uint8_t lora_sendBytes(const uint8_t* buffer, uint8_t length);
    int16_t lora_peek();
    uint8_t lora_random();
    int8_t lora_getTemperature();
    uint8_t lora_locked();
    void lora_handleDio0Rise();

    // callbacks
     void lora_receiveData(void);
     void lora_sendDataComplete(void);
};