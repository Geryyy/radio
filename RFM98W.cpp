#include "RFM98W.h"
#include "globals.h"
#include "logprintf.h"



/* libs: 
* https://github.com/sandeepmistry/arduino-LoRa/blob/master/src/LoRa.cpp
* https://github.com/ARMmbed/mbed-semtech-lora-rf-drivers/blob/master/SX1276/SX1276_LoRaRadio.cpp
*/

// #define _debug 0

RFM98W::RFM98W(PinName MOSI, PinName MISO, PinName SCK, PinName CS, PinName RESET, PinName INTERRUPT, uint32_t timeout,  bool debug)
		: spi(MOSI,MISO,SCK), cs(CS,1), reset(RESET,1), dio0(INTERRUPT)
{
	_debug = debug;
	debugprint("RFM98W()");


	packetlength = LORA_PACKET_LENGTH; 
	rxdata = &rxbuffer[0];
	rxlen = &rxbufferlen;
	

	blockSendTimeout = timeout;
    spi.format(8,0); /* 8 bits; mode 0: CPOL = 0, CPHA = 0 */
	spi.frequency(1000000);

	dio0.mode(PullUp);
	dio0.rise(callback(this, &RFM98W::DIO0_IRQHandler));

	/* todo: implement */
	loraSettings_t lora_settings;
	memset(&lora_settings, 0, sizeof(loraSettings_t));

	lora_settings.codingRateDenominator = LORA_CODING_RATE;
	lora_settings.crc = LORA_CRC;
	lora_settings.frequency = LORA_FREQUENCY;
	lora_settings.preambleLength = LORA_PREAMBLE_LENGTH;
	lora_settings.receivecallback = NULL; //lora_dataReadyCallback;
	lora_settings.signalBandwith = LORA_SIGNAL_BANDWIDTH;
	lora_settings.spreadingFactor = LORA_SPREADING_FACTOR;
	lora_settings.syncword = LORA_SYNC_WORD;
	lora_settings.txPower = LORA_TX_POWER;
	lora_settings.messageSize = LORA_PACKET_LENGTH;
	lora_settings.transmitCompleteClb = NULL; //lora_transmitComplete;
	lora_init(&lora_settings);

	// startreceive();
	eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
}

int RFM98W::getpacketlength(){
	return packetlength;
}

int RFM98W::transmit(uint8_t* data, uint16_t len){
	// uint8_t loraPacket[LORA_PACKET_LENGTH]; /* todo: use packetlength */
	uint32_t index = 0;
	uint32_t txlen = len;


	// for(int i = 0; i<sizeof(loraPacket); i++){
	// 	loraPacket[i] = 0;
	// }

	while(1) {
		if(txlen>packetlength){
			index += lora_sendBytes(&data[index], packetlength);
			txlen -= packetlength;
		}
		else{
			index += lora_sendBytes(&data[index], txlen);
			break;
		}
	}
	return index;
}

int RFM98W::setreceive(void){
	lora_setReceive();
	return 1;
}

int RFM98W::sleep(){
	lora_setIdle();// lora_setSleep();
	return 1;
}

int RFM98W::checkrxdata(){
	return lora_available();
}

uint16_t RFM98W::getrxdata(uint8_t* data, uint16_t maxlen){
	// return number of read bytes
	return lora_readBytes(data, maxlen);
}

int RFM98W::getrssi(){
	return lora_packetRssi();
}

float RFM98W::getsnr(){
	return lora_packetSnr();
}

void RFM98W::debugprint(const char* msg){
	if(_debug){
		xprintf("DEBUG:\t%s\n",msg);
	}
}


// /* implementation of virtual function defined in parent class */
// int RFM98W::serviceRadio(){
//     uint8_t loraPacket[LORA_PACKET_LENGTH];
// 	uint8_t ret = 1;

// 	debugprint("serviceRadio()");

// 	lora_poll();

// 	if (lora_getStatus() == send)
// 		return 1;

// 	uint8_t i;
// 	uint32_t txlen = fifo_datasize(&sendFifo);
// 	uint32_t dataRead;
// 	while(txlen>0)
// 		if (txlen >= LORA_PACKET_LENGTH) {
// 			sendFifo_mutex.lock();
// 			dataRead = fifo_read_bytes(loraPacket, &sendFifo, LORA_PACKET_LENGTH);
// 			sendFifo_mutex.unlock();
// 			lora_sendBytes(loraPacket, LORA_PACKET_LENGTH);
// 		}
// 		else
// 		{
// 			if(beginnToWaitTimestamp == 0)
// 			{
// 				beginnToWaitTimestamp = time(NULL);
// 			}
// 			// send data after timeout and pad remaining packet space with zero data
// 			else if((time(NULL)-beginnToWaitTimestamp) > blockSendTimeout)
// 			{
// 				beginnToWaitTimestamp = 0;
// 				sendFifo_mutex.lock();
// 				dataRead = fifo_read_bytes(loraPacket, &sendFifo, LORA_PACKET_LENGTH);
// 				sendFifo_mutex.unlock();
// 				for (i = dataRead; i < sizeof(loraPacket); i++) {
// 					loraPacket[i] = 0;
// 				}
// 				lora_sendBytes(loraPacket, LORA_PACKET_LENGTH);
// 			}
// 		}
// 		// stopreceive();
// 	}
// 	else{	
		
// 		// stopreceive();
// 	}
// 	startreceive();
// 	return ret;
// }

int RFM98W::sendBytes(unsigned char *data, int len){
    uint8_t loraPacket[LORA_PACKET_LENGTH];
	uint8_t ret = SUCCESS;

	debugprint("sendBytes()");

	// lora_poll();

	if (lora_getStatus() == send)
		return ERROR;

	uint32_t dataAvailable = len; //fifo_datasize(&sendFifo);

	while (dataAvailable > 0) {
		if (dataAvailable >= LORA_PACKET_LENGTH) {
			// dataRead = data; //fifo_read_bytes(loraPacket, &sendFifo, LORA_PACKET_LENGTH);
			memcpy(loraPacket, data, LORA_PACKET_LENGTH);
			lora_sendBytes(loraPacket, LORA_PACKET_LENGTH);
			dataAvailable -= LORA_PACKET_LENGTH;
			data += LORA_PACKET_LENGTH;
		}
		else
		{
			memset(loraPacket, 0x00, LORA_PACKET_LENGTH);
			memcpy(loraPacket,data,dataAvailable);
			lora_sendBytes(loraPacket, LORA_PACKET_LENGTH);
			dataAvailable -= dataAvailable;
		}
	}
	return ret;
}


//###########################PUBLIC FUNCTIONS##############################################

void RFM98W::lora_reset()
{
	debugprint("lora_reset()");

	reset.write(0);
	wait_ms(10);
	reset.write(1);
	wait_ms(10);
}

uint8_t RFM98W::lora_readVersion()
{
	uint8_t reg;
	reg = lora_readRegister(REG_VERSION);
	return reg;
}

void RFM98W::lora_init(loraSettings_t* settings)
{
	debugprint("lora_init()");

	loraLocked = 0;
	lora_reset();

	// start SPI
	lora_setSleep();
	lora_setFrequency(settings->frequency);
	lora_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	lora_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
	lora_writeRegister(REG_LNA, lora_readRegister(REG_LNA) | 0x03);
	lora_setTxPower(settings->txPower);
	lora_setSpreadingFactor(settings->spreadingFactor);
	lora_setCodingRate4(settings->codingRateDenominator);
	lora_setSignalBandwidth(settings->signalBandwith);
	lora_setPreambleLength(settings->preambleLength);
	//lora_setSyncWord(settings->syncword);
	lora_setMessageSize(settings->messageSize);
	if (settings->crc)
	{
		lora_crc();
	}
	else
	{
		lora_noCrc();
	}

	// if (settings->receivecallback)
	// 	lora_onReceive(settings->receivecallback);
	// else
	// 	lora_onReceive(0);


	// if (settings->transmitCompleteClb)
	// 	_transmitCompleteClb = settings->transmitCompleteClb;
	// else
	// 	_transmitCompleteClb = 0;

	lora_setIdle();

	// Chip_GPIO_SetupPinInt(LPC_GPIO, 1, 9, GPIO_INT_RISING_EDGE);
}

void RFM98W::lora_deinit()
{
	// put in sleep mode
	lora_setSleep();
	// stop SPI
}

uint8_t RFM98W::lora_ready()
{
	debugprint("lora_ready()");

	loraStatus_e stat = lora_getStatus();
	return (stat != send) && (stat != receive) && (stat != disconnected);
}


int RFM98W::startreceive(){
	return lora_setReceive();
}

int RFM98W::stopreceive(){
	lora_setIdle();
	return SUCCESS;
}

//###########################PRIVATE FUNCTIONS##############################################


uint8_t RFM98W::lora_singleTransfer(uint8_t address, uint8_t value)
{
	if (loraLocked) return 0;
	loraLocked = 1;
	uint8_t receive[2] =
	{ 0, 0 };
	uint8_t transmit[2] =
	{ address, value };

	cs.write(0);
	wait_us(10);
	// Chip_GPIO_DisableInt(LPC_GPIO, 1, 0x200);
	spi.write((const char*) transmit, (int) sizeof(transmit), (char*) receive, (int) sizeof(receive));
	// Chip_GPIO_EnableInt(LPC_GPIO, 1, 0x200);
	wait_us(10);
	cs.write(1);
	loraLocked = 0;
	return receive[1];
}

void RFM98W::lora_fifoTransfer(uint8_t address, const uint8_t* values, uint8_t length)
{

	if (loraLocked) return;
	loraLocked = 1;
	uint8_t data[256];
	data[0] = address | 0x80;
	memcpy(&data[1], values, length);

	cs.write(0);
	wait_us(10);
	// Chip_GPIO_DisableInt(LPC_GPIO, 1, 0x200);
	spi.write((const char*) data, (int) (length + 1), (char*) NULL, (int) 0);
	// Chip_GPIO_EnableInt(LPC_GPIO, 1, 0x200);
	wait_us(10);
	cs.write(1);
	loraLocked = 0;
}

void RFM98W::DIO0_IRQHandler(void)
{
	// then defer the handleDio0Rise call to the other thread
 	queue.call(callback(this, &RFM98W::lora_handleDio0Rise));
}

void RFM98W::lora_poll()
{
	debugprint("lora_poll()");

	if (dio0.read())
	{
		lora_handleDio0Rise();
	}
// #ifdef Lora_Block
// #endif
}





/************ low level IO ************/

uint8_t RFM98W::lora_readRegister(uint8_t address)
{
	return lora_singleTransfer(address & 0x7f, 0x00);
}

void RFM98W::lora_writeRegister(uint8_t address, uint8_t value)
{
	lora_singleTransfer(address | 0x80, value);
}

void RFM98W::lora_writeRegisterSafe(uint8_t address, uint8_t value)
{
	uint8_t mode = lora_getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP)) lora_setSleep();
	lora_writeRegister(address, value);
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP)) lora_writeRegister(REG_OP_MODE, mode);
}



loraStatus_e RFM98W::lora_getStatus()
{
	uint8_t mode = lora_getMode();
	if(mode == (MODE_LONG_RANGE_MODE | MODE_TX))
	return send;
	if(mode == (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS) || mode == (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
	return receive;
	if(mode == (MODE_LONG_RANGE_MODE | MODE_STDBY))
	return idle;
	if(mode == (MODE_LONG_RANGE_MODE | MODE_SLEEP))
	return send;
	return disconnected;
}



uint8_t RFM98W::lora_sendBytes(const uint8_t* buffer, uint8_t length)
{
	debugprint("lora_sendBytes()");

	loraStatus_e stat = lora_getStatus();
	if((stat == send) || (stat == disconnected))
		return 0;
	lora_setIdle();
	lora_writeRegister(REG_DIO_MAPPING_1, 1 << 6);
	lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
	lora_fifoTransfer(REG_FIFO,buffer,length);
	if(messageSize == 0)
	lora_writeRegister(REG_PAYLOAD_LENGTH, length);
	lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	while ((lora_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
	lora_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	stat = idle;
	return length;
}

void RFM98W::lora_setMessageSize(uint8_t size)
{
	debugprint("lora_setMessageSize()");

	if(size > 0)
	{
		lora_implicitHeaderMode();
			lora_writeRegister(REG_PAYLOAD_LENGTH,size);
	}
	else
	{
		lora_explicitHeaderMode();
	}
	messageSize = size;
}

uint8_t RFM98W::lora_getMessageSize()
{
	return messageSize;
}

int RFM98W::lora_setReceive()
{
	debugprint("lora_setReceive()");

	if(!lora_ready())
		return ERROR;

	lora_writeRegister(REG_DIO_MAPPING_1, 0);
	if(lora_getMode() != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS))
	{
		lora_writeRegister(REG_PAYLOAD_LENGTH, lora_getMessageSize());
		lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
		lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	}

	return SUCCESS;
}



int RFM98W::lora_setReceiveSingle()
{
	debugprint("lora_setReceiveSingle()");

	if(!lora_ready())
		return ERROR;

	lora_writeRegister(REG_DIO_MAPPING_1, 0);
	if(lora_getMode() != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
	{
		lora_writeRegister(REG_PAYLOAD_LENGTH, lora_getMessageSize());
		lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
		lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}
	return SUCCESS;
}



uint8_t RFM98W::lora_parsePacket() {
	uint8_t packetLength = 0;

	debugprint("lora_parsePacket()");

	loraStatus_e stat = lora_getStatus();
	if(stat != send && stat != disconnected)
	{
			lora_writeRegister(REG_PAYLOAD_LENGTH, lora_getMessageSize());

			uint8_t irqFlags = lora_readRegister(REG_IRQ_FLAGS);
			// clear IRQ's
			lora_writeRegister(REG_IRQ_FLAGS, irqFlags);

			if ((irqFlags & IRQ_RX_DONE_MASK) && ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)){
				// received a packet
				_packetIndex = 0;

				// read packet length
				if (messageSize > 0) {
					packetLength = messageSize;
					} else {
					packetLength = lora_readRegister(REG_RX_NB_BYTES);
				}

				// set FIFO address to current RX address
				lora_writeRegister(REG_FIFO_ADDR_PTR,lora_readRegister(REG_FIFO_RX_CURRENT_ADDR));

				// put in standby mode
				lora_setIdle();

			} else if (lora_getMode()
			!= (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
				// not currently in RX mode

				// reset FIFO address
				lora_writeRegister(REG_FIFO_ADDR_PTR, 0);

				// put in single RX mode
				lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
			}
		return packetLength;
	}
	return 0;
}

int16_t RFM98W::lora_packetRssi() {
	uint8_t reg;
	debugprint("lora_packetRssi()");
	reg = lora_readRegister(REG_PKT_RSSI_VALUE);
	// rssi in dBm
	return (-137+reg);
}

float RFM98W::lora_packetSnr() {
	uint8_t reg;
	debugprint("lora_packetSnr()");
	reg = lora_readRegister(REG_PKT_SNR_VALUE);
	return ((int8_t) reg) * 0.25;
}

uint8_t RFM98W::lora_available() {
	debugprint("lora_available()");

	loraStatus_e stat = lora_getStatus();
	if(stat == send ||stat == disconnected)
	return 0;
	uint8_t reg;
	reg = lora_readRegister(REG_RX_NB_BYTES);
	
	if(_debug){
		xprintf("\t nr of bytes available: %d\n",reg-_packetIndex);
	}

	return (reg - _packetIndex);
}



// int8_t RFM98W::lora_getTemperature()
// {
// 	int8_t temp;
// 	uint8_t regVal;

// 	debugprint("lora_getTemperature()");

// 		regVal = lora_readRegister(REG_TEMP);

// 	temp = regVal & 0x7F;
// 	if((regVal & 0x80))
// 	{
// 		temp *= -1;
// 	}
// 	return temp;
// }

// void RFM98W::lora_onReceive(void (*callback)(void)) {
// 	_onReceive = callback;
// }


uint8_t RFM98W::lora_getMode()
{
	uint8_t mode;
	mode = lora_readRegister(REG_OP_MODE);

	if(_debug){
		debugprint("lora_getMode()");
		switch(mode){
			case MODE_LONG_RANGE_MODE: debugprint("long range mode"); break;
			case MODE_SLEEP: debugprint("sleep mode"); break;
			case MODE_STDBY: debugprint("standby mode"); break;
			case MODE_TX: debugprint("tx mode"); break;
			case MODE_RX_CONTINUOUS: debugprint("rx continuous mode"); break;
			case MODE_RX_SINGLE: debugprint("rx single mode"); break;
			default: debugprint("default mode"); break;
		}
	}

	return mode;
}


void RFM98W::lora_handleDio0Rise()
{
	debugprint("lora_handleDio0Rise()");

	uint8_t irqFlags;
	lora_writeRegister(REG_PAYLOAD_LENGTH, lora_getMessageSize());
	irqFlags = lora_readRegister(REG_IRQ_FLAGS);
	lora_writeRegister(REG_IRQ_FLAGS, irqFlags);

	if(_debug){
		xprintf("lora irqFlags: %d\n", irqFlags);
	}

	if (irqFlags & IRQ_RX_DONE_MASK)
	{
		if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
		{
			_packetIndex = 0;
			// if (_onReceive) _onReceive();
			/* proccess rx data */
			lora_receiveData();
		}
	}
	else if (irqFlags & IRQ_TX_DONE_MASK)
	{
		// if (_transmitCompleteClb) _transmitCompleteClb();
		lora_sendDataComplete();
	}
	/* todo test */
	lora_setReceive(); 
}

 void RFM98W::lora_receiveData(void) {
	debugprint("lora_receiveData()");

	int dataSize;
	uint8_t data[LORA_PACKET_LENGTH];
	dataSize = lora_readBytes(data, LORA_PACKET_LENGTH);

	if(_debug){
		xprintf("received Bytes:\n");
		for(int i = 0; i<dataSize; i++){
			xprintf("%c",data[i]);
		}
	}

	/* copy data to be accessed by radio object */
	if(dataSize > 0 && dataSize <= MAX_RX_BUFFER_SIZE){
		debugprint("----> copy data to rxbuffer <----");
		for(int i=0; i<dataSize; i++){
			rxbuffer[i] = data[i];
		}
		rxbufferlen = dataSize;
	}
}

 void RFM98W::lora_sendDataComplete(void){
	debugprint("lora_sendDataComplete()");

	/* todo: move smp to radio class */
	// if(fifo_datasize(&sendFifo) < LORA_PACKET_LENGTH)
	// 	lora_setReceive(); //Goto receive after transmition

}



/************ IO Methodes ************/

int16_t RFM98W::lora_read() {
	debugprint("lora_read()");
#warning implementation might be wrong..
	if (!lora_available()) {
		return -1;
	}

	_packetIndex++;

	uint8_t reg;
		reg = lora_readRegister(REG_FIFO);

	return reg;
}

uint16_t RFM98W::lora_readBytes(uint8_t* buffer, uint16_t length)
{
	uint16_t i = 0;
	uint8_t data;

	debugprint("lora_readBytes()");

	while((data = lora_read()) >= 0)
	{
		if(i == length)
		break;
		buffer[i] = data;
		i++;
	}
	return i;
}

int16_t RFM98W::lora_peek() {
	debugprint("lora_peek()");

	if (!lora_available()) {
		return -1;
	}
	uint8_t b;
		// store current FIFO address
		int currentAddress = lora_readRegister(REG_FIFO_ADDR_PTR);

		// read
		b = lora_readRegister(REG_FIFO);

		// restore FIFO address
		lora_writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

/************ Config Methodes ************/

void RFM98W::lora_setIdle() {
	debugprint("lora_setIdle()");

	lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void RFM98W::lora_setSleep() {
	debugprint("lora_setSleep()");

	lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void RFM98W::lora_setTxPower(uint8_t level) {
	debugprint("lora_setTxPower()");

	if (level < 2) {
		level = 2;
		} else if (level > 17) {
		level = 17;
	}
		lora_writeRegisterSafe(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void RFM98W::lora_setFrequency(uint32_t frequency) {
	_frequency = frequency;

	debugprint("lora_setFrequency()");

	uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
	uint8_t mode = lora_getMode();
	if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
	lora_setSleep();
		lora_writeRegister(REG_FRF_MSB, (uint8_t) (frf >> 16));
		lora_writeRegister(REG_FRF_MID, (uint8_t) (frf >> 8));
		lora_writeRegister(REG_FRF_LSB, (uint8_t) (frf >> 0));
		if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		lora_writeRegister(REG_OP_MODE, mode);
}

void RFM98W::lora_setSpreadingFactor(uint8_t sf) {
	debugprint("lora_setSpreadingFactor()");

	if (sf < 6) {
		sf = 6;
		} else if (sf > 12) {
		sf = 12;
	}
	uint8_t mode = lora_getMode();
	if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
	lora_setSleep();
		// if (sf == 6) {
		// 	lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		// 	lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
		// 	} else {
		// 	lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		// 	lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
		// }

		lora_writeRegister(REG_MODEM_CONFIG_2,
		(lora_readRegister(REG_MODEM_CONFIG_2) & 0x0f)
		| ((sf << 4) & 0xf0));
		if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		lora_writeRegister(REG_OP_MODE, mode);
}

void RFM98W::lora_setSignalBandwidth(uint32_t sbw) {
	uint8_t bw;

	debugprint("lora_setSignalBandwidth()");

	if (sbw <= 7.8E3) {
		bw = 0;
		} else if (sbw <= 10.4E3) {
		bw = 1;
		} else if (sbw <= 15.6E3) {
		bw = 2;
		} else if (sbw <= 20.8E3) {
		bw = 3;
		} else if (sbw <= 31.25E3) {
		bw = 4;
		} else if (sbw <= 41.7E3) {
		bw = 5;
		} else if (sbw <= 62.5E3) {
		bw = 6;
		} else if (sbw <= 125E3) {
		bw = 7;
		} else if (sbw <= 250E3) {
		bw = 8;
		} else if (sbw <= 500E3){
		bw = 9;
	}
		lora_writeRegisterSafe(REG_MODEM_CONFIG_1,
		(lora_readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void RFM98W::lora_setCodingRate4(uint8_t denominator) {
	debugprint("lora_setCodingRate4()");

	if (denominator < 5) {
		denominator = 5;
		} else if (denominator > 8) {
		denominator = 8;
	}

	uint8_t cr = denominator - 4;
		lora_writeRegisterSafe(REG_MODEM_CONFIG_1,
		(lora_readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void RFM98W::lora_setPreambleLength(uint16_t length) {
	debugprint("lora_setPreambleLength()");

	uint8_t mode = lora_getMode();
	if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
	lora_setSleep();
		lora_writeRegister(REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
		lora_writeRegister(REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
		if(mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		lora_writeRegister(REG_OP_MODE, mode);
}

void RFM98W::lora_setSyncWord(uint8_t sw) {
	debugprint("setSyncWord()");
#warning regsiter used by the driver does not exist on chip
	// lora_writeRegisterSafe(REG_SYNC_WORD, sw);
}


void RFM98W::lora_crc() {
		lora_writeRegisterSafe(REG_MODEM_CONFIG_2,
		lora_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void RFM98W::lora_noCrc() {
		lora_writeRegisterSafe(REG_MODEM_CONFIG_2,
		lora_readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

// uint8_t RFM98W::lora_random() {
// 	uint8_t reg;
// 		reg = lora_readRegister(REG_RSSI_WIDEBAND);
// 	return reg;
// }

void RFM98W::lora_explicitHeaderMode()
{
		lora_writeRegister(REG_MODEM_CONFIG_1,
		lora_readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void RFM98W::lora_implicitHeaderMode() {
		lora_writeRegister(REG_MODEM_CONFIG_1,
		lora_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

