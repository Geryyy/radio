
#include "Radio/Radio.h"
#include "mbed.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstddef>


Radio::Radio(SMPcallback_t frameReadyCallback, SMPcallback_t rogueFrameCallback, bool debug){
    _debug = debug;
    debugprint("Radio()");
    // SMP
    fifo_init(&fifo,buffer,sizeof(buffer));
    smp.buffer = &fifo;
    smp.frameReadyCallback = frameReadyCallback;
    smp.rogueFrameCallback = rogueFrameCallback;
    SMP_Init(&smp);

    // initialize Data Fifos
    fifo_init(&sendFifo, sendBuffer, sizeof(sendBuffer));
	fifo_init(&receiveFifo, receiveBuffer, sizeof(receiveBuffer));
}


uint32_t Radio::readData(uint8_t* data, uint32_t max_len){
    debugprint("readData()");
    receiveFifo_mutex.lock();
    uint32_t rx_len = fifo_read_bytes(data, &receiveFifo, max_len);
    receiveFifo_mutex.unlock();
    return rx_len;
}

/* writes data to sendfifo and returns number of written bytes */
uint32_t Radio::sendData(uint8_t* data, uint32_t len){
    debugprint("readData()");
    sendFifo_mutex.lock();
    uint32_t tx_len =  fifo_write_bytes(data, &sendFifo, len);
    sendFifo_mutex.unlock();
    return tx_len;
}

int Radio::readPacket(char* data, int* len){
    return 0;
}

int Radio::sendPacket(char* data, int len){
    uint32_t txlen = SMP_Send((unsigned char*)data,len,transmitBuffer,sizeof(transmitBuffer), &messageStart);

     /* in debug mode: print input data pointer and data length in terminal */
    if(_debug){
        printf("LoraRadio::write(%p, %d)\n",data,len);
        printf("\tinput data: ");
        for(int i = 0;i<len;i++){
            printf("%c",data[i]);
        }
        printf("\n\tSMP frame:  ");
        for(uint32_t i = 0;i<txlen;i++){
            printf("%.2x",messageStart[i]);
        }
        printf("\n\n");
    }

    /* send data over radio */
    if(sendData(messageStart, txlen) == txlen)
        return SUCCESS;
    else
        return ERROR;
}

void Radio::debugprint(const char* msg){
    if(_debug){
        debug_mutex.lock();
        printf("DEBUG:\t%s\n",msg);
        debug_mutex.unlock();
    }
}