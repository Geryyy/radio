#include "Radio.h"
#include "radiophy.h"
#include "mbed.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstddef>
#include "logprintf.h"
/* for LOG and WARNING macros: */
#define MODULE_NAME "RADIO"

Radio::Radio(SMPcallback_t frameReadyCallback, SMPcallback_t rogueFrameCallback, RadioPHY* radiophy, mode_e mode, bool debug){
    _debug = debug;

    // timing
    timing.Tonair = 0.5; // seconds .. time to transmit lora packet over air
    timing.Ttx = timing.Tonair;
    timing.Tidle = 0*timing.Tonair; // 4
    timing.Trx = 10*timing.Tonair; // 3
    timing.Tsleep = 0*timing.Tonair; // 4

    debugprint("Radio::Radio()");
    // radio physical layer object
    phy = radiophy;
    state = INIT;
    opmode = mode;

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

void Radio::run(float TZyklus){
    static state_e mstate;
    static float t = 0.0;

    /* state machine
     * INIT --> TX --> IDLE --> RX --> SLEEP ---
     *           ^_____________________________|
     */
    do{
        // time in current state
        if(state != mstate) 
            t = 0.0;
        else 
            t = t + TZyklus;

        // update state merker
        mstate = state;
        
        // state machine
        switch(opmode){
            case Radio::remote:{
                    if(state == INIT && t>0.0){
                        state = TX;
                    }
                    else if(state == TX && t > timing.Ttx){
                        state = IDLE;                     
                        phy->sleep();
                    }
                    else if(state == IDLE && t > timing.Tidle){
                        state = RX;
                        phy->setreceive();
                    }
                    else if(state == RX && t > timing.Trx){
                        state = SLEEP;
                        phy->sleep();
                    }
                    else if(state == SLEEP && t > timing.Tsleep){
                        state = TX;
                    }
                    //else;
                }  
                break;
            case Radio::host:{
                    if(state == INIT && t>0.0){
                        state = TX;
                    }
                    else if(state == TX && t > (timing.Ttx)){
                        state = RX; 
                        phy->setreceive(); 
                    }
                    else if(state == RX && t > (timing.Tidle + timing.Trx + timing.Tsleep)){
                        state = TX;
                    }
                    //else;
                }
                break;
            default: LOG("Radio::run() invalid mode");
        }  
    }while(mstate != state);

    // actions 
    if(state == RX){
        // receive data
        int len = readPacket();
        // sync state machine with remote/host's state machine
        if(len>0){
            t = 0.0;
        }
    }

    if(state == IDLE){
        int rssi = phy->getrssi();
        if(_debug){
            xprintf("rssi: %d\n",rssi);
        }
    }

    if(state == TX){
        // transmit data
        if(!TxBuf.empty()){
            int packetlength = phy->packetlength;
            uint8_t* packet = new uint8_t[packetlength];
            if(packet != NULL){
                int i;
                for( i= 0; i<packetlength; i++){
                    if(TxBuf.empty())
                        break;
                    TxBuf.pop(packet[i]);
                }
                phy->transmit(packet,i);
            }
            delete [] packet;
        }   
    }

    if(_debug){
        // if(state!=mstate)
        {
            xprintf("DEBUG: Radio::run() mode=");
            switch(opmode){
                case Radio::remote: xprintf("remote  "); break;
                case Radio::host:   xprintf("host  "); break;
                default:            xprintf("invalid mode  ");
            }
            xprintf("state=");
            switch(state){
                case INIT:   xprintf("INIT "); break;
                case TX:     xprintf("TX   "); break;
                case IDLE:   xprintf("IDLE "); break;
                case RX:     xprintf("RX   "); break;
                case SLEEP:  xprintf("SLEEP"); break;
                default:     xprintf("default");
            }
            xprintf("  t=%5.2f\n",t);
        }
    }
}


uint32_t Radio::readData(uint8_t* data, uint32_t max_len){
    debugprint("Radio::readData()");
    // receiveFifo_mutex.lock();
    // uint32_t rx_len = fifo_read_bytes(data, &receiveFifo, max_len);
    // receiveFifo_mutex.unlock();
    int i = 0;
    while(!RxBuf.empty()){
        if(i > max_len){
            break;
        }
        else{
            RxBuf.pop(data[i]);
        }
        i++;
    }
    return i;
}


int Radio::readPacket(){
    
    uint16_t len = *(phy->rxlen);
    if(_debug){
        xprintf("DEBUG: Radio::readPacket() \tlen=%d data: ",len);
    }

    for(int i=0; i<len; i++){
        uint8_t c = phy->rxdata[i]; 
        RxBuf.push(c); // save raw data for readData() function
        SMP_RecieveInBytes(&c, 1, &smp);
        if(_debug){
            xprintf("<%d>:%c",i,(char)c);
        }
    }
    xprintf("\n");

    // mark data as read
    *(phy->rxlen) = 0;
    return len; 
}

bool Radio::hasreceived(){
    if(_debug){
        // xprintf("Radio::hasreceived(): phy->rxlen = %d\n",*(phy->rxlen));
    }
    return ((*(phy->rxlen) > 0) ? true : false);
}

int Radio::sendPacket(char* data, int len){
    uint32_t txlen = SMP_Send((unsigned char*)data,len,transmitBuffer,sizeof(transmitBuffer), &messageStart);

     /* in debug mode: print input data pointer and data length in terminal */
    if(_debug){
        xprintf("DEBUG: Radio::sendPacket() \tlen=%d input data: ",len);
        for(int i = 0;i<len;i++){
            xprintf("%c",data[i]);
        }
        xprintf("\tSMP frame:  ");
        for(uint32_t i = 0;i<txlen;i++){
            xprintf("%.2x",messageStart[i]);
        }
        xprintf("\n");
    }

    /* send data over radio */
    int i;
    for(i = 0;i<txlen;i++){
        if(TxBuf.full()){
            debugprint("RADIO::sendPacket() ERROR: TXBUF FULL!");
            break;
        }
        TxBuf.push(messageStart[i]);
    }
    return i;
}

uint32_t Radio::sendData(uint8_t* data, uint32_t len){

     /* in debug mode: print input data pointer and data length in terminal */
    if(_debug){
        xprintf("DEBUG: Radio::sendData() \tlen=%d input data: ",len);
        for(int i = 0;i<len;i++){
            xprintf("%c",data[i]);
        }
        xprintf("\n");
    }

    /* send data over radio */
    int i;
    for(i = 0;i<len;i++){
        if(TxBuf.full()){
            debugprint("RADIO::sendData() ERROR: TXBUF FULL!");
            break;
        }
        TxBuf.push(data[i]);
    }
    return i;
}

void Radio::debugprint(const char* msg){
    if(_debug){
        xprintf("DEBUG: %s\n",msg);
    }
}
