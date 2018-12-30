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
    timing.Tidle = 4*timing.Tonair;
    timing.Trx = 3*timing.Tonair;
    timing.Tsleep = 4*timing.Tonair;

    debugprint("Radio()");
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
            xprintf("Radio::run() mode=");
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
    debugprint("readData()");
    receiveFifo_mutex.lock();
    uint32_t rx_len = fifo_read_bytes(data, &receiveFifo, max_len);
    receiveFifo_mutex.unlock();
    return rx_len;
}

// /* writes data to sendfifo and returns number of written bytes */
// uint32_t Radio::sendData(uint8_t* data, uint32_t len){
//     debugprint("sendData()");
//     sendFifo_mutex.lock();
//     uint32_t tx_len =  fifo_write_bytes(data, &sendFifo, len);
//     sendFifo_mutex.unlock();
//     return tx_len;
// }

int Radio::readPacket(){
    debugprint("Radio::readPacket()");
    // uint16_t len = *(phy->rxlen);
    // if(len > 0 && len < maxlen){
    //     for(int i=0; i<len; i++){
    //         uint8_t c = phy->rxdata[i]; 
    //         data[i] = c;
    //         if(_debug){
    //             xprintf("%c",(char)c);
    //         }

    //     }
    // }
    // debugprint("\n"); //newline
    // // mark data as read
    // *(phy->rxlen) = 0;
    // return len; 


    uint16_t len = *(phy->rxlen);
    
    for(int i=0; i<len; i++){
        uint8_t c = phy->rxdata[i]; 
        SMP_RecieveInBytes(&c, 1, &smp);
        if(_debug){
            xprintf("%c",(char)c);
        }
    }
    
    debugprint("\n"); //newline
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
        xprintf("LoraRadio::write(%p, %d)\n",data,len);
        xprintf("\tinput data: ");
        for(int i = 0;i<len;i++){
            xprintf("%c",data[i]);
        }
        xprintf("\n\tSMP frame:  ");
        for(uint32_t i = 0;i<txlen;i++){
            xprintf("%.2x",messageStart[i]);
        }
        xprintf("\n\n");
    }

    /* send data over radio */
    int i;
    for(i = 0;i<txlen;i++){
        if(TxBuf.full())
            break;
        TxBuf.push(messageStart[i]);
    }
    return i;
    // if(phy->transmit(messageStart,txlen) == txlen)
    //     return SUCCESS;
    // else 
    //     return ERROR;
}

void Radio::debugprint(const char* msg){
    if(_debug){
        // debug_mutex.lock();
        xprintf("DEBUG:\t%s\n",msg);
        // debug_mutex.unlock();
    }
}