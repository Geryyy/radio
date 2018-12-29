
#include "Radio.h"
#include "radiophy.h"
#include "mbed.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstddef>




Radio::Radio(SMPcallback_t frameReadyCallback, SMPcallback_t rogueFrameCallback, RadioPHY* radiophy, bool debug){
    _debug = debug;

    // timing
    timing.Tonair = 0.5; // seconds .. time to transmit lora packet over air
    timing.Ttx = timing.Tonair;
    timing.Tidle = 1*timing.Tonair;
    timing.Trx = 4*timing.Tonair;
    timing.Tsleep = 1*timing.Tonair;

    debugprint("Radio()");
    // radio physical layer object
    phy = radiophy;
    state = INIT;

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
    if(state == INIT && t>0.0){
        // do init things
        state = TX;
    }
    else if(state == TX && t > timing.Ttx){
        

        state = IDLE;
        //set radio to idle
        // phy->sleep();
    }
    else if(state == IDLE && t > timing.Tidle){

        state = RX;
        // set radio to receive mode
        phy->setreceive();
    }
    else if(state == RX && t > timing.Trx){

        // readPacket();

        
        state = SLEEP;
        // set radio to sleep
        // phy->sleep();
    }
    else if(state == SLEEP && t > timing.Tsleep){
        // wake up radio
        state = TX;
        
    }
    // else;

    if(state != mstate) 
        t = 0.0;
    else 
        t = t + TZyklus;

    // actions 
    if(state == RX){
        // receive data
        readPacket();
    }

    if(state == IDLE){
        int rssi = phy->getrssi();
        if(_debug){
            printf("rssi: %d\n",rssi);
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
            printf("Radio::run() state = ");
            switch(state){
                case INIT:  printf("INIT"); break;
                case TX:  printf("TX"); break;
                case IDLE:  printf("IDLE"); break;
                case RX:  printf("RX"); break;
                case SLEEP:  printf("SLEEP"); break;
                default: printf("default");
            }
            printf("\n");
        }
    }

    // update state merker
    mstate = state;
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
    //             printf("%c",(char)c);
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
            printf("%c",(char)c);
        }
    }
    
    debugprint("\n"); //newline
    // mark data as read
    *(phy->rxlen) = 0;
    return len; 
}

bool Radio::hasreceived(){
    if(_debug){
        // printf("Radio::hasreceived(): phy->rxlen = %d\n",*(phy->rxlen));
    }
    return ((*(phy->rxlen) > 0) ? true : false);
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
        debug_mutex.lock();
        printf("DEBUG:\t%s\n",msg);
        debug_mutex.unlock();
    }
}