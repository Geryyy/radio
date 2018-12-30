/* @file: radio.h
 * @author: Gerald Ebmer
 * @date: 13.06.2018
 * @brief: header file for radio class
 */

#ifndef Radio_H
#define RADIO_H
#include "mbed.h"
#include "libsmp.h"
#include "libfifo.h"
#include "globals.h"
#include "radiophy.h"

#define TX_MAX (127) 
#define BUF_SIZE (256)

typedef signed char (*SMPcallback_t)(fifo_t* buffer);

class Radio{

private:
    enum state_e{
        INIT = 0,
        TX,
        IDLE,
        RX,
        SLEEP
    };

    typedef struct timing_s{
        float Tonair, Ttx, Tidle, Trx, Tsleep;
    }timing_t;

    state_e state;
    timing_t timing;
    CircularBuffer<uint8_t, BUF_SIZE> TxBuf;
    CircularBuffer<uint8_t, BUF_SIZE> RxBuf;

public:
    Radio(SMPcallback_t frameReadyCallback, SMPcallback_t rogueFrameCallback, RadioPHY* radiophy, bool debug);
    void run(float TZyklus);

    /* brief: call function periodically to transmit data in sendFifo and to receive Data to receiveFifo 
     *        pure virtual method -> has to be implemented in derived class 
     */
    // virtual int serviceRadio() = 0; 


    /* send data */
    uint32_t readData(uint8_t* data, uint32_t max_len);
    uint32_t sendData(uint8_t* data, uint32_t len);

    /* send data using SMP protocol ( (c) Peter Kremsner ) */
    // Callback<void(char*,int)>receivePacket;
    // int readPacket(char* data, int* len);
    int sendPacket(char* data, int len);
    int readPacket(void);
    bool hasreceived();



protected:
    bool _debug;
    // Mutex debug_mutex;

    Mutex sendFifo_mutex;
    Mutex receiveFifo_mutex;
    fifo_t sendFifo;
    fifo_t receiveFifo;

    // SMP
    unsigned char buffer[SMP_SEND_BUFFER_LENGTH(GET_BUFFER_SIZE(LORA_PACKET_LENGTH * 1))]; // smp rx buffer, estimate packet size for received smp packet
    fifo_t fifo;
    smp_struct_t smp;
    unsigned int frameLength; //Länge des SMP Frames
    unsigned char transmitBuffer[SMP_SEND_BUFFER_LENGTH(1000)]; 
    unsigned char* messageStart; //Pointer auf den Start der Nachricht im transmitBuffer
    signed char (*smp_frameReady)(fifo_t* buffer); //Frame wurde empfangen

    void debugprint(const char* msg);

private:
    RadioPHY* phy;
    uint8_t sendBuffer[GET_BUFFER_SIZE(LORA_PACKET_LENGTH * 10)];
    uint8_t receiveBuffer[GET_BUFFER_SIZE(LORA_PACKET_LENGTH * 10)];

    // virtual int sendBytes(unsigned char *data, int len) = 0; // rein virtuelle Methode

};

#endif // RADIO_H