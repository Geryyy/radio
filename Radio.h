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

public:
    enum mode_e{
        remote = 0, /* saves energy */
        host /* does not care about energy consumption */
    };
    int rssi;

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
    mode_e opmode;
    CircularBuffer<uint8_t, BUF_SIZE> TxBuf;
    CircularBuffer<uint8_t, BUF_SIZE> RxBuf;

public:
    Radio(SMPcallback_t frameReadyCallback, SMPcallback_t rogueFrameCallback, RadioPHY* radiophy, mode_e mode, bool debug);
    /* call run periodically */
    void run(float TZyklus);

    /* send and receive raw data */
    uint32_t readData(uint8_t* data, uint32_t max_len);
    uint32_t sendData(uint8_t* data, uint32_t len);

    /* send data using SMP protocol ( (c) Peter Kremsner ) */
    int sendPacket(char* data, int len);
    int readPacket(void);
    bool hasreceived();

protected:
    bool _debug;

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
};

#endif // RADIO_H