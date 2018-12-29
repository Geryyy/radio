#ifndef RADIO_PHY_H
#define RADIO_PHY_H


class RadioPHY{

    // Radio_PHY();

public:
    int packetlength;
    uint8_t *rxdata;
    uint16_t *rxlen;
    virtual int getpacketlength() = 0;
    virtual int transmit(uint8_t* data, uint16_t len) = 0;
    virtual int setreceive(void) = 0;
    virtual int sleep() = 0;
    virtual int checkrxdata() = 0;
    virtual uint16_t getrxdata(uint8_t* data, uint16_t maxlen);
    virtual int getrssi() = 0;
    virtual float getsnr();
};

#endif // RADIO_PHY_H