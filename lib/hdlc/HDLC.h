/*
 Copyright 2016 Djones A. Boni

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#ifndef HDLC_H_
#define HDLC_H_

#include <stdint.h>
#include <string.h>

#define HDLC_TEMPLATE                                                          \
        int16_t (&readByte)(void),                                             \
        void (&writeByte)(uint8_t data),                                       \
        uint16_t rxBuffLen,                                                    \
        class HDLC_CRC

#define HDLC_TEMPLATETYPE                                                      \
        readByte,                                                              \
        writeByte,                                                             \
        rxBuffLen,                                                             \
        HDLC_CRC

template<HDLC_TEMPLATE>
class HDLC
{
private:
    static const uint8_t DATAINVBIT;
    static const uint8_t DATASTART;
    static const uint8_t DATAESCAPE;
    static const uint8_t DATAESCAPELIST[];

public:
    static const uint16_t RXBFLEN = rxBuffLen;

    HDLC();
    void init();

    void transmitBlock(const void* vdata, uint16_t len);

    void transmitStart();
    void transmitByte(uint8_t data);
    void transmitBytes(const void* vdata, uint16_t len);
    void transmitEnd();

    uint16_t receive();

    uint16_t copyReceivedMessage(uint8_t (&buff)[RXBFLEN]) const;
    uint16_t copyReceivedMessage(uint8_t *buff, uint16_t pos, uint16_t num) const;

// private:
    static void escapeAndWriteByte(uint8_t data) {
        const uint8_t n = sizeof(DATAESCAPELIST)/sizeof(DATAESCAPELIST[0]);
        for(int8_t i = 0; i < n; ++i)
        {
            if(data == DATAESCAPELIST[i])
            {
                writeByte(DATAESCAPE);
                data ^= DATAINVBIT;
                break;
            }
        }
        writeByte(data);
    }

    enum {
        ESCAPED   = -1,
        RECEIVING = 0,
        OK        = 1,
        CRCERR    = 2
    };

    HDLC_CRC txcrc;

    int8_t status;
    uint16_t len;
    HDLC_CRC crc;
    uint8_t data[RXBFLEN];
};



template<HDLC_TEMPLATE>
const uint8_t HDLC<HDLC_TEMPLATETYPE>::DATAINVBIT = 0x20U;

template<HDLC_TEMPLATE>
const uint8_t HDLC<HDLC_TEMPLATETYPE>::DATASTART  = '~';

template<HDLC_TEMPLATE>
const uint8_t HDLC<HDLC_TEMPLATETYPE>::DATAESCAPE = '}';

template<HDLC_TEMPLATE>
const uint8_t HDLC<HDLC_TEMPLATETYPE>::DATAESCAPELIST[] =
        { DATASTART, DATAESCAPE };



template<HDLC_TEMPLATE>
HDLC<HDLC_TEMPLATETYPE>::HDLC()
{
    init();
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::init()
{
    len = 0U;
    status = RECEIVING;
    crc.init();
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::
        transmitBlock(const void* vdata, uint16_t len)
{
    transmitStart();
    transmitBytes(vdata, len);
    transmitEnd();
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::transmitStart()
{
    writeByte(DATASTART);
    txcrc.init();
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::transmitByte(uint8_t data)
{
    escapeAndWriteByte(data);
    txcrc.update(data);
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::
        transmitBytes(const void* vdata, uint16_t len)
{
    const uint8_t* data = (const uint8_t*)vdata;
    while(len)
    {
        transmitByte(*data);
        ++data;
        --len;
    }
}

template<HDLC_TEMPLATE>
void HDLC<HDLC_TEMPLATETYPE>::transmitEnd()
{
    txcrc.final();
    for(int8_t i = 0; i < txcrc.size; ++i)
        escapeAndWriteByte(txcrc[i]);
    writeByte(DATASTART);
}

template<HDLC_TEMPLATE>
uint16_t HDLC<HDLC_TEMPLATETYPE>::receive()
{
    int16_t c = readByte();
    if(c == -1)
        return 0U;

    if(status >= OK)
        init();

    uint16_t retv = 0U;

    if(c == DATASTART)
    {
        if(status == RECEIVING && len != 0U)
        {
            if(crc.good())
            {
                status = OK;
                len -= crc.size;
                retv = len;
            }
            else
            {
                status = CRCERR;
            }
        }
        else
        {
            init();
        }
    }
    else
    {
        if(status == ESCAPED)
        {
            status = RECEIVING;

            c ^= DATAINVBIT;
            crc.update(c);
            if(len < RXBFLEN)
                data[len] = c;
            ++len;
        }
        else if(c != DATAESCAPE)
        {
            crc.update(c);
            if(len < RXBFLEN)
                data[len] = c;
            ++len;
        }
        else
        {
            status = ESCAPED;
        }
    }

    return retv;
}

template<HDLC_TEMPLATE>
uint16_t HDLC<HDLC_TEMPLATETYPE>::copyReceivedMessage(uint8_t (&buff)[RXBFLEN]) const
{
    const uint16_t datalen = (len > RXBFLEN) ? RXBFLEN : len;
    memcpy(buff, data, datalen);
    return datalen;
}

template<HDLC_TEMPLATE>
uint16_t HDLC<HDLC_TEMPLATETYPE>::
        copyReceivedMessage(uint8_t *buff, uint16_t pos, uint16_t num) const
{
    const uint16_t datalen = (len > RXBFLEN) ? RXBFLEN : len;
    if(pos < RXBFLEN)
    {
        num = (pos + num) > datalen ? (datalen - pos) : num;
        memcpy(buff, &data[pos], num);
    }
    else
    {
        num = 0;
    }
    return num;
}

#endif /* HDLC_H_ */
