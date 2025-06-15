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

#ifndef HDLC_TL1B_H_
#define HDLC_TL1B_H_

#define HDLC_TL1B_TEMPLATE                                                     \
        int16_t (&readByte)(void),                                             \
        void (&writeByte)(uint8_t data),                                       \
        uint16_t rxBuffLen,                                                    \
        class CRC,                                                             \
        uint8_t seqMax,                                                        \
        uint8_t noAckLim

#define HDLC_TL1B_TEMPLATEDEFAULT                                              \
        int16_t (&readByte)(void),                                             \
        void (&writeByte)(uint8_t data),                                       \
        uint16_t rxBuffLen,                                                    \
        class CRC,                                                             \
        uint8_t seqMax = 63U,                                                  \
        uint8_t noAckLim = 5U

#define HDLC_TL1B_TEMPLATETYPE                                                 \
        readByte,                                                              \
        writeByte,                                                             \
        rxBuffLen,                                                             \
        CRC,                                                                   \
        seqMax,                                                                \
        noAckLim

#define HDLC_TL1B_BASE_TEMPLATETYPE                                            \
        readByte,                                                              \
        writeByte,                                                             \
        rxBuffLen + 1U,                                                        \
        CRC

#include "HDLC.h"

template<HDLC_TL1B_TEMPLATEDEFAULT>
class HDLC_TL1B:
        private HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>
{
private:
    static const uint8_t MASK    = 0xC0U;
    static const uint8_t MASKINV = 0x3FU;
    static const uint8_t RESET   = 0x00U;
    static const uint8_t ACK     = 0x40U;
    static const uint8_t NACK    = 0x80U;
    static const uint8_t DATA    = 0xC0U;

public:
    static const uint16_t RXBFLEN = rxBuffLen;

    HDLC_TL1B();
    void init();

    void transmitReset();
    void transmitBlock(const void* vdata, uint16_t len);

    void transmitStart();
    void transmitByte(uint8_t data);
    void transmitBytes(const void* vdata, uint16_t len);
    void transmitEnd();

    uint16_t receive();

    uint16_t copyReceivedMessage(uint8_t (&buff)[RXBFLEN]) const;

private:
    void transmitAck(uint8_t rxs);
    void transmitNack(uint8_t rxs);

    uint8_t count_seq;
    uint8_t count_tx_noack;
};

template<HDLC_TL1B_TEMPLATE>
HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::HDLC_TL1B()
{
    init();
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::init()
{
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::init();
    count_seq = seqMax;
    count_tx_noack = 0U;
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitReset()
{
    init();
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitStart();
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitByte(RESET);
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitEnd();
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitBlock(const void* vdata, uint16_t len)
{
    transmitStart();
    transmitBytes(vdata, len);
    transmitEnd();
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitStart()
{
    if(++count_tx_noack >= noAckLim)
        transmitReset();

    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitStart();
    count_seq = (count_seq < seqMax) ? (count_seq + 1U) : 0U;
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitByte(DATA | count_seq);
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitByte(uint8_t data)
{
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitByte(data);
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
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

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::transmitEnd()
{
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitEnd();
}

template<HDLC_TL1B_TEMPLATE>
uint16_t HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::receive()
{
    uint16_t datalen = HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::receive();
    if(datalen != 0U)
    {
        datalen -= 1U;

        uint8_t frameseq;
        HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::copyReceivedMessage(&frameseq, 0U, 1U);

        uint8_t frame = frameseq & MASK;
        uint8_t rxs = frameseq & MASKINV;
        if(frame == DATA)
        {
            transmitAck(rxs);
        }
        else if(frame == ACK)
        {
            count_tx_noack = 0U;
        }
        else if(frame == NACK)
        {
        }
        else if(frame == RESET)
        {
        }
        else
        {
        }
    }
    return datalen;
}

template<HDLC_TL1B_TEMPLATE>
uint16_t HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        copyReceivedMessage(uint8_t (&buff)[RXBFLEN]) const
{
    uint16_t datalen = HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::
            copyReceivedMessage(buff, 1U, RXBFLEN);
    return datalen;
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitAck(uint8_t rxs)
{
    rxs &= MASKINV;
    rxs |= ACK;
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitStart();
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitByte(rxs);
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitEnd();
}

template<HDLC_TL1B_TEMPLATE>
void HDLC_TL1B<HDLC_TL1B_TEMPLATETYPE>::
        transmitNack(uint8_t rxs)
{
    rxs &= MASKINV;
    rxs |= NACK;
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitStart();
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitByte(rxs);
    HDLC<HDLC_TL1B_BASE_TEMPLATETYPE>::transmitEnd();
}

#endif /* HDLC_TL1B_H_ */
