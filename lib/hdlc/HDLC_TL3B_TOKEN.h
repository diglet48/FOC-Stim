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

#ifndef HDLC_TL3B_TOKEN_H_
#define HDLC_TL3B_TOKEN_H_

#include "HDLC.h"

#define HDLC_TL3B_TOKEN_TEMPLATE                                               \
        int16_t (&readByte)(void),                                             \
        void (&writeByte)(uint8_t data),                                       \
        uint16_t rxBuffLen,                                                    \
        class CRC

#define HDLC_TL3B_TOKEN_TEMPLATETYPE                                           \
        readByte,                                                              \
        writeByte,                                                             \
        rxBuffLen,                                                             \
        CRC

#define HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE                                      \
        readByte,                                                              \
        writeByte,                                                             \
        rxBuffLen,                                                             \
        CRC

template<HDLC_TL3B_TOKEN_TEMPLATE>
class HDLC_TL3B_TOKEN:
        private HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>
{
public:
    enum Command_t {
        CMD_RESET = 0,
        CMD_GIVE_TOKEN,
        CMD_ACK_TOKEN,
        CMD_WRITE,
        CMD_READ
    };

    enum TokenState_t {
        TOKEN_HAVE = 0,
        TOKEN_PASSING,
        TOKEN_DONT_HAVE
    };

    struct MessageHeader_t {
        Command_t command;
        uint8_t from;
        uint8_t to;
    };

    static const uint16_t RXBFLEN = rxBuffLen;

    HDLC_TL3B_TOKEN(uint8_t address, bool master = false);

    void transmitReset();
    void transmitGiveToken(uint8_t to_addr);
private:
    void transmitAckToken(uint8_t to_addr);

    void transmitStart(Command_t command, uint8_t to_addr);

public:
    void transmitStartWrite(uint8_t to_addr);
    void transmitStartRead(uint8_t to_addr);

    void transmitByte(uint8_t data);
    void transmitBlock(const void* vdata, uint16_t len);
    void transmitEnd();

    uint16_t receive();

    MessageHeader_t copyMessageHeader();
    uint16_t copyMessageData(uint8_t *buff, uint16_t pos, uint16_t num) const;
    uint16_t copyMessageData(uint8_t (&buff)[RXBFLEN]) const;

    void setAddress(uint8_t address) { Address = address; }
    uint8_t getAddress() const { return Address; }
    uint16_t getRxCount() const { return RxCount; }
    uint16_t getTxCount() const { return TxCount; }
    TokenState_t getTokenState() const { return TokenState; }
    bool haveToken() const { return TokenState == TOKEN_HAVE; }
    uint8_t getTokenAddress() const { return TokenAddress; }

private:
    uint8_t Address;
    uint16_t RxCount;
    uint16_t TxCount;
    TokenState_t TokenState;
    uint8_t TokenAddress;
};

template<HDLC_TL3B_TOKEN_TEMPLATE>
HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        HDLC_TL3B_TOKEN(uint8_t address, bool master)
{
    setAddress(address);
    RxCount = 0;
    TxCount = 0;
    TokenState = master ? TOKEN_HAVE : TOKEN_DONT_HAVE;
    TokenAddress = 0;
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::transmitReset()
{
    TokenState = TOKEN_HAVE;
    transmitStart(CMD_RESET, 0); /* broadcast */
    transmitEnd();
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitGiveToken(uint8_t to_addr)
{
    transmitStart(CMD_GIVE_TOKEN, to_addr);
    transmitEnd();

    TokenAddress = to_addr;
    TokenState = TOKEN_PASSING;
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitAckToken(uint8_t to_addr)
{
    transmitStart(CMD_ACK_TOKEN, to_addr);
    transmitEnd();
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitStartWrite(uint8_t to_addr)
{
    transmitStart(CMD_WRITE, to_addr);
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitStartRead(uint8_t to_addr)
{
    transmitStart(CMD_READ, to_addr);
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitStart(Command_t command, uint8_t to_addr)
{
    ++TxCount;

    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitStart();
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitByte(command); /* Command */
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitByte(Address); /* From */
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitByte(to_addr); /* To */
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::transmitByte(uint8_t data)
{
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitByte(data);
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        transmitBlock(const void* vdata, uint16_t len)
{
    const uint8_t* data = (const uint8_t*)vdata;
    while(len)
    {
        transmitByte(*data);
        ++data;
        --len;
    }
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
void HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::transmitEnd()
{
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::transmitEnd();
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
uint16_t HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::receive()
{
    uint16_t datalen = HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::receive();

    if(datalen >= 3U)
    {
        ++RxCount;
        datalen -= 3U;

        MessageHeader_t header = copyMessageHeader();

        if(
                (header.to == 0U || header.to == Address) &&
                header.from != Address)
        {
            /* Message for me. */

            switch(header.command) {

                case CMD_RESET:
                    TokenState = TOKEN_DONT_HAVE;
                    datalen = 0U;
                    break;

                case CMD_GIVE_TOKEN:
                    if(header.to != 0)
                    {
                        /* Do not accept a token given in a broadcast. */
                        TokenState = TOKEN_HAVE;
                        transmitAckToken(header.from);
                        TokenAddress = header.from;
                    }
                    else
                    {
                        /* Error. Token given in a broadcast. Not my fault. */
                    }
                    datalen = 0U;
                    break;

                case CMD_ACK_TOKEN:
                    if(header.to != 0)
                    {
                        /* Do not accept a token acknowledged in a broadcast. */
                        TokenState = TOKEN_DONT_HAVE;
                    }
                    else
                    {
                        /* Error. Token acknowledged in a broadcast. Not my fault. */
                    }
                    datalen = 0U;
                    break;

                case CMD_WRITE:
                case CMD_READ:
                default:
                    break;
            }
        }
        else
        {
            /* Message not for me. */
            datalen = 0U;
        }
    }
    else
    {
        /* Invalid message (too short). */
        datalen = 0U;
    }

    return datalen;
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
typename HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::MessageHeader_t
        HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        copyMessageHeader()
{
    uint8_t buff[3U];
    HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::copyReceivedMessage(&buff[0U], 0U, sizeof(buff));
    MessageHeader_t header = { static_cast<Command_t>(buff[0U]), buff[1U], buff[2U] };
    return header;
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
uint16_t HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        copyMessageData(uint8_t *buff, uint16_t pos, uint16_t num) const
{
    uint16_t datalen = HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::
            copyReceivedMessage(buff, pos + 3U, num);
    return datalen;
}

template<HDLC_TL3B_TOKEN_TEMPLATE>
uint16_t HDLC_TL3B_TOKEN<HDLC_TL3B_TOKEN_TEMPLATETYPE>::
        copyMessageData(uint8_t (&buff)[RXBFLEN]) const
{
    uint16_t datalen = HDLC<HDLC_TL3B_TOKEN_BASE_TEMPLATETYPE>::
            copyReceivedMessage(buff, 3U, RXBFLEN);
    return datalen;
}

#endif /* HDLC_TL3B_TOKEN_H_ */
