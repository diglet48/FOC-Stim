# [HDLC](https://github.com/djboni/hdlc)

by [Djones A. Boni](https://twitter.com/djonesboni)


HDLC stads for High-Level Data Link Control. This library adapts the HDLC
protocol for standard serial, allowing easy communication with CRC error
detection among microcontrollers connected via a serial bus.

For more information about HDLC and take a look at 
[RFC-1662 - PPP in HDLC-like Framing](https://tools.ietf.org/html/rfc1662)
or in the doc/rfc1662.txt file.

Implemented as a C++ class template, the HDLC Data Link Layer can be extended
using class inheritance. The library also provides an One Byte Transport Layer
derived class template as an extension example.

You can use HDLC both for closed- and open-source projects. You are also
free to keep changes to this library to yourself. However we'll enjoy your
improvements and suggestions. `:-)`

You are free to copy, modify, and distribute HDLC with attribution under
the terms of the
[Apache License Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
See the doc/LICENSE file for details.


## Supported hardware

The project has been developed and tested with AVR microcontroller family
(ATmega 2560) using [Arduinutil](https://github.com/djboni/arduinutil) as
development platform.

However it should be very easy to port to another microcontroller family and
development platform.


## How to use HDLC

* Create an HDLC object
* Send data with HDLC
* Receive data with HDLC

```cpp
#include "HDLC.h"
#include "Arduinutil.h"

HDLC<Serial1_read, Serial1_writeByte, 16> hdlc;

void hdlc_sendMsg() {
    uint8_t msg[] = "Hello world!";
    hdlc.transmitBlock(msg, sizeof(msg));
}

void hdlc_receiveMsg() {
    if(hdlc.receive() != 0U)
    {
        uint8_t buff[hdlc.RXBFLEN];
        uint16_t size = hdlc.copyReceivedMessage(buff);

        Serial_print("Msg[%u]=%s\n", size, buff);
    }
}

int main(void)
{
    init();
    Serial_begin(115200, SERIAL_8N1);
    Serial1_begin(19200, SERIAL_8N1);

    Serial_print("Init\n");

    hdlc_sendMsg();

    for(;;)
    {
        hdlc_receiveMsg();
    }
}
```


## Contributing to HDLC

If you have suggestions for improving HDLC, please
[open an issue or pull request on GitHub](https://github.com/djboni/hdlc).


## Important files

[README.md](https://github.com/djboni/hdlc/blob/master/README.md)
Fast introduction (this file).

[doc/LICENCE](https://github.com/djboni/hdlc/blob/master/doc/LICENSE)
Complete license text.

