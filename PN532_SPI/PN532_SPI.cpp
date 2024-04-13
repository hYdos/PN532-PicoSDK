
#include "PN532_SPI.h"
#include "PN532_debug.h"

#define STATUS_READ 2
#define DATA_WRITE 1
#define DATA_READ 3


PN532_SPI::PN532_SPI(spi_inst_t *spi, int rx, int sck, int tx, int csn) {
    command = 0;
    _spi = spi;
    _rx = rx;
    _sck = sck;
    _tx = tx;
    _csn = csn;
}

void PN532_SPI::begin() {
    spi_init(_spi, 1000 * 5000); // 5Mhz
    gpio_set_function(_rx, GPIO_FUNC_SPI);
    gpio_set_function(_sck, GPIO_FUNC_SPI);
    gpio_set_function(_tx, GPIO_FUNC_SPI);
    gpio_set_function(_csn, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(_rx, _tx, _sck, _csn, GPIO_FUNC_SPI));
}

int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen) {
    command = header[0];
    writeFrame(header, hlen, body, blen);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    WRITE_BUFFER[0] = timeout;

    while (!isReady()) {
        sleep_ms(1);
        timeout--;
        if (0 == timeout) {
            DMSG("Time out when waiting for ACK\n");
            return -2;
        }
    }
    if (readAckFrame()) {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout) {
    uint16_t time = 0;
    while (!isReady()) {
        sleep_ms(1);
        time++;
        if (time > timeout)
        {
            return PN532_TIMEOUT;
        }
    }

    sleep_ms(1);

    int16_t result;
    do {
        uint8_t send[1] = {DATA_READ};
        spi_write_blocking(_spi, send, sizeof(send));
        uint8_t response[3];
        spi_read_blocking(_spi, 0, response, sizeof(response));

        if (0x00 != response[0] || // PREAMBLE
            0x00 != response[1] || // STARTCODE1
            0xFF != response[2]    // STARTCODE2
        ) {
            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t length = read();
        if (0 != (uint8_t)(length + read()))
        { // checksum of length
            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t cmd = command + 1; // response command
        if (PN532_PN532TOHOST != read() || (cmd) != read())
        {
            result = PN532_INVALID_FRAME;
            break;
        }

        DMSG("read:  ");
        DMSG_HEX(cmd);

        length -= 2;
        if (length > len)
        {
            for (uint8_t i = 0; i < length; i++)
            {
                DMSG_HEX(read()); // dump message
            }
            DMSG("\nNot enough space\n");
            read();
            read();
            result = PN532_NO_SPACE; // not enough space
            break;
        }

        uint8_t sum = PN532_PN532TOHOST + cmd;
        for (uint8_t i = 0; i < length; i++)
        {
            buf[i] = read();
            sum += buf[i];

            DMSG_HEX(buf[i]);
        }
        DMSG('\n');

        uint8_t checksum = read();
        if (0 != (uint8_t)(sum + checksum))
        {
            DMSG("checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            break;
        }
        read(); // POSTAMBLE

        result = length;
    } while (0);

    return result;
}

bool PN532_SPI::isReady() {
    uint8_t status = write_read(STATUS_READ) & 1;
    return status;
}

void PN532_SPI::writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen) {
    uint8_t data[] = {DATA_WRITE, PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2};
    spi_write_blocking(_spi, data, sizeof(data));

    write(DATA_WRITE);
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    write(length);
    write(~length + 1); // checksum of length

    write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++) {
        write(header[i]);
        sum += header[i];

        DMSG_HEX(header[i]);
    }
    for (uint8_t i = 0; i < blen; i++) {
        write(body[i]);
        sum += body[i];

        DMSG_HEX(body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);

    DMSG('\n');
}

int8_t PN532_SPI::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};

    uint8_t ackBuf[sizeof(PN532_ACK)];

    sleep_ms(1);
    write(DATA_READ);

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        ackBuf[i] = read();
    }

    return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}
