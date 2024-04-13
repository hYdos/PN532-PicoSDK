
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__

#include "PN532Interface.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <pico/binary_info/code.h>
#include <pico/time.h>
#include <string.h>

#define SPI_BUFFER_SIZE 1

uint8_t WRITE_BUFFER[SPI_BUFFER_SIZE];
uint8_t READ_BUFFER[SPI_BUFFER_SIZE];

class PN532_SPI : public PN532Interface
{
public:
    PN532_SPI(spi_inst_t *spi, int rx, int sck, int tx, int csn);

    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);

    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);

private:
    spi_inst_t *_spi;
    int _rx;
    int _sck;
    int _tx;
    int _csn;
    uint8_t command;

    bool isReady();
    void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int8_t readAckFrame();

    inline void write(uint8_t data) {
        WRITE_BUFFER[0] = data;
        spi_write_blocking(_spi, &data, 1);
    };

    inline uint8_t write_read(uint8_t data) {
        WRITE_BUFFER[0] = data;
        spi_write_read_blocking(_spi, &data, READ_BUFFER, 1);
        return READ_BUFFER[0];
    };

    inline uint8_t read() {
        spi_read_blocking(_spi, 0, READ_BUFFER, 1);
        return READ_BUFFER[0];
    }
};

#endif
