#include <Arduino.h>
#include <SPI.h>

#include "Si4463.h"

Si4463::Si4463(SPIClassRP2040 * spi, pin_size_t cs, pin_size_t sdn, pin_size_t irq, pin_size_t cts_irq) {
    _spi = spi;
    _cs = cs;
    _sdn = sdn;
    _irq = irq;
    _cts_irq = cts_irq;
}

void Si4463::begin()
{
    pinMode(_cs, OUTPUT);
    pinMode(_sdn, OUTPUT);
	pinMode(_irq, INPUT);

    digitalWrite(_sdn, HIGH);

    _spi->begin(true);
    _spi->beginTransaction(SPISettings(32768, MSBFIRST, SPI_MODE0));
}

void Si4463::powerOnReset()
{
  digitalWrite(_sdn, LOW);
  delay(1000);						// wait for RF4463 stable

	// send power up command
  uint8_t tx_buf[]={0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};
  _spi->transfer(tx_buf, 7);

  delay(200);
}

bool Si4463::checkCTS()
{
  uint16_t timeOutCnt;
    timeOutCnt=RF4463_CTS_TIMEOUT;

  Serial.println("Send request");

  uint8_t rx = 0;
  uint16_t count = 0;
  do {
    digitalWrite(_cs, LOW);
    rx = _spi->transfer(RF4463_CMD_READ_BUF);
    if (rx == RF4463_CTS_REPLY) {
      digitalWrite(_cs, LOW);
      break;
    }
    _spi->transfer(0XFF);
    // Serial.printf("CTS=%02x, %d\n", rx, digitalRead(_cts_irq));
    count ++;
  } while ( rx != RF4463_CTS_REPLY && count < RF4463_CTS_TIMEOUT );

  return rx == RF4463_CTS_REPLY;
}

bool Si4463::getCommand2(uint8_t length, uint8_t command, uint8_t* paraBuf) {
  uint8_t * _txbuf = (uint8_t * ) malloc(length * sizeof(uint8_t));
  memset(_txbuf, 0x00, length);

  _spi->transfer(_txbuf, paraBuf, length);		// read parameters
  digitalWrite(_cs, HIGH);

  free(_txbuf);
  return true;
}

bool Si4463::getCommand(uint8_t length, uint8_t command, uint8_t * paraBuf) {

    uint8_t rx = _spi->transfer(command);
    delayMicroseconds(1);

    if(!checkCTS())	{
        return false;  
    }

    uint8_t * _txbuf = (uint8_t * ) malloc(length * sizeof(uint8_t));
    memset(_txbuf, 0xFF, length);

    _spi->transfer(_txbuf, paraBuf, length);		// read parameters
    digitalWrite(_cs, HIGH);

    free(_txbuf);
    return true;
}

bool Si4463::checkDevice()
{
	uint8_t buf[9];
	uint16_t partInfo;

	getCommand(9, RF4463_CMD_PART_INFO, buf);		// read part info to check if 4463 works
		
  for (int i = 0; i < 9; i ++) {
    Serial.printf("Buf[%d]=%d\n", i, buf[i]);
  }

	partInfo=buf[1]<<8|buf[2];
	return partInfo == 0x4463;
}

bool Si4463::setCommand(uint8_t length, uint8_t command, uint8_t* paraBuf)
{
  uint8_t * tx_buf = (uint8_t * ) malloc((length + 1) * sizeof(uint8_t));
  tx_buf[0] = command;   		// COMMAND
  memcpy(tx_buf + 1, paraBuf, length);

  _spi->transfer(tx_buf, length + 1);

  free(tx_buf);
	return true;
}
