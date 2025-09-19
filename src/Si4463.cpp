#include <Arduino.h>
#include <SPI.h>

#include "Si4463.h"



Si4463::Si4463(SPIClassRP2040 * spi, pin_size_t _cs, pin_size_t sdn, pin_size_t irq, pin_size_t cts_irq) {
  _spi = spi;
  _cs = CS;
  _sdn = SDN;
  _irq = IRQ;
  _cts_irq = CTS_IRQ;
}

void cts2() {
  Serial.println("CTS triggered - from Si4463.cpp");
}

void Si4463::begin()
{
  pinMode(SDN, OUTPUT);
  
  pinMode(CS, OUTPUT);
  pinMode(SDN, OUTPUT);
  pinMode(CTS_IRQ, INPUT);

  attachInterrupt(6, cts2, FALLING); // CTS_IRQ

  // Disable the module on boot
  digitalWrite(SDN, HIGH);

  delay(1000);

  SPI1.begin(false);
  SPI1.beginTransaction(SPISettings(32768, MSBFIRST, SPI_MODE0));

  digitalWrite(SDN, LOW);

  while (digitalRead(CTS_IRQ) == LOW){}
}

void Si4463::powerOnReset()
{
  digitalWrite(SDN, LOW);

  // Wait for the device to boot properly so it can be
  // accessed via SPI
  while (digitalRead(CTS_IRQ) == LOW){}

	// Send power up command, with XTAL params
  uint8_t tx_buf[]={0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};
  write(tx_buf, 7);
}

void Si4463::write(uint8_t * buf, size_t len) {
  digitalWrite(CS, LOW);
  _spi->transfer(buf, len);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
}

void Si4463::cmdResp(uint8_t cmd, uint8_t * buf, size_t len) {
  uint8_t tx_buf2[]={cmd};

  digitalWrite(CS, LOW);
  SPI1.transfer(tx_buf2, 1);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  uint16_t rx;
  uint16_t count = 0;
  while(rx != 0xFF && count ++ < 1000000) {
    digitalWrite(CS, LOW);
    rx = SPI1.transfer16(0x44FF);
    
    if (rx == 0){
      delayMicroseconds(40);
      digitalWrite(CS, HIGH);
      delayMicroseconds(80);
    }
  }

  if (rx == 0xFF) {
    Serial.println("CTS received");
  } else {
    Serial.println("Timeout waiting for CTS");
    return;
  }

  for (size_t i = 0; i < len; i++) {
    buf[i] = SPI1.transfer(0xFF);
  }

  digitalWrite(CS, HIGH);
}

bool Si4463::checkCTS() {
  uint16_t rx;
  uint16_t count = 0;
  while(rx != RF4463_CTS_REPLY && count < RF4463_CTS_TIMEOUT) {
    digitalWrite(_cs, LOW);
    rx = _spi->transfer16(0x44FF);
    
    // Si4463 will return 0x00 until it is ready,
    // End the SPI transaction and try again. Also assert _cs
    // if we are at the timeout limit
    if (rx == 0 || count == RF4463_CTS_TIMEOUT - 1){
      delayMicroseconds(40);
      digitalWrite(_cs, HIGH);
      delayMicroseconds(80);
    }

    // intentionally leave the _cs line low is CTS is received
    // so the app code can immediately read the response
    count ++;
  }

  // If rx != RF4463_CTS_REPLY then we timed out
  return rx == RF4463_CTS_REPLY;
}

// void Si4463::cmdResp(uint8_t cmd, uint8_t * buf, size_t len) {
//   uint8_t tx_buf[]={cmd};
//   write(tx_buf, 1);
//   delayMicroseconds(80);
//   if (!checkCTS()) {
//     Serial.println("CTS Timeout");
//   }
//   read(buf, len);
// }

void Si4463::noOp() {
  uint8_t buf[]={RF4463_CMD_NOP};
  write(buf, 1);
}

bool Si4463::checkDevice()
{
	uint8_t buf[9];
	uint16_t partInfo;

	cmdResp(RF4463_CMD_PART_INFO, buf, 9);		// read part info to check if 4463 works

	partInfo=buf[1]<<8|buf[2];

  Serial.printf("Part Number: %04x\n", partInfo);
  return partInfo == 0x4463;
}

