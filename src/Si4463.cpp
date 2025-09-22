#include <Arduino.h>
#include <SPI.h>

#include "Si4463.h"
#include "radio_config.h"

Si4463::Si4463(SPIClassRP2040 *spi, pin_size_t _cs, pin_size_t sdn,
               pin_size_t irq, pin_size_t cts_irq) {
  _spi = spi;
  _cs = CS;
  _sdn = SDN;
  _irq = IRQ;
  _cts_irq = CTS_IRQ;
}

void cts2() { Serial.println("CTS triggered - from Si4463.cpp"); }

void Si4463::begin() {
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

  while (digitalRead(CTS_IRQ) == LOW) {
  }
}

void Si4463::powerOnReset() {
  digitalWrite(SDN, LOW);

  // Wait for the device to boot properly so it can be
  // accessed via SPI
  while (digitalRead(CTS_IRQ) == LOW) {
  }

  // Send power up command, with XTAL params
  uint8_t tx_buf[] = {0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};
  writeBuf(tx_buf, 7);
}

void Si4463::readBuf(uint8_t *buf, size_t len) {
  uint8_t tx_buf[len];
  memset(tx_buf, 0xFF, len);

  digitalWrite(CS, LOW);
  _spi->transfer(tx_buf, buf, len);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
}

void Si4463::writeBuf(uint8_t *buf, size_t len) {
  digitalWrite(CS, LOW);
  _spi->transfer(buf, len);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
}

void Si4463::setCmd(uint8_t cmd, uint8_t *param, size_t len) {
  uint8_t tx_buf[len + 1];

  tx_buf[0] = cmd;
  memcpy(tx_buf + 1, param, len);

  writeBuf(tx_buf, len + 1);

  // Clear the internal rx buffer in the RF4463
  noOp();
}

void Si4463::getCmd(uint8_t cmd, uint8_t *buf, size_t len) {
  uint8_t tx_buf2[] = {cmd};

  digitalWrite(CS, LOW);
  SPI1.transfer(tx_buf2, 1);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  uint16_t rx;
  uint16_t count = 0;
  while (rx != 0xFF && count++ < 1000000) {
    digitalWrite(CS, LOW);
    rx = SPI1.transfer16(0x44FF);

    if (rx == 0) {
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

  readBuf(buf, len);

  digitalWrite(CS, HIGH);
}

void Si4463::setConfig(uint8_t *parameters, size_t paraLen) {
  // command buf starts with length of command in RADIO_CONFIGURATION_DATA_ARRAY
  uint8_t cmdLen;
  uint8_t command;
  uint16_t pos;
  uint8_t buf[30];

  // power up command had already send
  paraLen = paraLen - 1;
  cmdLen = parameters[0];
  pos = cmdLen + 1;

  while (pos < paraLen) {
    cmdLen = parameters[pos++] - 1;        // get command lend
    command = parameters[pos++];           // get command
    memcpy(buf, parameters + pos, cmdLen); // get parameters

    setCmd(command, buf, cmdLen);
    pos = pos + cmdLen;
  }
}

bool Si4463::checkCTS() {
  uint16_t rx;
  uint16_t count = 0;
  while (rx != RF4463_CTS_REPLY && count < RF4463_CTS_TIMEOUT) {
    digitalWrite(_cs, LOW);
    rx = _spi->transfer16(0x44FF);

    // Si4463 will return 0x00 until it is ready,
    // End the SPI transaction and try again. Also assert _cs
    // if we are at the timeout limit
    if (rx == 0 || count == RF4463_CTS_TIMEOUT - 1) {
      delayMicroseconds(40);
      digitalWrite(_cs, HIGH);
      delayMicroseconds(80);
    }

    // intentionally leave the _cs line low is CTS is received
    // so the app code can immediately read the response
    count++;
  }

  // If rx != RF4463_CTS_REPLY then we timed out
  return rx == RF4463_CTS_REPLY;
}

void Si4463::noOp() {
  uint8_t buf[] = {RF4463_CMD_NOP};
  writeBuf(buf, 1);
}

void Si4463::setSyncWords(uint8_t *syncWords, size_t len) {
  if ((len == 0) || (len > 3))
    return;

  uint8_t buf[5];
  buf[0] = len - 1;

  memcpy(buf + 1, syncWords, len);

  setProperties(RF4463_PROPERTY_SYNC_CONFIG, sizeof(buf), buf);
}

void Si4463::setTxPower(uint8_t power) {
  if (power > 127) // max is 127
    return;

  uint8_t buf[4] = {0x08, 0x00, 0x00, 0x3d};
  buf[1] = power;

  setProperties(RF4463_PROPERTY_PA_MODE, sizeof(buf), buf);
}

bool Si4463::checkDevice() {
  uint8_t buf[9];
  uint16_t partInfo;

  getCmd(RF4463_CMD_PART_INFO, buf, 9); // read part info to check if 4463 works

  partInfo = buf[1] << 8 | buf[2];

  Serial.printf("Part Number: %04x\n", partInfo);
  return partInfo == 0x4463;
}

void Si4463::configureGPIO() {
  uint8_t buf[6];

  // set antenna switch,in RF4463 is GPIO2 and GPIO3
  // don't change setting of GPIO2,GPIO3,NIRQ,SDO
  buf[0] = RF4463_GPIO_INV_CTS;
  buf[1] = RF4463_GPIO_INV_CTS;
  buf[2] = RF4463_GPIO_RX_STATE;
  buf[3] = RF4463_GPIO_TX_STATE;
  buf[4] = RF4463_NIRQ_INTERRUPT_SIGNAL;
  buf[5] = RF4463_GPIO_SPI_DATA_OUT;

  setCmd(RF4463_CMD_GPIO_PIN_CFG, buf, 6);
}

void Si4463::setProperties(uint16_t startProperty, uint8_t length,
                           uint8_t *paraBuf) {

  uint8_t tx_buf[4 + length];
  tx_buf[0] = RF4463_CMD_SET_PROPERTY;
  tx_buf[1] = startProperty >> 8;   // GROUP
  tx_buf[2] = length;               // NUM_PROPS
  tx_buf[3] = startProperty & 0xff; // START_PROP

  memcpy(tx_buf + 4, paraBuf, length);

  writeBuf(tx_buf, length + 4);
}

void Si4463::getProperties(uint16_t startProperty, uint8_t len,
                           uint8_t *paraBuf) {

  uint8_t buf[4];
  buf[0] = RF4463_CMD_GET_PROPERTY;
  buf[1] = startProperty >> 8;   // GROUP
  buf[2] = len;                  // NUM_PROPS
  buf[3] = startProperty & 0xff; // START_PROP

  writeBuf(buf, 4);

  uint8_t rx_buf[len + 2];
  uint8_t tx_buf[len + 2];
  tx_buf[0] = RF4463_CMD_READ_BUF;
  memset(tx_buf + 2, 0xFF, len);

  digitalWrite(CS, LOW);
  _spi->transfer(tx_buf, rx_buf, len + 2);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  memcpy(paraBuf, rx_buf + 2, len);
}

void Si4463::txPacket(uint8_t *sendbuf, uint8_t sendLen) {
  uint16_t txTimer;

  fifoReset();                   // clr fifo
  writeTxFifo(sendbuf, sendLen); // load data to fifo
  setTxInterrupt();
  clrInterrupts(); // clr int factor
  enterTxMode();   // enter TX mode

  txTimer = RF4463_TX_TIMEOUT;
  while (txTimer--) {
    if (digitalRead(IRQ) == LOW) // wait INT
    {
      uint8_t buf[3];
      setCmd(RF4463_CMD_GET_INT_STATUS, buf, sizeof(buf));
      Serial.printf("INT STATUS: %02x %02x %02x\n", buf[0], buf[1], buf[2]);
      Serial.println("Success!");
      return;
    }
    delay(1);
  }

  Serial.println("Timeout waiting for TX interrupt");
}

void Si4463::fifoReset() {
  uint8_t buf[1] = {0x03};
  setCmd(RF4463_CMD_FIFO_INFO, buf, 1); // clr fifo
}

void Si4463::writeTxFifo(uint8_t *databuf, uint8_t length) {
  setProperties(RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length),
                &length);
  uint8_t buf[length + 1];
  buf[0] = length;
  memcpy(buf + 1, databuf, length);
  setCmd(RF4463_CMD_TX_FIFO_WRITE, buf, length + 1);
}

void Si4463::setTxInterrupt() {
  uint8_t buf[3] = {0x01, 0x20, 0x00}; // enable PACKET_SENT interrupt
  setProperties(RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

void Si4463::clrInterrupts() {
  uint8_t buf[] = {0x00, 0x00, 0x00};
  setCmd(RF4463_CMD_GET_INT_STATUS, buf, sizeof(buf));
}

void Si4463::enterTxMode() {
  uint8_t buf[] = {0x00, 0x30, 0x00, 0x00};
  buf[0] = RF4463_FREQ_CHANNEL;
  setCmd(RF4463_CMD_START_TX, buf, sizeof(buf));
}

uint8_t Si4463::rxPacket(uint8_t *recvbuf) {
  uint8_t rxLen;
  rxLen = readRxFifo(recvbuf); // read data from fifo
  fifoReset();                 // clr fifo

  return rxLen;
}

bool Si4463::rxInit() {
  uint8_t length;
  length = 50;
  setProperties(RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length),
                &length); // reload rx fifo size
  fifoReset();            // clr fifo
  setRxInterrupt();
  clrInterrupts(); // clr int factor
  enterRxMode();   // enter RX mode
  return true;
}

void Si4463::enterRxMode() {
  uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
  buf[0] = RF4463_FREQ_CHANNEL;
  setCmd(RF4463_CMD_START_RX, buf, sizeof(buf));
}

void Si4463::setRxInterrupt() {
  uint8_t buf[3] = {0x03, 0x18, 0x00}; // enable PACKET_RX interrupt
  setProperties(RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

uint8_t Si4463::readRxFifo(uint8_t *databuf) {
  // if(!checkCTS()) {
  //   return 0;
  // }
  uint8_t readLen;
  digitalWrite(CS, LOW);

  SPI1.transfer(RF4463_CMD_RX_FIFO_READ);
  readLen = SPI1.transfer(0XFF);

  Serial.println("Read Length: " + String(readLen));

  uint8_t tx_buf[readLen];
  memset(tx_buf, 0xFF, readLen);
  SPI1.transfer(tx_buf, databuf, readLen);

  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  return readLen;
}