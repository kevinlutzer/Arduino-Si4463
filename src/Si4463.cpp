#include <Arduino.h>
#include <SPI.h>

#include "Si4463.h"
#include "radio_config.h"

Si4463::Si4463(SPIClassRP2040 *spi, pin_size_t _cs, pin_size_t sdn,
               pin_size_t irq, pin_size_t cts_irq, uint32_t spi_freq) {
  _spi = spi;
  _cs = CS;
  _sdn = SDN;
  _irq = IRQ;
  _cts_irq = CTS_IRQ;

  _spi_settings = SPISettings(spi_freq, MSBFIRST, SPI_MODE0);
}

void Si4463::begin() {
  pinMode(SDN, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(SDN, OUTPUT);
  pinMode(CTS_IRQ, INPUT);

  // Disable the module on boot, we only have to wait
  // 50ms (from the datasheet) before we can enable it
  digitalWrite(SDN, HIGH);

  delay(50);

  this->_spi->begin(false);

  // Begin the transaction. This will setup the internal interrupts
  // within the SPI hardware. Assume that the user of this library
  // will **not** be using the SPI bus for anything else
  this->_spi->beginTransaction(this->_spi_settings);

  digitalWrite(SDN, LOW);

  // Block until we get the CTS_IRQ signal from the GPIO1 pin
  while (digitalRead(CTS_IRQ) == LOW)
    ;
}

void Si4463::reset() {
  digitalWrite(SDN, LOW);

  // Wait for the device to boot properly so it can be
  // accessed via SPI
  while (digitalRead(CTS_IRQ) == LOW)
    ;

  uint8_t tx_buf[] = {RF_POWER_UP};
  this->writeBuf(tx_buf, 7);
  this->noOp();
}

void Si4463::writeBuf(uint8_t *buf, size_t len) {
  digitalWrite(CS, LOW);
  this->_spi->transfer(buf, len);

  // We need to wait an extra 40us as the the transfer function
  // is asynchronous. Adding 40us will ensure that the CS line
  // is deactivated after the last bit is clocked out
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
}

void Si4463::setCmd(uint8_t cmd, uint8_t *param, size_t len) {
  uint8_t tx_buf[len + 1];

  tx_buf[0] = cmd;
  memcpy(tx_buf + 1, param, len);

  this->writeBuf(tx_buf, len + 1);

  // Clear the internal rx buffer in the RF4463
  // We need to do this for every command we send or a
  // subsequent read may pull from the stream of the last command
  this->noOp();
}

bool Si4463::getCmd(uint8_t cmd, uint8_t *buf, size_t len) {
  uint8_t tx_buf2[] = {cmd};

  // Start the SPI transaction to send the command we want to read from
  digitalWrite(CS, LOW);
  this->_spi->transfer(cmd);

  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  // Send the read buf command followed by 0xFF to read the CTS reply
  // note if we send 0x00, that will reset the internal state machine
  uint8_t tx[] = {RF4463_CMD_READ_BUF, 0xFF};
  uint8_t rx[2];
  uint16_t count = 0;

  // We now need to poll the CTS line until we get the CTS reply
  // or we timeout
  while (rx[1] != RF4463_CTS_REPLY && count++ < RF4463_CTS_TIMEOUT) {
    digitalWrite(CS, LOW);

    this->_spi->transfer(tx, rx, sizeof(rx));

    // If we get 0x00 back, the Si4463 is not ready yet
    // to reply. We need to deassert CS and try the whole
    // SPI transaction again
    if (rx[1] == 0) {
      delayMicroseconds(40);
      digitalWrite(CS, HIGH);
      delayMicroseconds(80);
    }
  }

  // Check if we timed out
  if (rx[1] != RF4463_CTS_REPLY) {
    return false;
  }

  uint8_t tx_buf[len];
  memset(tx_buf, 0xFF, len);

  digitalWrite(CS, LOW);
  this->_spi->transfer(tx_buf, buf, len);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);

  return true;
}

void Si4463::applyDefaultConfig() {
  // command buf starts with length of command in RADIO_CONFIGURATION_DATA_ARRAY
  uint8_t *parameters, cmdLen, command, buf[RF4463_CONFIGURATION_DATA_MAX_LEN];
  size_t paraLen;
  uint16_t pos;

  parameters = RF4463_CONFIGURATION_DATA;
  paraLen = sizeof(RF4463_CONFIGURATION_DATA);

  // power up command had already send
  paraLen = paraLen - 1;
  cmdLen = parameters[0];
  pos = cmdLen + 1;

  while (pos < paraLen) {
    cmdLen = parameters[pos++] - 1;        // get command lend
    command = parameters[pos++];           // get command
    memcpy(buf, parameters + pos, cmdLen); // get parameters

    this->setCmd(command, buf, cmdLen);
    pos = pos + cmdLen;
  }
}

bool Si4463::checkCTS() {
  uint16_t rx;
  uint16_t count = 0;
  while (rx != RF4463_CTS_REPLY && count < RF4463_CTS_TIMEOUT) {
    digitalWrite(_cs, LOW);
    rx = this->_spi->transfer16(0x44FF);

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
  this->writeBuf(buf, 1);
}

void Si4463::setSyncWords(uint8_t *syncWords, size_t len) {
  if ((len == 0) || (len > 3))
    return;

  uint8_t buf[5];
  buf[0] = len - 1;

  memcpy(buf + 1, syncWords, len);

  this->setProperties(RF4463_PROPERTY_SYNC_CONFIG, sizeof(buf), buf);
}

void Si4463::setTxPower(uint8_t power) {
  if (power > 127)
    return;

  uint8_t buf[4];

  // Read the existing config to avoid overwriting other PA settings
  this->getProperties(RF4463_PROPERTY_PA_MODE, 4, buf);

  // Modify only the power level
  // PA_PWR_LVL is byte 1, bits 6:0
  buf[1] = power;

  this->setProperties(RF4463_PROPERTY_PA_MODE, sizeof(buf), buf);
}

uint16_t Si4463::getDeviceID() {
  uint8_t buf[9];
  if (!getCmd(RF4463_CMD_PART_INFO, buf, 9)) {
    return 0;
  }

  return buf[1] << 8 | buf[2];
}

void Si4463::configureGPIO() {
  // set antenna switch,in RF4463 is GPIO2 and GPIO3
  // don't change setting of GPIO2,GPIO3,NIRQ,SDO
  uint8_t buf[] = {RF4463_GPIO_INV_CTS,          RF4463_GPIO_INV_CTS,
                   RF4463_GPIO_RX_STATE,         RF4463_GPIO_TX_STATE,
                   RF4463_NIRQ_INTERRUPT_SIGNAL, RF4463_GPIO_SPI_DATA_OUT};

  setCmd(RF4463_CMD_GPIO_PIN_CFG, buf, sizeof(buf));
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
  this->_spi->transfer(tx_buf, rx_buf, len + 2);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  memcpy(paraBuf, rx_buf + 2, len);
}

void Si4463::txPacket(uint8_t *sendbuf, uint8_t sendLen) {
  uint16_t txTimer;

  this->clearFifo(SI4464_FIFO_INFO_RESET_TX_BIT);
  this->writeTxFifo(sendbuf, sendLen); // load data to fifo
  this->setTxInterrupt();
  this->clearInterrupts(); // clr int factor
  this->enterTxMode();     // enter TX mode

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

void Si4463::writeTxFifo(uint8_t *databuf, uint8_t length) {
  this->setProperties(RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length),
                      &length);
  uint8_t buf[length + 1];
  buf[0] = length;
  memcpy(buf + 1, databuf, length);
  setCmd(RF4463_CMD_TX_FIFO_WRITE, buf, length + 1);
}

void Si4463::setTxInterrupt() {
  uint8_t buf[3] = {0x01, 0x20, 0x00}; // enable PACKET_SENT interrupt
  this->setProperties(RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

void Si4463::clearInterrupts() {
  uint8_t buf[] = {0x00, 0x00, 0x00};
  this->setCmd(RF4463_CMD_GET_INT_STATUS, buf, sizeof(buf));
}

void Si4463::enterTxMode() {
  uint8_t buf[] = {0x00, 0x30, 0x00, 0x00};
  buf[0] = RF4463_FREQ_CHANNEL;
  this->setCmd(RF4463_CMD_START_TX, buf, sizeof(buf));
}

uint8_t Si4463::rxPacket(uint8_t *recvbuf) {
  uint8_t rxLen;
  rxLen = this->readRxFifo(recvbuf); // read data from fifo

  this->clearFifo(SI4464_FIFO_INFO_RESET_RX_BIT);

  return rxLen;
}

bool Si4463::rxInit() {
  uint8_t length;
  length = 50;
  this->setProperties(RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length),
                      &length); // reload rx fifo size
  this->clearFifo(SI4464_FIFO_INFO_RESET_RX_BIT);
  this->setRxInterrupt();
  this->clearInterrupts(); // clr int factor
  this->enterRxMode();     // enter RX mode
  return true;
}

void Si4463::enterRxMode() {
  uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
  buf[0] = RF4463_FREQ_CHANNEL;
  this->setCmd(RF4463_CMD_START_RX, buf, sizeof(buf));
}

void Si4463::setRxInterrupt() {
  uint8_t buf[3] = {0x03, 0x18, 0x00}; // enable PACKET_RX interrupt
  this->setProperties(RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

void Si4463::clearFifo(uint8_t fifo_reg) {
  uint8_t buf[] = {1 << fifo_reg};
  setCmd(RF4463_CMD_FIFO_INFO, buf, sizeof(buf)); // clr fifo
}

uint8_t Si4463::readRxFifo(uint8_t *databuf) {
  uint8_t readLen;
  digitalWrite(CS, LOW);

  this->_spi->transfer(RF4463_CMD_RX_FIFO_READ);
  readLen = this->_spi->transfer(0XFF);

  Serial.println("Read Length: " + String(readLen));

  uint8_t tx_buf[readLen];
  memset(tx_buf, 0xFF, readLen);
  this->_spi->transfer(tx_buf, databuf, readLen);

  delayMicroseconds(40);
  digitalWrite(CS, HIGH);
  delayMicroseconds(80);

  return readLen;
}