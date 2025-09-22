#include <Arduino.h>
#include <SPI.h>
#include <Si4463.h>

//
// Pins
//

#define LED 25 // Built-in LED on Raspberry Pi Pico
#define SDN 16 // System reset for the SI4463 chip. Active low
#define IRQ 17 // IRQ Pin from the SI4463 to the Pi

#define SCK 10
#define MOSI 11
#define MISO 12
#define CS 13
#define CTS_IRQ 6 // Optional, can be used to detect when CTS goes high

Si4463 radio = Si4463(&SPI1, CS, SDN, IRQ, CTS_IRQ);

void cts() { Serial.println("CTS triggered"); }

void readCommand(uint8_t cmd, size_t len) {
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

  for (size_t i = 0; i < len; i++) {
    Serial.printf("Buf[%d]=%02x\n", i, SPI1.transfer(0xFF));
  }
}

void _init() {

  uint8_t buf[20];

  // frequency adjust
  // frequency will inaccurate if change this parameter
  buf[0] = 98;
  radio.setProperties(RF4463_PROPERTY_GLOBAL_XO_TUNE, 1, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_GLOBAL_XO_TUNE, 1, buf);
  Serial.printf("XO TUNE: %02x\n", buf[0]);

  // tx = rx = 64 byte,PH mode ,high performance mode
  buf[0] = 0x40;
  radio.setProperties(RF4463_PROPERTY_GLOBAL_CONFIG, 1, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_GLOBAL_CONFIG, 1, buf);
  Serial.printf("GLOBAL CONFIG: %02x\n", buf[0]);

  // set preamble
  buf[0] = 0x08; //  8 bytes Preamble
  buf[1] = 0x14; //  detect 20 bits
  buf[2] = 0x00;
  buf[3] = 0x0f;
  buf[4] = RF4463_PREAMBLE_FIRST_1 | RF4463_PREAMBLE_LENGTH_BYTES |
           RF4463_PREAMBLE_STANDARD_1010;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  buf[8] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PREAMBLE_TX_LENGTH, 9, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PREAMBLE_TX_LENGTH, 9, buf);
  Serial.print("PREAMBLE TX LENGTH: ");
  for (size_t i = 0; i < 9; i++) {
    Serial.printf("%02x ", buf[i]);
  }
  Serial.println();

  // set sync words
  buf[0] = 0x2d;
  buf[1] = 0xd4;
  radio.setSyncWords(buf, 2);

  // set CRC
  buf[0] = RF4463_CRC_SEED_ALL_1S | RF4463_CRC_ITU_T;
  radio.setProperties(RF4463_PROPERTY_PKT_CRC_CONFIG, 1, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PKT_CRC_CONFIG, 1, buf);
  Serial.printf("CRC CONFIG: %02x\n", buf[0]);

  buf[0] = RF4463_CRC_ENDIAN;
  radio.setProperties(RF4463_PROPERTY_PKT_CONFIG1, 1, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PKT_CONFIG1, 1, buf);
  Serial.printf("PKT CONFIG1: %02x\n", buf[0]);

  buf[0] = RF4463_IN_FIFO | RF4463_DST_FIELD_ENUM_2;
  buf[1] = RF4463_SRC_FIELD_ENUM_1;
  buf[2] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PKT_LEN, 3, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PKT_LEN, 3, buf);
  Serial.print("PKT LEN: ");
  for (size_t i = 0; i < 3; i++) {
    Serial.printf("%02x ", buf[i]);
  }
  Serial.println();

  // set length of Field 1 -- 4
  // variable len,field as length field,field 2 as data field
  // didn't use field 3 -- 4
  buf[0] = 0x00;
  buf[1] = 0x01;
  buf[2] = RF4463_FIELD_CONFIG_PN_START;
  buf[3] = RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
           RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE;
  buf[4] = 0x00;
  buf[5] = 50;
  buf[6] = RF4463_FIELD_CONFIG_PN_START;
  buf[7] = RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
           RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE;
  ;
  buf[8] = 0x00;
  buf[9] = 0x00;
  buf[10] = 0x00;
  buf[11] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PKT_FIELD_1_LENGTH_12_8, 12, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PKT_FIELD_1_LENGTH_12_8, 12, buf);
  Serial.print("PKT FIELD 1 LENGTH: ");
  for (size_t i = 0; i < 12; i++) {
    Serial.printf("%02x ", buf[i]);
  }
  Serial.println();

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PKT_FIELD_4_LENGTH_12_8, 8, buf);
  memset(buf, 0, sizeof(buf));
  radio.getProperties(RF4463_PROPERTY_PKT_FIELD_4_LENGTH_12_8, 8, buf);
  Serial.print("PKT FIELD 4 LENGTH: ");
  for (size_t i = 0; i < 8; i++) {
    Serial.printf("%02x ", buf[i]);
  }
  Serial.println();

  // set max tx power
  radio.setTxPower(127);
}

void _setup() {
  Serial.begin(115200); // Set baud rate
  // while (!Serial);

  Serial.println("BLAH");

  SPI1.setRX(MISO);
  SPI1.setCS(CS);
  SPI1.setSCK(SCK);
  SPI1.setTX(MOSI);

  Serial.println("BEGIN");

  radio.begin();

  Serial.println("POWER ON RESET");

  radio.powerOnReset();

  delay(1000);

  uint8_t buf[20];

  Serial.println("SET CONFIG");

  // Set RF parameter,like frequency,data rate etc
  radio.setConfig(RF4463_CONFIGURATION_DATA, sizeof(RF4463_CONFIGURATION_DATA));

  Serial.println("CONFIGURE GPIO");

  radio.configureGPIO();

  Serial.println("SET");

  delay(2000);

  _init();
}

void setup() {
  _setup();
  delay(1000);
  radio.checkDevice();
}

void loop() {
  uint8_t buf[] = "Hello World!";
  digitalWrite(LED, HIGH);
  radio.txPacket(buf, sizeof(buf));
  delay(2000);
  digitalWrite(LED, LOW);
  delay(1000);
}