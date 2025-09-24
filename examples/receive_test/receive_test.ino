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

void setup() {
  Serial.begin(115200); // Set baud rate

  SPI1.setRX(MISO);
  SPI1.setCS(CS);
  SPI1.setSCK(SCK);
  SPI1.setTX(MOSI);

  radio.begin();
  radio.powerOnReset();

  // Set RF parameter,like frequency,data rate etc
  radio.applyDefaultConfig();
  radio.configureGPIO();

  Serial.printf("Device ID: %04x\n", radio.getDeviceID());

  uint8_t buf[20];

  // frequency adjust
  // frequency will inaccurate if change this parameter
  buf[0] = 98;
  radio.setProperties(RF4463_PROPERTY_GLOBAL_XO_TUNE, 1, buf);

  // tx = rx = 64 byte,PH mode ,high performance mode
  buf[0] = 0x40;
  radio.setProperties(RF4463_PROPERTY_GLOBAL_CONFIG, 1, buf);

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

  // set sync words
  buf[0] = 0x2d;
  buf[1] = 0xd4;
  radio.setSyncWords(buf, 2);

  // set CRC
  buf[0] = RF4463_CRC_SEED_ALL_1S | RF4463_CRC_ITU_T;
  radio.setProperties(RF4463_PROPERTY_PKT_CRC_CONFIG, 1, buf);

  buf[0] = RF4463_CRC_ENDIAN;
  radio.setProperties(RF4463_PROPERTY_PKT_CONFIG1, 1, buf);

  buf[0] = RF4463_IN_FIFO | RF4463_DST_FIELD_ENUM_2;
  buf[1] = RF4463_SRC_FIELD_ENUM_1;
  buf[2] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PKT_LEN, 3, buf);

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

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  radio.setProperties(RF4463_PROPERTY_PKT_FIELD_4_LENGTH_12_8, 8, buf);

  radio.rxInit();
}

uint8_t rx_buf[256];
uint8_t rx_len;

void loop() {
  if (digitalRead(IRQ) == LOW) { // wait INT
    Serial.println("Message received!");
    radio.clearInterrupts();
    rx_len = radio.rxPacket(rx_buf); // read rx data
    Serial.print("Received: ");
    for (size_t i = 0; i < rx_len; i++) {
      Serial.printf("%c", rx_buf[i]);
    }
    radio.rxInit();
  } else {
    Serial.println("No message");
  }

  delay(500);
}