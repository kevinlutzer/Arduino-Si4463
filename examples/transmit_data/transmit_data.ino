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
uint8_t len = 50; // max packet length

static Si4463Property *properties[] = {
    // frequency adjust
    // frequency will inaccurate if change this parameter
    new Si4463Property(RF4463_PROPERTY_GLOBAL_XO_TUNE, (uint8_t[]){98}, 1),
    // tx = rx = 64 byte,PH mode ,high performance mode
    new Si4463Property(RF4463_PROPERTY_GLOBAL_CONFIG, (uint8_t[]){0x40}, 1),
    new Si4463Property(RF4463_PROPERTY_GLOBAL_CONFIG, (uint8_t[]){0x40}, 1),
    new Si4463Property(RF4463_PROPERTY_PREAMBLE_TX_LENGTH,
                       (uint8_t[]){0x08, 0x14, 0x00, 0x0f,
                                   RF4463_PREAMBLE_FIRST_1 |
                                       RF4463_PREAMBLE_LENGTH_BYTES |
                                       RF4463_PREAMBLE_STANDARD_1010,
                                   0x00, 0x00, 0x00, 0x00},
                       9),
    // set CRC
    new Si4463Property(RF4463_PROPERTY_PKT_CRC_CONFIG,
                       (uint8_t[]){RF4463_CRC_SEED_ALL_1S | RF4463_CRC_ITU_T},
                       1),
    new Si4463Property(RF4463_PROPERTY_PKT_LEN,
                       (uint8_t[]){RF4463_IN_FIFO | RF4463_DST_FIELD_ENUM_2,
                                   RF4463_SRC_FIELD_ENUM_1, 0x00},
                       3),
    // set length of Field 1 -- 4
    // variable len,field as length field,field 2 as data field
    // didn't use field 3 -- 4
    new Si4463Property(
        RF4463_PROPERTY_PKT_FIELD_1_LENGTH_12_8,
        (uint8_t[]){
            0x00, 0x01, RF4463_FIELD_CONFIG_PN_START,
            RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
                RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE,
            0x00, 50, RF4463_FIELD_CONFIG_PN_START,
            RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
                RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE,
            0x00, 0x00, 0x00, 0x00},
        12),
    // set packet length
    new Si4463Property(RF4463_PROPERTY_PKT_LEN, &len, sizeof(len)),
};

void setup() {
  Serial.begin(115200); // Set baud rate
  while (!Serial.available())
    ;

  SPI1.setRX(MISO);
  SPI1.setCS(CS);
  SPI1.setSCK(SCK);
  SPI1.setTX(MOSI);

  radio.begin();
  radio.reset();

  // Set RF parameter,like frequency,data rate etc
  radio.applyDefaultConfig();
  radio.configureGPIO();

  Serial.printf("Device ID: %04x\n", radio.getDeviceID());

  for (auto prop : properties) {
    radio.setProperties(prop);
    delete prop;
  }

  // set max tx power
  radio.setTxPower(23);
}

void loop() {
  uint8_t buf[] = "Hello World!";
  digitalWrite(LED, HIGH);
  radio.txPacket(buf, sizeof(buf));
  delay(2000);
  digitalWrite(LED, LOW);
  delay(1000);
}