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

void cts() {
  Serial.println("CTS triggered");
}

void readCommand(uint8_t cmd, size_t len) {
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
    Serial.printf("Buf[%d]=%02x\n", i, SPI1.transfer(0xFF));
  }
}

void _setup() {
  Serial.begin(115200); // Set baud rate
  while (!Serial);

  SPI1.setRX(MISO);
  SPI1.setCS(CS);
  SPI1.setSCK(SCK);
  SPI1.setTX(MOSI);

  radio.begin();
  radio.powerOnReset();
}

void setup() {
  _setup();


  Serial.println("TEST");
  delay(1000);

  readCommand(0x23, 3);

  delay(1000);

  digitalWrite(CS, LOW);
  uint8_t cd[] = {0x00};
  SPI1.transfer(cd, 1);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);

  delay(1000);

  readCommand(0x01, 6);

  delayMicroseconds(50);

  digitalWrite(CS, LOW);
  uint8_t c[] = {0x014, 0b00001111};
  SPI1.transfer(c, 2);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);

  delayMicroseconds(50);

  digitalWrite(CS, LOW);
  uint8_t def[] = {0x00};
  SPI1.transfer(def, 1);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);

  delayMicroseconds(50);

  readCommand(0x23, 3);

  delayMicroseconds(50);

  delay(5000);

  digitalWrite(CS, LOW);
  uint8_t aasd[] = {0x00};
  SPI1.transfer(aasd, 1);
  delayMicroseconds(40);
  digitalWrite(CS, HIGH);

  delay(5000);

  readCommand(0x01, 6); 
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}