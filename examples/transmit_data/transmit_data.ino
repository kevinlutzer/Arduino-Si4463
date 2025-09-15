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

Si4463 radio = Si4463(&SPI1, CS, SDN, IRQ);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(SDN, OUTPUT);
	pinMode(IRQ, INPUT);

  // Disable the module on boot
  digitalWrite(SDN, HIGH);

  // Initialize the SPI
  SPI1.setRX(MISO);
  SPI1.setCS(CS);
  SPI1.setSCK(SCK);
  SPI1.setTX(MOSI);

  Serial.begin(115200); // Set baud rate
  while (!Serial);   
  Serial.println("Serial started");

  // Init the module
  radio.begin();
  radio.powerOnReset();

  delay(5000);

  // heck if RF4463 works
	if(!radio.checkDevice()){
    Serial.println("Failed");
	} else {
    Serial.println("Success");
  }

  // radio.setProperties(0x0000, 2, (uint8_t[]){0x40, 0x00}); // GLOBAL_XO_TUNE_2

  // set antenna switch,in RF4463 is GPIO2 and GPIO3
	// don't change setting of GPIO2,GPIO3,NIRQ,SDO
  uint8_t buf[6];
  uint8_t buf_rx[6];

	buf[0]  = RF4463_GPIO_NO_CHANGE; 
	buf[1]  = RF4463_GPIO_NO_CHANGE;
	buf[2]  = RF4463_GPIO_RX_STATE;
	buf[3]  = RF4463_GPIO_TX_STATE;
	buf[4]  = RF4463_NIRQ_INTERRUPT_SIGNAL; 
	buf[5]  = RF4463_GPIO_SPI_DATA_OUT; 

	if (!radio.setCommand(6,RF4463_CMD_GPIO_PIN_CFG,buf)) {
    Serial.println("Failed to send command");
  } else {
    Serial.println("Sent the command successfully");
  }

  // delay(2000);

  // if (!radio.getCommand(6, RF4463_CMD_GPIO_PIN_CFG, buf_rx)) {
  //   Serial.println("Failed to get command");
  // }

  // for (int i = 0; i < 6; i ++) {
  //   Serial.printf("Buf[%d] %d, %d \n", i, buf[i], buf_rx[i]);
  // }

}

void loop() {
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}