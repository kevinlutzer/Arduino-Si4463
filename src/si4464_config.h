/**
 * @file SI4463_config.h
 * @brief Configuration parameters for the SI4463 radio module.
 */

#ifndef SI4463_CONFIG_H
#define SI4463_CONFIG_H

// Si4463 null byte. Used for padding commands as 0x00 will cause a
// command reset in the Si4463 state machine
#define SI4463_NULL_BYTE 0xFF

//
// 3.2.2. PART_INFO - Reports basic information about the device
//
#define SI4463_PART_INFO_CMD 0x01
#define SI4463_PART_INFO_CMD_LEN 9
#define SI4463_PART_INFO_CHIPREV_BYTE 0
#define SI4463_PART_INFO_PART1_BYTE 1
#define SI4463_PART_INFO_PART2_BYTE 2
#define SI4463_PART_INFO_PBUILD_BYTE 3
#define SI4463_PART_INFO_ID1_BYTE 4
#define SI4463_PART_INFO_ID2_BYTE 5
#define SI4463_PART_INFO_CUSTOMER_BYTE 6
#define SI4463_PART_INFO_ROMID_BYTE 7

//
// 3.2.6. GPIO_PIN_CFG - Configures the GPIO pins.
//
#define SI4463_GPIO_PIN_CFG_CMD 0x13
#define SI4463_GPIO_PIN_CFG_CMD_LEN 7
#define SI4463_GPIO_PIN_CFG_GPIO_0_BYTE 0
#define SI4463_GPIO_PIN_CFG_GPIO_1_BYTE 1
#define SI4463_GPIO_PIN_CFG_GPIO_2_BYTE 2
#define SI4463_GPIO_PIN_CFG_GPIO_3_BYTE 3
#define SI4463_GPIO_PIN_CFG_NIRQ_BYTE 4
#define SI4463_GPIO_PIN_CFG_SDO_BYTE 5
#define SI4463_GPIO_PIN_CFG_GEN_CONFIG_BYTE 6
// Note that some of these mode codes maybe different pin to pin
// consult the datasheet for details.
#define SI4463_GPIO_MODE_NO_CHANGE 0
#define SI4463_GPIO_MODE_DISABLED 1
#define SI4463_GPIO_MODE_LOW 2
#define SI4463_GPIO_MODE_HIGH 3
#define SI4463_GPIO_MODE_INPUT 4
#define SI4463_GPIO_MODE_32_KHZ_CLOCK 5
#define SI4463_GPIO_MODE_BOOT_CLOCK 6
#define SI4463_GPIO_MODE_DIVIDED_MCU_CLOCK 7
#define SI4463_GPIO_MODE_CTS 8
#define SI4463_GPIO_MODE_INV_CTS 9
#define SI4463_GPIO_MODE_HIGH_ON_CMD_OVERLAP 10
#define SI4463_GPIO_MODE_SPI_DATA_OUT 11
#define SI4463_GPIO_MODE_NIRQ_INTERRUPT_SIGNAL 39
#define SI4463_GPIO_MODE_TX_STATE 32
#define SI4463_GPIO_MODE_RX_STATE 33

//
// 3.2.8. FIFO_INFO - Provides access to transmit and receive fifo counts and
// reset.
//
#define SI4463_FIFO_INFO_CMD 0x15
#define SI4463_FIFO_INFO_RESET_TX_BIT 1
#define SI4463_FIFO_INFO_RESET_RX_BIT 2

//
// 3.5.36. SYNC_CONFIG
//
#define SI4463_SYNC_CONFIG_PROP 0x1100
#define SI4463_SYNC_CONFIG_PROP_LEN 5

//
// 3.5.104. PA_MODE - PA operating mode and groups.
//

#define SI4463_PA_MODE_PROP 0x2200
#define SI4463_PA_MODE_PROP_LEN 4
#define SI4463_PA_MODE_PROP_PWL_LEVEL_BYTE 1

#endif // SI4463_CONFIG_H