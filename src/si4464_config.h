/**
 * @file si4464_config.h
 * @brief Configuration parameters for the Si4464 radio module.
 */

#ifndef SI4464_CONFIG_H
#define SI4464_CONFIG_H

// 3.2.2. PART_INFO - Reports basic information about the device
#define SI4464_PART_INFO_CMD 0x01

#define SI4464_PART_INFO_CMD_LEN 9

#define SI4464_PART_INFO_CHIPREV_BYTE 0
#define SI4464_PART_INFO_PART1_BYTE 1
#define SI4464_PART_INFO_PART2_BYTE 2 
#define SI4464_PART_INFO_PBUILD_BYTE 3
#define SI4464_PART_INFO_ID1_BYTE 4
#define SI4464_PART_INFO_ID2_BYTE 5
#define SI4464_PART_INFO_CUSTOMER_BYTE 6
#define SI4464_PART_INFO_ROMID_BYTE 7

// 3.2.8. FIFO_INFO - Provides access to transmit and receive fifo counts and
// reset.
#define SI4464_FIFO_INFO_CMD 0x15
#define SI4464_FIFO_INFO_RESET_TX_BIT 1
#define SI4464_FIFO_INFO_RESET_RX_BIT 2

//
#define SI4463_CMD_RX_FIFO_READ 0x77

#endif // SI4464_CONFIG_H