/**
 * @file si4464_config.h
 * @brief Configuration parameters for the Si4464 radio module.
 */

#ifndef SI4464_CONFIG_H
#define SI4464_CONFIG_H

// 3.2.8. FIFO_INFO - Provides access to transmit and receive fifo counts and
// reset.
#define SI4464_FIFO_INFO_CMD 0x15
#define SI4464_FIFO_INFO_RESET_TX_BIT 1
#define SI4464_FIFO_INFO_RESET_RX_BIT 2

#endif // SI4464_CONFIG_H