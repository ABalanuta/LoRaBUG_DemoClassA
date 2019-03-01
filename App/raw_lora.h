/*
 * raw_lora.h
 *
 *  Created on: Sep 9, 2018
 *      Author: Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#ifndef APP_RAW_LORA_H_
#define APP_RAW_LORA_H_

void raw_init();
void raw_test();
void raw_lora_t(Uint16 nMessages,
              Uint32 freq,
              Uint16 bandwidth,
              Uint8  datarate,
              Uint8  payloadSize,
              Uint16 delay,
              Uint8 preambleLen,
              Uint8 coderate,
              Int8 tx_power,
              Bool crcOn,
              Bool iqInverted);

#endif /* APP_RAW_LORA_H_ */
