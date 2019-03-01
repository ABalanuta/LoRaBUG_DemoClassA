/*
 * raw_lora.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

/* XDCtools Header files */
#include <xdc/std.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>

/* LoRa Radio Header files */
#include "board.h" // The LoRaMac-node/src/boads/LoRaBug/board.h file
#include "radio.h"

#include "io.h"

/* Event handle and constants */
//#define EVENT_TXDONE    Event_Id_20
//#define EVENT_TXTIMEOUT Event_Id_21
//static Event_Struct eventsStruct;
//static Event_Handle eventsHandler;


/* Radio callback handlers */
//static void TxDone() {
//    Event_post(events, EVENT_TXDONE);
//}
//static void TxTimeout() {
//    Event_post(events, EVENT_TXTIMEOUT);
//}
//static const RadioEvents_t RadioEventHandlers = {
//                                                 .TxDone    = TxDone,
//                                                 .TxTimeout = TxTimeout,
//};



/* Radio Settings */
//#define RF_CH                                       2          // Ch
//#define RF_FREQUENCY                                902.3e6 + (RF_CH - 1) * 200e3 // Hz (915MHz band is 902e3 to 928e3)
//#define RF_FREQUENCY                                903100000

#define TX_OUTPUT_POWER                             20          // dBm
#define LORA_BANDWIDTH                              0           // [0: 125 kHz,
                                                                //  1: 250 kHz,
                                                                //  2: 500 kHz,
                                                                //  3: Reserved]

#define LORA_SPREADING_FACTOR                       9           // [SF7..SF12]
#define LORA_CODINGRATE                             1           // [1: 4/5,
                                                                //  2: 4/6,
                                                                //  3: 4/7,
                                                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8           // Same for Tx and Rx (uint16_t)
#define LORA_SYMBOL_TIMEOUT                         5           // Symbols - Up to 1023 symbols (for RX)
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define CRC_ON                                      true
#define TRANSMISSION_TIMEOUT                        3000        // ms

static const uint8_t  myZeroes[] = "000000000000000000000000000000000000000000000000000000000000000";
const uint32_t rf_ch_1[] = {902.3e6, 902.5e6, 902.7e6, 902.9e6,
                            903.1e6, 903.3e6, 903.5e6, 903.7e6};


void raw_init(uint32_t freq){

}

//static void OnTxDone(void) {
//    uartprintf("OnTxDone\r\n");
//    Event_post(eventsHandler, EVENT_TXDONE);
//}
//
//static void OnTxTimeout(void) {
//    uartprintf("OnTxTimeout\r\n");
//    Event_post(eventsHandler, EVENT_TXTIMEOUT);
//}
//
//static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
//    uartprintf("OnRxDone\r\n");
//}
//
//static void OnRxTimeout(void) {
//    uartprintf("OnRxTimeout\r\n");
//}
//
//static void OnRxError(void) {
//    uartprintf("OnRxError\r\n");
//}
//
//static void OnCadDone( bool channelActivityDetected) {
//    uartprintf("OnCadDone\r\n");
//}

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
              Bool iqInverted)
{

    Radio.Sleep();

    switch(bandwidth){
    case 125:
        bandwidth = 0;
        break;
    case 250:
        bandwidth = 1;
        break;
    case 500:
        bandwidth = 2;
        break;
    default:
        bandwidth = 0;
        break;
    }


    /*!
     * Radio events function pointer
     */
//    static RadioEvents_t RadioEvents;
//
//    // Radio initialization
//    RadioEvents.TxDone = OnTxDone;
//    RadioEvents.RxDone = OnRxDone;
//    RadioEvents.TxTimeout = OnTxTimeout;
//    RadioEvents.RxTimeout = OnRxTimeout;
//    RadioEvents.RxError = OnRxError;
//    RadioEvents.CadDone = OnCadDone;

    //radios->Init(&RadioEvents);

    //BoardDeInitMcu();

    //BoardInitMcu();
    //BoardInitPeriph();
    //DisableLoraTask();
    //StopLoraTask();


//    Radio.Init(&RadioEvents);
    Radio.SetChannel(freq); // Must set channel before SetTxConfig

    Radio.SetTxConfig(MODEM_LORA,
                      tx_power,
                      0, // Frequency Deviation (FSK only)
                      bandwidth,
                      datarate,
                      coderate,
                      preambleLen,
                      false,
                      crcOn,
                      false, // FreqHopOn
                      0,     // HopPeriod
                      iqInverted,
                      TRANSMISSION_TIMEOUT);

//    Radio.Write( REG_LR_PAYLOADMAXLENGTH, 0xFF);
//    uartprintf("Set Large Payload Params\r\n");
//    uartprintf("Payload Length = %u\r\n", Radio.Read( REG_LR_PAYLOADLENGTH ));
//    uartprintf("Max Payload = %u\r\n", Radio.Read( REG_LR_PAYLOADMAXLENGTH ));

    for (int r = nMessages; r != 0; r--){
        setLed(Board_RLED, Board_LED_ON);
        //uartprintf("#### Loop # %d/%d\r", r, nMessages);
        Radio.Send((uint8_t *)myZeroes, payloadSize);

        // Wait for TX_DONE
        while (!PIN_getInputValue(Board_SX_DIO0)) { } //  Wait For trigger
        //Event_pend(eventsHandler, Event_Id_NONE, EVENT_TXDONE | EVENT_TXTIMEOUT, BIOS_WAIT_FOREVER);

        Radio.Sleep();
        setLed(Board_RLED, Board_LED_OFF);

        DELAY_MS(delay);
    }

    Radio.Sleep();
    uartprintf("#### Done \r\n");

}


//void raw_test(){
//
//    setLed(Board_GLED, Board_LED_ON);
//
//    //UInt key = Task_disable(); // Disables Task Scheduling
//
//    //DisableLoraTask();
//
//    //BoardDeInitMcu();
//    //PIN_close(sxSpiSleepPinHandle);
//    //SpiInit( &SX1276.Spi, (PinNames)Board_SX_MOSI, (PinNames)Board_SX_MISO, (PinNames)Board_SX_SCK, (PinNames)NC );
//    //SX1276IoInit( );
//
//    //SPI_close(SPI_Handle handle);
//    //BoardInitMcu( );
//    //DisableLoraTask();
//
//    //Radio.Init((RadioEvents_t *)&RadioEventHandlers);
//    //Radio.SetPublicNetwork(true);
//
//    static int nMax = 10000;
//
//    for (int n = 0; n < nMax; n++){
//        uartprintf("#### Loop # %d/%d\r", n, nMax);
//        for (int i = 0; i < 8; i++){
//            Radio.SetChannel(rf_ch_1[i]); // Must set channel before SetTxConfig
//            Radio.SetTxConfig(MODEM_LORA,
//                              TX_OUTPUT_POWER,
//                              0, // Frequency Deviation (FSK only)
//                              LORA_BANDWIDTH,
//                              LORA_SPREADING_FACTOR,
//                              LORA_CODINGRATE,
//                              LORA_PREAMBLE_LENGTH,
//                              LORA_FIX_LENGTH_PAYLOAD_ON,
//                              CRC_ON,
//                              false, // FreqHopOn
//                              0,     // HopPeriod
//                              LORA_IQ_INVERSION_ON,
//                              TRANSMISSION_TIMEOUT);
//            for (int r = 0; r < 1; r++){
//                setLed(Board_RLED, Board_LED_ON);
//                Radio.Send((uint8_t *)msg, sizeof(msg)-1);
//                Radio.Sleep();
//                DELAY_MS(100);
//                setLed(Board_RLED, Board_LED_OFF);
//                DELAY_MS(100);
//            }
//        }
//    }
//
//}


//void raw_test(){
//
//    setLed(Board_GLED, Board_LED_ON);
//
//    //UInt key = Task_disable(); // Disables Task Scheduling
//
//
//    Radio.Init((RadioEvents_t *)&RadioEventHandlers);
//    Radio.SetPublicNetwork(true);
//    while(1){
//        for (int i = 0; i < 8; i++){
//            Radio.SetChannel(rf_ch_1[i]); // Must set channel before SetTxConfig
//
//            Radio.SetTxConfig(MODEM_LORA,
//                              TX_OUTPUT_POWER,
//                              0, // Frequency Deviation (FSK only)
//                              LORA_BANDWIDTH,
//                              LORA_SPREADING_FACTOR,
//                              LORA_CODINGRATE,
//                              LORA_PREAMBLE_LENGTH,
//                              LORA_FIX_LENGTH_PAYLOAD_ON,
//                              CRC_ON,
//                              false, // FreqHopOn
//                              0,     // HopPeriod
//                              LORA_IQ_INVERSION_ON,
//                              TRANSMISSION_TIMEOUT);
//            for (int r = 0; r < 1; r++){
//
//                setLed(Board_RLED, Board_LED_ON);
//
//                Radio.Send((uint8_t *)msg, sizeof(msg)-1);
//                //UInt e = Event_pend(events, Event_Id_NONE, EVENT_TXDONE|EVENT_TXTIMEOUT, BIOS_WAIT_FOREVER);
//                Radio.Sleep();
//
//            //                switch (e) {
//            //                case EVENT_TXDONE:
//            //                    // Green
//            //                    setLed(Board_RLED, Board_LED_OFF);
//            //                    break;
//            //                case EVENT_TXTIMEOUT:
//            //                    setLed(Board_RLED, Board_LED_ON);
//            //                    setLed(Board_GLED, Board_LED_ON);
//            //                    Event_pend(events, Event_Id_NONE, Event_Id_NONE, BIOS_WAIT_FOREVER);
//            //                    break;
//            //                default:
//            //                    break;
//            //                }
//
//            DELAY_MS(500);
//            setLed(Board_RLED, Board_LED_OFF);
//            DELAY_MS(1000);
//        }
//        //DelayMs(2);
//        //UInt e = Event_pend(events, Event_Id_NONE, EVENT_TXDONE|EVENT_TXTIMEOUT, BIOS_WAIT_FOREVER);
//        //Radio.Sleep();
//
//        //        switch (e) {
//        //        case EVENT_TXDONE:
//        //            // Green
//        //            setLed(Board_RLED, Board_LED_OFF);
//        //            break;
//        //        case EVENT_TXTIMEOUT:
//        //            setLed(Board_RLED, Board_LED_ON);
//        //            setLed(Board_GLED, Board_LED_ON);
//        //            Event_pend(events, Event_Id_NONE, Event_Id_NONE, BIOS_WAIT_FOREVER);
//        //            break;
//        //        default:
//        //            break;
//        //        }
//
//        //DELAY_MS(200);
//    }
//    //Event_pend(events, Event_Id_NONE, Event_Id_NONE, BIOS_WAIT_FOREVER);
//    //}
//
//    //Task_restore(key); // Re-enables Task Scheduling
//}

