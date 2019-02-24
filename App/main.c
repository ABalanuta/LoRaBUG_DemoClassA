
/* XDCtools Header files */

#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

#include <driverlib/sys_ctrl.h> // SysCtrlSystemReset()

/* Board Header files */
#include "Board_LoRaBUG.h"

#include <string.h> // strlen in uartputs and LoRaWan code
#include <math.h>

#include "Commissioning.h"

#include "board.h"
#include "io.h"

#include "LoRaMac.h"

#include <ti/sysbios/hal/Seconds.h>
#include <time.h>

#include "cc.h" //command and control libraries
#include "raw_lora.h"

#include "driverlib/aon_wuc.h"



#define TASKSTACKSIZE   2048

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* Runtime Events */
#define EVENT_STATECHANGE Event_Id_00
static Event_Struct runtimeEventsStruct;
static Event_Handle runtimeEvents;

/*------------------------------------------------------------------------*/
/*                      Start of LoRaWan Demo Code                        */
/*------------------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 15s, value in [ms].
 */
//#define APP_TX_DUTYCYCLE                            60000
volatile static Uint32 APP_TX_DUTYCYCLE = 48*60*60*1000;

/*!
 * Defines a random delay for application data transmission duty cycle. 2s,
 * value in [ms].
 */
//#define APP_TX_DUTYCYCLE_RND                        0000
volatile static Uint32 APP_TX_DUTYCYCLE_RND = 0;
/*!
 * Default datarate
 */
//#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_DEFAULT_DATARATE                    DR_4

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            100

/*!
 * User application data buffer size BAND_915
 */
#define LORAWAN_APP_DATA_SIZE                       9

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
static TimerEvent_t Led4Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;



struct ExperimentTest_s
{
    bool Running;
    bool ClockInicialized;
    Uint32 DutyCycle;
    Uint8 CommandReceived;

    Uint32 ExecTime;
    Uint16 Nmessages;
    Uint32 freq;
    Uint16 bw;
    Uint8 sf;
    Uint8 payloadSize;

}ExperimentTest;

static Int32 timeToExec = 0;
static Int32 lastTime = -1;


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    static uint32_t counter = 0;
    uint32_t batteryVoltage = 0;
    uint8_t	batteryLevel = 0;
    AppDataSize = 0;

    uartprintf("# PrepareTxFrame for Port %d\r\n", port);

    switch( port )
    {
    case 2:
    batteryVoltage = BoardGetBatteryVoltage();
    batteryLevel = BoardGetBatteryLevel();

    memset(AppData, '\0', sizeof(AppData));

    // Copy Counter
    memcpy(AppData, &counter, sizeof(counter));
    AppDataSize += sizeof(counter);
    counter++;

    // Copy Battery Voltage
    memcpy(AppData + AppDataSize, &batteryVoltage, sizeof(batteryVoltage));
    AppDataSize += sizeof(batteryVoltage);

    // Copy Battery Level
    memcpy(AppData + AppDataSize, &batteryLevel, sizeof(batteryLevel));
    AppDataSize += sizeof(batteryLevel);

    break;

    // Default Experiment Port
    case 100:
        // Default Init Clock Request
        if ( !ExperimentTest.ClockInicialized ) {
            Seconds_set((xdc_UInt32)0);
            uartputs("### Requesting Time !");
         }
        AppDataSize = 1;
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    //printf("# SendFrame\n");

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    //printf("# OnTxNextPacketTimerEvent\n");
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
//    GpioWrite( &Led1, 1 );
    setLed(Board_GLED, 0);
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
//    GpioWrite( &Led2, 1 );
    setLed(Board_RLED, 0);
}

/*!
 * \brief Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void )
{
    TimerStop( &Led4Timer );
    // Switch LED 4 OFF
//    GpioWrite( &Led4, 1 );
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    //printf("# McpsConfirm\n");
    //uartputs("# McpsConfirm");
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {

        uartprintf("# McpsConfirm ReqType:%d UpCounter:%d \r\n", mcpsConfirm->McpsRequest, mcpsConfirm->UpLinkCounter);

        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                //uartputs("# Got McpsConfirm: MCPS_UNCONFIRMED\n");

                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                //uartputs("# Got McpsConfirm: MCPS_CONFIRMED");
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
//        GpioWrite( &Led1, 0 );
        setLed(Board_GLED, 1);
        TimerStart( &Led1Timer );
    }
    else{
        uartprintf("!! McpsConfirm LoRaMAC %d Error\r\n", mcpsConfirm->Status);
        //DeviceState = DEVICE_STATE_JOIN;
        //NextTx = false;
    }

    NextTx = true;
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    //uartputs("# McpsIndication");
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        uartprintf("!! Got McpsIndication Error, Event:%d\r\n",mcpsIndication->Status);
        return;
    }

    uartprintf("### Got McpsIndication Type:%d  \r\n",mcpsIndication->McpsIndication);
    uartprintf("### Received %d bytes on Port %d \r\n", mcpsIndication->BufferSize, mcpsIndication->Port);
    if (mcpsIndication->BufferSize > 0) uarthexdump(mcpsIndication->Buffer, mcpsIndication->BufferSize);

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            uartputs("# Got McpsIndication: MCPS_UNCONFIRMED");
            break;
        }
        case MCPS_CONFIRMED:
        {
            uartputs("# Got McpsIndication: MCPS_CONFIRMED");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            uartputs("# Got McpsIndication: MCPS_PROPRIETARY");
            break;
        }
        case MCPS_MULTICAST:
        {
            uartputs("# Got McpsIndication: MCPS_MULTICAST");
            break;

        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    UInt32 tempUInt32 = 0;
    UInt16 tempUInt16 = 0;


    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
//                GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
                setLed(Board_RLED, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0);
            }
            break;

        // Set the Current Time and send it back
        case 100:
            memcpy(&tempUInt32, mcpsIndication->Buffer, sizeof(tempUInt32));
            uartprintf("### Seconds %d \r\n", tempUInt32);
            Seconds_set((xdc_UInt32)tempUInt32);
            ExperimentTest.ClockInicialized = true;
            print_clock();

            //enable Time Printing
            setTimePrint(true);

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            tempUInt32 = Seconds_get();
            memcpy(AppData + AppDataSize, &tempUInt32, sizeof(tempUInt32));
            AppDataSize += sizeof(tempUInt32);


            NextTx = SendFrame( );

            break;

        // Received a ping must ack
        case 101:
            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            // Add RSSI
            memcpy(AppData + AppDataSize, &(mcpsIndication->Rssi), 2);
            AppDataSize += 2;
            // Add SNR
            memcpy(AppData + AppDataSize, &(mcpsIndication->Snr), 1);
            AppDataSize += 1;

            NextTx = SendFrame( );
            break;

        // Change Update Rate
        case 102:
            memcpy(&tempUInt32, mcpsIndication->Buffer, sizeof(tempUInt32));
            uartprintf("### UpdateRate is now %d seconds \r\n", tempUInt32);

            ExperimentTest.DutyCycle = tempUInt32 * 1000;
            //ExperimentTest.Running = true;
            APP_TX_DUTYCYCLE = tempUInt32 * 1000;

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            memcpy(AppData + AppDataSize, &tempUInt32, sizeof(tempUInt32));
            AppDataSize += sizeof(tempUInt32);

            NextTx = SendFrame( );

            TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
            DeviceState = DEVICE_STATE_CYCLE;
            break;

        // Device
        case 103:
            break;

        // Receive Experiment Config
        case 104:

            if (mcpsIndication->BufferSize != 11) {
                uartputs("### Invalid Config Size, Aborting ...");
                break;
            }

            memcpy(&ExperimentTest.ExecTime, mcpsIndication->Buffer, sizeof(ExperimentTest.ExecTime));
            memcpy(&ExperimentTest.Nmessages, mcpsIndication->Buffer + 4 , sizeof(ExperimentTest.Nmessages));
            memcpy(&tempUInt16, mcpsIndication->Buffer + 6, sizeof(tempUInt16));

            ExperimentTest.freq = 900000000 + (Uint32)(tempUInt16)*100000;
            ExperimentTest.bw = *(mcpsIndication->Buffer + 8) * 25;
            ExperimentTest.payloadSize = *(mcpsIndication->Buffer + 9);
            ExperimentTest.sf = *(mcpsIndication->Buffer + 10);

            uartputs(  "### Config Settings");
            uartprintf("#### Experiment Starts at: %s\r", getTimeStrFromSeconds(ExperimentTest.ExecTime));
            uartprintf("#### # Messages: %d\r\n",ExperimentTest.Nmessages);
            uartprintf("#### PayloadSize: %d\r\n",ExperimentTest.payloadSize);
            uartprintf("#### Freq: %d\r\n",ExperimentTest.freq);
            uartprintf("#### BW: %d\r\n",ExperimentTest.bw);
            uartprintf("#### SF: %d\r\n",ExperimentTest.sf);

            ExperimentTest.Running = 0;

            // Update Time to exec
            if((ExperimentTest.ExecTime - Seconds_get()) > 0)
                timeToExec = ExperimentTest.ExecTime - Seconds_get();

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            NextTx = SendFrame( );
            break;

        // Start Experiment
        case 105:
            uartputs("### Received Start Experiment Command");
            uartprintf("#### Experiment Starts at: %s\r", getTimeStrFromSeconds(ExperimentTest.ExecTime));
            uartprintf("#### Now is at: %s\r", getTimeStrFromSeconds(Seconds_get()));
            ExperimentTest.Running = 1;

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            NextTx = SendFrame( );
            break;

        // Cancel Experiment
        case 106:
            uartputs("### Received Cancel Experiment Command");
            ExperimentTest.Running = 0;

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            NextTx = SendFrame( );
            break;

        // Reboot Device
        case 107:
            uartputs("### Received Cancel Experiment Command");
            ExperimentTest.Running = 0;

            uartputs("### Sending Ack");
            AppDataSize = 0;
            ExperimentTest.CommandReceived = mcpsIndication->Port;
            memset(AppData, '\0', sizeof(AppData));
            memcpy(AppData + AppDataSize, &ExperimentTest.CommandReceived, sizeof(ExperimentTest.CommandReceived));
            AppDataSize += sizeof(ExperimentTest.CommandReceived);

            NextTx = SendFrame( );

            DelayMs(1000);
            SysCtrlSystemReset();
            break;


        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
//    GpioWrite( &Led2, 0 );
    setLed(Board_RLED, 1);
    TimerStart( &Led2Timer );

    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    uartputs("# MlmeConfirm");
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            //printf("# MlmeConfirm: Join\n");
            uartputs("# MlmeConfirm: Join");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
                uartputs("# MlmeConfirm: Joined");
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            //printf("# MlmeConfirm: Link Check\n");
            uartputs("# MlmeConfirm: Link Check");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

void maintask(UArg arg0, UArg arg1)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );
    uartputs("# Board initialized");

    // Construct event for power efficient operation
    Event_construct(&runtimeEventsStruct, NULL);
    runtimeEvents = Event_handle(&runtimeEventsStruct);

    // Todo Remove
    ExperimentTest.Running = true;
    ExperimentTest.ExecTime = Seconds_get() + 10;

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                //printf("# DeviceState: DEVICE_STATE_INIT\n");
                uartputs("# DeviceState: DEVICE_STATE_INIT");
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, 25 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 25 );

                TimerInit( &Led4Timer, OnLed4TimerEvent );
                TimerSetValue( &Led4Timer, 25 );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                // Change device to Class C
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_C;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                //printf("# DeviceState: DEVICE_STATE_JOIN\n");
                uartputs("# DeviceState: DEVICE_STATE_JOIN");

                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
//                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                //printf("# DeviceState: DEVICE_STATE_SEND\n");
                uartputs("# DeviceState: DEVICE_STATE_SEND");

                //process();


                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );
                    NextTx = SendFrame( );
                }

                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                DeviceState = DEVICE_STATE_CYCLE;

                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                uartputs("# DeviceState: DEVICE_STATE_CYCLE");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                //uartputs("# DeviceState: DEVICE_STATE_SLEEP");
                // Wake up through events
//                TimerLowPowerHandler( );
                //Task_sleep(TIME_MS * 10);
                //Task_yield();
                //Event_pend(runtimeEvents, Event_Id_NONE, EVENT_STATECHANGE, BIOS_WAIT_FOREVER);
                //timeToExec = ExperimentTest.ExecTime - Seconds_get();

                //while (tempUInt32 > 0){

                if ( ExperimentTest.Running ){

                    timeToExec = ExperimentTest.ExecTime - Seconds_get();

                    if (timeToExec > 0 && timeToExec != lastTime) {
                        lastTime = timeToExec;
                        uartprintf("### Starting in %d Seconds \r\n", (Int32)timeToExec);
                    }
                    else if(timeToExec <= 0){
                        uartprintf("Booom !!! \r\n");

                        //TODO do Stuff
                        raw_test();

                        uartprintf("Done, Rebooting !!! \r\n");

                        // Then Reboot
                        //HWREGBITW(AON_WUC_BASE+ AON_WUC_O_JTAGCFG, 0x08 ) = 0;//
                        //```HWREG(AON_WUC_BASE + AON_WUC_O_JTAGCFG) = 0;```
                        AONWUCJtagPowerOff();
                        DelayMs(500);
                        SysCtrlSystemReset();
                    }

                }
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }

}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    // Board_initI2C();
    Board_initSPI();
    Board_initUART();
    // Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr) maintask, &taskParams,
                   NULL);

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
