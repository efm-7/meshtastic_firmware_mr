#include "E7MRWrapper.h"
#include "E7MRInterface.h"
#include "MeshService.h"
#include "sleep.h"

E7MRWrapper::E7MRWrapper()
{
   //instantiate all radios
    //instance = this;
    mainRadio = 0; 
    numRadios = 3;
    pinMode(SX126X_CS, OUTPUT);
    pinMode(SX126X_1_CS, OUTPUT);
    pinMode(SX126X_2_CS, OUTPUT);
    digitalWrite(SX126X_CS, HIGH);
    digitalWrite(SX126X_1_CS, HIGH);
    digitalWrite(SX126X_2_CS, HIGH);

    SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);
    
    a_RadioLibHAL = new E7MRLockingArduinoHal(SPI, spiSettings);
    a_RadioLibHAL1 = new E7MRLockingArduinoHal(SPI, spiSettings);
    a_RadioLibHAL2 = new E7MRLockingArduinoHal(SPI, spiSettings);

    a_rIf2 = new E7MRInterface(a_RadioLibHAL2, "Radio2", SX126X_2_CS, SX126X_2_DIO1, SX126X_2_RESET, SX126X_2_BUSY);
    a_rIf1 = new E7MRInterface(a_RadioLibHAL1, "Radio1", SX126X_1_CS, SX126X_1_DIO1, SX126X_1_RESET, SX126X_1_BUSY);
    a_rIf = new E7MRInterface(a_RadioLibHAL, "Radio0", SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);

    v_rIf= {a_rIf, a_rIf1, a_rIf2};

    //enableLoraInterrupt();
    //v_hal[mainRadio] = new E7MRLockingArduinoHal(SPI, spiSettings);
    //v_rIf[mainRadio] = new E7MRInterface(v_hal[mainRadio], "Radio 0", SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().

bool E7MRWrapper::init()
{
    bool rtVl = true;

    for(std::vector<E7MRInterface>::size_type i = 0; i != v_rIf.size(); i++) {
        
        if(i != mainRadio){
            v_rIf[i]->disableTx();
        }else{
            v_rIf[i]->enableTx();
        }
        v_rIf[i]->enableRx();
        //v_rIf[i]->disableTx();
        if(!v_rIf[i]->init()) rtVl = false;
    }


    LOG_DEBUG("*********************** E7MRWrapper::init() ***********************");
    return rtVl;
}

bool E7MRWrapper::reconfigure()
{
    bool rtVl = true;
    for(E7MRInterface *i : v_rIf){
        if(!i->reconfigure()) rtVl = false;
    }
    LOG_DEBUG("*********************** E7MRWrapper::reconfigure() ***********************");
    return rtVl;
}

bool E7MRWrapper::sleep()
{
    bool rtVl = true;
    for(E7MRInterface *i : v_rIf){
        if(!i->sleep()) rtVl = false;
    }
    LOG_DEBUG("***********************  E7MRWrapper::sleep *********************** ");
    return rtVl;
}

bool E7MRWrapper::isIRQPending()
{
    bool rtVl = false;
    for(E7MRInterface *i : v_rIf){
        if(i->isIRQPending()) rtVl = true;
    }
    LOG_DEBUG("*********************** E7MRWrapper::isIRQPending *********************** ");
    return rtVl;
}

void E7MRWrapper::disableInterrupt()
{
    LOG_DEBUG("*********************** E7MRWrapper::disableInterrupt *********************** ");
    for(E7MRInterface *i : v_rIf){
        i->disableInterrupt();
    }
}

void E7MRWrapper::setStandby()
{
    LOG_DEBUG("***********************  E7MRWrapper::configHardwareForSend *********************** ");
    for(E7MRInterface *i : v_rIf){
        i->setStandby();
    }
}

/**
 * Add SNR data to received messages
 */
void E7MRWrapper::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    LOG_DEBUG("*********************** E7MRWrapper::disableInterrupt ***********************");
    for(E7MRInterface *i : v_rIf){
        i->addReceiveMetadata(mp);
    }
}

/** We override to turn on transmitter power as needed.
 */
void E7MRWrapper::configHardwareForSend()
{
    LOG_DEBUG("*********************** E7MRWrapper::configHardwareForSend ***********************");
    v_rIf[mainRadio]->configHardwareForSend();

    //not ready for this yet
    /*
    for(E7MRInterface *i : v_rIf){
        i->configHardwareForSend(); 
    }*/
}

// For power draw measurements, helpful to force radio to stay sleeping
// #define SLEEP_ONLY

void E7MRWrapper::startReceive()
{
    LOG_DEBUG("*********************** E7MRWrapper::startReceive ***********************");
    for(E7MRInterface *i : v_rIf){
        i->startReceive(); 
    }
}

/** Is the channel currently active? */
bool E7MRWrapper::isChannelActive()
{
    bool rtVl = false;

    for(E7MRInterface *i : v_rIf){
        if(i->isChannelActive()) rtVl = true;
    }
    LOG_DEBUG("*********************** E7MRWrapper::isChannelActive ***********************");
    return rtVl;
}

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
bool E7MRWrapper::isActivelyReceiving()
{
    bool rtVl = false;
    // The IRQ status will be cleared when we start our read operation. Check if we've started a header, but haven't yet
    // received and handled the interrupt for reading the packet/handling errors.
    //########################## TODO: consolidate responses from all radios
    //return receiveDetected(lora0.getIrqFlags(), RADIOLIB_SX126X_IRQ_HEADER_VALID, RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
    //rtVl = v_rIf[mainRadio]->isActivelyReceiving();
    for(E7MRInterface *i : v_rIf){
        if(i->isActivelyReceiving()) rtVl = true;
    }
    LOG_DEBUG("*********************** E7MRWrapper::isActivelyReceiving ***********************");
    return rtVl;
}

//##################


/// Send a packet (possibly by enquing in a private fifo).  This routine will
/// later free() the packet to pool.  This routine is not allowed to stall because it is called from
/// bluetooth comms code.  If the txmit queue is empty it might return an error
ErrorCode E7MRWrapper::send(meshtastic_MeshPacket *p)
{
    LOG_DEBUG("*********************** E7MRWrapper::send ***********************");
    ErrorCode rtVl = 0;

    for(std::vector<E7MRInterface>::size_type i = 0; i != v_rIf.size(); i++) {
        if(i != mainRadio){
            v_rIf[i]->disableInterrupt();
        }else{
            rtVl = v_rIf[i]->send(p);
        }
    }
    return rtVl;
}

void E7MRWrapper::setTransmitDelay()
{
    LOG_DEBUG("*********************** E7MRWrapper::setTransmitDelay ***********************");
    v_rIf[mainRadio]->setTransmitDelay();
}

void E7MRWrapper::startTransmitTimer(bool withDelay)
{
    LOG_DEBUG("*********************** E7MRWrapper::startTransmitTimer ***********************");
    v_rIf[mainRadio]->startTransmitTimer();
}

void E7MRWrapper::startTransmitTimerSNR(float snr)
{
    LOG_DEBUG("*********************** E7MRWrapper::startTransmitTimerSNR ***********************");
    v_rIf[mainRadio]->startTransmitTimerSNR(snr);
}

void E7MRWrapper::handleTransmitInterrupt()
{
    LOG_DEBUG("*********************** E7MRWrapper::handleTransmitInterrupt ***********************");
   v_rIf[mainRadio]->handleTransmitInterrupt();
}

void E7MRWrapper::handleReceiveInterrupt()
{
    LOG_DEBUG("*********************** E7MRWrapper::handleReceiveInterrupt ***********************");
    for(E7MRInterface *i : v_rIf){
        i->handleTransmitInterrupt(); 
    }
}

/** start an immediate transmit */
bool E7MRWrapper::startSend(meshtastic_MeshPacket *txp)
{
    bool rtVl = false;
    rtVl = v_rIf[mainRadio]->startSend(txp);
    LOG_DEBUG("*********************** E7MRWrapper::startSend ***********************");
    return rtVl;
}

meshtastic_QueueStatus E7MRWrapper::getQueueStatus()
{
    meshtastic_QueueStatus qs;

    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue.getFree();
    qs.maxlen = txQueue.getMaxLen();
    LOG_DEBUG("*********************** E7MRWrapper::getQueueStatus ***********************");
    return qs;
}

bool E7MRWrapper::receiveDetected(uint16_t irq, ulong syncWordHeaderValidFlag, ulong preambleDetectedFlag)
{
    bool rtVl = false;
    for(E7MRInterface *i : v_rIf){
        if(i->receiveDetected(irq, syncWordHeaderValidFlag, preambleDetectedFlag)) rtVl = true;
        LOG_DEBUG("*********************** E7MRWrapper::receiveDetected (%d) for (%s) is (%d)  ***********************", irq, i->moduleName, rtVl);
    }
    
    return rtVl;
}
bool E7MRWrapper::cancelSending(NodeNum from, PacketId id)
{
    bool rtVl = false;
    rtVl = v_rIf[mainRadio]->cancelSending(from,id);

    LOG_DEBUG("*********************** E7MRWrapper::cancelSending ***********************");
    return rtVl;
}

bool E7MRWrapper::canSendImmediately()
{
   bool rtVl = false;
   rtVl = v_rIf[mainRadio]->canSendImmediately();
   LOG_DEBUG("*********************** E7MRWrapper::canSendImmediately ***********************");
    return rtVl;
}
/*
void E7MRWrapper::isrRxLevel0()
{
    for(E7MRInterface *i : v_rIf){
        
    }
}
*/
void E7MRWrapper::completeSending()
{
    LOG_DEBUG("*********************** E7MRWrapper::completeSending ***********************");
    v_rIf[mainRadio]->completeSending();
}

void E7MRWrapper::triggerRx(int radioNo){
    v_rIf[radioNo]->handleReceiveInterrupt();
}
