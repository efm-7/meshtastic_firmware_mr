#pragma once

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
//#include "E7MRadioInterface.h"
#include "E7MRInterface.h"
#include <vector>

/**
 * Our adapter for e7MR radios
 */
class E7MRWrapper : public RadioInterface
//class E7MRWrapper : public E7MRadioInterface
{
  MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);

  public:
    E7MRWrapper();

    //static E7MRWrapper *instance;

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
     bool init() override;

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
     bool reconfigure() override;

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
     bool sleep() override;

    /*
    bool isIRQPending() override { return lora.getIrqFlags() != 0; 
    
    }*/
   
     bool isIRQPending() override;
   

     ErrorCode send(meshtastic_MeshPacket *p) override;

     int mainRadio;

     int numRadios;

     void triggerRx(int radioNo);

  private:
   


    void setTransmitDelay();

    /** random timer with certain min. and max. settings */
    void startTransmitTimer(bool withDelay = true);

    /** timer scaled to SNR of to be flooded packet */
    void startTransmitTimerSNR(float snr);

    void handleTransmitInterrupt();
    
    void handleReceiveInterrupt();

    /** start an immediate transmit
     *  This method is virtual so subclasses can hook as needed, subclasses should not call directly
     *  @return true if packet was sent
     */
     bool startSend(meshtastic_MeshPacket *txp);

    meshtastic_QueueStatus getQueueStatus();

  protected:

    E7MRInterface *a_rIf = NULL;
    E7MRInterface *a_rIf1 = NULL;
    E7MRInterface *a_rIf2 = NULL;
    E7MRLockingArduinoHal *a_RadioLibHAL = NULL;
    E7MRLockingArduinoHal *a_RadioLibHAL1 = NULL;
    E7MRLockingArduinoHal *a_RadioLibHAL2 = NULL;

    std::vector<E7MRLockingArduinoHal *> v_hal;
    //std::vector<E7MRInterface *> v_rIf; // = std::vector<E7MRInterface>(3);
    
    //static std::vector<E7MRInterface *> v_rIf = E7MRInterface::v_rIf;
    std::vector<E7MRInterface *> v_rIf;
    std::vector<E7MRInterface *> v_m7_rIf2;// 
    /**
     * Glue functions called from ISR land
     */
     void disableInterrupt();

    /**
     * Enable a particular ISR callback glue function
     */
    /*
    virtual void enableInterrupt(void (*callback)()) { 
      lora.setDio1Action(callback); 
    }*/

    /** can we detect a LoRa preamble on the current channel? */
     bool isChannelActive();

    /** are we actively receiving a packet (only called during receiving state) */
     bool isActivelyReceiving();

    /**
     * Start waiting to receive a message
     */
     void startReceive();

    /**
     *  We override to turn on transmitter power as needed.
     */
     void configHardwareForSend();

    /**
     * Add SNR data to received messages
     */
     void addReceiveMetadata(meshtastic_MeshPacket *mp);

     void setStandby();

    bool receiveDetected(uint16_t irq, ulong syncWordHeaderValidFlag, ulong preambleDetectedFlag);

     bool cancelSending(NodeNum from, PacketId id) override;

    /** Could we send right now (i.e. either not actively receiving or transmitting)? */
     bool canSendImmediately();

    /**
     * Raw ISR handler that just calls our polymorphic method
     */
    //static void isrRxLevel0();

       /**
     * If a send was in progress finish it and return the buffer to the pool */
    void completeSending();

};
