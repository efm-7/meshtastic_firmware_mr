#pragma once

#include "E7MR-RadioLibInterface.h"

/**
 * Our adapter for e7MR radios
 */
class E7MRInterface : public E7MRadioLibInterface
{
  
  public:
    E7MRInterface(E7MRLockingArduinoHal *hal, String moduleName, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                    RADIOLIB_PIN_TYPE busy);

    /** Our ISR code currently needs this to find our active instance
     */
    //E7MRInterface *instance;

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool init();// override;

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool reconfigure();// override;

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    bool sleep();// override;

    /*
    bool isIRQPending() override { return lora.getIrqFlags() != 0; 
    
    }*/
    String moduleName;

    //bool enableTX;

    bool isIRQPending(){
      return lora.getIrqFlags() != 0; 
      LOG_DEBUG("::::::::::: E7MRInterface::isIRQPending");
    }

    /**
     * Glue functions called from ISR land
     */
    virtual void disableInterrupt() override;

    virtual void setStandby() override;

        /**
     * Enable a particular ISR callback glue function
     */
    //virtual void enableInterrupt(void (*callback)()) { lora.setDio1Action(callback); }
    virtual void enableInterrupt(void (*callback)());

    /** can we detect a LoRa preamble on the current channel? */
    virtual bool isChannelActive() override;

    /** are we actively receiving a packet (only called during receiving state) */
    virtual bool isActivelyReceiving() override;

    /**
     * Start waiting to receive a message
     */
    virtual void startReceive() override;

    /**
     *  We override to turn on transmitter power as needed.
     */
    virtual void configHardwareForSend() override;

    /**
     * Add SNR data to received messages
     */
    virtual void addReceiveMetadata(meshtastic_MeshPacket *mp) override;

    //static std::vector<E7MRLockingArduinoHal *> v_hal;
    //static std::vector<E7MRInterface *> v_m7_rIf; // = std::vector<E7MRInterface>(3);


  protected:
  	float currentLimit = 140; // Higher OCP limit for SX126x PA

    /**
     * Specific module instance
     */
    SX1262 lora;


};
