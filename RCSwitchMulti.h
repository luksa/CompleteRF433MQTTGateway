#ifndef _RCSwitchMulti_h
#define _RCSwitchMulti_h

#ifndef RCSwitchMulti_DEBUG_PACKETS
#define RCSwitchMulti_DEBUG_PACKETS false
#endif

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include <wiringPi.h>
    #include <stdint.h>
    #include <stddef.h>
    #define CHANGE 1
#ifdef __cplusplus
extern "C"{
#endif
typedef uint8_t boolean;
typedef uint8_t byte;

#if !defined(NULL)
#endif
#ifdef __cplusplus
}
#endif
#endif


// Number of maximum High/Low changes per packet.
// We can handle up to (unsigned long) => long low pulse + 2 (preamble) + 4 * (2 bit intro + 24 bit learncode + 1 bit low + on/off bit + 4 device index bits) + 1
#define RCSwitchMulti_MAX_CHANGES 1+2+4*(2+24+1+1+4)+1 + 10

#define PROTOCOL_NATSEN 1
#define PROTOCOL_INTERTECHNO 3

class RCSwitchMulti {

  public:
    RCSwitchMulti();
    
    void enableReceive(int interrupt);
    void enableReceive();
    void disableReceive();
    bool available();
    void resetAvailable();

    void handlePulse(unsigned int width);

  
    unsigned long getReceivedValue();
    unsigned int getReceivedBitlength();
    unsigned int getReceivedDelay();
    unsigned int getReceivedProtocol();
    unsigned int* getReceivedRawdata();
  
    void setReceiveTolerance(int nPercent);



  private:
    void handleInterrupt();
    static bool receiveIntertechno(unsigned int changeCount);
    static bool receiveNatsen(unsigned int changeCount);
    int nReceiverInterrupt;

    static int nReceiveTolerance;
    volatile static unsigned long nReceivedValue;
    volatile static unsigned int nReceivedBitlength;
    volatile static unsigned int nReceivedDelay;
    volatile static unsigned int nReceivedProtocol;
    static unsigned int timings[RCSwitchMulti_MAX_CHANGES];
};

#endif
