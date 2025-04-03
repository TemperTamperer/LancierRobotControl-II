#ifndef radio_H
#define radio_H

#include <RFM69.h>
#include <SPI.h>

class radio {
public:
    radio(uint8_t csPin, uint8_t irqPin, uint8_t nodeId, uint8_t networkId, uint8_t frequency);
    void initialize();
    void checkForMessages();

private:
    void processIncomingMessage();
    RFM69 radioUnit;
    uint8_t nodeId;
    uint8_t networkId;
    uint8_t frequency;
    String targetID;
};

#endif
