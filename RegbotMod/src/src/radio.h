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
    String robotID;

    // Member variables to store message processing data
    String message;
    String segment;
    int startIndex;
    int endIndex;
    int dIndex;
    int hIndex;
    String distanceStr;
    String headingStr;
    float distance;
    float heading;
};

#endif
