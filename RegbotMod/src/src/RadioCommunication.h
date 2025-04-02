#ifndef RADIOCOMMUNICATION_H
#define RADIOCOMMUNICATION_H

#include <Arduino.h>
#include <RFM69.h>

class RadioCommunication {
public:
    RadioCommunication(uint8_t nodeID, uint8_t networkID, uint8_t frequency, uint8_t spiCsPin, uint8_t irqPin, bool isHighPower = false);
    void initialize();
    bool receiveMessage(String& message);
    void processMessage(const String& message, const String& targetID);

private:
    RFM69 radio;
    uint8_t nodeID;
    uint8_t networkID;
    uint8_t frequency;
    uint8_t spiCsPin;
    uint8_t irqPin;
    bool isHighPower;
};

#endif // RADIOCOMMUNICATION_H
