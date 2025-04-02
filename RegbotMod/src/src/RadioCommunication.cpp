#include "RadioCommunication.h"

RadioCommunication::RadioCommunication(uint8_t nodeID, uint8_t networkID, uint8_t frequency, uint8_t spiCsPin, uint8_t irqPin, bool isHighPower)
    : radio(spiCsPin, irqPin, isHighPower), nodeID(nodeID), networkID(networkID), frequency(frequency), spiCsPin(spiCsPin), irqPin(irqPin), isHighPower(isHighPower) {}

void RadioCommunication::initialize() {
    radio.initialize(frequency, nodeID, networkID);
    if (isHighPower) {
        radio.setHighPower(); // Only for RFM69HW/HCW
    }
    Serial.println("Radio initialized");
}

bool RadioCommunication::receiveMessage(String& message) {
    if (radio.receiveDone()) {
        message = String((char*)radio.DATA);
        if (radio.ACKRequested()) {
            radio.sendACK();
        }
        return true;
    }
    return false;
}

void RadioCommunication::processMessage(const String& message, const String& targetID) {
    int startIndex = message.indexOf(targetID);
    if (startIndex != -1) {
        int endIndex = message.indexOf(' ', startIndex);
        if (endIndex == -1) {
            endIndex = message.length();
        }
        String segment = message.substring(startIndex, endIndex);

        Serial.println(segment);

        int dIndex = segment.indexOf('D');
        int hIndex = segment.indexOf('H');

        if (dIndex != -1 && hIndex != -1) {
            String distanceStr = segment.substring(dIndex + 1, hIndex);
            String headingStr = segment.substring(hIndex + 1);

            float distance = distanceStr.toFloat();
            float heading = headingStr.toFloat();

            Serial.print("Robot ID: ");
            Serial.println(targetID.substring(1));
            Serial.print("Distance: ");
            Serial.println(distance);
            Serial.print("Heading: ");
            Serial.println(heading);
        } else {
            Serial.println("Error: Invalid segment format.");
        }
    } else {
        Serial.println("Target ID not found in message.");
    }
}
