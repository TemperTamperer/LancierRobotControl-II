#include "radio.h"

radio::radio(uint8_t csPin, uint8_t irqPin, uint8_t nodeId, uint8_t networkId, uint8_t frequency)
    : radioUnit(csPin, irqPin, false), nodeId(nodeId), networkId(networkId), frequency(frequency), targetID(":90") {}

void radio::initialize() {
    Serial.begin(57600);
    radioUnit.initialize(frequency, nodeId, networkId);
    Serial.println("Radio initialized");
}

void radio::checkForMessages() {
    if (radioUnit.receiveDone()) {
        processIncomingMessage();
    }
}

void radio::processIncomingMessage() {
    if (radioUnit.DATALEN > 0 && radioUnit.DATA != nullptr) {
        String message = String((char*)radioUnit.DATA);
        int startIndex = message.indexOf(targetID);
        if (startIndex != -1) {
            int endIndex = message.indexOf(' ', startIndex);
            if (endIndex == -1) {
                endIndex = message.length();
            }
            String segment = message.substring(startIndex, endIndex);
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
}
