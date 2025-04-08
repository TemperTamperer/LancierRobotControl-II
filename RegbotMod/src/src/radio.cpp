#include "radio.h"
#include "ucontrol.h"
#include "uencoder.h"
#include "urobot.h"

radio::radio(uint8_t csPin, uint8_t irqPin, uint8_t networkId, uint8_t frequency, uint8_t masterNodeId)
    : radioUnit(csPin, irqPin, false), networkId(networkId), frequency(frequency), masterNodeId(masterNodeId) {}

void radio::initialize() {
    Serial.begin(57600);
    nodeId = robot.deviceID;
    radioID = ":" + String(robot.deviceID);
    radioUnit.initialize(frequency, nodeId, networkId);
    
    Serial.println("Radio initialized");
}

void radio::checkForMessages() {
    sendMessage();
    //if (radioUnit.receiveDone()) {
    //    processIncomingMessage();
    //}
}

void radio::processIncomingMessage() {
    if (radioUnit.DATALEN > 0 && radioUnit.DATA != nullptr) {
        message = String((char*)radioUnit.DATA);
        startIndex = message.indexOf(radioID);
        if (startIndex != -1) {
            endIndex = message.indexOf(' ', startIndex);
            if (endIndex == -1) {
                endIndex = message.length();
            }
            segment = message.substring(startIndex, endIndex);
            dIndex = segment.indexOf('D');
            hIndex = segment.indexOf('H');

            if (dIndex != -1 && hIndex != -1) {
                distanceStr = segment.substring(dIndex + 1, hIndex);
                headingStr = segment.substring(hIndex + 1);
                distance = distanceStr.toFloat();
                heading = headingStr.toFloat();

                //Resets robots distance travled and heading angle 
                encoder.distance = 0;
                encoder.pose[2] = 0;
                
                //Assigns recived values to reference values for controllers
                control.mission_pos_ref = distance;
                control.ctrl_turn_ref = heading;
               
                Serial.print("Robot ID: ");
                Serial.println(radioID.substring(1));
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

void radio::sendMessage() {

    Serial.println("Sending");
    char payload[] = "hello from test node";
    if (radioUnit.sendWithRetry(1, payload, sizeof(payload), 3, 200)) {
      Serial.println("ACK received");
    } else {
      Serial.println("No ACK");
    }
  }
