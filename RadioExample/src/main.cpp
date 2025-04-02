// **********************************************************************************
//
// Test RFM69 Radio.
//
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>          // https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>                // Included with Arduino IDE

// Node and network config
#define NODEID        2    // The ID of this node (must be different for every node on network)
#define NETWORKID     100  // The network ID
#define FREQUENCY      RF69_433MHZ
#define SERIAL_BAUD   57600
#define RF69_SPI_CS   6
#define RF69_IRQ_PIN  33
String targetID = ":90";

void processIncomingMessage();

RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false);

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Initialize the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  Serial.println("Setup complete");
}

// Main loop
unsigned long previousMillis = 0;
const long sendInterval = 3000;



void loop() {
  // Receive
  if (radio.receiveDone()) {
    processIncomingMessage();
    delay(100);
  }
}

void processIncomingMessage() {
  if (radio.DATALEN > 0 && radio.DATA != nullptr) {
    // Create a String from the incoming data
    String message = String((char*)radio.DATA);

    // Find the target ID in the message
    int startIndex = message.indexOf(targetID);
    if (startIndex != -1) {
      // Find the position of the next space after the target ID
      int endIndex = message.indexOf(' ', startIndex);
      if (endIndex == -1) {
        endIndex = message.length(); // If no space is found, set to end of message
      }
      String segment = message.substring(startIndex, endIndex);

      Serial.println(segment);

      // Parse the distance and heading values
      int dIndex = segment.indexOf('D');
      int hIndex = segment.indexOf('H');

      if (dIndex != -1 && hIndex != -1) {
        String distanceStr = segment.substring(dIndex + 1, hIndex);
        String headingStr = segment.substring(hIndex + 1);

        float distance = distanceStr.toFloat();
        float heading = headingStr.toFloat();

        // Output the extracted values
        Serial.print("Robot ID: ");
        Serial.println(targetID.substring(1)); // Remove the colon for display
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