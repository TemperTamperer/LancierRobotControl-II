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

float distance;
float heading;

String message;
String segment;

int dIndex;
int hIndex;

String distanceStr;
String headingStr;

void processIncomingMessage();

RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false);

void setup() {
  // Initialize the radio
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  Serial.println("Radio initialized");
}


byte ackCount=0;
uint32_t packetCount = 0;
void loop() {
  // Receive
  if (radio.receiveDone())
  {
    Serial.print("#[");
    Serial.print(++packetCount);
    Serial.print(']');
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    //if (spy) Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    //for (byte i = 0; i < radio.DATALEN; i++)
    //  Serial.print((char)radio.DATA[i]);
    //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 10))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
  }
}

void processIncomingMessage() {
  if (radio.DATALEN > 0 && radio.DATA != nullptr) {
    // Create a String from the incoming dataF
    message = String((char*)radio.DATA);

    // Find the target ID in the message
    int startIndex = message.indexOf(targetID);
    if (startIndex != -1) {
      // Find the position of the next space after the target ID
      int endIndex = message.indexOf(' ', startIndex);
      if (endIndex == -1) {
        endIndex = message.length(); // If no space is found, set to end of message
      }
      segment = message.substring(startIndex, endIndex);

      // Serial.println(segment);

      // Parse the distance and heading values
      dIndex = segment.indexOf('D');
      hIndex = segment.indexOf('H');

      if (dIndex != -1 && hIndex != -1) {
        distanceStr = segment.substring(dIndex + 1, hIndex);
        headingStr = segment.substring(hIndex + 1);

        distance = distanceStr.toFloat();
        heading = headingStr.toFloat();
        
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