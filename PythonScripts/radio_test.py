"""
import time
import lgpio as GPIO
from RFM69 import Radio, FREQ_315MHZ, FREQ_433MHZ, FREQ_868MHZ, FREQ_915MHZ
import queue


node_id = 1
network_id = 100
recipient_id = 0
board = {'isHighPower': True, 'interruptPin': 15, 'resetPin': 31}

with Radio(FREQ_433MHZ, node_id, network_id, verbose=True, **board) as radio:
    print ("Starting loop...")
    time.sleep(10)
    while True:
        print ("Sending")
        if radio.send(recipient_id, ":90D1H50 :91D1H50", attempts=3, waitTime=100, require_ack=True):
            print ("Acknowledgement received")
        else:
            print ("No Acknowledgement")
        
        # print("recieveing")
        # packet = radio.get_packet()
        # if packet:
            # print(f"Received packet from node {packet.sender}: {packet.message}")
        # else:
            # print("Nothing was received")
	 
"""        

import threading
import time
import lgpio as GPIO
from RFM69 import Radio, FREQ_315MHZ, FREQ_433MHZ, FREQ_868MHZ, FREQ_915MHZ
import queue
import os
import re
from dancer import Dancer

nodeId = 1
networkId = 100
recipientId = 0
board = {'isHighPower': True, 'interruptPin': 15, 'resetPin': 31}
FIFO_PATH = "/tmp/optitrack_data"

dancer1 = Dancer(90)

initToggle = False

currentMessage = ""
lastOptiData = ""

commandFlag = False
printToggle = False
beginFlag = False


def help():
    print("Command list")
    print("     :idDdistHhead - Position commad - example: :90D0.5H10 (ping id 90, 0.5 m forward, 10 degrees to right)")
    print("     begin - Starts the dance given in svg file")
    print("     stop - Stops movement of all dancers and waits for start signal")  
    print("     start - Resumes normal function after stop call")
    print("     first - Commands all robots to go back to first positions in a straight line (!BEWARE of collisions)")
    print("     print - Toggle printing outgoing commands")
    print("     maxvel f - Sets the maximum velocity to given float (default 0.5)") 
    #print("     bat - Pings all dancers about their radio status and wait for response to be displayed")

def initDancers(data):
    global dancers
    
    dancers = []
    dancerCount = 0
    pattern = r":(\d+)X(-?\d+\.\d+)Y(-?\d+\.\d+)H(-?\d+\.\d+)"
    matches = re.findall(pattern, data)
    
    for match in matches:
        dancer = Dancer(int(match[0]))
        dancer.updatePose(data)
        dancers.append(dancer)
        dancerCount += 1
        print(dancer)
    
    print("Optitrack sees " + str(dancerCount) + " rigid bodies")
    print(str(len(dancers)) + " dancer objects have been created with Optitrack id's")
        
            
        
        
    

def sendMessage(radio, recipientId):
    global currentMessage
    global commandFlag
    global printToggle
    global lastOptiData
    
    """
    while True:
        #print ("Sending")
        if commandFlag == True:
            if printToggle:
                print (currentMessage)
            if radio.send(recipientId, currentMessage, attempts=100, waitTime=100, require_ack=True):
                print ("Acknowledgement received")
            commandFlag = False
                
    """
    
    with open(FIFO_PATH, 'r') as fifo: 
        print("fifo Initilzed")
        while True:
            lastOptiData = fifo.readline().strip()  # Blocks until data arrives
            if lastOptiData:
                dancer1.updatePose(lastOptiData)
                target = [-4 ,4]
                dancer1.updateTarget(target)
                currentMessage = dancer1.poseToTarget()
                print(currentMessage)
                radio.send(recipientId, currentMessage, attempts=1, waitTime=1, require_ack=False)
                time.sleep(0)
                initDancers(lastOptiData)
                #print(dancer1)
                #print(f"Received from OptiTrack: {lastOptiData}")  # Print raw data
                #print();
                    
            #print ("Sending")
            if commandFlag == True:
                if printToggle:
                    print (currentMessage)
                if radio.send(recipientId, currentMessage, attempts=10, waitTime=100, require_ack=True):
                    print ("Acknowledgement received")
                commandFlag = False
    
        
def inputCommand():
    global currentMessage
    global commandFlag
    global printToggle
    while True:
        newCommand = input("Enter command (help: for list of commands): ")
        if newCommand.casefold() == "help".casefold():
            help()
        elif newCommand.casefold() == "print".casefold():
            printToggle = not printToggle
        elif newCommand.casefold() == "stop".casefold():
            currentMessage = "stop"
            commandFlag = True
        elif newCommand.casefold() == "start".casefold():
            currentMessage = "start"
            commandFlag = True
        else:
            currentMessage = '"' + newCommand + '"'
            commandFlag = True

def main():
    with Radio(FREQ_433MHZ, nodeId, networkId, verbose=True, **board) as radio:

        if not os.path.exists(FIFO_PATH):
            os.mkfifo(FIFO_PATH)
        
        print ("Initilizing threads")
        # Create and start the sender thread
        senderThread= threading.Thread(target=sendMessage, args=(radio, recipientId), daemon=True)
        senderThread.start()

        # Create and start the input listener thread
        inputThread = threading.Thread(target=inputCommand, daemon=True)
        inputThread.start()

        # Wait for threads to complete (they actually run indefinitely)
        senderThread.join()
        inputThread.join()
        
        while True:
            time.sleep(1)
        
if __name__ == "__main__":
    main()



        
        
        
        
"""       
        
# pylint: disable=missing-function-docstring,redefined-outer-name
import time
import lgpio as GPIO
import asyncio
from aiohttp import ClientSession
from RFM69 import Radio, FREQ_433MHZ

async def call_API(url, packet):
    async with ClientSession() as session:
        print("Sending packet to server")
        async with session.post(url, json=packet.to_dict('%c')) as response:
            response = await response.read()
            print("Server responded", response)

async def receiver(radio):
    while True:
        print("Receiver")
        for packet in radio.get_packets():
            print("Packet received", packet.to_dict())
            await call_API("http://httpbin.org/post", packet)
        await asyncio.sleep(10)

async def send(radio, to, message):
    print ("Sending")
    if radio.send(to, message, attempts=3, waitTime=100):
        print ("Acknowledgement received")
    else:
        print ("No Acknowledgement")

async def pinger(radio):
    print("Pinger")
    counter = 0
    while True:
        await send(radio, 2, "ping {}".format(counter))
        counter += 1
        await asyncio.sleep(5)


loop = asyncio.get_event_loop()
node_id = 1
network_id = 100
recipient_id = 2
board = {'isHighPower': True, 'interruptPin': 40, 'resetPin': 31}

with Radio(FREQ_433MHZ, node_id, network_id, verbose=True, **board) as radio:
    print ("Started radio")
    loop.create_task(receiver(radio))
    loop.create_task(pinger(radio))
    loop.run_forever()

loop.close()
"""
