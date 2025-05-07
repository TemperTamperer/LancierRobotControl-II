import threading
import time
import lgpio as GPIO
from RFM69 import Radio, FREQ_315MHZ, FREQ_433MHZ, FREQ_868MHZ, FREQ_915MHZ
import queue
import os
import re
from dancer import Dancer
from generateRobotPath import print_test

nodeId = 1 #1 should be reserved for RPi
networkId = 100 #arbitary as long as consistent with dancers
recipientId = 0 #recipient 0 sends to all nodes on network
board = {'isHighPower': True, 'interruptPin': 18, 'resetPin': 31}
FIFO_PATH = "/tmp/optitrack_data"

dancers = []

route = [[]]
stopPoints = [] 
orientationVectors = []

currentMessage = ""

commandFlag = False
printToggle = False
beginFlag = False
firstToggle = False
danceFlag = False

skipCounter = 0

def help():
    print("Command list")
    print("     :idDdistHhead - Position commad - example: :90D0.5H10 (ping id 90, 0.5 m forward, 10 degrees to right)")
    print("     generate /path - generates path from given svg file. Ex: generate 'SVG/Sine.svg'")
    print("     begin - Starts the dance given in svg file")
    print("     stop - Stops movement of all dancers and waits for start signal")  
    print("     start - Resumes normal function after stop call")
    print("     first - Commands all robots to go back to first positions in a straight line (!BEWARE of collisions)")
    print("     print - Toggle printing outgoing commands")
    print("     maxvel f - Sets the maximum velocity to given float (default 0.5)") 
    #print("     bat - Pings all dancers about their radio status and wait for response to be displayed")

def initDancers():
    global dancers
    
    dancers = []

    data = receiveOptiData()

    pattern = r":(\d+)X(-?\d+\.\d+)Y(-?\d+\.\d+)H(-?\d+\.\d+)"
    matches = re.findall(pattern, data)
    
    for match in matches:
        dancer = Dancer(int(match[0]))
        dancers.append(dancer)
    
    updateDancersPose(data)
    
    for i in range(len(dancers)):
        print(dancers[i])
        
    
    print("Optitrack sees " + str(len(dancers)) + " rigid bodies and dancer objects have been created with Optitrack ID's")
            
def receiveOptiData():
    with open(FIFO_PATH, 'r') as fifo:
        data = fifo.readline().strip()
    return data  

def updateDancersPose(data):
    global dancers

    for i in range(len(dancers)):
        dancers[i].updatePose(data)

def generatePath(svgFilePath):
    global dancers
    route, stopPoints, orientationVectors = print_test("SVG/" + svgFilePath + ".svg")
    for i in range(len(dancers)):
        dancers[i].initPath(route[i])
        print("Generated path with " + str(len(route[i])) + " points")

def updateDancersTarget():
    global dancers
    global skipCounter
    
    if skipCounter > 1:
        skipCounter = 0
        for i in range(len(dancers)):
            dancers[i].nextTarget()
    else:
        skipCounter = skipCounter + 1
    
        
def dancersToFirst():
    global dancers
    global currentMessage
    
    for i in range(len(dancers)):
        dancers[i].goToFirst()

def dancersToTarget():
    global dancers
    global currentMessage
    
    for i in range(len(dancers)):
        currentMessage += dancers[i].poseToTarget()

def printDancers():
    global dancers
    
    for i in range(len(dancers)):
        print(dancers[i])
        
def printCache():
    global dancers
    
    for i in range(len(dancers)):
        print(dancers[i].getCache())


def sendMessage(radio, recipientId):
    global currentMessage
    global commandFlag
    global printToggle
    global danceFlag
    global firstToggle
    global dancers
    
    while True:
        currentMessage = ""
        lastOptiData = receiveOptiData()
        if lastOptiData: #true if anything is in the pipe
            updateDancersPose(lastOptiData)
            if danceFlag:
                updateDancersTarget()
                dancersToTarget()
                print(currentMessage)
                printDancers()
            elif firstToggle:
                dancersToFirst()
                dancersToTarget()
                print(currentMessage)
            
            radio.send(recipientId, currentMessage, attempts=1, waitTime=1, require_ack=False)       
        """
        if printToggle == True:
            currentMessage = ":90D10H0:91D10H0:92D10H0"
            if radio.send(recipientId, currentMessage, attempts=10, waitTime=100, require_ack=False):
                print ("Acknowledgement received")
            print(currentMessage)
            commandFlag = False
        """
        
def inputCommand():
    global currentMessage
    global commandFlag
    global printToggle
    global danceFlag
    global firstToggle
    global dancers
    
    while True:
        newCommand = input("Enter command (help: for list of commands): ")
        if newCommand.casefold() == "help".casefold():
            help()
        elif newCommand.casefold() == "init".casefold():
            initDancers()
        elif "generate" in newCommand:
            generatePath(newCommand.split()[1])
        elif newCommand.casefold() == "dance".casefold():
            danceFlag = True
            for i in range(len(dancers)):
                dancers[i].resetStep
        elif newCommand.casefold() == "stop".casefold():
            #currentMessage = "stop"
            danceFlag = not danceFlag
        elif newCommand.casefold() == "first".casefold():
            firstToggle = not firstToggle
        elif newCommand.casefold() == "print".casefold():
            #printToggle = not printToggle
            #print(printToggle)
            printCache()
        elif newCommand.casefold() == "continue".casefold():
            currentMessage = "continue"
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
