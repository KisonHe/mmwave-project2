#!/usr/bin/python3
# -*- coding: UTF-8 -*-

# Hardware and system
import serial
import time
import logging

# Threadding and queue
import threading
import queue

# Math
import numpy as np

# # Read config
# import yaml

# User lib
from door import door

# Set things up

# yamlfile = open("config.yml", "r")
# data = yaml.load(yamlfile, Loader=yaml.FullLoader)

DPLOT = True
if (DPLOT):
    import cv2
    # import pyqtgraph as pg
    # from pyqtgraph.Qt import QtGui

logging.basicConfig(level=logging.INFO)
# logging.basicConfig(level=logging.WARNING,filename='/home/kisonhe/Downloads/photos/test.log', filemode='w')
# main_door = door(-2.5,3.5,2.5,3.5,DPLOT)
main_door = door(2.2,4.0,0.3,4.0,DPLOT)
q = queue.Queue()
q.maxsize = 1


def takePhotoThread():
    global q
    import os
    import cv2
    from datetime import datetime
    directory = '/home/kisonhe/Downloads/photos'
    os.chdir(directory)
    cap = cv2.VideoCapture("/dev/video6")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    while(1):
        ret, frame = cap.read()
        try:
            status = q.get_nowait()
        except Exception as e: # queue is empty
            pass
        else:   # we got queue!
            if (status > 0):
                tmpstr = "进门"
            else:
                tmpstr = "出门"
            logging.warning("Taking photo of " + tmpstr + "!")
            filename = str(datetime.now().strftime("IMG_%Y%m%d_%H%M%S")) + tmpstr + ".jpg"
            cv2.imwrite(filename, frame)
            # cv2.imshow("Suspect", frame)
            # k = cv2.waitKey(1)

# Change the configuration file name
configFileName = '/home/kisonhe/githubPrjects/mmwave-project2/mmw_pplcount_demo_default.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0


# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports

    CLIport = serial.Serial('/dev/ttyACM0', 115200)
    Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        numTxAnt = 2
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            digOutSampleRate = int(splitWords[11])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters 
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData16xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    MMWDEMO_UART_MSG_POINT_CLOUD_2D = 6
    MMWDEMO_UART_MSG_TARGET_LIST_2D = 7
    MMWDEMO_UART_MSG_TARGET_INDEX_2D = 8
    maxBufferSize = 2**15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    targetLengthInBytes = 68
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    targetDetected = 0 # Checks if a person has been detected
    frameNumber = 0
    targetObj = {}
    pointObj = {}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
    
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
    
        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)
    
        # Check that startIdx is not empty
        if startIdx:
    
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
    
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
    
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[20:20 + 4], word)
    
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
        # Initialize the pointer index
        idX = 0
    
        # Read the header
        idX += 20
        totalPacketLen = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 24
    
        word = [1, 2 ** 8]
    
        numTLVs = np.matmul(byteBuffer[idX:idX + 2], word)
        idX += 4
    
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
        # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
            
            # Initialize the tlv type
            tlv_type = 0
    
            try: 
                # Check the header of the TLV message
                tlv_type = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                tlv_length = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
            except:
                pass
    
            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_POINT_CLOUD_2D:
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
                # Calculate the number of detected points
                numInputPoints = (tlv_length - tlvHeaderLengthInBytes) // pointLengthInBytes
    
                # Initialize the arrays
                rangeVal = np.zeros(numInputPoints, dtype=np.float32)
                azimuth = np.zeros(numInputPoints, dtype=np.float32)
                dopplerVal = np.zeros(numInputPoints, dtype=np.float32)
                snr = np.zeros(numInputPoints, dtype=np.float32)
    
                for objectNum in range(numInputPoints):
                    # Read the data for each object
                    rangeVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    azimuth[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    dopplerVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    snr[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
    
                    # Store the data in the detObj dictionary
                pointObj = {"numObj": numInputPoints, "range": rangeVal, "azimuth": azimuth,\
                            "doppler": dopplerVal, "snr": snr}
            
                dataOK = 1
    
            elif tlv_type == MMWDEMO_UART_MSG_TARGET_LIST_2D:
    
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
                # Calculate the number of target points
                numTargetPoints = (tlv_length - tlvHeaderLengthInBytes) // targetLengthInBytes
    
                # Initialize the arrays
                targetId = np.zeros(numTargetPoints, dtype=np.uint32)
                posX = np.zeros(numTargetPoints, dtype=np.float32)
                posY = np.zeros(numTargetPoints, dtype=np.float32)
                velX = np.zeros(numTargetPoints, dtype=np.float32)
                velY = np.zeros(numTargetPoints, dtype=np.float32)
                accX = np.zeros(numTargetPoints, dtype=np.float32)
                accY = np.zeros(numTargetPoints, dtype=np.float32)
                EC = np.zeros((3, 3, numTargetPoints), dtype=np.float32)  # Error covariance matrix
                G = np.zeros(numTargetPoints, dtype=np.float32)  # Gain
    
                for objectNum in range(numTargetPoints):
                # Read the data for each object
                    targetId[objectNum] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                    posX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    posY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    accX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    accY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    G[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
    
                # Store the data in the detObj dictionary
                targetObj = {"targetId": targetId, "posX": posX, "posY": posY, \
                             "velX": velX, "velY": velY, "accX": accX, "accY": accY, \
                             "EC": EC, "G": G, "numTargets":numTargetPoints}
                
                targetDetected = 1
    
            elif tlv_type == MMWDEMO_UART_MSG_TARGET_INDEX_2D:
                # Calculate the length of the index message
                numIndices = tlv_length - tlvHeaderLengthInBytes
                indices = byteBuffer[idX:idX + numIndices]
                idX += numIndices
    
    
        # Remove already processed data
        if idX > 0:
            shiftSize = totalPacketLen
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
    
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, targetDetected, frameNumber, targetObj, pointObj

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
    global main_door
    dataOk = 0
    targetDetected = 0
    global targetObj
    global pointObj
    x = []
    y = []
      
    # Read and parse the received data
    dataOk, targetDetected, frameNumber, targetObj, pointObj = readAndParseData16xx(Dataport, configParameters)
    
    if targetDetected:
        # print(targetObj)
        # print(targetObj["numTargets"])
        x = -targetObj["posX"]
        y = targetObj["posY"]
        logging.info("x:"+str(x)+"y:"+str(y)+'\r')
        if (DPLOT):
            cv2.imshow("Kison's mmwave Radar",main_door.drawPeople(x,y))
            cv2.waitKey(1)
        # TODO:Hook here
        # result = someone_at_door()
        result = main_door.takePhoto(x,y)
        if (result != 0):
            try:
                q.put_nowait(result)
            except Exception as e:
                logging.error("Fail to put queue:"+str(e))

        logging.info(str(result))
        pass
    
    return dataOk


# -------------------------    MAIN   -----------------------------------------  

# start photo taking thread
t1 = threading.Thread(target=takePhotoThread)
t1.start()
# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

if (DPLOT):
    cv2.namedWindow("Kison's mmwave Radar")
    cv2.imshow("Kison's mmwave Radar",main_door.image)

# Main loop 
targetObj = {}  
pointObj = {}
frameData = {}    
currentIndex = 0
while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()
        
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = targetObj
            currentIndex += 1
        
        time.sleep(0.033) # Sampling frequency of 30 Hz
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        break
