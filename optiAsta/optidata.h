

#include <cinttypes>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>

#include "main.h"

class URigid
{
public:
  int ID;
  float pos[3];
  float rotq[4];
  float posErr;
  bool valid;
  
  void printRigid();
};

class UOptitrack
{
protected:
  int MessageID = 0;
  int nBytes = 0;
  int frameNumber = 0;
  int nMarkerSets = 0;
  // positions
  static const int MAX_MARKERS = 30;
  URigid * markers[MAX_MARKERS] = {nullptr};
  int markersCnt = 0;
  // common timestamp
  unsigned int timecode = 0;   // not used
  unsigned int timecodeSub = 0;// not used
  static const int MTL = 128;
  char szTimecode[MTL] = "";
  double timestamp;
  // more timing
  uint64_t cameraMidExposureTimestamp = 0; // not used
  uint64_t cameraDataReceivedTimestamp = 0; // not used
  uint64_t transmitTimestamp = 0; // from server
  
  void print();
  
  URigid * findMarker(int id);
  
public:
  // major NatNet unpack, modified version of the one in PacketClient.cpp
  void unpack(char * pData);
};

