

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
#include "optidata.h"


/**
 * copy paste function from 
 * https://stackoverflow.com/questions/212528/get-the-ip-address-of-the-machine
 * to find IP string for this machine 
 * \param ipString is string to return result,
 * \param ipStringLen is maximum length of the provided string */
void findIP(char * ipString, int ipStringLen)
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  
  getifaddrs(&ifAddrStruct);
  
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
      continue;
    }
    if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
      // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, ipString, ipStringLen);
      printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
    } else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
      // is a valid IP6 Address
      tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
      char addressBuffer[INET6_ADDRSTRLEN];
//       inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
//       printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
      inet_ntop(AF_INET6, tmpAddrPtr, ipString, ipStringLen);
    } 
  }
  if (ifAddrStruct!=NULL) 
    freeifaddrs(ifAddrStruct);
}

/**
 * setup parameters and start */
bool startNatNetConnection(const char * argv0)
{ // find IP/name of server and this machine
  const int MIL = 100;
  char ipStr[MIL];
  const char * optitrackServerInAsta = "DESKTOP-OVLVRUD.local";
  // IP of this machine
  findIP(ipStr, MIL);
  strncpy(ipStr, "192.168.1.85", MIL);
  // generate an argc, argv set
  const char * argv1[4] = {argv0, optitrackServerInAsta, ipStr, "\0"};
  // run NatNet connection for optitrackServer
  // also starts threads for package reception
  // takes 3 string parameters (app name, server IP/name, client IP)
  bool started = setup(3, (char **)argv1);
  return started;
}


UOptitrack * frame = nullptr;

/**
 * Unpack NatNet frame.
 * Called from receive thread.
 * A new frame is detected 
 * \param pData is pointer to binary frame to int unpack(char* pData) */
 void unpack(char * pData)
 { // should maybe be expanded to 
   // at least 2 frames with resource lock
   // activated during unpack/use
   if (frame == nullptr)
     frame = new UOptitrack();
   frame->unpack(pData);
 }


int main(int argc, char *argv[]) 
{ // estavlish connection
  bool isOK = startNatNetConnection(argv[0]);
  int c;
  char szRequest[512];
  bool bExit = false;
  while (not bExit) 
  {
    c = getchar();
    switch (c) 
    {
      case 'q':
        bExit = true;
        break;
      default:
        break;
    }
  }
};
