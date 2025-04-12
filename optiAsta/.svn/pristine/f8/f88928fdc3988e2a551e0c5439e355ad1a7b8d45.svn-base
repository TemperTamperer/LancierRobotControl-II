

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

/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

void URigid::printRigid()
{
   printf("# ID : %d (valid=%d)\n", ID, valid);
   printf("# pos: [%3.3f,%3.3f,%3.3f]\n", pos[0], pos[1], pos[2]);
   printf("# ori: [%3.4f,%3.4f,%3.4f,%3.4f]\n", rotq[0], rotq[1], rotq[2], rotq[3]);
};


/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

void UOptitrack::print()
{
    printf("# ---- there is %d rigid bodies\n", markersCnt);
    for (int i = 0; i < markersCnt; i++)
        markers[i]->printRigid();
    printf("# ---- timing\n");
    //     TimecodeStringify(timecode, timecodeSub, szTimecode, MTL);
//     printf("# Mid exposure  : %ld\n", cameraMidExposureTimestamp);
    printf("# Timestamp     : %3.3f s\n", timestamp);
    printf("# ---- end ----\n");
}
  
URigid * UOptitrack::findMarker(int id)
{
    URigid * a = markers[0];
    bool found = false;
    for (int i = 0; i < markersCnt; i++)
    {
        a = markers[i];
        if (a == nullptr)
        {
        break;
        }
        if (a->ID == id)
        {
        found = true;
        break;
        }
    }
    if (not found and (markersCnt < (MAX_MARKERS - 1)))
    {
        printf("# creating oblect for ID=%d\n", id);
        a = new URigid();
        markers[markersCnt++] = a;
        a->ID = id;
    }
    return a;
}
  
void UOptitrack::unpack(char * pData)
  { // Checks for NatNet Version number. Used later in function.
    // Packets may be different depending on NatNet version.
    int major = NatNetVersion[0];
    int minor = NatNetVersion[1];
    
    char *ptr = pData;
    
//     printf("Begin Packet\n-------\n");
    
    // First 2 Bytes is message ID
//     int MessageID = 0;
    memcpy(&MessageID, ptr, 2);
    ptr += 2;
//     printf("Message ID : %d\n", MessageID);
    
    // Second 2 Bytes is the size of the packet
//     int nBytes = 0;
    memcpy(&nBytes, ptr, 2);
    ptr += 2;
//     printf("Byte count : %d\n", nBytes);
    
    if (MessageID == 7)      // FRAME OF MOCAP DATA packet
    {
      // Next 4 Bytes is the frame number
//       int frameNumber = 0;
      memcpy(&frameNumber, ptr, 4);
      ptr += 4;
//       printf("Frame # : %d\n", frameNumber);
      
      // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
//       int nMarkerSets = 0;
      memcpy(&nMarkerSets, ptr, 4);
      ptr += 4;
      if (false)
        printf("Marker Set Count : %d\n", nMarkerSets);
      
      // Loop through number of marker sets and get name and data
      for (int i = 0; i < nMarkerSets; i++) {
        // Markerset name
        char szName[256];
        strcpy(szName, ptr);
        int nDataBytes = (int) strlen(szName) + 1;
        ptr += nDataBytes;
        printf("Model Name: %s\n", szName);
        
        // marker data
        int nMarkers = 0;
        memcpy(&nMarkers, ptr, 4);
        ptr += 4;
        printf("Marker Count : %d\n", nMarkers);
        
        for (int j = 0; j < nMarkers; j++) {
          float x = 0;
          memcpy(&x, ptr, 4);
          ptr += 4;
          float y = 0;
          memcpy(&y, ptr, 4);
          ptr += 4;
          float z = 0;
          memcpy(&z, ptr, 4);
          ptr += 4;
          printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
        }
      }
      
      // Loop through unlabeled markers
      int nOtherMarkers = 0;
      memcpy(&nOtherMarkers, ptr, 4);
      ptr += 4;
      // OtherMarker list is Deprecated
      //printf("Unidentified Marker Count : %d\n", nOtherMarkers);
      for (int j = 0; j < nOtherMarkers; j++) {
        float x = 0.0f;
        memcpy(&x, ptr, 4);
        ptr += 4;
        float y = 0.0f;
        memcpy(&y, ptr, 4);
        ptr += 4;
        float z = 0.0f;
        memcpy(&z, ptr, 4);
        ptr += 4;
        
        // Deprecated
        //printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
      }
      
      // Loop through rigidbodies
      int nRigidBodies = 0;
      memcpy(&nRigidBodies, ptr, 4);
      ptr += 4;
//       printf("Rigid Body Count : %d\n", nRigidBodies);
      for (int j = 0; j < nRigidBodies; j++) {
        // Rigid body position and orientation
        int ID = 0;
        memcpy(&ID, ptr, 4);
        // 
//         printf("# data for object %d\n", ID);
        // create a structure for rigid body
        URigid * b = findMarker(ID);
        if (b == nullptr)
        { // no more space
          printf("# -------------- no more space for rigid body data\n");
          return;
        }
        // fill with position and orientation
        ptr += 4;
//         float x = 0.0f;
        memcpy(&b->pos[0], ptr, 4);
        ptr += 4;
//         float y = 0.0f;
        memcpy(&b->pos[1], ptr, 4);
        ptr += 4;
//         float z = 0.0f;
        memcpy(&b->pos[2], ptr, 4);
        ptr += 4;
//         float qx = 0;
        memcpy(&b->rotq[0], ptr, 4);
        ptr += 4;
//         float qy = 0;
        memcpy(&b->rotq[1], ptr, 4);
        ptr += 4;
//         float qz = 0;
        memcpy(&b->rotq[2], ptr, 4);
        ptr += 4;
//         float qw = 0;
        memcpy(&b->rotq[3], ptr, 4);
        ptr += 4;
        
        
        // NatNet version 2.0 and later
        if (major >= 2) {
          // Mean marker error
//           float fError = 0.0f;
          memcpy(&b->posErr, ptr, 4);
          ptr += 4;
//           printf("Mean marker error: %3.2f\n", fError);
        }
        
        // NatNet version 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2)) {
          // params
          short params = 0;
          memcpy(&params, ptr, 2);
          ptr += 2;
          // 0x01 : rigid body was successfully tracked in this frame
          bool bTrackingValid = params & 0x01;
          b->valid = bTrackingValid;
//           if (bTrackingValid) {
//             printf("Tracking Valid: True\n");
//           } else {
//             printf("Tracking Valid: False\n");
//           }
        }
      } // Go to next rigid body
      // print status for all bodies
      print();
      
      
      // Skeletons (NatNet version 2.1 and later)
      if (((major == 2) && (minor > 0)) || (major > 2)) {
        int nSkeletons = 0;
        memcpy(&nSkeletons, ptr, 4);
        ptr += 4;
        if (nSkeletons > 0)
          printf("Skeleton Count : %d\n", nSkeletons);
        
        // Loop through skeletons
        for (int j = 0; j < nSkeletons; j++) {
          // skeleton id
          int skeletonID = 0;
          memcpy(&skeletonID, ptr, 4);
          ptr += 4;
          
          // Number of rigid bodies (bones) in skeleton
          int nRigidBodies = 0;
          memcpy(&nRigidBodies, ptr, 4);
          ptr += 4;
          printf("Rigid Body Count : %d\n", nRigidBodies);
          
          // Loop through rigid bodies (bones) in skeleton
          for (int j = 0; j < nRigidBodies; j++) {
            // Rigid body position and orientation
            int ID = 0;
            memcpy(&ID, ptr, 4);
            ptr += 4;
            float x = 0.0f;
            memcpy(&x, ptr, 4);
            ptr += 4;
            float y = 0.0f;
            memcpy(&y, ptr, 4);
            ptr += 4;
            float z = 0.0f;
            memcpy(&z, ptr, 4);
            ptr += 4;
            float qx = 0;
            memcpy(&qx, ptr, 4);
            ptr += 4;
            float qy = 0;
            memcpy(&qy, ptr, 4);
            ptr += 4;
            float qz = 0;
            memcpy(&qz, ptr, 4);
            ptr += 4;
            float qw = 0;
            memcpy(&qw, ptr, 4);
            ptr += 4;
            printf("ID : %d\n", ID);
            printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
            printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);
            
            // Before NatNet 3.0, marker data was here
            if (major < 3) {
              // associated marker positions
              int nRigidMarkers = 0;
              memcpy(&nRigidMarkers, ptr, 4);
              ptr += 4;
              printf("Marker Count: %d\n", nRigidMarkers);
              int nBytes = nRigidMarkers * 3 * sizeof(float);
              float *markerData = (float *) malloc(nBytes);
              memcpy(markerData, ptr, nBytes);
              ptr += nBytes;
              
              if (major >= 2) {
                // associated marker IDs
                nBytes = nRigidMarkers * sizeof(int);
                int *markerIDs = (int *) malloc(nBytes);
                memcpy(markerIDs, ptr, nBytes);
                ptr += nBytes;
                
                // associated marker sizes
                nBytes = nRigidMarkers * sizeof(float);
                float *markerSizes = (float *) malloc(nBytes);
                memcpy(markerSizes, ptr, nBytes);
                ptr += nBytes;
                
                for (int k = 0; k < nRigidMarkers; k++) {
                  printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
                          k,
                          markerIDs[k],
                          markerSizes[k],
                          markerData[k * 3],
                          markerData[k * 3 + 1],
                          markerData[k * 3 + 2]);
                }
                
                if (markerIDs)
                  free(markerIDs);
                if (markerSizes)
                  free(markerSizes);
                
              } else {
                for (int k = 0; k < nRigidMarkers; k++) {
                  printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
                          markerData[k * 3], markerData[k * 3 + 1],
                          markerData[k * 3 + 2]);
                }
              }
              if (markerData)
                free(markerData);
            }
            
            // Mean marker error (NatNet version 2.0 and later)
            if (major >= 2) {
              float fError = 0.0f;
              memcpy(&fError, ptr, 4);
              ptr += 4;
              printf("Mean marker error: %3.2f\n", fError);
            }
            
            // Tracking flags (NatNet version 2.6 and later)
            if (((major == 2) && (minor >= 6)) || (major > 2)) {
              // params
              short params = 0;
              memcpy(&params, ptr, 2);
              ptr += 2;
              // 0x01 : rigid body was successfully tracked in this frame
              bool bTrackingValid = params & 0x01;
            }
            
          } // next rigid body
          
        } // next skeleton
      }
      
      // labeled markers (NatNet version 2.3 and later)
      if (((major == 2) && (minor >= 3)) || (major > 2)) {
        int nLabeledMarkers = 0;
        memcpy(&nLabeledMarkers, ptr, 4);
        ptr += 4;
        if (nLabeledMarkers > 0)
          printf("Labeled Marker Count : %d\n", nLabeledMarkers);
        
        // Loop through labeled markers
        for (int j = 0; j < nLabeledMarkers; j++) {
          // id
          // Marker ID Scheme:
          // Active Markers:
          //   ID = ActiveID, correlates to RB ActiveLabels list
          // Passive Markers:
          //   If Asset with Legacy Labels
          //      AssetID 	(Hi Word)
          //      MemberID	(Lo Word)
          //   Else
          //      PointCloud ID
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          int modelID, markerID;
          DecodeMarkerID(ID, &modelID, &markerID);
          
          
          // x
          float x = 0.0f;
          memcpy(&x, ptr, 4);
          ptr += 4;
          // y
          float y = 0.0f;
          memcpy(&y, ptr, 4);
          ptr += 4;
          // z
          float z = 0.0f;
          memcpy(&z, ptr, 4);
          ptr += 4;
          // size
          float size = 0.0f;
          memcpy(&size, ptr, 4);
          ptr += 4;
          
          // NatNet version 2.6 and later
          if (((major == 2) && (minor >= 6)) || (major > 2)) {
            // marker params
            short params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;
            // marker was not visible (occluded) in this frame
            bool bOccluded = (params & 0x01) != 0;
            // position provided by point cloud solve
            bool bPCSolved = (params & 0x02) != 0;
            // position provided by model solve
            bool bModelSolved = (params & 0x04) != 0;
            if (major >= 3) {
              // marker has an associated model
              bool bHasModel = (params & 0x08) != 0;
              // marker is an unlabeled marker
              bool bUnlabeled = (params & 0x10) != 0;
              // marker is an active marker
              bool bActiveMarker = (params & 0x20) != 0;
            }
            
          }
          
          // NatNet version 3.0 and later
          float residual = 0.0f;
          if (major >= 3) {
            // Marker residual
            memcpy(&residual, ptr, 4);
            ptr += 4;
          }
          if (true)
          {
            printf("ID  : [MarkerID: %d] [ModelID: %d]\n", markerID, modelID);
            printf("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
            printf("size: [%3.2f]\n", size);
            printf("err:  [%3.2f]\n", residual);
          }
        }
      }
      
      // Force Plate data (NatNet version 2.9 and later)
      if (((major == 2) && (minor >= 9)) || (major > 2)) {
        int nForcePlates;
        memcpy(&nForcePlates, ptr, 4);
        ptr += 4;
        for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++) {
          // ID
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          printf("Force Plate : %d\n", ID);
          
          // Channel Count
          int nChannels = 0;
          memcpy(&nChannels, ptr, 4);
          ptr += 4;
          
          // Channel Data
          for (int i = 0; i < nChannels; i++) {
            printf(" Channel %d : ", i);
            int nFrames = 0;
            memcpy(&nFrames, ptr, 4);
            ptr += 4;
            for (int j = 0; j < nFrames; j++) {
              float val = 0.0f;
              memcpy(&val, ptr, 4);
              ptr += 4;
              printf("%3.2f   ", val);
            }
            printf("\n");
          }
        }
      }
      
      // Device data (NatNet version 3.0 and later)
      if (((major == 2) && (minor >= 11)) || (major > 2)) {
        int nDevices;
        memcpy(&nDevices, ptr, 4);
        ptr += 4;
        for (int iDevice = 0; iDevice < nDevices; iDevice++) {
          // ID
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          printf("Device : %d\n", ID);
          
          // Channel Count
          int nChannels = 0;
          memcpy(&nChannels, ptr, 4);
          ptr += 4;
          
          // Channel Data
          for (int i = 0; i < nChannels; i++) {
            printf(" Channel %d : ", i);
            int nFrames = 0;
            memcpy(&nFrames, ptr, 4);
            ptr += 4;
            for (int j = 0; j < nFrames; j++) {
              float val = 0.0f;
              memcpy(&val, ptr, 4);
              ptr += 4;
              printf("%3.2f   ", val);
            }
            printf("\n");
          }
        }
      }
      
      // software latency (removed in version 3.0)
//       if (major < 3) {
//         float softwareLatency = 0.0f;
//         memcpy(&softwareLatency, ptr, 4);
//         ptr += 4;
//         printf("software latency : %3.3f\n", softwareLatency);
//       }
      
      // timecode
//       unsigned int timecode = 0;
      memcpy(&timecode, ptr, 4);
      ptr += 4;
//       unsigned int timecodeSub = 0;
      memcpy(&timecodeSub, ptr, 4);
      ptr += 4;
//       char szTimecode[128] = "";
//       TimecodeStringify(timecode, timecodeSub, szTimecode, 128);
      
      // timestamp
//       double timestamp = 0.0f;
      
      // NatNet version 2.7 and later - increased from single to double precision
      if (((major == 2) && (minor >= 7)) || (major > 2)) {
        memcpy(&timestamp, ptr, 8);
        ptr += 8;
      } 
      else 
      {
        float fTemp = 0.0f;
        memcpy(&fTemp, ptr, 4);
        ptr += 4;
        timestamp = (double) fTemp;
      }
//       printf("Timestamp : %3.3f\n", timestamp);
      
      // high res timestamps (version 3.0 and later)
      if (major >= 3) {
//         uint64_t cameraMidExposureTimestamp = 0;
        memcpy(&cameraMidExposureTimestamp, ptr, 8);
        ptr += 8;
//         printf("Mid-exposure timestamp : %" PRIu64 "\n",
//                 cameraMidExposureTimestamp);
        
//         uint64_t cameraDataReceivedTimestamp = 0;
        memcpy(&cameraDataReceivedTimestamp, ptr, 8);
        ptr += 8;
//         printf("Camera data received timestamp : %" PRIu64 "\n",
//                 cameraDataReceivedTimestamp);
        
//         uint64_t transmitTimestamp = 0;
        memcpy(&transmitTimestamp, ptr, 8);
        ptr += 8;
//         printf("Transmit timestamp : %" PRIu64 "\n", transmitTimestamp);
      }
      
      // frame params
      short params = 0;
      memcpy(&params, ptr, 2);
      ptr += 2;
      // 0x01 Motive is recording
      bool bIsRecording = (params & 0x01) != 0;
      // 0x02 Actively tracked model list has changed
      bool bTrackedModelsChanged = (params & 0x02) != 0;
      
      
      // end of data tag
      int eod = 0;
      memcpy(&eod, ptr, 4);
      ptr += 4;
//       printf("End Packet\n-------------\n");
      
    } else if (MessageID == 5) // Data Descriptions
    {
      // number of datasets
      int nDatasets = 0;
      memcpy(&nDatasets, ptr, 4);
      ptr += 4;
      printf("Dataset Count : %d\n", nDatasets);
      
      for (int i = 0; i < nDatasets; i++) {
        printf("Dataset %d\n", i);
        
        int type = 0;
        memcpy(&type, ptr, 4);
        ptr += 4;
        printf("Type : %d\n", type);
        
        if (type == 0)   // markerset
        {
          // name
          char szName[256];
          strcpy(szName, ptr);
          int nDataBytes = (int) strlen(szName) + 1;
          ptr += nDataBytes;
          printf("Markerset Name: %s\n", szName);
          
          // marker data
          int nMarkers = 0;
          memcpy(&nMarkers, ptr, 4);
          ptr += 4;
          printf("Marker Count : %d\n", nMarkers);
          
          for (int j = 0; j < nMarkers; j++) {
            char szName[256];
            strcpy(szName, ptr);
            int nDataBytes = (int) strlen(szName) + 1;
            ptr += nDataBytes;
            printf("Marker Name: %s\n", szName);
          }
        } else if (type == 1)   // rigid body
        {
          if (major >= 2) {
            // name
            char szName[MAX_NAMELENGTH];
            strcpy(szName, ptr);
            ptr += strlen(ptr) + 1;
            printf("Name: %s\n", szName);
          }
          
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          printf("ID : %d\n", ID);
          
          int parentID = 0;
          memcpy(&parentID, ptr, 4);
          ptr += 4;
          printf("Parent ID : %d\n", parentID);
          
          float xoffset = 0;
          memcpy(&xoffset, ptr, 4);
          ptr += 4;
          printf("X Offset : %3.2f\n", xoffset);
          
          float yoffset = 0;
          memcpy(&yoffset, ptr, 4);
          ptr += 4;
          printf("Y Offset : %3.2f\n", yoffset);
          
          float zoffset = 0;
          memcpy(&zoffset, ptr, 4);
          ptr += 4;
          printf("Z Offset : %3.2f\n", zoffset);
          
          // Per-marker data (NatNet 3.0 and later)
          if (major >= 3) {
            int nMarkers = 0;
            memcpy(&nMarkers, ptr, 4);
            ptr += 4;
            
            // Marker positions
            nBytes = nMarkers * 3 * sizeof(float);
            float *markerPositions = (float *) malloc(nBytes);
            memcpy(markerPositions, ptr, nBytes);
            ptr += nBytes;
            
            // Marker required active labels
            nBytes = nMarkers * sizeof(int);
            int *markerRequiredLabels = (int *) malloc(nBytes);
            memcpy(markerRequiredLabels, ptr, nBytes);
            ptr += nBytes;
            
            for (int markerIdx = 0; markerIdx < nMarkers; ++markerIdx) {
              float *markerPosition = markerPositions + markerIdx * 3;
              const int markerRequiredLabel = markerRequiredLabels[markerIdx];
              
              printf("\tMarker #%d:\n", markerIdx);
              printf("\t\tPosition: %.2f, %.2f, %.2f\n",
                      markerPosition[0],
                      markerPosition[1],
                      markerPosition[2]);
              
              if (markerRequiredLabel != 0) {
                printf("\t\tRequired active label: %d\n", markerRequiredLabel);
              }
            }
            
            free(markerPositions);
            free(markerRequiredLabels);
          }
        } else if (type == 2)   // skeleton
        {
          char szName[MAX_NAMELENGTH];
          strcpy(szName, ptr);
          ptr += strlen(ptr) + 1;
          printf("Name: %s\n", szName);
          
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          printf("ID : %d\n", ID);
          
          int nRigidBodies = 0;
          memcpy(&nRigidBodies, ptr, 4);
          ptr += 4;
          printf("RigidBody (Bone) Count : %d\n", nRigidBodies);
          
          for (int i = 0; i < nRigidBodies; i++) {
            if (major >= 2) {
              // RB name
              char szName[MAX_NAMELENGTH];
              strcpy(szName, ptr);
              ptr += strlen(ptr) + 1;
              printf("Rigid Body Name: %s\n", szName);
            }
            
            int ID = 0;
            memcpy(&ID, ptr, 4);
            ptr += 4;
            printf("RigidBody ID : %d\n", ID);
            
            int parentID = 0;
            memcpy(&parentID, ptr, 4);
            ptr += 4;
            printf("Parent ID : %d\n", parentID);
            
            float xoffset = 0;
            memcpy(&xoffset, ptr, 4);
            ptr += 4;
            printf("X Offset : %3.2f\n", xoffset);
            
            float yoffset = 0;
            memcpy(&yoffset, ptr, 4);
            ptr += 4;
            printf("Y Offset : %3.2f\n", yoffset);
            
            float zoffset = 0;
            memcpy(&zoffset, ptr, 4);
            ptr += 4;
            printf("Z Offset : %3.2f\n", zoffset);
          }
        }
        
      }   // next dataset
      
      printf("End Packet\n-------------\n");
      
    } else {
      printf("Unrecognized Packet Type.\n");
    }
      
  }


