/**
main.h*/

#define MAX_PACKETSIZE              100000    // actual packet size is dynamic
#define MAX_NAMELENGTH              256

bool setup(int argc, char *argv[]);
void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID);
bool TimecodeStringify(unsigned int inTimecode,
                       unsigned int inTimecodeSubframe,
                       char *Buffer,
                       size_t BufferSize);
/**
 * New dataframe received, to handle 
 * \param pData is pointer to binary NatNet data to unpack
 * */
void unpack(char * pData);

extern int NatNetVersion[4];
extern int ServerVersion[4];
