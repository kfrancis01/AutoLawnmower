//Mower type and structure definitions

//these 2 unions are used throughout MM packet parsing to easily break up fields into their
//    their various pieces and data types.
// 2 byte fields
typedef union {
  unsigned char b[2];
  unsigned int w;
  int wi;
} uni_8x2_16;

// 4 byte fields
typedef union {
  unsigned char b[4];
  float f;
  unsigned long v32;
  long vi32;
} uni_8x4_32;

//Mower primary data packet structure, HedgeHog high res
struct MMHighResPacket
{
    //header
    unsigned char  destAddr;   //defined as and should be: 0xff
    unsigned char  packetType; //type of packet and should be: 0x47
    uni_8x2_16     dataCode;   //sub-type of data and should be: 0x0011
    unsigned char  dataBytes;  //number of bytes for rest of message, less CRC, should be: 0x16

    //detail
    uni_8x4_32     timeStamp;  //since app started
    long           x;        //coordinates in millimeters
    long           y;
    long           z;
    unsigned char  flags;      //7 flags (on/off) as bits in a single byte
    unsigned char  hedgeAddr;  //ID of the hedge
    uni_8x2_16     hedgeOrientation;  //which way the hedge is facing, placement of hedge
                                       //  on mower
    uni_8x2_16   timePassed; //ultrasound flight time, largely ignore this
    uni_8x2_16   CRC_16;     //CRC used to verify complete packet correctness
};

struct MMBeaconPacket
{
    //This is a highly unique message type in terms of length.
    //  Each beacon extends the overall length of this message by 14 bytes, which
    //    renders this structure invalid when there are more than 4 beacons and one hedge.
    //header
    unsigned char  destAddr;   //defined as and should be: 0xff
    unsigned char  packetType; //type of packet and should be: 0x47
    uni_8x2_16     dataCode;   //sub-type of data and should be: 0x0012
    unsigned char  dataBytes;  //number of bytes for rest of message, less CRC, should be: 0x40

    //detail
    unsigned char  numBeacons; //number of beacons in message
    unsigned char  beacon1;    //arbitrary order of beacon addresses, first one
    long           xb1;        //coordinates in millimeters
    long           yb1;
    long           zb1;
    unsigned char  reserved1;

    unsigned char  beacon2;    //arbitrary order of beacon addresses, second one
    long           xb2;        //coordinates in millimeters
    long           yb2;
    long           zb2;
    unsigned char  reserved2;

    unsigned char  beacon3;    //arbitrary order of beacon addresses, third one
    long           xb3;        //coordinates in millimeters
    long           yb3;
    long           zb3;
    unsigned char  reserved3;

    unsigned char  beacon4;    //arbitrary order of beacon addresses, fourth one
    long           xb4;        //coordinates in millimeters
    long           yb4;
    long           zb4;
    unsigned char  reserved4;

    uni_8x2_16   CRC_16;     //CRC used to verify complete packet correctness
};
