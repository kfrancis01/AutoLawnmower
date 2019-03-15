//#include <SoftwareSerial.h>  //needed only if using UNO(one hardware serial port) board.
//  recode Serial1 and Serial2 uses as SoftwareSerial.
#include "Mower3.h"
#include <Kangaroo.h>
//global constants, these do not change during a given execution

//operating mode values (OPMODE) can be combined as bits and tested during execution separately
//   and set at compile time and never modified during execution.
//   if printing anything TESTING must be set to setup the Serial0 port
const int DEBUG = 1;       //set to 0 for production, 1 will give Serial monitor info
//OPMODE uses compile time math to set OPMODE flags.
//const int OPMODE2= 2;     //OPMODES are set using bits, powers of 2,
//    use this and/or next one for other reasons
//const int OPMODE=DEBUG+OPMODE2;  //should likely be 0 for full up production use.
const int OPMODE=DEBUG;  //should likely be all 0's for full up production use.

const long ARBAUDRATE= 57600;
const unsigned long microsPerReading= 100;  //number of microseconds between work efforts
//as measured within loop()

//global variables available everywhere to everything
unsigned long microsPrevious= 0;    //these 2 are used to measure time between work efforts
unsigned long microsNow= 0;

/////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PARTI

//buffer is extra large so we can at least pull in one full and complete packet,
//    no matter where in the buffer it starts, other bytes are ignored/discarded
#define HEDGEHOG_BUF_SIZE 140
#define HEDGEHOG_MM_DATA_SIZE 0x16
#define POSITION_DATAGRAM_BEACON_ID 0x0012    //used to find static beacon locations
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011     //this is our target DATAGRAM_ID

//SoftwareSerial MMSerial(19, 18); //Input(RX),Output(TX): only needed for UNO
const long MMBAUDRATE= 57600; //baudrate for Marvelmind configured to transmit data
const long KRBAUDRATE= 57600;  //kangaroo baudrate
const unsigned long hhLoopWait= 1000;
const unsigned char hiResPacketSize= 27;  //HiResPacketSize less CRC
const unsigned char beaconPacketSize= 62;  //BeaconPacketSize less CRC
const unsigned char HHADDRESS= 5;  //change this if you renumber the beacons/hedge

long hedgehog_x;// temp X coordinate of hedgehog, mm
long hedgehog_y;// temp Y coordinate of hedgehog, mm
long hedgehog_z;// temp Z coordinate(height) of hedgehog, mm

bool hedgehog_pos_updated= false;// flag of new data from hedgehog received
bool beacon_packet_updated= false;
int packet_received;
bool high_resolution_mode;

unsigned long timer = 0;
float dt = 0;

byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];  //raw buffer for reading in unparsed bytes
byte hedgehog_serial_buf_ofs;

uni_8x2_16 hedgehog_data_id;    //in case of mower project this should be 0x11

MMHighResPacket  dataPacket;    //holds one fully parsed high resolution hedgeLOC data packet
MMBeaconPacket   beaconPacket;  //static beacon positions, only read/need one

bool hiResType;
bool beaconType;
bool beaconRead;

// TODO check that all variables are accounted for
// Path Creation Variables
int currentC; // index for current checkpoint in array
int LapNumber = 0; // incremental variable
CheckPoint CP[50]; // TODO fix this initialization
long separation = 2000; // Separation Distance between checkpoints
long Cpx; // checkpoint X
long Cpy; // Checkpoint Y
long tolerance = 50; // 5cm Navigation Tolerance
bool firstLoop = 1; // boolean for first time through loop
long xb1, xb2, xb3,xb4,yb1,yb2,yb3,yb4; //beacon positions
int ta; // turn angle global variable
long offset = 1000; //total offset
long x1,x2,x3,x4,y1,y2,y3,y4; // offset beacon positions
EndPoint End[20]; // TODO fix this initialization
// EndPoint[20] = { 0 }; ^^^ Could be used instead of above line
long rowOffset = 250; // separation between rows (50cm for now)
long Endx, Endy; //End of row position
const double Pi = 3.1415926;
int numberOfRows = 0;
// long dist;

// Movement Variables
int encoderSpeed = 3; // encoder ticks per second
double TicksPerDegree = 0.78; // Ratio of Ticks to the degrees turned
long xOld; // last x position
long yOld; // last y position

KangarooSerial  K(Serial2);
KangarooChannel Drive(K, 'D');
KangarooChannel Turn(K, 'T');

////////////////////////////
//    MMSerial hedgehog support initialization
///////////////////////////


void setup_hedgehog() {

  // Serial1.begin(MMBAUDRATE); // set Baud speed for hedgehog

  //init any required global MM variables with their default values
  hedgehog_serial_buf_ofs = 0;
  hedgehog_pos_updated = false;
  high_resolution_mode = false;

  //these should be static values in the highres packet header
  dataPacket.destAddr= 0xff;
  dataPacket.packetType= 0x47;
  dataPacket.dataCode.w= 0x0011;
  dataPacket.dataBytes= 22;
  dataPacket.hedgeAddr= HHADDRESS;

  dataPacket.timeStamp.v32= 0;

  //preset current record type flags
  hiResType= false;
  beaconType= false;
  beaconRead= false;

  //These should be the static BEACON values read from the first available BeaconPacket.
  //  After the first all other BeaconPackets will be dropped.
  beaconPacket.destAddr= 0xff;
  beaconPacket.packetType= 0x47;
  beaconPacket.dataCode.w= 0x0012;
  beaconPacket.dataBytes= 57; //4 beacon LOCs plus  for the beacon count
  beaconPacket.numBeacons= 0;

  ////////////////////////////////////////////
  // get beacon data
  //use find_beacon() to find/retrieve BeaconPacket
  ///////////////////////////////////////////
  
  while ( !beaconRead) {  //loop until you find one
    find_beacon();
    //delay(1);  //just wait a bit and look again
  }

  // Since beacon# is based on order in which they come in
  // redefines x# and y# based on actual beacon address
  if (beaconPacket.beacon1==1){xb1=beaconPacket.xb1; yb1=beaconPacket.yb1;}
  else if (beaconPacket.beacon1==2){xb2=beaconPacket.xb1; yb2=beaconPacket.yb1;}
  else if (beaconPacket.beacon1==3){xb3=beaconPacket.xb1; yb3=beaconPacket.yb1;}
  else if (beaconPacket.beacon1==4){xb4=beaconPacket.xb1; yb4=beaconPacket.yb1;}

  if (beaconPacket.beacon2==1){xb1=beaconPacket.xb2; yb1=beaconPacket.yb2;}
  else if (beaconPacket.beacon2==2){xb2=beaconPacket.xb2; yb2=beaconPacket.yb2;}
  else if (beaconPacket.beacon2==3){xb3=beaconPacket.xb2; yb3=beaconPacket.yb2;}
  else if (beaconPacket.beacon2==4){xb4=beaconPacket.xb2; yb4=beaconPacket.yb2;}

  if (beaconPacket.beacon3==1){xb1=beaconPacket.xb3; yb1=beaconPacket.yb3;}
  else if (beaconPacket.beacon3==2){xb2=beaconPacket.xb3; yb2=beaconPacket.yb3;}
  else if (beaconPacket.beacon3==3){xb3=beaconPacket.xb3; yb3=beaconPacket.yb3;}
  else if (beaconPacket.beacon3==4){xb4=beaconPacket.xb3; yb4=beaconPacket.yb3;}

  if (beaconPacket.beacon4==1){xb1=beaconPacket.xb4; yb1=beaconPacket.yb4;}
  else if (beaconPacket.beacon4==2){xb2=beaconPacket.xb4; yb2=beaconPacket.yb4;}
  else if (beaconPacket.beacon4==3){xb3=beaconPacket.xb4; yb3=beaconPacket.yb4;}
  else if (beaconPacket.beacon4==4){xb4=beaconPacket.xb4; yb4=beaconPacket.yb4;}
  //end of redefined beacon positions
  
  //check the Beacon CRC
  unsigned int tmp= hedgehog_set_crc16( &(beaconPacket.destAddr), beaconPacketSize);

  if ( tmp != beaconPacket.CRC_16.w) {
    if ( OPMODE & DEBUG) {
      Serial.print( "CRCs not equal: Packet CRC= " );
      Serial.print( beaconPacket.CRC_16.w, HEX);
      Serial.print( "    Calc CRC= " );
      Serial.println( tmp, HEX);
    }
  }

  hiResType= false;
  beaconType= false;

  ////////////////////////////////////////////////////////////////////////
  // TODO add setup functions that create path here
  // OFFSET CALCULATION FUNCTION CALL
  // END OF ROW POINTS CALCULATION
  // ADD ANY ADDITIONAL SET UP PROTOCOLS HERE
  // AND ADD MOVEMENT AND ADDITIONAL PATH CREATION STEPS IN LOOP_HEDGEHOG
  /////////////////////////////////////////////////////////////////////////
  
  //now the beaconPacket is ready to pass into mower ambulate methods after each HiResPacket!!!
  //ready to use continuous Arduino looping.
}  // end setup_hedgehog

int readFullBuffer1( byte buffer[], int bufSize) {
  int retLen= -1;  //will indicate bytes actually read.
  unsigned char byt;

  int byteCount= 0;
  while ( byteCount < bufSize) {
    if ( Serial1.available()) {
      byt= Serial1.read();
      buffer[byteCount]= byt;
      byteCount++;
    }
  }
  retLen= byteCount;

  //entire buffer should be full and we can now start processing it
  return retLen;
}

void clearReadBuffer() {
  //clear buffer and any other data for next read
  for ( int j= 0; j < HEDGEHOG_BUF_SIZE; j++) {
    hedgehog_serial_buf[j]= 0x00;
  }
}

bool transferHiResPacket( unsigned char* buf) {
  bool ret= false;
  uni_8x2_16 un16;
  uni_8x4_32 un32;

  //just skip the header fields
  dataPacket.dataBytes = buf[4];

  un32.b[0] = buf[5];
  un32.b[1] = buf[6];
  un32.b[2] = buf[7];
  un32.b[3] = buf[8];
  dataPacket.timeStamp.v32 = un32.v32;

  // coordinates of hedgehog (X), mm
  un32.b[0] = buf[9];
  un32.b[1] = buf[10];
  un32.b[2] = buf[11];
  un32.b[3] = buf[12];
  dataPacket.x = un32.vi32;

  // coordinates of hedgehog (Y), mm
  un32.b[0] = buf[13];
  un32.b[1] = buf[14];
  un32.b[2] = buf[15];
  un32.b[3] = buf[16];
  dataPacket.y = un32.vi32;

  // coordinates of hedgehog (Z), mm
  un32.b[0] = buf[17];
  un32.b[1] = buf[18];
  un32.b[2] = buf[19];
  un32.b[3] = buf[20];
  dataPacket.z = un32.vi32;

  dataPacket.flags= buf[21];
  dataPacket.hedgeAddr= buf[22];

  un16.b[0] = buf[23];
  un16.b[1] = buf[24];
  dataPacket.hedgeOrientation= un16;

  un16.b[0] = buf[25];
  un16.b[1] = buf[26];
  dataPacket.timePassed= un16;

  un16.b[0] = buf[27];
  un16.b[1] = buf[28];
  dataPacket.CRC_16= un16;

  hedgehog_pos_updated = true; // flag of new data from hedgehog received

  return ret;
}  //end transferHiResPacket()

bool transferBeaconPacket( unsigned char* buf) {
  uni_8x2_16 un16;
  uni_8x4_32 un32;
  bool ret= false;

  //pick up one complete BeaconPacket to fill this needed info
  //  All other BeaconPackets will be dropped.
  //the header values should not change, nor should the number of beacons for Mower Project
  beaconPacket.destAddr= 0xff;
  beaconPacket.packetType= 0x47;
  beaconPacket.dataCode.w= 0x0012;
  beaconPacket.dataBytes= 62; //4 beacon LOCs plus  for the beacon count

  //these next few lines would be replaced by reading the static beacon packet!!!
  beaconPacket.numBeacons= 4; //mower project using only 4 beacons

  //just skip the header fields
  beaconPacket.beacon1= buf[6];
  un32.b[0] = buf[7];
  un32.b[1] = buf[8];
  un32.b[2] = buf[9];
  un32.b[3] = buf[10];
  beaconPacket.xb1= un32.vi32;
  un32.b[0] = buf[11];
  un32.b[1] = buf[12];
  un32.b[2] = buf[13];
  un32.b[3] = buf[14];
  beaconPacket.yb1= un32.vi32;
  un32.b[0] = buf[15];
  un32.b[1] = buf[16];
  un32.b[2] = buf[17];
  un32.b[3] = buf[18];
  beaconPacket.zb1= un32.vi32;
  beaconPacket.reserved1= buf[19];  //[19]unimportant until protocol changes from MarvelMind

  beaconPacket.beacon2= buf[20];  //[20]
  un32.b[0] = buf[21];
  un32.b[1] = buf[22];
  un32.b[2] = buf[23];
  un32.b[3] = buf[24];
  beaconPacket.xb2= un32.vi32;
  un32.b[0] = buf[25];
  un32.b[1] = buf[26];
  un32.b[2] = buf[27];
  un32.b[3] = buf[28];
  beaconPacket.yb2= un32.vi32;
  un32.b[0] = buf[29];
  un32.b[1] = buf[30];
  un32.b[2] = buf[31];
  un32.b[3] = buf[32];
  beaconPacket.zb2= un32.vi32;
  beaconPacket.reserved2= buf[33];  //[33]unimportant until protocol changes from MM

  beaconPacket.beacon3= buf[34];
  un32.b[0] = buf[35];
  un32.b[1] = buf[36];
  un32.b[2] = buf[37];
  un32.b[3] = buf[38];
  beaconPacket.xb3= un32.vi32;
  un32.b[0] = buf[39];
  un32.b[1] = buf[40];
  un32.b[2] = buf[41];
  un32.b[3] = buf[42];
  beaconPacket.yb3= un32.vi32;
  un32.b[0] = buf[43];
  un32.b[1] = buf[44];
  un32.b[2] = buf[45];
  un32.b[3] = buf[46];
  beaconPacket.zb3= un32.vi32;
  beaconPacket.reserved3= buf[47];  //[47]unimportant until protocol changes from MM

  beaconPacket.beacon4= buf[48];
  un32.b[0] = buf[49];
  un32.b[1] = buf[50];
  un32.b[2] = buf[51];
  un32.b[3] = buf[52];
  beaconPacket.xb4= un32.vi32;
  un32.b[0] = buf[53];
  un32.b[1] = buf[54];
  un32.b[2] = buf[55];
  un32.b[3] = buf[56];
  beaconPacket.yb4= un32.vi32;
  un32.b[0] = buf[57];
  un32.b[1] = buf[58];
  un32.b[2] = buf[59];
  un32.b[3] = buf[60];
  beaconPacket.zb4= un32.vi32;
  beaconPacket.reserved4= buf[61];  //[61]unimportant until protocol changes from MM

  un16.b[0]= buf[62];
  un16.b[1]= buf[63];
  beaconPacket.CRC_16= un16;

  beaconRead= true;

  return ret;
}  //end transferBeaconPacket()

void printRawBuffer(int bytes) {

  for( int i= 0; i < bytes; i++) {
    Serial.print( hedgehog_serial_buf[i], HEX);
    Serial.print( " ");
  }
  Serial.println();
}

//like loop_hedgehog(), but this is not part of the main lop process and
//    is geared to find a static beacon packet, save it and return so loop can start.
void find_beacon() {
  int incoming_byte;
  int total_received_in_loop;
  bool good_byte;
  byte packet_size;
  uni_8x2_16 un16;
  uni_8x4_32 un32;

  //read full double buffer
  //this method wastes bytes
  int bytesRead= readFullBuffer1( hedgehog_serial_buf, HEDGEHOG_BUF_SIZE);
/*
  if (OPMODE & DEBUG) {
    //print raw packet
    Serial.println( bytesRead);
    printRawBuffer( bytesRead);
  }
*/

  //find header bytes of a Beacon Packet, only need one beaconPacket
  int bufndx= 0;
  //make sure there is room for a whole packet
  for ( ; bufndx < bytesRead- (beaconPacketSize+6); bufndx++) {
    //only need one beacon packet, they won't move, so look for them first
    if ((hedgehog_serial_buf[bufndx] == beaconPacket.destAddr) &&
      (hedgehog_serial_buf[bufndx+1] == beaconPacket.packetType) &&
      (hedgehog_serial_buf[bufndx+2] == beaconPacket.dataCode.b[0]) &&
      (hedgehog_serial_buf[bufndx+3] == beaconPacket.dataCode.b[1])  &&
      (hedgehog_serial_buf[bufndx+4] == beaconPacket.dataBytes) ) {
      //now we have found the start of a beaconPacket, so skip out of search loop
      beaconType= true;
//      if (OPMODE & DEBUG) {
//        Serial.println( "\nBEACON!\n");
//      }
    break;
    }  //end search cases and start search at next char
  }  //end for search loop

  //transfer packet into Packet structures
  if ( beaconType) {
    transferBeaconPacket( &(hedgehog_serial_buf[bufndx]));
    if ( OPMODE & DEBUG) {
      printBeaconPacket();
    }
  }
/*
  else {
    if (OPMODE & DEBUG) {
      Serial.print( ".");
    }
  }
*/

  //clear buffer and any other data for next read
  clearReadBuffer();

  //exit loop for finding and reading first beacon packet
}  //void find_beacon()


////////////////////////////////
// MM hedgehog service loop, looking for all only High res packets for now.
//   This process can throw away some packets because they just keep streaming,
//     so just read in at least twice as many bytes as you need for your largest packet,
//     find the first good packet, process it, and loop.
//   Logic synopsis:
//      1) read in a full double sized buffer, extra bytes are thrown away,
//            loop again until buffer is full.
//      2) find beginning of desired packet type by looking for and matching the first 5
//            bytes of your acceptable packet types.
//      3) fill the remaining selected packet structure from buffer
//      4) use completed packet to determine next action for robot
//            (new team geo-locating function)
//      5) clear buffer for next read
//      6) exit loop_hedge() so Arduino can skip to next iteration of loop
///////////////////////////////

void loop_hedgehog() {
  int incoming_byte;
  int total_received_in_loop;
  bool good_byte;
  byte packet_size;
  uni_8x2_16 un16;
  uni_8x4_32 un32;

  //read full double buffer
  //this method wastes bytes
  int bytesRead= readFullBuffer1( hedgehog_serial_buf, HEDGEHOG_BUF_SIZE);
/*
  if (OPMODE & DEBUG) {
    Serial.print( "\nFull Buffer length: ");
    Serial.println( bytesRead);
  }
*/

  //find header bytes of the HiRes or Beacon Packets, only need one beaconPacket
  hiResType= false;
  int bufndx= 0;
  //make sure there is room for a whole packet
  for ( ; bufndx < bytesRead- (hiResPacketSize+6); bufndx++) {
    //we only care about the HiResPackets now
    if ((hedgehog_serial_buf[bufndx] == dataPacket.destAddr) &&
      (hedgehog_serial_buf[bufndx+1] == dataPacket.packetType) &&
      (hedgehog_serial_buf[bufndx+2] == dataPacket.dataCode.b[0]) &&
      (hedgehog_serial_buf[bufndx+3] == dataPacket.dataCode.b[1]) &&
      (hedgehog_serial_buf[bufndx+4] == dataPacket.dataBytes))  {
      //now we have found a HiResPacket, so skip out of search loop
      hiResType= true;
      break;
      }
  }  //end for search loop


  //transfer packet into Packet structures
  if ( hiResType) {
    transferHiResPacket( &(hedgehog_serial_buf[bufndx]));
    if (OPMODE & DEBUG) {
      printHiResPacket();
    }
  } else {
    //unknown/unhandled packet type, reset for next loop_hedgehog()
    clearReadBuffer();
    return;
  }
  //all data transfered, so setup for next step

  //CALL AMBULATE FUNCTIONS TO MOVE MOWER  //TODO: team needs to write this/these
  //you may not want to perform movement commands with each and every packet,
  //establish some reasonable bracketing limits within which there will be no
  //movement commands and beyond which there will be.
  //This may have to be trial and error.
  if(CurrentC == 0){
  RowCreate();
  }
  
  checkIncrementCP();
  thetaAdjust();
  if(ta >= tol){
    AdjustPos(); 
    
  }  
  if(Cpx == 0 && CPy == 0){ // TODO Conditional telling if current checkpoint is the Endpoint, and we're at it
                            // TODO Check the data types for the CPx & CPy when unitialized
  // if (Cpx
      turnOneEighty;  
      CurrentC = 0; // Reinitialize the Checkpoints
  } else {  // TODO insert some conditional to control forward movement
    Forward(20); // TODO make travel distance equal to distance between CPs
  }
  

  

  //clear buffer and any other data for next read
  clearReadBuffer();

  //exit loop for reading next hedge location
}  //void loop_hedgehog()


//Just a debug print function for complete data packets
void printHiResPacket() {
  if( OPMODE & DEBUG) {
    //header is mostly common between packet types
    Serial.print( "0x");
    Serial.print( dataPacket.destAddr, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( dataPacket.packetType, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( dataPacket.dataCode.w, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( dataPacket.dataBytes, HEX);
    Serial.print( ", ");

    Serial.print( dataPacket.timeStamp.v32);
    Serial.print( ", ");
    Serial.print( dataPacket.x);
    Serial.print( ", ");
    Serial.print( dataPacket.y);
    Serial.print( ", ");
    Serial.print( dataPacket.z);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( dataPacket.flags, HEX);
    Serial.print( ", ");
    Serial.print( dataPacket.hedgeAddr);
    Serial.print( ", ");
    Serial.print( dataPacket.hedgeOrientation.w);
    Serial.print( ", ");
    Serial.print( dataPacket.timePassed.w);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( dataPacket.CRC_16.w, HEX);
    if ( !beaconRead) Serial.print( "   NO BEACON YET.");
    Serial.print( "\n");
  }
}

//Just a debug print function for complete data packets
void printBeaconPacket() {
  if( OPMODE & DEBUG) {

    Serial.print( "\nBEACON:\n");
    //header is mostly common between packet types
    Serial.print( "0x");
    Serial.print( beaconPacket.destAddr, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.packetType, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.dataCode.w, HEX);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.dataBytes, HEX);
    Serial.print( ", ");

    //unique part
    Serial.print( beaconPacket.numBeacons);
    Serial.print( ", ");
    Serial.print( beaconPacket.beacon1);
    Serial.print( ", ");
    Serial.print( beaconPacket.xb1);
    Serial.print( ", ");
    Serial.print( beaconPacket.yb1);
    Serial.print( ", ");
    Serial.print( beaconPacket.zb1);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.reserved1, HEX);
    Serial.print( ", ");
    Serial.print( beaconPacket.beacon2);
    Serial.print( ", ");
    Serial.print( beaconPacket.xb2);
    Serial.print( ", ");
    Serial.print( beaconPacket.yb2);
    Serial.print( ", ");
    Serial.print( beaconPacket.zb2);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.reserved2, HEX);
    Serial.print( ", ");
    Serial.print( beaconPacket.beacon3);
    Serial.print( ", ");
    Serial.print( beaconPacket.xb3);
    Serial.print( ", ");
    Serial.print( beaconPacket.yb3);
    Serial.print( ", ");
    Serial.print( beaconPacket.zb3);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.reserved3, HEX);
    Serial.print( ", ");
    Serial.print( beaconPacket.beacon4);
    Serial.print( ", ");
    Serial.print( beaconPacket.xb4);
    Serial.print( ", ");
    Serial.print( beaconPacket.yb4);
    Serial.print( ", ");
    Serial.print( beaconPacket.zb4);
    Serial.print( ", ");
    Serial.print( "0x");
    Serial.print( beaconPacket.reserved4, HEX);
    Serial.print( ", ");

    Serial.print( "0x");
    Serial.print( beaconPacket.CRC_16.w, HEX);
    Serial.println();
  }
}

// Calculate CRC-16 of hedgehog packet, and drop it into the buffer
unsigned int hedgehog_set_crc16(byte *buf, byte size)
{ uni_8x2_16 sum;
byte shift_cnt;
byte byte_cnt;

sum.w = 0xffffU;

for (byte_cnt = size; byte_cnt > 0; byte_cnt--)
{
  sum.w = (unsigned int) ((sum.w / 256U) * 256U + ((sum.w % 256U) ^ (buf[size - byte_cnt])));

  for (shift_cnt = 0; shift_cnt < 8; shift_cnt++)
  {
    if ((sum.w & 0x1) == 1) sum.w = (unsigned int)((sum.w >> 1) ^ 0xa001U);
    else sum.w >>= 1;
  }
}

buf[size] = sum.b[0];
buf[size + 1] = sum.b[1]; // little endian

dataPacket.CRC_16= sum;

//if ( OPMODE & DEBUG) { Serial.println( "Done crc."); Serial.flush();}
return sum.w;
}// hedgehog_set_crc16

//////////////////////////////////////////////
//  END OF MARVELMIND HEDGEHOG RELATED PART //
//////////////////////////////////////////////

////////////////////////////
// DO NOT EDIT MAIN SETUP //
////////////////////////////

void setup()
{
  Serial.begin(ARBAUDRATE);
  if ( OPMODE & DEBUG) {
    while (!Serial) {;} // wait for serial port to connect.
    //  Needed for native USB/COM port only
    Serial.println( "\nWaiting on data, Beacon search begins...");
    Serial.flush();

  }
  Serial2.begin(KRBAUDRATE);
  //  and replace references to MMSerial with Serial1 and vice-versa
  Serial1.begin(MMBAUDRATE);  //use Serial1 to avoid the SoftwareSerial library

  setup_hedgehog(); //MMSerial hedgehog support initialize

  microsPrevious = 0;
  packet_received= 0;

  if ( OPMODE & DEBUG) { Serial.println( "Setup done."); }//Serial.flush();}
} // end main setup

///////////////////////////
// DO NOT EDIT MAIN LOOP //
///////////////////////////

void loop() {
  // Use of delay(XXX); is not recommended as good Arduiono programming practice,
  //  use microsecond/time passage check instead
  //  if it's not time for work, then skip hedge loop
  //if ( OPMODE & DEBUG ) { Serial.println( "L+"); Serial.flush();}
  microsNow = micros();
  if ((microsNow - microsPrevious) >= microsPerReading) {
    if (firstLoop){
      loop_hedgehog();
      xOld = dataPacket.x;  // set old x and y position as hedge position after update for first hedge loop
      yOld = dataPacket.y;
      firstLoop = 0;
    }
    else{
      xOld = dataPacket.x; // set old x and y position as hedge position before update
      yOld = dataPacket.y;
      loop_hedgehog();// MMSerial hedgehog service loop
    }

    if ( OPMODE & DEBUG) {
/*
      if ( !packet_received) {
        Serial.print( "-END\n");
        Serial.flush();
      }
*/
    }
  }  //  end microsecond check
  microsPrevious = microsNow;  //just account for above work in msTime check
  //if ( OPMODE & DEBUG) { Serial.println( "L-"); Serial.flush();}
}  //end void loop()
 
 
//////////////////////////////////////////
// MOVEMENT AND PATH CREATION FUNCTIONS //
//////////////////////////////////////////

// TODO Test this function
// TODO call this in relevant loop_hedgehog position
void RowCreate(){ // TODO use unit vectors instead of trig
// PathCreate Creates a series of checkpoints along the current path that the mower should follow. 
// The path will be created 
  // NOTE: Need to Clear CPs array evertime this command is initialized  
  long RowMag = (sqrt( (Endx-dataPacket.x)^2 + (Endy-dataPacket.y)^2 ));
  long theta = round(acos((Endx-dataPacket.x)/RowMag)); //Radians
  int NumberOfCPs = ceil(RowMag / separation);   // Number of Checkpoints along the path
  // Need to Round up every time
 
  for(int ii=0; ii<=NumberOfCPs; ii++){

    // NOTE: May need to update with tolerances later
        
    if(ii == 0){ 
      // first point 
      CP[ii].x = dataPacket.x + separation*cos(theta);
      CP[ii].y = dataPacket.y + separation*sin(theta);    
    } else if(ii >= NumberOfCPs){
      // End Point
      CP[ii].x = Endx;
      CP[ii].y = Endy;
    } else{
       // everything after first point
      CP[ii].x = CP[ii-1].x + separation*cos(theta);
      CP[ii].y = CP[ii-1].y + separation*sin(theta);      
    }       
  }
  Cpx=CP[0].x;
  Cpy=CP[0].y;
  // currentC=0;
  return;
}

// TODO Correct TicksPerDegree
// TODO call this in relevant loop_hedgehog position
void turnOneEighty(){
  // Turning function for mower at end of each lap

  // This section may not be needed later
  //double ratio = 60; // Ticks / cycle
  //double diameter = 31.75; // cm
  //int dist = 95; //cm
  //double travel = 

  int Travel = round(90 * TicksPerDegree); // Ticks to make a 90 deg turn
    
  // Equivalent of If statement
  // Syntax 'ConditionalStatement' ? 'ValueifTrue' : 'ValueifFalse'
  Travel = LapNumber % 2 == 0 ? Travel : -1 * Travel;

  // Initiate 90 Deg Turn
  Turn.pi(Travel,encoderSpeed).wait(); // Initiate Turn
 

  // Back up a little
  Drive.pi(-20,encoderSpeed).wait();
  // May need to add virtual delay later
  // NOTE: Need to find correct backup distance by testing

  // Make another 90 Deg Turn
  Turn.pi(Travel,encoderSpeed).wait(); // Initiate Turn

  LapNumber++; // Increment Lap Number everytime a full turn occurs
  Endx = End[LapNumber].x;
  Endy = End[LapNumber].y;
  Forward();
  return;
}


// TODO Test this function
// TODO call this in relevant loop_hedgehog position
void Forward(long dist){
  Drive.pi(dist,encoderSpeed); //specific distance to travel at a given speed
  //ensure that this requires a forward command at regular intervals to continue
  //prevent mower from getting a mind of its own
  return;
}

// TODO Test this function
// TODO call this in relevant loop_hedgehog position
void AdjustPos(){
  //given a theta in degrees
  //turn by desired amount

  int Travel = round(ta * TicksPerDegree);
  Turn.pi(Travel,encoderSpeed).wait(); // Initiate Turn
  Forward(20); //start forward protocol after adjustment
  return;
}

// TODO Test this function
// TODO call this in relevant loop_hedgehog position
void checkIncrementCP(){
  // CP array
  // lapnumber = row number of CP matrix
  //Check w/ tolerance to Cpx & Cpy
  
  long mag = sqrt((Cpx-dataPacket.x)^2+(Cpy-dataPacket.y)^2); // Distance to CP
  if (mag <= tolerance){ //if distance between check point and current pos is less than tolerance do stuff
    //increment to next CP
    currentC++;  //increment Checkpoint array index
    Cpx=CP[currentC].x; // Change to next CP x value
    Cpy=CP[currentC].y; // change to next CP y value
  }
  return;
}

// TODO Test this function
// TODO call this in relevant loop_hedgehog position
void thetaAdjust(){
  float rActual = sqrt( (dataPacket.x - xOld)^2 + (dataPacket.y - yOld)^2); // distance traveled from last measurement
  float rDes = sqrt( (Cpx - dataPacket.x)^2 + (Cpy - dataPacket.y)^2); // distance to CP
  float tDes = asin( (Cpy - dataPacket.y)/rDes) * 180/Pi; //desired theta based on position and next Checkpoint in degrees
  float tActual = 180 - asin( (dataPacket.y - yOld)/rActual )*180/Pi; //actual trajectory theta in degrees
  ta = round( tDes - tActual ); //theta to adjust by to point toward Checkpoint
  return; // Angle to adjust by in degrees
}

// TODO Test this function
// TODO call this in setup_hedgehog
void offsetCreate(){ // Create offset positions between beacons for path creation
  // xb1, yb1, xb2, yb2, xb3, yb3, xb4, yb4 Beacon positions
  // r=offset/sqrt(2)
  // x1 x2 x3 x4 y1 y2 y3 y4  Offset beacon positions
  long r = offset/sqrt(2);
  long magOneTwo = sqrt((xb2-xb1)^2+(yb2-yb1)^2); 
  long magThreeFour = sqrt((xb4-xb3)^2+(yb4-yb3)^2);

  // Beacon unit vectors
  float TxOneTwo = (xb2-xb1)/magOneTwo; 
  float TyOneTwo = (yb2-yb1)/magOneTwo;
  float TxThreeFour = (xb4-xb3)/magThreeFour; 
  float TyThreeFour = (yb4-yb3)/magThreeFour;

  // Psuedo 
  
  position along 1-2 and 3-4
  long x1prime = xb1 + r*TxOneTwo;
  long y1prime = yb1 + r*TyOneTwo;
  long x2prime = xb2 - r*TxOneTwo;
  long y2prime = yb2 - r*TyOneTwo; 
  long x3prime = xb3 + r*TxThreeFour;
  long y3prime = yb3 + r*TyThreeFour;
  long x4prime = xb4 - r*TxThreeFour;
  long y4prime = yb4 - r*TyThreeFour;

  long magOneFour = sqrt((x4prime - x1prime)^2 + (y4prime - y1prime)^2);
  long magTwoThree = sqrt((x3prime - x2prime)^2 + (y3prime - y2prime)^2);

  // Psuedo Offset Unit Vectors
  float TxOneFour = (x4prime - x1prime)/magOneFour;
  float TyOneFour = (y4prime - y1prime)/magOneFour;
  float TxTwoThree = (x3prime - x2prime)/magTwoThree;
  float TyTwoThree = (y3prime - y2prime)/magTwoThree;

  // Final offset positions along 1prime - 4prime and 2prime - 3prime
  x1 = x1prime + r*TxOneFour;
  y1 = y1prime + r*TyOneFour;
  x4 = x4prime - r*TxOneFour;
  y4 = y4prime - r*TyOneFour;
  x2 = x2prime + r*TxTwoThree;
  y2 = y2prime + r*TyTwoThree;
  x3 = x3prime - r*TxTwoThree;
  y3 = y3prime - r*TyTwoThree;

  return;
}

// TODO Test this function
// TODO call this in setup_hedgehog
void createEndPoints() {
  long magOneFour = sqrt((x4-x1)^2+(y4-y1)^2);
  long magTwoThree = sqrt((x3-x2)^2+(y3-y2)^2);
  
  if (magOneFour >= magTwoThree){
    numberOfRows = ceil(magOneFour/rowOffset);  
  } else {
    numberOfRows = ceil(magTwoThree/rowOffset);
  }
  
  // unit vectors between 1-4 and 2-3
  float TxOneFour = (x4 - x1)/magOneFour; 
  float TyOneFour = (y4 - y1)/magOneFour;
  float TxTwoThree = (x3 - x2)/magTwoThree;
  float TyTwoThree = (y3 - y2)/magTwoThree;  

  //odd number of rows end point on 2-3 vector
  //even number of rows end point on 1-4 vector
  for (int ii=0; ii <= numberOfRows; ii++){
    if (ii==0) {
      End[ii].x = x2;
      End[ii].y = y2;
    } else if (ii == 1){
      End[ii].x = x1 + rowOffset*TxOneFour; // 2nd endpointx
      End[ii].y = y1 + rowOffset*TyOneFour; // 2nd endpointy
    } else if (ii < numberOfRows && ii % 2 == 0){ // for all other even row numbers
      End[ii].x = End[ii-2].x + 2*rowOffset*TxTwoThree;
      End[ii].y = End[ii-2].y + 2*rowOffset*TyTwoThree;
    } else if (ii < numberOfRows && ii % 2 != 0){ // for all other odd row numbers
      End[ii].x = End[ii-2].x + 2*rowOffset*TxOneFour;
      End[ii].y = End[ii-2].y + 2*rowOffset*TyOneFour;
    } else if (ii >= numberOfRows && (numberOfRows) % 2 == 0){ // Last row end point if # of rows is even
      End[ii].x = x3; 
      End[ii].y = y3;
    } else if (ii >= numberOfRows && (numberOfRows) % 2 != 0){ // Last row end point if # of rows is odd
      End[ii].x = x4;
      End[ii].y = y4;
    }// conditional end
  } // loop end
  Endx=End[0].x;
  Endy=End[0].y;
  return;
} // EndPoint function end
