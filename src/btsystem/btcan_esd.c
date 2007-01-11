/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcan_esd.c
 *  Author .............Brian Zenowich
 *  Creation Date ......24 Mar 2003
 *  Addtl Authors ......Traveler Hauptman
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *   This file must be linked against a closed-source proprietary driver
 *   library (libntcan) from esd electronics (http://esd-electronics.com).
 *
 *  REVISION HISTORY:
 *  24 Mar 2003 - BZ
 *    File created & documented. 
 *  16 Dec 2004 - BZ, TH
 *    Initial port to linux + RTAI
 *                                                                      *
 *======================================================================*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <errno.h>
//th041216#include <sys/syspage.h>
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <linux/version.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "ntcan.h"
#include "btcan.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

#define MAX_BUS             (4)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )



#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)



/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
typedef unsigned long DWORD;

/*==============================*
* GLOBAL file-scope variables  *
*==============================*/
HANDLE              canDev[MAX_BUS];
pthread_mutex_t     commMutex;

/* keyword, index */
const struct propTxtStruct propTxt[]={
   { "OFFSET1", OFFSET1 }, /* Gimbals offset 1 (Q4.12 rad) */
   { "OFFSET2", OFFSET2 }, /* Gimbals offset 2 (Q4.12 rad) */
   { "OFFSET3", OFFSET3 }, /* Gimbals offset 3 (Q4.12 rad) */
   { "TENSION", TENSION }, /* Tensioner output: 0=Off, 1=On */
   { "VALUE32", VALUE32 }, /* 32-bit value to poke/peek */
   { "IFAULT", IFAULT }, /* Ignore fault count */
   { "ILOGIC", ILOGIC }, /* Logic current (tbd) */
   { "IMOTOR", IMOTOR }, /* Motor current (mA) */
   { "MAXPWR", MAXPWR }, /* Max allowed power (tbd) */
   { "SAMPLE", SAMPLE }, /* Sample time (tbd) */
   { "UPSECS", UPSECS }, /* Up seconds in operation (tbd) */
   { "VLOGIC", VLOGIC }, /* Logic voltage (tbd) */
   { "VOLTH1", VOLTH1 }, /* Voltage high warning level (safety) */
   { "VOLTH2", VOLTH2 }, /* Voltage high critical level (safety) */
   { "VOLTL1", VOLTL1 }, /* Voltage low warning level (safety) */
   { "VOLTL2", VOLTL2 }, /* Voltage low critical level (safety) */
   { "ACCEL", ACCEL }, /* Acceleration (Q8.8 cts/ms/ms) */
   { "ECMAX", ECMAX }, /* Encoder correction max value */
   { "ECMIN", ECMIN }, /* Encoder correction min value */
   { "ERROR", ERROR }, /* Error (tbd) */
   { "GAIN1", GAIN1 }, /* Gimbals/safety gain 1 (Q4.12 rad/3V) */
   { "GAIN2", GAIN2 }, /* Gimbals/safety gain 2 (Q4.12 rad/3V) */
   { "GAIN3", GAIN3 }, /* Gimbals/safety gain 3 (Q4.12 rad/3V) */
   { "HALLH", HALLH }, /* Hall history bitfield */
   { "HALLS", HALLS }, /* Hall feedback bitfield: CBA */
   { "IKCOR", IKCOR }, /* Current sense correction factor */
   { "IOFST", IOFST }, /* Current offset calibration */
   { "MOFST", MOFST }, /* Mechanical offset calibration */
   { "OTEMP", OTEMP }, /* Over temperature alarm (tbd) */
   { "POLES", POLES }, /* Number of magnets on rotor */
   { "PTEMP", PTEMP }, /* Peak temperature recorded (tbd) */
   { "RATIO", RATIO }, /* Output angle multiplier (tbd) */
   { "TENST", TENST }, /* Tension total (tbd) */
   { "TENSO", TENSO }, /* Tension offset (tbd) */
   { "TETAE", TETAE }, /* Motor electrical angle (write requires LOCK) */
   { "THERM", THERM }, /* Thermistor (motor) temperature */
   { "TSTOP", TSTOP }, /* Time until considered stopped */
   { "UNITS", UNITS }, /* Units of input angle (tbd) */
   { "VALUE", VALUE }, /* Value to poke/peek */
   { "ADDR", ADDR }, /* Address to peek/poke */
   { "ANA0", ANA0 }, /* Analog input (pin 4:0-3V, 3:Gnd) */
   { "ANA1", ANA1 }, /* Analog input (pin 2:0-3V, 3:Gnd) */
   { "BAUD", BAUD }, /* Baud rate */
   { "DIG0", DIG0 }, /* Dig I/O: -1=In,0=Lo,1=Hi,2-100=%PWM (pin 41:0-3.3V, 44:Gnd) */
   { "DIG1", DIG1 }, /* Dig I/O: -1=In,0=Lo,1=Hi (pin 43:0-3.3V, 44:Gnd) */
   { "DUMP", DUMP }, /* Log dump mode: 0=Manual, 1=Auto */
   { "FIND", FIND }, /* Find command for CAN */
   { "GRPA", GRPA }, /* Comm group A */
   { "GRPB", GRPB }, /* Comm group B */
   { "GRPC", GRPC }, /* Comm group C */
   { "IOFF", IOFF }, /* Initialization offset */
   { "IPNM", IPNM }, /* CommandedCurrent / Nm (ratio) */
   { "IVEL", IVEL }, /* Initialization velocity (tbd) */
   { "JIDX", JIDX }, /* Joint index */
   { "LCVC", LCVC }, /* Loop control velocity coefficient */
   { "LFAP", LFAP }, /* Loop feedback block contains absolute position */
   { "LFDP", LFDP }, /* Loop feedback block contains delta position */
   { "LOAD", LOAD }, /* Load command for CAN */
   { "LOCK", _LOCK }, /* Lock */
   { "LOG1", LOG1 }, /* Log variable address 1 */
   { "LOG2", LOG2 }, /* Log variable address 2 */
   { "LOG3", LOG3 }, /* Log variable address 3 */
   { "LOG4", LOG4 }, /* Log variable address 4 */
   { "MECH", MECH }, /* Mechanical angle (cts) */
   { "MODE", MODE }, /* Mode: 0=Idle, 2=Torque, 3=PID, 4=Vel, 5=Trap */
   { "PIDX", PIDX }, /* Puck index for torque */
   { "ROLE", ROLE }, /* Role: 0=Tater, 1=Gimbals, 2=Safety, 3=Wraptor */
   { "SAFE", SAFE }, /* Safety debug bitfield: Safe/Shunt/Bus (safety) */
   { "SAVE", SAVE }, /* Save command for CAN */
   { "STAT", STAT }, /* Status: 0=Reset/Monitor, 2=Ready/Main */
   { "TEMP", TEMP }, /* Temperature (puck internal) */
   { "TORQ", TORQ }, /* Torque command */
   { "VBUS", VBUS }, /* Bus voltage (V) */
   { "VERS", VERS }, /* Firmware version */
   { "VNOM", VNOM }, /* Nominal voltage (safety) */
   { "ZERO", ZERO }, /* Zeroed status */
   { "CTS", CTS }, /* Counts per revolution */
   { "DEF", DEF }, /* Default command for CAN */
   { "HSG", HSG }, /* High strain gage (tbd) */
   { "IKI", IKI }, /* Current sense integral gain */
   { "IKP", IKP }, /* Current sense proportional gain */
   { "ISQ", ISQ }, /* iSq current sense for debugging */
   { "LCV", LCV }, /* Loop control block contains velocity */
   { "LFS", LFS }, /* Loop feedback block contains strain */
   { "LFT", LFT }, /* Loop feedback block contains temperature */
   { "LFV", LFV }, /* Loop feedback block contains velocity */
   { "LOG", LOG }, /* Log status: 0=Off, 1=Once, 2=Continuous */
   { "LSG", LSG }, /* Low strain gage (tbd) */
   { "MCV", MCV }, /* Max close velocity (cts/ms) */
   { "MDS", MDS }, /* Max duty sum for power limiting (tbd) */
   { "MOV", MOV }, /* Max open velocity (cts/ms) */
   { "MPE", MPE }, /* Max position error (tbd) */
   { "PEN", PEN }, /* Pendant: -65='A', -2=Reset, -1=Clear, >0=Light */
   { "PWR", PWR }, /* Observed power (tbd) */
   { "TL1", TL1 }, /* Torque warning level (safety) */
   { "TL2", TL2 }, /* Torque critical level (safety) */
   { "VL1", VL1 }, /* Velocity warning level (safety, Q4.12 m/s, rad/s) */
   { "VL2", VL2 }, /* Velocity critical level (safety, Q4.12 m/s, rad/s) */
   { "AP", AP }, /* Actual position (cts) */
   { "CT", CT }, /* Close torque */
   { "DP", DP }, /* Default position */
   { "DS", _DS }, /* Default step */
   { "EN", EN }, /* Enable bitfield */
   { "ID", ID }, /* CANbus ID */
   { "KD", KD }, /* Differential gain */
   { "KI", KI }, /* Integral gain */
   { "KP", KP }, /* Proportional gain */
   { "MT", MT }, /* Max torque */
   { "MV", MV }, /* Max velocity (cts/ms) */
   { "OD", OD }, /* Odometer (tbd) */
   { "OT", OT }, /* Open torque */
   { "SG", SG }, /* Strain gage (tbd) */
   { "SN", SN }, /* Serial number */
   { "B", B }, /* Brake: 0=Off, 1=On */
   { "E", E }, /* Endpoint target (cts) */
   { "P", P }, /* Position command (cts) */
   { "V", V }, /* Velocity command (cts/ms) */

   { "",  0  }
};


const char* Prop2Name(int prop)
{
   int cnt;

   cnt = 0;
   while (cnt < PROP_END && prop != propTxt[cnt].idx) {
      cnt++;
   }
   return propTxt[cnt].key;
}

int Name2Prop(char *name)
{
   int cnt;

   cnt = 0;
   while (cnt < PROP_END && strcmp(propTxt[cnt].key,name)!= 0) {
      cnt++;
   }
   return propTxt[cnt].idx;
}

/* keyword, index, readFunction, writeFunction, defaultVal, type */
const int dataType[]=
{
   /* VERS */  L16 ,
   /* ROLE */ L16 | EE ,
   /* SN */ L16 | EE ,
   /* ID */ L16 | EE ,
   /* ERROR */ L16 ,
   /* STAT */ L16 ,
   /* ADDR */ L16 ,
   /* VALUE */ L16 ,
   /* MODE */ L16 ,
   /* TORQ */ L16 ,
   /* V */ L16 ,
   /* B */ L16 ,
   /* P */ L32 ,
   /* P2 */ L16 ,
   /* E */ L32 ,
   /* E2 */ L16 ,
   /* MT */ L16 | EE ,
   /* MV */ L16 | EE ,
   /* MCV */ L16 | EE ,
   /* MOV */ L16 | EE ,
   /* MOFST */ L16 | EE ,
   /* IOFST */ L16 | EE ,
   /* PTEMP */ L16 | EE ,
   /* UPSECS */ L16 | EE ,
   /* OD */ L16 | EE ,
   /* MDS */ L16 | EE ,
   /* AP */ L32 | EE ,
   /* AP2 */ L16 ,
   /* MECH */ L32 ,
   /* MECH2 */ L16 ,
   /* CTS */ L32 | EE ,
   /* CTS2 */ L16 ,
   /* DP */ L32 | EE ,
   /* DP2 */ L16 ,
   /* OT */ L32 | EE ,
   /* OT2 */ L16 ,
   /* CT */ L32 | EE ,
   /* CT2 */ L16 ,
   /* BAUD */ L16 ,
   /* TEMP */ L16 ,
   /* OTEMP */ L16 ,
   /* LOCK */ L16 ,
   /* DIG0 */ L16 ,
   /* DIG1 */ L16 ,
   /* ANA0 */ L16 ,
   /* ANA1 */ L16 ,
   /* THERM */ L16 ,
   /* VBUS */ L16 ,
   /* IMOTOR */ L16 ,
   /* VLOGIC */ L16 ,
   /* ILOGIC */ L16 ,
   /* GRPA */ L16 | EE ,
   /* GRPB */ L16 | EE ,
   /* GRPC */ L16 | EE ,
   /* PIDX */ L16 | EE ,
   /* ZERO */ L16 ,
   /* SG */ L16 ,
   /* HSG */ L16 | EE ,
   /* LSG */ L16 | EE ,
   /* DS */ L16 | EE ,
   /* IVEL */ L16 | EE ,
   /* IOFF */ L16 | EE ,
   /* MPE */ L16 | EE ,
   /* EN */ L16 ,
   /* TSTOP */ L16 | EE ,
   /* KP */ L16 | EE ,
   /* KD */ L16 | EE ,
   /* KI */ L16 | EE ,
   /* SAMPLE */ L16 | EE ,
   /* ACCEL */ L16 | EE ,
   /* TENSION */ L16 ,
   /* UNITS */ L16 | EE ,
   /* RATIO */ L16 | EE ,
   /* LOG */ L16 ,
   /* DUMP */ L16 ,
   /* LOG1 */ L16 ,
   /* LOG2 */ L16 ,
   /* LOG3 */ L16 ,
   /* LOG4 */ L16 ,
   /* GAIN1 */ L16 | EE ,
   /* GAIN2 */ L16 | EE ,
   /* GAIN3 */ L16 | EE ,
   /* OFFSET1 */ L16 | EE ,
   /* OFFSET2 */ L16 | EE ,
   /* OFFSET3 */ L16 | EE ,
   /* PEN */ L16 ,
   /* SAFE */ L16 ,
   /* SAVE */ L16 ,
   /* LOAD */ L16 ,
   /* DEF */ L16 ,
   /* VL1 */ L16 ,
   /* VL2 */ L16 ,
   /* TL1 */ L16 ,
   /* TL2 */ L16 ,
   /* VOLTL1 */ L16 | EE ,
   /* VOLTL2 */ L16 | EE ,
   /* VOLTH1 */ L16 | EE ,
   /* VOLTH2 */ L16 | EE ,
   /* MAXPWR */ L16 ,
   /* PWR */ L16 ,
   /* IFAULT */ L16 ,
   /* IKP */ L16 | EE ,
   /* IKI */ L16 | EE ,
   /* IKCOR */ L16 | EE ,
   /* VNOM */ L16 ,
   /* TENST */ L16 | EE ,
   /* TENSO */ L16 | EE ,
   /* JIDX */ L16 | EE ,
   /* IPNM */ L16 | EE ,
   /* HALLS */ L16 ,
   /* HALLH */ L32 ,
   /* HALLH2 */ L16 ,
   /* POLES */ L16 | EE ,
   /* ECMAX */ L16 ,
   /* ECMIN */ L16 ,
   /* ISQ */ L16 ,
   /* TETAE */ L16 ,
   /* FIND */ L16 ,
   /* LCV */ L16 ,
   /* LCVC */ L16 ,
   /* LFV */ L16 ,
   /* LFS */ L16 ,
   /* LFAP */ L16 ,
   /* LFDP */ L16 ,
   /* LFT */ L16 ,
   /* VALUE32 */ L16
};

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
int compile(int property, long longVal, unsigned char *data, int *dataLen, int isForSafety);
int parseMessage(int id, int len, unsigned char *messageData, int *node, int *property, long *value);
int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking);
int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking);

/*==============================*
 * Functions                    *
 *==============================*/
void allowMessage(int bus, int id, int mask)
{
   int i;
   for(i = 0; i < 2048; i++)
      if((i & ~mask) == id)
         canIdAdd(canDev[bus], i);
}

int initCAN(int bus)
{
   DWORD retvalue;

   pthread_mutex_init(&commMutex, NULL);

   retvalue = canOpen(bus, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);
   if(retvalue != NTCAN_SUCCESS) {
      syslog(LOG_ERR, "initCAN(): canOpen() failed with error %d", retvalue);
      return(1);
   }

   retvalue = canSetBaudrate(canDev[bus], 0); // 1 = 1Mbps, 2 = 500kbps, 3 = 250kbps
   if(retvalue != 0) {
      syslog(LOG_ERR, "initCAN(): canSetBaudrate() failed with error %d", retvalue);
      return(1);
   }

   // Mask 3E0: 0000 0011 1110 0000
   allowMessage(bus, 0x0000, 0x03E0); // Messages sent directly to host
   allowMessage(bus, 0x0403, 0x03E0); // Group 3 messages
   allowMessage(bus, 0x0406, 0x03E0); // Group 6 messages

   return(0);
}

void freeCAN(int bus)
{
   canClose(canDev[bus]);
}

int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking)
{
   CMSG    msg;
   DWORD   retvalue;
   long    msgCt = 1;
   int     i;

   if(blocking) {
      retvalue = canRead(canDev[bus], &msg, &msgCt, NULL);
   } else {
      retvalue = canTake(canDev[bus], &msg, &msgCt);
   }
   if(retvalue != NTCAN_SUCCESS) {
      syslog(LOG_ERR, "canReadMsg(): canRead/canTake error: %ld", retvalue);
      if(retvalue == NTCAN_RX_TIMEOUT)
         return(1);
      else
         return(2);
   }
   if(msgCt == 1) {
      *id = msg.id;
      *len = msg.len;
      for(i = 0; i < msg.len; i++)
         data[i] = msg.data[i];

      return(0);
   }

   return(1); // No message received, return err
}

int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking)
{
   CMSG    msg;
   DWORD   retvalue;
   long    msgCt = 1;
   int     i;

   msg.id = id;
   msg.len = len & 0x0F;
   for(i = 0; i < len; i++)
      msg.data[i] = data[i];

   if(blocking) {
      retvalue = canWrite(canDev[bus], &msg, &msgCt, NULL);
   } else {
      retvalue = canSend(canDev[bus], &msg, &msgCt);
   }

   if(retvalue != NTCAN_SUCCESS) {
      syslog(LOG_ERR, "canSendMsg(): canWrite/Send() failed with error %d", retvalue);
      return(1);
   }
}

int wakePuck(int bus, int who)
{
   setProperty(bus, who, STAT, FALSE, STATUS_READY);
   usleep(300000); // Wait 300ms for puck to initialize

   return(0);
}

int setTorques(int bus, int group, int *values)
{
   unsigned char   data[8];
   int             err;
   int             i;

   /* Bound the torque commands */
   for (i = 0; i < 4; i++) {
      values[i] = Border(values[i], -8191, 8191);
   }

   /* Special value-packing compilation: Packs (4) 14-bit values into 8 bytes */
   /*     0        1        2        3        4        5        6        7    */
   /* ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd */

   data[0] = TORQ | 0x80; /* Set the "Set" bit */
   data[1] = (unsigned char)(( values[0] >> 6) & 0x00FF);
   data[2] = (unsigned char)(((values[0] << 2) & 0x00FC) | ((values[1] >> 12) & 0x0003) );
   data[3] = (unsigned char)(( values[1] >> 4) & 0x00FF);
   data[4] = (unsigned char)(((values[1] << 4) & 0x00F0) | ((values[2] >> 10) & 0x000F) );
   data[5] = (unsigned char)(( values[2] >> 2) & 0x00FF);
   data[6] = (unsigned char)(((values[2] << 6) & 0x00C0) | ((values[3] >> 8) & 0x003F) );
   data[7] = (unsigned char)( values[3] & 0x00FF);

   // Send the data
   pthread_mutex_lock(&commMutex);
   err = canSendMsg(bus, GROUPID(group), 8, data, TRUE);
   pthread_mutex_unlock(&commMutex);
}

int getPositions(int bus, int group, int howMany, long *pos)
{
   int             err;
   unsigned char   data[8];
   int             len;
   int             msgID;
   int             id;
   int             property;
   long            reply;

   // Compile the packet
   data[0] = (unsigned char)AP;

   pthread_mutex_lock(&commMutex);

   // Send the packet
   err = canSendMsg(bus, GROUPID(group), 1, data, TRUE);

   // Wait for each reply
   while(howMany) {
      err = canReadMsg(bus, &msgID, &len, data, TRUE);

      // If no error
      if(!err) {
         // Parse the reply
         err = parseMessage(msgID, len, data, &id, &property, &reply);
         if(property == AP) {
            pos[id] = reply;
            --howMany;
         } else {
            syslog(LOG_ERR, "getPositions(): Asked group %d for position, received property %d = %ld from id %d", property, reply, id);
         }
      } else { 
         // Timeout or other error
         pthread_mutex_unlock(&commMutex);
         return(err);
      }
   }
   
   pthread_mutex_unlock(&commMutex);
   
   return(0);
}

int setProperty(int bus, int id, int property, int verify, long value)
{
   long            response;
   unsigned char   data[8];
   int             len;
   int             err;

   //syslog(LOG_ERR, "About to compile setProperty, property = %d", property);
   // Compile 'set' packet
   err = compile(property, value, data, &len, id == SAFETY_MODULE);

   //syslog(LOG_ERR, "After compilation data[0] = %d", data[0]);
   data[0] |= 0x80; // Set the 'Set' bit

   // Send the packet
   pthread_mutex_lock(&commMutex);
   err = canSendMsg(bus, (id & 0x0400) ? id : NODE2ADDR(id), len, data, TRUE);
   pthread_mutex_unlock(&commMutex);

   // BUG: This will not verify properties from groups of pucks
   if(verify) {
      // Get the new value of the property
      getProperty(bus, id, property, &response);

      // Compare response to value
      if(response != value)
         return(1);
   }
   return(0);
}

int getProperty(int bus, int id, int property, long *reply)
{
   int err;
   unsigned char data[8];
   int len_in;
   int id_in;
   int property_in;

   // Compile the packet
   data[0] = (unsigned char)property;

   pthread_mutex_lock(&commMutex);

   // Send the packet
   err = canSendMsg(bus, NODE2ADDR(id), 1, data, TRUE);

   // Wait for 1 reply
   err = canReadMsg(bus, &id_in, &len_in, data, TRUE);

   pthread_mutex_unlock(&commMutex);

   if(!err) {
      // Parse the reply
      err = parseMessage(id_in, len_in, data, &id_in, &property_in, reply);

      // Check that the id and property match
      if((id == id_in) && (property == property_in))
         return(0);
      else {
         syslog(LOG_ERR, "getProperty(): returned id or property do not match");
         return(1);
      }
   } else {
      syslog(LOG_ERR, "getProperty(): canReadMsg error = %d", err);
      return(1);
   }
}

int getBusStatus(int bus, long *status)
{
   int err;
   unsigned char data[8];
   int id;
   int len_in;
   int id_in;
   int property_in;

   pthread_mutex_lock(&commMutex);
   //err = canReadMsg(bus, &id_in, &len_in, data, FALSE);

   for(id = 0; id < MAX_NODES; id++) {
      // Compile the packet
      data[0] = (unsigned char)STAT;

      // Initialize status to "NOT_FOUND"
      status[id] = -1;

      // Send the packet
      err = canSendMsg(bus, NODE2ADDR(id), 1, data, TRUE);

      // Wait 1ms
      usleep(1000);

      // Try to get 1 reply (non-blocking read)
      err = canReadMsg(bus, &id_in, &len_in, data, FALSE);

      // If no error
      if(!err) {
         // Parse the reply
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
         err = parseMessage(id_in, len_in, data, &id_in, &property_in, &status[id]);
#else

         err = parseMessage(id_in, len_in, data, &id_in, &property_in, &status[id]);
#endif

      } else
         syslog(LOG_ERR, "getBusStatus(): canReadMsg returned error");
   }

   pthread_mutex_unlock(&commMutex);
#if BTDEBUG <= 8

   syslog(LOG_ERR,"getBusStatus: Only status != -1 is shown.");
   for(id = 0; id < MAX_NODES; id++) {
      if (status[id] != -1)
         syslog(LOG_ERR,"getBusStatus: status[%d] = %d", id, status[id]);
   }
#endif
}

/** Parse the data payload received from a Barrett Motor Controller.
    Allows selection of the CAN controller.
    
    \return 0 for no error
    \return 1 for <illegal message header> (syslog output is generated)
   
   \verbatim
   THIS (OPTIONAL) INFO WILL BE PLACED IN A GREY BOX AS PRE-FORMATTED TEXT
 
     You may draw simple diagrams explaining key concepts this way:
     
              -----Head-----
              |            |
          --Node1--      Node2
          |       |
        Sub1     Sub2
        
   \endverbatim  
*/
int parseMessage(
   /* Input */
   int id                      /** The message ID */,
   int len                     /** The data payload length */,
   unsigned char *messageData  /** Pointer to the message data payload */,

   /* Output */
   int *node       /** The controller node ID of the received message */,
   int *property   /** The property this message applies to */,
   long *value     /** The value of the property being processed */)
{
   int i;
   int dataHeader;

   *node = ADDR2NODE(id);
   if (*node == -1)
      syslog(LOG_ERR,"msgID:%x ",id);
   dataHeader = ((messageData[0] >> 6) & 0x0002) | ((id & 0x041F) == 0x0403);
   //messageData[0] &= 0x7F;
   //syslog(LOG_ERR,"Entering parsemessage");
   switch (dataHeader) {
   case 3:  /* Data is a packed 22-bit position, SET */
      *value = 0x00000000;
      *value |= ( (long)messageData[0] << 16) & 0x003F0000;
      *value |= ( (long)messageData[1] << 8 ) & 0x0000FF00;
      *value |= ( (long)messageData[2] ) & 0x000000FF;

      if (*value & 0x00200000) /* If negative */
         *value |= 0xFFC00000; /* Sign-extend */

      *property = AP;
      //syslog(LOG_ERR,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);
      break;
   case 2:  /* Data is normal, SET */
      *property = messageData[0] & 0x7F;
      //syslog(LOG_ERR, "Received property: %d", *property);
      /* Store the value, second byte of message is zero (for DSP word alignment) */
      *value = 0;
      for (i = 0; i < len - 2; i++)
         *value |= ((unsigned long)messageData[i + 2] << (i * 8))
                   & (0x000000FF << (i * 8));

      if (*value & (1 << ((i*8) - 1)))
         *value |= 0xFFFFFFFF << (i * 8); /* Sign extend the value */

      //syslog(LOG_ERR, "Received normal set property: %d from node: %d value:%d", *property, *node, *value);
      //syslog(LOG_ERR,"parsemessage after %d",value);
      break;
   case 0:  /* Assume firmware request (GET) */
         *property = -(messageData[0] & 0x7F); /* A negative (or zero) property means GET */
      *value = 0;
      //syslog(LOG_ERR, "Received normal get property: %d from node: %d value:%d", *property, *node, *value);
      break;
   default:
         syslog(LOG_ERR, "<Illegal Message Header> %d\n", dataHeader);
      return(1);
   }
   //if (*property != 8) syslog(LOG_ERR,"Value in parsemessage is: %d",*value);
   return (0);

}

/** Convert a property and value into a valid btcan packet.
    Used by getProperty() and setProperty() to build the data payload 
    section of a CAN message based on a given property and value.
    
    \return 0 for success
    \return non-zero, otherwise
   
*/
int compile(
   int property        /** The property being compiled (use the enumerations in btcan.h) */,
   long longVal        /** The value to set the property to */,
   unsigned char *data /** A pointer to a character buffer in which to build the data payload */,
   int *dataLen        /** A pointer to the total length of the data payload for this packet */,
   int isForSafety        /** A flag indicating whether this packet is destined for the safety circuit or not */)
{
   int i;

   // Check the property
   if(property > PROP_END) {
      syslog(LOG_ERR,"compile(): Invalid property = %d", property);
      return(1);
   }

   /* Insert the property */
   data[0] = property;
   data[1] = 0; /* To align the values for the tater's DSP */

   /* Append the value */
   for (i = 2; i < 6; i++) {
      data[i] = (char)(longVal & 0x000000FF);
      longVal >>= 8;
   }

   /* Record the proper data length */
   *dataLen = (dataType[property] & 0x0007) + 2;

   //if (i & 0x0003) *dataLen = 3; /* 8-bits */
   //else if (i & 0x000C) *dataLen = 4; /* 16-bits */
   //else if (i & 0x0030) *dataLen = 5; /* 24-bits */
   //else if (i & 0x00C0) *dataLen = 6; /* 32-bits */

   return (0);
}

