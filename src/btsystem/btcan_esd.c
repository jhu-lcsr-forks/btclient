/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcan.c
 *  Author .............Brian Zenowich
 *  Creation Date ......24 Mar 2003
 *  Addtl Authors ......Traveler Hauptman
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  24 Mar 2003 - BZ
 *    File created & documented. 
 *  16 Dec 2004 - BZ, TH
 *    Initial port to linux + RTAI
 *                                                                      *
 *======================================================================*/

/** \file btcan.c
    Handles all communication with the robot over the CAN bus.
    Requires library files "libcan.a" and "libmitop.a".
    
 */


/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <errno.h>
//th041216#include <sys/syspage.h>
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>

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

#define mbxID               (0)
#define BASE_ID             (0)

#define MAX_BUS             (4)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

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

/* keyword, index, readFunction, writeFunction, defaultVal, type */
const int dataType[]={
     /* "VERS",    VERS,*/     L16 ,
     /* "ROLE",    ROLE,*/     L16 | EE ,
     /* "SN",      SN, */      L16 | EE ,
     /* "ID",      ID,*/       L16 | EE ,
     /* "ERR",     ERR,*/      L16 ,
     /* "STAT",    STAT,*/     L16 ,
     /* "ADDR",    ADDR,*/     L16 ,
     /* "VALUE",   VALUE,*/    L16 ,
     /* "MODE",    MODE,*/     L16 ,
     /* "D",       D,*/        L16 ,
     /* "TORQ",    TORQ,*/     L16 ,
     /* "P",       P,*/        L16 ,
     /* "V",       V,*/        L16 ,
     /* "E",       E,*/        L16 ,
     /* "B",       B,*/        L16 ,
     /* "MD",      MD,*/       L16 ,
     /* "MT",      MT, */      L16 | EE ,
     /* "MV",      MV, */      L16 | EE ,
     /* "MCV",     MCV,*/      L16 | EE ,
     /* "MOV",     MOV,*/      L16 | EE ,
     /* "MOFST",   MOFST,*/    L16 | EE ,
     /* "IOFST",   IOFST,*/    L16 | EE ,
     /* "PTEMP",   PTEMP,*/    L16 | EE ,
     /* "UPSECS",  UPSECS,*/   L16 | EE ,
     /* "OD",      OD, */      L16 | EE ,
     /* "MDS",     MDS,*/      L16 | EE ,
    
     /* "AP",      AP, */      L32 | EE ,
     /* "AP2",     AP2  */     L16 ,
     /* "MECH",    MECH,*/     L32 ,
     /* "MECH2",   MECH2,*/    L16 ,
     /* "CTS",     CTS,*/      L32 | EE ,
     /* "CTS2",    CTS2,*/     L16 ,
     /* "DP",      DP, */      L32 | EE ,
     /* "DP2",     DP2  */     L16 ,
     /* "OT",      OT, */      L32 | EE ,
     /* "OT2",     OT2  */     L16 ,
     /* "CT",      CT, */      L32 | EE ,
     /* "CT2",     CT2  */     L16 ,
    
     /* "BAUD",    BAUD,*/     L16 ,
     /* "TEMP",    TEMP,*/     L16 ,
     /* "OTEMP",   OTEMP,*/    L16 ,
     /* "LOCK",    LOCK,*/     L16 ,
     /* "DIG0",    DIG0,*/     L16 ,
     /* "DIG1",    DIG1,*/     L16 ,
     /* "ANA0",    ANA0,*/     L16 ,
     /* "ANA1",    ANA1,*/     L16 ,
     /* "THERM",   THERM,*/    L16 ,
     /* "VBUS",    VBUS,*/     L16 ,
     /* "IMOTOR",  IMOTOR,*/   L16 ,
     /* "VLOGIC",  VLOGIC,*/   L16 ,
     /* "ILOGIC",  ILOGIC,*/   L16 ,
    
     /* "GRPA",    GRPA,*/     L16 | EE ,
     /* "GRPB",    GRPB,*/     L16 | EE ,
     /* "GRPC",    GRPC,*/     L16 | EE ,
     /* "PIDX",    PIDX,*/     L16 | EE ,
     /* "ZERO",    ZERO,*/     L16 ,
    
     /* "SG",      SG,*/       L16 ,
     /* "HSG",     HSG,*/      L16 | EE ,
     /* "LSG",     LSG,*/      L16 | EE ,
     /* "DS",      DS,*/       L16 | EE ,
     /* "IVEL",    IVEL,*/     L16 | EE ,
     /* "IOFF",    IOFF,*/     L16 | EE ,
     /* "MPE",     MPE,*/      L16 | EE ,
     /* "EN",      EN, */      L16 ,
     /* "TSTOP",   TSTOP,*/    L16 | EE ,
     /* "KP",      KP,*/       L16 | EE ,
     /* "KD",      KD,*/       L16 | EE ,
     /* "KI",      KI, */      L16 | EE ,
     /* "SAMPLE",  SAMPLE,*/   L16 | EE ,
     /* "ACCEL",   ACCEL,*/    L16 | EE ,
     /* "TENSION", TENSION,*/  L16 ,
     
     /* "UNITS",   UNITS,*/    L16 | EE ,
     /* "RATIO",   RATIO,*/    L16 | EE ,
    
     /* "LOG",     LOG,*/      L16 ,
     /* "DUMP",    DUMP,*/     L16 ,
     /* "LOG1",    LOG1,*/     L16 ,
     /* "LOG2",    LOG2,*/     L16 ,
     /* "LOG3",    LOG3,*/     L16 ,
     /* "LOG4",    LOG4,*/     L16 ,
    
     /* "GAIN1",   GAIN1,*/    L16 | EE ,
     /* "GAIN2",   GAIN2,*/    L16 | EE ,
     /* "GAIN3",   GAIN3,*/    L16 | EE ,
     /* "OFFSET1", OFFSET1,*/  L16 | EE ,
     /* "OFFSET2", OFFSET2,*/  L16 | EE ,
     /* "OFFSET3", OFFSET3,*/  L16 | EE ,
    
     /* "PEN",     PEN,*/      L16 ,
     /* "SAFE",    SAFE,*/     L16 ,
     /* "SAVE",    SAVE,*/     L16 ,
     /* "LOAD",    LOAD,*/     L16 ,
     /* "DEF",     DEF,*/      L16 ,
    
     /* "VL1",     VL1,*/      L16 ,
     /* "VL2",     VL2,*/      L16 ,
     /* "TL1",     TL1,*/      L16 ,
     /* "TL2",     TL2,*/      L16 ,
     /* "VOLTL1",  VOLTL1,*/   L16 ,
     /* "VOLTL2",  VOLTL2,*/   L16 ,
     /* "VOLTH1",  VOLTH1,*/   L16 ,
     /* "VOLTH2",  VOLTH2,*/   L16 ,
     /* "MAXPWR",  MAXPWR,*/   L16 ,
     /* "PWR",     PWR,*/      L16 ,
     /* "IFAULT",  IFAULT,*/   L16 
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
    for(i = 0; i < 2048; i++) if((i & ~mask) == id) canIdAdd(canDev[bus], i);
}

int initCAN(int bus)
{
    DWORD retvalue;
    
    pthread_mutex_init(&commMutex, NULL);
    
    retvalue = canOpen(bus, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);
    if(retvalue != NTCAN_SUCCESS)
    {
        syslog(LOG_ERR, "initCAN(): canOpen() failed with error %d", retvalue);
        return(1);
    }
    
    retvalue = canSetBaudrate(canDev[bus], 0); // 1 = 1Mbps, 2 = 500kbps, 3 = 250kbps
    if(retvalue != 0)
    {
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
    
    if(blocking)
    {
        retvalue = canRead(canDev[bus], &msg, &msgCt, NULL);
    }else{
        retvalue = canTake(canDev[bus], &msg, &msgCt);
    }
    if(retvalue != NTCAN_SUCCESS)
    {
        syslog(LOG_ERR, "canReadMsg(): canRead/canTake error: %ld", retvalue);
        if(retvalue == NTCAN_RX_TIMEOUT)
            return(1);
        else
            return(2);
    }
    if(msgCt == 1)
    {
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
    
    if(blocking)
    {
        retvalue = canWrite(canDev[bus], &msg, &msgCt, NULL);
    }else{
        retvalue = canSend(canDev[bus], &msg, &msgCt);
    }
    
    if(retvalue != NTCAN_SUCCESS)
    {
        syslog(LOG_ERR, "canSendMsg(): canWrite/Send() failed with error %d", retvalue);
        return(1);
    }
}

int wakePuck(int bus, int who)
{
    setProperty(bus, who, STAT, FALSE, STATUS_READY);
    usleep(10000); // Wait 10ms for puck to initialize
    
    return(0);
}

int setTorques(int bus, int group, int *values)
{
    unsigned char   data[8];
    int             err;
    int             i;
    
    /* Bound the torque commands */
    for (i = 0; i < 4; i++)
    {
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
    while(howMany)
    {
        err = canReadMsg(bus, &msgID, &len, data, TRUE);
        
        // If no error
        if(!err)
        {
            // Parse the reply
            err = parseMessage(msgID, len, data, &id, &property, &reply);
            if(property == AP)
            {
                pos[id] = reply;
                --howMany;
            }
            else
            {
                syslog(LOG_ERR, "getPositions(): Asked group %d for position, received property %d = %ld from id %d", property, reply, id);
            }
        }
        else // Timeout or other error
        {
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
    err = canSendMsg(bus, NODE2ADDR(id), len, data, TRUE);
    pthread_mutex_unlock(&commMutex);
    
    if(verify)
    {
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
    
    if(!err)
    {
        // Parse the reply
        err = parseMessage(id_in, len_in, data, &id_in, &property_in, reply);
        
        // Check that the id and property match
        if((id == id_in) && (property == property_in))
            return(0);
        else
        {
            syslog(LOG_ERR, "getProperty(): returned id or property do not match");
            return(1);
        }
    }
    else
    {
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
    
    for(id = 1; id <= 30; id++)
    {
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
        if(!err)
        {
            // Parse the reply
            err = parseMessage(id_in, len_in, data, &id_in, &property_in, &status[id-1]);
        }
        else syslog(LOG_ERR, "getBusStatus(): canReadMsg returned error");
    }
    
    pthread_mutex_unlock(&commMutex);
    for(id=1;id<=30;id++) syslog(LOG_ERR,"status[%d]=%d",id,status[id]);
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
  switch (dataHeader)
  {
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
    if(property > PROP_END)
    {
        syslog(LOG_ERR,"compile(): Invalid property = %d", property);
        return(1);
    }
    
    /* Insert the property */
    data[0] = property;
    data[1] = 0; /* To align the values for the tater's DSP */
    
    /* Append the value */
    for (i = 2; i < 6; i++)
    {
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


/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003, 2004 Barrett Technology, Inc.           *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
