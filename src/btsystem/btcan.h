/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcan.h
 *  Author .............Brian Zenowich
 *  Creation Date ......24 Mar 2003
 *  Addtl Authors ......Traveler Hauptman, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  24 Mar 2003 - BZ
 *    File created & documented. 
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *                                                                      *
 *======================================================================*/
#ifndef _BTCAN_H
#define _BTCAN_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
//#include <inttypes.h>
//#include <pthread.h>

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif
/* #define D(x) <blank> ...for no debug info */
/* #define D(x) x       ...for debug info */
#define D(x) 
#define SAFETY_MODULE (10)
#define MAX_NODES    (31)

#define L08       (1)
#define L16       (2)
#define L24       (3)
#define L32       (4)

#define EE        (0x0008)

#define mbxID               (0)
#define BASE_ID             (0)

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

/* Public Data Structures */
/** CAN device information data structure
*/

/* Public Functions */
int initCAN(int bus);
void freeCAN(int bus);
int getBusStatus(int bus, long *status);
int wakePuck(int bus, int who);
int getProperty(int bus, int who, int property, long *reply);
int setProperty(int bus, int who, int property, int verify, long value);
int getPositions(int bus, int group, int howMany, long *pos);
int setTorques(int bus, int group, int *values);


/*! bcastGroup */
enum {
    WHOLE_ARM = 0,
    LOWER_ARM = -1,
    UPPER_ARM = -2
};

enum {
    ROLE_TATER,
    ROLE_GIMBALS,
    ROLE_SAFETY,
    ROLE_WRAPTOR
};

/*! Control_mode states */
enum {
    MODE_IDLE,
    MODE_DUTY_CYCLE,
    MODE_TORQUE,
    MODE_PID,
    MODE_VELOCITY,
    MODE_TRAPEZOIDAL
};

enum {
    STATUS_OFFLINE = -1,
    STATUS_RESET,
    STATUS_ERR,
    STATUS_READY
};

enum {
    DEG,        /* 0-360        */
    RAD,        /* 0-6.28       */
    GRAD,       /* 0-400        */
    PERCENT,    /* 0-100        */
    NATIVE      /* 0-CTS*RATIO  */
};

enum {
    SAVED_ERR = 7,
    IGNORE_ERR,
    IS_ACTIVE
    
};

enum {
	VERS,
	ROLE,
	SN,
	ID,
	ERROR,
	STAT,
	ADDR,
	VALUE,
	MODE,
	D,
	TORQ,
	P,
	V,
	E,
	B,
	MD,
	MT,
	MV,
	MCV,
	MOV,
	MOFST,
	IOFST,
	PTEMP,
	UPSECS,
	OD,
	MDS,
	AP,
	AP2,
	MECH,
	MECH2,
	CTS,
	CTS2,
	DP,
	DP2,
	OT,
	OT2,
	CT,
	CT2,
	BAUD,
	TEMP,
	OTEMP,
	_LOCK,
	DIG0,
	DIG1,
	ANA0,
	ANA1,
	THERM,
	VBUS,
	IMOTOR,
	VLOGIC,
	ILOGIC,
	GRPA,
	GRPB,
	GRPC,
	PIDX,
	JIDX,
	ZERO,
	IPNM,
	SG,
	HSG,
	LSG,
	_DS,
	IVEL,
	IOFF,
	MPE,
	EN,
	TSTOP,
	KP,
	KD,
	KI,
	SAMPLE,
	ACCEL,
	TENSION,
	UNITS,
	RATIO,
	LOG,
	DUMP,
	LOG1,
	LOG2,
	LOG3,
	LOG4,
	GAIN1,
	GAIN2,
	GAIN3,
	OFFSET1,
	OFFSET2,
	OFFSET3,
	PEN,
	SAFE,
	SAVE,
	LOAD,
	DEF,
	VL1,
	VL2,
	TL1,
	TL2,
	VOLTL1,
	VOLTL2,
	VOLTH1,
	VOLTH2,
	MAXPWR,
	PWR,
	IFAULT,
	IKP,
	IKI,
	IKCOR,
	VNOM,
	TENST,
	TENSO,
	
	PROP_END
};


enum bus_status_enum{
    BUS_ERROR = -1,
    BUS_OFF,
    BUS_ON
};


enum {
	ERR_NONE,
	ERR_READ_ONLY,
	ERR_OUT_OF_RANGE
};


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTCAN_H */

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
