/* ======================================================================== *
 *  Module ............. btdiag
 *  File ............... btdiag.c
 *  Creation Date ...... 14 Oct 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file btdiag.c
    \brief An interactive demo of WAM capabilities.
 
    This is a full-featured demo. If the source is intimidating, I suggest
    you start with the examples in the ../../examples directory. You will
    be a WAM code expert in no time!
 
    This is the primary WAM demo application. It shows off many
    features of the WAM library.
    This application is also used for diagnostics and testing.
 
The user can switch between Cartesian space and joint space. The toggle 
variable is a pointer to the present btstatecontroller.
 
Note that if a trajectory is modified, you must call 's' scale trajectory on
it to properly establish the time values.
 
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#ifdef S_SPLINT_S
#include <err.h>
#else
#include <pthread.h>
#endif
#include <syslog.h>
#include <signal.h>
/* Decipher function error codes using the defined system constants.
 * For the error text, use syslog("LOG_ERR", "f(): %s", strerror(errnum));
 */
//#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/io.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"
#include "bthaptics.h"
#include "btserial.h"
#include "aob.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
enum{SCREEN_MAIN, SCREEN_HELP};

#ifdef XENOMAI
#define Ts (0.004)
#else 
#define Ts (0.004)
#endif


/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define _kbhit() (0)
#define _getch()
#define Error()

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
typedef struct {
   RTIME t;
   char c;
}keyEventStruct;

typedef struct {
   /* Per-WAM State */
   btstatecontrol *active_bts;
   vect_n *jdest;
   vect_n *cdest;
   vect_n *active_pos;
   vect_n *active_dest;
   vect_n *active_trq;
   via_trj_array **vta;
   via_trj_array *vt_j;
   via_trj_array *vt_c;
   btrt_thread_struct wam_thd;
   btgeom_state pstate;
}wamData_struct;

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
wam_struct *wam[4];
wamData_struct wamData[4];

/* Global State */
int mode;
int constraint;
int haptics;
int pauseCnt = -2;
int screen = SCREEN_MAIN;
btrt_mutex disp_mutex;
int entryLine;
int useGimbals      = FALSE;
int done            = FALSE;
int alldone         = FALSE;
int quiet           = FALSE;
//int prev_mode;
int cteach = 0;
int NoSafety;
int angular = 0;
int cplay = 0;
int move_prep = 0; /* Are we currently in move prep? */
int loop_trj = 0;  /* Are we looping the trajectory? */
int demo = 0;

/* Global data */
btthread audio_thd;
btrt_thread_struct disp_thd;
int busCount = 0;
char *command_help[100];
int num_commands;
PORT p; // Serial port
int force = 0;

/* Event logger */
int eventIdx;
RTIME eventStart;
keyEventStruct keyEvent[500];

/* Other */
double vel = 0.5, acc = 2.0;
char active_file[250];
char *user_def = "User edited point list";
matr_h *cdest;
//vect_n *entry;
//vect_3 *xyz;
vect_3 *RxRyRz;

int startDone = FALSE;
btrt_thread_struct  StartupThread;
btrt_thread_struct  event_thd;

extern long jointPosition[];

#if 0
vectray *vr;
via_trj_array **vta = NULL,*vt_j = NULL,*vt_c = NULL;
extern int isZeroed;
/*static RT_TASK *mainTask; */
btthread audio_thd,disp_thd;
btrt_thread_struct wam_thd;
double sample_rate;
btreal Jpos_filt[7];
btfilter *j5,*j6,*j7;
btfilter *yk_xfilt, *yk_yfilt, *yk_zfilt;
matr_mn *Th;
#endif

/******* Haptics *******/
btgeom_plane planes[10];
bteffect_wall wall[10];
bteffect_wickedwall wickedwalls[10];
bthaptic_object objects[20];
btgeom_sphere spheres[10];
bteffect_bulletproofwall bpwall[10];
btgeom_box boxs[10];
//bteffect_global myglobal;
vect_3 *p1,*p2,*p3,*zero_v3;
bthaptic_scene bth;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void RenderMAIN_SCREEN(void);
void RenderHELP_SCREEN(void);
void RenderJOINTSPACE_SCREEN(void);
void RenderCARTSPACE_SCREEN(void);
void ProcessInput(int c);
void Shutdown(void);
void DisplayThread(void);
void DemoThread(void);
void AudioThread(void);
void StartDisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void init_haptics(void);
void read_keys(char *filename);
int  WAMcallback(struct btwam_struct *wam);

/*==============================*
 * Functions                    *
 *==============================*/
 
/* Define character identification macros */
#define isAlpha(c) 		((c >= 'A') && (c <= 'Z'))
#define isDigit(c) 		((c >= '0') && (c <= '9'))
#define isSpace(c) 		( c == SP )
#define toUpper(c) 		( ((c >= 'a') && (c <= 'z')) ? (c - 'a' + 'A') : (c) )

#define INVALID   (0)
/* Define NONE as (N)egative (ONE), tee hee */
#define NONE	  (-1)
#define MAX_INT   (0x7FFF)


#define US        (0x0008)
#define MAX_CMD_LEN (32)
#define D(x) 

typedef struct
{
	char	*ptr;
	char	command[MAX_CMD_LEN];
}COMMAND;



struct textKey {
	char	key[8];
	int	idx;
};



#define ROLE_OPT_MAG_SER (0x0100)
#define ROLE_OPT_MAG_HALL (0x0200)
#define ROLE_OPT_MAG_ENC (0x0400)
#define ROLE_OPT_STRAIN (0x0800)
#define ROLE_OPT_PPS (0x1000)




/* Command enumeration. Append only. */
enum {
	xCMD_LOAD, xCMD_SAVE,	xCMD_RESET, xCMD_DEF, xCMD_GET, xCMD_FIND, xCMD_SET, xCMD_HOME,	
	xCMD_KEEP, xCMD_LOOP, xCMD_PASS, xCMD_VERS, xCMD_ERR, xCMD_HI, xCMD_IC, xCMD_IO,
	xCMD_TC, xCMD_TO, xCMD_C, xCMD_M, xCMD_O, xCMD_T, xCMD_HELP, xCMD_END
};


/* Command text. Keyword, enumeration. Add alphabetically. */
const struct textKey cmdTxt[]={
	{ "FLOAD", xCMD_LOAD },	
	{ "FSAVE", xCMD_SAVE },	
	{ "RESET", xCMD_RESET },	
	{ "FDEF", xCMD_DEF },	
	{ "FGET", xCMD_GET },	
	{ "FIND", xCMD_FIND },	
	{ "FSET", xCMD_SET },	
	{ "HOME", xCMD_HOME },	
	{ "KEEP", xCMD_KEEP },	
	{ "LOAD", xCMD_LOAD },	
	{ "LOOP", xCMD_LOOP },	
	{ "PASS", xCMD_PASS },	
	{ "SAVE", xCMD_SAVE },	
	{ "VERS", xCMD_VERS },	
	{ "DEF", xCMD_DEF },	
	{ "ERR", xCMD_ERR },	
	{ "GET", xCMD_GET },	
	{ "SET", xCMD_SET },	
	{ "HI", xCMD_HI },
	{ "IC", xCMD_IC },
	{ "IO", xCMD_IO },                                   
	{ "TC", xCMD_TC },
	{ "TO", xCMD_TO },
	{ "C", xCMD_C },
	{ "M", xCMD_M },
	{ "O", xCMD_O },
	{ "T", xCMD_T },
	{ "?", xCMD_HELP },

	{ "", NONE }
};

/* Common */
enum {
	xVERS = 0,
	xROLE, /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
	xSN,
	xID,
	xERROR,
	xSTAT,
	xADDR,
	xVALUE,
	xMODE,
	xTEMP,
	xPTEMP,
	xOTEMP,
	xBAUD,
	xLOCK,
	xDIG0,
	xDIG1,
	xFET0,
	xFET1,
	xANA0,
	xANA1,
	xTHERM,
	xVBUS,
	xIMOTOR,
	xVLOGIC,
	xILOGIC,
	xSG,
	xGRPA,
	xGRPB,
	xGRPC,
	xCMD, /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
	xSAVE,
	xLOAD,
	xDEF,
	xFIND,
	xX0,
	xX1,
	xX2,
	xX3,
	xX4,
	xX5,
	xX6,
	xX7,
	
	xCOMMON_END
};



/* Tater */
enum {
	xT = xCOMMON_END,
	xMT,
	xV,
	xMV,
	xMCV,
	xMOV,
	xP, /* 32-Bit Present Position */
	xP2,
	xDP, /* 32-Bit Default Position */
	xDP2,
	xE, /* 32-Bit Endpoint */
	xE2,
	xOT, /* 32-Bit Open Target */
	xOT2,
	xCT, /* 32-Bit Close Target */
	xCT2,
	xM, /* 32-Bit Move command for CAN*/
	xM2,
	xDS,
	xMOFST,
	xIOFST,
	xUPSECS,
	xOD,
	xMDS,
	xMECH, /* 32-Bit */
	xMECH2,
	xCTS, /* 32-Bit */
	xCTS2,
	xPIDX,
	xHSG,
	xLSG,
	xIVEL,
	xIOFF, /* 32-Bit */
	xIOFF2,
	xMPE,
	xHOLD,
	xTSTOP,
	xKP,
	xKD,
	xKI,
	xACCEL,
	xTENST,
	xTENSO,
	xJIDX,
	xIPNM,
	xHALLS,
	xHALLH, /* 32-Bit */
	xHALLH2,
	xPOLES,
	xIKP,
	xIKI,
	xIKCOR,
	xEN, /* 32-Bit */
	xEN2,
	xJP, /* 32-Bit */
	xJP2,
	xJOFST, /* 32-Bit */
	xJOFST2,
	xTIE,
	xECMAX,
	xECMIN,
	xLFLAGS,
	xLCTC,
	xLCVC,
	
	xPROP_END
};

const struct textKey propTxtCommon[]={
    { "ILOGIC", xILOGIC }, /* Logic current (tbd) */
    { "IMOTOR", xIMOTOR }, /* Motor current (2048+205/1A) */
    { "VLOGIC", xVLOGIC }, /* Logic voltage (tbd) */
    { "ERROR", xERROR }, /* Error (tbd) */
    { "OTEMP", xOTEMP }, /* Over temperature alarm (tbd) */
    { "PTEMP", xPTEMP }, /* Peak temperature recorded (tbd) */
    { "THERM", xTHERM }, /* Thermistor (motor) temperature */
    { "VALUE", xVALUE }, /* Value to poke/peek */
    { "ADDR", xADDR }, /* Address to peek/poke */
    { "ANA0", xANA0 }, /* Analog input (pin 4:0-3V, 3:Gnd) */
    { "ANA1", xANA1 }, /* Analog input (pin 2:0-3V, 3:Gnd) */
    { "BAUD", xBAUD }, /* Baud rate/100. 96=9600 bps */
    { "DIG0", xDIG0 }, /* Dig I/O: -1=In,0=Lo,1=Hi,2-100=%PWM (pin 41:0-3.3V, 44:Gnd) */
    { "DIG1", xDIG1 }, /* Dig I/O: -1=In,0=Lo,1=Hi (pin 43:0-3.3V, 44:Gnd) */
    { "FET0", xFET0 }, /* Tensioner output: 0=Off, 1=On */
    { "FET1", xFET1 }, /* Brake output: 0=Off, 1=On */
    { "FIND", xFIND }, /* Find command for CAN */
    { "GRPA", xGRPA }, /* Comm group A */
    { "GRPB", xGRPB }, /* Comm group B */
    { "GRPC", xGRPC }, /* Comm group C */
    { "LOAD", xLOAD }, /* Load command for CAN */
    { "LOCK", xLOCK }, /* Lock */
    { "MODE", xMODE }, /* Mode: 0=Idle, 2=Torque, 3=PID, 4=Vel, 5=Trap */
    { "ROLE", xROLE }, /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
    { "SAVE", xSAVE }, /* Save command for CAN */
    { "STAT", xSTAT }, /* Status: 0=Reset/Monitor, 2=Ready/Main */
    { "TEMP", xTEMP }, /* Temperature (puck internal) */
    { "VBUS", xVBUS }, /* Bus voltage (V) */
    { "VERS", xVERS }, /* Firmware version */
    { "CMD", xCMD }, /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
    { "DEF", xDEF }, /* Default command for CAN */
    { "ID", xID }, /* CANbus ID */
    { "SG", xSG }, /* Strain gage (tbd) */
    { "SN", xSN }, /* Serial number */
    { "X0", xX0 }, /* Gimbals offset 1 (Q4.12 rad) */
    { "X1", xX1 }, /* Gimbals offset 2 (Q4.12 rad) */
    { "X2", xX2 }, /* Gimbals offset 3 (Q4.12 rad) */
    { "X3", xX3 }, /* Gimbals/safety gain 1 (Q4.12 rad/3V) */
    { "X4", xX4 }, /* Gimbals/safety gain 2 (Q4.12 rad/3V) */
    { "X5", xX5 }, /* Gimbals/safety gain 3 (Q4.12 rad/3V) */
    { "X6", xX6 }, /* tbd */
    { "X7", xX7 }, /* tbd */

    { "", NONE }    
};


const struct textKey propTxtTater[]={
    { "LFLAGS", xLFLAGS }, /* Loop feedback flags */
    { "UPSECS", xUPSECS }, /* Up seconds in operation (tbd) */
    { "ACCEL", xACCEL }, /* Acceleration (Q8.8 cts/ms/ms) */
    { "ECMAX", xECMAX }, /* Encoder correction max value */
    { "ECMIN", xECMIN }, /* Encoder correction min value */
    { "HALLH", xHALLH }, /* 32-Bit Hall history bitfield */
    { "HALLS", xHALLS }, /* Hall feedback bitfield: CBA */
    { "IKCOR", xIKCOR }, /* Current sense correction factor */
    { "IOFST", xIOFST }, /* Current offset calibration */
    { "JOFST", xJOFST }, /* Joint encoder calibration offset */
    { "MOFST", xMOFST }, /* Mechanical offset calibration */
    { "POLES", xPOLES }, /* Number of magnets on rotor */
    { "TENSO", xTENSO }, /* Tension offset (tbd) */
    { "TENST", xTENST }, /* Tension total (tbd) */
    { "TSTOP", xTSTOP }, /* Time until considered stopped */
    { "HOLD", xHOLD }, /* Flag to hold position after move */
    { "IOFF", xIOFF }, /* Initialization offset */
    { "IPNM", xIPNM }, /* CommandedCurrent / Nm (ratio) */
    { "IVEL", xIVEL }, /* Initialization velocity (tbd) */
    { "JIDX", xJIDX }, /* Joint index */
    { "LCTC", xLCTC }, /* Loop control torque coefficient */
    { "LCVC", xLCVC }, /* Loop control velocity coefficient */
    { "MECH", xMECH }, /* 32-Bit Mechanical angle (cts) */
    { "PIDX", xPIDX }, /* Puck index for torque */
    { "CTS", xCTS }, /* 32-Bit Counts per revolution */
    { "HSG", xHSG }, /* High strain gage (tbd) */
    { "IKI", xIKI }, /* Current sense integral gain */
    { "IKP", xIKP }, /* Current sense proportional gain */
    { "LSG", xLSG }, /* Low strain gage (tbd) */
    { "MCV", xMCV }, /* Max close velocity (cts/ms) */
    { "MDS", xMDS }, /* Max duty sum for power limiting (tbd) */
    { "MOV", xMOV }, /* Max open velocity (cts/ms) */
    { "MPE", xMPE }, /* Max position error (tbd) */
    { "TIE", xTIE }, /* Flag to tie inner and outer links */
    { "CT", xCT }, /* 32-Bit Close Target */
    { "DP", xDP }, /* 32-Bit Default Position */
    { "DS", xDS }, /* Default step */
    { "EN", xEN }, /* Enable bitfield */
    { "JP", xJP }, /* Joint encoder position */
    { "KD", xKD }, /* Differential gain */
    { "KI", xKI }, /* Integral gain */
    { "KP", xKP }, /* Proportional gain */
    { "MT", xMT }, /* Max torque */
    { "MV", xMV }, /* Max velocity (cts/ms) */
    { "OD", xOD }, /* Odometer (tbd) */
    { "OT", xOT }, /* 32-Bit Open Target */
    { "E", xE }, /* 32-Bit Endpoint */
    { "M", xM }, /* 32-Bit Move command for CAN */
    { "P", xP }, /* 32-Bit Position. R=Act, W=Cmd */
    { "T", xT }, /* Torque command */
    { "V", xV }, /* Velocity (cts/ms).  R=Act, W=Cmd */

    { "", NONE } 
};

/*==============================*
 * PRIVATE Function prototypes  *
 *==============================*/
void uppercase(char *s);
long parseMotorSelect(COMMAND *c);
int parseCommand(COMMAND *c);
int parseProperty(COMMAND *c);
inline int parseText(COMMAND *c, const struct textKey *t);

/*==============================*
 * Functions                    *
 *==============================*/

void uppercase(char *s)
{ 
	while(*s++ = toUpper(*s)) {} 
}

void parseInput(COMMAND *c)
{
	int 	err, i;
	long 	motorSelect;
	int 	command;
	int 	p;
	long 	value = 0;
	char	str[12];
	
	/* Uppercase the (null terminated) input string */
	uppercase(c->command);

    //syslog(LOG_ERR,"[%s] ", c->command);

	/* {<WS>} {MotorSelect} {<WS>} Command <WS> {Parameter} <WS> {Value} {<WS>} */
	D(commWriteSerial(1,"["));
	D(commWriteSerial(strlen(c->command),c->command));
	D(commWriteSerial(1,"]"));
	
	c->ptr = c->command;
	
	/* Skip WhiteSpace */
	while(*c->ptr == ' ') { ++c->ptr; }
	
	/* Determine the motor select */
	// 0 1 2 3 4 5 6 7 G S IL (InnerLinks) OL (OuterLinks)
	motorSelect = parseMotorSelect(c);
	if(!motorSelect){ // If no motors specified
		
			motorSelect = 0x001E; // Specify all: 00011110
	}
	
	D(commWriteSerial(4," ms="));
	D(commWriteSerial(ltoa((long)motorSelect, str), str));
	
	/* Skip WhiteSpace */
	while(*c->ptr == ' ') { ++c->ptr; }
		
	/* Determine the command */
	// RESET FSET SET FGET GET FLOAD LOAD FSAVE SAVE FDEF DEF
	// HOME IO IC HI VERS ERR O C M T ?
	
	command = parseText(c, cmdTxt);
	
	D(commWriteSerial(3," c="));
	D(commWriteSerial(ltoa((long)command, str), str));
	if(command == NONE) // If no command specified
	{
//		serErr(ERR_NO_CMD);
		return;
	}
	
	/* Skip WhiteSpace */
	while(*c->ptr == ' ') { ++c->ptr; }
	
	/* Determine the property */
	switch(command){
	case xCMD_SET: // "SET"
	case xCMD_GET: // "GET"
	case xCMD_FIND: // "FIND"
	case xCMD_LOAD: // LOAD
	case xCMD_SAVE: // SAVE
	case xCMD_DEF:  // DEF
		p = parseText(c, propTxtCommon);
		if(p == NONE) // If no common property specified
		{
				p = parseText(c, propTxtTater); // Check for tater prop
				
		}
		D(commWriteSerial(3," p="));
		D(commWriteSerial(ltoa((long)p, str), str));
	
		/* Skip WhiteSpace */
		while(*c->ptr == ' ') { ++c->ptr; }
	}
	
	/* Read a value */
	switch(command){
	case xCMD_SET: // "SET"
		/* Determine the value */
		err = parse_atol(c->ptr, &value);
		break;
	case xCMD_M: // "M"
		/* Determine the value */
		err = parse_atol(c->ptr, &value);
		if(!err){ // If we have a value, just do a "SET M"
			command = xCMD_SET;
			p = xM;
		}
		break;
	}
	
	D(commWriteSerial(3," v="));
	D(commWriteSerial(ltoa(value, str), str));
	
	D(commWriteSerial(4," EN="));
	D(commWriteSerial(ltoa((long)property[EN], str), str));
	
	D(commWriteSerial(4," ID="));
	D(commWriteSerial(ltoa((long)property[ID], str), str));
	
	D(commWriteSerial(4," me="));
	D(commWriteSerial(ltoa((long)me, str), str));
	
	D(commWriteSerial(2,"\015\012")); // CRLF
	
	syslog(LOG_ERR,"[%s] m=0x%0.2x, c=%d, p=%d, v=%d", c->command, motorSelect, command, p, value);
	/* Call the cmd function, dependent on status */
	//if(property[STAT] == STATUS_READY || p == VERS || p == STAT)
	//	(cmd[command].f)(motorSelect, p, (void*)&value);

    for( i = 0; i < 4; i++){
        motorSelect >>= 1;
        if(motorSelect & 0x01){
            switch(command){
                case xCMD_LOAD:
                setProperty(0,i+11,xLOAD,0,p);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xLOAD, p);
                break;
                case xCMD_SAVE:
                setProperty(0,i+11,xSAVE,0,p);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xSAVE, p);
                break;
                case xCMD_RESET:
                setProperty(0,i+11,xCMD,0,xCMD_RESET);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_RESET);
                break;
                case xCMD_DEF:
                setProperty(0,i+11,xDEF,0,p);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xDEF, p);
                break;
                case xCMD_GET:
                
                break;
                case xCMD_SET:
                setProperty(0,i+11,p,0,value);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, p, value);
                break;
                case xCMD_HOME:
                setProperty(0,i+11,xCMD,0,xCMD_HOME);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_HOME);
                break;
                case xCMD_HI:
                setProperty(0,i+11,xCMD,0,xCMD_HI);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_HI);
                break;
                case xCMD_IC:
                setProperty(0,i+11,xCMD,0,xCMD_IC);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_IC);
                break;
                case xCMD_IO:
                setProperty(0,i+11,xCMD,0,xCMD_IO);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_IO);
                break;
                case xCMD_C:
                setProperty(0,i+11,xCMD,0,xCMD_C);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_C);
                break;
                case xCMD_M:
                setProperty(0,i+11,xCMD,0,xCMD_M);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_M);
                break;
                case xCMD_O:
                setProperty(0,i+11,xCMD,0,xCMD_O);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_O);
                break;
                case xCMD_T:
                setProperty(0,i+11,xCMD,0,xCMD_T);
                syslog(LOG_ERR, "Hand: %d SET %d to %ld", i+11, xCMD,xCMD_T);
                break;
                default:
                
                break;
            }
        }
    }

    /* Wait required for movements */
    waitForStop();

#if 0
	/* Response required for GET, VERS, ERR, ? */
	switch(command){
	case xCMD_GET: // "GET"
		if(forOnlyMe(motorSelect))
			commWriteSerial(ltoa(value,str), str);
		commWriteSerial(2,"\015\012");
		break;
		
	case xCMD_VERS: // "VERS"
		commWriteSerial(18, "Firmware Version: ");
		commWriteSerial(ltoa(value,str), str);
		commWriteSerial(2,"\015\012");
		break;
		
	case xCMD_ERR: // "ERR"
		commWriteSerial(34, "To err is human.  I'm not human.\015\012");
		break;
		
	case xCMD_HELP: // "?"
		commWriteSerial(59, "It is pitch dark, you are likely to be eaten by a grue...\015\012");
		break;
	}
#endif
}

/* Convert [1234567GS][IL][OL] into a Wraptor motor select bitfield */
long parseMotorSelect(COMMAND *c)
{
	long motorSelect; /* XXXXXXXX76543210 */

	motorSelect = 0;

	while (1)
	{
		if(isDigit(*c->ptr))
		{
				motorSelect |= 1L << (*c->ptr - '0');
				++c->ptr;
			
			continue;
	}
		switch (*c->ptr) {
		case ('S'):
			// Must NOT be followed by 'E' (for SET)
			if(*(c->ptr+1) == 'E') return(motorSelect);
			// Must NOT be followed by 'AV' (for SAVE)
			if( (*(c->ptr+1) == 'A') && (*(c->ptr+2) == 'V') ) return(motorSelect);
			motorSelect |= 0x0010; // Motor 4 = 00010000
			++c->ptr;
			break;
		case ('G'):
			// Must NOT be followed by 'E' (for GET)
			if(*(c->ptr+1) == 'E') return(motorSelect);
			motorSelect |= 0x000E; // 1, 2, 3  = 00001110
			++c->ptr;
			break;
		case ('I'):
			// Must be followed by 'L' (for Inner Link)
			if(*(c->ptr+1) != 'L') return(motorSelect);
			motorSelect |= 0x000E; // 1, 2, 3 = 00001110
			c->ptr += 2;
			break;
		case ('O'): // The letter oh
			// Must be followed by 'L' (for Outer Link)
			if(*(c->ptr+1) != 'L') return(motorSelect);
			motorSelect |= 0x00E0; // 5, 6, 7 = 11100000
			c->ptr += 2;
			break;
		default:
			return(motorSelect);
			//break;
		}
	}
}

/* Determine the property (for GET FGET SET FSET) */
inline int parseText(COMMAND *c, const struct textKey *t)
{
	int i;
	
	i = 0;
	while(*t[i].key)
	{
		if(strstr(c->ptr, t[i].key) == c->ptr) // Command matches, followed by...
			if(*(c->ptr + strlen(t[i].key)) <= ' ') // ...space (32) or null (0)
				break; // Found the command
		++i; 
	}
	c->ptr += strlen(t[i].key);
		
	return(t[i].idx);
}

/** Converts a string to a long. 

    \return 0 for successful conversion, 1 for no conversion
*/
int parse_atol(
    char *s /**<. Pointer to string to convert to integer */,
	long *v /**<. Pointer to location to put the value */)
{
    int i = 0;  /* counter */
    short sign; /* is there a sign in the string */ 
    long addval; /* the value of the character currently adding */
    long result;   /* the result of the function */
    
    result = 0;

    if((sign = (*s == '-')) || *s == '+')
        i++;
    while(*(s+i)>='0' && *(s+i)<='9')
    {
        addval = *(s+i) - '0';
        if (result == (addval = (result * 10 + addval))/10) 
            result = addval;
        ++i; 
    }
    if(sign) 
        result = -result;
	
	*v = result;
	
	return(!i);
}

void Cleanup(){
   int i;

   for(i = 0; i < busCount; i++){
      wamData[i].wam_thd.done = 1;
      //btrt_thread_stop(&wamData[i].wam_thd); // Exit control loop and JOIN
      //btrt_thread_exit(&wamData[i].wam_thd); // Delete task
   }
   usleep(10000);
   
   /* Close the log file */
   CloseDL(&(wam[0]->log));
   
   /* Decode the file from binary->text */
   DecodeDL("datafile.dat", "dat.csv", 1);
   
   CloseSystem(); // Free the actuator memory and CAN device(s)
   StartupThread.done = 1;
   //btrt_thread_stop(&StartupThread);
   //btrt_thread_exit(&StartupThread);
}

void Startup(void *thd){
   int err, i, id;

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned err = %d", err);
      exit(1);
   }
    
   /* Initialize and get a handle to the robot(s) */
   for(i = 0; i < busCount; i++){
      if(!(wam[i] = OpenWAM("../../wam.conf", i)))
         exit(1);
   }

   /* Set the safety limits for each bus (WAM) */
   for(i = 0; i < busCount; i++){
      if(!NoSafety) {
         /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
          * For now, the joint and tip velocities are ignored and
          * the elbow velocity provided is used for all three limits.
          */
         setSafetyLimits(i, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s
   
         /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical)
          * Note: The pucks are limited internally (see 'MT' in btsystem.c) 
          * Note: btsystem.c bounds the outbound torque to 8191, so 9000
          * tells the safety system to never register a critical fault
          */
         setProperty(i, SAFETY_MODULE, TL2, FALSE, 5700); 
         setProperty(i, SAFETY_MODULE, TL1, FALSE, 1800);
         
         /* Set the Max Torque (MT) for each motor
          * Notes: 
          * 1 Amp = 1024 puck torque units
          * Puck torques are saturated at 8191 by the communications layer
          * Cable limits were chosen to be approx 20% of the rated breaking strength
          *
          * Motor             1      2      3      4      5      6      7
          * Stall (Nm)       1.490  1.490  1.490  1.490  0.356  0.356  0.091
          * Peak (Nm)        6.31   6.31   6.31   6.31   1.826  1.826  0.613
          * Cable limit (Nm) 1.8    1.8    1.8    1.6    0.6    0.6    N/A
          * Nm/A -published  0.457  0.457  0.457  0.457  0.236  0.236  0.067
          * Nm/A -actual     0.379  0.379  0.379  0.379  0.157  0.157  0.058
          *
          * Example: 
          * What should the motor 3 Max Torque (MT) be set to in order to not
          * exceeed the designed cable limit?
          * (1.8 Nm) / (0.379 Nm/A) * (1024 Units/A) = 4863
          */
          
         setProperty(i, 1, MT, FALSE, 4860); // Cable limit = 4860
         setProperty(i, 2, MT, FALSE, 4860); // Cable limit = 4860
         setProperty(i, 3, MT, FALSE, 4860); // Cable limit = 4860
         setProperty(i, 4, MT, FALSE, 4320); // Cable limit = 4320
         setProperty(i, 5, MT, FALSE, 3900); // Cable limit = 3900
         setProperty(i, 6, MT, FALSE, 3900); // Cable limit = 3900
         setProperty(i, 7, MT, FALSE, 3200); // J7 Gears (max stall = 1600)
         
      }
      
      /* Prepare the WAM data */
      wamData[i].jdest = new_vn(len_vn(wam[i]->Jpos));
      wamData[i].cdest = new_vn(len_vn((vect_n*)wam[i]->HMpos));
   
      /* The WAM can be in either Joint mode or Cartesian mode.
       * We want a single set of variables (active_) to eliminate the need
       * for a whole bunch of if() statements.
       */
      wamData[i].active_bts = &(wam[i]->Jsc);
      setmode_bts(wamData[i].active_bts,SCMODE_IDLE);
      wamData[i].active_pos = wam[i]->Jpos;
      wamData[i].active_trq = wam[i]->Jtrq;
      wamData[i].active_dest = wamData[i].jdest;
      
      /* Create a new trajectory */
      wamData[i].vt_j = new_vta(len_vn(wam[i]->Jpos),50);
      wamData[i].vt_c = new_vta(len_vn((vect_n*)wam[i]->HMpos),50);
      wamData[i].vta = &wamData[i].vt_j;
      register_vta(wamData[i].active_bts,*wamData[i].vta);
      
      /* Initialize the control period */
      wamData[i].wam_thd.period = Ts;
      
      /* Register the control loop's local callback routine */
      registerWAMcallback(wam[i], WAMcallback);
   }
   
   startDone = TRUE;
   
   while (!btrt_thread_done((btrt_thread_struct*)thd)){
      usleep(10000);
   }
   btrt_thread_exit((btrt_thread_struct*)thd);
}

void xMainEventThread(void *thd){
   char     chr;
   int      i, j, cnt;
   
   /* Main event loop, ~10Hz */
   while (!done) {

      /* Are we currently in move_prep, waiting to start the trajectory? */
      if (move_prep)
      {
         /* If all moves are done ... */
         for(i = 0; i < busCount; i++) if (!MoveIsDone(wam[i])) break;
         if (i == busCount)
         {
            /* ... start each WAM's trajectory. */

            /* Start hand playback */
            cplay = 1;
            eventStart = btrt_get_time();
            eventIdx = 0;

            /* Start the actual trajectories */
            for(i = 0; i < busCount; i++) { 
               moveparm_bts(wamData[i].active_bts,vel,acc);
               if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
                  setmode_bts(wamData[i].active_bts,SCMODE_POS);
               start_trj_bts(wamData[i].active_bts);
            }

            /* and we're done with move_prep. */
            move_prep = 0;
         }
      }

      /* Are we currently done with trajectories, and should we loop? */
      if (loop_trj)
      {
         /* If all state controllers are in state BTTRAJ_DONE ... */
         for(i = 0; i < busCount; i++)
            if (get_trjstate_bts(wamData[i].active_bts) != BTTRAJ_DONE ) break;
         if (i == busCount)
         {
            /* ... restart each move. */
            for(i = 0; i < busCount; i++) { 
               /* Set up the trajectory */
               MoveSetup(wam[i],vel,acc);
               MoveWAM(wam[i], (*(wamData[i].active_bts->btt.reset))(&(wamData[i].active_bts->btt)) );
            }
            move_prep = 1;
         }
      }

      /* Check the active trajectory for completion for each bus */
      for(i = 0; i < busCount; i++){ 
         if (get_trjstate_bts(wamData[i].active_bts) == BTTRAJ_DONE && !loop_trj) {  // BZ-16Nov2005
            stop_trj_bts(wamData[i].active_bts);
            //setmode_bts(active_bts,prev_mode);
            cplay = 0;
         }
         
         /* Handle the data logger */
         //evalDL(&(wam[i]->log));
      }

      /* Check and handle user keypress */
      if ((chr = getch()) != ERR)
         ProcessInput(chr);
      
      /* If we are in playback mode, handle the BarrettHand teach commands */
      if(cplay){
         if(keyEvent[eventIdx].c){
            if((btrt_get_time() - eventStart) > keyEvent[eventIdx].t){
               ProcessInput(keyEvent[eventIdx].c);
               ++eventIdx;  
            }
         }
      }
#if 1   
      /* If the WAM has paused due to obstruction, wait */
      if(pauseCnt > 0){
         pauseCnt--;
         if(pauseCnt == 0){
            for(i = 0; i < busCount; i++){
               const_vn(wamData[i].active_dest, 0.0, 0.0, 0.0, 2.6, 0.0, 0.0, 0.0);
               
               MoveSetup(wam[i], vel, acc);
               MoveWAM(wam[i], wamData[i].active_dest);
               
               //moveparm_bts(wamData[i].active_bts,vel,acc);
               //if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
               //   setmode_bts(wamData[i].active_bts,SCMODE_POS);
               //if(moveto_bts(wamData[i].active_bts,wamData[i].active_dest))
               //   syslog(LOG_ERR,"Moveto Aborted");
            }

            /* Wait for all WAMs to finish moving */
            while (1)
            {
               for(i = 0; i < busCount; i++) if (!MoveIsDone(wam[i])) break;
               if (i == busCount) break;
               usleep(10000);
            }
            
            /* Enable the haptic scene */
            bth.state = 1;
            for(i = 0; i < busCount; i++)
               setmode_bts(wamData[i].active_bts, SCMODE_IDLE);
            pauseCnt = -2;
            loop_trj = 0;
         }
      }
      
      if(pauseCnt <= -2){
         /* Continue to wait while there is (interactive) movement */
         for(i = 0; i < busCount; i++){
            if((fabs(wamData[i].pstate.vel->q[0]) > 0.01) ||
               (fabs(wamData[i].pstate.vel->q[1]) > 0.01) ||
               (fabs(wamData[i].pstate.vel->q[2]) > 0.01) ){
               pauseCnt = -2;
            }
         }

         /* Check for loaded trajectory */
         for(i = 0; i < busCount; i++){
            if(*wamData[i].vta != NULL){
               if(numrows_vr(get_vr_vta(*wamData[i].vta)) > 10){
                  pauseCnt--;
                  break;
               }
            }
         }

         /* Turn off haptics and start playback */
         if(pauseCnt < -300){
            bth.state = 0;
            pauseCnt = -1;

            /* ... restart each move. */
            for(i = 0; i < busCount; i++) { 
               /* Set up the trajectory */
               MoveSetup(wam[i],vel,acc);
               MoveWAM(wam[i], (*(wamData[i].active_bts->btt.reset))(&(wamData[i].active_bts->btt)) );
            }
            move_prep = 1;
            loop_trj = 1;
         }
      }
      
      /* If there is an obstruction, pause the WAM playback */
      if (pauseCnt > -2) {
         for(i = 0; i < busCount; i++){
            for(cnt=0;cnt<wam[i]->dof;cnt++){
               if(fabs(getval_vn(wam[i]->Jtrq,cnt) - getval_vn(wam[i]->Gtrq,cnt)) > getval_vn(wam[i]->torq_limit,cnt)){
                  //syslog(LOG_ERR, "OverTorque on J%d, Jtrq[]=%s, Jpos[]=%s, Jref[]=%s", 
                  //cnt+1, sprint_vn(vect_buf1, wam[i]->Jtrq),
                  //sprint_vn(vect_buf2, wam[i]->Jpos),
                  //sprint_vn(vect_buf2, wam[i]->Jtref));
                  //sprintf(vect_buf4, "< %ld, %ld, %ld, %ld>", wam[i]->act[0].puck.position, wam[i]->act[1].puck.position, wam[i]->act[2].puck.position, wam[i]->act[3].puck.position));
                  pauseCnt = 50; /* 5 seconds (ish) */
                  /* Pause all WAMs */
                  for(j = 0; j < busCount; j++) {
                     pause_trj_bts(wamData[j].active_bts,5);
                  }
                  break;
               }
            }
            /* double-break if we've paused */
            if (cnt != wam[i]->dof) break;
         } 
      }

#endif

      /* Sleep for 0.1s. This roughly defines the event loop frequency */
      usleep(100000);
   }
   alldone = TRUE;
   btrt_thread_exit((btrt_thread_struct*)thd);
}

void MainEventThread(void *thd){
   char     chr;
   int      i, j, cnt;
   char     vect_buf1[500], vect_buf2[500], vect_buf3[500], vect_buf4[500];
   long     pos[7];
   
   /* Main event loop, ~10Hz */
   while (!done) {

      /* Are we currently in move_prep, waiting to start the trajectory? */
      if (move_prep)
      {
         /* If all moves are done ... */
         for(i = 0; i < busCount; i++) if (!MoveIsDone(wam[i])) break;
         if (i == busCount)
         {
            /* ... start each WAM's trajectory. */

            /* Start hand playback */
            cplay = 1;
            eventStart = btrt_get_time();
            eventIdx = 0;

            /* Start the actual trajectories */
            for(i = 0; i < busCount; i++) { 
               moveparm_bts(wamData[i].active_bts,vel,acc);
               if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
                  setmode_bts(wamData[i].active_bts,SCMODE_POS);
               start_trj_bts(wamData[i].active_bts);
            }

            /* and we're done with move_prep. */
            move_prep = 0;
         }
      }

      /* Are we currently done with trajectories, and should we loop? */
      if (loop_trj)
      {
         /* If all state controllers are in state BTTRAJ_DONE ... */
         for(i = 0; i < busCount; i++)
            if (get_trjstate_bts(wamData[i].active_bts) != BTTRAJ_DONE ) break;
         if (i == busCount)
         {
            /* ... restart each move. */
            for(i = 0; i < busCount; i++) { 
               /* Set up the trajectory */
               MoveSetup(wam[i],vel,acc);
               MoveWAM(wam[i], (*(wamData[i].active_bts->btt.reset))(&(wamData[i].active_bts->btt)) );
            }
            move_prep = 1;
         }
      }

      /* Check the active trajectory for completion for each bus */
      for(i = 0; i < busCount; i++){ 
         if (get_trjstate_bts(wamData[i].active_bts) == BTTRAJ_DONE && !loop_trj) {  // BZ-16Nov2005
            stop_trj_bts(wamData[i].active_bts);
            //setmode_bts(active_bts,prev_mode);
            cplay = 0;
         }
         
         /* Handle the data logger */
         //evalDL(&(wam[i]->log));
      }

      /* Check and handle user keypress */
      if ((chr = getch()) != ERR)
         ProcessInput(chr);
      
      /* If we are in playback mode, handle the BarrettHand teach commands */
      if(cplay){
         if(keyEvent[eventIdx].c){
            if((btrt_get_time() - eventStart) > keyEvent[eventIdx].t){
               ProcessInput(keyEvent[eventIdx].c);
               ++eventIdx;  
            }
         }
      }
      
      /* If the WAM has paused due to obstruction, wait */
      if(pauseCnt > 0){
         pauseCnt--;
	 if(pauseCnt == 0){
            for(i = 0; i < busCount; i++){
               unpause_trj_bts(wamData[i].active_bts,0.125);
            }
         }
      } 
#if 0
      /* If there is an obstruction, pause the WAM playback */
      for(i = 0; i < busCount; i++){
         for(cnt=0;cnt<wam[i]->dof;cnt++){
            if(fabs(getval_vn(wam[i]->Jtrq,cnt) - getval_vn(wam[i]->Gtrq,cnt)) > getval_vn(wam[i]->torq_limit,cnt)){
               //syslog(LOG_ERR, "OverTorque on J%d, Jtrq[]=%s, Jpos[]=%s, Jref[]=%s", 
               //cnt+1, sprint_vn(vect_buf1, wam[i]->Jtrq),
               //sprint_vn(vect_buf2, wam[i]->Jpos),
               //sprint_vn(vect_buf2, wam[i]->Jtref));
               //sprintf(vect_buf4, "< %ld, %ld, %ld, %ld>", wam[i]->act[0].puck.position, wam[i]->act[1].puck.position, wam[i]->act[2].puck.position, wam[i]->act[3].puck.position));
               pauseCnt = 50; /* 5 seconds (ish) */
               /* Pause all WAMs */
               for(j = 0; j < busCount; j++) {
                  pause_trj_bts(wamData[j].active_bts,5);
               }
               break;
            }
         }
         /* double-break if we've paused */
         if (cnt != wam[i]->dof) break;
      } 
#endif     
      /* Sleep for 0.1s. This roughly defines the event loop frequency */
      usleep(100000);
   }
   alldone = TRUE;
   btrt_thread_exit((btrt_thread_struct*)thd);
   
}

/** Entry point for the application.
    Initializes the system and executes the main event loop.
*/
int main(int argc, char **argv)
{
   char     chr;
   int      i;
   int      err = 0;
   char     thd_name[5];
   
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
#ifdef RTAI
   rt_allow_nonroot_hrt();
#endif

   /* Figure out what the keys do and print it on screen.
    * Parses this source file for lines containing "case '", because
    * that is how we manage keypresses in ProcessInput().
    */
   system("grep \"case '\" btdiag.c | sed 's/[[:space:]]*case \\(.*\\)/\\1/' > keys.txt");
   read_keys("keys.txt");

   /* Initialize the ncurses screen library */
   init_ncurses();
   atexit((void*)endwin);

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);
   syslog(LOG_ERR,"...Starting btdiag program...");

   /* Look through the command line arguments for "-q" */
   //mvprintw(18,0,"argc=%d",argc);
   //for(i = 0; i < argc; i++) mvprintw(20+i,0,"%s",argv[i]);
   for(i = 1; i < argc; i++) {
      if(!strcmp(argv[i],"-q"))
         quiet = TRUE; // Flag to skip the startup walkthrough text
   }

   /* Do we want to bypass the safety circuit + pendant? 
    * This is for diagnostics only and should not normally be used .
    */
   NoSafety = 0;
   for(i = 1; i < argc; i++) {
      if(!strcmp(argv[i],"-ns"))
         NoSafety = 1;
   }
   
   if(!quiet) {
      /* Lead the user through a proper WAM startup */
      mvprintw(1,0,"Make sure the all WAM power and signal cables are securely");
      mvprintw(2,0,"fastened, then turn on the main power to WAM and press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
      mvprintw(4,0,"Make sure all E-STOPs are released, then press Shift-Idle");
      mvprintw(5,0,"on the control pendant. Then press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
      mvprintw(7,0,"Place WAM in its home (folded) position, then press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
   }

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);
   
   /* Read the WAM configuration file */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig returned err = %d", err);
      exit(1);
   }
   
   /* RT task for setup of CAN Bus */
   btrt_thread_create(&StartupThread, "StTT", 45, (void*)Startup, NULL);
   while(!startDone)
      usleep(10000);
   
   /* NOTE: Buffer dumps are handled by the WAMMaintenanceThread() which is spun
    * off inside OpenWAM(). Buffer data is recorded automatically from the 
    * WAMControlThread()'s TriggerDL() function call.
    */
    
   /* Configure the logDivider: 1 = every control cycle, 2 = every other cycle, etc. */
   wam[0]->logDivider = 1;
   
   /* Prepare the datalogger with the max number of btreal fields per record */
   PrepDL(&(wam[0]->log), 35);
   
   /* Add a pointers to some data to log */
   /* NOTE: log_time begins counting seconds when the WAMControlThread() is started */
   AddDataDL(&(wam[0]->log), &(wam[0]->log_time), sizeof(double), BTLOG_DOUBLE, "Time(s)");
   AddDataDL(&(wam[0]->log), valptr_vn((vect_n*)wam[0]->Jpos),sizeof(btreal) * len_vn((vect_n*)wam[0]->Jpos), BTLOG_BTREAL,"Jpos(rad)");
   AddDataDL(&(wam[0]->log), valptr_vn((vect_n*)wam[0]->Mpos),sizeof(btreal) * len_vn((vect_n*)wam[0]->Mpos), BTLOG_BTREAL,"Mpos(rad)");
   AddDataDL(&(wam[0]->log), valptr_vn((vect_n*)wam[0]->Jtrq),sizeof(btreal) * len_vn((vect_n*)wam[0]->Jtrq), BTLOG_BTREAL,"Jtrq(rad)");
   AddDataDL(&(wam[0]->log), valptr_vn((vect_n*)wam[0]->Mtrq),sizeof(btreal) * len_vn((vect_n*)wam[0]->Mtrq), BTLOG_BTREAL,"Mtrq(rad)");
   AddDataDL(&(wam[0]->log), (void*)(jointPosition+1), sizeof(long) * len_vn((vect_n*)wam[0]->Mtrq), BTLOG_LONG, "JointEncoder(cts)");

   /* Initialize the datalogging buffer size and output file.
    * Once the buffer is full, the data is written to disk. Datalogging continues
    * even as the buffer is being written out.
    */
   InitDL(&(wam[0]->log), 1000, "datafile.dat");
   
   cdest = new_mh();
   RxRyRz = new_v3();
   
   /* Initialize the haptic scene */
   init_haptics();
   
   /* Spin off the WAM control thread(s) */
   for(i = 0; i < busCount; i++) {
      strcpy(thd_name,"WAMx");
      thd_name[3] = '0' + i;
      btrt_thread_create(&wamData[i].wam_thd, thd_name, 50, (void*)WAMControlThread, (void*)wam[i]);
   }
   
   /* Initialize the active teach filename (to blank) */
   active_file[0] = 0;

   /* Open serial port */
   if(err = serialOpen(&p, "/dev/ttyS0")) {
      syslog(LOG_ERR, "Error opening serial port: %d", err);
   }
   serialSetBaud(&p, 9600); // The BarrettHand defaults to 9600 baud
   
   /* Initialize the display mutex */
   test_and_log(
      btrt_mutex_init(&disp_mutex),
      "Could not initialize mutex for displays.");
      
   /* Spin off the display thread */
   btrt_thread_create(&disp_thd, "DISP", 10, (void*)DisplayThread, NULL);
   
   /* Spin off the BHand demo thread */
   btrt_thread_create(&disp_thd, "DEMO", 10, (void*)DemoThread, NULL);
   //btthread_create(&disp_thd, 0, (void*)DisplayThread, NULL);
   
   /* Spin off the audio thread */
   //btthread_create(&audio_thd,0,(void*)AudioThread,NULL);

   /* Spin off the main event loop */
   btrt_thread_create(&event_thd, "EVENT", 20, (void*)MainEventThread, NULL);
   
   while(!alldone){
      usleep(100000);
   }
   
   usleep(100000);
   Cleanup();
   exit(1);
}

/* This function is called from the WAMControlThread() after the positions
 * have been received from the WAM (and after all the kinematics are calculated)
 * but before torques are sent to the WAM.
 */
int WAMcallback(struct btwam_struct *w)
{
   int i;


   if(force){
   const_v3(w->Cforce, 0.0, 0.0, 10.0);
   const_v3(w->Ctrq, 0.0, 0.0, 0.0);
         apply_tool_force_bot(&(w->robot), w->Cpoint, w->Cforce, w->Ctrq);
         //apply_force_bot(&(w->robot), 2, w->Cpoint, w->Cforce, w->Ctrq);
   }

   /* Handle haptic scene for the specified WAM */
   for(i = 0; i < busCount; i++){
      if(wam[i] == w){
         /* Filter the Cartesian endpoint position to generate the endpoint
          * velocity and acceleration.
          */
         eval_state_btg(&wamData[i].pstate, w->Cpos);
         
         /* Evaluate the haptic scene.
          * Uses the WAM's position and velocity along with the scene definition
          * to determine the required end-of-arm force vector.
          */
         eval_bthaptics(&bth, (vect_n*)w->Cpos, (vect_n*)wamData[i].pstate.vel, (vect_n*)zero_v3, (vect_n*)w->Cforce);
         
         /* Apply the calculated force vector to the robot */
         apply_tool_force_bot(&(w->robot), w->Cpoint, w->Cforce, w->Ctrq);
      }
   }
   
   return 0;
}

/* Parse the input file into an array of strings to be displayed on the help screen */
void read_keys(char *filename)
{
   FILE *inf;
   int done = 0;
   int cnt = 0;
   int len = 100;
   int ret;

   num_commands = 0;
   inf = fopen(filename,"r");
   ret = getline(&(command_help[num_commands]),&len,inf);
   if (inf != NULL) {
      while (!done) {
         command_help[num_commands] = (char*)btmalloc(100);
         ret = getline(&(command_help[num_commands]),&len,inf);
         if (ret == -1 || num_commands > 98)
            done = 1;
         else
            command_help[num_commands][ret-1] = 0;
         num_commands++;
      }
   }
   fclose(inf);
   inf = fopen("test.out","w");
   for(cnt = 0;cnt < num_commands;cnt++)
      fprintf(inf,"%s",command_help[cnt]);
   fclose(inf);
}

/* Initialize the haptic scene with various objects */
void init_haptics(void)
{
   int cnt, i;
   btreal xorig,yorig,zorig;
   int objectCount = 0;

   /* Define some variables to store point data */
   p1 = new_v3();
   p2 = new_v3();
   p3 = new_v3();
   
   /* Define the offset from the origin */
   xorig = 0.0;
   yorig = 0.0;
   zorig = 0.10;

   /* Allocate a scene with space for 10 objects */
   new_bthaptic_scene(&bth, 10);
   
   /* For each WAM (bus), initialize a structure to track velocity and accel, given position */
   for(i = 0; i < busCount; i++){
      /* Initialize the position filter (btgeometry) */
      init_state_btg(&wamData[i].pstate, Ts, 30.0); // Update rate, filter cutoff Hz
      
      /* Define the Haptic interaction point with respect to the WAM tool frame */
      const_v3(wam[i]->Cpoint, 0.0, 0.0, 0.0); // Define the interaction point to be at the tool
   }
   
   /* To add a haptic object to the scene, you must:
    * 1) Define the object geometry
    * 2) Define a type of haptic interaction (method of force response)
    * 3) Tie the object geometry and haptic interaction together into a haptic object
    * 4) Add the new haptic object to the scene
    */
    
   /* Create workspace bounding box */
   init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
   init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
   init_normal_box_bth(&objects[objectCount],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
   addobject_bth(&bth,&objects[objectCount++]);
   
   /* Create nested spheres */
   init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0); // Inner sphere, outer wall
   init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1); // Inner sphere, inner wall
   init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0); // Outer sphere, outer wall
   init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1); // Outer sphere, inner wall
   /* Perform steps 2-4 (above) for each sphere */
   for(cnt = 0;cnt < 4;cnt++) {
      init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
      init_normal_sphere_bth(&objects[objectCount],&spheres[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
      addobject_bth(&bth,&objects[objectCount++]);
   }
}

/* Initialize the ncurses screen library */
void init_ncurses(void)
{
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
}

/** Traps the Ctrl-C signal.
    Quits the program gracefully when Ctrl-C is hit.
*/
void sigint_handler()
{
   Cleanup();
   exit(1);
}

int waitForStop(){
    double F1, F2, F3, F4;
    double EPS = 0.001;
    
    do{
        F1 = wam[0]->Jpos->q[wam[0]->dof-4];
        F2 = wam[0]->Jpos->q[wam[0]->dof-3];
        F3 = wam[0]->Jpos->q[wam[0]->dof-2];
        F4 = wam[0]->Jpos->q[wam[0]->dof-1];
        usleep(250000); // Sleep for 1/4 sec
    }while(fabs(F1 - wam[0]->Jpos->q[wam[0]->dof-4]) > EPS ||
       fabs(F2 - wam[0]->Jpos->q[wam[0]->dof-3]) > EPS ||
       fabs(F3 - wam[0]->Jpos->q[wam[0]->dof-2]) > EPS ||
       fabs(F4 - wam[0]->Jpos->q[wam[0]->dof-1]) > EPS );
       
    return(0);
}




int Command(char *cmd){
    while(!demo) usleep(250000);
    
    if(!strcmp("123FSET MCV 200",cmd)){
        setProperty(0,11,MCV,FALSE,400);
        setProperty(0,12,MCV,FALSE,400);
        setProperty(0,13,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("123FSET MOV 200",cmd)){
        setProperty(0,11,MOV,FALSE,400);
        setProperty(0,12,MOV,FALSE,400);
        setProperty(0,13,MOV,FALSE,400);
        return 0;
    }
    if(!strcmp("4FSET MCV 100",cmd)){
        setProperty(0,14,MCV,FALSE,200);
        return 0;
    }
    if(!strcmp("4FSET MOV 100",cmd)){
        setProperty(0,14,MOV,FALSE,200);
        return 0;
    }
    if(!strcmp("GO",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("SO",cmd)){
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("123C",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123O",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("SC",cmd)){
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("GC",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("C",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M",cmd)){
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SIC",cmd)){
        setProperty(0,14,CMD,FALSE,14);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 11000",cmd)){
        setProperty(0,11,E,FALSE,-130000);
        setProperty(0,12,E,FALSE,-130000);
        setProperty(0,13,E,FALSE,-130000);
        
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 11500",cmd)){
        setProperty(0,11,E,FALSE,-135300);
        setProperty(0,12,E,FALSE,-135300);
        setProperty(0,13,E,FALSE,-135300);
        
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("4FSET DS 630",cmd)){
        setProperty(0,14,_DS,FALSE,-7300);
        return 0;
    }
    if(!strcmp("123FSET DP 4000",cmd)){
        setProperty(0,11,E,FALSE,-47000);
        setProperty(0,12,E,FALSE,-47000);
        setProperty(0,13,E,FALSE,-47000);
        return 0;
    }
    if(!strcmp("SFSET MCV 100",cmd)){
        setProperty(0,14,MCV,FALSE,200);
        return 0;
    }
    if(!strcmp("1234O",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 9000",cmd)){
        setProperty(0,13,E,FALSE,-106000);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 9000",cmd)){
        setProperty(0,12,E,FALSE,-106000);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 9000",cmd)){
        setProperty(0,11,E,FALSE,-106000);
        setProperty(0,11,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 17000",cmd)){
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 17000",cmd)){
        setProperty(0,12,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 17000",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 13000",cmd)){
        setProperty(0,13,E,FALSE,-153000);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 13000",cmd)){
        setProperty(0,12,E,FALSE,-153000);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 13000",cmd)){
        setProperty(0,11,E,FALSE,-153000);
        setProperty(0,11,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3FSET MCV 75",cmd)){
        setProperty(0,13,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("2FSET MCV 75",cmd)){
        setProperty(0,12,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("1FSET MCV 75",cmd)){
        setProperty(0,11,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("SFSET MCV 50",cmd)){
        setProperty(0,14,MCV,FALSE,100);
        return 0;
    }
    if(!strcmp("3FSET MCV 65",cmd)){
        setProperty(0,13,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("2FSET MCV 65",cmd)){
        setProperty(0,12,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("1FSET MCV 65",cmd)){
        setProperty(0,11,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("O",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 11500",cmd)){
        setProperty(0,13,E,FALSE,-135300);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 14000",cmd)){
        setProperty(0,12,E,FALSE,-164700);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SM 1000",cmd)){
        setProperty(0,14,E,FALSE,-11600);
        setProperty(0,14,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 2000",cmd)){
        setProperty(0,11,E,FALSE,-23500);
        setProperty(0,12,E,FALSE,-23500);
        setProperty(0,13,E,FALSE,-23500);
        
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 5000",cmd)){
        setProperty(0,11,E,FALSE,-58800);
        setProperty(0,12,E,FALSE,-58800);
        setProperty(0,13,E,FALSE,-58800);
        
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3FSET MCV 200",cmd)){
        setProperty(0,13,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("3FSET MCV 85",cmd)){
        setProperty(0,13,MCV,FALSE,170);
        return 0;
    }
    if(!strcmp("2FSET MCV 200",cmd)){
        setProperty(0,12,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("1FSET MCV 200",cmd)){
        setProperty(0,11,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("2FSET DP 11500",cmd)){
        setProperty(0,12,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("GM",cmd)){
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1FSET DP 6000",cmd)){
        setProperty(0,11,E,FALSE,-70000);
        return 0;
    }
    if(!strcmp("2FSET DP 6000",cmd)){
        setProperty(0,12,E,FALSE,-70000);
        return 0;
    }
    if(!strcmp("1FSET DP 11500",cmd)){
        setProperty(0,11,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("3FSET DP 11500",cmd)){
        setProperty(0,13,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("3C",cmd)){
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("2C",cmd)){
        setProperty(0,12,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("1C",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("12M 11500",cmd)){
        setProperty(0,11,E,FALSE,-135300);
        setProperty(0,12,E,FALSE,-135300);
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SM 1570",cmd)){
        setProperty(0,14,E,FALSE,-17975);
        setProperty(0,14,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3O",cmd)){
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("2O",cmd)){
        setProperty(0,12,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("1O",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("123SC",cmd)){
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123SO",cmd)){
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 15000",cmd)){
        setProperty(0,12,E,FALSE,-176500);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 10000",cmd)){
        setProperty(0,13,E,FALSE,-117600);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 5000",cmd)){
        setProperty(0,11,E,FALSE,-58800);
        setProperty(0,11,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    
    syslog(LOG_ERR, "Unhandled BHand command: %s", cmd);

}

/** Spins in a loop, updating the BHand.
    Runs as its own thread.
*/
void DemoThread()
{
   int result, i;
#if 0
   while(!demo) usleep(250000);
   setProperty(0,11,CMD,FALSE,13);
   waitForStop();
        setProperty(0,12,CMD,FALSE,13);
        waitForStop();
        setProperty(0,13,CMD,FALSE,13);
        waitForStop();
        setProperty(0,14,CMD,FALSE,13);
        waitForStop();
#endif
   /* Loop forever, rendering the appropriate screen information */
   while (!done) {
      
	/* Set up FPG, SAMPLE, ACCEL, and Max Velocities for fingers 1-3 - BZ 19Sep2003 */
	if( result=Command ( "123FSET FPG 5" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123FSET TSTOP 300" ) )
		Error();
	if( _kbhit() )
		;
	
	if( result=Command ( "123FSET MCV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123FSET MOV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123FSET SAMPLE 63" ) )
		Error();
	if( _kbhit() )
		;
	
	if( result=Command ( "123FSET ACCEL 4" ) )
		Error();
	if( _kbhit() )
		;

	/* Set up FPG, SAMPLE, ACCEL, and Max Velocities for spread - BZ 19Sep2003 */
	if( result=Command ( "4FSET FPG 5" ) )
		Error();
	if( _kbhit() )
		;
	
	if( result=Command ( "4FSET MCV 100" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "4FSET MOV 100" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "4FSET SAMPLE 200" ) )
		Error();
	if( _kbhit() )
		;
	
	if( result=Command ( "4FSET ACCEL 8" ) )
		Error();
	if( _kbhit() )
		;

	/* Perform supervisory demonstration */
	if( result=Command ( "GO" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "SO" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123O" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "SC" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123O" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1M 5000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3M 10000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2M 15000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123SO" ) )
		Error();
	if( _kbhit() )
		;

//AC 010711 Removed following command because it was casing fingers to hit eachother (version 4.21)

	//if( result=Command ( "123SC" ) )
	//	Error();
	//if( _kbhit() )
	//	;

	if( result=Command ( "SC" ) )
		Error();
	if( _kbhit() )
		;
	
	if( result=Command ( "123C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1O" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3O" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2O" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "SM 1570" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "12M 11500" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123O" ) )
		Error();
	if( _kbhit() )
		;

	
	/* Do the "Back and Forth" */
	if( result=Command ( "3FSET DP 11500" ) )
		Error();
	if( _kbhit() )
		;

	for(i = 0; i < 3; i++)
	{
		if( result=Command ( "1FSET DP 11500" ) )
			Error();
		if( _kbhit() )
			;

		if( result=Command ( "2FSET DP 6000" ) )
			Error();
		if( _kbhit() )
			;

		if( result=Command ( "GM" ) )
			Error();
		if( _kbhit() )
			;
		
		if( result=Command ( "1FSET DP 6000" ) )
			Error();
		if( _kbhit() )
			;

		if( result=Command ( "2FSET DP 11500" ) )
			Error();

		if( result=Command ( "GM" ) )
			Error();
		if( _kbhit() )
			;
	}

	if( result=Command ( "123M 11500" ) )
		Error();
	if( _kbhit() )
		;

	/* End of Back and Forth */

	if( result=Command ( "1FSET MCV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2FSET MCV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3FSET MCV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123M 5000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "SC" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123M 2000" ) )
		Error();
	if( _kbhit() )
		;

	/* Do finger jumble */

	if( result=Command ( "SM 1000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2M 14000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3M 11500" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "O" ) )
		Error();
	if( _kbhit() )
		;

	/* End finger Jumble */

	if( result=Command ( "1FSET MCV 65" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2FSET MCV 65" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3FSET MCV 85" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "SFSET MCV 50" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123SC" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1FSET MCV 75" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2FSET MCV 75" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3FSET MCV 75" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1M 13000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3M 13000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2M 13000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1M 17000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3M 17000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2M 17000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1M 9000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "3M 9000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "2M 9000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "1234O" ) )
		Error();
	if( _kbhit() )
		;

	/* Do Spread Inc Close with finger movement */

	if( result=Command ( "SFSET MCV 100" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123FSET MCV 200" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "123FSET DP 4000" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "4FSET DS 630" ) )
		Error();
	if( _kbhit() )
		;

	for(i = 0; i < 5; i++)
	{
		if( result=Command ( "123M 11000" ) )
			Error();
		if( _kbhit() )
			;

		if( result=Command ( "123M" ) )
			Error();
		if( _kbhit() )
			;

		if( result=Command ( "SIC" ) )
			Error();
		if( _kbhit() )
			;
	}


	if( result=Command ( "C" ) )
		Error();
	if( _kbhit() )
		;

	if( result=Command ( "GO" ) )
		Error();
	if( _kbhit() )
		;

   }

}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
   /* Clear the screen buffer */
   clear();
   
   /* Display the cleared screen */
   refresh();
   
   /* Loop forever, rendering the appropriate screen information */
   while (!done) {
      /* Try to obtain a mutex lock to refresh the screen.
       * The only time the mutex is unavailable is when the user is
       * typing an answer to an on-screen prompt.
       * See start_entry() and finish_entry()
       */
      test_and_log(
         btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
         
      /* Render the appropriate screen, based on the "screen" variable */
      switch(screen) {
      case SCREEN_MAIN:
         RenderMAIN_SCREEN();
         break;
      case SCREEN_HELP:
         RenderHELP_SCREEN();
         break;
      }
      
      /* Release the mutex lock */
      btrt_mutex_unlock(&(disp_mutex));
      
      /* Slow this loop down to about 10Hz */
      usleep(100000);
   }

}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
   int err;
   test_and_log(
      btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
   move(entryLine, 1);
   echo();
   timeout(-1);
}

/** Unlocks the display mutex.
    Allows the computer to resume automatically updating the screen.
*/
void finish_entry()
{
   noecho();
   timeout(0);
   move(entryLine, 1);
   addstr("                                                                              ");
   refresh();
   btrt_mutex_unlock( &(disp_mutex) );
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderMAIN_SCREEN()
{
   int cnt, idx, Mid, cp;
   int line, line2;
   int cpt, nrows;
   double gimb[4],tacc,tvel;
   vectray* vr;
   char vect_buf1[2500];

   /***** Display the interface text *****/
   line = 0;

   mvprintw(line , 0, "Barrett Technology - Diagnostic Application\t\tPress 'h' for help");
   line+=2;

   // Show MODE
   if (wamData[0].active_bts == &(wam[0]->Jsc)) {
      mvprintw(line, 0, "Mode       : Joint Space    ");
   } else if (wamData[0].active_bts == &(wam[0]->Csc)) {
      mvprintw(line, 0, "Mode       : Cartesian Space");
   } else {
      mvprintw(line, 0, "Mode       : Undefined!!!   ");
   }
   ++line;

   // Show CONSTRAINT
   if (getmode_bts(wamData[0].active_bts)==SCMODE_IDLE)
      mvprintw(line, 0, "Constraint : IDLE      ");
   else if (getmode_bts(wamData[0].active_bts)==SCMODE_POS)
      mvprintw(line, 0, "Constraint : POSITION  ");
   else if (getmode_bts(wamData[0].active_bts)==SCMODE_TRJ)
      mvprintw(line, 0, "Constraint : TRAJECTORY");
   else
      mvprintw(line, 0, "Constraint : UNDEFINED!");
   ++line;
   
   // Show the state of gravity compensation
   mvprintw(line, 0, "GravityComp: %3.1f, %s", GetGravityComp(wam[0]),
            GetGravityUsingCalibrated(wam[0]) ? "Calibrated   " : "Uncalibrated " );
   ++line;
   
   // Show the state of the haptic scene
   if (bth.state) {
      mvprintw(line, 0, "Haptics    : ON    ");
   } else {
      mvprintw(line, 0, "Haptics    : OFF    ");
   }
   line+=1;

   // Show TRAJECTORY
   if (cteach)
      mvprintw(line, 0, "Trajectory : Teaching continuous trajectory");
   else if (*wamData[0].vta == NULL)
      mvprintw(line, 0, "Trajectory : NONE                         ");
   else
      mvprintw(line, 0, "Trajectory : %s                           ",*active_file?active_file:"NONE");
   ++line;
   ++line;

   for(cnt = 0; cnt < busCount; cnt++){
      mvprintw(line, 0, "Name       : %s", wam[cnt]->name);
      ++line;
      /*
      mvprintw(line, 0, "Velocity   : %+8.4f  ",vel);
      ++line;
      mvprintw(line, 0, "Accel      : %+8.4f  ",acc);
      ++line;
      */

	  //mvprintw(line, 0, "J Encoder  : <%ld, %ld, %ld, %ld> ", 
	  	//jointPosition[1], jointPosition[2], jointPosition[3], jointPosition[4]);
      //line+=1;
      mvprintw(line, 0, "J Position : %s ", sprint_vn(vect_buf1, wam[cnt]->Jpos));
      line+=1;
      mvprintw(line, 0, "J Torque   : %s ", sprint_vn(vect_buf1, wam[cnt]->Jtrq));
      line+=1;
      mvprintw(line, 0, "C Position : \n%s ", sprint_mn(vect_buf1, (matr_mn*)wam[cnt]->HMpos));
      line+=5;
      
      /* Get rotation matrix in RxRyRz format */
      RtoXYZf_m3( wam[cnt]->HMpos, RxRyRz );
      mvprintw(line, 0, "C Rotation : %s ", sprint_vn(vect_buf1, RxRyRz));
      line+=2;
      
      //mvprintw(line, 0, "TrajState  : %d ", wamData[cnt].active_bts->btt.state);
      //line+=1;
      
      if (*wamData[cnt].vta != NULL) { // print current point
         vr = get_vr_vta(*wamData[cnt].vta);
         cpt = get_current_idx_vta(*wamData[cnt].vta);
         nrows = numrows_vr(vr);
         mvprintw(line,0,"Teach Point: %d of %d      ",cpt,nrows-1);
         line++;
   
         mvprintw(line  , 0 , "Previous   :\t\t\t\t\t\t\t\t\t\t");
         mvprintw(line+1, 0 , "Current    :\t\t\t\t\t\t\t\t\t\t");
         mvprintw(line+2, 0 , "Next       :\t\t\t\t\t\t\t\t\t\t");
   
         // Previous
         if (nrows > 0 && cpt > 0)
            mvprintw(line, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));
   
         // Current
         if (nrows > 0) {
            if (nrows != cpt) {
               mvprintw(line+1, 13 , "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
            } else {
               mvprintw(line+1, 13 , "END OF LIST");
            }
         } else {
            mvprintw(line+1, 13 , "EMPTY LIST");
         }
   
         // Next
         if (nrows > 1) {
            if (cpt < nrows-1) {
               mvprintw(line+2, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
            } else if (cpt == nrows-1) {
               mvprintw(line+2, 13, "END OF LIST");
            }
         }
         line += 3;
      } else {
         line++;
         line++;
         mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
         line += 2;
      }
   }
   
   //mvprintw(line, 0, "PauseCnt   : %d    ", pauseCnt);
   //++line;
   
   line +=1;
   mvprintw(line,0,"");
   entryLine = line;
   refresh();
}

/* Show the available commands on-screen */
void RenderHELP_SCREEN()
{
   int cnt, line = 0;

   mvprintw(line, 0, "Help Screen - (press 'h' to toggle)");
   line += 2;
   for (cnt = 0; cnt < num_commands;cnt++) {
      if (cnt % 2) {
         mvprintw(line,40,"%.39s",command_help[cnt]);
         line += 1;
      } else {
         mvprintw(line,0,"%.39s",command_help[cnt]);

      }
   }
   refresh();
}

/* Clear the screen while honoring the mutex lock */
void clearScreen(void)
{
   btrt_mutex_lock(&(disp_mutex));
   clear();
   btrt_mutex_unlock(&(disp_mutex));
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
   int cnt,elapsed = 0, i;
   double ftmp,tacc,tvel;
   int dtmp,status;
   COMMAND cmd;
   char fn[250],fn2[250],chr;
   int ret;
   int done1;
   btreal zPos;

   switch (c)
   {
   case 'e':
      wam[0]->JposControl.pid[0].Kp = 0.0;
      wam[0]->JposControl.pid[0].Kd = 0.0;
      wam[0]->JposControl.pid[0].Ki = 0.0;
      
   break;
   case 'x'://eXit
   case  'X'://eXit
      done = 1;
      break;
   case 'f'://Toggle Force
      force = !force;
      break;
   case '!'://Set puck to mode TORQ
      if (NoSafety)
         EnergizeActuators();
      //setProperty(0, 1, MODE, FALSE, 2);
      break;
   case '@'://Set puck to mode IDLE
      if (NoSafety)
         IdleActuators();
      //setProperty(0, 1, MODE, FALSE, 0);
      break;
   case '#'://Data logging on
      for(i = 0; i < busCount; i++){
         DLon(&(wam[i]->log));
      }
      break;
   case '3'://Data logging off
      for(i = 0; i < busCount; i++){
         DLoff(&(wam[i]->log));
      }
      break;

   case '4'://BHand GC
      serialWriteString(&p, "\rGC\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '5'://BHand GO
      serialWriteString(&p, "\rGO\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '6'://BHand GM 8000
      serialWriteString(&p, "\rGM 8000\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '7'://BHand SO
      serialWriteString(&p, "\rSO\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '8'://BHand SM 1000
      serialWriteString(&p, "\rSM 1000\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '9'://BHand SM 1500
      serialWriteString(&p, "\rSM 1500\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '0'://BHand HI
      if (ioperm(0x378,1,1)) 
         fprintf(stderr, "ERROR: Can't gain access to parallel port\n"), exit(1);
      outb((unsigned char)0x05, 0x378); 	// Output Data to the Parallel Port
      sleep(1);
      
      serialWriteString(&p, "\rHI\r");
      if(cteach){
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = btrt_get_time() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;

   case 'g'://Set gravity compensation
      //gravity = !gravity;
      start_entry();
      addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
      refresh();
      scanw("%lf\n",  &tvel);
      for(i = 0; i < busCount; i++){
         SetGravityComp(wam[i],tvel);
      }
      finish_entry();
      break;
   case 'H'://Hand command
      //gravity = !gravity;
      start_entry();
      addstr("GCL BarrettHand Command: ");
      refresh();
      getstr( fn);
      //syslog(LOG_ERR, "fn[%s]", fn);
      strcpy(cmd.command, fn);
    //syslog(LOG_ERR, "cmd[%s]", cmd.command);
      finish_entry();
      parseInput(&cmd);
      break;
   case 'G'://Toggle gcravity compensation mode
      if (GetGravityUsingCalibrated(wam[0]))
         SetGravityUsingCalibrated(wam[0],0);
      else
         SetGravityUsingCalibrated(wam[0],1);
      break;
      
   case '_'://Refresh display
      clearScreen();
      break;
   case '\t'://Toggle jointspace and cartesian space
      for(i = 0; i < busCount; i++){
         destroy_vta(wamData[i].vta); //empty out the data if it was full
         setmode_bts(&(wam[i]->Jsc),SCMODE_IDLE);
         setmode_bts(&(wam[i]->Csc),SCMODE_IDLE);
   
         if (wamData[i].active_bts == &(wam[i]->Jsc)) { //switch to cartesian space mode.
            SetCartesianSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Csc);
            wamData[i].active_pos = (vect_n*)wam[i]->HMpos;
            wamData[i].active_trq = (vect_n*)wam[i]->HMft;
            wamData[i].active_dest = wamData[i].cdest;
            wamData[i].vta = &wamData[i].vt_c;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         } else {
            SetJointSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Jsc);
            wamData[i].active_pos = wam[i]->Jpos;
            wamData[i].active_trq = wam[i]->Jtrq;
            wamData[i].active_dest = wamData[i].jdest;
            wamData[i].vta = &wamData[i].vt_j;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      }
      clearScreen();
      break;
   case 'D'://Haptics on
      bth.state = 1;
      break;
   case 'd'://Haptics off
      bth.state = 0;
      break;
   case 'p'://Turn on/off Constraint
      for(i = 0; i < busCount; i++){
         if (getmode_bts(wamData[i].active_bts)!=SCMODE_IDLE)
            setmode_bts(wamData[i].active_bts,SCMODE_IDLE);
         else
            setmode_bts(wamData[i].active_bts,SCMODE_POS);
         /*
         setrange_vn((vect_n*)xyz, wam[i]->R6ref, 0, 3, 3);
         XYZftoR_m3(r_mat, xyz);
         getcol_m3(ns, r_mat, 0);
         getcol_m3(os, r_mat, 1);
         getcol_m3(as, r_mat, 2);
         //set_m3(r_mat, wam[i]->robot.tool->origin);
         */
      }
      
      
      break;
   case '.'://Play loaded trajectory
      for(i = 0; i < busCount; i++){
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {
   
            /* Set up the trajectory */
            MoveSetup(wam[i],vel,acc);
            MoveWAM(wam[i], (*(wamData[i].active_bts->btt.reset))(&(wamData[i].active_bts->btt)) );
            move_prep = 1;

            loop_trj = 0;
            pauseCnt = -2;
         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
      break;

   case 'b'://Simulate loaded trajectory

      //sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
      break;

   case '?'://Loop loaded trajectory
      for(i = 0; i < busCount; i++){
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {
   
            /* Set up the trajectory */
            MoveSetup(wam[i],vel,acc);
            MoveWAM(wam[i], (*(wamData[i].active_bts->btt.reset))(&(wamData[i].active_bts->btt)) );
            move_prep = 1;

            loop_trj = 1;
            pauseCnt = -2;
         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
      pauseCnt = -1;
      break;

   case '/'://Stop loaded trajectory
      for(i = 0; i < busCount; i++){
         stop_trj_bts(wamData[i].active_bts);
         //setmode_bts(active_bts,prev_mode);
         loop_trj = 0;
         move_prep = 0;
         pauseCnt = -1;
      }
      break;
   case 'Y'://Start continuous teach
      for(i = 0; i < busCount; i++){
         sprintf(fn, "teachpath_%d", i);
         StartContinuousTeach(wam[i], 25, fn); // Begin logging data at 1/25 the control rate
      }
      cteach = 1;
      eventIdx = 0;
      eventStart = btrt_get_time()-750000000L;
      keyEvent[eventIdx].c = 0;
      
      break;
   case 'y'://Stop continuous teach
      for(i = 0; i < busCount; i++){
         StopContinuousTeach(wam[i]);
         sprintf(fn, "teachpath_%d", i);
         sprintf(fn2, "teach_%d.csv", i);
         DecodeDL(fn, fn2, 0);
         cteach = 0;
         stop_trj_bts(wamData[i].active_bts);
         /** \internal \todo sleeps that are necessary might be forgotten. Can we eliminate the need?*/
         usleep(10000); //needed to give the command a chance to work.
         destroy_vta(wamData[i].vta); //empty out the data if it was full
         strcpy(active_file,"teach.csv");
         *wamData[i].vta = read_file_vta(fn2, 20);
         register_vta(wamData[i].active_bts, *wamData[i].vta);
      }
      break;
   case 'l'://Load trajectory from file
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);
         
         for(i = 0; i < busCount; i++){
            sprintf(fn, "%s_%d", active_file, i);
            destroy_vta(wamData[i].vta); //empty out the data if it was full
            *wamData[i].vta = read_file_vta(fn, 20);
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;
   case 'w'://Save trajectory to a file
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);
         for(i = 0; i < busCount; i++){
            sprintf(fn, "%s_%d", active_file, i);
            if (*wamData[i].vta != NULL) {
               write_file_vta(*wamData[i].vta, fn);
            }
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;
   case 'n'://Create a new trajectory
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter the max number of points that will be in your trajectory: ");
         refresh();
         ret = scanw("%d", &dtmp);
         
         for(i = 0; i < busCount; i++){
            destroy_vta(wamData[i].vta);
            strcpy(active_file, user_def);
            *wamData[i].vta = new_vta(len_vn(wamData[i].active_pos), dtmp);
            register_vta(wamData[i].active_bts, *wamData[i].vta);
         }
         active_file[0] = 0;
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;

   case 'N'://Toggle angular hold
      break;

   case 'M'://Move to a location
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter comma seperated destination \".2,.4,...\": ");
         refresh();
         getstr(fn); // Read a user-entered string
         strcat(fn,"\n");
         //syslog(LOG_ERR,"Moveto:%s",fn);
         finish_entry();
         //fill_vn(wamData[0].active_dest,0.25);
         csvto_vn(wamData[0].active_dest, fn); // Convert from string to vector
         
         // If we are performing a Cartesian move
         if(wamData[0].active_bts == &(wam[0]->Csc)){
            // Convert from X, Y, Z, Rx, Ry, Rz to a homogeneous matrix
            setrange_vn((vect_n*)RxRyRz, wamData[0].active_dest, 0, 3, 3); // Extract the rotations
            XYZftoR_m3((matr_3*)cdest, RxRyRz); // Convert from RxRyRz to R[3x3]
            ELEM(cdest, 0, 3) = wamData[0].active_dest->q[0]; // Insert the X position
            ELEM(cdest, 1, 3) = wamData[0].active_dest->q[1]; // Insert the Y position
            ELEM(cdest, 2, 3) = wamData[0].active_dest->q[2]; // Insert the Z position
            set_vn(wamData[0].active_dest, (vect_n*)cdest); // Copy the result back to the command dest
         }
         
         // Set up the move velocity and acceleration
         MoveSetup(wam[0], vel, acc);
         //moveparm_bts(wamData[0].active_bts,vel,acc);
         
         // Start the move
         MoveWAM(wam[0], wamData[0].active_dest);
         // Enter position control mode, if we are not there already
         //if (getmode_bts(wamData[0].active_bts) != SCMODE_POS)
         //   setmode_bts(wamData[0].active_bts, SCMODE_POS);

         // Start the move
         //if(moveto_bts(wamData[0].active_bts, wamData[0].active_dest))
         //   syslog(LOG_ERR,"Moveto Aborted");

      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;
      
      //'m': Move to the presently selected trajectory point

   case '<'://Select next trajectory point
      for(i = 0; i < busCount; i++){
         prev_point_vta(*wamData[i].vta);
      }
      break;
   case '>'://Select previous trajectory point
      for(i = 0; i < busCount; i++){
         next_point_vta(*wamData[i].vta);
      }
      break;
   case '+'://Insert a point in the trajectory
      for(i = 0; i < busCount; i++){
         if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
            ins_point_vta(*wamData[i].vta, wamData[i].active_pos);
         }
      }
      break;
   case '-'://Remove a point in the trajectory
      for(i = 0; i < busCount; i++){
         if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
            del_point_vta(*wamData[i].vta);
         }
      }
      break;
   case 's'://Adjust trj times using a velocity
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter trajectory velocity: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         for(i = 0; i < busCount; i++){
            if(*wamData[i].vta != NULL)
               dist_adjust_vta(*wamData[i].vta,tvel);
         }
         finish_entry();
      }
      break;
   case 'S'://Scale trajectory in time
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter scale factor: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         for(i = 0; i < busCount; i++){
            if(wamData[i].vta != NULL)
               time_scale_vta(*wamData[i].vta,tvel);
         }
         finish_entry();
      }
      break;
   case 'A'://Set the corner acceleration
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Corner Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &tacc);
         for(i = 0; i < busCount; i++){
            if(*wamData[i].vta != NULL)
               set_acc_vta(*wamData[i].vta, tacc);
         }
         finish_entry();
      }
      break;
   case 'a'://Set the move acceleration
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &acc);
         finish_entry();
      }
      break;
   case 'v'://Set the move velocity
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Velocity: ");
         refresh();
         ret = scanw("%lf\n", &vel);
         finish_entry();
      }
      break;
   case ','://Pause/Unpause trajectory
      for(i = 0; i < busCount; i++){
         status =  movestatus_bts(wamData[i].active_bts);
         if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
            unpause_trj_bts(wamData[i].active_bts,2);
         else
            pause_trj_bts(wamData[i].active_bts,2);
      }
      break;
   case 'h'://Toggle Help
      clearScreen();
      screen = !screen;
      break;

    case ' '://Toggle BHand Demo
    demo = !demo;
   case 27://Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR) {
         if (chr == 91) {
            if ((chr = getch()) != ERR) {
               if (chr == 67) //Right arrow
               {
               } else if (chr == 68) //Left arrow
               {
               } else {
                  while(getch()!=ERR) {
                     // Do nothing
                  }
                  syslog(LOG_ERR,"Caught unknown keyhit 27-91-%d",c);
               }
            }
         } else {
            while(getch()!=ERR) {
               // Do nothing
            }
            syslog(LOG_ERR,"Caught unknown keyhit 27-%d",c);
         }
      }
      break;

   default:
      while(getch()!=ERR) {
         // Do nothing
      }
      syslog(LOG_ERR,"Caught unknown keyhit %d",c);

      break;
   }
}
