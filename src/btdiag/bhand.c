/* ======================================================================== *
 *  Module ............. btdiag
 *  File ............... bhand.c
 *  Creation Date ...... 11 Jan 2010
 *  Author ............. Brian Zenowich
 *  Addtl Authors ......
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2010 Barrett Technology, Inc. <support@barrett.com>
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

/** \file bhand.c
    \brief An interactive demo of bhand capabilities.

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
#include <stdio.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"
#include "btcan.h"
#include "bhand.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define INVALID   (0)
/* Define NONE as (N)egative (ONE), tee hee */
#define NONE   (-1)
#define MAX_INT   (0x7FFF)

#define US        (0x0008)

#define ROLE_OPT_MAG_SER (0x0100)
#define ROLE_OPT_MAG_HALL (0x0200)
#define ROLE_OPT_MAG_ENC (0x0400)
#define ROLE_OPT_STRAIN (0x0800)
#define ROLE_OPT_PPS (0x1000)

/* Command enumeration. Append only. */
enum {
    xCMD_LOAD, xCMD_SAVE, xCMD_RESET, xCMD_DEF, xCMD_GET, xCMD_FIND, xCMD_SET, xCMD_HOME,
    xCMD_KEEP, xCMD_LOOP, xCMD_PASS, xCMD_VERS, xCMD_ERR, xCMD_HI, xCMD_IC, xCMD_IO,
    xCMD_TC, xCMD_TO, xCMD_C, xCMD_M, xCMD_O, xCMD_T, xCMD_HELP, xCMD_END
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


/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define _kbhit() (0)
#define _getch()
#define Error()

/* Define character identification macros */
#define isAlpha(c)   ((c >= 'A') && (c <= 'Z'))
#define isDigit(c)   ((c >= '0') && (c <= '9'))
#define isSpace(c)   ( c == SP )
#define toUpper(c)   ( ((c >= 'a') && (c <= 'z')) ? (c - 'a' + 'A') : (c) )
#define D(x)

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/


struct textKey
{
    char key[8];
    int     idx;
};

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
extern wam_struct *wam[];
extern int demo;
extern int done;

/* Command text. Keyword, enumeration. Add alphabetically. */
const struct textKey cmdTxt[]=
    {
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

const struct textKey propTxtCommon[]=
    {
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

const struct textKey propTxtTater[]=
    {
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
 * PRIVATE Function Prototypes  *
 *==============================*/
void uppercase(char *s);
long parseMotorSelect(COMMAND *c);
inline int parseText(COMMAND *c, const struct textKey *t);
int waitForStop(void);
int parse_atol(char *s, long *v );
int Command(char *cmd);

/*==============================*
 * Functions                    *
 *==============================*/

void uppercase(char *s)
{
    while(*s++ = toUpper(*s)) {}
}

void parseInput(COMMAND *c)
{
    int  err, i;
    long  motorSelect;
    int  command;
    int  p;
    long  value = 0;
    char str[12];

    /* Uppercase the (null terminated) input string */
    uppercase(c->command);

    //syslog(LOG_ERR,"[%s] ", c->command);

    /* {<WS>} {MotorSelect} {<WS>} Command <WS> {Parameter} <WS> {Value} {<WS>} */
    D(commWriteSerial(1,"["));
    D(commWriteSerial(strlen(c->command),c->command));
    D(commWriteSerial(1,"]"));

    c->ptr = c->command;

    /* Skip WhiteSpace */
    while(*c->ptr == ' ') {
        ++c->ptr;
    }

    /* Determine the motor select */
    // 0 1 2 3 4 5 6 7 G S IL (InnerLinks) OL (OuterLinks)
    motorSelect = parseMotorSelect(c);
    if(!motorSelect) { // If no motors specified

        motorSelect = 0x001E; // Specify all: 00011110
    }

    D(commWriteSerial(4," ms="));
    D(commWriteSerial(ltoa((long)motorSelect, str), str));

    /* Skip WhiteSpace */
    while(*c->ptr == ' ') {
        ++c->ptr;
    }

    /* Determine the command */
    // RESET FSET SET FGET GET FLOAD LOAD FSAVE SAVE FDEF DEF
    // HOME IO IC HI VERS ERR O C M T ?

    command = parseText(c, cmdTxt);

    D(commWriteSerial(3," c="));
    D(commWriteSerial(ltoa((long)command, str), str));
    if(command == NONE) // If no command specified
    {
        //  serErr(ERR_NO_CMD);
        return;
    }

    /* Skip WhiteSpace */
    while(*c->ptr == ' ') {
        ++c->ptr;
    }

    /* Determine the property */
    switch(command) {
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
        while(*c->ptr == ' ') {
            ++c->ptr;
        }
    }

    /* Read a value */
    switch(command) {
    case xCMD_SET: // "SET"
        /* Determine the value */
        err = parse_atol(c->ptr, &value);
        break;
    case xCMD_M: // "M"
        /* Determine the value */
        err = parse_atol(c->ptr, &value);
        if(!err) { // If we have a value, just do a "SET M"
            command = xCMD_SET;
            p = xM;
        }
        break;
    }

    syslog(LOG_ERR,"[%s] m=0x%0.2x, c=%d, p=%d, v=%d", c->command, motorSelect, command, p, value);
    /* Call the cmd function, dependent on status */
    //if(property[STAT] == STATUS_READY || p == VERS || p == STAT)
    // (cmd[command].f)(motorSelect, p, (void*)&value);

    for( i = 0; i < 4; i++) {
        motorSelect >>= 1;
        if(motorSelect & 0x01) {
            switch(command) {
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
    switch(command) {
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

    while (1) {
        if(isDigit(*c->ptr)) {
            motorSelect |= 1L << (*c->ptr - '0');
            ++c->ptr;

            continue;
        }
        switch (*c->ptr) {
        case ('S'):
                        // Must NOT be followed by 'E' (for SET)
                        if(*(c->ptr+1) == 'E')
                            return(motorSelect);
            // Must NOT be followed by 'AV' (for SAVE)
            if( (*(c->ptr+1) == 'A') && (*(c->ptr+2) == 'V') )
                return(motorSelect);
            motorSelect |= 0x0010; // Motor 4 = 00010000
            ++c->ptr;
            break;
        case ('G'):
                        // Must NOT be followed by 'E' (for GET)
                        if(*(c->ptr+1) == 'E')
                            return(motorSelect);
            motorSelect |= 0x000E; // 1, 2, 3  = 00001110
            ++c->ptr;
            break;
        case ('I'):
                        // Must be followed by 'L' (for Inner Link)
                        if(*(c->ptr+1) != 'L')
                            return(motorSelect);
            motorSelect |= 0x000E; // 1, 2, 3 = 00001110
            c->ptr += 2;
            break;
        case ('O'): // The letter oh
                        // Must be followed by 'L' (for Outer Link)
                        if(*(c->ptr+1) != 'L')
                            return(motorSelect);
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
    while(*(s+i)>='0' && *(s+i)<='9') {
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


/** waitForStop()
 *  Spins in a loop until it sees no movement from the fingers.
 */
int waitForStop()
{
    double F1, F2, F3, F4;
    double EPS = 0.001;

    do {
        F1 = wam[0]->Jpos->q[wam[0]->dof-4];
        F2 = wam[0]->Jpos->q[wam[0]->dof-3];
        F3 = wam[0]->Jpos->q[wam[0]->dof-2];
        F4 = wam[0]->Jpos->q[wam[0]->dof-1];
        usleep(250000); // Sleep for 1/4 sec
    } while(fabs(F1 - wam[0]->Jpos->q[wam[0]->dof-4]) > EPS ||
            fabs(F2 - wam[0]->Jpos->q[wam[0]->dof-3]) > EPS ||
            fabs(F3 - wam[0]->Jpos->q[wam[0]->dof-2]) > EPS ||
            fabs(F4 - wam[0]->Jpos->q[wam[0]->dof-1]) > EPS );

    return(0);
}

int Command(char *cmd)
{
    while(!demo)
        usleep(250000);

    if(!strcmp("123FSET MCV 200",cmd)) {
        setProperty(0,11,MCV,FALSE,400);
        setProperty(0,12,MCV,FALSE,400);
        setProperty(0,13,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("123FSET MOV 200",cmd)) {
        setProperty(0,11,MOV,FALSE,400);
        setProperty(0,12,MOV,FALSE,400);
        setProperty(0,13,MOV,FALSE,400);
        return 0;
    }
    if(!strcmp("4FSET MCV 100",cmd)) {
        setProperty(0,14,MCV,FALSE,200);
        return 0;
    }
    if(!strcmp("4FSET MOV 100",cmd)) {
        setProperty(0,14,MOV,FALSE,200);
        return 0;
    }
    if(!strcmp("GO",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("SO",cmd)) {
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("123C",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123O",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("SC",cmd)) {
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("GC",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("C",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M",cmd)) {
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SIC",cmd)) {
        setProperty(0,14,CMD,FALSE,14);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 11000",cmd)) {
        setProperty(0,11,E,FALSE,-130000);
        setProperty(0,12,E,FALSE,-130000);
        setProperty(0,13,E,FALSE,-130000);

        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 11500",cmd)) {
        setProperty(0,11,E,FALSE,-135300);
        setProperty(0,12,E,FALSE,-135300);
        setProperty(0,13,E,FALSE,-135300);

        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("4FSET DS 630",cmd)) {
        setProperty(0,14,_DS,FALSE,-7300);
        return 0;
    }
    if(!strcmp("123FSET DP 4000",cmd)) {
        setProperty(0,11,E,FALSE,-47000);
        setProperty(0,12,E,FALSE,-47000);
        setProperty(0,13,E,FALSE,-47000);
        return 0;
    }
    if(!strcmp("SFSET MCV 100",cmd)) {
        setProperty(0,14,MCV,FALSE,200);
        return 0;
    }
    if(!strcmp("1234O",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 9000",cmd)) {
        setProperty(0,13,E,FALSE,-106000);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 9000",cmd)) {
        setProperty(0,12,E,FALSE,-106000);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 9000",cmd)) {
        setProperty(0,11,E,FALSE,-106000);
        setProperty(0,11,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 17000",cmd)) {
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 17000",cmd)) {
        setProperty(0,12,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 17000",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 13000",cmd)) {
        setProperty(0,13,E,FALSE,-153000);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 13000",cmd)) {
        setProperty(0,12,E,FALSE,-153000);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 13000",cmd)) {
        setProperty(0,11,E,FALSE,-153000);
        setProperty(0,11,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3FSET MCV 75",cmd)) {
        setProperty(0,13,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("2FSET MCV 75",cmd)) {
        setProperty(0,12,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("1FSET MCV 75",cmd)) {
        setProperty(0,11,MCV,FALSE,150);
        return 0;
    }
    if(!strcmp("SFSET MCV 50",cmd)) {
        setProperty(0,14,MCV,FALSE,100);
        return 0;
    }
    if(!strcmp("3FSET MCV 65",cmd)) {
        setProperty(0,13,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("2FSET MCV 65",cmd)) {
        setProperty(0,12,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("1FSET MCV 65",cmd)) {
        setProperty(0,11,MCV,FALSE,130);
        return 0;
    }
    if(!strcmp("O",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 11500",cmd)) {
        setProperty(0,13,E,FALSE,-135300);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 14000",cmd)) {
        setProperty(0,12,E,FALSE,-164700);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SM 1000",cmd)) {
        setProperty(0,14,E,FALSE,-11600);
        setProperty(0,14,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 2000",cmd)) {
        setProperty(0,11,E,FALSE,-23500);
        setProperty(0,12,E,FALSE,-23500);
        setProperty(0,13,E,FALSE,-23500);

        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("123M 5000",cmd)) {
        setProperty(0,11,E,FALSE,-58800);
        setProperty(0,12,E,FALSE,-58800);
        setProperty(0,13,E,FALSE,-58800);

        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3FSET MCV 200",cmd)) {
        setProperty(0,13,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("3FSET MCV 85",cmd)) {
        setProperty(0,13,MCV,FALSE,170);
        return 0;
    }
    if(!strcmp("2FSET MCV 200",cmd)) {
        setProperty(0,12,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("1FSET MCV 200",cmd)) {
        setProperty(0,11,MCV,FALSE,400);
        return 0;
    }
    if(!strcmp("2FSET DP 11500",cmd)) {
        setProperty(0,12,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("GM",cmd)) {
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1FSET DP 6000",cmd)) {
        setProperty(0,11,E,FALSE,-70000);
        return 0;
    }
    if(!strcmp("2FSET DP 6000",cmd)) {
        setProperty(0,12,E,FALSE,-70000);
        return 0;
    }
    if(!strcmp("1FSET DP 11500",cmd)) {
        setProperty(0,11,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("3FSET DP 11500",cmd)) {
        setProperty(0,13,E,FALSE,-135300);
        return 0;
    }
    if(!strcmp("3C",cmd)) {
        setProperty(0,13,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("2C",cmd)) {
        setProperty(0,12,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("1C",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("12M 11500",cmd)) {
        setProperty(0,11,E,FALSE,-135300);
        setProperty(0,12,E,FALSE,-135300);
        setProperty(0,11,MODE,FALSE,5);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("SM 1570",cmd)) {
        setProperty(0,14,E,FALSE,-17975);
        setProperty(0,14,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3O",cmd)) {
        setProperty(0,13,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("2O",cmd)) {
        setProperty(0,12,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("1O",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("123SC",cmd)) {
        setProperty(0,11,CMD,FALSE,18);
        setProperty(0,12,CMD,FALSE,18);
        setProperty(0,13,CMD,FALSE,18);
        setProperty(0,14,CMD,FALSE,18);
        waitForStop();
        return 0;
    }
    if(!strcmp("123SO",cmd)) {
        setProperty(0,11,CMD,FALSE,20);
        setProperty(0,12,CMD,FALSE,20);
        setProperty(0,13,CMD,FALSE,20);
        setProperty(0,14,CMD,FALSE,20);
        waitForStop();
        return 0;
    }
    if(!strcmp("2M 15000",cmd)) {
        setProperty(0,12,E,FALSE,-176500);
        setProperty(0,12,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("3M 10000",cmd)) {
        setProperty(0,13,E,FALSE,-117600);
        setProperty(0,13,MODE,FALSE,5);
        waitForStop();
        return 0;
    }
    if(!strcmp("1M 5000",cmd)) {
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
        // Error();
        //if( _kbhit() )
        // ;

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

        for(i = 0; i < 3; i++) {
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

        for(i = 0; i < 5; i++) {
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

void runBatch(char *filename)
{
    FILE    *inFile;
    char    line[1024];
    COMMAND cmd;
    long ms = 0L;

    if((inFile=fopen(filename,"r"))==NULL) {
        return;
    }

    while(1) {
        if(fgets(line, 1024, inFile) == NULL)
            break;
        line[strlen(line)-1] = '\0';  // Overwrite newline with termination
        uppercase(line);
        //syslog(LOG_ERR, "%s", line);
        if(strstr(line, "DELAY")) {
            sscanf(line, "%*s%ld", &ms);
            //syslog(LOG_ERR, "delay = %ld", ms);
            usleep(ms * 1000);
        } else {
            //stripComments(line);    // Strip the comments
            strcpy(cmd.command, line);
            parseInput(&cmd);
            usleep(250000);
        }
    }
    fclose(inFile);
}
