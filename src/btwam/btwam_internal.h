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

/*======================================================================*
 *  Module .............libbtwam
 *  File ...............btwam.h
 *  Author .............Sam Clanton
 *  Creation Date ......Q3 2004
 *  Addt'l Authors......Traveler Hauptman
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    Dec 17, 2004: TH
 *      Splitting libbt into libbtsystem & libbtwam. Adjust file names
 *      to support this. btwam -> btwamctl, btdriver -> btwam
 *                                                                      *
 *======================================================================*/

#ifndef _BTWAMINTERNAL_H
#define _BTWAMINTERNAL_H

#include <rtai_lxrt.h>
#include "btsystem.h"
#include "btjointcontrol.h"
#include "btmath.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btpath.h"
#include "btlogger.h"

int ActAngle2Mpos(vect_n *Mpos); //Extracts actuators 0 thru 6 into vect_n positions
int Mtrq2ActTrq(vect_n *Mtrq);  //Packs vect_n of torques into actuator array
void Mpos2Jpos(vect_n * Mpos, vect_n * Jpos); //convert motor angle to joint angle
void Jpos2Mpos(vect_n * Jpos, vect_n * Mpos);
void Jtrq2Mtrq(vect_n * Jtrq, vect_n * Mtrq); //conbert joint torque to motor torque

void GetJointPositions();
void SetJointTorques();

int read_wam_vector(FILE *in,vect_n *wv);
int write_wam_vector(FILE *out,vect_n *wv);
int dump_wam_vector(vect_n *wv);

SimpleCtl * GetWAMsc();


int LoadWAM(char *wamfile);
void SaveWAM(char *wamfile);
void DumpWAM2Syslog();


#endif /*_BTWAM_H*/

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
