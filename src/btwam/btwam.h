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

#ifndef _BTWAM_H
#define _BTWAM_H


#include "btsystem.h"
#include "SimpleControl.h"
#include "btmath.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btpath.h"


#define WAM2004

#ifndef BTINLINE
#define BTINLINE inline
#endif
//joint ratios
#ifndef WAM2004
#define mN1  35.87
#define mN2  28.21
#define mN3  28.21
#define mN4  17.77
#define mn3  1.68
#endif

#ifdef WAM2004
#define mN1  42.0
#define mN2  28.25
#define mN3  28.25
#define mN4  18.0
#define mn3  1.68
#endif

#define mN5  10.27
#define mN6  10.27
#define mN7  14.93
#define mn6  1


typedef struct {
  double q[10];
} wam_vector;
/* Define WAM types for the whereAmI() routine.
    4DOF    = 4-DOF WAM with standard outer link
    4DOF_G  = 4-DOF WAM with gimbals on outer link
    7DOF    = 7-DOF WAM
*/
enum {WAM_4DOF, WAM_4DOF_G, WAM_7DOF} wamType;
/**

*/
typedef struct {
  int id;
//State Variables
  int Gcomp; //0 = no gravity comp, 1 = gravity comp
  int wrist_attached; //0 no wrist, 1 yes wrist is attached.
  int use_new;
  int type; //enum {WAM_4DOF, WAM_4DOF_G, WAM_7DOF, Wrist_3DOF} 
  
//Actuator info
  actuator_struct *act;
  int num_actuators;
  int motor_position[7];

//Automatic zero finding
  int zero_order[7];
  vect_n *zero_offsets;
  vect_n *stop_torque;
  vect_n * park_location;  

//Motor <-> Joint State
  vect_n *Mpos,*Mtrq,*Jpos,*Jtrq;

//Kinematics & Dynamics
  double sample_rate;
  btrobot robot;
  
//Motion Control
  //Jointspace state controller
  SimpleCtl sc[7];
  
  //JointSpace Position control
  vect_n *Kp,*Kd,*Ki,*saturation;
  //JointSpace Moves
  vect_n *vel,*acc;
  
  //CartesianSpace Position control
  btPID pid[4]; //  x,y,z,quat
  quat qref,qact; //reference and actual orientations for quaternion control
  vect_n *Ttrq;
  vect_3 *Cpos,*Cforce,*Ctrq,*Cref;
  vect_3 *Ckp,*Ckd,*Cki,*Cpoint;
  
  //CartesianSpace Moves
  bttraptrj trj;
  btpath_pwl pth;
  
  btreal F;
}wam_struct;




int ActAngle2Mpos(vect_n *Mpos); //Extracts actuators 0 thru 6 into vect_n positions
int Mtrq2ActTrq(vect_n *Mtrq);  //Packs vect_n of torques into actuator array
void Mpos2Jpos(vect_n * Mpos, vect_n * Jpos); //convert motor angle to joint angle
void Jpos2Mpos(vect_n * Jpos, vect_n * Mpos);
void Jtrq2Mtrq(vect_n * Jtrq, vect_n * Mtrq); //conbert joint torque to motor torque

void GetJointPositions();
void SetJointTorques();

int read_vect_n(FILE *in,vect_n *wv);
int write_vect_n(FILE *out,vect_n *wv);
int dump_vect_n(vect_n *wv);

void getWAMjoints(vect_n *Mpos,vect_n *Mtrq,vect_n *Jpos,vect_n *Jtrq);
void getWAMmotor_position(int *mp);
int MotorID_From_ActIdx(int idx);

wam_struct * GetWAM(void);

int playViaTrajectoryFile(char *fileName, double timeScale);

int InitWAM(wam_struct *wamDriverDat, char *wamfile);
void CloseWAM();
SimpleCtl * GetWAMsc();
int LoadWAM(char *wamfile);
void SaveWAM(char *wamfile);
void DumpWAM2Syslog();
void WAMControlThread(void *data);

void SetWAMpos(vect_n *wv);
void MoveWAM(vect_n *pos);
void MovePropsWAM(vect_n *vel, vect_n *acc);
void CartesianMoveWAM(vect_n *pos, btreal vel, btreal acc); 
void ParkWAM();






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
