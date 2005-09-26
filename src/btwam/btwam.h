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

#ifndef _BTWAM_H
#define _BTWAM_H

#include <rtai_lxrt.h>
#include "btsystem.h"
#include "btjointcontrol.h"
#include "btmath.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btpath.h"
#include "btlogger.h"
#include "btos.h"


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


/** This structure maintains the present state of the wam

wam_struct is used by the WAM control loop and WAM API to maintain information 
about the state of the wam and to maintain the control and calculation data structures.

One (and only one) instance of this structure is created for each process. It is 
available to users to get information about the wam and to get pointers to the 
control objects.

*/
typedef struct btwam_struct{
  int id;
//State Variables
  int Gcomp; //0 = no gravity comp, 1 = gravity comp
  int wrist_attached; //0 no wrist, 1 yes wrist is attached.
  int use_new;
  int type; //enum {WAM_4DOF, WAM_4DOF_G, WAM_7DOF, Wrist_3DOF} 
  int isZeroed;
  
//user callback
  int (*force_callback)(struct btwam_struct *wam);
  
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
  vect_n *Mpos,*Mtrq,*Jpos,*Jvel,*Jacc,*Jref,*Jtrq;

//Kinematics & Dynamics
  double sample_rate;
  btrobot robot;
  
  
//Motion Control
  
  //Jointspace state controller
  SimpleCtl sc[7];
  btstatecontrol Jsc;
  btposition_interface Jbtp;
  btPID d_jpos_ctl[7];
  btPID_array d_jpos_array;
  //JointSpace Position control
  vect_n *Kp,*Kd,*Ki,*saturation;
  //JointSpace Moves
  bttrajectory_interface Jbtt;
  vect_n *vel,*acc;
  
  //CartesianSpace Position control
  btPID pid[6]; //  x,y,z,quat
  quat *qref,*qact,*qaxis,*forced; //reference and actual orientations for quaternion control
  vect_n *Ttrq;
  vect_3 *Cpos,*Cforce,*Ctrq,*Cref;
  vect_3 *Ckp,*Ckd,*Cki,*Cpoint;
  btreal qerr;
  
  //CartesianSpace Moves
  bttraptrj trj;
  btpath_pwl pth;
  
  //Cartesian Controllers
  vect_n *R6pos,*R6vel,*R6acc,*R6ref,*R6force,*R6trq;
  double dt;
  btstatecontrol Csc;
  btposition_interface Cbtp;
  bttrajectory_interface Ctrj;
  btPID d_pos_ctl[6];
  btPID_array d_pos_array;
  
  btreal F;
  
  //Loop timing info
  RTIME loop_time,loop_period,readpos_time,writetrq_time,user_time;
  pthread_mutex_t loop_mutex; //This mutex is set while the wam control loop is in operation. It is to help slow loops access control loop data
  
  //Data logging
  btlogger log;
  int logdivider;
  btreal log_time;
  //Continuous path record
  btlogger cteach;
  int divider,counter;
  btreal teach_time;
}wam_struct;

/**  Final API **/

btreal getGcomp();
void setGcomp(btreal scale);
int BlankWAMcallback(struct btwam_struct *wam);
void registerWAMcallback(void *func);

/** Old API **/

int MotorID_From_ActIdx(int idx);

wam_struct * GetWAM(void);

int InitWAM(char *wamfile);
void CloseWAM();
void SetWAMpos(vect_n *wv);/**\bug Change to DefineWAMpos()*/

void WAMControlThread(void *data);




void getLagrangian4(double *A, double *B, double *C, double *D);


void MoveWAM(vect_n *pos); /** \bug depreciated: use moveto_bts() */
void MovePropsWAM(vect_n *vel, vect_n *acc);/** \bug depreciated: use moveprops_bts() */
void CartesianMoveWAM(vect_n *pos); /** \bug depreciated: use moveto_bts() */
void CartesianMovePropsWAM(btreal vel, btreal acc); /** \bug depreciated: use moveprops_bts() */


// Continuous Teach & Play Recording
void StartContinuousTeach(int Joint,int Div,char *filename); //joint: 0 = Cartesian, 1 = Joint Space
void StopContinuousTeach(); 

void ServiceContinuousTeach();

ct_traj* LoadContinuousTeach(char* filename); //allocate and return vectray if successful



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
