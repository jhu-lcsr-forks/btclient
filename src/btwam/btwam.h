/*======================================================================*
 *  Module .............libbtwam
 *  File ...............btwam.h
 *  Author .............Sam Clanton
 *  Creation Date ......Q3 2004
 *  Addt'l Authors......Traveler Hauptman
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    Dec 17, 2004: TH
 *      Splitting libbt into libbtsystem & libbtwam. Adjust file names
 *      to support this. btwam -> btwamctl, btdriver -> btwam
 *                                                                      *
 *======================================================================*/
/** \file btwam.h
   \brief Provides functions for controlling a WAM.
 
    btwam assumes that you have a 7 DOF or 4 DOF wam that is being controlled
on a CAN bus. It provides some high-level functions to help people get up and
running quickly.

See #btwam_struct


*/
#ifndef _BTWAM_H
#define _BTWAM_H
//#include <rtai_lxrt.h>
//#include <native/task.h>
//#include <native/timer.h>
#include "btsystem.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btjointcontrol.h"
#include "btpath.h"
#include "btlogger.h"
#include "btparser.h"
#include "btmath.h"
#include "btos.h"

#define VN_SIZE (7)

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
control objects. This structure is not protected with mutexes so you should not 
write to it after starting the control loop thread. 

It may be read from at any time but keep in mind that some of the data structures
may be in mid-update.

  \code
  
  btthread wam_thd;
  wam_struct *wam;
  
  err = ReadSystemFromConfig("wam.conf"); //Prep btsystem
  wam = OpenWAM("wam.conf");
  if(!wam)
  {
    exit(1);
  }
  wam_thd.period = 0.002;
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)&wam_thd);
  while(!done){
    //If this was your program you'd be done now!
  }
  btthread_stop(&wam_thd); //Kill WAMControlThread
    
  \endcode

\todo Remove unused sub-structures.
\todo Document items of interest.
*/
typedef struct btwam_struct{
  int id;
  char name[256];
//State Variables
  int Gcomp; // 0 = no gravity comp, 1 = gravity comp
  int wrist_attached; //0 no wrist, 1 yes wrist is attached.
  int use_new;
  int type; //enum {WAM_4DOF, WAM_4DOF_G, WAM_7DOF, Wrist_3DOF} 
/** \todo Query safety board to see if we already have a valid home position*/
  int isZeroed;   
  int space; // 0 = Joint; 1 = Cartesian;
  int dof;
  
//user callback
  int (*force_callback)(struct btwam_struct *wam);
  
//Actuator info
  actuator_struct *act;
  int num_actuators;
  int *motor_position;

//Automatic zero finding
  int *zero_order;
  vect_n *zero_offsets;
  vect_n *stop_torque;
  vect_n *park_location;  
  vect_n *torq_limit;  

//Motor <-> Joint State
  vect_n *Mpos,*Mtrq,*Jpos,*Jvel,*Jacc,*Jref,*Jtref,*Jtrq;
  vect_n *N,*n;
  matr_mn *M2JP,*J2MT,*J2MP;
  vect_n *motorInertias;
//Kinematics & Dynamics
  double sample_rate;
  btrobot robot;
  
  
//Motion Control
  
  //Jointspace state controller
  
  btstatecontrol Jsc;
  
  btPID *d_jpos_ctl;
  btPID_array d_jpos_array;
  //JointSpace Position control
  vect_n *Kp,*Kd,*Ki,*saturation;
  //JointSpace Moves
  
  vect_n *vel,*acc;
  
  //CartesianSpace Position control
  SimpleCtl *sc;
  btPID pid[12]; //  x,y,z,rot
  quat *qref,*qact,*qaxis,*forced; //reference and actual orientations for quaternion control
  vect_n *Ttrq;
  vect_3 *Cpos,*Cforce,*Ctrq,*Cref;
  vect_3 *Ckp,*Ckd,*Cki,*Cpoint;
  vect_n *Cvel;
  vect_n *Gtrq;
  btreal qerr;
  
  //CartesianSpace Moves
  bttraptrj trj;
  btpath_pwl pth;
  
  //Cartesian Controllers
  vect_n *R6pos,*R6vel,*R6acc,*R6ref,*R6tref,*R6force,*R6trq;
  double dt;
  btstatecontrol Csc;

  btPID d_pos_ctl[12];
  btPID_array d_pos_array;
  
  btreal F;
  
  //Loop timing info
  RTIME loop_time; //!< Time in nanoseconds from start to end of control loop processing.
  RTIME loop_period; //!< Sample rate.
  RTIME readpos_time; //!< Total Time to read positions
  RTIME writetrq_time; //!< Total Time to send torques
  RTIME user_time; //!< Time spent in user callback
  RTIME Jsc_time; //!< Time spent in joint control
  RTIME Csc_time; //!< Time spent in Cartesian control
  double skipmax;
  RT_MUTEX loop_mutex; //This mutex is set while the wam control loop is in operation. It is to help slow loops access control loop data
  
  //Data logging
  btlogger log;
  int logdivider;
  btreal log_time;
  //Continuous path record
  btlogger cteach;
  int divider,counter;
  btreal teach_time;
  
  //High level
  int js_or_cs; //!< 0 = Joint space, 1 = Cartesian space
  btstatecontrol *active_sc;
  int idle_when_done; //!< !0 = When done with the present move switch to idle mode
  
  btthread maint;
  vect_n *G; //!< Torques due to gravity
  int M4_reversed;

}wam_struct;



/*************  WAM  API  ******************/

wam_struct* OpenWAM(char *wamfile, int bus); //NULL -> wam.conf
void CloseWAM(wam_struct* wam); //Cleanupint BlankWAMcallback(struct btwam_struct *wam);

void registerWAMcallback(wam_struct* wam,void *func);
void WAMControlThread(void *data); //data points to wam_struct* wam
void WAMControlThread1(void *data); //data points to wam_struct* wam
void WAMControlThread2(void *data); //data points to wam_struct* wam

void DefineWAMpos(wam_struct *w,vect_n *wv);

btreal GetGravityComp(wam_struct *w);
void SetGravityComp(wam_struct *w,btreal scale);

void SetCartesianSpace(wam_struct* wam);
void SetJointSpace(wam_struct* wam);
void SetPositionConstraint(wam_struct* wam, int onoff);

void MoveSetup(wam_struct* wam,btreal vel,btreal acc);
void MoveWAM(wam_struct* wam,vect_n * dest);
int MoveIsDone(wam_struct* wam);
void MoveStop(wam_struct* wam);

void ParkWAM(wam_struct* wam);
// Continuous Teach & Play Recording
void StartContinuousTeach(wam_struct* wam,int Joint,int Div,char *filename); //joint: 0 = Cartesian, 1 = Joint Space
void StopContinuousTeach(wam_struct* wam); 
void ServiceContinuousTeach(wam_struct* wam);

void setSafetyLimits(int bus, double jointVel, double tipVel, double elbowVel);

/******************************************/
/** \internal Below this line all functions need work 

*/



int AddEndpointForce(wam_struct* wam,vect_n *force); //Cartesian only

//MovePause(wam_struct* wam);
//MoveContinue(wam_struct* wam);



long GetTime(); //Wrapper for rt_get_cpu_time_ns()
long GetElapsedTime(); //Static time variable


/******************************************/


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
