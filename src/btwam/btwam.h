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

#include "btsystem.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btpath.h"
#include "btlogger.h"
#include "btparser.h"
#include "btmath.h"
#include "btos.h"
#include "btgeometry.h"

#ifndef BTINLINE
#define BTINLINE inline
#endif

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
 
\todo Replace btPID-specific control structures with generic structures
*/
typedef struct btwam_struct
{
   int id; //!< Unused
   char name[256]; //!< Give your robot a name, just for fun. Specified in wam.conf.
   
   //State Variables
   int isZeroed; //!< If TRUE, allow torques from Cartesian control. Defaults to FALSE, set to TRUE after #DefineWAMpos().
   int dof; //!< Robot degrees of freedom. Specified in wam.conf.
   int idle_when_done; //!< !0 = When done with the present move switch to idle mode
   
   //user callback
   int (*force_callback)(struct btwam_struct *wam); //!< A pointer to a function registered with #registerWAMcallback().

   //Actuator info
   actuator_struct *act; //!< Low-level actuator data. You probably do not need to worry about this.
   int num_actuators; //!< Low-level actuator count. Copied from buses[bus].num_pucks
   int *motor_position; //!< Low-level actuator->joint mapping

   //Automatic zero finding
   int *zero_order; //!< Defines the order in which the joints are initialized
   vect_n *zero_offsets; //!< Distance from a stop to consider the zero-position (rad)
   vect_n *stop_torque; //!< How hard the joint pushes into the stop (Nm)
   vect_n *park_location; //!< The default home/rest position (rad)
   vect_n *torq_limit; //!< ???

   //Motor <-> Joint State
   vect_n *Mpos, *Mtrq; //!< Motor positions (rad), motor torques (Nm)
   vect_n *N,*n; //!< Transmission ratios
   matr_mn *M2JP, *J2MP, *J2MT; //!< Transmission matrices: motor->joint position, joint->motor position, and joint->motor torque
   //vect_n *motorInertias;

   //Kinematics & Dynamics
   //double sample_rate;
   btrobot robot; //!< All kinematics and dynamics information for this robot

   //Motion Control
   btstatecontrol *active_sc; //!< A pointer to the active state controller (usually Jsc or Csc)
   double dt;
   
   //Joint space controller
   btstatecontrol Jsc; //!< The state controller for joint space
   btPID_array JposControl; //!< Array of btPID control objects (data structures) used for joint position control
   vect_n *Jpos; //!< Joint position (rad) (read-only)
   vect_n *Jvel; //!< Joint velocity (rad/s) (read-only)
   vect_n *Jacc; //!< Joint acceleration (rad/s/s) (read-only)
   vect_n *Jtrq; //!< Joint torque command (Nm). If you write/use you own torque controller, you can use this.
   vect_n *Jref; //!< Joint position reference/command (rad). If you write/use your own position controller, you can use this.
   vect_n *Jtref; //!< Joint trajectory reference. Tracks progress through a trajectory.
   vect_n *vel, *acc; //!< Velocity and acceleration for joint space moves
   
   //Cartesian space controller
   btstatecontrol Csc; //!< The state controller for Cartesian space
   btPID_array CposControl; //!< Array of btPID control objects (data structures) used for Cartesian position control
   vect_n *Ttrq; //!< Joint torques generated by RNE force/torque + gravity calculations
   vect_3 *Cpos; //!< Cartesian end-point position XYZ (m)
   vect_3 *Cvel; //!< Cartesian end-point velocity (m/s). 
   vect_3 *Cacc; //!< Cartesian end-point acceleration (m/s/s). 
   vect_3 *Cforce; //!< Cartesian force XYZ (N)
   vect_3 *Ctrq; //!< Cartesian torque RxRyRz (Nm)
   vect_3 *Cref; //!< Cartesian position reference XYZ (m)
   vect_3 *Cpoint; //!< XYZ translation (constant) from the tool center point (TCP) to the Cartesian control point (end-point)
   matr_h *HMpos; // Position
   matr_h *HMvel; // Velocity
   matr_h *HMacc; // Acceleration
   matr_h *HMref; // Position reference
   matr_h *HMtref; // Trajectory reference
   matr_h *HMft; // Force/torque matrix
   
   // Quaternian control
   quat *qref,*qact,*qaxis,*forced; //!< Reference and actual orientations for quaternion control
   btreal qerr;
   
   //Cartesian Controllers
   //btPID pid[12]; //  x,y,z,rot
   //vect_n *R6pos,*R6vel,*R6acc,*R6ref,*R6tref,*R6force,*R6trq;
   
   //btPID d_pos_ctl[12];
   //btPID_array d_pos_array;

   //Cartesian space moves
   bttraptrj trj;
   btpath_pwl pth;
   
   vect_n *Gtrq; //!< Joint torques due to gravity compensation
   //btreal F;

   //Loop timing info
   RTIME loop_time; //!< Time in nanoseconds from start to end of control loop processing.
   RTIME loop_period; //!< Sample rate.
   RTIME readpos_time; //!< Total Time to read positions
   RTIME writetrq_time; //!< Total Time to send torques
   RTIME user_time; //!< Time spent in user callback
   RTIME Jsc_time; //!< Time spent in joint control
   RTIME Csc_time; //!< Time spent in Cartesian control
   double skipmax;
   btrt_mutex loop_mutex; //This mutex is set while the wam control loop is in operation. It is to help slow loops access control loop data

   //Data logging
   btlogger log;
   int logdivider;
   btreal log_time;
   
   //Continuous path record
   btlogger cteach;
   int divider,counter;
   btreal teach_time;

   btrt_thread_struct maint; //!< Periodic (default 10Hz) maintenance thread for teach & play. Configured in #OpenWAM().
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
