/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcontrol.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 31, 2005 
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
 *   A simple Torque, PID & Trapezoidal velocity trajectory controller                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  050331 TH - Seeded from SimpleControl code
 *                                                                      
 *======================================================================*/

#ifndef _BTCONTROL_H
#define _BTCONTROL_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

#include <pthread.h>
#include "btmath.h"
#include "btos.h"
#ifndef PI
#define PI 3.141596
#endif /*PI*/

/*================================================PID stuff================================*/

/** PID regulator

btPID maintains configuration and state information for a generic PID controller.

init_btPID() or init_err_btPID() initialize the data structure. init_err_btPID is
used when you will be feeding btPID the error (when a non-euclidian metric is used for 
instance).

btPID maintains it's internal state. Thus you may feed it all information each 
cycle (measured value 'y', reference value 'yref', time step 'dt') or if these 
are typically unchanging you can send them only when they change and call step_btPID().

\code
{
  btPID pid;
  btreal dt,targ_pos,meas_pos,torque;
  
  init_btPID(&pid);
  setgains_btPID(&pid,1000.0,20.0,1.0);
  
  start_btPID(&pid);
  
  dt = .001;
  targ_pos = 10.0;
  while(1){
    meas_pos = Some_get_pos_func();
    torque = eval_btPID(&pid,meas_pos,targ_pos,dt);
    Some_set_torque_func(torque);
  }

\endcode
*/
typedef struct  
{
  btreal Kp; //!< Proportional gain
  btreal Kd; //!< Derivative gain
  btreal Ki; //!< Integral gain

  btreal e;  //!< Error (yref - y)
  btreal de; //!< Derivative of error
  btreal fe;
  btreal se; //!< Sum of error
  
  btreal laste;
  int firsttick;   //!< handles special case de for first tick
  int state; //!< 0=off, 1=on
  int external_error_calc; //!< If this is !=0 we expect that the error calculation is done externally
  btreal dt;        //!< Defaults to 1
  btreal saturation; //!< Defaults to zero (doesn't ever saturate)

  btreal y; //!< Measured control parameter
  btreal yref; //!< Commanded control parameter
  btreal lastresult; //!< Last output of the regulator
 
  btmutex mutex;
}btPID;
//Create-Destroy
void init_btPID(btPID *pid);
void init_err_btPID(btPID *pid); //setup for use with eval_err_btPID
//Data Access
void setgains_btPID(btPID *pid, btreal Kp, btreal Kd, btreal Ki);
void setsaturation_btPID(btPID *pid, btreal saturation);
void getgains_btPID(btPID *pid, btreal *Kp, btreal *Kd, btreal *Ki);
void getsaturation_btPID(btPID *pid, btreal *saturation);
void getinputs_btPID(btPID *pid, btreal *y, btreal *yref, btreal *dt);


//State Control
void reset_btPID(btPID *pid);
void stop_btPID(btPID *pid);
void start_btPID(btPID *pid);

//Synchronous Evaluation
btreal eval_btPID(btPID *pid, btreal y, btreal yref, btreal dt);
btreal eval_err_btPID(btPID *pid, btreal error, btreal dt);

//Asynchronous Evaluation
void setinputs_btPID(btPID *pid, btreal y, btreal yref, btreal dt);
void sety_btPID(btPID *pid, btreal y);
void setyref_btPID(btPID *pid, btreal yref);
void setdt_btPID(btPID *pid, btreal dt);
btreal step_btPID(btPID *pid);
btreal lastresult_btPID(btPID *pid);

/*================================================Trajectory stuff================================*/
/**
State: 
 - 0 = uninitialized
 - 1 = initialized
 - 2 = position control is on and it's setpoint is at the trajectory start
 - 3 = running
 - 4 = pausing. we are decellerating to a stop (this should be done by warping time rather than distance.
 - 5 = paused. we are in the middle of a curve
 - 6 = unpausing. we are accellerating to match the desired trajectory
 - 7 = done
*/
/*! \brief Trajectory generator

  trajectory provides configuration and state information for a set of trapezoidal tragectory
  generation functions specified in trajectory.c.
  
  Make sure you set acc, and vel. These are the constant acceleration used and the 
  maximum velocity allowed respectively.

*/

typedef struct 
{
  //state machine
  int state; //!< 0: done, 1:run
  
  //internal state
  btreal cmd;
  btreal end;
  btreal acc;
  btreal vel;
  btreal t;
  btreal t1,t2; // Calculated inflection points of trapezoidal velocity curve
  btreal x1,x2;

}bttraptrj;

btreal evaluate_traptrj(bttraptrj *traj,btreal dt);
void start_traptrj(bttraptrj *traj, btreal dist);
void setprofile_traptrj(bttraptrj *traj, btreal vel, btreal acc);



typedef struct {
  double vel,acc1,acc2,dt_vel,dt_acc1,dt_acc2;
}Seg_int;

typedef struct 
{
  int state;
  double t;
  int col; //column of point list table
  int idx,n;
  double Qdot,Qdot_next;
  double v_prev,v_next;
  Seg_int seg;
  
  
  //===========parameters
  double trj_acc;
  
  
  //===========state
  int segment; //0 = acc, 1=vel
  double dt_acc,dt_vel; 
  double t_acc, t_vel, q_acc, q_vel;
  double t0,q0,v0;
  double acc,vel;
  double last_vel;
  
  
  vectray *vr; //Pointer to a trajectory file
  int row; //Present row of the trajectory file
  int rows; //Total number of rows in trajectory file 
     
  
}via_trj;


double eval_via_trj(via_trj *trj,double dt);
void start_via_trj(via_trj *trj,double start,int n,int col,double acc);
void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, double v_prev, double v_next, double seg_acc, int end);


/*================================================Ramp object================================*/

enum btramp_state {BTRAMP_MAX = 0, BTRAMP_MIN, BTRAMP_UP, BTRAMP_DOWN, BTRAMP_PAUSE};
/** Constant acceleration function.

btramp is used to smoothly transition a variable from one value to another over 
a period of time. It is thread safe. See init_btramp().

Use set_btramp() to change the state and control the 
btramp object. btramp states are as follows. 

- BTRAMP_MAX = scaler is set to the minimum value  
- BTRAMP_MIN = scaler is set to the min value each evaluation
- BTRAMP_UP = Scaler is increased by rate*dt each evaluation. When scaler >= BTRAMP_MAX, 
the state changes to BTRAMP_MAX
- BTRAMP_DOWN = Scaler is decreased by rate*dt each evaluation. When scaler >= BTRAMP_MIN, 
the state changes to BTRAMP_MIN
- BTRAMP_PAUSE, Default = Scaler is not touched.
*/

typedef struct 
{
  btreal *scaler;
  btreal min,max;
  btreal rate; // dscaler/dt units per second
  int state; 
  pthread_mutex_t mutex;
}btramp;

void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate);
void set_btramp(btramp *r,enum btramp_state state);

btreal get_btramp(btramp *r);
btreal eval_btramp(btramp *r,btreal dt);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTCONTROL_H */

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
