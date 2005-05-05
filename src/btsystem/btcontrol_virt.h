/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcontrol_virt.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Apr 28, 2005 
 *  
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
 *   Virtualized control functions                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  
 *                                                                      
 *======================================================================*/

 /* \file btcontrol_virt.h  
    \brief Virtual interfaces for control functions
    
    
*/ 
#ifndef _BTCONTROL_VIRT_H
#define _BTCONTROL_VIRT_H
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

enum trjstate {BTTRAJ_OFF,BTTRAJ_STOPPED = 0,BTTRAJ_INPREP,BTTRAJ_READY,BTTRAJ_RUN,BTTRAJ_DONE,BTTRAJ_PAUSING,BTTRAJ_UNPAUSING,BTTRAJ_PAUSED};
/**
Trajectory States
 - -1 = Off
 - 0 = Stopped
 - 1 = InPrep
 - 2 = Ready
 - 3 = Running
 - 4 = Done
 - 5 = Pausing
 - 6 = Paused
 - 7 = Unpausing


Trajectory Actions and state changes
 - Engage: [-1,5,4,0]->0 : yref = y; Start the constraint block at the present position
 - DisEngage: [0,4]->-1 : Turn off the constraint block; [2-8]->0 : Nothing
 - EStop: [2-8]->0: Stop trajectory, reset
 - PrepTrj: 0->3: Set up move from present location to trajectory start position
 - 3->4: Done moving to start position. Wait for StartTrj or autostart
 - StartTrj: 4->5: Start running trajectory
 - Pause(t): 5->6->7: Ramp dt down to 0 over t seconds
 - Unpause(t): 7->8->5: Ramp dt up from 0 over t seconds
 - LoadTrj: 1->1,2->2,9->2: Set up trajectory for operation

*/


/*================================================Position object================================*/
/** A virtual interface for position control
*/
typedef struct 
{
  void *pos_reg;
  
  vect_n* (*init)(void *dat, vect_n* q, vect_n* qref);
  vect_n* (*eval)(void *dat, vect_n* q, vect_n* qref, btreal dt);

  btreal time;  //elapsed time
  vect_n *q,*dq; //state
  vect_n *t; //output
  
  int state; //ready,running, paused, done
  int direction; //1 = start at end and run backwards
}btposition;

//setup
void setreg_btpos(btposition *trj,void *pos_dat, void *initfunc, void *evalfunc);

//use
void setqref_btpos(btposition *trj,vect_n* qref);

vect_n* eval_btpos(btposition *trj,vect_n* q,btreal dt);
int start_btpos(btposition *trj); //calling this while running will reset the position controller
int getstate_btpos(btposition *trj);

/*================================================Trajectory object================================*/
/** bttrajectory sets up virtual functions for running an arbitrary trajectory along
an arbitrary curve. Curve and trajectory initialization are done outside.

we assume that the curve (and trajectory) are capable of maintaining their state
variables


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
typedef struct 
{
  void *trj;
  void *crv;
  
  vect_n* (*init_S)(void *dat,btreal s);
  btreal (*init_T)(void *dat,btreal t);
  
  btreal (*S_of_dt)(void *dat,btreal dt);
  vect_n* (*Q_of_ds)(void *dat,btreal ds);
  
  
  btreal t;  //elapsed time
  btreal s1; //arc location on curve 1
  btreal acc_pause,acc_stop; //pause and hard stop decellerations
  vect_n *q; //results
  
  int state; //ready,running, paused, done
  int direction; //1 = start at end and run backwards
}bttrajectory;

//setup
void setpath_bttrj(bttrajectory *trj,void *crv_dat, void *initfunc, void *evalfunc);
void settraj_bttrj(bttrajectory *trj,void *trj_dat, void *initfunc, void *evalfunc);

//use
vect_n* eval_bttrj(bttrajectory *trj,btreal dt);
int start_bttrj(bttrajectory *trj);

int getstate_bttrj(bttrajectory *trj);
void stop_bttrj(bttrajectory *trj);
/*================================================State Controller object====================*/
/*! \brief A state controller for switching between position control and torque control

This structure stores state information along with pid and trajectory info for
a simple state controller. 

If the position control is off, a zero torque is returned.

*/
typedef struct
{
  int mode; //!< 0:idle, 1:torque, 2:pos 3:pos + trj
  vect_n* t;
  vect_n* q,*qref;
  
  double last_dt;
  btposition* pos;
  bttrajectory *trj;
  int error; //nonzero if there are any errors

  pthread_mutex_t mutex;
}btstatecontrol;

int init_bts(btstatecontrol *sc, int size);
int set_bts(btstatecontrol *sc, btposition* pos, bttrajectory *trj);
vect_n* eval_bts(btstatecontrol *sc, vect_n* position, double dt);
int setmode_bts(btstatecontrol *sc, int mode);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTCONTROL_H */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2005 Barrett Technology, Inc.                 *
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
