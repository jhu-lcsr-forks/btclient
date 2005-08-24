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
/**
Trajectory States
 - -1 = Off
 - 0 = Stopped
 - 1 = InPrep: Moving from constraint on position to trajectory start position
 - 2 = Ready: Constraint parameter is at the start value
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
enum trjstate {BTTRAJ_OFF = -1,BTTRAJ_STOPPED = 0,BTTRAJ_INPREP,BTTRAJ_READY,
               BTTRAJ_RUN,BTTRAJ_DONE,BTTRAJ_PAUSING,BTTRAJ_UNPAUSING,BTTRAJ_PAUSED};

enum scstate {SCMODE_IDLE=0, SCMODE_TORQUE, SCMODE_POS, SCMODE_TRJ};
/*================================================Position object================================*/
/** A virtual interface for position control
*/
typedef struct btposition_interface_struct
{
  void *pos_reg;
  
  //vect_n* (*init)(void *dat, vect_n* q, vect_n* qref);
  vect_n* (*eval)(struct btposition_interface_struct* btp);
  void (*reset)(struct btposition_interface_struct* btp);
  void (*pause)(struct btposition_interface_struct* btp);
  //void (*set_ref)(void *dat, vect_n* ref);
 
  vect_n *q,*dq,*ddq; //Buffer for present state
  vect_n *qref; //Buffer for desired state
  double *dt;
  
  vect_n *t; //Buffer for output torque

  pthread_mutex_t mutex;
}btposition_interface;

//setup
void mapdata_btpos(btposition_interface *btp,vect_n* q, vect_n* dq, vect_n* ddq, 
                   vect_n* qref, vect_n* t, double *dt);

/*================================================Trajectory object================================*/
/** bttrajectory sets up virtual functions for running an arbitrary trajectory along
an arbitrary curve. Curve and trajectory initialization are done outside.

we assume that the curve (and trajectory) are capable of maintaining their state
variables

see trjstate for more state info
*/
typedef struct bttrajectory_interface_struct
{
  double *dt; // pointer to location of dt info
  vect_n *q; //results
  
  void *dat; //Trajectory object pointer
  
  vect_n* (*eval)(struct bttrajectory_interface_struct *btt);
  int (*getstate)(struct bttrajectory_interface_struct *btt);
  void (*stop)(struct bttrajectory_interface_struct *btt);


  
 
  pthread_mutex_t mutex;
}bttrajectory_interface;

bttrajectory_interface * new_bttrajectory();
//setup
void setpath_bttrj(bttrajectory_interface *trj,void *crv_dat, void *initfunc, void *evalfunc);
void settraj_bttrj(bttrajectory_interface *trj,void *trj_dat, void *initfunc, void *evalfunc);

//use
vect_n* eval_bttrj(bttrajectory_interface *trj,btreal dt);
int start_bttrj(bttrajectory_interface *trj);

int getstate_bttrj(bttrajectory_interface *trj);
void stop_bttrj(bttrajectory_interface *trj);
/*================================================State Controller object====================*/
/*! \brief A state controller for switching between position control and torque control

This structure stores state information along with pid and trajectory info for
a simple state controller. The primary function of this object is to provide bumpless transfer between position 
control, moving trajectory control, and no control (idle)

If the position control is off, a zero torque is returned.

*/
typedef struct
{
  int mode; //!< 0:idle, 1:torque, 2:pos 3:pos + trj
  vect_n* t; //!< Internal buffer for torques
  vect_n* q,*dq,*ddq,*qref; //!< Internal buffer for position and reference position
  double *dt;
  double last_dt; //history: the last time step used.
  btposition_interface* btp;
  bttrajectory_interface *trj;
  int error; //nonzero if there are any errors

  pthread_mutex_t mutex;
}btstatecontrol;
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq, 
                   vect_n* qref, vect_n* t, double *dt);
int init_bts(btstatecontrol *sc);
int set_bts(btstatecontrol *sc, btposition_interface* pos, bttrajectory_interface *trj);
vect_n* eval_bts(btstatecontrol *sc);
int setmode_bts(btstatecontrol *sc, int mode);


/********  Generic (simple) default implementation for virtual functions ******/
/**

pid_pos_ctl


btstatecontrol* new_default_statecontrol()
{
	malloc(sizeof(btstatecontrol)
	init_bts()
	
	
	
}






*/



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
