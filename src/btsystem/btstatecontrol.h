/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btstatecontrol.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Apr 28, 2005 
 *  
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:                                                              
 *   Virtualized control functions                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  
 *                                                                      
 *======================================================================*/
/*! \file btstatecontrol.h
 
    \brief Virtual interfaces for control functions
    
    Position control is a subset of constraint imposition. With position control 
    of a single joint we attempt to constrain the actual position to match some 
    target position. With virtual joint stops we seek to constrian a joint position
    to remain inside a certain range.
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
#include "btpath.h"
#ifndef PI
#define PI 3.141596
#endif /*PI*/

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
  btmutex mutex;
}btramp;

void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate);
void set_btramp(btramp *r,enum btramp_state state);
void setrate_btramp(btramp *r,btreal rate);
btreal get_btramp(btramp *r);
btreal eval_btramp(btramp *r,btreal dt);
int getstate_btramp(btramp *r);
btreal rate_eval_btramp(btramp *r,btreal dt,btreal rate);


/**
\internal
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



/*
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

  bttraptrj provides configuration and state information for a set of trapezoidal trajectory
  generation functions specified in btstatecontrol.c.
  
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

/*================================================Position object================================*/
/** A virtual interface for position control
*/
typedef struct btposition_interface_struct
{
  void *dat;
  
  //vect_n* (*init)(void *dat, vect_n* q, vect_n* qref);
  vect_n* (*eval)(struct btposition_interface_struct* btp);
  void (*reset)(struct btposition_interface_struct* btp);
  void (*pause)(struct btposition_interface_struct* btp);
  //void (*set_ref)(void *dat, vect_n* ref);
 
  vect_n *q,*dq,*ddq; //Buffer for present state
  vect_n *qref; //Buffer for desired state
  double *dt;
  
  vect_n *t; //Buffer for output torque

  btmutex mutex; //unused?
}btposition_interface;

//setup
void mapdata_btpos(btposition_interface *btp,vect_n* q, vect_n* dq, vect_n* ddq, 
                   vect_n* qref, vect_n* t, double *dt);

/*================================================Trajectory object================================*/
/** bttrajectory sets up virtual functions for running an arbitrary trajectory along
an arbitrary curve. Curve and trajectory initialization are done outside.


see trjstate for more state info
*/
typedef struct bttrajectory_interface_struct
{
  double *dt; // pointer to location of dt info
  vect_n *qref; //results
  
  void *dat; //Trajectory object pointer
  
  vect_n* (*reset)(struct bttrajectory_interface_struct *btt);
  vect_n* (*eval)(struct bttrajectory_interface_struct *btt);
  int (*getstate)(struct bttrajectory_interface_struct *btt);

  // Straight trajectory
  btpath_pwl pth;
  bttraptrj trj;
  int state;
  btmutex mutex;  //unused?
}bttrajectory_interface;
//void init_bttrj(bttrajectory_interface *btt);
void mapdata_bttrj(bttrajectory_interface *btt, vect_n* qref, double *dt);

/*================================================State Controller object====================*/
/*! \brief A state controller for switching between position control and torque control

This structure stores state information along with pid and trajectory info for
a simple state controller. The primary function of this object is to provide bumpless transfer between position 
control, moving trajectory control, and no control (idle)

If the position control is off, a zero torque is returned.

Interlocks:
SCMODE is set to POS or IDLE by user. SCMODE is automatically escalated to TRJ from POS by movement commands





Typical use:
Initialization:
\code
  btstatecontrol Jsc;
  btposition_interface Jbtp;
  vect_n *Mpos,*Mtrq,*Jpos,*Jvel,*Jacc,*Jref,*Jtrq;
  double dt;
  
  //<snip> initialize vect_n objects
  
  init_bts(&Jsc);
  map_btstatecontrol(&Jsc, Jpos, Jvel, Jacc, 
                      Jref, Jtrq, &dt);
  btposition_interface_mapf_btPID(&Jsc, &(d_jpos_array));
\endcode
Use:
\code
  via_trj_array *vta;  //A trajectory object that implements the sc virtual function interface
  vta = read_file_vta("teach.csv"); //create a trajectory object
  register_vta(&Jsc,vta); //wrapper for maptrajectory_bts() see btcontrol.c
  //notice that once we register our specific trajectory object, everything 
  //is done through the generic _bts() functions.
  
  prep_trj_bts(&Jsc,0.5,0.5); //Move the wam to the start position of the trajectory
  while (Jsc.btt.state == BTTRAJ_INPREP){ //wait until we are there
          usleep(100000);
  }
  start_trj_bts(&wam->Jsc); //Start the trajectory we registered
  sleep(10); //run for 10 seconds
  stop_trj_bts(&wam->Jsc); //stop the trajectory
\endcode
Meanwhile in another loop:
\code
eval_bts(&(WAM.Jsc));
\endcode

You can implement a position controller and a trajectory controller. Both can be 
"cold swapped". ie, you can turn off your controller, replace it with another control
object dynamically and then turn it back on.

See the via_trj_array object for an example of implementing an object that can be
plugged in. For a trajectory we implement an eval, reset, and getstate function. 
These are then registered using the maptrajectory_bts() function.
\code
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt);
void register_vta(btstatecontrol *sc,via_trj_array *vt);
void register_vta(btstatecontrol *sc,via_trj_array *vt)
{ 
  maptrajectory_bts(sc,(void*) vt,
                    bttrajectory_interface_reset_vt,
                    bttrajectory_interface_eval_vt,
                    bttrajectory_interface_getstate_vt);
}
\endcode

\todo btstatecontrol trajectory looping is a kludge. We really should handle 
this at the trajectory level


*/

typedef struct
{
  int mode; //!< 0:idle, 1:torque, 2:pos 3:pos + trj
  vect_n* t; //!< Internal buffer for torques
  vect_n* q,*dq,*ddq,*qref; //!< Internal buffer for position and reference position

  double *dt;
  double local_dt; //for time warping...
  double dt_scale;
  btramp ramp;
  double last_dt; //history: the last time step used.
  
  btposition_interface btp;
  bttrajectory_interface btt;
  int error; //nonzero if there are any errors

  // Straight trajectory
  btpath_pwl pth;
  bttraptrj trj;
  btreal vel,acc;
  int prep_only,loop_trj;
  btmutex mutex;
}btstatecontrol;
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq, 
                                           vect_n* qref, vect_n* t, double *dt);
int init_bts(btstatecontrol *sc);
//int set_bts(btstatecontrol *sc, btposition_interface* pos, bttrajectory_interface *trj);
vect_n* eval_bts(btstatecontrol *sc);

int setmode_bts(btstatecontrol *sc, int mode);
int getmode_bts(btstatecontrol *sc);
//int prep_trj_bts(btstatecontrol *sc); Depreciated
int moveto_bts(btstatecontrol *sc,vect_n* dest);
void moveparm_bts(btstatecontrol *sc,btreal vel, btreal acc);
int movestatus_bts(btstatecontrol *sc);
int start_trj_bts(btstatecontrol *sc);
int stop_trj_bts(btstatecontrol *sc);
int pause_trj_bts(btstatecontrol *sc,btreal period);
int unpause_trj_bts(btstatecontrol *sc,btreal period);


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTCONTROL_H */



