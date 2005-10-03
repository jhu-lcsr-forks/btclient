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
#include "btcontrol_virt.h"
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
typedef struct
{
  btPID* pid;
  int elements;
}btPID_array;
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

//btposition_interface stuff
void btposition_interface_pause_btPID(struct btposition_interface_struct* btp);
void btposition_interface_reset_btPID(struct btposition_interface_struct* btp);
vect_n* btposition_interface_eval_btPID(struct btposition_interface_struct* btp);
void btposition_interface_mapf_btPID(btstatecontrol *sc, btPID_array *pid);
/*================================================Trajectory stuff================================*/
/** A piecewize linear interpolating trajectory

  ct_traj stores an array of time,position data and 
  plays it back by linear interpolation.

*/
typedef struct 
{
  btpath_pwl *pwl;
  
  int state;
  btreal start_error; //maximum error between present location and starting location
  
}ct_traj;

void create_ct(ct_traj *trj,vectray *vr);
int readfile_ct(ct_traj* ct,char* filename);
vect_n* init_ct(ct_traj *trj);
vect_n* eval_ct(ct_traj *trj, btreal dt);

int done_ct(ct_traj *trj);
int bttrajectory_interface_getstate_ct(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_ct(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_ct(struct bttrajectory_interface_struct *btt);
void bttrajectory_interface_mapf_ct(btstatecontrol *sc,ct_traj *trj);




typedef struct {
  double vel,acc1,acc2,dt_vel,dt_acc1,dt_acc2;
}Seg_int;

/** A piecewise linear trajectory with parabolic blending.


*/
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
  double trj_acc; //acceleration that controls the blending
  
  
  //===========state
  int segment; //0 = acc, 1=vel
  double dt_acc,dt_vel; 
  double t_acc, t_vel, q_acc, q_vel;
  double t0,q0,v0;
  double acc,vel;
  double last_vel;
  double last_cmd; //added for temp debugging
  double last_et;//added for temp debugging
  
  vectray *vr; //Pointer to a trajectory file
  int row; //Present row of the trajectory file
  int rows; //Total number of rows in trajectory file 
     
  
}via_trj;


/*internal*/
void SetAcc_vt(via_trj *trj,double acc);
double eval_via_trj(via_trj *trj,double dt);
double start_via_trj(via_trj *trj,int col);
void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, double v_prev, double v_next, double seg_acc, int end);

/** A piecewise linear trajectory with parabolic blending using vect_n data.

A simple edit api is provided to simplify programming user interaction with this 
data structure.

see via_trj data structure for details. 
btcontrol.c has the API documentation.

This data structure is used for implementing a trajectory object for the
btstatecontrol object. See bttrajectory_interface_struct object for information 
on creating your own.

*/
typedef struct
{
  via_trj* trj;
  int elements;
}via_trj_array;
/*Memory Management API*/
//via_trj_array* malloc_new_vta(int num_columns);

via_trj_array* new_vta(int num_columns,int max_rows);
void destroy_vta(via_trj_array** vt);
vectray* get_vr_vta(via_trj_array* vt);

// Edit API
void next_point_vta(via_trj_array* vt);
void prev_point_vta(via_trj_array* vt);
void first_point_vta(via_trj_array* vt);
void last_point_vta(via_trj_array* vt);
int ins_point_vta(via_trj_array* vt, vect_n *pt);
int del_point_vta(via_trj_array* vt);
int get_current_point_vta(via_trj_array* vt);
int set_current_point_vta(via_trj_array* vt,int idx);


int scale_vta(via_trj_array* vt,double vel,double acc);
void set_acc_vta(via_trj_array* vt,btreal acc);

//File I/O
void write_file_vta(via_trj_array* vt,char *filename);
via_trj_array* read_file_vta(char* filename,int extrapoints);

/* Interface */
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt);
void register_vta(btstatecontrol *sc,via_trj_array *vt);


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
