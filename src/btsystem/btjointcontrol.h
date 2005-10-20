/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............SimpleControl.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Nov 24, 2002 
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation (version 2).
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
 *
 *  NOTES:                                                              
 *   A simple Torque, PID & Trapezoidal velocity trajectory controller                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  021124 - TH - File created                                          
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *                                                                      
 *======================================================================*/
/*! \file btjointcontrol.h
    \brief A simple Torque, PID & Trapezoidal velocity trajectory controller

    btjointcontrol provides a set of basic control algorithms that are often useful
    when controlling robots.
    The use of these functions is straighforward
    -# Create a sturcture of type SimpleCtl
    -# SCinit to get everything in a reasonable state.
    -# SCevaluate inside your read-position set-torque loop.
    -# SCsetmode to switch to the mode you want

    - SCstarttrj to move the PID setpoint along a trapezoidal trajectory

*/

#ifndef _SIMPLECONTROL_H
#define _SIMPLECONTROL_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
/*================================================PID stuff================================*/
#include <pthread.h>
#include "btmath.h"
#include "btstatecontrol.h"

#ifndef PI
#define PI 3.141596
#endif /*PI*/
#define		MAX_FILT_SIZE  5		   /* max size of filter coefficient arrays */
#define		FIRST_ORDER 	1
#define		SECOND_ORDER   2
#define		FOURTH_ORDER   4
/*Moved enum to btstatecontrol.h*/
//enum {SCMODE_IDLE,SCMODE_TORQUE,SCMODE_PID,SCMODE_TRJ,SCMODE_POS};
/** PID regulator

PIDregulator maintains configuration and state information for a generic PID controller.

Usage:
  -# declare a PIDregulator variable (ex: PIDregulator pid;)
  -# Use PIDreset() to set accumulated error to zero and yref to y
  -# In your control loop:
  - Set pid.y to the present value of y;
  - Use PIDcalc() to calculate the result
  
*/
typedef struct  
{
  double Kp; //!< Proportional gain
  double Kd; //!< Derivative gain
  double Ki; //!< Integral gain

  double e;  //!< Error (yref - y)
  double de; //!< Derivative of error
  double fe;
  double se; //!< Sum of error
  
  double laste;
  int firsttick;   //!< handles special case de for first tick

  double dt;        //!< Defaults to 1
  double saturation; //!< Defaults to zero (doesn't ever saturate)

  double y; //!< Measured control parameter
  double yref; //!< Commanded control parameter
  double lastresult; //!< Last output of the regulator
  int filter_de; //!< Should we use a filter on de?
  btfilter *de_filter,*se_filter;
}PIDregulator;

void PIDinit(PIDregulator *pid,double Kp, double Kd, double Ki, double dt);
void PIDmeasured(PIDregulator *pid, double y);
void PIDcommand(PIDregulator *pid, double yref);
void PIDreset(PIDregulator *pid); 
double PIDcalc(PIDregulator *pid);

/*================================================Trajectory stuff================================*/

typedef struct
{
  double x,v,vel,acc;
}trj_leg_struct;

typedef struct
{
  double x,v,t;
}trj_corner_struct;

typedef struct
{
  trj_leg_struct *point;
  int num;
}point_list_struct;


/*  Trajectory generator (this is an aborted attempt at a smooth trajectory generator. Left in in case it's needed later)

  trajectory provides configuration and state information for a set of trapezoidal tragectory
  generation functions specified in trajectory.c.
  
  Make sure you set acc, and vel. These are the constant acceleration used and the 
  maximum velocity allowed respectively.

*/
typedef struct 
{
  int state; //!< 0: done, 1:run, 2: running crippled
  point_list_struct pl;
  int current_point;
  
  int loop; // 0 don't loop, 1 do loop
  int phase; // start, coast, end
  int crippled;
  
  double cmd;
  double vel;
  double extra_t;
  trj_corner_struct c0,c1,c2,c3;

}npoint_traj;



/*! \brief Trajectory generator

  trajectory provides configuration and state information for a set of trapezoidal tragectory
  generation functions specified in trajectory.c.
  
  Make sure you set acc, and vel. These are the constant acceleration used and the 
  maximum velocity allowed respectively.

*/

typedef struct 
{
  int state; //!< 0: done, 1:run
  double cmd;
  double start_point;
  double sign;
  double end_point;
  double end;
  double acc;
  double vel;
  double t;
  double t1,t2; // Calculated inflection points of trapezoidal velocity curve
  double x1,x2;

}trajectory;

double evaluate_trajectory(trajectory *traj,double dt);
void init_trajectory(trajectory *traj, double start, double end);
double calc_traj_time(trajectory *traj, double start, double end, double vel, double acc);
void sync_trajectory(trajectory *traj, double start, double end, double vel, double acc, double t);
double scale_vel(trajectory *traj, double start, double end, double vel, double acc, double t);
double scale_acc(trajectory *traj, double start, double end, double vel, double acc, double t);
double linterp(double x1, double y1, double x2, double y2, double x);

/*================================================Controller stuff================================*/
/*! \brief Simple torque, pid, and trapezoidal trajectory controller

This structure stores state information along with pid and trajectory info for
a simple motor controller. The functions that use this structure are defined in
SimpleControl.c.

*/
typedef struct
{
  int mode; //!< 0:idle, 1:torque, 2:pid 3:vel 4:trj
  double requested_torque;
  double command_torque;
  double kill_torque;
  double last_dt;
  double position;
  PIDregulator pid;
  trajectory trj;
  int error; //nonzero if there are any errors
  double ddebug;
  int idebug;
  btmutex mutex;
}SimpleCtl;

int SCinit(SimpleCtl *sc);
double SCevaluate(SimpleCtl *sc, double position, double dt);
int SCsetmode(SimpleCtl *sc, int mode);
int SCsettorque(SimpleCtl *sc,double torque);
int SCsetpid(SimpleCtl *sc,double kp,double kd,double ki, double saturation);//!!implement saturation variable
int SCsetcmd(SimpleCtl *sc,double cmd);
int SCsettrjprof(SimpleCtl *sc,double vel,double acc);
int SCstarttrj(SimpleCtl *sc,double end);
int SCfindLimit(SimpleCtl *sc,double offset); //!!trajectory to opposite stop until kill_trq, then to this stop

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _SIMPLECONTROL_H */



