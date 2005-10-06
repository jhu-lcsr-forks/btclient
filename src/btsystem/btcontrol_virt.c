/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcontrol_virt.c
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

/*! \file btcontrol_virt.c
 
    \brief Virtual interfaces for control functions
    
    Position control is a subset of constraint imposition. With position control 
    of a single joint we attempt to constrain the actual position to match some 
    target position. With virtual joint stops we seek to constrian a joint position
    to remain inside a certain range.
    
*/
#include <math.h>

#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include "btcontrol.h"
#include "btos.h"
#include "btmath.h"
#include "btcontrol_virt.h"
#include "btjointcontrol.h"
/************************** universal trajectory functions *****************/

void setprofile_traptrj(bttraptrj *traj, btreal vel, btreal acc)
{
  traj->vel = fabs(vel);
  traj->acc = fabs(acc);
}

/** Calculates all the variables necessary to set up a trapezoidal trajectory.
 
\param dist the length of the trajectory
 
*/
void start_traptrj(bttraptrj *traj, btreal dist) //assumes that velocity and acceleration have been set to reasonable values
{
  double ax; //acceleration x

  traj->cmd = 0;

  traj->t1 = (traj->vel / traj->acc); //Calculate the "ramp-up" time

  //Get the point at which acceleration stops
  ax = 0.5*traj->acc*traj->t1*traj->t1; //x = 1/2 a t^2

  traj->end = dist;
  if (ax > dist/2) //If the acceleration completion point is beyond the halfway point
    ax = dist/2; //Stop accelerating at the halfway point
  traj->x1 = ax;   //Set the top left point of the trapezoid
  traj->x2 = dist - ax; //Set the top right point of the trapezoid
  traj->t2 = (dist-(2*ax)) / traj->vel + traj->t1; //Find the time to start the "ramp-down"
  traj->t = 0;
  traj->state = BTTRAJ_RUN;
  if (dist == 0.0)
    traj->state = BTTRAJ_STOPPED;
}


/*! Calculate next position on the trajectory
 
evaluate_trajectory generates points on a trapezoidal velocity trajectory. The points accerate with
constant acceleration specified by acc up to a maximum velocity specified by vel. The max velocity
is maintained until approximately the time it will take to decellerate at acc to a velocity of zero.
 
\param *traj pointer to the trajectory structure
\param dt the time step to take (in the same units as vel and acc)
 
\return The value returned is the next point on the trajectory after a step of dt.
 
 
*/
btreal evaluate_traptrj(bttraptrj *traj, btreal dt)
{
  btreal remaining_time,newtime,result;

  if (traj->state == BTTRAJ_RUN)
  {
    traj->t += dt;
    if (traj->cmd < traj->x1) //If we are in "accel" stage
      traj->cmd = 0.5 * traj->acc * traj->t * traj->t;  //x = 1/2 a t^2
    else if (traj->cmd < traj->x2) //If we are in "cruise" stage
      traj->cmd = traj->x1 + traj->vel * (traj->t - traj->t1); //x = x + v t
    else //We are in "decel" stage
    {
      /* time to hit zero = sqrt( (target position - current position)*2/a)
                           new position = 1/2 acc * (time to hit zero - dt)^2 */
      remaining_time = sqrt((traj->end - traj->cmd)*2/traj->acc);
      if (dt > remaining_time)
      {
        traj->cmd = traj->end;
        traj->state = BTTRAJ_STOPPED;
      }

      newtime = (remaining_time-dt);
      if(newtime <= 0)
      {
        traj->cmd = traj->end;
        traj->state = BTTRAJ_STOPPED;
      }
      traj->cmd =  traj->end - 0.5 * traj->acc * newtime * newtime;
    }

  }
  result = traj->cmd;

  if (isnan(result))
  {
    syslog(LOG_ERR, "nan in eval_traj");
    traj->cmd = traj->end;
    traj->state = BTTRAJ_STOPPED;
    return traj->end;
  }
  return result;
}



/**************************** state controller functions ************************/
/** Register data input and output variables with virtual trajectory object */
void mapdata_bttrj(bttrajectory_interface *btt, vect_n* qref, double *dt)
{
  btt->qref = qref;
  btt->dt = dt;
}

/** Register data input and output variables with virtual position control object */
void mapdata_btpos(btposition_interface *btp,vect_n* q, vect_n* dq, vect_n* ddq,
                   vect_n* qref, vect_n* t, double *dt)
{
  btp->q = q;
  btp->qref = qref;
  btp->dq = dq;
  btp->ddq = ddq;
  btp->dt = dt;
  btp->t = t;
}


/** Register data input and output pointers for the btstatecontrol object.
Allocates a piecewise linear path for moving to start points. */
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq,
                        vect_n* qref, vect_n* t, double *dt)
{
  sc->q = q;
  sc->qref = qref;
  init_pwl(&(sc->pth),len_vn(qref),2);
  sc->dq = dq;
  sc->ddq = ddq;
  sc->dt = dt;
  sc->t = t;


  //threm
  //init_pwl(&(sc->btt.pth),len_vn(qref),2);

}
/** Register virtual functions and data for a trajectory with btstatecontrol */
void maptrajectory_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* getstate)
{
  btmutex_lock(&(sc->mutex));

  if (sc->btt.state == BTTRAJ_STOPPED)
  {
    sc->btt.reset = reset;
    sc->btt.eval = eval;
    sc->btt.getstate = getstate;
    sc->btt.dat = dat;
    mapdata_bttrj(&(sc->btt),sc->qref,sc->dt);
    set_vn(sc->qref,sc->q); //Initialize trajectory output to a sane value
    sc->btt.state = BTTRAJ_STOPPED;
  }
  btmutex_unlock(&(sc->mutex));
}
/** Register virtual functions and data for a position control object with btstatecontrol*/
void mapposition_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* pause)
{
  btmutex_lock(&(sc->mutex));
  if (sc->mode != SCMODE_POS && sc->mode != SCMODE_TRJ)
  {
    sc->btp.reset = reset;
    sc->btp.eval = eval;
    sc->btp.pause = pause;
    sc->btp.dat = dat;
    mapdata_btpos(&(sc->btp),sc->q,sc->dq,sc->ddq,sc->qref,sc->t,sc->dt);
    //put everything into a known state
    //set_vn(sc->btp.qref,sc->btp.q);
    //(*(sc->btp.reset))(&sc->btp);
    sc->mode = SCMODE_IDLE;
    sc->btt.state = BTTRAJ_STOPPED;
  }
  btmutex_unlock(&(sc->mutex));
}

/*! Initialize the state controller structure
 
  Loads a preallocated bts object with initial values.
*/
int init_bts(btstatecontrol *sc)
{
  
  int err;
  BTPTR_OK(sc,"moveparm_bts")
  sc->mode = 0;
  sc->last_dt = 1;
  sc->btt.dat = NULL;
  sc->btp.dat = NULL;
  sc->prep_only = 0;
  sc->vel = 0.25; //Safe value for units of radians and meters
  sc->acc = 0.25;
  sc->local_dt = 0.0;
  sc->dt_scale = 1.0;
  init_btramp(&(sc->ramp),&(sc->dt_scale),0.0,1.0,2);
  set_btramp(&(sc->ramp),BTRAMP_MAX);
  
  btmutex_init(&(sc->mutex));
}

inline vect_n* eval_trj_bts(btstatecontrol *sc)
{ 
  int state;
  BTPTR_OK(sc,"moveparm_bts")
  
  state = sc->btt.state;
  if (state == BTTRAJ_INPREP)
  {
    if (sc->trj.state == BTTRAJ_STOPPED)
    {
      if (sc->prep_only)
        sc->btt.state = BTTRAJ_DONE;
      else
        sc->btt.state = BTTRAJ_READY;
    }
    set_vn(sc->qref, getval_pwl(&(sc->pth),evaluate_traptrj(&(sc->trj),*(sc->dt))));
  }

  else if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED){
    if (state == BTTRAJ_PAUSING && getstate_btramp(&(sc->ramp)) == BTRAMP_MIN) sc->btt.state = BTTRAJ_PAUSED;
    else if (state == BTTRAJ_UNPAUSING && getstate_btramp(&(sc->ramp)) == BTRAMP_MAX) sc->btt.state = BTTRAJ_RUN;
    set_vn(sc->btt.qref, (*(sc->btt.eval))(&(sc->btt)));//evaluate path
  }
  
  return sc->btt.qref;
}

/*! Evaluate the state Controller
 
  premise: When switching from torque control to position control, our current
  positiont is our reference point. This can only be moved by a trajectory.
 
  Changing the state of the position and trajectory controllers is handled by those objects
 
\param position the measured position
\param dt the time step we want to evaluate with.
\return Returns a torque command.
*/
vect_n* eval_bts(btstatecontrol *sc)
{
  double newcommand;
  double newtorque;
  double cmptorque;
  int err;
  BTPTR_OK(sc,"moveparm_bts")

  btmutex_lock(&(sc->mutex));
  
  sc->last_dt = *(sc->dt);

  switch (sc->mode)
  {
  case SCMODE_IDLE://Idle
    fill_vn(sc->t,0.0);
    break;
  case SCMODE_TRJ://PID
    eval_btramp(&(sc->ramp),*(sc->dt));
    sc->local_dt = *(sc->dt) * sc->dt_scale;
    eval_trj_bts(sc);
  case SCMODE_POS://PID
    set_vn(sc->t,(*(sc->btp.eval))(&sc->btp));
    break;
  default:
    sc->error = 1;
    fill_vn(sc->t,0.0);
    break;
  }
  btmutex_unlock(&(sc->mutex));
  return sc->t;
}


int getmode_bts(btstatecontrol *sc){
  return sc->mode;
}
/*! \brief Set the state controller mode
 
  Sets the controller mode. To keep anything from "jumping" unexpectedly, we reset our
  pid controller whenever we switch into pidmode.
  
  We expect that the position state variable q is being continuously updated by 
  calling eval_bts() in a control loop.
 
*/
int setmode_bts(btstatecontrol *sc, int mode)
{
  int err;
  double tmp;
  BTPTR_OK(sc,"moveparm_bts")

  btmutex_lock(&(sc->mutex));

  switch (mode)
  {
  case SCMODE_POS:
    if (sc->btp.dat == NULL)
      sc->mode = SCMODE_IDLE;
    else
    {
      if (sc->btt.state != BTTRAJ_STOPPED)
        sc->btt.state = BTTRAJ_STOPPED;

      set_vn(sc->btp.qref,sc->btp.q);
      (*(sc->btp.reset))(&sc->btp);
      sc->mode = SCMODE_POS;
    }
    break;
  case SCMODE_TRJ:
    if (sc->btp.dat == NULL )
      sc->mode = SCMODE_IDLE;
    else if (sc->btt.dat != NULL)
    {
      if (sc->btt.state != BTTRAJ_STOPPED)
        sc->btt.state = BTTRAJ_STOPPED;

      set_vn(sc->btp.qref,sc->btp.q);
      (*(sc->btp.reset))(&sc->btp);
      sc->mode = SCMODE_TRJ;
    }
    break;
  default:
    sc->mode = SCMODE_IDLE;
    break;
  }
  btmutex_unlock(&(sc->mutex));
  return 0;
}
/** Move the wam from its present position to the starting position of the
loaded trajectory. 

\return 0 = success
       -1 = Trajectory not in a stopped state. (Prerequisite)
       -2 = Statecontroller not in TRJ state. (prerequisite)
*/
int prep_trj_bts(btstatecontrol *sc)
{
  char vect_buf1[200];
  int ret;
  BTPTR_OK(sc,"moveparm_bts")
  
  if(sc->mode != SCMODE_TRJ) return -2;
  
  btmutex_lock(&(sc->mutex));
  
  if(sc->btt.state == BTTRAJ_STOPPED)
  {
    clear_pwl(&(sc->pth));
    add_arclen_point_pwl(&(sc->pth),sc->q);
    syslog_vn("prep start:",sc->q);
    add_arclen_point_pwl(&(sc->pth),(*(sc->btt.reset))(&(sc->btt)));
    syslog_vn("prep end:",idx_vr(sc->pth.vr,1));
    
    setprofile_traptrj(&(sc->trj), sc->vel, sc->acc);
    syslog(LOG_ERR,"prep Acc: %f Vel:%f",sc->vel, sc->acc);
    start_traptrj(&(sc->trj), arclength_pwl(&(sc->pth)));
    syslog(LOG_ERR,"prep Arclen:%f",arclength_pwl(&(sc->pth)));
    sc->btt.state = BTTRAJ_INPREP;
    sc->prep_only = 0;
    ret = 0;
  }
  else{
    ret = -1;
  }
  btmutex_unlock(&(sc->mutex));

  return ret;
  //return prep_bttrj(&sc->btt,sc->q,vel,acc);
}
/** Move the wam along the loaded trajectory. */
int moveto_bts(btstatecontrol *sc,vect_n* dest)
{
  char vect_buf1[200];
  int ret;
  BTPTR_OK(sc,"moveparm_bts")
  
  btmutex_lock(&(sc->mutex));
  
  if(sc->btt.state == BTTRAJ_STOPPED)
  {
    clear_pwl(&(sc->pth));
    add_arclen_point_pwl(&(sc->pth),sc->q);
    add_arclen_point_pwl(&(sc->pth),dest);

    setprofile_traptrj(&(sc->trj), sc->vel, sc->acc);
    start_traptrj(&(sc->trj), arclength_pwl(&(sc->pth)));
    sc->btt.state = BTTRAJ_INPREP;
    sc->prep_only = 1;
    ret = 0;
  }
  else{
    ret = -1;
  }  
  btmutex_unlock(&(sc->mutex));
  return ret;

}
/** Set the acceleration and velocity with which to move with
during the initial prep move */
void moveparm_bts(btstatecontrol *sc,btreal vel, btreal acc)
{
  BTPTR_OK(sc,"moveparm_bts")
  btmutex_lock(&(sc->mutex));
  
  sc->vel = vel; //Safe value for units of radians and meters
  sc->acc = acc;
  btmutex_unlock(&(sc->mutex));

}
/** Return the state of the present trajectory generator */
int movestatus_bts(btstatecontrol *sc)
{
  return sc->btt.state;
}
/** Start the trajectory generator. You must have first moved to
the start of the trajectory with prep_trj_bts() */
int start_trj_bts(btstatecontrol *sc)
{
  int ret = 0;
  BTPTR_OK(sc,"moveparm_bts")
  
  btmutex_lock(&(sc->mutex));
  
  if(sc->btt.state == BTTRAJ_READY)
  {
    sc->btt.state = BTTRAJ_RUN;
    ret = -1;
  }
  btmutex_unlock(&(sc->mutex));

  return ret;
}
/** Stop the trajectory generator */
int stop_trj_bts(btstatecontrol *sc)
{
  BTPTR_OK(sc,"moveparm_bts")
  
  btmutex_lock(&(sc->mutex));
  
  sc->btt.state = BTTRAJ_STOPPED;
  btmutex_unlock(&(sc->mutex));
  return 0;
}
int pause_trj_bts(btstatecontrol *sc,btreal period)
{
  int state;
  BTPTR_OK(sc,"moveparm_bts")
  state = sc->btt.state;
  if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED){
  setrate_btramp(&(sc->ramp),period);
  set_btramp(&(sc->ramp),BTRAMP_DOWN);
    
  
  btmutex_lock(&(sc->mutex));
  
  sc->btt.state = BTTRAJ_PAUSING;
  btmutex_unlock(&(sc->mutex));
  }
}
int unpause_trj_bts(btstatecontrol *sc,btreal period)
{
  int state;
  BTPTR_OK(sc,"moveparm_bts")
  state = sc->btt.state;
  if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED){
  setrate_btramp(&(sc->ramp),period);
  set_btramp(&(sc->ramp),BTRAMP_UP);
    
  btmutex_lock(&(sc->mutex));
  
  sc->btt.state = BTTRAJ_UNPAUSING;
  btmutex_unlock(&(sc->mutex));
  }
}

/**************************** state controller functions ************************/

/**************************** btramp functions **********************************/
/** Initialize the data for a btramp object.
 
This should be called only once as it allocates memory for a mutex object.
*/
void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate)
{
  btmutex_init(&(r->mutex));
  r->scaler = var;
  r->min = min;
  r->max = max;
  r->rate = rate;
}
/** Set the state of a btramp object.
 
See btramp for valid values of state.
*/
void set_btramp(btramp *r,enum btramp_state state)
{
  btmutex_lock(&(r->mutex));
  
  r->state = state;
  btmutex_unlock(&(r->mutex));
}
/** Return the present value of a btramp scaler.
 
*/
btreal get_btramp(btramp *r)
{
  return *(r->scaler);
}
/** Return the present state of a btramp.
   see btramp_state for return values.
 
*/
int getstate_btramp(btramp *r)
{
  return r->state;
}
/** Evaluate a btramp object for time slice dt
 
See btramp documentation for object states. Note that BTRAMP_UP and BTRAMP_DOWN 
will degenerate into BTRAMP_MAX and BTRAMP_MIN respectively. 
*/
btreal eval_btramp(btramp *r,btreal dt)
{
  btmutex_lock(&(r->mutex));
  
  if (r->state == BTRAMP_MAX)
  {
    *(r->scaler) = r->max;
  }
  else if (r->state == BTRAMP_MIN)
  {
    *(r->scaler) = r->min;
  }
  else if (r->state == BTRAMP_UP)
  {
    *(r->scaler) += dt*r->rate;
    if (*(r->scaler) > r->max)
    {
      *(r->scaler) = r->max;
      r->state = BTRAMP_MAX;
    }
  }
  else if (r->state == BTRAMP_DOWN)
  {
    *(r->scaler) -= dt*r->rate;
    if (*(r->scaler) < r->min)
    {
      *(r->scaler) = r->min;
      r->state = BTRAMP_MIN;
    }
  }
  //default case is to do nothing
  btmutex_unlock(&(r->mutex));
  return *(r->scaler);
}
void setrate_btramp(btramp *r,btreal rate)
{
  btmutex_lock(&(r->mutex));
    
  r->rate = rate;
    
  btmutex_unlock(&(r->mutex));

}
/**************************** btramp functions **********************************/




