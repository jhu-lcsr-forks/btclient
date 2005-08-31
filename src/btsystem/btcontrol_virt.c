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




/**************************** position functions ******************************/
/** \bug This function is not needed and should be deleted*/
void init_bttrj(bttrajectory_interface *btt)
{
  btt->state = BTTRAJ_STOPPED;
  test_and_log(
    pthread_mutex_init(&(btt->mutex),NULL),
    "Could not initialize  mutex for init_bttrj.");
}
void mapdata_bttrj(bttrajectory_interface *btt, vect_n* qref, double *dt)
{
  btt->qref = qref;
  btt->dt = dt;
}


vect_n* eval_bttrj(bttrajectory_interface *btt)
{
  if (btt->state == BTTRAJ_INPREP)
  {
    if (btt->trj.state == BTTRAJ_STOPPED)
    {
      btt->state = BTTRAJ_READY;
    }
    set_vn(btt->qref, getval_pwl(&(btt->pth),evaluate_traptrj(&(btt->trj),*(btt->dt))));
  }

  else if (btt->state == BTTRAJ_RUN)
    set_vn(btt->qref, (*(btt->eval))(btt));//evaluate path

  return btt->qref;
}

int prep_bttrj(bttrajectory_interface *btt,vect_n* q, btreal vel, btreal acc)
{
  char vect_buf1[200];

  if(btt->state == BTTRAJ_STOPPED)
  {
    clear_pwl(&(btt->pth));
    add_arclen_point_pwl(&(btt->pth),q);
    add_arclen_point_pwl(&(btt->pth),(*(btt->reset))(btt));

    setprofile_traptrj(&(btt->trj), vel, acc);
    start_traptrj(&(btt->trj), arclength_pwl(&(btt->pth)));
    btt->state = BTTRAJ_INPREP;

    return 1;
  }

  return 0;
}
int start_bttrj(bttrajectory_interface *btt)
{
  int ret = 0;
  if(btt->state == BTTRAJ_READY)
  {
    btt->state = BTTRAJ_RUN;
    ret = 1;
  }
  return ret;
}
int stop_bttrj(bttrajectory_interface *btt)
{
  btt->state == BTTRAJ_STOPPED;
  return 0;
}

int getstate_bttrj(bttrajectory_interface *btt)
{
  return btt->state;
}
/**************************** trajectory functions ******************************/
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

/**************************** trajectory functions ******************************/
/**************************** state controller functions ************************/
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq,
                        vect_n* qref, vect_n* t, double *dt)
{
  sc->q = q;
  sc->qref = qref;
  sc->dq = dq;
  sc->ddq = ddq;
  sc->dt = dt;
  sc->t = t;
  mapdata_btpos(&(sc->btp),q,dq,ddq,qref,t,dt);
  mapdata_bttrj(&(sc->btt),qref,dt);
  init_pwl(&(sc->btt.pth),len_vn(qref),2);

}
void maptrajectory_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* getstate)
{
  test_and_log( pthread_mutex_lock(&(sc->mutex)),"mapobj_bttrj lock mutex failed");

  if (sc->btt.state == BTTRAJ_STOPPED)
  {
    sc->btt.reset = reset;
    sc->btt.eval = eval;
    sc->btt.getstate = getstate;
    sc->btt.dat = dat;
    sc->btt.state = BTTRAJ_STOPPED;
  }
  test_and_log(pthread_mutex_unlock(&(sc->mutex)),"mapobj_bttrj unlock mutex (idle) failed");
}
void mapposition_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* pause)
{
  test_and_log( pthread_mutex_lock(&(sc->mutex)),"mapobj_bttrj lock mutex failed");
  if (sc->mode == SCMODE_TORQUE)
  {
    sc->btp.reset = reset;
    sc->btp.eval = eval;
    sc->btp.pause = pause;
    sc->btp.dat = dat;
    //put everything into a known state
    set_vn(sc->btp.qref,sc->btp.q);
    (*(sc->btp.reset))(&sc->btp);
    sc->mode = SCMODE_TORQUE;
    sc->btt.state = BTTRAJ_STOPPED;
  }
  test_and_log(pthread_mutex_unlock(&(sc->mutex)),"mapobj_bttrj unlock mutex (idle) failed");
}
/** Assign position and trajectory objects to the state controller
 
\bug This should be able to be used "on the fly" providing sane transfer between 
position control techniques.
 
\bug This function no longer used after rearrangement
*/
int set_bts(btstatecontrol *sc, btposition_interface* pos, bttrajectory_interface *trj)
{
  /** \bug bumpless tranfer between control techniques here*/
  if (sc->mode == SCMODE_TORQUE)
  {
    //sc->btp = pos;
    /** \bug init pos function here */
    //sc->btt = trj;
    /** \bug init trj function here */
  }

}
/*! Initialize the state controller structure
 
  Loads a preallocated bts object with initial values.
*/
int init_bts(btstatecontrol *sc)
{
  int err;

  sc->mode = 0;
  sc->last_dt = 1;
  sc->btt.dat = NULL;
  sc->btp.dat = NULL;

  test_and_log(
    pthread_mutex_init(&(sc->mutex),NULL),
    "Could not initialize  mutex for state controller.");
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


  test_and_log(
    pthread_mutex_lock(&(sc->mutex)),"SCevaluate lock mutex failed");
  sc->last_dt = *(sc->dt);

  switch (sc->mode)
  {
  case SCMODE_TORQUE://Idle
    fill_vn(sc->t,0.0);
    test_and_log(
      pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock (idle) mutex failed");
    return sc->t;
    break;
  case SCMODE_TRJ://PID
    eval_bttrj(&sc->btt);
  case SCMODE_POS://PID
    set_vn(sc->t,(*(sc->btp.eval))(&sc->btp));
    test_and_log(
      pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock (idle) mutex failed");
    return sc->t;
    break;
  default:
    sc->error = 1;
    fill_vn(sc->t,0.0);
    test_and_log(
      pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock (idle) mutex failed");
    return sc->t;
  }
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

  test_and_log(
    pthread_mutex_lock(&(sc->mutex)),"SCsetmode lock mutex failed");

  switch (mode)
  {
  case SCMODE_POS:
    if (sc->btp.dat == NULL)
      sc->mode = SCMODE_TORQUE;
    else
    {
      if (sc->btt.state != BTTRAJ_STOPPED)
        stop_bttrj(&sc->btt);

      set_vn(sc->btp.qref,sc->btp.q);
      (*(sc->btp.reset))(&sc->btp);
      sc->mode = SCMODE_POS;
    }
    break;
  case SCMODE_TRJ:
    if (sc->btp.dat == NULL )
      sc->mode = SCMODE_TORQUE;
    else if (sc->btt.dat == NULL)
    { //do nothing
    }
    else
    {
      if (sc->btt.state != BTTRAJ_STOPPED)
        stop_bttrj(&sc->btt);

      set_vn(sc->btp.qref,sc->btp.q);
      (*(sc->btp.reset))(&sc->btp);
      sc->mode = SCMODE_TRJ;
    }
    break;
  default:
    break;
  }
    test_and_log(
      pthread_mutex_unlock(&(sc->mutex)),"SCsetmode unlock (idle) mutex failed");
    return 0;
  }
  int prep_trj_bts(btstatecontrol *sc,btreal vel, btreal acc)
  {
    return prep_bttrj(&sc->btt,sc->q,vel,acc);
  }
  int start_trj_bts(btstatecontrol *sc)
  {
    return start_bttrj(&sc->btt);
  }
  int stop_trj_bts(btstatecontrol *sc)
  {
    return stop_bttrj(&sc->btt);
  }

  /**************************** state controller functions ************************/
