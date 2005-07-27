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
/**************************** position functions ******************************/

/** Allocate memory for a new btposition object and initialize data structures*/
btposition* new_btposition(int size)
{
  btposition *btp;
  if ((btp = (btposition *)malloc(sizeof(btposition))) == NULL) 
  {
    syslog(LOG_ERR,"new_btposition: memory allocation failed");
    return NULL;
  }
  btp->q = new_vn(size);
  btp->qref = new_vn(size);
  btp->dq = new_vn(size);
  btp->t = new_vn(size);
}
/** Set up virtual interface*/
void setreg_btpos(btposition *btp,void *pos_dat, void *initfunc, void *evalfunc)
{
  btp->pos_reg = pos_dat;
  btp->init = initfunc;
  btp->eval = evalfunc;
  
}
/** Data access function to set the reference point. */
void setqref_btpos(btposition *btp,vect_n* qref)
{
  set_vn(btp->qref,qref);
}
/** Main evaluation function */
vect_n* eval_btpos(btposition *btp,vect_n* q,btreal dt)
{
  test_and_log(
      pthread_mutex_lock(&(btp->mutex)),"eval_btpos lock mutex failed");
	if (btp->state == 2){
    set_vn(btp->q,q);
    btp->dt = dt;
    set_vn(btp->t,(*(btp->eval))(btp->pos_reg,q,btp->qref,dt);
  }
  else {
    set_vn(btp->t, 0.0);
  }
  test_and_log(
      pthread_mutex_unlock(&(btp->mutex)),"eval_btpos unlock mutex failed");
  return btp->t;
}
int start_btpos(btposition *btp)
{
  test_and_log(
      pthread_mutex_lock(&(btp->mutex)),"eval_btpos lock mutex failed");
  if (btp->state == 0) return -1;
  else if (btp->state == 1 || btp->state == 2) {
    set_vn(btp->t,(*(btp->init))(btp->pos_reg,q,btp->qref);
    btp->state = 2;
  }
  test_and_log(
      pthread_mutex_unlock(&(btp->mutex)),"eval_btpos unlock mutex failed");
      
  return 0;
}
void stop_btpos(btposition *btp){
  btp->state = 1;
}
int getstate_btpos(btposition *btp)
{
  return btp->state;
}

/**************************** position functions ******************************/
/**************************** trajectory functions ******************************/
bttrajectory * new_bttrajectory()
{
  bttrajectory *bttrj;
  if ((bttrj = (bttrajectory *)malloc(sizeof(bttrajectory))) == NULL)
  {
    syslog(LOG_ERR,"new_bttrajectory: memory allocation failed");
    return NULL;
  }
  return bttrj;
}
void setpath_bttrj(bttrajectory *trj,void *crv_dat, void *initfunc, void *evalfunc)
{
  trj->crv = crv_dat;
  trj->init_T = initfunc;
  trj->S_of_dt = evalfunc;
}
void settraj_bttrj(bttrajectory *trj,void *trj_dat, void *initfunc, void *evalfunc)
{
  trj->trj = trj_dat;
  trj->init_S = initfunc;
  trj->Q_of_ds = evalfunc;
}

int start_bttrj(bttrajectory *trj)
{
  double ret1;
  vect_n *ret2;
  
  ret1 = (*(trj->init_T))(trj->trj,0.0);
  ret2 = (*(trj->init_S))(trj->crv,ret1);
  
}

vect_n* eval_bttrj(bttrajectory *trj,btreal dt)
{
  double ret1;
  vect_n *ret2;
  
  ret1 = (*(trj->S_of_dt))(trj->trj,dt);
  ret2 = (*(trj->Q_of_ds))(trj->crv,ret1);
  
  return ret2;
}
/**************************** trajectory functions ******************************/
/**************************** state controller functions ************************/
/** Assign position and trajectory objects to the state controller

\bug This should be able to be used "on the fly" providing sane transfer between 
position control techniques.


*/
int set_bts(btstatecontrol *sc, btposition* pos, bttrajectory *trj)
{
	/** \bug bumpless tranfer between control techniques here*/ 
  if (sc->mode == SCMODE_TORQUE){
    sc->pos = pos;
    /** \bug init pos function here */
    sc->trj = trj;
    /** \bug init trj function here */
  }
   
}
/*! Initialize the state controller structure 

  Loads a preallocated bts object with initial values.
*/
int init_bts(btstatecontrol *sc, int size)
{
    int err;

    sc->mode = 0;
    sc->last_dt = 1;
    sc->t = new_vn(size);
    sc->q = new_vn(size);
    sc->qref = new_vn(size);
    
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
vect_n* eval_bts(btstatecontrol *sc, vect_n* position, double dt)
{
    double newcommand;
    double newtorque;
    double cmptorque;
    int err;

    
    test_and_log(
      pthread_mutex_lock(&(sc->mutex)),"SCevaluate lock mutex failed");
    sc->last_dt = dt;
    set_vn(sc->q,position);
    switch (sc->mode)
    {
        case SCMODE_TORQUE://Idle
            fill_vn(sc->t,0.0);
            test_and_log(
              pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock (idle) mutex failed");
            return sc->t;
            break;
       case SCMODE_TRJ://PID
            setqref_btpos(sc->pos,eval_bttrj(sc->trj,dt));
        case SCMODE_PID://PID
            set_vn(sc->t,eval_btpos(sc->pos,sc->q,dt));
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
      
    if (getstate_bttrj(sc->trj) != BTTRAJ_STOPPED) stop_bttrj(sc->trj);
    setqref_btpos(sc->pos,sc->q);
    eval_btpos(sc->pos,sc->q,sc->last_dt);
    sc->mode = mode;
    
    test_and_log(
       pthread_mutex_unlock(&(sc->mutex)),"SCsetmode unlock (idle) mutex failed");
    return 0;
}


/**************************** state controller functions ************************/
