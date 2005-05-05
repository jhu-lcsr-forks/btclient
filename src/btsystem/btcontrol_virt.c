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

/*! \file 

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

int set_bts(btstatecontrol *sc, btposition* pos, bttrajectory *trj)
{
  if (sc->mode == SCMODE_TORQUE){
    sc->pos = pos;
    /** \bug init pos function here */
    sc->trj = trj;
    /** \bug init trj function here */
  }
   
}
/**************************** state controller functions ************************/
/*! Initialize the state controller structure */
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
