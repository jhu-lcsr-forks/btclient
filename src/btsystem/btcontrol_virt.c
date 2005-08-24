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
  mapdata_btpos(sc->btp,q,dq,ddq,qref,t,dt);
}

/** Assign position and trajectory objects to the state controller

\bug This should be able to be used "on the fly" providing sane transfer between 
position control techniques.


*/
int set_bts(btstatecontrol *sc, btposition_interface* pos, bttrajectory_interface *trj)
{
	/** \bug bumpless tranfer between control techniques here*/ 
  if (sc->mode == SCMODE_TORQUE){
    sc->btp = pos;
    /** \bug init pos function here */
    sc->trj = trj;
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
           // set_vn(sc->btp->qref,eval_bttrj(sc->trj,dt));
        case SCMODE_POS://PID
            set_vn(sc->t,(*(sc->btp->eval))(sc->btp));
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
      
    //if (getstate_bttrj(sc->trj) != BTTRAJ_STOPPED) stop_bttrj(sc->trj);
    
    set_vn(sc->btp->qref,sc->btp->q);
    (*(sc->btp->reset))(sc->btp);
    //(*(sc->btp->eval))(sc->btp);
    sc->mode = mode;
    
    test_and_log(
       pthread_mutex_unlock(&(sc->mutex)),"SCsetmode unlock (idle) mutex failed");
    return 0;
}


/**************************** state controller functions ************************/
