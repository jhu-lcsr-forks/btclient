/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btjointcontrol.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Nov 24, 2002
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *
 *  NOTES:
 *   A simple Torque, PID & Trapezoidal velocity trajectory controller
 *
 *  REVISION HISTORY:
 *  021124 - TH - File created
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *  050429 - TH - Update to use new h files and math library
 *
 *======================================================================*/

/*! \file btjointcontrol.c
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
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include "btjointcontrol.h"
#include "btcan.h"
#include "btos.h"
#include "btmath.h"
#include "btcontrol_virt.h"

#define sign(x) (x>=0?1:-1)      //Integer sign
#define Sgn(x) (x>=0.0?1.0:-1.0) //Double sign
//enum {SCMODE_IDLE,SCMODE_TORQUE,SCMODE_PID,SCMODE_TRJ};
/*! Initialize the SimpleCtl structure */
int SCinit(SimpleCtl *sc)
{
    int err;

    sc->mode = SCMODE_IDLE;
    sc->requested_torque = 0.0;
    sc->kill_torque = 500000;
    sc->last_dt = 1;
    sc->command_torque = 0;
    sc->trj.state = 0;
    sc->trj.acc = .00000001;
    sc->trj.vel = .00000001;
    PIDinit(&(sc->pid),0, 0, 0, 0);
    
    test_and_log(
      pthread_mutex_init(&(sc->mutex),NULL),
      "Could not initialize  mutex for simple controller.");
}

/*! Evaluate the Simple Controller

  See the documentation for SimpleCtl for an overview.

  This function takes as input the current position and the time since the last evaluation.
It returns a torque.

\param position the measured position
\param dt the time step we want to evaluate with.
\return Returns a torque command.
*/
double SCevaluate(SimpleCtl *sc, double position, double dt)
{
    double newcommand;
    double newtorque;
    double cmptorque;
    int err;

    
    test_and_log(
      pthread_mutex_lock(&(sc->mutex)),"SCevaluate lock mutex failed");
    
    sc->position = position;
    switch (sc->mode)
    {
        case SCMODE_IDLE://Idle
            test_and_log(
              pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock mutex (idle) failed");
            sc->command_torque = 0.0;
            return sc->command_torque;
            break;
        case SCMODE_TORQUE://Torque
            sc->command_torque = sc->requested_torque;
            test_and_log(
              pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock mutex (idle) failed");
            return sc->command_torque;
            break;
        case SCMODE_PID://PID
            if (sc->trj.state == BTTRAJ_RUN)
            {
                newcommand = evaluate_trajectory(&(sc->trj),dt);
                sc->pid.yref = newcommand;
            }
            sc->pid.y = sc->position;
            sc->pid.dt = dt;
            newtorque = PIDcalc(&(sc->pid));
            
            sc->command_torque = newtorque;

            test_and_log(
              pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock mutex (idle) failed");
            return newtorque;
            break;
        default:
            sc->error = 1;
            test_and_log(
              pthread_mutex_unlock(&(sc->mutex)),"SCevaluate unlock mutex (idle) failed");
            return 0.0;
        }

}



/*! \brief Set the controller mode

  Sets the controller mode. To keep anything from "jumping" unexpectedly, we reset our
  pid controller whenever we switch into pidmode.

*/
int SCsetmode(SimpleCtl *sc, int mode)
{
    int err;
    double tmp;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetmode lock mutex failed: %d", err);
    }
    if (sc->trj.state != BTTRAJ_DONE) sc->trj.state = BTTRAJ_DONE;
    sc->pid.y = sc->position;
    PIDreset(&(sc->pid));
    tmp = PIDcalc(&(sc->pid));
    sc->mode = mode;
    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetmode unlock mutex failed: %d", err);
    }
    return 0;
}


int SCsettorque(SimpleCtl *sc,double torque)
{
    int err;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsettorque lock mutex failed: %d", err);
    }
    sc->requested_torque = torque; //use a buffer since we are not necessarily running from the same thread as evaluatecontroller
    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsettorque unlock mutex failed: %d", err);
    }

    if (sc->mode != SCMODE_TORQUE)
    {
        return -1;
    }
    return 0;
}

int SCsetpid(SimpleCtl *sc,double kp,double kd,double ki, double saturation) //!! not thread safe yet! We may update these in the middle of their use.
{

    int err;

    if (sc->trj.state == BTTRAJ_RUN || sc->mode == SCMODE_PID) //make sure we don't screw with stuff in the middle of a trajectory.
        return -1;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetpid lock mutex failed: %d", err);
    }
    sc->pid.Kp = kp;
    sc->pid.Kd = kd;
    sc->pid.Ki = ki;
    sc->pid.saturation = saturation;

    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetpid unlock mutex failed: %d", err);
    }

    return 0;
}

int SCsetcmd(SimpleCtl *sc,double cmd) //sets pid command position. if in traj mode, stops trajectory
{
    int err;

    if (sc->trj.state != BTTRAJ_DONE)
        return -1;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetcmd lock mutex failed: %d", err);
    }
    sc->pid.yref = cmd;
    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsetcmd unlock mutex failed: %d", err);
    }
    return 0;
}

int SCsettrjprof(SimpleCtl *sc,double vel,double acc)
{
    int err;

    if (sc->trj.state == BTTRAJ_RUN)
        return -1;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsettrjprof lock mutex failed: %d", err);
    }
    sc->trj.vel = fabs(vel);
    sc->trj.acc = fabs(acc);

    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCsettrjprof unlock mutex failed: %d", err);
    }
    return 0;

}
int SCstarttrj(SimpleCtl *sc,double end)
{
    int err;
    double here;

    if (sc->trj.state != BTTRAJ_DONE)
        return -1;

    err = pthread_mutex_lock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCstarttrj lock mutex failed: %d", err);
    }
    here = sc->pid.yref;
    init_trajectory(&(sc->trj),here,end);

    err = pthread_mutex_unlock( &(sc->mutex) );
    if(err != 0)
    {
        syslog(LOG_ERR, "SCstarttrj unlock mutex failed: %d", err);
    }
    return 0;
}

/*! Calculates all the variables necessary to set up a trajectory.

\param start the starting position
\param end the destination position


*/
void init_trajectory(trajectory *traj, double start, double end) //assumes that velocity and acceleration have been set to reasonable values
{
    double ax; //acceleration x
    double dist;


    traj->cmd = 0;
    traj->start_point = start;
    traj->end_point = end;

    traj->t1 = (traj->vel / traj->acc); //Calculate the "ramp-up" time

    dist = traj->end_point - traj->start_point;

    if (dist > 0) traj->sign = 1;
    else traj->sign = -1;

    //Get the point at which acceleration stops
    ax = 0.5*traj->acc*traj->t1*traj->t1; //x = 1/2 a t^2

    dist = fabs(dist);
    traj->end = dist;
    if (ax > dist/2) //If the acceleration completion point is beyond the halfway point
        ax = dist/2; //Stop accelerating at the halfway point
    traj->x1 = ax;   //Set the top left point of the trapezoid
    traj->x2 = dist - ax; //Set the top right point of the trapezoid
    traj->t2 = (dist-(2*ax)) / traj->vel + traj->t1; //Find the time to start the "ramp-down"
    traj->t = 0;
    traj->state = BTTRAJ_RUN;
    if (dist == 0.0) traj->state = BTTRAJ_DONE;
}

double calc_traj_time(trajectory *traj, double start, double end, double vel, double acc)
{
    double ax; //acceleration x
    double dist,t1,t2,time;


    t1 = (vel / acc); //Calculate the "ramp-up" time

    dist = end - start;

    //Get the point at which acceleration stops
    ax = 0.5*acc*t1*t1; //x = 1/2 a t^2
    dist = fabs(dist);

    if (ax > dist/2) //If we never reach max velacity, back calculate for time
    {
        ax = dist/2; //Stop accelerating at the halfway point
        time = 2*sqrt(2*ax/acc);
    }
    else
    {
        time = (dist-(2*ax)) / vel + 2*t1;
    }
    return time;
}

void sync_trajectory(trajectory *traj, double start, double end, double vel, double acc, double t)
{
    double ax; //acceleration x
    double dist,t1,t2,time;


    t1 = (vel / acc); //Calculate the "ramp-up" time
    if (2*t1 > t) t1 = t;
    t2 = t - 2*t1;
    dist = fabs(end - start);

    traj->acc= dist/(t1*t1+t1*t2);
    traj->vel= traj->acc*t1;
}

double scale_vel(trajectory *traj, double start, double end, double vel, double acc, double t)
{
    double ax; //acceleration x
    double dist,t1,t2,time,acc1;


    t1 = (vel / acc); //Calculate the "ramp-up" time
    if (2*t1 > t) t1 = t;
    t2 = t - 2*t1;
    dist = fabs(end - start);

    acc1= dist/(t1*t1+t1*t2);
    return (acc1*t1);
}
double scale_acc(trajectory *traj, double start, double end, double vel, double acc, double t)
{
    double ax; //acceleration x
    double dist,t1,t2,time,acc1;


    t1 = (vel / acc); //Calculate the "ramp-up" time
    if (2*t1 > t) t1 = t;
    t2 = t - 2*t1;
    dist = fabs(end - start);
    acc1 = dist/(t1*t1+t1*t2);
    return(acc1);
}

/*! Calculate next position on the trajectory

evaluate_trajectory generates points on a trapezoidal velocity trajectory. The points accerate with
constant acceleration specified by acc up to a maximum velocity specified by vel. The max velocity
is maintained until approximately the time it will take to decellerate at acc to a velocity of zero.

\param *traj pointer to the trajectory structure
\param dt the time step to take (in the same units as vel and acc)

\return The value returned is the next point on the trajectory after a step of dt.


*/
double evaluate_trajectory(trajectory *traj,double dt)
{
    double remaining_time,newtime,result;

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
                traj->state = BTTRAJ_DONE;
            }

            newtime = (remaining_time-dt);
            if(newtime <= 0)
            {
                traj->cmd = traj->end;
                traj->state = BTTRAJ_DONE;
            }
            traj->cmd =  traj->end - 0.5 * traj->acc * newtime * newtime;
        }

    }
    result = (traj->start_point + traj->sign*traj->cmd);

    if (isnan(result))
    {
        syslog(LOG_ERR, "nan in eval_traj");
        traj->cmd = traj->end;
        traj->state = BTTRAJ_DONE;
        return traj->end_point;
    }
    return (traj->start_point + traj->sign*traj->cmd);
}


/*! Initializes the PIDregulator data structure

The function sets the main parameters of a PID regulator. You
will only have to use this function a single time when you create the variable. Before
you start using PIDcalc, remember to set y and yref.
*/
void PIDinit(PIDregulator *pid,double Kp, double Kd, double Ki, double dt)
{
    pid->Kp = Kp;
    pid->Kd = Kd;
    pid->Ki = Ki;
    pid->dt = dt;
    pid->filter_de = 0;
    pid->se = 0;
    pid->e = 0;
    pid->laste = 0;
    pid->de = 0;
    pid->firsttick = 1;
    pid->y = 0;
    pid->yref = 0;
    pid->lastresult = 0;
    
    pid->de_filter = new_btfilter(5);
    init_btfilter_diff(pid->de_filter, 2, dt, 10);
    
    pid->se_filter = new_btfilter(5);
    init_btfilter_diff(pid->se_filter, 2, dt, 10);
    


}
/** Reset the state parameters of the PID regulator

  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void PIDreset(PIDregulator *pid)
{
    pid->se = 0;
    pid->firsttick = 1;
    pid->yref = pid->y;
}

/** Increments the state by dt and returns the output of the regulator.

PIDcalc updates the PIDregulator state variable by incrementing the time by dt
and then calculates the error between y and yref.

\param *pid A pointer to the pre-allocated and initialized PIDregulator structure.

\return The regulator output for the present dt, y, and yref.
*/
double PIDcalc(PIDregulator *pid)
{
    pid->e = pid->yref - pid->y;

    if (pid->firsttick)
    {
        pid->laste = pid->e;
        pid->firsttick = 0;
    }

    pid->de = (pid->e-pid->laste)/pid->dt;  //Backward euler
    if (pid->filter_de)
        pid->de = eval_btfilter(pid->de_filter,pid->e);
    pid->laste = pid->e;

    pid->se += pid->e;
    pid->fe = eval_btfilter(pid->se_filter,pid->se);
    if (pid->saturation != 0)
        if ((fabs(pid->se)*pid->Ki) > pid->saturation)
        pid->se = sign(pid->se)*pid->saturation;

    pid->lastresult = pid->Kp*pid->e+pid->Kd*pid->de+pid->Ki*pid->se;

    return(pid->lastresult); //bz
}

double linterp(double x1, double y1, double x2, double y2, double x)
{
    if ((x2-x1) != 0.0)
        return (x*(y2-y1)/(x2-x1));
    else
        return (y2+y1)/2;
}

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003, 2004, 2005 Barrett Technology, Inc.     *
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


