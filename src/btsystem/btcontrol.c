/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcontrol.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Apr 01, 2002
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *
 *  NOTES:
 *   
 *
 *  REVISION HISTORY:
 *  021124 - TH - File created
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *
 *======================================================================*/

/*! \file btcontrol.c
    \brief Systems controls algorithms and objects. 


-# PID object
-# Via Trajectory object

*/
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include "btcontrol.h"
#include "btos.h"
#include "btcontrol_virt.h"

#define BTDUMMYPROOF 


#define sign(x) (x>=0?1:-1)
#define Sgn(x) (x>=0.0?1.0:-1.0)


/**************************************************************************/

/*! Initializes the btPID object

The function sets the main parameters of a PID regulator. You
will only have to use this function a single time when you create the variable. Before
you start using PIDcalc, remember to set y and yref.
*/
void init_btPID(btPID *pid)
{
    pid->Kp = 0.0;
    pid->Kd = 0.0;
    pid->Ki = 0.0;
    pid->dt = 1.0;

    pid->se = 0;
    pid->e = 0;
    pid->laste = 0;
    pid->de = 0;
    pid->firsttick = 1;
    pid->y = 0;
    pid->yref = 0;
    pid->lastresult = 0;
    pid->state = 0; 
    pid->external_error_calc = 0;

    test_and_log(
      pthread_mutex_init(&(pid->mutex),NULL),
      "Could not initialize mutex for btPID.");

}
/** Initialize a btPID object for use with an external error.

*/
void init_err_btPID(btPID *pid)
{
  pid->external_error_calc = 1;
}
/** Reset the state parameters of the PID regulator

  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void reset_btPID(btPID *pid)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"reset_btPID lock mutex failed");
      
    pid->se = 0;
    pid->firsttick = 1;
    pid->yref = pid->y;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"reset_btPID unlock mutex failed");
}

/** Start the PID regulator

  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void start_btPID(btPID *pid)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"reset_btPID lock mutex failed");
      
    pid->se = 0;
    pid->firsttick = 1;
    pid->yref = pid->y;
    pid->state = 1;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"reset_btPID unlock mutex failed");
}
/** Stop the PID regulator

  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void stop_btPID(btPID *pid)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"reset_btPID lock mutex failed");
      
    pid->state = 0;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"reset_btPID unlock mutex failed");
}
/** Increments the state by dt and returns the output of the regulator.

step_btPID() updates the btPID state variable by incrementing the time by dt
and then calculates the error between y and yref.

\param *pid A pointer to the pre-allocated and initialized PIDregulator structure.

\return The regulator output for the present dt, y, and yref.
*/
btreal step_btPID(btPID *pid)
{
  test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"step_btPID lock mutex failed");
      
  if (pid->state){
    if (!pid->external_error_calc){
      pid->e = pid->yref - pid->y;
    }
    if (pid->firsttick)
    {
        pid->laste = pid->e;
        pid->firsttick = 0;
    }

    pid->de = (pid->e-pid->laste)/pid->dt;  //Backward euler
    pid->laste = pid->e;

    pid->se += pid->e;

    if (pid->saturation != 0)
        if ((fabs(pid->se)*pid->Ki) > pid->saturation)
        pid->se = sign(pid->se)*pid->saturation;

    pid->lastresult = pid->Kp*pid->e+pid->Kd*pid->de+pid->Ki*pid->se;

    
  }
  else{
    pid->lastresult = 0.0;
  }
  
  test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"step_btPID unlock mutex failed");
  return(pid->lastresult); //bz
}
/** Set measured value 'y'. See btPID object of more info.*/
void sety_btPID(btPID *pid, btreal y)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"sety_btPID lock mutex failed");
      
    pid->y = y;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"sety_btPID unlock mutex failed");
}
/** Set reference value 'yref'. See btPID object of more info.*/
void setyref_btPID(btPID *pid, btreal yref)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"setyref_btPID lock mutex failed");
      
    pid->yref = yref;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"setyref_btPID unlock mutex failed");
}
/** Set time step value 'dt'. See btPID object of more info.*/
void setdt_btPID(btPID *pid, btreal dt)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"setdt_btPID lock mutex failed");
      
    pid->dt = dt;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"setdt_btPID unlock mutex failed");
}
/** Get the last calculated effort value. See btPID object of more info.*/
btreal lastresult_btPID(btPID *pid)
{
  btreal ret;
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"lastresult_btPID lock mutex failed");
      
    ret = pid->lastresult;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"lastresult_btPID unlock mutex failed");
      
    return ret;
}
/** Evaluate the btPID object. See btPID object of more info.*/
btreal eval_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"eval_btPID lock mutex failed");
    
    pid->y = y;
    pid->yref = yref; 
    pid->dt = dt;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"eval_btPID unlock mutex failed");
      
    return step_btPID(pid);
}

btreal eval_err_btPID(btPID *pid, btreal error, btreal dt)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"eval_btPID lock mutex failed");
    
    pid->e = error;
    pid->dt = dt;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"eval_btPID unlock mutex failed");
      
    return step_btPID(pid);
}
/** Set all the input values. See btPID object of more info.*/
void setinputs_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"setinputs_btPID lock mutex failed");
    
    pid->y = y;
    pid->yref = yref; 
    pid->dt = dt;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"setinputs_btPID unlock mutex failed");
}
/** Set the gains. 

If the library is compiled with BTDUMMYPROOF, we will verify that you only
change the gains when tho regulator is in the off state. See btPID object of more info.*/
void setgains_btPID(btPID *pid, btreal Kp, btreal Kd, btreal Ki)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"setgains_btPID lock mutex failed");
      
#ifdef BTDUMMYPROOF
    if (!pid->state){  //Only allow setting gains when the regulator is not active
#endif

    pid->Kp = Kp;
    pid->Kd = Kd; 
    pid->Ki = Ki;
    
#ifdef BTDUMMYPROOF
    } 
    else{
      syslog(LOG_ERR,"setgains_btPID ignored because PID regulator is active");
    }
#endif

    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"setgains_btPID unlock mutex failed");
}
/** Set saturation of the regulator. If set anti-windup will kick in above this value. See btPID object of more info.

\bug verif anti-windup functionality
*/
void setsaturation_btPID(btPID *pid, btreal saturation)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"setsaturation_btPID lock mutex failed");
    
      pid->saturation = saturation;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"setsaturation_btPID unlock mutex failed");
}
/** Get the present gain values. */
void getgains_btPID(btPID *pid, btreal *Kp, btreal *Kd, btreal *Ki)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"getgains_btPID lock mutex failed");
    
    *Kp = pid->Kp;
    *Kd = pid->Kd;
    *Ki = pid->Ki;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"getgains_btPID unlock mutex failed");
}
/** Get the present input values */
void getinputs_btPID(btPID *pid, btreal *y, btreal *yref, btreal *dt)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"getinputs_btPID lock mutex failed");
    
    *y = pid->y;
    *yref = pid->yref;
    *dt = pid->dt;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"getinputs_btPID unlock mutex failed");
}
/** Get the present saturation values */
void getsaturation_btPID(btPID *pid, btreal *saturation)
{
    test_and_log(
      pthread_mutex_lock(&(pid->mutex)),"getsaturation_btPID lock mutex failed");
    
    *saturation = pid->saturation;
    
    test_and_log(
      pthread_mutex_unlock(&(pid->mutex)),"getsaturation_btPID unlock mutex failed");
}
/************************* btPID interface functions ***************************/
/* Repackages the above PID routines for vectors
   Provides the default function plugins for the btstate controller (in btcontrol_virt)
*/
void btposition_interface_mapf_btPID(btstatecontrol *sc, btPID_array *pid)
{
  mapposition_bts(sc,(void*) pid,
                     btposition_interface_reset_btPID,
                     btposition_interface_eval_btPID,
                     btposition_interface_pause_btPID);
}

vect_n* btposition_interface_eval_btPID(struct btposition_interface_struct* btp)
{
  btPID_array *pids;
  btPID* this;
  int cnt;
  
  pids = (btPID_array*)btp->dat;
  
  for (cnt = 0; cnt < pids->elements; cnt++){
    this = &(pids->pid[cnt]);
    sety_btPID(this,getval_vn(btp->q,cnt));//load all pointer data
    setyref_btPID(this,getval_vn(btp->qref,cnt));//load all pointer data
    setdt_btPID(this,*(btp->dt));
    step_btPID(this);//eval
    setval_vn(btp->t,cnt,lastresult_btPID(this));
  }
  return btp->t;
}

void btposition_interface_reset_btPID(struct btposition_interface_struct* btp)
{
  btPID_array *pids;
  int cnt;
  
  pids = (btPID_array*)btp->dat;
  
  for (cnt = 0; cnt < pids->elements; cnt++){
    start_btPID(&(pids->pid[cnt]));
  }
}

void btposition_interface_pause_btPID(struct btposition_interface_struct* btp)
{
  btPID_array *pids;
  int cnt;
  
  pids = (btPID_array*)btp->dat;
  
  for (cnt = 0; cnt < pids->elements; cnt++){
    stop_btPID(&(pids->pid[cnt]));
  }
}
/*
void btposition_interface_set_ref_btPID(void *dat,vect_n* ref)
{
  btPID_array *pids;
  int cnt;
  
  pids = (btPID_array*)dat;
  
  for (cnt = 0; cnt < pids->elements; cnt++){
    setyref_btPID(&(pids->pid[cnt]),getval_vn(ref,cnt));
  }
}
*/

/**************************** Continuous trajectory ***************************/


void create_ct(ct_traj *trj,vectray *vr)
{
  trj->state = BTTRAJ_OFF;
  trj->pwl = new_pwl();
  
  init_pwl_from_vectray(trj->pwl,vr);
  
}
int readfile_ct(ct_traj* ct,char* filename)
{
  vectray *vr;

  read_csv_file_vr(filename,&vr);
  create_ct(ct,vr);
  destroy_vr(vr);
  return 0;
}
vect_n* init_ct(ct_traj *trj)
{
  return dsinit_pwl(trj->pwl,0.0);
}
vect_n* eval_ct(ct_traj *trj, btreal dt)
{
  return ds_pwl(trj->pwl,dt);
}

int done_ct(ct_traj *trj)
{
  if (trj->pwl->proxy_s == arclength_pwl(trj->pwl))
    return 1;
  else return 0;
}

int bttrajectory_interface_getstate_ct(struct bttrajectory_interface_struct *btt)
{
  ct_traj* ct;
  ct = (ct_traj*)btt->dat;
  
  if (done_ct(ct))
    return BTTRAJ_DONE;
  else return BTTRAJ_RUN;
}

vect_n* bttrajectory_interface_reset_ct(struct bttrajectory_interface_struct *btt)
{
  ct_traj* ct;
  //syslog(LOG_ERR,"Starting reset ct");
  ct = (ct_traj*)btt->dat;
  return dsinit_pwl(ct->pwl,0.0);
}

vect_n* bttrajectory_interface_eval_ct(struct bttrajectory_interface_struct *btt)
{
  ct_traj* ct;
  ct = (ct_traj*)btt->dat;
  return ds_pwl(ct->pwl,*(btt->dt));
}

void bttrajectory_interface_mapf_ct(btstatecontrol *sc,ct_traj *trj)
{
  maptrajectory_bts(sc,(void*) trj,
                       bttrajectory_interface_reset_ct,
                       bttrajectory_interface_eval_ct,
                       bttrajectory_interface_getstate_ct);

}
/** \bug No error checking or error return*/


/******************************************************************************/

/** Assumes trj->vr has been loaded with a valid vectray


*/
void start_via_trj(via_trj *trj,double start,int n,int col,double acc)
{
    double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next;

    trj->idx = 0;
    trj->n = n;
    trj->col = col;
    setval_vn(idx_vr(trj->vr,0),col,start); //force starting point to be our current point
    trj->trj_acc = acc;

    trj->segment = 0;//acc first

    Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
    Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
    trj->v_prev = 0;
    trj->v_next = Dq_next/Dt_next;
    CalcSegment(&(trj->seg),getval_vn(idx_vr(trj->vr,trj->idx),trj->col), getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col),
                getval_vn(idx_vr(trj->vr,trj->idx),0), getval_vn(idx_vr(trj->vr,trj->idx+1),0),trj->v_prev,trj->v_next,trj->trj_acc, 0);

    trj->acc = trj->seg.acc1;
    trj->dt_acc = trj->seg.dt_acc1;
    trj->t=0;
    trj->t0=0;
    trj->t_acc = trj->dt_acc;
    trj->q0 = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
    trj->v0 = 0;
    trj->state = BTTRAJ_RUN;

    /*syslog(LOG_ERR, "col=%d idx=%d vel=%f t_vel=%f t_acc=%f q0=%f v0=%f t=%f t0=%f q1=%f q2=%f t1=%f t2=%f",
           trj->col,trj->idx,trj->vel,trj->t_vel,trj->t_acc,
           trj->q0,trj->v0,trj->t,trj->t0,(trj->tf)[trj->idx][trj->col], (trj->tf)[trj->idx+1][trj->col],
                (trj->tf)[trj->idx][0], (trj->tf)[trj->idx+1][0]); */
}

double eval_via_trj(via_trj *trj,double dt)
{
    double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next,acc_next,t_acc_next,end,et,cmd;
    double tn,tp,qn,qp;
    if (trj->state == BTTRAJ_RUN)
    {
        trj->t += dt;

        if ((trj->segment == 0) && (trj->t > trj->t_acc))
        { //done with acc, set up vel
            if (trj->idx >= trj->n-1)
            {
                trj->state = BTTRAJ_STOPPED;
            }
            else
            {
                trj->vel = trj->seg.vel;
                trj->dt_vel = trj->seg.dt_vel;
                trj->t0 = trj->t_acc;
                trj->t_vel = trj->dt_vel + trj->t0;
                trj->q0 = trj->q0 + trj->v0*trj->dt_acc + 0.5*trj->dt_acc*trj->dt_acc*trj->acc;
                trj->segment = 1;
            }

        }
        else if((trj->segment == 1) && (trj->t > trj->t_vel))
        {//setup acc segment
            trj->idx++;
            if (trj->idx >= trj->n-1)
            {
                trj->acc = trj->seg.acc2;
                trj->dt_acc = trj->seg.dt_acc2;
                trj->t0 = trj->t_vel;
                trj->t_acc = trj->dt_acc + trj->t0;
                trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
                trj->v0 = trj->vel;
                trj->segment = 0;//acc first
            }
            else
            {
              tp = getval_vn(idx_vr(trj->vr,trj->idx-1),0);
              tn = getval_vn(idx_vr(trj->vr,trj->idx),0);
              qn = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
              qp = getval_vn(idx_vr(trj->vr,trj->idx-1),trj->col);
              Dt = tn - tp;
              Dq = qn - qp;
              trj->v_prev = Dq/Dt;
              
              if (trj->idx >= trj->n-2){
                end = 2;
                trj->v_next = 0;
              }
              else{
                end = 1;
                Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
                Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
                trj->v_next = Dq_next/Dt_next;
              }
              CalcSegment(&(trj->seg),getval_vn(idx_vr(trj->vr,trj->idx),trj->col), getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col),
                            tn,getval_vn(idx_vr(trj->vr,trj->idx+1),0),trj->v_prev,trj->v_next,trj->trj_acc, end);
                trj->segment = 0;//acc first
                trj->acc = trj->seg.acc1;
                trj->dt_acc = trj->seg.dt_acc1;
                trj->t0 = trj->t_vel;
                trj->t_acc = trj->dt_acc + trj->t0;
                trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
                trj->v0 = trj->vel;
            }

           /*syslog(LOG_ERR, "col=%d idx=%d vel=%f t_vel=%f t_acc=%f q0=%f v0=%f t=%f t0=%f q1=%f q2=%f t1=%f t2=%f",
           trj->col,trj->idx,trj->vel,trj->t_vel,trj->t_acc,
           trj->q0,trj->v0,trj->t,trj->t0,(trj->tf)[trj->idx][trj->col], (trj->tf)[trj->idx+1][trj->col],
                (trj->tf)[trj->idx][0], (trj->tf)[trj->idx+1][0]); */
        }



        if (trj->segment == 0)
        {//in acceleration segment
            et = trj->t - trj->t0;
            cmd = trj->q0 + trj->v0*et + 0.5*et*et*trj->acc;
        }
        else
        {
            et = trj->t - trj->t0; //elapsed time
            cmd = trj->q0 + trj->vel*et;
        }

    }
    //syslog(LOG_ERR, "VIA %d %d %f", trj->idx, trj->col, cmd);
    return cmd;

}

void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, double v_prev, double v_next, double seg_acc, int end)
{
    double dt,dq;
    double vel,acc1,acc2,dt_vel,dt_acc1,dt_acc2;

           
    dt = t2 - t1;
    dq = q2 - q1;

    if (end == 0)
    {
        acc1 = Sgn(dq)*seg_acc;
        dt_acc1 = dt - sqrt(dt*dt - 2*dq/acc1);
        vel = dq/(dt - dt_acc1);
        acc2 = Sgn(v_next - vel)*seg_acc;
        dt_acc2 = (v_next - vel)/acc2;
        dt_vel = dt - dt_acc1 - dt_acc2/2;
    }
    else if (end == 1)
    {
        vel = dq/dt;
        acc1 = Sgn(vel - v_prev)*seg_acc;
        dt_acc1 = (vel - v_prev)/acc1;
        acc2 = Sgn(v_next - vel)*seg_acc;
        dt_acc2 = (v_next - vel)/acc2;
        dt_vel = dt - dt_acc1/2 - dt_acc2/2;
    }
    else
    {
        acc2 = Sgn(dq)*seg_acc;
        dt_acc1 = dt - sqrt(dt*dt + 2*dq/acc2);
        vel = dq/(dt - dt_acc2);
        acc1 = Sgn((vel - v_prev))*seg_acc;
        dt_acc1 = ((vel - v_prev))/acc1;
        dt_vel = dt - dt_acc2 - dt_acc1/2;
    }

    seg->vel = vel;
    seg->acc1 = acc1;
    seg->acc2 = acc2;
    seg->dt_vel = dt_vel;
    seg->dt_acc1 = dt_acc1;
    seg->dt_acc2 = dt_acc2;
}



/**************************** btramp functions **********************************/
/** Initialize the data for a btramp object.

This should be called only once as it allocates memory for a mutex object.
*/
void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate)
{
  test_and_log(
      pthread_mutex_init(&(r->mutex),NULL),
      "Could not initialize  mutex for simple controller.");
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
 test_and_log(
      pthread_mutex_lock(&(r->mutex)),"btramp_up lock mutex failed");
  r->state = state;
  test_and_log(
      pthread_mutex_unlock(&(r->mutex)),"btramp_up unlock mutex failed");
}
/** Return the present value of a btramp scaler.

*/
btreal get_btramp(btramp *r)
{
  return *(r->scaler);
}
/** Evaluate a btramp object for time slice dt

See btramp documentation for object states. Note that BTRAMP_UP and BTRAMP_DOWN 
will degenerate into BTRAMP_MAX and BTRAMP_MIN respectively. 
*/
btreal eval_btramp(btramp *r,btreal dt)
{
   test_and_log(
      pthread_mutex_lock(&(r->mutex)),"eval_btramp lock mutex failed");
  if (r->state == BTRAMP_MAX){
    *(r->scaler) = r->max;
  }
  else if (r->state == BTRAMP_MIN){
    *(r->scaler) = r->min;
  }
  else if (r->state == BTRAMP_UP){
    *(r->scaler) += dt*r->rate;
    if (*(r->scaler) > r->max){
      *(r->scaler) = r->max;
      r->state = BTRAMP_MAX;
    }
  }
  else if (r->state == BTRAMP_DOWN){
    *(r->scaler) -= dt*r->rate;
    if (*(r->scaler) < r->min) {
      *(r->scaler) = r->min;
      r->state = BTRAMP_MIN;
    }
  }
  //default case is to do nothing
  test_and_log(
      pthread_mutex_unlock(&(r->mutex)),"eval_btramp unlock mutex failed");
  return *(r->scaler);
}

/**************************** btramp functions **********************************/

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


