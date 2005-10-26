/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcontrol.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Apr 01, 2002
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
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


#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include "btmath.h"
#include "btcontrol.h"
#include "btos.h"
#include "btstatecontrol.h"

#define BT_DUMMY_PROOF

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

  btmutex_init(&pid->mutex);

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
  btmutex_lock(&(pid->mutex));

  pid->se = 0;
  pid->firsttick = 1;
  pid->yref = pid->y;

  btmutex_unlock(&(pid->mutex));
}

/** Start the PID regulator
 
  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void start_btPID(btPID *pid)
{
  btmutex_lock(&(pid->mutex));

  pid->se = 0;
  pid->firsttick = 1;
  pid->yref = pid->y;
  pid->state = 1;

  btmutex_unlock(&(pid->mutex));
}
/** Stop the PID regulator
 
  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
*/
void stop_btPID(btPID *pid)
{
  btmutex_lock(&(pid->mutex));

  pid->state = 0;

  btmutex_unlock(&(pid->mutex));
}
/** Increments the state by dt and returns the output of the regulator.
 
step_btPID() updates the btPID state variable by incrementing the time by dt
and then calculates the error between y and yref.
 
\param *pid A pointer to the pre-allocated and initialized PIDregulator structure.
 
\return The regulator output for the present dt, y, and yref.
*/
btreal step_btPID(btPID *pid)
{
  btmutex_lock(&(pid->mutex));

  if (pid->state)
  {
    if (!pid->external_error_calc)
    {
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
  else
  {
    pid->lastresult = 0.0;
  }

  btmutex_unlock(&(pid->mutex));
  return(pid->lastresult); //bz
}
/** Set measured value 'y'. See btPID object of more info.*/
void sety_btPID(btPID *pid, btreal y)
{
  btmutex_lock(&(pid->mutex));

  pid->y = y;

  btmutex_unlock(&(pid->mutex));
}
/** Set reference value 'yref'. See btPID object of more info.*/
void setyref_btPID(btPID *pid, btreal yref)
{
  btmutex_lock(&(pid->mutex));

  pid->yref = yref;

  btmutex_unlock(&(pid->mutex));
}
/** Set time step value 'dt'. See btPID object of more info.*/
void setdt_btPID(btPID *pid, btreal dt)
{
  btmutex_lock(&(pid->mutex));

  pid->dt = dt;

  btmutex_unlock(&(pid->mutex));
}
/** Get the last calculated effort value. See btPID object of more info.*/
btreal lastresult_btPID(btPID *pid)
{
  btreal ret;
  btmutex_lock(&(pid->mutex));

  ret = pid->lastresult;

  btmutex_unlock(&(pid->mutex));

  return ret;
}
/** Evaluate the btPID object. See btPID object of more info.*/
btreal eval_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
  btmutex_lock(&(pid->mutex));

  pid->y = y;
  pid->yref = yref;
  pid->dt = dt;

  btmutex_unlock(&(pid->mutex));

  return step_btPID(pid);
}

btreal eval_err_btPID(btPID *pid, btreal error, btreal dt)
{
  btmutex_lock(&(pid->mutex));

  pid->e = error;
  pid->dt = dt;

  btmutex_unlock(&(pid->mutex));

  return step_btPID(pid);
}
/** Set all the input values. See btPID object of more info.*/
void setinputs_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
  btmutex_lock(&(pid->mutex));

  pid->y = y;
  pid->yref = yref;
  pid->dt = dt;

  btmutex_unlock(&(pid->mutex));
}
/** Set the gains.
 
If the library is compiled with BT_DUMMY_PROOF, we will verify that you only
change the gains when tho regulator is in the off state. See btPID object of more info.*/
void setgains_btPID(btPID *pid, btreal Kp, btreal Kd, btreal Ki)
{
  btmutex_lock(&(pid->mutex));

#ifdef BT_DUMMY_PROOF

  if (!pid->state)
  {  //Only allow setting gains when the regulator is not active
#endif

    pid->Kp = Kp;
    pid->Kd = Kd;
    pid->Ki = Ki;

#ifdef BT_DUMMY_PROOF

  }
  else
  {
    syslog(LOG_ERR,"setgains_btPID ignored because PID regulator is active");
  }
#endif

  btmutex_unlock(&(pid->mutex));
}
/** Set saturation of the regulator. If set anti-windup will kick in above this value. See btPID object of more info.
 
\bug verif anti-windup functionality
*/
void setsaturation_btPID(btPID *pid, btreal saturation)
{
  btmutex_lock(&(pid->mutex));
  
  pid->saturation = saturation;

  btmutex_unlock(&(pid->mutex));
}
/** Get the present gain values. */
void getgains_btPID(btPID *pid, btreal *Kp, btreal *Kd, btreal *Ki)
{
  btmutex_lock(&(pid->mutex));

  *Kp = pid->Kp;
  *Kd = pid->Kd;
  *Ki = pid->Ki;

  btmutex_unlock(&(pid->mutex));
}
/** Get the present input values */
void getinputs_btPID(btPID *pid, btreal *y, btreal *yref, btreal *dt)
{
  btmutex_lock(&(pid->mutex));

  *y = pid->y;
  *yref = pid->yref;
  *dt = pid->dt;

  btmutex_unlock(&(pid->mutex));
}
/** Get the present saturation values */
void getsaturation_btPID(btPID *pid, btreal *saturation)
{
  btmutex_lock(&(pid->mutex));

  *saturation = pid->saturation;

  btmutex_unlock(&(pid->mutex));
}
/************************* btPID interface functions ***************************/
/* Repackages the above PID routines for vectors
   Provides the default function plugins for the btstate controller (in btstatecontrol)
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

  for (cnt = 0; cnt < pids->elements; cnt++)
  {
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

  for (cnt = 0; cnt < pids->elements; cnt++)
  {
    start_btPID(&(pids->pid[cnt]));
  }
}

void btposition_interface_pause_btPID(struct btposition_interface_struct* btp)
{
  btPID_array *pids;
  int cnt;

  pids = (btPID_array*)btp->dat;

  for (cnt = 0; cnt < pids->elements; cnt++)
  {
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
  destroy_vr(&vr);
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
  else
    return 0;
}

int bttrajectory_interface_getstate_ct(struct bttrajectory_interface_struct *btt)
{
  ct_traj* ct;
  ct = (ct_traj*)btt->dat;

  if (done_ct(ct))
    return BTTRAJ_DONE;
  else
    return BTTRAJ_RUN;
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
  vect_n* ret;
  ct = (ct_traj*)btt->dat;
  ret = ds_pwl(ct->pwl,*(btt->dt));
  if (done_ct(ct))
    btt->state = BTTRAJ_DONE;
  return ret;
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

enum VT_SEG {VTS_IN_ACC = 0, VTS_IN_VEL};
/** Assumes trj->vr has been loaded with a valid vectray
 
 
*/
double start_via_trj(via_trj *trj,int col)
{
  double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next,ret;

  trj->idx = 0;
  trj->n = numrows_vr(trj->vr);
  trj->col = col+1;
  ret = getval_vn(idx_vr(trj->vr,0),trj->col); //force starting point to be our current point

  trj->last_cmd = 0.0;
  trj->segment = VTS_IN_ACC;//acc first

  Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
  Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
  trj->v_prev = 0;
  trj->v_next = Dq_next/Dt_next;
  CalcSegment(&(trj->seg),getval_vn(idx_vr(trj->vr,trj->idx),trj->col), getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col),
              getval_vn(idx_vr(trj->vr,trj->idx),0), getval_vn(idx_vr(trj->vr,trj->idx+1),0),trj->v_prev,trj->v_next,trj->trj_acc, 0);
  
  trj->acc = trj->seg.acc1;
  trj->dt_acc = trj->seg.dt_acc1;
  trj->v_prev = trj->seg.vel;
  trj->t=0;
  trj->t0=0;
  trj->t_acc = trj->dt_acc;
  trj->q0 = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
  trj->v0 = 0;
  trj->state = BTTRAJ_RUN;
  return ret;
}

void SetAcc_vt(via_trj *trj,double acc)
{
  trj->trj_acc = acc;
}
double eval_via_trj(via_trj *trj,double dt)
{
  double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next,acc_next,t_acc_next,end,et,cmd;
  double tn,tp,qn,qp;
  if (trj->state == BTTRAJ_RUN)
  {
    trj->t += dt;  //increment time

    if ((trj->segment == VTS_IN_ACC) && (trj->t > trj->t_acc))
    { //done with acc, set up vel
      if (trj->idx >= trj->n-1)
      {
        trj->state = BTTRAJ_STOPPED;
      }
      else
      {
        trj->q0 = trj->q0 + trj->v0*trj->dt_acc + 0.5*trj->dt_acc*trj->dt_acc*trj->acc;
        trj->vel = trj->seg.vel;
        
        trj->t0 = trj->t_acc;
        trj->dt_vel = trj->seg.dt_vel;
        trj->t_vel = trj->dt_vel + trj->t0;
        trj->v0 = trj->vel;
        trj->acc = 0.0;
        trj->segment = VTS_IN_VEL;
      }

    }
    else if((trj->segment == VTS_IN_VEL) && (trj->t > trj->t_vel))
    { //setup acc segment
      trj->idx++;
      if (trj->idx >= trj->n-1) //Setup final deceleration
      {
        trj->acc = trj->seg.acc2;
        trj->dt_acc = trj->seg.dt_acc2;
        trj->t0 = trj->t_vel;
        trj->t_acc = trj->dt_acc + trj->t0;
        trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
        trj->v0 = trj->vel;
        trj->segment = VTS_IN_ACC;//acc first
      }
      else
      {
        tp = getval_vn(idx_vr(trj->vr,trj->idx-1),0);
        tn = getval_vn(idx_vr(trj->vr,trj->idx),0);
        qn = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
        qp = getval_vn(idx_vr(trj->vr,trj->idx-1),trj->col);
        Dt = tn - tp;
        Dq = qn - qp;
        //trj->v_prev = Dq/Dt;

        if (trj->idx >= trj->n-2)
        {
          end = 2;
          trj->v_next = 0.0;
        }
        else
        {
          end = 1;
          Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
          Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
          trj->v_next = Dq_next/Dt_next;
        }
        syslog(LOG_ERR, "CalcSegment: Col %d",trj->col);
        CalcSegment(&(trj->seg),getval_vn(idx_vr(trj->vr,trj->idx),trj->col), getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col),
                    tn,getval_vn(idx_vr(trj->vr,trj->idx+1),0),trj->v_prev,trj->v_next,trj->trj_acc, end);
        
                    
        trj->segment = 0;//acc first
        trj->acc = trj->seg.acc1;
        trj->dt_acc = trj->seg.dt_acc1;
        trj->t0 = trj->t_vel;
        trj->t_acc = trj->dt_acc + trj->t0;
        trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
        trj->v0 = trj->vel;
        trj->v_prev = trj->seg.vel;
      }


    }



    if (trj->segment == 0)
    {//in acceleration segment
      et = trj->t - trj->t0;
      cmd = trj->q0 + trj->v0*et + 0.5*et*et*trj->acc;
    }
    else
    {
      et = trj->t - trj->t0; //elapsed time
      //cmd = trj->q0 + trj->vel*et;
      cmd = trj->q0 + trj->v0*et + 0.5*et*et*trj->acc;
    }
    trj->last_et = et;
    trj->last_cmd = cmd;
  }
  else{
    cmd = trj->last_cmd;
  }
  
  return cmd;

}
/**
 
  see Notebook TH#6 pp146,147
  
  \param seg_acc - Acceleration with which to blend linear segments. Expected to be positive;
*/
void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, double v_prev, double v_next, double seg_acc, int end)
{
  double dt,dq;
  double vel,acc1,acc2,dt_vel,dt_acc1,dt_acc2;
  double min_acc,use_acc,q_acc1,q_vel,q_acc2;


  dt = t2 - t1;
  dq = q2 - q1;
#if 0
  syslog(LOG_ERR, "CalcSeg: in: q1: %f q2: %f  t1: %f t2: %f",q1,q2,t1,t2);
  syslog(LOG_ERR, "CalcSeg: in: vprev: %f vnext: %f  acc: %f end: %d",v_prev,v_next,seg_acc,end);
#endif
  use_acc = fabs(seg_acc); //Force to positive
  
  if (end == 0) //Starting segment (Accelerate to the next point)
  {
    min_acc = 8*fabs(dq)/(3*dt*dt);

    if (use_acc < min_acc)
    { //accelleration must get us to vel_mode at halfway point
      use_acc = min_acc;
      syslog(LOG_ERR, "CalcSegment: Boosted acceleration to %f to match velocity change",use_acc);
    }
    if (use_acc != 0.0)
      dt_acc1 = dt - sqrt(dt*dt - 2*fabs(dq)/use_acc);
    else
      dt_acc1 = 0.0;
    acc1 = Sgn(dq)*use_acc;
    q_acc1 = q1 + 0.5*acc1*dt_acc1*dt_acc1;
    //vel = (q2 - q_acc1)/(dt - dt_acc1);
    vel = acc1*dt_acc1;
    acc2 = Sgn(v_next - vel)*seg_acc;
    dt_acc2 = (v_next - vel)/acc2;
    dt_vel = dt - dt_acc1 - dt_acc2/2.0;
    if (dt_vel < 0.0){
      dt_vel = 0.0; 
      
      syslog(LOG_ERR, "CalcSegment: Init Acc: Not enough acceleration!");
      syslog(LOG_ERR, "CalcSegment: Init Acc: dt:%f dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1,dt_acc2/2.0);
    }
  }
  else if (end == 1)
  {
    vel = dq/dt;
    acc1 = Sgn(vel - v_prev)*use_acc;
    if (fabs(vel - v_prev) > dt*fabs(acc1)/2){
      acc1 = (vel - v_prev)*2/dt;
      syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to match velocity change",acc1);

    }
    if (acc1 != 0.0){
      dt_acc1 = (vel - v_prev)/acc1;
    }
    else {
      dt_acc1 = 0.0;
    }
    q_acc1 = q1 + 0.5*acc1*dt_acc1*dt_acc1;
    
    acc2 = Sgn(v_next - vel)*use_acc;
    dt_acc2 = (v_next - vel)/acc2;
    dt_vel = dt - dt_acc1/2 - dt_acc2/2;
    if (dt_vel < 0.0){
      dt_vel = 0.0; 

      syslog(LOG_ERR, "CalcSegment: Mid segment: Not enough acceleration!");
    }
  }
  else //Ending segment (decelerate)
  {
    min_acc = 8.0*fabs(dq)/(3.0*dt*dt);

    if (use_acc < min_acc)
    { //accelleration must get us to vel_mode at halfway point
      use_acc = min_acc;
      syslog(LOG_ERR, "CalcSegment: Boosted acc:%f to match end velocity change %f %f",acc1,dq,dt);
      //syslog(LOG_ERR, "CalcSegment:  %f %f %f %f",q1,q2,t1,t2);
    }
    if (use_acc != 0.0){
      dt_acc2 = dt - sqrt(dt*dt - 2*fabs(dq)/use_acc);
    }
    else {
      dt_acc1 = 0.0;
    }
    acc2 = -1.0*Sgn(dq)*use_acc;
    //vel = (dq - 0.5*acc2*dt_acc2*dt_acc2)/(dt - dt_acc2);
    vel = -1.0*acc2*dt_acc2;
    acc1 = Sgn((vel - v_prev))*use_acc;
    if (fabs(vel - v_prev) > dt*fabs(acc1)/2){
      acc1 = (vel - v_prev)*2/dt;
      syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to match velocity change",acc1);
    }
    if (acc1 != 0.0){
      dt_acc1 = (vel - v_prev)/acc1;
    }
    else {
      dt_acc1 = 0.0;
    }
    
    dt_vel = dt - dt_acc2 - dt_acc1/2;
     if (dt_vel < 0.0){
      dt_vel = 0.0; 

      syslog(LOG_ERR, "CalcSegment: Final Acc: Not enough acceleration!");
      syslog(LOG_ERR, "CalcSegment: Final Acc: dt:%f dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1,dt_acc2/2.0);

    }
  }

  seg->vel = vel;
  seg->acc1 = acc1;
  seg->acc2 = acc2;
  seg->dt_vel = dt_vel;
  seg->dt_acc1 = dt_acc1;
  seg->dt_acc2 = dt_acc2;
#if 0
  syslog(LOG_ERR, "CalcSeg: Time: dt_acc1: %f dt_vel: %f dt_acc2: %f",dt_acc1,dt_vel,dt_acc2);
  syslog(LOG_ERR, "CalcSeg: val: acc1: %f vel: %f  acc2: %f",acc1,vel,acc2);
  syslog(LOG_ERR, "CalcSeg: ");
#endif
}
/** Internal function to allocate memory for a vta object*/
via_trj_array* malloc_new_vta(int num_columns)
{
  void *vmem;
  via_trj_array* vt;
  
  
  vmem = btmalloc(sizeof(via_trj_array) + (size_t)num_columns * sizeof(via_trj));
    
  vt = (via_trj_array*)vmem;
  vt->trj = vmem + sizeof(via_trj_array);
  vt->elements = num_columns;
  return vt;
}
via_trj_array* read_file_vta(char* filename,int extrapoints)
{
  via_trj_array* vt;
  vectray *vr;
  int cnt;
  if (read_csv_file_vr(filename,&vr) != 0) return NULL;
  vr = resize_vr(&vr,maxrows_vr(vr)+extrapoints);
  
  vt = malloc_new_vta(numelements_vr(vr)-1);
  for(cnt = 0;cnt < vt->elements;cnt++){
    SetAcc_vt(&(vt->trj[cnt]),1.0);
    vt->trj[cnt].vr = vr;
  }
  
  return vt;
}
void write_file_vta(via_trj_array* vt,char *filename)
{
  if (vt != NULL) 
    write_csv_file_vr(filename,vt->trj[0].vr);
}
vect_n* sim_vta(via_trj_array* vt,double dt,double duration,char*filename)
{
  FILE *out;
  char buff[250];
  double t = dt;
  local_vn(qref,7);
  init_vn(qref,7);
  
  out = fopen(filename,"w");
  reset_vta(vt,dt,qref);
  fprintf(out,"%8.4f, %s\n",t,sprint_csv_vn(buff,qref));
  
  while (t < duration){
    t += dt;
    eval_vta(vt,dt,qref);
    fprintf(out,"%8.4f, %s\n",t,sprint_csv_vn(buff,qref));
  }
  
  fclose(out);
}
/** set acceleration on corners of via trajectory */
void set_acc_vta(via_trj_array* vt,btreal acc)
{
  int cnt;
  if (vt != NULL)
  for(cnt = 0;cnt < vt->elements;cnt++){
    SetAcc_vt(&(vt->trj[cnt]),acc);
  }
}
/** Allocate a vta object and vectray objects */
via_trj_array* new_vta(int num_columns,int max_rows)
{
  via_trj_array* vt;
  vectray *vr;
  int cnt;
  vt = malloc_new_vta(num_columns);
  vr = new_vr(num_columns+1,max_rows);
  for(cnt = 0;cnt < vt->elements;cnt++){
    SetAcc_vt(&(vt->trj[cnt]),1.0);
    vt->trj[cnt].vr = vr;
  }
  vt->vel = 0.5;
  return vt;
}
/** Free memory allocated during new_vr()
*/
void destroy_vta(via_trj_array** vt)
{
  if (*vt != NULL){
    destroy_vr(&(*vt)->trj[0].vr);
    btfree((void**)vt);
  }
}
/**Increment the edit point by one.

If the edit point reaches the end of the list,
it stays there.
*/
void next_point_vta(via_trj_array* vt)
{
  if (vt != NULL) 
    next_vr(vt->trj[0].vr);
}
/**Decrement the edit point by one.

If the edit point reaches the beginning of the list,
it stays there.
*/
void prev_point_vta(via_trj_array* vt)
{
  if (vt != NULL)
    prev_vr(vt->trj[0].vr);
}
/** set the edit point to the begining of the list*/
void first_point_vta(via_trj_array* vt)
{
  if (vt != NULL)
    start_vr(vt->trj[0].vr);
}
/**Set the edit point to the end of the list*/
void last_point_vta(via_trj_array* vt)
{
  if (vt != NULL)
    end_vr(vt->trj[0].vr);
}
/** Insert a location into the teach & play list
at the edit point. To add a location to the end of the list, you
may move the edit point to the end.
*/
int ins_point_vta(via_trj_array* vt, vect_n *pt)
{
  int i;
  vectray *vr;
  btreal t,d;
  local_vn(tmp,10);
  local_vn(last,10);

  init_vn(tmp,len_vn(pt)+1);
  init_vn(last,len_vn(pt));
  
  if (vt == NULL) return -1;
  else {
    vr = vt->trj[0].vr;
    setrange_vn(tmp,pt,1,0,len_vn(pt));
    setval_vn(tmp,0,0.0);
    i = edit_point_vr(vr);
    if (i > 0){
      setrange_vn(last,idx_vr(vr,i-1),0,1,len_vn(pt));
      d = norm_vn(sub_vn(pt,last));
      t = getval_vn(idx_vr(vr,i-1),0);
      if (vt->vel > 0.0)
        setval_vn(tmp,0,t + d/vt->vel);
      else
        setval_vn(tmp,0,t + d/0.1);
    }
    return insert_vr(vt->trj[0].vr,tmp);
  }
}
/** Delete the location at the edit point from the
teach & play list.

*/
int del_point_vta(via_trj_array* vt)
{
  if (vt == NULL) return -1;
  else return delete_vr(vt->trj[0].vr);
}
/** Return the index of the present edit point*/
int get_current_idx_vta(via_trj_array* vt)
{
  if (vt == NULL) return -1;
  else return edit_point_vr(vt->trj[0].vr);
}
/** Set the index of the present edit point */
int set_current_idx_vta(via_trj_array* vt,int idx)
{
  if (vt == NULL) return -1;
  else return edit_at_vr(vt->trj[0].vr,idx);
}
void get_current_point_vta(via_trj_array* vt, vect_n *dest)
{
  vectray *vr;
  vr = vt->trj[0].vr;
  set_vn(dest,edit_vr(vr));
}


/** Get a pointer to the vectray object being used by this via trajectory
object */
vectray* get_vr_vta(via_trj_array* vt)
{
  if (vt == NULL) return NULL;
  else return vt->trj[0].vr;
}

/** Set the time values in a trajectory.

Adjust all the time points in a via point trajectory array
based on input velocity. Also sets the corner acceleration


\bug Need to include acceleration in the calculation.
*/
int scale_vta(via_trj_array* vt,double vel,double acc)
{
  btreal arclen = 0.0,thislen;
  vectray *vr;
  int cnt;
  vect_n* lval,*rval;
  
  if (vt == NULL) return -1;
  
  vt->vel = vel;
  set_acc_vta(vt,acc);
  
  vr = vt->trj[0].vr;
  setval_vn(idx_vr(vr,0),0,0.0);
  
  for(cnt = 1;cnt < numrows_vr(vr);cnt++){
    lval = lval_vr(vr,cnt-1);
    rval = rval_vr(vr,cnt);
    arclen += norm_vn(sub_vn(subset_vn(lval,1,lval->n),subset_vn(rval,1,rval->n)));
    reset_vn(lval);
    reset_vn(rval); /** \bug the reset_vn() function is a bit of a hack, is there a better way?*/
    setval_vn(idx_vr(vr,cnt),0,arclen/vel);
  }
  return 0; 
}
vect_n* reset_vta(via_trj_array* vt,double dt,vect_n* qref)
{
  double ret;
  int cnt;

  for (cnt = 0;cnt<vt->elements;cnt++)
  {
    setval_vn(qref,cnt,start_via_trj(&(vt->trj[cnt]),cnt));
  }
  return qref;
  
}
vect_n* eval_vta(via_trj_array* vt,double dt,vect_n* qref)
{
  double ret;
  int cnt;
  for (cnt = 0;cnt<vt->elements;cnt++)
  {
    setval_vn(qref,cnt,eval_via_trj(&(vt->trj[cnt]),dt));
  }
  return qref;
}

/** Implements the getstate interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
*/
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt)
{

  int cnt;
  int ret;
  via_trj_array* vt;
  vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
  if(!btptr_ok(vt,"bttrajectory_interface_getstate_vt")) 
    return  BTTRAJ_OFF;
#endif  
  ret = BTTRAJ_DONE;
  for (cnt = 0;cnt<vt->elements;cnt++)
  {
    if (vt->trj[cnt].state == BTTRAJ_RUN)
      ret = BTTRAJ_RUN;
  }
  return ret;
}
/** Implements the reset interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
*/vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt)
{
  double ret;
  int cnt;
  via_trj_array* vt;
  vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
  if(!btptr_ok(vt,"bttrajectory_interface_reset_vt")) 
    return btt->qref;
#endif  
  for (cnt = 0;cnt<vt->elements;cnt++)
  {
    setval_vn(btt->qref,cnt,start_via_trj(&(vt->trj[cnt]),cnt));
  }
  return btt->qref;
}

/** Implements the eval interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
*/
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt)
{
  double ret;
  int cnt;
  via_trj_array* vt;
  vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
  if(!btptr_ok(vt,"bttrajectory_interface_eval_vt")) 
    return btt->qref;
#endif   

 
  for (cnt = 0;cnt<vt->elements;cnt++)
  {
    setval_vn(btt->qref,cnt,eval_via_trj(&(vt->trj[cnt]),*(btt->dt)));
  }
  return btt->qref;
}
/** Registers the necessary data and function pointers with the 
btstatecontrol object */
void register_vta(btstatecontrol *sc,via_trj_array *vt)
{ 
#ifdef BT_NULL_PTR_GUARD
  if (!btptr_ok(sc,"register_vta")) exit(1);
  if (!btptr_ok(vt,"register_vta")) syslog(LOG_ERR,"register_vta: Setting btt.dat to NULL");
#endif   
  maptrajectory_bts(sc,(void*) vt,
                    bttrajectory_interface_reset_vt,
                    bttrajectory_interface_eval_vt,
                    bttrajectory_interface_getstate_vt);
}
#undef BT_DUMMY_PROOF



