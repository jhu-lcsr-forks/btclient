/*======================================================================*
 *  Module .............libbtwam
 *  File ...............btwamctl.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......15 Feb 2003
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file btwam.c
    Provides functions for controlling a WAM that has a CAN infrastructure.
 
    btwam assumes that you have a 7 DOF or 4 DOF wam that is being controlled
on a CAN bus. It provides some high-level functions to help people get up and
running quickly.
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <semaphore.h>
#include <syslog.h>
//th041217#include <sys/neutrino.h>
#include <inttypes.h>
#include <math.h>
#include <errno.h>
#ifdef USE_RTAI31
#include <rtai_lxrt.h>
#endif
#ifdef USE_FUSION
#include <rtai/task.h>
#include <rtai/timer.h>
#endif

#define TWOPI 6.283185
/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btjointcontrol.h"
#include "btcan.h"
#include "btsystem.h"
#include "control_loop.h"
#include "btwam.h"
#include "btrobot.h"

//#include "WAMDHKin.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
extern sem_t     timer_semaphore;
extern int shutdown_threads,sample_period2;

wam_struct WAM;

double  **trajList;
extern int gimbalsInit;


int isZeroed = FALSE;

/* Define WAM link parameters in meters */
#define L3 0.558
#define L4 0.291
#define Lb 0.291 //4-DOF blank link length (not correct)
#define L7 0.065
#define d3 0.04763
#define Gx 0.1524
#define Gy 0.4097

#define LOW_TRQ_COEFFICIENT (0.75)

/*==============================*
 * Functions                    *
 *==============================*/
/** Initialize a WAM
 
  This function calls the necessary lower-level libbt functions to set
    up the pucks, enumerate them and scan in the wam configuration information.
 
  \retval 0 Completed successfully
  \retval -1 Some error occured, Check syslog.
*/
int InitWAM(char *wamfile)
{
  int cnt,ret;
  const double pi = 3.14159;
  
  WAM.zero_offsets = new_vn(10);
  WAM.stop_torque = new_vn(10);
  WAM.park_location = new_vn(10);
  WAM.Mpos = new_vn(10);
  WAM.Mtrq  = new_vn(10);
  WAM.Jpos = new_vn(10);
  WAM.Jtrq = new_vn(10);
  WAM.Ttrq = new_vn(10);
  WAM.Kp = new_vn(10);
  WAM.Kd = new_vn(10);
  WAM.Ki = new_vn(10);
  WAM.saturation = new_vn(10);
  WAM.vel = new_vn(10);
  WAM.acc = new_vn(10);
  WAM.Cpos = new_v3();
  WAM.Cpoint = new_v3();
  WAM.Cref = new_v3();
  WAM.Cforce = new_v3();
  WAM.Ctrq = new_v3();
  WAM.Ckp = new_v3();
  WAM.Ckd = new_v3();
  WAM.Cki = new_v3();
  WAM.use_new = 0;
  
  new_bot(&WAM.robot,4);
  
  link_geom_bot(&WAM.robot,0,0.0,0.0,0.0,-pi/2.0);
  link_geom_bot(&WAM.robot,1,0.0,0.0,0.0,pi/2.0);
  link_geom_bot(&WAM.robot,2,0.0,0.550,0.045,-pi/2.0);
  link_geom_bot(&WAM.robot,3,0.0,0.0,-0.045,pi/2.0);
  link_geom_bot(&WAM.robot,4,0.0,0.3574,0.0,0.0);
  
  link_mass_bot(&WAM.robot,0,C_v3(0.0,0.1405,-0.0061),12.044);
  link_mass_bot(&WAM.robot,1,C_v3(0.0,-0.0166,0.0096),5.903);
  link_mass_bot(&WAM.robot,2,C_v3(-0.0443,0.2549,0.0),2.08);
  link_mass_bot(&WAM.robot,3,C_v3(0.01465,0.0,0.1308),1.135);
  link_mass_bot(&WAM.robot,4,C_v3(0.0,0.0,0.03),2.000);
  //init_wam_btrobot(&(WAM.robot));
  fill_vn(WAM.robot.dq,0.0);
  fill_vn(WAM.robot.ddq,0.0);
  
  init_pwl(&WAM.pth,3,2); //Cartesian move path
  init_btPID(&(WAM.pid[0]));
  init_btPID(&(WAM.pid[1]));
  init_btPID(&(WAM.pid[2]));
  WAM.F = 0.0;
  
  
  if(test_and_log(
    InitializeSystem("actuators.dat","buses.dat","motors.dat","pucks.dat"),"Failed to initialize system"))
    {return -1;}
    
  SetEngrUnits(1);
  if(test_and_log(
    EnumerateSystem(),"Failed to enumerate system"))  {return -1;}

  //------------------------
  WAM.act = GetActuators(&(WAM.num_actuators));

  if(test_and_log(
    LoadWAM(wamfile),"Failed to load wam config file")) {return -1;}

  WAM.Gcomp = 0;

  for(cnt = 0; cnt < 7; cnt++)
  {
    SCinit(&(WAM.sc[cnt]));
    SCsetpid(&(WAM.sc[cnt]),getval_vn(WAM.Kp,cnt),getval_vn(WAM.Kd,cnt),getval_vn(WAM.Ki,cnt),getval_vn(WAM.saturation,cnt));
    SCsettrjprof(&(WAM.sc[cnt]),getval_vn(WAM.vel,cnt),getval_vn(WAM.acc,cnt));
  }
     
  test_and_log(
      pthread_mutex_init(&(WAM.loop_mutex),NULL),
      "Could not initialize mutex for WAM control loop.");
      
  return 0;
}

void CloseWAM()
{
  CloseSystem();
}

/** Returns a pointer to the SimpleCtl array used by the WAM functions
*/
SimpleCtl * GetWAMsc()
{
  return WAM.sc;
}
/** 
 Actuator Idx starts at 0 and counts up to number of actuators. 
 MotorID is keyed to the location on the wam (and hense the joint it drives)
 This relationship allows only calculating the control info for the joints that
 are in use.
*/
BTINLINE int MotorID_From_ActIdx(int idx)
{
  return WAM.motor_position[idx];
}
/** Returns a pointer to the WAM structure
*/
wam_struct * GetWAM()
{
  return &WAM;
}

/** Load wam configuration data
 
\verbatim
wam file format
<motor_position> x7  //matches actuator indexes to motor indexes. See Actangle2Mpos
<Zero0> x7
<StopTorque> x7
<Park0>x7
<Order> x7
<Kp> x7
<Kd> x7
<Ki> x7
<saturation> x7
<Vel> x7
<Acc> x7
\endverbatim
*/
int LoadWAM(char *wamfile)
{
  FILE *in;
  if ((in = fopen(wamfile, "r")) == NULL)
  {
    syslog(LOG_ERR, "Could not open actuator data file: %s", wamfile);
    return -1;
  }

  
  fscanf(in,"%d, %d, %d, %d, %d, %d, %d",&(WAM.motor_position[0]),&(WAM.motor_position[1]),&(WAM.motor_position[2]),&(WAM.motor_position[3]),&(WAM.motor_position[4]),&(WAM.motor_position[5]),&(WAM.motor_position[6]));
  read_wam_vector(in,(WAM.zero_offsets));
  read_wam_vector(in,(WAM.stop_torque));
  read_wam_vector(in,(WAM.park_location));
  fscanf(in,"%d, %d, %d, %d, %d, %d, %d",&(WAM.zero_order[0]),&(WAM.zero_order[1]),&(WAM.zero_order[2]),&(WAM.zero_order[3]),&(WAM.zero_order[4]),&(WAM.zero_order[5]),&(WAM.zero_order[6]));
  read_wam_vector(in,(WAM.Kp));
  read_wam_vector(in,(WAM.Kd));
  read_wam_vector(in,(WAM.Ki));
  read_wam_vector(in,(WAM.saturation));
  read_wam_vector(in,(WAM.vel));
  read_wam_vector(in,(WAM.acc));
  fclose(in);
  return 0;
}
/** Write WAM configuration information to a file.
*/
void SaveWAM(char *wamfile)
{
  FILE *out;
  if ((out = fopen(wamfile, "w")) == NULL)
  {
    syslog(LOG_ERR, "Could not open actuator data file: %s", wamfile);
    return;
  }

  
  fprintf(out,"%d, %d, %d, %d, %d, %d, %d\n",(WAM.motor_position[0]),(WAM.motor_position[1]),(WAM.motor_position[2]),(WAM.motor_position[3]),(WAM.motor_position[4]),(WAM.motor_position[5]),(WAM.motor_position[6]));
  write_wam_vector(out,(WAM.zero_offsets));
  write_wam_vector(out,(WAM.stop_torque));
  write_wam_vector(out,(WAM.park_location));
  fprintf(out,"%d, %d, %d, %d, %d, %d, %d\n",(WAM.zero_order[0]),(WAM.zero_order[1]),(WAM.zero_order[2]),(WAM.zero_order[3]),(WAM.zero_order[4]),(WAM.zero_order[5]),(WAM.zero_order[6]));
  write_wam_vector(out,(WAM.Kp));
  write_wam_vector(out,(WAM.Kd));
  write_wam_vector(out,(WAM.Ki));
  write_wam_vector(out,(WAM.saturation));
  write_wam_vector(out,(WAM.vel));
  write_wam_vector(out,(WAM.acc));
  fclose(out);
}
/** Syslog wam configuration information for debugging
*/
void DumpWAM2Syslog()
{
  syslog(LOG_ERR,"Dump of WAM data---------------------------");
  
  dump_wam_vector((WAM.zero_offsets));
  dump_wam_vector((WAM.stop_torque));
  dump_wam_vector((WAM.park_location));
  syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(WAM.zero_order[0]),(WAM.zero_order[1]),(WAM.zero_order[2]),(WAM.zero_order[3]),(WAM.zero_order[4]),(WAM.zero_order[5]),(WAM.zero_order[6]));
  syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(WAM.motor_position[0]),(WAM.motor_position[1]),(WAM.motor_position[2]),(WAM.motor_position[3]),(WAM.motor_position[4]),(WAM.motor_position[5]),(WAM.motor_position[6]));
  dump_wam_vector((WAM.Kp));
  dump_wam_vector((WAM.Kd));
  dump_wam_vector((WAM.Ki));
  dump_wam_vector((WAM.saturation));
  dump_wam_vector((WAM.vel));
  dump_wam_vector((WAM.acc));
  syslog(LOG_ERR,"Num pucks %d",WAM.num_actuators);
  syslog(LOG_ERR,"End dump of WAM data-----------------------------");
}

/** This function closes the control loop on the WAM using a PID regulator in joint space.
*/

void WAMControlThread(void *data)
{
  int                 i;
  int                 cnt;
  //    wam_vector          Gtrq;
  int                 idx;
  int Mid;
  //DoubleBuffer_struct *db;
  //data_to_log         *dat;
  double              dt = 0.002;
  double              newCmd,newYref;
  int                 err;
  long unsigned       counter = 0;
  int                 doTE;
  int period;
  RTIME last_loop,loop_start,loop_end,user_start,user_end,pos1_time,pos2_time,trq1_time,trq2_time;
  RT_TASK *WAMControlThreadTask;

  WAMControlThreadTask = rt_task_init(nam2num("WAMCon"), 0, 0, 0);

  rt_make_hard_real_time();
  syslog(LOG_ERR,"WAMControl initial hard");
  rt_task_make_periodic_relative_ns(WAMControlThreadTask, sample_period2, sample_period2);
  syslog(LOG_ERR,"WAMControl periodic %d", sample_period2);
  while (!shutdown_threads)
  {
    rt_task_wait_period();
    counter++;
    
    loop_start = rt_get_cpu_time_ns();
    WAM.loop_period = loop_start - last_loop;
    
    test_and_log(
      pthread_mutex_lock(&(WAM.loop_mutex)),"WAMControlThread lock mutex failed");
    
    
    rt_make_soft_real_time();
    
    pos1_time = rt_get_cpu_time_ns();
    
    GetPositions();
    
    pos2_time = rt_get_cpu_time_ns();
    WAM.readpos_time = pos2_time - pos1_time;
    
    rt_make_hard_real_time();
    ActAngle2Mpos((WAM.Mpos)); //Move motor angles into a wam_vector variable
    Mpos2Jpos((WAM.Mpos), (WAM.Jpos)); //Convert from motor angles to joint angles
    // Joint space stuff

    for (cnt = 0; cnt < WAM.num_actuators; cnt++) //Calculate control torque for each joint.
        {
            Mid = MotorID_From_ActIdx(cnt); //idx = joint we are controlling
            setval_vn(WAM.Jtrq,Mid,SCevaluate(&(WAM.sc[Mid]),getval_vn(WAM.Jpos,Mid), dt));
        }
    
    // Cartesian space stuff
    set_vn(WAM.robot.q,WAM.Jpos);

    eval_fk_bot(&WAM.robot);
    eval_fd_bot(&WAM.robot);
    set_v3(WAM.Cpos,Ln_to_W_bot(&WAM.robot,4,WAM.Cpoint));
    if (WAM.trj.state){
      WAM.F = evaluate_traptrj(&WAM.trj,dt);
      set_vn((vect_n*)WAM.Cref,getval_pwl(&WAM.pth, WAM.F));
    }
    for (cnt = 0; cnt < 3; cnt ++){
      setval_v3(WAM.Cforce,cnt,eval_btPID(&(WAM.pid[cnt]),getval_v3(WAM.Cpos,cnt), getval_v3(WAM.Cref,cnt), dt));
    }
    
    
    apply_force_bot(&WAM.robot,4, WAM.Cpoint, WAM.Cforce, C_v3(0.0,0.0,0.0));
    eval_bd_bot(&WAM.robot);
    get_t_bot(&WAM.robot,WAM.Ttrq);

    if(isZeroed)
    {
        set_vn(WAM.Jtrq,add_vn(WAM.Jtrq,WAM.Ttrq));
    }
    
    Jtrq2Mtrq((WAM.Jtrq), (WAM.Mtrq));  //Convert from joint torques to motor torques
    Mtrq2ActTrq(WAM.Mtrq); //Move motor torques from wam_vector variable into actuator database
    
    rt_make_soft_real_time();
    
    trq1_time = rt_get_cpu_time_ns();
    
    SetTorques();
    
    trq2_time = rt_get_cpu_time_ns();
    WAM.writetrq_time = trq2_time - trq1_time;
    
    rt_make_hard_real_time();
    
    loop_end = rt_get_cpu_time_ns();
    WAM.loop_time = loop_end - loop_start;
    last_loop = loop_start;
    
    pthread_mutex_unlock(&(WAM.loop_mutex));
    TriggerDL(&(WAM.log));
  }
  syslog(LOG_ERR, "WAM Control Thread: exiting");
  rt_make_soft_real_time();
  rt_task_delete(WAMControlThreadTask);
  pthread_exit(NULL);
}


/** Load a wam_vector with the actuator angles.
 ActAngle2Mpos loads a wam_vector with the actuator angles.
  \param *Mpos Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int ActAngle2Mpos(vect_n *Mpos) //Extracts actuators 0 thru 6 into wam_vector positions
{
  int num_act, cnt;

  num_act = WAM.num_actuators;
  for (cnt = 0; cnt < num_act; cnt++)
  {
    setval_vn(Mpos,WAM.motor_position[cnt],WAM.act[cnt].angle);
  }

  return 0;
}
/** Load the actuator database torques with the values from a wam_vector.
 Mtrq2ActTrq loads the actuator database torques with the values from a wam_vector.
  \param *Mtrq Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int Mtrq2ActTrq(vect_n *Mtrq) //Packs wam_vector of torques into actuator array
{
  actuator_struct *act;
  int num_act, cnt;

  act = GetActuators(&num_act);
  for (cnt = 0; cnt < num_act; cnt++)
  {
    act[cnt].torque = getval_vn(Mtrq,WAM.motor_position[cnt]);
  }

  return 0;
}
//=================================================working code
/** Transform wam_vector Motor positions to Joint positions
*/
void Mpos2Jpos(vect_n * Mpos, vect_n * Jpos) //convert motor angle to joint angle
{
  btreal pos[10];
  extract_vn(pos,Mpos);
  setval_vn(Jpos,0, ( -pos[0] / mN1));
  setval_vn(Jpos,1, (0.5 * pos[1] / mN2) - (0.5 * pos[2] / mN3));
  setval_vn(Jpos,2, ( -0.5 * pos[1] * mn3 / mN2) - (0.5 * mn3 * pos[2] / mN3));
  setval_vn(Jpos,3,  (pos[3] / mN4));
  setval_vn(Jpos,4, 0.5 * pos[4] / mN5 + 0.5 * pos[5] / mN5);
  setval_vn(Jpos,5, -0.5 * pos[4] * mn6 / mN5 + 0.5 * mn6 * pos[5] / mN5);
  setval_vn(Jpos,6, -1 * pos[6] / mN7);
}
//=================================================working code
/** Transform wam_vector Joint positions to Motor positions
*/
void Jpos2Mpos(vect_n * Jpos, vect_n * Mpos) //convert motor angle to joint angle
{
  btreal pos[10];
  extract_vn(pos,Jpos);
  setval_vn(Mpos,0, ( -pos[0] * mN1));
  setval_vn(Mpos,1, (pos[1] * mN2) - (pos[2] * mN3 / mn3));
  setval_vn(Mpos,2, ( -pos[1] * mN2) - ( pos[2] * mN3 / mn3));
  setval_vn(Mpos,3, (pos[3] * mN4));
  setval_vn(Mpos,4,pos[4] * mN5 - pos[5] * mN5 / mn6);
  setval_vn(Mpos,5, pos[4] * mN5 + pos[5] * mN5 / mn6);
  setval_vn(Mpos,6, -pos[6] / mN7);
}
/** Transform wam_vector Joint torques to Motor torques
*/
void Jtrq2Mtrq(vect_n * Jtrq, vect_n * Mtrq) //conbert joint torque to motor torque
{
  btreal trq[10];
  extract_vn(trq,Jtrq);
  setval_vn(Mtrq,0, ( -trq[0] / mN1));
  setval_vn(Mtrq,1, (0.5 * trq[1] / mN2) - (0.5 * trq[2] *mn3/ mN3));
  setval_vn(Mtrq,2, ( -0.5 * trq[1]  / mN2) - (0.5 * mn3 * trq[2] / mN3));
  setval_vn(Mtrq,3,  (trq[3] / mN4));
  setval_vn(Mtrq,4, 0.5 * trq[4] / mN5 - 0.5 * mn6 * trq[5] / mN5);
  setval_vn(Mtrq,5, 0.5 * trq[4] / mN5 + 0.5 * mn6 * trq[5] / mN5);
  setval_vn(Mtrq,6, -1 * trq[6] / mN7);
}


/** Reset the position sensors on the pucks to the passed in position in units of joint-space radians
*/
void SetWAMpos(vect_n *wv)
{
  int cnt;
  long tmp;
  vect_n *motor_angle;
  double result;
  
  motor_angle = new_vn(10);
  /* Tell the safety logic to ignore the next faults */
  SetByID(SAFETY_MODULE, IFAULT, 8);

  //convert from joint space to motor space, then from motor radians to encoder counts
  Jpos2Mpos(wv,motor_angle);
  for (cnt = 0;cnt < (WAM.num_actuators - 3 * gimbalsInit);cnt++)
  {
    if(getval_vn(motor_angle,WAM.motor_position[cnt]) == 0.0)
      result = 0;
    else
      result =( WAM.act[cnt].motor.counts_per_rev / TWOPI)* getval_vn(motor_angle,WAM.motor_position[cnt]);
    tmp = floor(result);
    //syslog(LOG_ERR,"ctsperrev = %d, motor_angle=%lf", WAM.act[cnt].motor.counts_per_rev, motor_angle.q[WAM.motor_position[cnt]]);
    //syslog(LOG_ERR,"tmp = %ld, result=%lf", tmp, result);
    SetProp(cnt,AP,tmp);
    SCevaluate(&(WAM.sc[WAM.motor_position[cnt]]),getval_vn(wv,cnt),0);
    SCsetmode(&(WAM.sc[WAM.motor_position[cnt]]),WAM.sc[WAM.motor_position[cnt]].mode);
    usleep(1000);
  }

  /* Tell the safety logic start monitoring tip velocity */
  SetByID(SAFETY_MODULE, ZERO, 1); /* 0 = Joint velocity, 1 = Tip velocity */

  isZeroed = TRUE;
}
/** Perform a coordinated move of the wam
*/
void MoveWAM(vect_n *pos)
{
  int cnt,ctr,idx,done = 0,count =0;

  for (cnt = 0;cnt < WAM.num_actuators;cnt++)
  {
       SCstarttrj(&(WAM.sc[WAM.motor_position[cnt]]),getval_vn(pos,WAM.motor_position[cnt]));
  }
  while (!done)
  {
    count++;
    ctr = 0;
    for (cnt = 0;cnt < WAM.num_actuators;cnt++)
    {
      ctr+=WAM.sc[WAM.motor_position[cnt]].trj.state;
    }
    if (ctr == 0)
      done = 1;
    usleep(1000);
    //if ((count % 1000) == 0)  syslog(LOG_ERR,"waited 1 second to reach position");
  }
}
/** Set the velocity and acceleration of a wam move
*/
void MovePropsWAM(vect_n *vel, vect_n *acc)
{
  int cnt,ctr,idx,done = 0,count =0;

  for (cnt = 0;cnt < WAM.num_actuators;cnt++)
  {
    SCsettrjprof(&(WAM.sc[WAM.motor_position[cnt]]),getval_vn(vel,WAM.motor_position[cnt]),getval_vn(acc,WAM.motor_position[cnt]));
  }
}
/** Perform a Cartesian move of the wam
*/
void CartesianMoveWAM(vect_n *pos, btreal vel, btreal acc)
{
  int cnt,ctr,idx,done = 0,count =0;

  //
  setprofile_traptrj(&WAM.trj,  vel, acc);
  clear_pwl(&WAM.pth);
  add_arclen_point_pwl(&WAM.pth,(vect_n*)WAM.Cref);
  add_arclen_point_pwl(&WAM.pth,pos);
  start_traptrj(&WAM.trj, arclength_pwl(&WAM.pth));
  
  while (!done)
  {
    if (WAM.trj.state == 0)
      done = 1;
    usleep(50000);
    //if ((count % 1000) == 0)  syslog(LOG_ERR,"waited 1 second to reach position");
  }
}

/** Perform a Cartesian move of the wam
*/
void ToolCartesianMoveWAM(vect_n *pos, btreal vel, btreal acc)
{
  int cnt,ctr,idx,done = 0,count =0;

  //
  setprofile_traptrj(&WAM.trj,  vel, acc);
  clear_pwl(&WAM.pth);
  add_arclen_point_pwl(&WAM.pth,(vect_n*)WAM.Cref);
  add_arclen_point_pwl(&WAM.pth,(vect_n*)T_to_W_bot(&WAM.robot,(vect_3*)pos));
  start_traptrj(&WAM.trj, arclength_pwl(&WAM.pth));
  
  while (!done)
  {
    if (WAM.trj.state == 0)
      done = 1;
    usleep(50000);
    //if ((count % 1000) == 0)  syslog(LOG_ERR,"waited 1 second to reach position");
  }
}
/** Find the center of a joint range and move to the zero_offsets position relative to the center
 
\bug Add code to check the puck for IsZerod so that we
know whether we have to zero or not...
*/

/** Move the WAM to the park position defined in the wam data file
*/
void ParkWAM()
{
  MovePropsWAM(scale_vn(0.2,WAM.vel),scale_vn(0.2,WAM.acc));
  MoveWAM(WAM.park_location);
}


/** fscanf a wam_vector from the file pointed to by FILE *in
*/
int read_wam_vector(FILE *in,vect_n *wv)
{
  int ret;
  btreal inreal[10];
  if (wv == NULL) syslog(LOG_ERR,"btmath: read_wam_vector: wv is NULL");
  fill_vn(wv,0.0);
  extract_vn(inreal,wv);
  ret = fscanf(in,"%lf, %lf, %lf, %lf, %lf, %lf, %lf",&(inreal[0]),&(inreal[1]),&(inreal[2]),&(inreal[3]),&(inreal[4]),&(inreal[5]),&(inreal[6]));
   
  if (ret == EOF)
    syslog(LOG_ERR,"read wam vector returned EOF");
  
  inject_vn(wv,inreal);
  return ret;
}

/** fprintf a wam_vector to FILE *out
*/
int write_wam_vector(FILE *out,vect_n *wv)
{

  fprintf(out,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",getval_vn(wv,0),getval_vn(wv,1),getval_vn(wv,2),getval_vn(wv,3),getval_vn(wv,4),getval_vn(wv,5),getval_vn(wv,6));
  return 0;
}
/** Syslog a wam_vector
*/
int dump_wam_vector(vect_n *wv)
{
  syslog(LOG_ERR,"%lf, %lf, %lf, %lf, %lf, %lf, %lf",getval_vn(wv,0),getval_vn(wv,1),getval_vn(wv,2),getval_vn(wv,3),getval_vn(wv,4),getval_vn(wv,5),getval_vn(wv,6));
  return 0;
}
/** Copies the internal state (position & torque of motors & joints) to user provided wam_vector
*/
void getWAMjoints(vect_n *Mpos,vect_n *Mtrq,vect_n *Jpos,vect_n *Jtrq)
{
  set_vn(Mpos,WAM.Mpos);
  set_vn(Mtrq,WAM.Mtrq);
  set_vn(Jpos,WAM.Jpos);
  set_vn(Jtrq,WAM.Jtrq );
}
/** Get data the matches actuator index to wam motor index
*/
void getWAMmotor_position(int *mp)
{
  int cnt;
  for(cnt = 0; cnt < 7; cnt++)
  {
    mp[cnt] = WAM.motor_position[cnt];
  }
}

int getGcomp()
{
  return WAM.Gcomp;
}
/** Enable or disable the gravity compensation calculation
 */
void setGcomp( int onoff)
{
  WAM.Gcomp = onoff;

}
/** Toggle the gravity compensation calculation
 */
void toggleGcomp()
{
  WAM.Gcomp = !WAM.Gcomp;
  if(WAM.Gcomp)
	  set_gravity_bot(&WAM.robot, 1.0);
  else
	  set_gravity_bot(&WAM.robot, 0.0);
}

void setSafetyLimits(double jointVel, double tipVel, double elbowVel)
{
  int conversion;
  syslog(LOG_ERR, "About to set safety limits, VL2 = %d", VL2);
  if((jointVel > 0) && (jointVel < 7)) /* If the vel (rad/s) is reasonable */
  {
    conversion = jointVel * 0x1000; /* Convert to Q4.12 rad/s */
    SetByID(SAFETY_MODULE, VL2, conversion);
  }

  if((tipVel > 0) && (tipVel < 7)) /* If the vel (m/s) is reasonable */
  {
    conversion = tipVel * 0x1000; /* Convert to Q4.12 rad/s */
    SetByID(SAFETY_MODULE, VL2, conversion);
  }

  if((elbowVel > 0) && (elbowVel < 7)) /* If the vel (m/s) is reasonable */
  {
    conversion = elbowVel * 0x1000; /* Convert to Q4.12 rad/s */
    SetByID(SAFETY_MODULE, VL2, conversion);
  }

}
/*
int playViaTrajectoryFile(char *fileName, double timeScale)
{
  //double  **trajList;
  int     err;
  int     rows, columns;
  int     i, cnt, idx;
  char outbuff[100];
  vect_n  *jointAngle, *vel, *acc;

  jointAngle = new_vn(10);
  vel  = new_vn(10);
  acc  = new_vn(10);
  
  //Initialize the trajectory
  err = init_traj_file(fileName, &trajList, &rows, &columns);
  if(err)
    return(err);
  syslog(LOG_ERR, "After init_traj_file, rows %d, columns %d", rows, columns);
  //Go to initial position with a preset vel and acc
  for(i = 0; i < columns-1; i++)
  {
    syslog(LOG_ERR, "Started loading %d", i);
    setval_vn(jointAngle,i,trajList[0][i+1]);
    setval_vn(vel,i,1.0);
    setval_vn(acc,i, 0.5);
    syslog(LOG_ERR, "Finished loading %d", i);
  }
  syslog(LOG_ERR, "Set initial jointAngle: %s",sprint_vn(outbuff,jointAngle));
  MovePropsWAM(vel, acc);
  MoveWAM(jointAngle);
  for (cnt = 0;cnt < WAM.num_actuators;cnt++)
  {
    idx = WAM.motor_position[cnt]; //idx = joint we are controlling
    SCstartViaTrjFile( &(WAM.sc[idx]), &trajList, rows, idx,timeScale);
  }

}
*/
/*======================================================================*
 *                                                                      *
 *             Copyright (c) 2003 Barrett Technology, Inc.              *
 *                        139 Main Street                               *
 *                       Kendall/MIT Square                             *
 *                  Cambridge, MA  02142-1528  USA                      *
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
 *  ******************************************************************  *
 *
 * CVS info: $Id$
 * CVS automatic log (prune as desired):
 * $Log$
 *
 *======================================================================*/
