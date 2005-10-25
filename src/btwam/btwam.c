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


/* Tags - 
This is a list of tags for functionality integrated into the InitWAM and WAMControlThread code
to make it easier to find and maintaind later

  //&cteach = Continuous teach & play
  //&prof = Loop time profiling
  //&log = Data logging

*/
#define TWOPI 6.283185
/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <stdio.h>
#include <math.h>


/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/


wam_struct WAM;
extern int gimbalsInit;
extern bus_struct *buses;

#define LOW_TRQ_COEFFICIENT (0.75)
//#define BTDOUBLETIME

/*==============================*
 * Internal use Functions       *
 *==============================*/
 
int ActAngle2Mpos(vect_n *Mpos); //Extracts actuators 0 thru 6 into vect_n positions
int Mtrq2ActTrq(vect_n *Mtrq);  //Packs vect_n of torques into actuator array
void Mpos2Jpos(vect_n * Mpos, vect_n * Jpos); //convert motor angle to joint angle
void Jpos2Mpos(vect_n * Jpos, vect_n * Mpos);
void Jtrq2Mtrq(vect_n * Jtrq, vect_n * Mtrq); //conbert joint torque to motor torque

void GetJointPositions();
void SetJointTorques();

int read_wam_vector(FILE *in,vect_n *wv);
int write_wam_vector(FILE *out,vect_n *wv);
int dump_wam_vector(vect_n *wv);

SimpleCtl * GetWAMsc();

int LoadWAM(char *wamfile);
void SaveWAM(char *wamfile);
void DumpWAM2Syslog();

/*==============================*
 * Functions                    *
 *==============================*/

void InitVectors(void){
  WAM.zero_offsets = new_vn(7);
  WAM.stop_torque = new_vn(7);
  WAM.park_location = new_vn(7);
  WAM.Mpos = new_vn(7);
  WAM.Mtrq  = new_vn(7);
  WAM.Jpos = new_vn(7);
  WAM.Jvel = new_vn(7);
  WAM.Jacc = new_vn(7);
  WAM.Jref = new_vn(7);
  WAM.Jtrq = new_vn(7);
  WAM.Ttrq = new_vn(7);
  WAM.Kp = new_vn(7);
  WAM.Kd = new_vn(7);
  WAM.Ki = new_vn(7);
  WAM.saturation = new_vn(7);
  WAM.vel = new_vn(7);
  WAM.acc = new_vn(7);
  WAM.Cpos = new_v3();
  WAM.Cpoint = new_v3();
  WAM.Cref = new_v3();
  WAM.Cforce = new_v3();
  WAM.Ctrq = new_v3();
  WAM.Ckp = new_v3();
  WAM.Ckd = new_v3();
  WAM.Cki = new_v3();
  WAM.use_new = 0;
  WAM.qref = new_q();
  WAM.qact = new_q();
  WAM.qaxis = new_q();
  WAM.forced = new_q();
  WAM.qerr = 0;
  WAM.R6pos = new_vn(6);
  WAM.R6ref = new_vn(6);
  WAM.R6vel = new_vn(6);
  WAM.R6acc = new_vn(6);
  WAM.R6trq = new_vn(6);
  WAM.R6force = new_vn(6);
}
 
/** Initialize a WAM
 
  This function calls the necessary lower-level libbt functions to set
    up the pucks, enumerate them and scan in the wam configuration information.
 
  \retval 0 Completed successfully
  \retval -1 Some error occured, Check syslog.
*/
wam_struct* OpenWAM(char *fn)
{
  int cnt,ret,err;
  const double pi = 3.14159;
  
  // Allocate memory for the WAM vectors
  InitVectors();
  
  /* Joint Control plugin initialization */
  for (cnt = 0; cnt < 7; cnt ++){
    init_btPID(&(WAM.d_jpos_ctl[cnt]));
  }
  WAM.d_jpos_array.pid = WAM.d_jpos_ctl;
  WAM.d_jpos_array.elements = 7;  
  init_bts(&WAM.Jsc);
  map_btstatecontrol(&WAM.Jsc, WAM.Jpos, WAM.Jvel, WAM.Jacc, 
                      WAM.Jref, WAM.Jtrq, &WAM.dt);
  btposition_interface_mapf_btPID(&WAM.Jsc, &(WAM.d_jpos_array));
  /* Joint Control plugin initialization */
  
  
  /* Control plugin initialization */
  for (cnt = 0; cnt < 3; cnt ++){
    init_btPID(&(WAM.d_pos_ctl[cnt]));
    setgains_btPID(&(WAM.d_pos_ctl[cnt]),2000.0,5.0,0.0);
  }
  WAM.d_pos_array.pid = WAM.d_pos_ctl;
  WAM.d_pos_array.elements = 6;  
  init_bts(&WAM.Csc);
  map_btstatecontrol(&WAM.Csc, WAM.R6pos, WAM.R6vel, WAM.R6acc, 
                      WAM.R6ref, WAM.R6force, &WAM.dt);
  btposition_interface_mapf_btPID(&WAM.Csc, &(WAM.d_pos_array));
  
  /* Control plugin initialization */
  
  init_pwl(&WAM.pth,3,2); //Cartesian move path

  for (cnt = 0; cnt < 3; cnt ++){
    init_btPID(&(WAM.pid[cnt]));
    setgains_btPID(&(WAM.pid[cnt]),2000.0,5.0,0.0);
  }
  setgains_btPID(&(WAM.pid[3]),8.0,0.1,0.0);
  //setgains_btPID(&(WAM.pid[3]),60.0,0.10,0.0);
  init_err_btPID(&(WAM.pid[3]));


  WAM.F = 0.0;
  WAM.isZeroed = FALSE;
  WAM.cteach.Log_Data = 0; //&cteach
  WAM.force_callback = BlankWAMcallback;
  WAM.log_time = 0.0;
  WAM.logdivider = 1;
  
    
  SetEngrUnits(1);
#ifdef BTOLDCONFIG
  if(test_and_log(
       InitializeSystem("actuators.dat","buses.dat","motors.dat","pucks.dat"),
       "Failed to initialize system"))
  {
    exit(-1);
  }
  atexit((void*)CloseSystem);//register CloseSystem for shutdown
  if(test_and_log(
    EnumerateSystem(),"Failed to enumerate system"))  {exit(-1);}

#else //BTOLDCONFIG
  //------------------------
  err = InitializeSystem();
  if(err){
      syslog(LOG_ERR, "OpenWAM: InitializeSystem returned err = %d", err);
      return(NULL);
  }
#endif
  WAM.act = GetActuators(&(WAM.num_actuators));

  new_bot(&WAM.robot,WAM.num_actuators);
  
  btreal theta, d, a, alpha, mass, tmpdbl;
  vect_3 *com;
  
  char robotType[256];
  char key[256];
  long reply;
  int link;
  com = new_v3(); 
  
  
  // Parse config file
  err = parseFile(fn);
  if(err){
      syslog(LOG_ERR, "OpenWAM: Error parsing config file- %s", fn);
      return(NULL);
  }
  //strcpy(robotType, buses[0].device_name);
  sprintf(key, "system.bus[0].name");
  parseGetVal(STRING, key, (void*)robotType);
  
  
  // Read park_location
  sprintf(key, "%s.home", robotType, link);
  parseGetVal(VECTOR, key, (void*)WAM.park_location);
      
  for(link = 0; link <= WAM.num_actuators; link++){
      // Get the DH parameters
      sprintf(key, "%s.link[%d].dh.theta", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&theta);
      sprintf(key, "%s.link[%d].dh.d", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&d);
      sprintf(key, "%s.link[%d].dh.a", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&a);
      sprintf(key, "%s.link[%d].dh.alpha", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&alpha);
      
      // Get the mass parameters
      sprintf(key, "%s.link[%d].com", robotType, link);
      parseGetVal(VECTOR, key, (void*)com);
      sprintf(key, "%s.link[%d].mass", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&mass);

      if(link != WAM.num_actuators){
          // Query for motor_position (JIDX)
#ifdef BTOLDCONFIG 
          WAM.motor_position[link] = link;
#else
          getProperty(WAM.act[0].bus, WAM.act[link].puck.ID, JIDX, &reply);
          WAM.motor_position[link] = link;/** \bug Finish this *///reply; 
#endif
          // Read joint PID constants
          sprintf(key, "%s.link[%d].pid.kp", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&(valptr_vn(WAM.Kp)[link]));
          sprintf(key, "%s.link[%d].pid.kd", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&(valptr_vn(WAM.Kd)[link]));
          //parseGetVal(DOUBLE, key, (void*)&WAM.Kd);
          sprintf(key, "%s.link[%d].pid.ki", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&(valptr_vn(WAM.Ki)[link]));
          //parseGetVal(DOUBLE, key, (void*)&WAM.Ki);
          sprintf(key, "%s.link[%d].pid.max", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&(valptr_vn(WAM.saturation)[link]));
          //parseGetVal(DOUBLE, key, (void*)&WAM.saturation);
          
          // Read joint vel/acc defaults
          sprintf(key, "%s.link[%d].vel", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&tmpdbl);
          setval_vn(WAM.vel, link, tmpdbl);
          sprintf(key, "%s.link[%d].acc", robotType, link);
          parseGetVal(DOUBLE, key, (void*)&tmpdbl);
          setval_vn(WAM.acc, link, tmpdbl);
      
          link_geom_bot(&WAM.robot, link, theta * pi, d, a, alpha * pi);
          link_mass_bot(&WAM.robot, link, com, mass);
      }else{
          tool_mass_bot(&WAM.robot, com, mass);
          tool_geom_bot(&WAM.robot, theta * pi, d, a, alpha * pi);
      }
      
  }
  
      
  // Link geometry - file
  // Link mass - file
  // motor_position - enum
  // zero_offsets - unused
  // stop_torque - unused
  // park_location - file
  // zero_order - unused
  // Joint PID constants Kp, Kd, Ki - file
  // Joint saturation limits - file
  // Joint vel/acc defaults - file

  /* WAM tipped forward hack */
  /*
  setrow_m3(WAM.robot.world->origin,0,0.0,0.0,1.0);
  setrow_m3(WAM.robot.world->origin,1,0.0,1.0,0.0);
  setrow_m3(WAM.robot.world->origin,2,-1.0,0.0,0.0);
  */
  /* end WAM tipped forward hack */
  

  
  fill_vn(WAM.robot.dq,0.0);
  fill_vn(WAM.robot.ddq,0.0);

  WAM.Gcomp = 0;
  set_gravity_bot(&WAM.robot, 0.0);


  for(cnt = 0; cnt < 7; cnt++)
  {
    SCinit(&(WAM.sc[cnt]));
    SCsetpid(&(WAM.sc[cnt]),getval_vn(WAM.Kp,cnt),getval_vn(WAM.Kd,cnt),getval_vn(WAM.Ki,cnt),getval_vn(WAM.saturation,cnt));
    SCsettrjprof(&(WAM.sc[cnt]),getval_vn(WAM.vel,cnt),getval_vn(WAM.acc,cnt));
    
    setgains_btPID(&(WAM.d_jpos_ctl[cnt]), getval_vn(WAM.Kp,cnt),getval_vn(WAM.Kd,cnt),getval_vn(WAM.Ki,cnt));
  }
     
  test_and_log(
      pthread_mutex_init(&(WAM.loop_mutex),NULL),
      "Could not initialize mutex for WAM control loop.");
      
  return(&WAM);
}

void CloseWAM(wam_struct* wam)
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



/** This function closes the control loop on the WAM using a PID regulator in joint space.

The following functionality is in this loop:
 - Joint Space Controller (pid, point-to-point trapezoidal trajectories)
 - Loop profiling - Period time, Read pos time, Write trq time, Calc time
 - Newton-Euler Recursive kinematics & dynamics
 - Data logging
 - Continuous teach & play
 
 \todo Make control loop profiling a compiler switch so it is not normally compiled

*/

void WAMControlThread(void *data)
{
  btthread* this_thd;
  int                 cnt;
  int                 idx;
  int                 Mid;
  double              dt,dt_targ,skiptarg,skipmax = 0.0;
  int                 err,skipcnt = 0;
  long unsigned       counter = 0;
  RTIME last_loop,loop_start,loop_end,user_start,user_end,pos1_time,pos2_time,trq1_time,trq2_time;
  RT_TASK *WAMControlThreadTask;
  double thisperiod;
  RTIME rtime_period,sampleCount;

  /* Set up timer*/
  this_thd = (btthread*)data;
  thisperiod = this_thd->period;
  rtime_period = (RTIME)(thisperiod * 1000000000.0);
  sampleCount = nano2count(rtime_period);
  rt_set_periodic_mode();
  start_rt_timer(sampleCount);

  WAMControlThreadTask = rt_task_init(nam2num("WAMCon"), 0, 0, 0);
  //mlockall(MCL_CURRENT | MCL_FUTURE);
//#ifdef  BTREALTIME
  rt_make_hard_real_time();
//#endif
  syslog(LOG_ERR,"WAMControl initial hard");
  rt_task_make_periodic_relative_ns(WAMControlThreadTask, rtime_period, rtime_period);
  dt_targ = thisperiod;
  skiptarg = 1.5 * dt_targ;
  dt = dt_targ;
  WAM.skipmax = 0.0;
  syslog(LOG_ERR,"WAMControl period Sec:%f, ns: %d",thisperiod, rtime_period);  
  last_loop = rt_get_cpu_time_ns();
  while (!btthread_done(this_thd))
  {
    rt_task_wait_period();
    counter++;
    
    loop_start = rt_get_cpu_time_ns(); //&prof
    WAM.loop_period = loop_start - last_loop; //&prof
    dt = (double)WAM.loop_period / 1000000000.0;
    if (dt > skiptarg){
      skipcnt++;
      if (dt > WAM.skipmax) WAM.skipmax = dt;
    }
    WAM.dt = dt;
    
    
    test_and_log(
      pthread_mutex_lock(&(WAM.loop_mutex)),"WAMControlThread lock mutex failed");
    
#ifdef BTDOUBLETIME
    rt_make_soft_real_time();
#endif
    pos1_time = rt_get_cpu_time_ns(); //&prof
    
    GetPositions();
    
    pos2_time = rt_get_cpu_time_ns(); //&prof
    WAM.readpos_time = pos2_time - pos1_time; //&prof
#ifdef BTDOUBLETIME    
    rt_make_hard_real_time();
#endif
    ActAngle2Mpos((WAM.Mpos)); //Move motor angles into a wam_vector variable
    Mpos2Jpos((WAM.Mpos), (WAM.Jpos)); //Convert from motor angles to joint angles
    
    // Joint space stuff
    pos1_time = rt_get_cpu_time_ns(); //&prof
    eval_bts(&(WAM.Jsc));
    pos2_time = rt_get_cpu_time_ns(); //&prof
    WAM.Jsc_time = pos2_time - pos1_time; //&prof
    
    // Cartesian space stuff
    set_vn(WAM.robot.q,WAM.Jpos);

    eval_fk_bot(&WAM.robot);
    eval_fd_bot(&WAM.robot);
    
    set_v3(WAM.Cpos,T_to_W_bot(&WAM.robot,WAM.Cpoint));
    set_vn(WAM.R6pos,(vect_n*)WAM.Cpos);
    
    pos1_time = rt_get_cpu_time_ns(); //&prof
    eval_bts(&(WAM.Csc));
    pos2_time = rt_get_cpu_time_ns(); //&prof
    WAM.user_time = pos2_time - pos1_time; //&prof
    
    set_v3(WAM.Cforce,(vect_3*)WAM.R6force);
   //setrange_vn((vect_n*)WAM.Ctrq,WAM.R6force,0,2,3);
    //Force application
    apply_tool_force_bot(&WAM.robot, WAM.Cpoint, WAM.Cforce, WAM.Ctrq);
    
    pos1_time = rt_get_cpu_time_ns(); //&prof
    (*WAM.force_callback)(&WAM);
    pos2_time = rt_get_cpu_time_ns(); //&prof
    WAM.user_time = pos2_time - pos1_time; //&prof
    
    eval_bd_bot(&WAM.robot);
    get_t_bot(&WAM.robot,WAM.Ttrq);

    if(WAM.isZeroed)
    {
        set_vn(WAM.Jtrq,add_vn(WAM.Jtrq,WAM.Ttrq));
    }
    
    Jtrq2Mtrq((WAM.Jtrq), (WAM.Mtrq));  //Convert from joint torques to motor torques
    Mtrq2ActTrq(WAM.Mtrq); //Move motor torques from wam_vector variable into actuator database
#ifdef BTDOUBLETIME
    rt_make_soft_real_time();
#endif
    trq1_time = rt_get_cpu_time_ns(); //&prof
    
    SetTorques();
    
    trq2_time = rt_get_cpu_time_ns(); //&prof
    WAM.writetrq_time = trq2_time - trq1_time; //&prof
#ifdef BTDOUBLETIME
    rt_make_hard_real_time();
#endif
    loop_end = rt_get_cpu_time_ns(); //&prof
    WAM.loop_time = loop_end - loop_start; //&prof
    last_loop = loop_start; //&prof
    
    pthread_mutex_unlock(&(WAM.loop_mutex));
    
    WAM.log_time += dt;
    if ((counter % WAM.logdivider) == 0){
      TriggerDL(&(WAM.log)); //&log
    }
    //&cteach {
    if (WAM.cteach.Log_Data){ 
      WAM.counter++;
      WAM.teach_time += dt;
      if ((counter % WAM.divider) == 0)
        TriggerDL(&(WAM.cteach));
    }//&cteach }
  }
  syslog(LOG_ERR, "WAM Control Thread: exiting");
  syslog(LOG_ERR, "----------WAM Control Thread Statistics:--------------");
  syslog(LOG_ERR,"WAMControl Skipped cycles %d, Max dt: %f",skipcnt,skipmax);
  syslog(LOG_ERR,"WAMControl Times: Readpos: %f, Calcs: %f, SendTrq: %f",WAM.readpos_time,WAM.loop_time-WAM.writetrq_time-WAM.readpos_time,WAM.writetrq_time);
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
void DefineWAMpos(wam_struct *w,vect_n *wv)
{
  int cnt;
  long tmp;
  vect_n *motor_angle;
  double result;
  
  motor_angle = new_vn(7);
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

  WAM.isZeroed = TRUE;
}
/** Perform a coordinated move of the wam
*/
void MoveWAM(wam_struct* wam, vect_n *pos)
{
  int cnt,ctr,idx,done = 0,count =0,Mid;
  

  syslog(LOG_ERR,"MoveWAM start");
  for (cnt = 0;cnt < wam->num_actuators;cnt++)
  {
    Mid = MotorID_From_ActIdx(cnt);
       SCstarttrj(&(wam->sc[Mid]),getval_vn(pos,Mid));
  }
  syslog(LOG_ERR,"MoveWAM:Trajectory initialized");
  while (!done)
  {
    count++;
    ctr = 0;
    for (cnt = 0;cnt < wam->num_actuators;cnt++)
    {
      if (wam->sc[Mid].trj.state != BTTRAJ_STOPPED)
        ctr++;
    }
    if (ctr == 0)
      done = 1;
    usleep(50000);
    if ((count % 20) == 0)  syslog(LOG_ERR,"waited 1 second to reach position");
    if ((count % 200) == 0)  {done = 1; syslog(LOG_ERR,"Aborting move");}
  }
}
/** Set the velocity and acceleration of a wam move
*/
void MovePropsWAM(vect_n *vel, vect_n *acc)
{
  int cnt,ctr,idx,done = 0,count =0,Mid;

  for (cnt = 0;cnt < WAM.num_actuators;cnt++)
  {
    Mid = MotorID_From_ActIdx(cnt);
    SCsettrjprof(&(WAM.sc[Mid]),getval_vn(vel,Mid),getval_vn(acc,Mid));
  }
  syslog(LOG_ERR,"MovePropsWAM:Trajectory initialized");
}

void CartesianMovePropsWAM(btreal vel, btreal acc)
{
    setprofile_traptrj(&WAM.trj,  vel, acc);
}
/** Perform a Cartesian move of the wam
*/
void CartesianMoveWAM(vect_n *pos)
{
  int cnt,ctr,idx,done = 0,count =0;

  clear_pwl(&WAM.pth);
  add_arclen_point_pwl(&WAM.pth,(vect_n*)WAM.Cref);
  add_arclen_point_pwl(&WAM.pth,pos);
  start_traptrj(&WAM.trj, arclength_pwl(&WAM.pth));
  
  while (!done)
  {
    if (WAM.trj.state == BTTRAJ_STOPPED)
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
    if (WAM.trj.state == BTTRAJ_STOPPED)
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
  MoveWAM(&WAM, WAM.park_location);
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
  set_vn(Jtrq,WAM.Jtrq);
}
/**  */
btreal GetGravityComp(wam_struct *w)
{
  return get_gravity_bot(&w->robot);
}
/** Enable or disable the gravity compensation calculation
 */
void SetGravityComp(wam_struct *w,btreal scale)
{
  if (scale == 0.0){
    w->Gcomp = 0;
    set_gravity_bot(&w->robot, 0.0);
  }
  else{
    w->Gcomp = 1;
    set_gravity_bot(&w->robot, scale);
  }
}

/** Sample the torques required to hold a given position
*/
void GCompSample(vect_n *trq, double p1, double p2, double p3, double p4)
{


    const_vn(trq, p1, p2, p3, p4);
    MoveWAM(&WAM, trq);
    usleep(4000000);

    //trq->q[0] = WAM.Jtrq.q[0];
    trq->q[1] = WAM.Jtrq->q[1];
    trq->q[2] = WAM.Jtrq->q[2];
    trq->q[3] = WAM.Jtrq->q[3];
}

/** Take 4DOF measurements to estimate the coefficients for calculating the effect of gravity
*/
void getLagrangian4(double *A, double *B, double *C, double *D)
{

    double t41,t21,t42,t22,t23,t33; 

    vect_n *trq;
    
    trq = new_vn(4);

    //PowerWAM();

    MovePropsWAM(WAM.vel,WAM.acc);

    GCompSample(trq,0,-1.5708,0,0);
    t41 = trq->q[3];
    t21 = trq->q[1];

    GCompSample(trq,0,-1.5708,0,1.5708);
    t42 = trq->q[3];
    //t22 = trq->q[1];

    GCompSample(trq,0,-1.5708,-1.5708,1.5708);
    //t23 = trq->q[1];
    t33 = trq->q[2];

    *A = -t42;
    *B = -t41;
    *C = t33 + *B;
    *D = t21 - t41;
    //  WAM.Gcomp = 1;

    //SaveWAM("gcomp.dat");

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
/** Initialize Continuous teach and play

Notes: 
 - Variable numbers of joints are handled by polling the robot structure

\param Joint 0 = Cartesian space record, 1 = Joint space record
\param Div An integer amount to divide the sample period by
\param filename The file you want to write the path to
*/
void StartContinuousTeach(int Joint,int Div,char *filename) //joint: 0 = Cartesian, 1 = Joint Space
{ 
  int joints;
  
  WAM.teach_time = 0.0;
  WAM.counter = 0;
  WAM.divider = Div;
  if (Joint){ //Only for joint space recording for now
    joints = WAM.num_actuators;
    PrepDL(&(WAM.cteach),2);
    AddDataDL(&(WAM.cteach),&(WAM.teach_time),sizeof(btreal),4,"Time");
    AddDataDL(&(WAM.cteach),valptr_vn(WAM.Jpos),sizeof(btreal)*joints,4,"Jpos");
    InitDL(&(WAM.cteach),1000,filename);
    DLon(&(WAM.cteach));
  }
  else { //Just records position. No orientation
    PrepDL(&(WAM.cteach),2);
    AddDataDL(&(WAM.cteach),&(WAM.teach_time),sizeof(btreal),4,"Time");
    AddDataDL(&(WAM.cteach),valptr_vn((vect_n*)WAM.Cpos),sizeof(btreal)*3,4,"Cpos");
    InitDL(&(WAM.cteach),1000,filename);
    DLon(&(WAM.cteach));    
  }
}  
void StopContinuousTeach()
{
  
  DLoff(&(WAM.cteach));
  CloseDL(&(WAM.cteach));
}

void ServiceContinuousTeach()
{
  evalDL(&(WAM.cteach));
}

void registerWAMcallback(wam_struct* wam, void *func)
{
  if (func != NULL)
    wam->force_callback = func;
  else 
      wam->force_callback = BlankWAMcallback;
}

int BlankWAMcallback(struct btwam_struct *wam)
{
  return 0;
}

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
