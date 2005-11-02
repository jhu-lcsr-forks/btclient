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
 
  //th cteach = Continuous teach & play
  //th prof = Loop time profiling
  //th log = Data logging
 
*/
#define TWOPI (2.0 * 3.14159265359)
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

void WAMMaintenanceThread(void *data);
int ActAngle2Mpos(wam_struct *wam,vect_n *Mpos); //Extracts actuators 0 thru 6 into vect_n positions
int Mtrq2ActTrq(wam_struct *wam,vect_n *Mtrq);  //Packs vect_n of torques into actuator array
void Mpos2Jpos(wam_struct *wam,vect_n * Mpos, vect_n * Jpos); //convert motor angle to joint angle
void Jpos2Mpos(wam_struct *wam,vect_n * Jpos, vect_n * Mpos);
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq); //conbert joint torque to motor torque
void InitVectors(wam_struct *wam);
void GetJointPositions();
void SetJointTorques();

void DumpWAM2Syslog();
int BlankWAMcallback(struct btwam_struct *wam);
/*==============================*
 * Functions                    *
 *==============================*/

void InitVectors(wam_struct *wam){
  wam->zero_offsets = new_vn(7);
  wam->stop_torque = new_vn(7);
  wam->park_location = new_vn(7);
  wam->Mpos = new_vn(7);
  wam->Mtrq  = new_vn(7);
  wam->Jpos = new_vn(7);
  wam->Jvel = new_vn(7);
  wam->Jacc = new_vn(7);
  wam->Jref = new_vn(7);
  wam->Jtref = new_vn(7);
  wam->Jtrq = new_vn(7);
  wam->Ttrq = new_vn(7);
  wam->Kp = new_vn(7);
  wam->Kd = new_vn(7);
  wam->Ki = new_vn(7);
  wam->saturation = new_vn(7);
  wam->vel = new_vn(7);
  wam->acc = new_vn(7);
  wam->Cpos = new_v3();
  wam->Cpoint = new_v3();
  wam->Cref = new_v3();
  wam->Cforce = new_v3();
  wam->Ctrq = new_v3();
  wam->Ckp = new_v3();
  wam->Ckd = new_v3();
  wam->Cki = new_v3();
  wam->use_new = 0;
  wam->qref = new_q();
  wam->qact = new_q();
  wam->qaxis = new_q();
  wam->forced = new_q();
  wam->qerr = 0;
  wam->R6pos = new_vn(6);
  wam->R6ref = new_vn(6);
  wam->R6tref = new_vn(6);
  wam->R6vel = new_vn(6);
  wam->R6acc = new_vn(6);
  wam->R6trq = new_vn(6);
  wam->R6force = new_vn(6);
  wam->js_or_cs = 0;
  wam->idle_when_done = 0;
  wam->active_sc = &wam->Jsc;
}

/** Initialize a WAM
 
  This function calls the necessary lower-level libbt functions to set
  up the pucks, enumerate them and scan in the wam configuration information.
 
  This function may kill the process using exit() if it runs into problems.
  
  \param fn wam configuration filename
  \retval NULL - Some error occured during initialization. Check /var/log/syslog.
  \retval wam_struct* - Pointer to wam data structure.
  
  See #btwam_struct
*/
wam_struct* OpenWAM(char *fn)
{
  int cnt,ret,err;
  const double pi = 3.14159;
  btreal theta, d, a, alpha, mass, tmpdbl;
  vect_3 *com;
  wam_struct *wam;
  char robotType[256];
  char key[256];
  long reply;
  int link;
  
  wam = (wam_struct*)btmalloc(sizeof(wam_struct));
  // Allocate memory for the WAM vectors
  InitVectors(wam);

  /* Joint Control plugin initialization */
  for (cnt = 0; cnt < 7; cnt ++)
  {
    init_btPID(&(wam->d_jpos_ctl[cnt]));
  }
  wam->d_jpos_array.pid = wam->d_jpos_ctl;
  wam->d_jpos_array.elements = 7;
  init_bts(&wam->Jsc);
  map_btstatecontrol(&wam->Jsc, wam->Jpos, wam->Jvel, wam->Jacc,
                     wam->Jref, wam->Jtref,wam->Jtrq, &wam->dt);
  btposition_interface_mapf_btPID(&wam->Jsc, &(wam->d_jpos_array));
  /* Joint Control plugin initialization */


  /* Control plugin initialization */
  for (cnt = 0; cnt < 3; cnt ++)
  {
    init_btPID(&(wam->d_pos_ctl[cnt]));
    setgains_btPID(&(wam->d_pos_ctl[cnt]),2000.0,5.0,0.0);
  }
  wam->d_pos_array.pid = wam->d_pos_ctl;
  wam->d_pos_array.elements = 6;
  init_bts(&wam->Csc);
  map_btstatecontrol(&wam->Csc, wam->R6pos, wam->R6vel, wam->R6acc,
                     wam->R6ref, wam->R6tref, wam->R6force, &wam->dt);
  btposition_interface_mapf_btPID(&wam->Csc, &(wam->d_pos_array));

  /* Control plugin initialization */

  init_pwl(&wam->pth,3,2); //Cartesian move path

  for (cnt = 0; cnt < 3; cnt ++)
  {
    init_btPID(&(wam->pid[cnt]));
    setgains_btPID(&(wam->pid[cnt]),2000.0,5.0,0.0);
  }
  setgains_btPID(&(wam->pid[3]),8.0,0.1,0.0);
  //setgains_btPID(&(wam->pid[3]),60.0,0.10,0.0);
  init_err_btPID(&(wam->pid[3]));


  wam->F = 0.0;
  wam->isZeroed = FALSE;
  wam->cteach.Log_Data = 0; //th cteach
  wam->force_callback = BlankWAMcallback;
  wam->log_time = 0.0;
  wam->logdivider = 1;


  SetEngrUnits(1);
#ifdef BTOLDCONFIG

  if(test_and_log(
            InitializeSystem("actuators.dat","buses.dat","motors.dat","pucks.dat"),
            "Failed to initialize system"))
  {
    exit(-1);
  }

  if(test_and_log(
            EnumerateSystem(),"Failed to enumerate system"))
  {
    exit(-1);
  }

#else //BTOLDCONFIG
  //------------------------
  err = InitializeSystem();
  if(err)
  {
    syslog(LOG_ERR, "OpenWAM: InitializeSystem returned err = %d", err);
    return(NULL);
  }
#endif
  wam->act = GetActuators(&(wam->num_actuators));

  new_bot(&wam->robot,wam->num_actuators);


  com = new_v3();


  // Parse config file
  err = parseFile(fn);
  if(err)
  {
    syslog(LOG_ERR, "OpenWAM: Error parsing config file- %s", fn);
    return(NULL);
  }
  //strcpy(robotType, buses[0].device_name);
  sprintf(key, "system.bus[0].name");
  parseGetVal(STRING, key, (void*)robotType);


  // Read park_location
  sprintf(key, "%s.home", robotType, link);
  parseGetVal(VECTOR, key, (void*)wam->park_location);

  for(link = 0; link <= wam->num_actuators; link++)
  {
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

    if(link != wam->num_actuators)
    {
      // Query for motor_position (JIDX)
#ifdef BTOLDCONFIG
      wam->motor_position[link] = link;
#else

      getProperty(wam->act[0].bus, wam->act[link].puck.ID, JIDX, &reply);
      wam->motor_position[link] = link;/** \todo Finish this *///reply;
#endif
      // Read joint PID constants
      sprintf(key, "%s.link[%d].pid.kp", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kp)[link]));
      sprintf(key, "%s.link[%d].pid.kd", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kd)[link]));
      //parseGetVal(DOUBLE, key, (void*)&wam->Kd);
      sprintf(key, "%s.link[%d].pid.ki", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Ki)[link]));
      //parseGetVal(DOUBLE, key, (void*)&wam->Ki);
      sprintf(key, "%s.link[%d].pid.max", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->saturation)[link]));
      //parseGetVal(DOUBLE, key, (void*)&wam->saturation);

      // Read joint vel/acc defaults
      sprintf(key, "%s.link[%d].vel", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&tmpdbl);
      setval_vn(wam->vel, link, tmpdbl);
      sprintf(key, "%s.link[%d].acc", robotType, link);
      parseGetVal(DOUBLE, key, (void*)&tmpdbl);
      setval_vn(wam->acc, link, tmpdbl);

      link_geom_bot(&wam->robot, link, theta * pi, d, a, alpha * pi);
      link_mass_bot(&wam->robot, link, com, mass);
    }
    else
    {
      tool_geom_bot(&wam->robot, theta * pi, d, a, alpha * pi);
      tool_mass_bot(&wam->robot, com, mass);


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
  setrow_m3(wam->robot.world->origin,0,0.0,0.0,1.0);
  setrow_m3(wam->robot.world->origin,1,0.0,1.0,0.0);
  setrow_m3(wam->robot.world->origin,2,-1.0,0.0,0.0);
  */
  /* end WAM tipped forward hack */



  fill_vn(wam->robot.dq,0.0);
  fill_vn(wam->robot.ddq,0.0);

  wam->Gcomp = 0;
  set_gravity_bot(&wam->robot, 0.0);


  for(cnt = 0; cnt < 7; cnt++)
  {
    SCinit(&(wam->sc[cnt]));
    SCsetpid(&(wam->sc[cnt]),getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt),getval_vn(wam->saturation,cnt));
    SCsettrjprof(&(wam->sc[cnt]),getval_vn(wam->vel,cnt),getval_vn(wam->acc,cnt));

    setgains_btPID(&(wam->d_jpos_ctl[cnt]), getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt));
  }

  test_and_log(
    pthread_mutex_init(&(wam->loop_mutex),NULL),
    "Could not initialize mutex for WAM control loop.");

  btthread_create(&wam->maint,0,(void*)WAMMaintenanceThread,wam);

  return(wam);
}
/** Free memory and close files opened by OpenWAM() */
void CloseWAM(wam_struct* wam)
{
  btthread_stop(&wam->maint);
  CloseSystem();
}


/**
 Actuator Idx starts at 0 and counts up to number of actuators. 
 MotorID is keyed to the location on the wam (and hense the joint it drives)
 This relationship allows only calculating the control info for the joints that
 are in use.
*/
BTINLINE int MotorID_From_ActIdx(wam_struct *wam,int idx)
{
  return wam->motor_position[idx];
}

/** Internal: Periodically services the data logging and teach&play data
structures.
 
*/
void WAMMaintenanceThread(void *data)
{
  btthread* this_thd;
  wam_struct *wam;

  this_thd = (btthread*)data;
  wam = (wam_struct*)this_thd->data;

  while (!btthread_done(this_thd))
  {
    if (wam->idle_when_done){
      if (get_trjstate_bts(wam->active_sc) == BTTRAJ_DONE){
        setmode_bts(wam->active_sc,SCMODE_IDLE);
        wam->idle_when_done = 0;
      }
    }
    evalDL(&(wam->log));
    evalDL(&(wam->cteach));
    usleep(100000); // Sleep for 0.1s
  }
  pthread_exit(NULL);
}

/** This function closes the control loop on the WAM using a PID regulator in joint space.
 
It is worth the programmers time to read through the code of this function and 
understand exactly what it is doing!
 
The following functionality is in this loop:
 - Joint Space Controller (pid, point-to-point trapezoidal trajectories)
 - Loop profiling - Period time, Read pos time, Write trq time, Calc time
 - Newton-Euler Recursive kinematics & dynamics
 - Data logging
 - Continuous teach & play recording
 
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
  wam_struct *wam;

  /* Set up timer*/
  this_thd = (btthread*)data;
  wam = this_thd->data;
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
  wam->skipmax = 0.0;
  syslog(LOG_ERR,"WAMControl period Sec:%f, ns: %d",thisperiod, rtime_period);
  last_loop = rt_get_cpu_time_ns();
  while (!btthread_done(this_thd))
  {
    rt_task_wait_period();
    counter++;

    loop_start = rt_get_cpu_time_ns(); //th prof
    wam->loop_period = loop_start - last_loop; //th prof
    dt = (double)wam->loop_period / 1000000000.0;
    if (dt > skiptarg)
    {
      skipcnt++;
      if (dt > wam->skipmax)
        wam->skipmax = dt;
    }
    wam->dt = dt;


    test_and_log(
      pthread_mutex_lock(&(wam->loop_mutex)),"WAMControlThread lock mutex failed");

#ifdef BTDOUBLETIME

    rt_make_soft_real_time();
#endif

    pos1_time = rt_get_cpu_time_ns(); //th prof

    GetPositions();

    pos2_time = rt_get_cpu_time_ns(); //th prof
    wam->readpos_time = pos2_time - pos1_time; //th prof
#ifdef BTDOUBLETIME

    rt_make_hard_real_time();
#endif

    ActAngle2Mpos(wam,(wam->Mpos)); //Move motor angles into a wam_vector variable
    Mpos2Jpos(wam,(wam->Mpos), (wam->Jpos)); //Convert from motor angles to joint angles

    // Joint space stuff
    pos1_time = rt_get_cpu_time_ns(); //th prof
    eval_bts(&(wam->Jsc));
    pos2_time = rt_get_cpu_time_ns(); //th prof
    wam->Jsc_time = pos2_time - pos1_time; //th prof

    // Cartesian space stuff
    set_vn(wam->robot.q,wam->Jpos);

    eval_fk_bot(&wam->robot);
    eval_fd_bot(&wam->robot);

    set_v3(wam->Cpos,T_to_W_bot(&wam->robot,wam->Cpoint));
    set_vn(wam->R6pos,(vect_n*)wam->Cpos);

    pos1_time = rt_get_cpu_time_ns(); //th prof
    eval_bts(&(wam->Csc));
    pos2_time = rt_get_cpu_time_ns(); //th prof
    wam->user_time = pos2_time - pos1_time; //th prof

    set_v3(wam->Cforce,(vect_3*)wam->R6force);
    //setrange_vn((vect_n*)wam->Ctrq,wam->R6force,0,2,3);
    //Force application
    apply_tool_force_bot(&wam->robot, wam->Cpoint, wam->Cforce, wam->Ctrq);

    pos1_time = rt_get_cpu_time_ns(); //th prof
    (*wam->force_callback)(wam);
    pos2_time = rt_get_cpu_time_ns(); //th prof
    wam->user_time = pos2_time - pos1_time; //th prof

    eval_bd_bot(&wam->robot);
    get_t_bot(&wam->robot,wam->Ttrq);

    if(wam->isZeroed)
    {
      set_vn(wam->Jtrq,add_vn(wam->Jtrq,wam->Ttrq));
    }

    Jtrq2Mtrq(wam,(wam->Jtrq), (wam->Mtrq));  //Convert from joint torques to motor torques
    Mtrq2ActTrq(wam,wam->Mtrq); //Move motor torques from wam_vector variable into actuator database
#ifdef BTDOUBLETIME

    rt_make_soft_real_time();
#endif

    trq1_time = rt_get_cpu_time_ns(); //th prof

    SetTorques();

    trq2_time = rt_get_cpu_time_ns(); //th prof
    wam->writetrq_time = trq2_time - trq1_time; //th prof
#ifdef BTDOUBLETIME

    rt_make_hard_real_time();
#endif

    loop_end = rt_get_cpu_time_ns(); //th prof
    wam->loop_time = loop_end - loop_start; //th prof
    last_loop = loop_start; //th prof

    pthread_mutex_unlock(&(wam->loop_mutex));

    wam->log_time += dt;
    if ((counter % wam->logdivider) == 0)
    {
      TriggerDL(&(wam->log)); //th log
    }
    //th cteach {
    if (wam->cteach.Log_Data)
    {
      wam->counter++;
      wam->teach_time += dt;
      if ((counter % wam->divider) == 0)
        TriggerDL(&(wam->cteach));
    }//th cteach }
  }
  syslog(LOG_ERR, "WAM Control Thread: exiting");
  syslog(LOG_ERR, "----------WAM Control Thread Statistics:--------------");
  syslog(LOG_ERR,"WAMControl Skipped cycles %d, Max dt: %f",skipcnt,skipmax);
  syslog(LOG_ERR,"WAMControl Times: Readpos: %f, Calcs: %f, SendTrq: %f",wam->readpos_time,wam->loop_time-wam->writetrq_time-wam->readpos_time,wam->writetrq_time);
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
int ActAngle2Mpos(wam_struct *wam,vect_n *Mpos) //Extracts actuators 0 thru 6 into wam_vector positions
{
  int num_act, cnt;

  num_act = wam->num_actuators;
  for (cnt = 0; cnt < num_act; cnt++)
  {
    setval_vn(Mpos,wam->motor_position[cnt],wam->act[cnt].angle);
  }

  return 0;
}
/** Load the actuator database torques with the values from a wam_vector.
 Mtrq2ActTrq loads the actuator database torques with the values from a wam_vector.
  \param *Mtrq Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int Mtrq2ActTrq(wam_struct *wam,vect_n *Mtrq) //Packs wam_vector of torques into actuator array
{
  actuator_struct *act;
  int num_act, cnt;

  act = GetActuators(&num_act);
  for (cnt = 0; cnt < num_act; cnt++)
  {
    act[cnt].torque = getval_vn(Mtrq,wam->motor_position[cnt]);
  }

  return 0;
}
//=================================================working code
/** Transform wam_vector Motor positions to Joint positions
*/
void Mpos2Jpos(wam_struct *wam,vect_n * Mpos, vect_n * Jpos) //convert motor angle to joint angle
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
void Jpos2Mpos(wam_struct *wam,vect_n * Jpos, vect_n * Mpos) //convert motor angle to joint angle
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
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq) //conbert joint torque to motor torque
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
 
Reset the position sensors on the pucks to the passed in position in units of joint-space radians
The wam must be in idle mode.
*/
void DefineWAMpos(wam_struct *wam,vect_n *wv)
{
  int cnt;
  long tmp;
  vect_n *motor_angle;
  double result;

  motor_angle = new_vn(7);
  /* Tell the safety logic to ignore the next faults */
  SetByID(SAFETY_MODULE, IFAULT, 8);

  //convert from joint space to motor space, then from motor radians to encoder counts
  Jpos2Mpos(wam,wv,motor_angle);
  for (cnt = 0;cnt < (wam->num_actuators - 3 * gimbalsInit);cnt++)
  {
    if(getval_vn(motor_angle,wam->motor_position[cnt]) == 0.0)
      result = 0;
    else
      result =( wam->act[cnt].motor.counts_per_rev / TWOPI)* getval_vn(motor_angle,wam->motor_position[cnt]);
    tmp = floor(result);
    //syslog(LOG_ERR,"ctsperrev = %d, motor_angle=%lf", wam->act[cnt].motor.counts_per_rev, motor_angle.q[wam->motor_position[cnt]]);
    //syslog(LOG_ERR,"tmp = %ld, result=%lf", tmp, result);
    SetProp(cnt,AP,tmp);
    SCevaluate(&(wam->sc[wam->motor_position[cnt]]),getval_vn(wv,cnt),0);
    SCsetmode(&(wam->sc[wam->motor_position[cnt]]),wam->sc[wam->motor_position[cnt]].mode);
    usleep(1000);
  }

  /* Tell the safety logic start monitoring tip velocity */
  SetByID(SAFETY_MODULE, ZERO, 1); /* 0 = Joint velocity, 1 = Tip velocity */

  wam->isZeroed = TRUE;
}


void SetCartesianSpace(wam_struct* wam)
{
  int trjstate;
  if (wam->active_sc == &wam->Jsc)//in joint space
  { 
    trjstate = get_trjstate_bts(wam->active_sc);
    if (trjstate != BTTRAJ_DONE && trjstate != BTTRAJ_STOPPED)
      stop_trj_bts(wam->active_sc);    
    setmode_bts(&(wam->Jsc),SCMODE_IDLE);
    setmode_bts(&(wam->Csc),SCMODE_IDLE);
    wam->active_sc = &wam->Csc;
  }
}

void SetJointSpace(wam_struct* wam)
{
  int trjstate;
  if (wam->active_sc == &wam->Csc)//in cartesian space
  { 
    trjstate = get_trjstate_bts(wam->active_sc);
    if (trjstate != BTTRAJ_DONE && trjstate != BTTRAJ_STOPPED)
      stop_trj_bts(wam->active_sc);
    setmode_bts(&(wam->Jsc),SCMODE_IDLE);
    setmode_bts(&(wam->Csc),SCMODE_IDLE);
    wam->active_sc = &wam->Jsc;
  }
}

/**   Turns position constraint on/off.

If the robot is being controlled in JointSpace (SetJointSpace), it will apply a PD loop to each joint.
If the robot is being controlled in CartesianSpace (SetCartesianSpace), it will apply a PD loop to the robot endpoint.

\param wam Pointer to wam structure you want to operate on.
\param onoff 0:Off !0:On

    
We only allow setting position mode if we are idling. 
Setting position mode while already in position mode will reset the position 
constraint causing unexpected behavior for a user at this level


*/
void SetPositionConstraint(wam_struct* wam, int onoff)
{
  int present_state;
  
  present_state = getmode_bts(wam->active_sc);
    if (onoff && present_state == SCMODE_IDLE)
       setmode_bts(wam->active_sc,SCMODE_POS); 
    else 
      setmode_bts(wam->active_sc,SCMODE_IDLE); 
}
/** Set the velocity and acceleration of a wam move
*/
void MoveSetup(wam_struct* wam,btreal vel,btreal acc)
{
  moveparm_bts(wam->active_sc,vel,acc);
}

void MoveWAM(wam_struct* wam,vect_n * dest)
{ 
  int present_state;
  present_state = getmode_bts(wam->active_sc);
  if (present_state != SCMODE_POS)
  {
    wam->idle_when_done = 1;
    setmode_bts(wam->active_sc,SCMODE_POS);
  }
  if(moveto_bts(wam->active_sc,dest))
      syslog(LOG_ERR,"MoveWAM:Aborted");
}
/**
\retval 0 Move is still going
\retval 1 Move is done
*/
int MoveIsDone(wam_struct* wam)
{
  int ret;
  ret = get_trjstate_bts(wam->active_sc);
  if (ret == BTTRAJ_DONE || ret == BTTRAJ_STOPPED)
    ret = 1;
  else 
    ret = 0;
  return ret;
}
void MoveStop(wam_struct* wam)
{
  stop_trj_bts(wam->active_sc);
}





/******************************************************************************/


/** Find the center of a joint range and move to the zero_offsets position relative to the center
 
\todo Add code to check the puck for IsZerod so that we
know whether we have to zero or not...
*/

/** Move the WAM to the park position defined in the wam data file
*/
void ParkWAM(wam_struct* wam)
{
  SetJointSpace(wam);
  MoveSetup(wam,0.25,0.25);
  MoveWAM(wam, wam->park_location);
}

/**Returns the present anti-gravity scaling
 
*/
btreal GetGravityComp(wam_struct *w)
{
  return get_gravity_bot(&w->robot);
}
/** Enable or disable the gravity compensation calculation
 */
void SetGravityComp(wam_struct *w,btreal scale)
{
  if (scale == 0.0)
  {
    w->Gcomp = 0;
    set_gravity_bot(&w->robot, 0.0);
  }
  else
  {
    w->Gcomp = 1;
    set_gravity_bot(&w->robot, scale);
  }
}

/** Sample the torques required to hold a given position
*/
void GCompSample(wam_struct *wam,vect_n *trq, double p1, double p2, double p3, double p4)
{


  const_vn(trq, p1, p2, p3, p4);
  MoveWAM(wam, trq);
  usleep(4000000);

  //trq->q[0] = WAM.Jtrq.q[0];
  trq->q[1] = WAM.Jtrq->q[1];
  trq->q[2] = WAM.Jtrq->q[2];
  trq->q[3] = WAM.Jtrq->q[3];
}

/** Take 4DOF measurements to estimate the coefficients for calculating the effect of gravity
*/
void getLagrangian4(wam_struct *wam,double *A, double *B, double *C, double *D)
{

  double t41,t21,t42,t22,t23,t33;

  vect_n *trq;

  trq = new_vn(4);

  MoveSetup(wam,0.5,0.5);

  GCompSample(wam,trq,0,-1.5708,0,0);
  t41 = trq->q[3];
  t21 = trq->q[1];

  GCompSample(wam,trq,0,-1.5708,0,1.5708);
  t42 = trq->q[3];
  //t22 = trq->q[1];

  GCompSample(wam,trq,0,-1.5708,-1.5708,1.5708);
  //t23 = trq->q[1];
  t33 = trq->q[2];

  *A = -t42;
  *B = -t41;
  *C = t33 + *B;
  *D = t21 - t41;
  
  free_vn(&trq);

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

/** Syslog wam configuration information for debugging
*/
void DumpWAM2Syslog(wam_struct *WAM)
{
  char buf[255];
  syslog(LOG_ERR,"Dump of WAM data---------------------------");
  syslog(LOG_ERR,"WAM:zero_offsets:%s",sprint_vn(buf,WAM->zero_offsets));
  syslog(LOG_ERR,"WAM:park_location:%s",sprint_vn(buf,WAM->park_location));
  syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(WAM->zero_order[0]),(WAM->zero_order[1]),(WAM->zero_order[2]),(WAM->zero_order[3]),(WAM->zero_order[4]),(WAM->zero_order[5]),(WAM->zero_order[6]));
  syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(WAM->motor_position[0]),(WAM->motor_position[1]),(WAM->motor_position[2]),(WAM->motor_position[3]),(WAM->motor_position[4]),(WAM->motor_position[5]),(WAM->motor_position[6]));
  syslog(LOG_ERR,"WAM:Kp:%s",sprint_vn(buf,WAM->Kp));
  syslog(LOG_ERR,"WAM:Kd:%s",sprint_vn(buf,WAM->Kd));
  syslog(LOG_ERR,"WAM:Ki:%s",sprint_vn(buf,WAM->Ki));
  syslog(LOG_ERR,"WAM:vel:%s",sprint_vn(buf,WAM->vel));
  syslog(LOG_ERR,"WAM:acc:%s",sprint_vn(buf,WAM->acc));
  syslog(LOG_ERR,"Num pucks %d",WAM->num_actuators);
  syslog(LOG_ERR,"End dump of WAM data-----------------------------");
}
/** Initialize Continuous teach and play
 
Notes: 
 - Variable numbers of joints are handled by polling the robot structure
 
\param Joint 0 = Cartesian space record, 1 = Joint space record
\param Div An integer amount to divide the sample period by
\param filename The file you want to write the path to
*/
void StartContinuousTeach(wam_struct *wam,int Joint,int Div,char *filename) //joint: 0 = Cartesian, 1 = Joint Space
{
  int joints;

  wam->teach_time = 0.0;
  wam->counter = 0;
  wam->divider = Div;
  if (Joint == 0)
  { //Just records position. No orientation
    PrepDL(&(wam->cteach),2);
    AddDataDL(&(wam->cteach),&(wam->teach_time),sizeof(btreal),4,"Time");
    AddDataDL(&(wam->cteach),valptr_vn((vect_n*)wam->Cpos),sizeof(btreal)*3,4,"Cpos");
    InitDL(&(wam->cteach),1000,filename);
    DLon(&(wam->cteach));
  }
  else
  { //Only for joint space recording for now
    joints = wam->num_actuators;
    PrepDL(&(wam->cteach),2);
    AddDataDL(&(wam->cteach),&(wam->teach_time),sizeof(btreal),4,"Time");
    AddDataDL(&(wam->cteach),valptr_vn(wam->Jpos),sizeof(btreal)*joints,4,"Jpos");
    InitDL(&(wam->cteach),1000,filename);
    DLon(&(wam->cteach));
  }
}
/** Stop recording positions to the continuous teach file */
void StopContinuousTeach(wam_struct *wam)
{

  DLoff(&(wam->cteach));
  CloseDL(&(wam->cteach));
}
/** \depreciated */
void ServiceContinuousTeach(wam_struct *wam)
{
  evalDL(&(wam->cteach));
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
