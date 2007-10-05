/*======================================================================*
 *  Module .............libbtwam
 *  File ...............btwam.c
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
to make it easier to find and maintain later
 
  //th cteach = Continuous teach & play
  //th prof = Loop time profiling
  //th log = Data logging
 
*/
#define TWOPI (2.0 * 3.14159265359)
#define DEBUG(x) x

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
//wam_struct WAM;
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
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq); //convert joint torque to motor torque
void InitVectors(wam_struct *wam);
void GetJointPositions();
void SetJointTorques();
void DumpWAM2Syslog();
int BlankWAMcallback(struct btwam_struct *wam);


/*==============================*
 * Functions                    *
 *==============================*/
void InitVectors(wam_struct *wam)
{
   wam->zero_offsets = new_vn(wam->dof);
   wam->stop_torque = new_vn(wam->dof);
   wam->park_location = new_vn(wam->dof);
   wam->Mpos = new_vn(wam->dof);
   wam->N = new_vn(wam->dof);
   wam->n = new_vn(wam->dof);
   wam->M2JP = new_mn(wam->dof,wam->dof);
   wam->J2MT = new_mn(wam->dof,wam->dof);
   wam->J2MP = new_mn(wam->dof,wam->dof);
   wam->Mtrq  = new_vn(wam->dof);
   wam->Jpos = new_vn(wam->dof);
   wam->Jvel = new_vn(wam->dof);
   wam->Jacc = new_vn(wam->dof);
   wam->Jref = new_vn(wam->dof);
   wam->Jtref = new_vn(wam->dof);
   wam->Jtrq = new_vn(wam->dof);
   wam->Ttrq = new_vn(wam->dof);
   wam->Gtrq = new_vn(wam->dof);
   wam->Kp = new_vn(wam->dof);
   wam->Kd = new_vn(wam->dof);
   wam->Ki = new_vn(wam->dof);
   wam->saturation = new_vn(wam->dof);
   wam->torq_limit = new_vn(wam->dof);
   wam->vel = new_vn(wam->dof);
   wam->acc = new_vn(wam->dof);
   wam->Cpos = new_v3();
   wam->Cvel = new_v3();
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
   
   /* The original idea was to store XYZypr in a vn(6).
    * But ypr caused ugly discontinuous math.
    * So now we store XYZ+rMatrix in a vn(12).
    */
   wam->R6pos = new_vn(12);
   wam->R6ref = new_vn(12);
   wam->R6tref = new_vn(12);
   wam->R6vel = new_vn(12);
   wam->R6acc = new_vn(12);
   wam->R6trq = new_vn(12);
   wam->R6force = new_vn(12);
   wam->js_or_cs = 0;
   wam->idle_when_done = 0;
   wam->active_sc = &wam->Jsc;

   wam->motor_position = (int*)btmalloc(wam->dof * sizeof(int));
   wam->zero_order = (int*)btmalloc(wam->dof * sizeof(int));
   wam->d_jpos_ctl = (btPID*)btmalloc(wam->dof * sizeof(btPID));
   //wam->sc = (SimpleCtl*)btmalloc(wam->dof * sizeof(SimpleCtl));

   wam->G = new_vn(wam->dof);
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
wam_struct* OpenWAM(char *fn, int bus)
{
   int cnt,ret,err;
   const double pi = 3.14159;
   btreal theta, d, a, alpha, mass, tmpdbl;
   vect_3 *com;
   wam_struct *wam;
   char key[256];
   long reply;
   int link;
   char robotName[256];
   int actcnt = 0;

   // Carve out some memory for the main data structure
   wam = (wam_struct*)btmalloc(sizeof(wam_struct));
   
   // Parse config file
   if(!*fn) { // If no config file was given
      strcpy(fn, "wam.conf"); // Default to wam.conf
   }
   err = parseFile(fn);
   if(err) {
      syslog(LOG_ERR, "OpenWAM: Error parsing config file- %s", fn);
      return(NULL);
   }
   //strcpy(robotName, buses[bus].device_name);

   //if(!*rName) { // If no robot name was given
   
   // Extract the robot name from the config file
   //sprintf(key, "system.bus[%d].name", bus);
   //parseGetVal(STRING, key, (void*)robotName);
   
   // The robot name is already in buses[].device_name from InitializeSystem()
   syslog(LOG_ERR, "device_name=%s", buses[bus].device_name);
   strcpy(wam->name, buses[bus].device_name); // Set the name in the wam struct
   strcpy(robotName, wam->name);
   
   //} else {
   //   strcpy(robotName, rName);
   //}

   syslog(LOG_ERR, "robotName=%s", robotName);

   // Read Degrees of Freedom
   sprintf(key, "%s.dof", robotName);
   parseGetVal(INT, key, (void*)&wam->dof);

   // Allocate memory for the WAM vectors
   InitVectors(wam);

   /* START of Joint Control plugin initialization */
   for (cnt = 0; cnt < wam->dof; cnt ++){
      init_btPID(&(wam->d_jpos_ctl[cnt]));
      setgains_btPID(&(wam->d_jpos_ctl[cnt]), getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt));
   }
   wam->d_jpos_array.pid = wam->d_jpos_ctl;
   wam->d_jpos_array.elements = wam->dof;
   init_bts(&wam->Jsc);
   map_btstatecontrol(&wam->Jsc, wam->Jpos, wam->Jvel, wam->Jacc,
                      wam->Jref, wam->Jtref,wam->Jtrq, &wam->dt);
   btposition_interface_mapf_btPID(&wam->Jsc, &(wam->d_jpos_array));
   /* END of Joint Control plugin initialization */


   /* START of Control plugin initialization */
   for (cnt = 0; cnt < 3; cnt ++) {
      init_btPID(&(wam->d_pos_ctl[cnt]));
      setgains_btPID(&(wam->d_pos_ctl[cnt]),2000.0,5.0,0.0);
   }
   for (cnt = 3; cnt < 12; cnt++) {
      init_btPID(&(wam->d_pos_ctl[cnt]));
      setgains_btPID(&(wam->d_pos_ctl[cnt]),0.0,0.0,0.0);
   }
   wam->d_pos_array.pid = wam->d_pos_ctl;
   wam->d_pos_array.elements = 12;
   init_bts(&wam->Csc);
   map_btstatecontrol(&wam->Csc, wam->R6pos, wam->R6vel, wam->R6acc,
                      wam->R6ref, wam->R6tref, wam->R6force, &wam->dt);
   btposition_interface_mapf_btPID(&wam->Csc, &(wam->d_pos_array));
   /* END of Control plugin initialization */

   init_pwl(&wam->pth,3,2); //Cartesian move path

   wam->F = 0.0;
   wam->isZeroed = FALSE;
   wam->cteach.Log_Data = 0; //th cteach
   wam->force_callback = BlankWAMcallback;
   wam->log_time = 0.0;
   wam->logdivider = 1;

   SetEngrUnits(1);

   err = GetActuators(bus, &wam->act, &wam->num_actuators);
   //syslog(LOG_ERR, "bus=%d, num_actuators=%d", bus, wam->num_actuators);
   com = new_v3();

   // Initialize the generic robot data structure
   new_bot(&wam->robot,wam->dof);

   // Read park_location
   sprintf(key, "%s.home", robotName);
   parseGetVal(VECTOR, key, (void*)wam->park_location);

   // Read torq_limit
   sprintf(key, "%s.tlimit", robotName);
   parseGetVal(VECTOR, key, (void*)wam->torq_limit);

   // Read the worldframe->origin transform matrix
   sprintf(key, "%s.world", robotName);
   parseGetVal(MATRIX, key, (void*)wam->robot.world->origin);

   // Read whether M4 is reversed (new as of 2007 WAMs)
   sprintf(key, "%s.M4_2007", robotName);
   wam->M4_reversed = 0; // Default to "pre-2007"
   parseGetVal(INT, key, (void*)&wam->M4_reversed);

   //Read transmission matrix
   sprintf(key, "%s.m2jp", robotName);
   parseGetVal(MATRIX, key, (void*)wam->M2JP);

   //Read transmission matrix
   sprintf(key, "%s.j2mt", robotName);
   parseGetVal(MATRIX, key, (void*)wam->J2MT);

   // Read the Transmission ratios
   sprintf(key, "%s.N", robotName);
   parseGetVal(VECTOR, key, (void*)wam->N);

   // Read the transmission ratios
   sprintf(key, "%s.n", robotName);
   parseGetVal(VECTOR, key, (void*)wam->n);

   // Find motor positions
   for (actcnt = 0;actcnt < wam->num_actuators;actcnt++)
   {
#ifdef BTOLDCONFIG
      wam->motor_position[actcnt] = actcnt;
#else

      getProperty(wam->act[actcnt].bus, wam->act[actcnt].puck.ID, ROLE, &reply);
      if (reply == ROLE_GIMBALS) {
         wam->motor_position[actcnt] = wam->act[actcnt].puck.ID - 1;
      } else {
         switch(wam->act[actcnt].puck.ID){
            case -1: //case 1: case 4: // xxx Remove me
            wam->motor_position[actcnt] = wam->act[actcnt].puck.ID-1;
            break;
            default:
            getProperty(wam->act[actcnt].bus, wam->act[actcnt].puck.ID, JIDX, &reply);
            wam->motor_position[actcnt] = reply-1;
            break;
         }
      }
#endif
      syslog(LOG_ERR,"Motor[%d] is joint %d",actcnt,wam->motor_position[actcnt]);
   }

   syslog(LOG_ERR,"OpenWAM(): for link from 0 to %d", wam->dof);
   for(link = 0; link <= wam->dof; link++)
   {
      // Get the DH parameters
      sprintf(key, "%s.link[%d].dh.theta", robotName, link);
      parseGetVal(DOUBLE, key, (void*)&theta);
      sprintf(key, "%s.link[%d].dh.d", robotName, link);
      parseGetVal(DOUBLE, key, (void*)&d);
      sprintf(key, "%s.link[%d].dh.a", robotName, link);
      parseGetVal(DOUBLE, key, (void*)&a);
      sprintf(key, "%s.link[%d].dh.alpha", robotName, link);
      parseGetVal(DOUBLE, key, (void*)&alpha);

      // Get the mass parameters
      sprintf(key, "%s.link[%d].com", robotName, link);
      parseGetVal(VECTOR, key, (void*)com);
      sprintf(key, "%s.link[%d].mass", robotName, link);
      parseGetVal(DOUBLE, key, (void*)&mass);

      // Get inertia matrix
      sprintf(key, "%s.link[%d].I", robotName, link);
      parseGetVal(MATRIX, key, (void*)wam->robot.links[link].I);

      sprintf(key, "%s.link[%d].rotorI", robotName, link);
      parseGetVal(VECTOR, key, (void*)wam->robot.links[link].rotorI);

      // Add the rotor inertia to the link inertia
      for(cnt = 0; cnt < 3; cnt++)
         ELEM(wam->robot.links[link].I, cnt, cnt) += wam->robot.links[link].rotorI->q[cnt];

      if(link != wam->dof)
      {
         // Query for motor_position (JIDX)

         // Read joint PID constants
         sprintf(key, "%s.link[%d].pid.kp", robotName, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kp)[link]));
         sprintf(key, "%s.link[%d].pid.kd", robotName, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kd)[link]));
         //parseGetVal(DOUBLE, key, (void*)&wam->Kd);
         sprintf(key, "%s.link[%d].pid.ki", robotName, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Ki)[link]));
         //parseGetVal(DOUBLE, key, (void*)&wam->Ki);
         sprintf(key, "%s.link[%d].pid.max", robotName, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->saturation)[link]));
         //parseGetVal(DOUBLE, key, (void*)&wam->saturation);

         // Read joint vel/acc defaults
         sprintf(key, "%s.link[%d].vel", robotName, link);
         parseGetVal(DOUBLE, key, (void*)&tmpdbl);
         setval_vn(wam->vel, link, tmpdbl);
         sprintf(key, "%s.link[%d].acc", robotName, link);
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

   //syslog(LOG_ERR, "Kinematics loaded, bus = %d", wam->act[0].bus);
   
   /* If the WAM is already zeroed, note it- else, zero it */
   reply = 0;
   getProperty(wam->act->bus, SAFETY_MODULE, ZERO, &reply);

   if(reply)
   {
      wam->isZeroed = TRUE;
      syslog(LOG_ERR, "WAM was already zeroed");
   }
   else
   {
      DefineWAMpos(wam, wam->park_location);
      syslog(LOG_ERR, "WAM zeroed by application");
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
   
   for (cnt = 0; cnt < wam->dof; cnt ++){
      setgains_btPID(&(wam->d_jpos_ctl[cnt]), getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt));
   }
   
#if 0
   for(cnt = 0; cnt < wam->dof; cnt++)
   {
      SCinit(&(wam->sc[cnt]));
      SCsetpid(&(wam->sc[cnt]),getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt),getval_vn(wam->saturation,cnt));
      SCsettrjprof(&(wam->sc[cnt]),getval_vn(wam->vel,cnt),getval_vn(wam->acc,cnt));

      //setgains_btPID(&(wam->d_jpos_ctl[cnt]), getval_vn(wam->Kp,cnt),getval_vn(wam->Kd,cnt),getval_vn(wam->Ki,cnt));
   }
#endif
   test_and_log(
      btrt_mutex_create(&(wam->loop_mutex)),
      "Could not initialize mutex for WAM control loop.");

   btrt_thread_create(&wam->maint, "MAINT", 30, (void*)WAMMaintenanceThread, (void*)wam);
   //btthread_create(&wam->maint, 0, (void*)WAMMaintenanceThread, wam);

   return(wam);
}

/** Free memory and close files opened by OpenWAM() */
void CloseWAM(wam_struct* wam)
{
   //btthread_stop(&wam->maint);
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

   while (!btthread_done(this_thd)) {
      if (get_trjstate_bts(wam->active_sc) == BTTRAJ_DONE) {
         if(wam->active_sc->loop_trj) {
            start_trj_bts(wam->active_sc);
         } else if (wam->idle_when_done) {
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
 - Loop profiling - Period time,
   btthread* this_thd; Read pos time, Write trq time, Calc time
 - Newton-Euler Recursive kinematics & dynamics
 - Data logging
 - Continuous teach & play recording
 
 \todo Make control loop profiling a compiler switch so it is not normally compiled
 
*/

void WAMControlThread(void *data)
{
   btrt_thread_struct* this_thd;
   int                 cnt;
   int                 idx;
   int                 Mid;
   double              dt,dt_targ,skiptarg;
   int                 err,skipcnt = 0;
   int		       firstLoop = 1;
   long unsigned       counter = 0;
   RTIME last_loop,loop_start,loop_end,user_start,user_end,pos1_time,pos2_time,trq1_time,trq2_time;
   double thisperiod;
   RTIME rtime_period, sampleCount;
   wam_struct *wam;
   unsigned char CANdata[8];
   int len_in;
   int id_in;
   unsigned long overrun;
   int      ret, numskip;
   RT_TASK *WAMControlThreadTask;

   unsigned long    max1, min1, dat1, max2, min2, dat2, max3, min3, dat3, max4, min4, dat4, startt1, stopt1, startt2, stopt2, startt3, stopt3;
   unsigned long    loop_time;
   RTIME    mean1=0, mean2=0, mean3=0, mean4=0;
   double   sumX1, sumX21, stdev1, sumX2, sumX22, stdev2, sumX3, sumX23, stdev3, sumX4, sumX24, stdev4;
   
   /*//DEBUG: Seting up warn signal for changes to secondary mode, XENOMAI
   btrt_set_mode_warn();*/

   /* Rotation control */
   vect_3 *ns, *n, *os, *o, *as, *a, *f3, *t3;
   vect_n *e, *ed, *last_e, *f6;
   matr_mn *kp, *kd;
   vect_3 *v3;
   
   v3 = new_v3();
   ns = new_v3();
   n = new_v3();
   os = new_v3();
   o = new_v3();
   as = new_v3();
   a = new_v3();
   e = new_vn(6);
   ed = new_vn(6);
   last_e = new_vn(6);
   kp = new_mn(6,6);
   kd = new_mn(6,6);
   f6 = new_vn(6);
   
   set_mn(kp, scale_mn(10.0, kp)); //20
   set_mn(kd, scale_mn(0.08, kd)); //0.1
   

   /* Set up timer*/
   this_thd = (btrt_thread_struct*)data;
   wam = this_thd->data;
   thisperiod = this_thd->period;
   rtime_period = (RTIME)(thisperiod * 1000000000.0);

#ifdef RTAI
   sampleCount = nano2count(rtime_period);
   rt_set_periodic_mode();
   start_rt_timer(sampleCount);

   WAMControlThreadTask = rt_task_init(nam2num("WAM1"), 5, 0, 0);
   rt_task_make_periodic_relative_ns(WAMControlThreadTask, rtime_period, rtime_period);
   
#endif

   dt_targ = thisperiod;
   skiptarg = 1.5 * dt_targ;
   dt = dt_targ;
   wam->skipmax = 0.0;
   syslog(LOG_ERR,"WAMControl period Sec:%f, ns: %d",thisperiod, rtime_period);

   /*Set up as hard real time*/
   btrt_set_mode_hard();
   
#ifdef XENOMAI
   /*Make task periodic*/
   test_and_log(
      rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks((rtime_period))),"WAMControlThread: rt_task_set_periodic failed, code");
#endif

   //#ifdef  BTREALTIME
   //btrt_set_mode_hard();
   //#endif

   last_loop = btrt_get_time();
 
   DEBUG(
      //sumX1 = sumX21 = 0;
      max1 = 0;
      min1 = 2E9;

      //sumX2 = sumX22 = 0;
      max2 = 0;
      min2 = 2E9;

      //sumX3 = sumX23 = 0;
      max3 = 0;
      min3 = 2E9;

      //sumX4 = sumX24 = 0;
      max4 = 0;
      min4 = 2E9;
      //numskip = 0;
   );


   while (!btrt_thread_done(this_thd))
   {

      /*Make sure task is still in Primary mode*/
      btrt_set_mode_hard();

      DEBUG(startt3 = btrt_get_time());

      btrt_task_wait_period();


      //counter++;

      DEBUG(stopt3 = btrt_get_time());

      loop_start = btrt_get_time(); //th prof
      loop_time = loop_start - last_loop;
      wam->loop_period = loop_time; //th prof
      last_loop = loop_start;
      /* Find elapsed time (in seconds) since the previous control cycle */
      dt = loop_time / (double)1E9;
      if(!firstLoop){
         if (dt > skiptarg) {
            skipcnt++;
         }
         if (dt > wam->skipmax)
            wam->skipmax = dt;
      }
      wam->dt = dt;

      test_and_log(
         btrt_mutex_lock(&(wam->loop_mutex)),"WAMControlThread lock mutex failed");

      pos1_time = btrt_get_time(); //th prof


      // Clear CAN bus of any unwanted messages
      canClearMsg(0);


      GetPositions(wam->act->bus);

      pos2_time = btrt_get_time(); //th prof
      wam->readpos_time = pos2_time - pos1_time; //th prof

      DEBUG(
      if(mean1 || firstLoop){ 
         dat1 = pos2_time - pos1_time;
      }else{
         mean1 = dat1 = pos2_time - pos1_time;
      }
      );

      ActAngle2Mpos(wam,(wam->Mpos)); //Move motor angles into a wam_vector variable
      //const_vn(wam->Mpos, 0.0, 0.0, 0.0, 0.0);
      Mpos2Jpos(wam,(wam->Mpos), (wam->Jpos)); //Convert from motor angles to joint angles

      // Joint space stuff
      pos1_time = btrt_get_time(); //th prof
      eval_bts(&(wam->Jsc));
      pos2_time = btrt_get_time(); //th prof
      wam->Jsc_time = pos2_time - pos1_time; //th prof

      // Cartesian space stuff
      set_vn(wam->robot.q,wam->Jpos);
//syslog(LOG_ERR, "C 1 9");
      eval_fk_bot(&wam->robot);
      //syslog(LOG_ERR, "C 1 10");
      
      // Uncomment for Jacobian and inertia matrix calc - requires powerful CPU
      eval_fj_bot(&wam->robot); 
      
      eval_fd_bot(&wam->robot);
//syslog(LOG_ERR, "C 1 11");
      eval_bd_bot(&wam->robot);
      //syslog(LOG_ERR, "C 1 12");
      get_t_bot(&wam->robot, wam->Gtrq);

      /* Fill in the Cartesian XYZ */
      set_v3(wam->Cpos,T_to_W_bot(&wam->robot,wam->Cpoint));
      set_vn(wam->R6pos,(vect_n*)wam->Cpos);
      
      /* Append the rMatrix to R6pos */
      for(cnt = 0; cnt < 3; cnt++){
         getcol_mh(v3, wam->robot.tool->origin, cnt);
         setrange_vn(wam->R6pos, (vect_n*)v3, 3+3*cnt, 0, 3);
      }

      pos1_time = btrt_get_time(); //th prof
      eval_bts(&(wam->Csc));
      pos2_time = btrt_get_time(); //th prof
      wam->user_time = pos2_time - pos1_time; //th prof

      set_v3(wam->Cforce,(vect_3*)wam->R6force);
      //setrange_vn((vect_n*)wam->Ctrq,wam->R6force,0,2,3);
      
      /* Cartesian Rotation Control */
      fill_v3(wam->Ctrq, 0.0); // Start with zero torque
      if(getmode_bts(wam->active_sc) > SCMODE_IDLE && wam->active_sc == &wam->Csc){
         // Reference rotation
         setrange_vn((vect_n*)ns, wam->R6ref, 0, 3, 3);
         setrange_vn((vect_n*)os, wam->R6ref, 0, 6, 3);
         setrange_vn((vect_n*)as, wam->R6ref, 0, 9, 3);
         
         // Actual rotation
         getcol_m3(n, wam->robot.tool->origin, 0);
         getcol_m3(o, wam->robot.tool->origin, 1);
         getcol_m3(a, wam->robot.tool->origin, 2);
         
         setrange_vn(e, (vect_n*)(scale_v3(0.5, add_v3(cross_v3(ns,n), add_v3(cross_v3(os,o), cross_v3(as,a))))), 3, 0, 3);
         set_vn(ed, scale_vn(1.0/thisperiod, add_vn(e, scale_vn(-1.0, last_e))));
         set_vn(f6, add_vn(matXvec_mn(kp, e, e->ret), matXvec_mn(kd, ed, ed->ret)));
         setrange_vn(wam->R6force, f6, 3, 3, 3); // Just for show
         setrange_vn((vect_n*)wam->Ctrq, f6, 0, 3, 3);
         //apply_tool_force_bot(&(w->robot), w->Cpoint, f3, t3);
         set_vn(last_e, e);
      }
#if 0
      //Cartesian angular constraint
      R_to_q(wam->qact,T_to_W_trans_bot(&wam->robot));
      set_q(wam->qaxis,mul_q(wam->qact,conj_q(wam->qref)));
      set_q(wam->forced,force_closest_q(wam->qaxis));

      //syslog_vn("qref: ", (vect_n*)wam->qref);
      //syslog_vn("qact: ", (vect_n*)wam->qact);

      wam->qerr = GCdist_q(wam->qref,wam->qact);
      set_v3(wam->Ctrq,scale_v3(eval_err_btPID(&(wam->pid[3]),wam->qerr,dt),GCaxis_q(wam->Ctrq,wam->qref,wam->qact)));

      //syslog(LOG_ERR, "qerr: %f", wam->qerr);
      //syslog_vn("Ctrq: ", (vect_n*)wam->Ctrq);
#endif
      //syslog_vn("qref: ", (vect_n*)wam->qref);
      //syslog_vn("qact: ", (vect_n*)wam->qact);
      //Force application
      apply_tool_force_bot(&wam->robot, wam->Cpoint, wam->Cforce, wam->Ctrq);

      pos1_time = btrt_get_time(); //th prof
      (*wam->force_callback)(wam);
      pos2_time = btrt_get_time(); //th prof
      wam->user_time = pos2_time - pos1_time; //th prof

      eval_bd_bot(&wam->robot);
      get_t_bot(&wam->robot,wam->Ttrq);

      if(wam->isZeroed)
      {
         set_vn(wam->Jtrq,add_vn(wam->Jtrq,wam->Ttrq));
      }

      Jtrq2Mtrq(wam,(wam->Jtrq), (wam->Mtrq));  //Convert from joint torques to motor torques
#if 0

      if(wam->dof == 8)
      {
         //8-DOF paint spraying demo code
         if(getmode_bts(&wam->Jsc) == SCMODE_IDLE)
            setval_vn(wam->Mtrq, 7, 0);
         else
            setval_vn(wam->Mtrq, 7, getval_vn(wam->Jref, 7));
      }
#endif

      Mtrq2ActTrq(wam,wam->Mtrq); //Move motor torques from wam_vector variable into actuator database
#ifdef BTDOUBLETIME

      //rt_make_soft_real_time();
      btrt_set_mode_soft();
#endif

      trq1_time = btrt_get_time(); //th prof

      SetTorques(wam->act->bus);

      trq2_time = btrt_get_time(); //th prof
      wam->writetrq_time = trq2_time - trq1_time; //th prof
      //DEBUG(dat2 = trq2_time - trq1_time);
      DEBUG(
      if(mean2 || firstLoop){ 
         dat2 = trq2_time - trq1_time;
      }else{
         mean2 = dat2 = trq2_time - trq1_time;
      }
      );
#ifdef BTDOUBLETIME

      //rt_make_hard_real_time();
      btrt_set_mode_hard();

#endif

      loop_end = btrt_get_time(); //th prof
      wam->loop_time = loop_end - loop_start; //th prof

      btrt_mutex_unlock(&(wam->loop_mutex));

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

      DEBUG(
         
         if(mean3 || firstLoop){ 
            dat3 = stopt3-startt3;
         }else{
            mean3 = dat3 = stopt3-startt3;
         }
         
         if(mean4 || firstLoop){ 
            dat4 = trq1_time-pos2_time;
         }else{
            mean4 = dat4 = trq1_time-pos2_time;
         }
      
         /* Track max/min */
	 if(!firstLoop){
         if(dat1 > max1)
            max1 = dat1;
         if((dat1 < min1) && (dat1 > 100))
            min1 = dat1;
         if(dat2 > max2)
            max2 = dat2;
         if((dat2 < min2) && (dat2 > 100))
            min2 = dat2;
         if(dat3 > max3)
            max3 = dat3;
         if((dat3 < min3) && (dat3 > 100))
            min3 = dat3;
         if(dat4 > max4)
            max4 = dat4;
         if((dat4 < min4) && (dat4 > 100))
            min4 = dat4;

         if(dat1 > mean1) mean1+=1;
         if(dat1 < mean1) mean1-=1;
         if(dat2 > mean2) mean2+=1;
         if(dat2 < mean2) mean2-=1;
         if(dat3 > mean3) mean3+=1;
         if(dat3 < mean3) mean3-=1;
         if(dat4 > mean4) mean4+=1;
         if(dat4 < mean4) mean4-=1;
	 }
                        /* Track sum(X) and sum(X^2) 
                        sumX1 += dat1;
                        sumX21 += dat1 * dat1;
                        sumX2 += dat2;
                        sumX22 += dat2 * dat2;
                        sumX3 += dat3;
                        sumX23 += dat3 * dat3;
                        sumX4 += dat4;
                        sumX24 += dat4 * dat4;*/
                        );

         if(firstLoop) firstLoop = 0;
   }
   syslog(LOG_ERR, "WAM Control Thread: exiting");
   syslog(LOG_ERR, "----------WAM Control Thread Statistics:--------------");
   DEBUG(
   /*
      mean1 = 1.0 * sumX1 / counter;
      stdev1 = sqrt((1.0 * counter * sumX21 - sumX1 * sumX1) / (counter * counter - counter));
      mean2 = 1.0 * sumX2 / counter;
      stdev2 = sqrt((1.0 * counter * sumX22 - sumX2 * sumX2) / (counter * counter - counter));
      mean3 = 1.0 * sumX3 / counter;
      stdev3 = sqrt((1.0 * counter * sumX23 - sumX3 * sumX3) / (counter * counter - counter));
      mean4 = 1.0 * sumX4 / counter;
      stdev4 = sqrt((1.0 * counter * sumX24 - sumX4 * sumX4) / (counter * counter - counter));
*/
      syslog(LOG_ERR, "Get Positions)Max: %ld", max1);
      syslog(LOG_ERR, "              Min: %ld", min1);
      syslog(LOG_ERR, "             Mean: %ld", mean1);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev1);*/
      syslog(LOG_ERR, "Set Torques)  Max: %ld", max2);
      syslog(LOG_ERR, "              Min: %ld", min2);
      syslog(LOG_ERR, "             Mean: %ld", mean2);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev2);*/
      syslog(LOG_ERR, "Wait time)    Max: %ld", max3);
      syslog(LOG_ERR, "              Min: %ld", min3);
      syslog(LOG_ERR, "             Mean: %ld", mean3);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev3);*/
      syslog(LOG_ERR, "Calc time)    Max: %ld", max4);
      syslog(LOG_ERR, "              Min: %ld", min4);
      syslog(LOG_ERR, "             Mean: %ld", mean4);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev4);*/
      /*syslog(LOG_ERR, "Skips %d", counter);*/
   );


   syslog(LOG_ERR,"WAMControl Skipped cycles %d, Max dt: %lf",skipcnt, wam->skipmax);
   //syslog(LOG_ERR,"WAMControl Times: Readpos: %lld, Calcs: %lld, SendTrq: %lld",wam->readpos_time,wam->loop_time-wam->writetrq_time-wam->readpos_time,wam->writetrq_time);

   /*Delete thread when done*/
   btrt_thread_exit(this_thd);
}

/** Load a wam_vector with the actuator angles.
 ActAngle2Mpos loads a wam_vector with the actuator angles.
  \param *Mpos Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int ActAngle2Mpos(wam_struct *wam,vect_n *Mpos)
{
   int num_act, cnt;

   //Extracts actuators 0 thru 6 into wam_vector positions
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
int Mtrq2ActTrq(wam_struct *wam,vect_n *Mtrq)
{
   actuator_struct *act;
   int num_act, cnt;

   //Packs wam_vector of torques into actuator array
   //act = GetActuators(&num_act);
   for (cnt = 0; cnt < buses[wam->act[0].bus].num_pucks; cnt++) {
      wam->act[cnt].torque = getval_vn(Mtrq,wam->motor_position[cnt]);
   }

   return 0;
}

//=================================================working code
/** Transform wam_vector Motor positions to Joint positions
*/
void Mpos2Jpos(wam_struct *wam,vect_n * Mpos, vect_n * Jpos)
{
   vect_n tmp_vn[2];
   btreal tmp_btreal[8];
   btreal pos[10];
   btreal mN[10];
   btreal mn[10];

   init_local_vn(tmp_vn,tmp_btreal,8);
   /*
   extract_vn(pos,Mpos);
   extract_vn(mN,wam->N);
   extract_vn(mn,wam->n);
   setval_vn(Jpos,0, ( -pos[0] / mN[0]));
   setval_vn(Jpos,1, (0.5 * pos[1] / mN[1]) - (0.5 * pos[2] / mN[2]));
   setval_vn(Jpos,2, ( -0.5 * pos[1] * mn[2] / mN[1]) - (0.5 * mn[2] * pos[2] / mN[2]));
   setval_vn(Jpos,3,  (pos[3] / mN[3]));
   setval_vn(Jpos,4, 0.5 * pos[4] / mN[4] + 0.5 * pos[5] / mN[4]);
   setval_vn(Jpos,5, -0.5 * pos[4] * mn[5] / mN[4] + 0.5 * mn[5] * pos[5] / mN[4]);
   setval_vn(Jpos,6, -1 * pos[6] / mN[6]);
   */
   set_vn(Jpos,matXvec_mn(wam->M2JP,Mpos,tmp_vn));
}

//=================================================working code
/** Transform wam_vector Joint positions to Motor positions
*/
void Jpos2Mpos(wam_struct *wam,vect_n * Jpos, vect_n * Mpos)
{
   btreal pos[10];
   btreal mN[10];
   btreal mn[10];
   extract_vn(pos,Jpos);
   extract_vn(mN,wam->N);
   extract_vn(mn,wam->n);
   setval_vn(Mpos,0, ( -pos[0] * mN[0]));
   setval_vn(Mpos,1, (pos[1] * mN[1]) - (pos[2] * mN[2] / mn[2]));
   setval_vn(Mpos,2, ( -pos[1] * mN[1]) - ( pos[2] * mN[2] / mn[2]));
   if(wam->M4_reversed)
      setval_vn(Mpos,3, (-pos[3] * mN[3]));
   else
      setval_vn(Mpos,3, (pos[3] * mN[3]));
   if(wam->dof > 4)
   {
      setval_vn(Mpos,4,pos[4] * mN[4] - pos[5] * mN[4] / mn[5]);
      setval_vn(Mpos,5, pos[4] * mN[4] + pos[5] * mN[4] / mn[5]);
      setval_vn(Mpos,6, -pos[6] * mN[6]);
   }
}

/** Transform wam_vector Joint torques to Motor torques
*/
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq)
{
   vect_n tmp_vn[2];
   btreal tmp_btreal[8];
   btreal trq[10];
   btreal mN[10];
   btreal mn[10];
   init_local_vn(tmp_vn,tmp_btreal,8);
   /*
   extract_vn(trq,Jtrq);
   extract_vn(mN,wam->N);
   extract_vn(mn,wam->n);
   setval_vn(Mtrq,0, ( -trq[0] / mN[0]));
   setval_vn(Mtrq,1, (0.5 * trq[1] / mN[1]) - (0.5 * trq[2] *mn[2]/ mN[2]));
   setval_vn(Mtrq,2, ( -0.5 * trq[1]  / mN[1]) - (0.5 * mn[2] * trq[2] / mN[2]));
   setval_vn(Mtrq,3,  (trq[3] / mN[3]));
   setval_vn(Mtrq,4, 0.5 * trq[4] / mN[4] - 0.5 * mn[5] * trq[5] / mN[4]);
   setval_vn(Mtrq,5, 0.5 * trq[4] / mN[4] + 0.5 * mn[5] * trq[5] / mN[4]);
   setval_vn(Mtrq,6, -1 * trq[6] / mN[6]);
   */
   set_vn(Mtrq,matXvec_mn(wam->J2MT,Jtrq,tmp_vn));

}

/** Reset the position sensors on the pucks to the passed in position in units of joint-space radians
 
Reset the position sensors on the pucks to the passed in position in units of joint-space radians
The wam must be in idle mode.
*/
void DefineWAMpos(wam_struct *wam, vect_n *wv)
{
   int cnt;
   long tmp;
   vect_n *motor_angle;
   double result;

   motor_angle = new_vn(wam->dof);
   /* Tell the safety logic to ignore the next faults */
   SetByID(wam->act->bus, SAFETY_MODULE, IFAULT, 8);

   //convert from joint space to motor space, then from motor radians to encoder counts
   Jpos2Mpos(wam, wv, motor_angle);
   for (cnt = 0;cnt < (wam->num_actuators - 3 * gimbalsInit);cnt++)
   {
      if(getval_vn(motor_angle,wam->motor_position[cnt]) == 0.0)
         result = 0;
      else
         result =( wam->act[cnt].motor.counts_per_rev / TWOPI)* getval_vn(motor_angle,wam->motor_position[cnt]);
      tmp = floor(result);
      //syslog(LOG_ERR,"ctsperrev = %d, motor_angle=%lf", wam->act[cnt].motor.counts_per_rev, motor_angle.q[wam->motor_position[cnt]]);
      //syslog(LOG_ERR,"tmp = %ld, result=%lf", tmp, result);
      setProperty(wam->act->bus,wam->act[cnt].puck.ID,AP,FALSE,tmp);
      //SCevaluate(&(wam->sc[wam->motor_position[cnt]]),getval_vn(wv,cnt),0);
      //SCsetmode(&(wam->sc[wam->motor_position[cnt]]),wam->sc[wam->motor_position[cnt]].mode);
      usleep(1000);
   }

   /* Tell the safety logic start monitoring tip velocity */
   SetByID(wam->act->bus, SAFETY_MODULE, ZERO, 1); /* 0 = Joint velocity, 1 = Tip velocity */

   wam->isZeroed = TRUE;
}


void SetCartesianSpace(wam_struct* wam)
{
   int trjstate;

   // In joint space?
   if (wam->active_sc == &wam->Jsc)
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

   // In cartesian space?
   if (wam->active_sc == &wam->Csc)
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
   if (present_state != SCMODE_POS) {
      //wam->idle_when_done = 1; // Commented out. We want the WAM to
      //hold position when the move is done.
      setmode_bts(wam->active_sc,SCMODE_POS);
   }else{
	   wam->idle_when_done = 0;
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
   if (scale == 0.0) {
      w->Gcomp = 0;
      set_gravity_bot(&w->robot, 0.0);
   } else {
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
   trq->q[1] = wam->Jtrq->q[1];
   trq->q[2] = wam->Jtrq->q[2];
   trq->q[3] = wam->Jtrq->q[3];
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

   destroy_vn(&trq);
}

void setSafetyLimits(int bus, double jointVel, double tipVel, double elbowVel)
{
   int conversion;
   syslog(LOG_ERR, "About to set safety limits, VL2 = %d", VL2);
   if((jointVel > 0) && (jointVel < 7)) /* If the vel (rad/s) is reasonable */
   {
      conversion = jointVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

   if((tipVel > 0) && (tipVel < 7)) /* If the vel (m/s) is reasonable */
   {
      conversion = tipVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

   if((elbowVel > 0) && (elbowVel < 7)) /* If the vel (m/s) is reasonable */
   {
      conversion = elbowVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

}

/** Syslog wam configuration information for debugging
*/
void DumpWAM2Syslog(wam_struct *wam)
{
   char buf[255];
   syslog(LOG_ERR,"Dump of WAM data---------------------------");
   syslog(LOG_ERR,"WAM:zero_offsets:%s",sprint_vn(buf,wam->zero_offsets));
   syslog(LOG_ERR,"WAM:park_location:%s",sprint_vn(buf,wam->park_location));
   syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(wam->zero_order[0]),(wam->zero_order[1]),(wam->zero_order[2]),(wam->zero_order[3]),(wam->zero_order[4]),(wam->zero_order[5]),(wam->zero_order[6]));
   syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(wam->motor_position[0]),(wam->motor_position[1]),(wam->motor_position[2]),(wam->motor_position[3]),(wam->motor_position[4]),(wam->motor_position[5]),(wam->motor_position[6]));
   syslog(LOG_ERR,"WAM:Kp:%s",sprint_vn(buf,wam->Kp));
   syslog(LOG_ERR,"WAM:Kd:%s",sprint_vn(buf,wam->Kd));
   syslog(LOG_ERR,"WAM:Ki:%s",sprint_vn(buf,wam->Ki));
   syslog(LOG_ERR,"WAM:vel:%s",sprint_vn(buf,wam->vel));
   syslog(LOG_ERR,"WAM:acc:%s",sprint_vn(buf,wam->acc));
   syslog(LOG_ERR,"Num pucks %d",wam->num_actuators);
   syslog(LOG_ERR,"End dump of WAM data-----------------------------");
}

/** Initialize Continuous teach and play
 
Notes: 
 - Variable numbers of joints are handled by polling the robot structure
 
\param Joint 0 = Cartesian space record, 1 = Joint space record
\param Div An integer amount to divide the sample period by
\param filename The file you want to write the path to
*/
void StartContinuousTeach(wam_struct *wam,int Joint,int Div,char *filename)
{
   int joints;

   wam->teach_time = 0.0;
   wam->counter = 0;
   wam->divider = Div;
   if (Joint == 0) { //Just records position. No orientation
      PrepDL(&(wam->cteach),2);
      AddDataDL(&(wam->cteach),&(wam->teach_time),sizeof(btreal),4,"Time");
      //AddDataDL(&(wam->cteach),valptr_vn((vect_n*)wam->Cpos),sizeof(btreal)*3,4,"Cpos");
      AddDataDL(&(wam->cteach),valptr_vn((vect_n*)wam->R6pos),sizeof(btreal)*12,4,"R6pos");

   } else { //Only for joint space recording for now
      joints = wam->num_actuators;
      PrepDL(&(wam->cteach),2);
      AddDataDL(&(wam->cteach),&(wam->teach_time),sizeof(btreal),4,"Time");
      AddDataDL(&(wam->cteach),valptr_vn(wam->Jpos),sizeof(btreal)*joints,4,"Jpos");
   }

   InitDL(&(wam->cteach),1000,filename);
   DLon(&(wam->cteach));
   TriggerDL(&(wam->cteach));
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
