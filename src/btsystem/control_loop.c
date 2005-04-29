/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btcan.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......23 Nov 2002
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *                                                                      *
 *======================================================================*/

 /*! \file control_loop.c
    \brief Infrastructure to simplify adding and using your own control threads.
    
    This module adds code to start up timing threads that will call a user specified 
  control thread at regular intervals.
    
   
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <syslog.h>
#include <stddef.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <semaphore.h>
#include <malloc.h>
//th041216#include <process.h>
#include <time.h>
#include <inttypes.h>
//th041216#include <sys/sched.h>
#include <sys/mman.h>
//th041216#include <sys/neutrino.h>
//th041216#include <sys/netmgr.h>
//th041216#include <sys/syspage.h>
#ifdef USE_RTAI31
#include <rtai_lxrt.h>
#endif
#ifdef USE_FUSION
#include <rtai/task.h>
#include <rtai/timer.h>
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "control_loop.h"
#include "btcan.h"
#include "btsystem.h"
//th041216#include "traj.h"

sem_t     timer_semaphore;
pthread_t timer_thd_id;
pthread_t control_thd_id;
int shutdown_threads=1;
uint64_t ms_timer,sem_cntr;
long sample_period2;

/** Creates a timing thread and links it to your control loop function 

  \param priority The priority to run the timer and control loop threads at.
  \param sample_period An integer to multiply the system clock base epoch by. 
  If an integer multiple is not used, there will be problems of missing control 
  loops.

*/
void start_control_threads(int priority, long sample_period, void *function,void *args)
{

  pthread_attr_t       control_attr;
  struct sched_param   control_param;

  RTIME sampleCount;
  sample_period2 = sample_period;
  sampleCount = nano2count((RTIME) sample_period);

  rt_set_periodic_mode(); /* for clarity */

    /* start control_thread - the thread which controls the robot */
  pthread_attr_init(&control_attr);
  pthread_attr_setinheritsched(&control_attr, PTHREAD_EXPLICIT_SCHED);
  control_param.sched_priority = priority;
  pthread_attr_setschedparam(&control_attr, &control_param);
  
  shutdown_threads = 0;
  
  pthread_create(&control_thd_id, &control_attr, function, args);
  sched_setscheduler(control_thd_id, SCHED_FIFO, &control_param);

  if (control_thd_id == -1)
  {
    syslog(LOG_ERR,"start_linux_control_thread:Couldn't start control thread!");
    exit(-1);
  }

  start_rt_timer(sampleCount);


  
}
/** Stops the control threads

*/
void stop_control_threads()
{
  
 shutdown_threads = 1;
 usleep(2E5);

}

/** Example control loop for use with start_control_threads() Do not use this! */
void ControlThread(void *data)
{
  int num_actuators;
  actuator_struct *act;
  SimpleCtl *sc;
  int torques[5] = {0,0,0,0,0};
  long int position;
  
  num_actuators = ((control_thd_parms *)data)->num_act;
  act = ((control_thd_parms *)data)->act;
  sc = ((control_thd_parms *)data)->sc;

  
  
  while(!shutdown_threads)
  {
    sem_wait(&timer_semaphore);


  
    //TimedPuckGet(act->P.puck_ID,ACTUAL_POSITION,&position,10000);  
    
    
  }
  
  pthread_exit(NULL);
}
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
