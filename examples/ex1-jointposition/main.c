/*======================================================================*
 *  Module .............Example 1 - WAM Position
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......26 May 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 * 
 *                                                                      *
 *======================================================================*/

/** \file main.c
    A minimalist program for the wam that prints out the position of the 
    end point of the WAM
 
 */


#include <syslog.h>
#include <rtai_lxrt.h>
#include <stdlib.h> 
#include <stdio.h>
#include <errno.h>

#include "btwam.h"

int main(int argc, char **argv)
{
  char buf[50];
  char chr;
  int  done = 0;
  int  useGimbals = 0;
  struct sched_param mysched;
  int  err;
  static RT_TASK *mainTask;
  wam_struct *wam;

  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

  /* Initialize rtlinux subsystem */
  mysched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
  if (sched_setscheduler(0, SCHED_FIFO, &mysched) == -1)
  {
    syslog(LOG_ERR, "Error setting up linux (native) scheduler");
  }
  mainTask = rt_task_init(nam2num("main01"), 0, 0, 0); /* defaults */

  /* Initialize Bussed actuator system */
  err = InitializeSystem("actuators.dat","buses.dat","motors.dat","pucks.dat");
  if (err)
  {
    syslog(LOG_ERR, "Failed to initialize system");
    closelog();
    freebtptr();
    exit(1);
  }

  /* Check and handle any command line arguments */
  if(argc > 1)
  {
    if(!strcmp(argv[1],"-g")) // If gimbals are being used
    {
      initGimbals();
      useGimbals = 1;
      syslog(LOG_ERR, "Gimbals expected.");
    }
  }

  /* Set up the WAM data structure, init kinematics, dynamics, haptics */
  err =  InitWAM("wam.dat");
  if(err)
  {
    CloseSystem();
    closelog();
    freebtptr();
    exit(1);
  }

  /* Retrieve pointer to WAM data structure */
  wam = GetWAM();

  /* Start up wam control loop */
  start_control_threads(10, 0.002, WAMControlThread, (void *)0);

  while (!done)
  {

    if ((chr = getchar()) == 'x') //Check buffer for keypress
      done = 1;

    //print present position
    printf("Position = %s \n",sprint_vn(buf,(vect_n*)wam->Cpos));

    usleep(100000); // Sleep for 0.1s
  }

  stop_control_threads();
  closelog();
  rt_task_delete(mainTask);
  freebtptr();
}








