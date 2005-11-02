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
#include <signal.h>
#include "btwam.h"

btthread wam_thd;
wam_struct *wam;

void sigint_handler(){
  btthread_stop(&wam_thd); // Stop the WAMControlThread 
  CloseWAM(wam); // Free the wam data structure
  printf("\n\n");
  exit(1);
}

int main(int argc, char **argv)
{
  char buf[80];
  char chr;
  int  done = 0;
  int  useGimbals = 0;
  int  err;

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  atexit((void*)closelog);

  signal(SIGINT, sigint_handler);

#ifndef BTOLDCONFIG
  err = ReadSystemFromConfig("wam.conf"); 
#else //BTOLDCONFIG
#endif //BTOLDCONFIG    
  wam = OpenWAM("wam.conf");
  if(!wam)
  {
    exit(1);
  }            

  /* Check and handle any command line arguments */
  if(argc > 1)
  {
    if(!strcmp(argv[1],"-g")) // If gimbals are being used
    {
      initGimbals(wam);
      useGimbals = 1;
      syslog(LOG_ERR, "Gimbals expected.");
    }
  }

  /* Start up wam control loop */
  wam_thd.period = 0.002;
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)wam);

  printf("\nPress Ctrl-C to exit...\n");
  while (!done)
  {
    //print present position
    printf("\rPosition = %s\t",sprint_vn(buf,(vect_n*)wam->Jpos));
    fflush(stdout);
    usleep(100000); // Sleep for 0.1s
  }

}





