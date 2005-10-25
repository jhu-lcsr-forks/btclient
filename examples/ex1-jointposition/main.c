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
  int  err;
  btthread wam_thd;
  wam_struct *wam;

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  atexit((void*)closelog);



#ifndef BTOLDCONFIG
  err = ReadSystemFromConfig("wamConfig.txt"); 
#else //BTOLDCONFIG
#endif //BTOLDCONFIG    
  wam = OpenWAM("wamConfig.txt");
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
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)&wam_thd);

  while (!done)
  {

    if ((chr = getchar()) == 'x') //Check buffer for keypress
      done = 1;

    //print present position
    printf("Position = %s \n",sprint_vn(buf,(vect_n*)wam->Jpos));

    usleep(100000); // Sleep for 0.1s
  }

  btthread_stop(&wam_thd); //Kill WAMControlThread 

  exit(1);
}








