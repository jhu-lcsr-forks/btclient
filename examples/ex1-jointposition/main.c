/*======================================================================*
 *  Module .............Example 1 - WAM Position
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *                      Brian Zenowich
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
    A minimalist program for the WAM that prints out the position of the 
    end point of the WAM
 */

/* Provide access to standard C input/output functions such as printf() */
#include <stdio.h>

/* Provides atexit(), registers functions to execute upon process termination */
#include <stdlib.h>

/* The syslog daemon is used primarily for sending debug information to a log 
 * file. Note that syslog calls break realtime scheduling, so try not to use
 * syslog() from within a realtime thread!
 */
#include <syslog.h>

/* Allow us to catch the Ctrl-C signal and exit gracefully */
#include <signal.h>

/* Include the standard WAM header file */
#include "btwam.h"

/* Define the realtime threads */
btrt_thread_struct    rt_thd, wam_thd;

/* OpenWAM() will populate the WAM data structure */
wam_struct  *wam;

/* Flag to let the main() thread know when initialization is complete */
int startDone;

/* Function prototypes */
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);

/* If Ctrl-C is pressed, exit gracefully */
void sigint_handler()
{
   Cleanup();
   exit(1);
}

/* The CANbus card must be initialized and called from a realtime thread.
 * The rt_thread is spun off from main() to handle the initial communications.
 */
void rt_thread(void *thd){
   int err;

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned err = %d", err);
      exit(1);
   }
    
   /* Initialize and get a handle to the robot on the first bus */
   if((wam = OpenWAM("../../wam.conf", 0)) == NULL){
      syslog(LOG_ERR, "OpenWAM failed");
      exit(1);
   }
   
   /* Notify main() thread that the initialization is complete */
   startDone = TRUE;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd)){
      usleep(10000);
   }
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}

/* Exit the realtime threads and close the system */
void Cleanup(){
   /* Tell the WAM control thread to exit */
   wam_thd.done = TRUE;
   
   /* Wait for the control thread to exit, then free any data and device locks
    * associated with the WAM. The wait is so that we do not free the device
    * while the control loop is still using it!
    */
   usleep(10000);
   CloseSystem();
   
   /* Tell the initial communcation thread to exit */
   rt_thd.done = TRUE;
   
   /* Put some distance between the last printed data and the user's prompt */
   printf("\n\n");
}

/* Program entry point */
int main(int argc, char **argv)
{
   int   err;        // Generic error variable for function calls
   int   busCount;   // Number of WAMs defined in the configuration file
   char  buf[256];   // String used by sprint_vn() to convert the joint angle data to text
   
   /* Allow hard real time process scheduling for non-root users */
#ifdef RTAI   
   rt_allow_nonroot_hrt();
#else
   /* Xenomai non-root scheduling is coming soon! */
#endif

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Read the WAM configuration file */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig returned err = %d", err);
      exit(1);
   }
   
   /* Spin off the RT task to set up the CAN Bus.
    * RTAI priorities go from 99 (least important) to 1 (most important)
    * Xenomai priorities go from 1 (least important) to 99 (most important)
    */
   startDone = FALSE;
   btrt_thread_create(&rt_thd,"rtt", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

   /* Spin off the WAM control loop */
   wam_thd.period = 0.002; // Control loop period in seconds
   btrt_thread_create(&wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);

   /* Loop until Ctrl-C is pressed */
   printf("\nPress Ctrl-C to exit...\n");
   while(1) {
      /* Display the WAM's joint angles on-screen (see **NOTE below) */
      printf("\rPosition (rad) = %s\t",sprint_vn(buf,(vect_n*)wam->Jpos));
      fflush(stdout);
      usleep(100000); // Wait a moment
   }
   
   /* We will never get here, but the compiler (probably) does not know this,
    * and it is expecting a return value from main(). Make it happy.
    */
   return(0); 
}

/**NOTE:
 * sprint_vn() and vect_n are from our own math library. 
 * See src/btsystem/btmath.c. They are specialized tools for handling vectors. 
 * The WAM's joint positions are stored in the "wam" data structure as a vector 
 * (wam->Jpos). sprint_vn() is like sprintf() for our vector data structure.
 */
 

