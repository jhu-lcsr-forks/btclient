/*======================================================================*
 *  Module .............WAM autotension
 *  File ...............main.c
 *  Author .............Kevin Doo
 *  Creation Date ......12 Aug 2008
 *                                                                      *
 *======================================================================*/

/** \file main.c
    An autotension procedure for the WAM arm.  (works for 4 and 7 DOF WAMS)
 */

/* Include the standard WAM header file */
#include "btTension.h"
#include "btos.h"
#include "btmath.h"
#include "btcan.h"
#include "btserial.h"
#include "btcontrol.h"

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>


#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#else
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif


/* Define the realtime threads */
btrt_thread_struct    rt_thd, wam_thd, autotension_thd;

/* OpenWAM() will populate the WAM data structure */
wam_struct  *wam;

/* Flag to let the main() thread know when initialization is complete */
int startDone;

/** Global Variables (from btTension.h) */
/* all these variables are used in btTension.c */
extern int findingJointStop; //  0 = false, 1 = true
extern int findingTang;      //  0 = false, 1 = true
extern int tensioning;       //  0 = false, 1 = true
extern int hasTool;          //  0 = false, 1 = true
extern int autotensioning;   //  0 = false, 1 = true
extern int currentDirection; //  1  = Positive Direction
									  // -1  = Negative Direction
extern int currentJoint;     //  Joints are from 0 to 6
extern float numerator;    // 0 to 100%
extern float denominator;
extern float currentTorque[6];
extern float radianInc;  

const float maxJointTorque[6] = {75.6, 50.85, 30.26, 28.8, 6.8, 6.8};
const float maxT[6] = {0.9,0.9,0.9,0.8,0.3,0.3};

int DOF = -1;                //initially set to -1 (default)

/* Function prototypes */
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);
void mainLoop(void *thd);
void sigint_handler();

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
	/* close the main Thread*/
	autotension_thd.done = TRUE;
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

/* WAMcallback is a function that is called from the WAM Control Thread in btwam.c;
   WAMcallback allows for direct changes to JointTorques (Jtrq) and JointPositions (through Jref).
	WAMcallback is called every 2000 ms in the WAM Control Thread and is used for finding the
	joint stops of the WAM.
	*/
int WAMcallback(wam_struct *w)
{
	if(findingJointStop){
		float adjustment = 0.0;
		
		/* sets how much percent of maxTorque is to be used for finding a JointStop */
		if(DOF == 6 && (currentJoint == 2 || currentJoint == 3 || currentJoint == 4)){
			if(hasTool){
				adjustment = 1.0;
			} else {
				adjustment = 0.75;
			}
		} else if (DOF == 4 && (currentJoint == 2 || currentJoint == 3 || currentJoint == 4)){
			if(hasTool){
				adjustment = 0.75;
			} else {
				adjustment = 0.5;
			}
		} else {
			adjustment = .4;
		}
		
		/* is the currentJoint torque is below the set percent amount, move the joint slightly */
		if(fabs(((vect_n*)w->Mtrq)->q[currentJoint-1]) < adjustment * (maxT[currentJoint-1])){
			((vect_n*)w->Jref)->q[currentJoint-1] += (radianInc/50.)*currentDirection;
		} else {
		/* if the joint torque is above the set percent amount, most likely the joint stop has 
		   been found */
			findingJointStop = 0;
		/* set the joints reference point to the current position of the joint */
			((vect_n*)w->Jref)->q[currentJoint-1] = ((vect_n*)w->Jpos)->q[currentJoint-1];
		}
	}
	
	return 0;
}

/* WAMmotorcallback uses is a special added function call into btwam.c
   it can change both Joint and individual Motor values (something WAMcallback cannot).
	WAMmotorcallback is used for finding the tang and tensioning motors */
int WAMmotorcallback(wam_struct *w)
{
	if(findingTang || tensioning){
		float adjustment = 0.0, change;
		
		/* sets how much percent of maxTorque is to be used for finding a JointStop */
		if(findingTang && !tensioning){
			if(currentJoint == 1){
				adjustment = .30;
			} else {
				adjustment = .15;
			}
		} 

		/* decides how much relative tension is to be applied to the current motor */
		switch(currentJoint){
			case 1: change = currentTorque[0] + (0.9 * adjustment);                 
					  break;
			case 2: change = currentTorque[1] + (0.9 * adjustment);
					  break;
			case 3: change = currentTorque[2] + (0.9 * adjustment);
					  break;
			case 4: change = currentTorque[3] + (0.8 * adjustment);
					  break;
			/* cases 5 and 6 are in testing */
		   case 5: if (findingTang == 1) {
					  	change = currentTorque[5] + (0.3 * adjustment);
					  } else if (findingTang == 2) {
						change = currentTorque[4] + (0.3 * adjustment);
				     }
					  break;
			case 6: if (findingTang == 1) {
					  	change = currentTorque[4] + (0.3 * adjustment);
					  } else if (findingTang == 2) {
						change = currentTorque[5] + (0.3 * adjustment);
				     }
					  break;
			default:
					  break;
		}
		
		/* makes sure the change in tension never exceeds the maximum possible tension value 
		   (we do not want to overtension any motors) */
			
		/* NOTE: currentJoint here should be treated as currentMotor */
		
		/* works fine as motors 5 and 6 have the same max Torque */
		if(change > maxT[currentJoint - 1]){
			change = maxT[currentJoint - 1];
		} else if (change < -1.0 * maxT[currentJoint - 1]){
			change = -1.0 * maxT[currentJoint - 1];
		}
			
		if(tensioning){
			switch(currentJoint){
				case 1: change = 0.9;               
						  break;
				case 2: change = 0.9;      
						  break;
				case 3: change = 0.9;      
						  break;
				case 4: change = 0.8;      
						  break;
				case 5: change = 0.3;
						  break;
				case 6: change = 0.3;
						  break;
				default:
						  break;
			}
		}
		
		/* set the motor torque's value to the change value */
		if(currentJoint != 5 && currentJoint != 6){
			((vect_n*)w->Mtrq)->q[currentJoint-1] = change;
		} else {
			/* wrist motors require special cases for locking in double tang */
			if (findingTang == 1) {
				if(currentJoint == 5) {
					((vect_n*)w->Mtrq)->q[5] = change;
				} else if (currentJoint == 6) {
					((vect_n*)w->Mtrq)->q[4] = change;
				}
			} else if (findingTang == 2) {
				if(currentJoint == 5){
					((vect_n*)w->Mtrq)->q[5] = 0.0;
					((vect_n*)w->Mtrq)->q[4] = change;
				} else if (currentJoint == 6){
					((vect_n*)w->Mtrq)->q[4] = 0.0;
					((vect_n*)w->Mtrq)->q[5] = change;
				}
			}
		}
	}
	
	return 0;
}

void mainLoop(void *thd) {
	
	FILE * logFile;
	
	logFile = (FILE *)fopen ("tensionData.log","a");
	
	printf("\nInitializing WAM arm.\n");
	
	/*sets initial parameters*/
	initialize(wam, logFile);
	DOF = getDOF();
	int i;	
	
	printf("\nDone initializing.\n");
	printf("\nBeginning autotensioning of a %d DOF WAM...\n", DOF);
	
	/* continues prompting for inputs until told to quit in menu */
	do {
		i = menu();
	} while (i);
		
	setToHomePosition();
	printf("\nAutotensioning is complete.\n");
	
	fclose(logFile);
	
	promptNextStep();
	btrt_thread_exit((btrt_thread_struct*)thd);
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
   mlockall(MCL_CURRENT | MCL_FUTURE);
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
   btrt_thread_create(&rt_thd, "rttd", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

	registerWAMcallback(wam, WAMcallback);
	registerWAMmotorcallback(wam, WAMmotorcallback);
	
   /* Spin off the WAM control loop */
   wam_thd.period = 0.002; // Control loop period in seconds
   btrt_thread_create(&wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);
		usleep(500000);
	
   /* Loop until Ctrl-C is pressed */
   printf("\nPress Ctrl-C to exit...\n");
	
	printf("\nMake sure WAM is in the home position");
	printf("\nHit enter twice to continue...\n");
	
	promptNextStep();
	printf("\nStarting the main thread");
	/* Spin off another thread to run autotensioning functions */
	btrt_thread_create(&autotension_thd, "main", 90, (void*)mainLoop, (void*)wam);
	
	int i;
	float toFill;
	
   while(1) {
      /* Print out the current progress of autotensioning (percentage) */
		
		if(autotensioning){
			
			toFill = ((numerator/denominator) * 100)/2.0;
			
			printf("\rProgress: ");
			for(i = 1; i <= 50; i++){
				if( i == 25){
					printf("%.2f%%", (toFill * 2.0));
				} else if(i <= (int)toFill){
					printf("+");
				} else {
					printf(".");
				}
			}
			fflush(stdout);
  		}
      usleep(100000); // Wait a moment
   }
   
   /* We will never get here, but the compiler (probably) does not know this,
    * and it is expecting a return value from main(). Make it happy.
    */
   return(0); 
}

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
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

