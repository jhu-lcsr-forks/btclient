/* ======================================================================== *
 *  Module ............... Example 5 - Simple WAM Move Program
 *  File ................. main.c
 *  Author ............... Traveler Hauptman
 *                         Brian Zenowich
 *                         Christopher Dellin
 *  Creation Date ........ 26 May 2005
 *  Last Modified Date ... 20 Aug 2008
 *                                                                          *
 * ======================================================================== */

#include <stdio.h>    /* printf(), getchar(), setvbuf() */
#include <unistd.h>   /* usleep() */
#include <sys/mman.h> /* mlockall() */

#include "btwam.h"    /* All barrett functions */

/* Global file-scope variables */
btrt_thread_struct can_thd, wam_thd;
wam_struct * wam;
int startDone;

/* The CANbus card must be initialized and called from a realtime thread.
 * This thread is spun off from main() to handle the initial setup. */
void can_thd_function(void *thd)
{
   int err;

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      printf("  !! InitializeSystem returned err = %d", err);
      exit(1);
   }
    
   /* Initialize and get a handle to the robot on the first bus */
   if((wam = OpenWAM("../../wam.conf", 0)) == NULL){
      printf("  !! OpenWAM failed");
      exit(1);
   }
   
   /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
    * For now, the joint and tip velocities are ignored and
    * the elbow velocity provided is used for all three limits. */
   setSafetyLimits(0, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s

   /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical).
    * Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c) 
    * Note: btsystem.c bounds the outbound torque to 8191, so entering a
    * value of 9000 for TL2 would tell the safety system to never register a 
    * critical fault. */
   setProperty(0, SAFETY_MODULE, TL2, FALSE, 4700);
   setProperty(0, SAFETY_MODULE, TL1, FALSE, 1800);
   
   /* Notify main() thread that the initialization is complete */
   startDone = TRUE;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd))
      usleep(10000);
   
   /* Close the system */
   CloseSystem();
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}

/* Program entry point */
int main()
{
   int err;         /* Generic error variable for function calls */
   int busCount;    /* Number of WAMs defined in the configuration file;
                     * unused here. */
   vect_n * vector; /* A generic vector */
   matr_3 * matrix; /* A generic 4x4 transform matrix */
   
   /* This keeps this program's memory from being swapped out, reducing
    * the chance of disruptions from other programs on the system. */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
#ifdef RTAI
   /* Allow hard real time process scheduling for non-root users 
    * Xenomai non-root scheduling is coming soon! */
   rt_allow_nonroot_hrt();
#endif
   
   /* Turn off output buffering for printf() */
   setvbuf(stdout,0,_IONBF,0);
   
   /* Make sure the WAM is on and in the idle state. */
   printf("\n");
   printf("Welcome to Example 101: a Simple WAM Move Program.\n");
   printf("\n");
   printf("To continue, ensure the following steps have been taken:\n");
   printf("  a) All WAM power and signal cables are securely fastened.\n");
   printf("  b) The WAM is powered on (receiving power).\n");
   printf("  c) All E-STOPs are released.\n");
   printf("  d) The WAM is in Shift+Idle mode.\n");
   printf("  e) The WAM is in its home (folded) position.\n");
   printf("Press [Enter] to continue.\n");
   while (getchar()!='\n') usleep(10000);
   
   /* Initialize the system's buses.
    * We must do this before doing other stuff.
    * We provide the configuration file location
    * and a variable to hold the number of buses found. */
   printf("Initializing system buses ...");
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      printf("  !! ReadSystemFromConfig returned err = %d\n", err);
      exit(1);
   }
   printf(" %d buses found.\n", busCount);
   
   /* Set up the CAN bus
    * with a separate realtime thread using can_thd_function() from above. */
   printf("Setting up the CAN bus thread ...");
   startDone = FALSE;
   btrt_thread_create(&can_thd,"rtt",45,(void*)can_thd_function,NULL);
   while (!startDone) usleep(10000);
   printf(" done.\n");

   /* Spin off the WAM control loop
    * with a separate realtime thread using WAMControlThread() from btwam. */
   printf("Setting up the WAM control thread ...");
   wam_thd.period = 0.002; /* Control loop period in seconds */
   btrt_thread_create(&wam_thd,"ctrl",90,(void*)WAMControlThread,(void*)wam);
   printf(" done.\n");
   
   /* Let the user activate the WAM
    * before attempting to send torque commands */
   printf("Shift+Activate the WAM, and press [Enter] to continue.\n");
   while (getchar()!='\n') usleep(10000);
   
   /* Set gravity scale to 1.0g */
   printf("Setting gravity compensation to 1.0 g.\n");
   SetGravityComp(wam, 1.0);
   printf("\n");
   
   
   /* Start the joint space move */
   printf("Press [Enter] to start a joint space move,\n");
   printf("  with velocity = %f, acceleration = %f.", 0.5, 0.5);
   while (getchar()!='\n') usleep(10000);
   SetJointSpace(wam);
   MoveSetup( wam, 0.5, 0.5 );
   vector = new_vn( wam->dof );
   const_vn( vector, 0.1, -1.57, 0.79, 1.57,
                     1.57, -0.79, 0.79 ); /* Note: 4-DOF will ignore these */
   MoveWAM(wam,vector);
   destroy_vn(&vector);

   /* Stop the joint space move */
   while (!MoveIsDone(wam)) usleep(10000);
   printf("  ... done.\n");

   /* Disable the position constraint (back to just gravity comp) */   
   printf("Press [Enter] to disable the joint-space position constraint.\n");
   while (getchar()!='\n') usleep(10000);
   SetPositionConstraint(wam,0);
   
   
   /* Start the Cartesian space move
    * Note 1: the WAM uses a 4x4 matrix format for Cartesion position and
    *         orientation. The following code uses X,Y,Z in meters
    *         and the Euler angles Rx,Ry,Rz in radians.
    *         Possible positions/orientations for a 4-DOF WAM include:
    *              --- position ---  --- orientation ---
    *           a)    0,    0, 0.65, -0.34, 1.11,     0
    *           b)    0, 0.59, 0.51, -1.57,    0,     0
    *           c)    0, 0.59, 0.51, -0.95, 0.24, -0.71
    *           d)    0, 0.59, 0.51, -0.56, 0.96, -1.49
    * Note 2: By default the 4-DOF WAM's config file contains 0 gains for
    *         Cartesian orientation control. To enable this control,
    *         add/edit the following lines in your wam.conf:
    *           Rx_pd = <10.0, 0.04>
    *           Ry_pd = <10.0, 0.04>
    *           Rz_pd = <10.0, 0.04>   */
   printf("Press [Enter] to start a cartesian space move,\n");
   printf("  with velocity = %f, acceleration = %f.", 0.5, 0.5);
   while (getchar()!='\n') usleep(10000);
   SetCartesianSpace(wam);
   MoveSetup( wam, 0.5, 0.5 );
   matrix = new_m3();
   /* Insert the X, Y, and Z locations */
   setval_mn((matr_mn *)matrix, 0, 3, 0.0);
   setval_mn((matr_mn *)matrix, 1, 3, 0.0);
   setval_mn((matr_mn *)matrix, 2, 3, 0.65);
   /* Set rotation matrix from RxRyRz Euler angles */
   vector = new_vn( 3 );
   const_vn( vector, -0.34, 1.11,     0 );
   XYZftoR_m3(matrix, (vect_3 *)vector);
   destroy_vn(&vector);
   /* Copy the matrix elements into a 16-element vector */
   vector = new_vn( 16 );
   set_vn(vector,(vect_n *)matrix);
   MoveWAM(wam,vector);
   destroy_vn(&vector);
   destroy_mn((matr_mn **)&matrix);
   
   /* Stop the Cartesian space move */
   while (!MoveIsDone(wam)) usleep(10000);
   printf("  ... done.\n");

   /* Disable the position constraint (back to just gravity comp) */   
   printf("Press [Enter] to disable the Cartesian position constraint.\n");
   while (getchar()!='\n') usleep(10000);
   SetPositionConstraint(wam,0);
   
   
   /* Allow the user to re-home and shift-idle the WAM*/
   printf("Place the WAM back into its home position,\n");
   printf("  idle it with Shift+Idle,\n");
   printf("  and press [Enter] to end the program.\n");
   while (getchar()!='\n') usleep(10000);
   
   /* Stop the WAM control loop thread */
   printf("Stopping the control thread ...");
   wam_thd.done = TRUE;
   usleep(10000);
   printf(" done.\n");
   
   /* Stop the CAN bus thread */
   printf("Closing the system ...");
   can_thd.done = TRUE;
   usleep(50000);
   printf(" done.\n");
   
   printf("\n");
   return 0; 
}
