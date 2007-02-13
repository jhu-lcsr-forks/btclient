/*======================================================================*
 *  Module .............autotension
 *  File ...............main.c
 *  Author .............Brian Zenowich
 *  Creation Date ......25 Jan 2006
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file main.c
    \brief Autotensioning application for the WAM.
    
    This program allows a user tension the WAM's mechanical cables.
 
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>

#include <rtai_lxrt.h>
#include <rtai_sem.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
int useGimbals  = FALSE;
int done        = FALSE;

wam_struct  *wam;
btthread    wam_thd;
int         i3 = 0, i4 = 0;
long        t = 0;
double      x3 = 0.0, x4 = 0.0;
double      A3, A4, P3, P4, S3, S4;
double      Tt = 1.0, Ts = 0.002, m3, m4;
btPID*      jPID;
via_trj_array *vt_j = NULL;
int         cycle = 0;

#define pi (3.14159)

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
int mygetch();
int WAMcallback(struct btwam_struct *wam);
void readParameters(char *fn);

/*==============================*
 * Functions                    *
 *==============================*/
/** Entry point for the application.
    Initializes the system and executes the main event loop.
*/
int main(int argc, char **argv)
{
   int      err;
   int      i;
   char     robotName[128];
   char     vect_buf1[250];
   vect_n*  jdest;
   char     key[255];

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   /* Lead the user through a proper WAM startup */
   printf("\nMake sure the all WAM power and signal cables are securely");
   printf("\nfastened, then turn on the main power to WAM and press <Enter>");
   mygetch();

   printf("\n\nMake sure all E-STOPs are released, then press Shift-Idle");
   printf("\non the control pendant. Then press <Enter>");
   mygetch();

   printf("\n\nPlace WAM in its home (folded) position, then press <Enter>");
   mygetch();

   err = ReadSystemFromConfig("wam.conf");

   /* If the robot name was given on the command line, use it */
   *robotName = 0;
   for(i = 1; i < argc-1; i++)
   {
      if(!strcmp(argv[i],"-n"))
         strcpy(robotName, argv[i+1]);
   }

   /* Initialize and get a handle to the robot */
   if(!(wam = OpenWAM("wam.conf", robotName)))
   {
      printf("\n\nThere was an error while initializing the robot.");
      printf("\nPlease check /var/log/syslog for details.\n\n");
      exit(1);
   }

   /* Check and handle any additional command line arguments */
   for(i = 1; i < argc-1; i++)
   {
      if(!strcmp(argv[i], "-g")) // If gimbals are being used
      {
         initGimbals(wam);
         useGimbals = TRUE;
         syslog(LOG_ERR, "Gimbals expected.");
      }
   }

   // Set up data logger
   wam->logdivider = 1;
   PrepDL(&(wam->log),35); // Allocate space for up to 35 variables
   AddDataDL(&(wam->log),&(wam->log_time),sizeof(double),2,"Time");
   AddDataDL(&(wam->log),valptr_vn(wam->Jtrq),sizeof(btreal)*7,BTLOG_BTREAL,"Jtrq");
   AddDataDL(&(wam->log),valptr_vn(wam->Jpos),sizeof(btreal)*7,BTLOG_BTREAL,"Jpos");
   InitDL(&(wam->log),1000,"datafile.dat");

   // Read the tensioning/cycling parameters from a file
   readParameters("param.txt")
   m3 = Ts/P3;
   m4 = Ts/P4;
   Tt = Tt * 3600 / Ts;

   //printf("\nA3 = %lf, A4 = %lf, P3 = %lf, P4 = %lf, S3 = %lf, S4 = %lf\n",
   //   A3, A4, P3, P4, S3, S4);
      
   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Set the safety limits */
   setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous
   setProperty(0, 10, TL2, FALSE, 5400); //Eliminate torque faults in silly places
   setProperty(0, 10, TL1, FALSE, 1800); //Eliminate torque faults in silly places

   const_v3(wam->Cpoint, 0.0, 0.0, 0.0);

   /* Create a new trajectory */
   vt_j = new_vta(wam->dof,50);
   register_vta(&(wam->Jsc),vt_j);
   
   /* Initialize the jointspace PID controllers */
   jPID = (btPID*)btmalloc(wam->dof * sizeof(btPID));
   for(i = 0; i < wam->dof; i++){
      init_btPID(&jPID[i]);
      setdt_btPID(&jPID[i], Ts);
      setgains_btPID(&jPID[i], 1000.0, 5.0, 1.0);
      setsaturation_btPID(&jPID[i], 0.0);
   }
   
   /* Spin off the WAM control thread */
   wam_thd.period = Ts;
   registerWAMcallback(wam, WAMcallback);
   btthread_create(&wam_thd, 90, (void*)WAMControlThread, (void*)wam);

   printf("\n\nPlease activate the WAM by pressing Shift-Activate on the");
   printf("\ncontrol pendant, then press <Enter>");
   mygetch();

   // Turn on gravity compensation
   SetGravityComp(wam, 1.0);

   // Hold position
   setmode_bts(wam->active_sc,SCMODE_POS);

   // Allocate and zero a 7 element vector
   jdest = new_vn(7);

   // Move to initial position
   MoveWAM(wam, const_vn(jdest, 0.0, 0.0, A3*sin(x3)+S3, A4*sin(x4)+S4, 0.0, 0.0, 0.0));
   while(!MoveIsDone(wam)) 
      usleep(1000);

   // Start the local PID controller
   sety_btPID(&jPID[2], wam->Jpos->q[2]);
   sety_btPID(&jPID[3], wam->Jpos->q[3]);
   start_btPID(&jPID[2]);
   start_btPID(&jPID[3]);
   cycle = 1;
   while(1){
      
      
      
      if(t > Tt)
      {
         t = 0;
         //Tension(M4);
         //Move to start (0, -2pi, A3*sin(x3)+S3, A4*sin(x4)+S4);
      }
   }
   // MoveWAM to zero position
   //MoveSetup(wam, 0.5, 0.5);
   //MoveWAM(wam, jdest);

   //mygetch();
#if 0
   // Set low max torque

   // MoveWAM to initial tensioning position
   MoveWAM(wam, const_vn(jdest, -3.14, +2.2, -3.14, +3.5, 0.0, 0.0, 0.0));
   while(!MoveIsDone(wam))
      usleep(100000); // Sleep for 0.1s

   // MoveWAM to present position (stop pushing)
   MoveWAM(wam, wam->Jpos);
   while(!MoveIsDone(wam))
      usleep(100000); // Sleep for 0.1s

   // Back off J1 by the tension offset (TENSO) of M1

   // Back off J2 by the tension offset of M2

   // Back off J4 by the tension offset of M4

   // Activate solenoids M1, M2, M4
   setProperty(0, 1, TENSION, FALSE, 1);
   setProperty(0, 2, TENSION, FALSE, 1);
   setProperty(0, 4, TENSION, FALSE, 1);

   // Set tensioning max torque

   // MoveWAM to apply tension
   MoveWAM(wam, const_vn(jdest, -3.14, +2.2, <no change>, +3.5, 0.0, 0.0, 0.0));
   while(!MoveIsDone(wam))
      usleep(100000); // Sleep for 0.1s

   // Release solenoids M1, M2, M4
   setProperty(0, 1, TENSION, FALSE, 0);
   setProperty(0, 2, TENSION, FALSE, 0);
   setProperty(0, 4, TENSION, FALSE, 0);

   // Record tension totals

   // MoveWAM to present position (stop pushing)
   MoveWAM(wam, wam->Jpos);
   while(!MoveIsDone(wam))
      usleep(100000); // Sleep for 0.1s

   // Set normal max torque

   // Work the tension through
   for(i = 0; i < 8; i++)
   {
      MoveWAM(wam, const_vn(jdest, +2.5, -1.8, +2.5, -0.7, 0.0, 0.0, 0.0));
      while(!MoveIsDone(wam))
         usleep(100000); // Sleep for 0.1s
      MoveWAM(wam, const_vn(jdest, -2.5, +1.8, -2.5, +3.0, 0.0, 0.0, 0.0));
      while(!MoveIsDone(wam))
         usleep(100000); // Sleep for 0.1s
   }

#endif

   printf("\n\nExiting...\n\n");
   btthread_stop(&wam_thd); //Kill WAMControlThread
   exit(1);
}

int WAMcallback(struct btwam_struct *wam)
{
   btreal j3, j4;
   
   if(cycle){
      i3++;
      if(i3*Ts > P3)
         i3 = 0;
      x3 = 2 * pi * i3 * m3;
      
      i4++;
      if(i4*Ts > P4)
         i4 = 0;
      x4 = 2 * pi * i4 * m4;
      
      j3 = A3*sin(x3)+S3;
      j4 = A4*sin(x4)+S4;
      //printf("\rj3 = %lf, j4 = %lf", j3, j4);
      
      wam->Jtrq->q[2] = eval_btPID(&jPID[2], wam->Jpos->q[2], j3, Ts);
      wam->Jtrq->q[3] = eval_btPID(&jPID[3], wam->Jpos->q[3], j4, Ts);
      
      t++;
      //cycle = 0;
   }
   return 0;
}

/** Traps the Ctrl-C signal.
    Quits the program gracefully when Ctrl-C is hit.
*/
void sigint_handler()
{
   done = TRUE;
   
   printf("\n\nExiting...\n\n");
   btthread_stop(&wam_thd); //Kill WAMControlThread
   exit(1);
}

/** Return immediately after any key is pressed.
    You do not have to wait till the enter key is hit .
*/
int mygetch()
{
   /* Handles to old and new termios structures */
   struct termios oldt, newt;
   int ch;

   /* store current termios to restore back later */
   tcgetattr( STDIN_FILENO, &oldt );
   newt = oldt;

   /* set to canonical. turn off echo */
   newt.c_lflag &= ~( ICANON | ECHO );
   tcsetattr( STDIN_FILENO, TCSANOW, &newt );

   /* Read the character */
   ch = getchar();

   /* Reset terminal */
   tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
   return ch;
}

#if 0

Tension(M){
   M1(-J1, 0, 0, pi/2);
   M2(0, J2, -J3, pi/4);
   M3(0, -J2, -J3, pi/4);
   M4(0, 0, 0, -J4);

   Move to Mx();
   Activate tensioner
   Rotate until engaged
   Record
}
#endif

void readParameters(char *fn){
   parseFile(fn);
   
   sprintf(key, "J3_Amplitude");
   parseGetVal(DOUBLE, key, (void*)&A3);
   
   sprintf(key, "J4_Amplitude");
   parseGetVal(DOUBLE, key, (void*)&A4);
   
   sprintf(key, "J3_Period");
   parseGetVal(DOUBLE, key, (void*)&P3);
   
   sprintf(key, "J4_Period");
   parseGetVal(DOUBLE, key, (void*)&P4);
   
   sprintf(key, "J3_Shift");
   parseGetVal(DOUBLE, key, (void*)&S3);
   
   sprintf(key, "J4_Shift");
   parseGetVal(DOUBLE, key, (void*)&S4);

   A3 = A3 * pi / 180;
   A4 = A4 * pi / 180;
   S3 = S3 * pi / 180;
   S4 = S4 * pi / 180;
}
