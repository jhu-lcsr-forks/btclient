/*======================================================================*
 *  Module .............ex4-haptics
 *  File ...............main.c
 *  Author .............Brian Zenowich
 *  Creation Date ......19 Jan 2006
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file main.c
    \brief A demo of WAM haptics capabilities.
 
    Read the code to see what you can do with it.
    
This program allows a user to test and interact with the haptics
features of the WAM library.
 
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
#include "bthaptics.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
int useGimbals  = FALSE;
int done        = FALSE;

wam_struct      *wam;
btthread        wam_thd;

/******* Haptics *******/
btgeom_plane                planes[10];
bteffect_wall               wall[10];
bteffect_wickedwall         wickedwalls[10];

bteffect_magneticwall     magneticwalls[10];

bthaptic_object             objects[20];
btgeom_sphere               spheres[10];
bteffect_bulletproofwall    bpwall[10];
btgeom_box                  boxs[10];
vect_3                      *p1,*p2,*p3,*zero_v3;
bthaptic_scene              bth;
btgeom_state                pstate;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void init_haptics(void);
int mygetch();

/*==============================*
 * Functions                    *
 *==============================*/
/** Entry point for the application.
    Initializes the system and executes the main event loop.
*/
int main(int argc, char **argv)
{
   int     err;
   int     i;
   char    robotName[128];
   char    vect_buf1[250];
   double  x, y, z;
   int     busCount;

   /* Lead the user through a proper WAM startup */
   printf("\nMake sure the all WAM power and signal cables are securely");
   printf("\nfastened, then turn on the main power to WAM and press <Enter>");
   mygetch();

   printf("\n\nMake sure all E-STOPs are released, then press Shift-Idle");
   printf("\non the control pendant. Then press <Enter>");
   mygetch();

   printf("\n\nPlace WAM in its home (folded) position, then press <Enter>");
   mygetch();

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   err = ReadSystemFromConfig("../../wam.conf", &busCount);

   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned error: %d", err);
      exit(1);
   }

   /* Initialize and get a handle to the robot */
   if(!(wam = OpenWAM("../../wam.conf", 0)))
      exit(1);

   /* Check and handle any additional command line arguments */
   for(i = 1; i < argc-1; i++) {
      if(!strcmp(argv[i],"-g")) // If gimbals are being used
      {
         initGimbals(wam);
         useGimbals = TRUE;
         syslog(LOG_ERR, "Gimbals expected.");
      }
   }

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Set the safety limits */
   setSafetyLimits(0, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s

   // Set the puck torque safety limits (TL1 = Warning, TL2 = Critical)
   // Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c)
   // Note: btsystem.c bounds the outbound torque to 8191, so 9000
   // tell the safety system to never register a critical fault
   setProperty(0, SAFETY_MODULE, TL2, FALSE, 9000); //4700);
   setProperty(0, SAFETY_MODULE, TL1, FALSE, 2000); //1800

   // Initialize a haptic scene
   init_haptics();

   /* Spin off the WAM control thread */
   wam_thd.period = 0.002;
   btthread_create(&wam_thd,90,(void*)WAMControlThread1,(void*)wam);

   printf("\n\nPlease activate the WAM by pressing Shift-Activate on the");
   printf("\ncontrol pendant, then press <Enter>");
   mygetch();

   // Turn on gravity compensation
   SetGravityComp(wam, 1.1);

   // Turn on haptic scene
   bth.state = TRUE;

   printf("\n\nRunning -- Press Ctrl-C to quit\n");
   while (!done) {
      x = getval_vn(wam->R6pos, 0);
      y = getval_vn(wam->R6pos, 1);
      z = getval_vn(wam->R6pos, 2);
      printf("\rForce (Nm): %s x=%0.3f y=%0.3f z=%0.3f", sprint_v3(vect_buf1,wam->Cforce), x, y, z);


      // To turn off haptics in particular 3D space:

      //        if (y<0.1 && y>-0.1)
      //          bth.state = FALSE;
      //        else
      //          bth.state = TRUE;

      // Note: Turning off haptics applies to any space in 3D.
      //       x, y, and z variables have already been defined.
      //       Use them to establish spatial area of interest.



      fflush(stdout);
      usleep(100000); // Sleep for 0.1s
   }

   printf("\n\nExiting...\n\n");
   btthread_stop(&wam_thd); //Kill WAMControlThread
   exit(1);
}

int WAMcallback(struct btwam_struct *wam)
{
   eval_state_btg(&(pstate),wam->Cpos);

   //Velocity damping with force field:
   //Initialize a given point, using values ptx, pty, and ptz.
   //These values can be changed as desired.
   //Use this point as a reference for the area of least
   //resistance. The resistance will increase radially from
   //this point in 3D space in proportion to the distance between
   //the end of the arm and the reference point.


   vect_n* pt = new_vn(3);
   btreal ptx = 0.0;
   btreal pty = 0.0;
   btreal ptz = 0.0;
   setval_v3((vect_3*)pt, 0, ptx);
   setval_v3((vect_3*)pt, 1, pty);
   setval_v3((vect_3*)pt, 2, ptz);



   double Kscale = -100;

   //Velocity damping with radial force field, Kscale = -100
   //set_vn((vect_n*)wam->Cforce, add_vn((vect_n*)wam->Cforce, scale_vn(D_Pt2Pt((vect_3*)pt, wam->Cpos) * Kscale, (vect_n*)pstate.vel)));

   //Velocity damping with force field along y axis, Kscale = -100
   //set_vn((vect_n*)wam->Cforce, add_vn((vect_n*)wam->Cforce, scale_vn(fabs(getval_vn((vect_n*)wam->Cpos, 1)) * Kscale, (vect_n*)pstate.vel)));

   //Velocity damping, Kscale = -10;
   //set_vn((vect_n*)wam->Cforce, add_vn((vect_n*)wam->Cforce, scale_vn(Kscale, (vect_n*)pstate.vel)));
   eval_bthaptics(&bth,(vect_n*)wam->Cpos,(vect_n*)pstate.vel, (vect_n*)zero_v3, (vect_n*)wam->Cforce );
   apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce , wam->Ctrq);
   return 0;
}



void init_haptics(void)
{
   int cnt;
   btreal xorig,yorig,zorig;
   int objectCount = 0;

   p1 = new_v3();
   p2 = new_v3();
   p3 = new_v3();
   xorig = 0.0;
   yorig = 0.0;
   zorig = 0.10;

   new_bthaptic_scene(&bth,10);
   init_state_btg(&pstate,0.002,30.0);

   // Create workspace bounding box
   //    init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
   //    init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
   //    init_normal_box_bth(&objects[objectCount],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
   //    addobject_bth(&bth,&objects[objectCount++]);


   // Create nested spheres


   //    double nested spheres tangent to each other
   //    (use with wickedwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.02,zorig+0.0),1);
   //    init_sp_btg( &spheres[2],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.1,zorig+0.0),0);
   //    init_sp_btg( &spheres[3],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.12,zorig+0.0),1);

   //    init_sp_btg( &spheres[4],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[5],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.02,zorig+0.0),1);
   //    init_sp_btg( &spheres[6],cgetval_vn((vect_n*)wam->Cpos, 1)onst_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.1,zorig+0.0),0);
   //    init_sp_btg( &spheres[7],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.12,zorig+0.0),1);


   //    2 spheres tangent to each other
   //    (use with wickedwall)
   //      init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //      init_sp_btg( &spheres[1],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.02,zorig+0.0),1);

   //      init_sp_btg( &spheres[2],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //      init_sp_btg( &spheres[3],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.02,zorig+0.0),1);


   //    2 solid spheres tangent to each other
   //    (use with bulletproofwall or magneticwall)
   init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   init_sp_btg( &spheres[1],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);



   //    Overlapping spheres
   //    (use with wickedwall)
   //      init_sp_btg( &spheres[0],const_v3(p1,0.4,0.1,zorig+0.0),const_v3(p2,0.4,-0.12,zorig+0.0),0);
   //      init_sp_btg( &spheres[1],const_v3(p1,0.4,0.1,zorig+0.0),const_v3(p2,0.4,-0.1,zorig+0.0),1);

   //      init_sp_btg( &spheres[2],const_v3(p1,0.4,-0.1,zorig+0.0),const_v3(p2,0.4,0.12,zorig+0.0),0);
   //      init_sp_btg( &spheres[3],const_v3(p1,0.4,-0.1,zorig+0.0),const_v3(p2,0.4,0.1,zorig+0.0),1);





   //    Original parameters:
   //    (use with wickedwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1);
   //    init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1);




   for(cnt = 0;cnt < 2;cnt++) {
      //if number of spheres changes, must change value of cnt


      init_magneticwall(&magneticwalls[cnt],0.0,0.0,0.05,4000.0,10.0,10.0);
      //init_bulletproofwall(&bpwall[cnt],0.0,0.0,0.05,4000.0,10.0,10.0);
      //init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);

      init_normal_sphere_bth(&objects[objectCount],&spheres[cnt],(void*)&magneticwalls[cnt],magneticwall_nf);
      //remember to change type of wall within init_normal_sphere declaration here
      //if changing between bulletproofwall, wickedwall, magneticwall, etc.

      addobject_bth(&bth,&objects[objectCount++]);
   }

   const_v3(wam->Cpoint,0.0,-0.0,0.0);

   registerWAMcallback(wam,WAMcallback);

}



/** Traps the Ctrl-C signal.
    Quits the program gracefully when Ctrl-C is hit.
*/
void sigint_handler()
{
   done = TRUE;
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

