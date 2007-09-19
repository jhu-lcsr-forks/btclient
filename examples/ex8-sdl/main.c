/*======================================================================*
 *  Module .............btdiag
 *  File ...............btdiag.c
 *  Author .............Traveler Hauptman
 *                      Brian Zenowich
 *  Creation Date ......14 Oct 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
    This is the primary WAM demo application. It shows off many
    features of the WAM library.
    This application is also used for diagnostics and testing.
 *                                                                      *
 *======================================================================*/

/** \file btdiag.c
    \brief An interactive demo of WAM capabilities.
 
    Read the code to see what you can do with it.
    
This program allows a user to test and interact with the teach and play
features of the WAM library.
 
The user can switch between Cartesian space and joint space. The toggle 
variable is a pointer to the present btstatecontroller.
 
Note that if a trajectory is modified, you must call 's' scale trajectory on
it to properly establish the time values.
 
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
//#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <SDL/SDL.h>
#include "SDL/SDL_ttf.h"

#include <rtai_lxrt.h>
#include <rtai_sem.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"
#include "bthaptics.h"
#include "btserial.h"
//#include "aob.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
enum{SCREEN_MAIN, SCREEN_HELP};
#define Ts (0.002)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
typedef struct
{
   RTIME t;
   char c;
}
keyEventStruct;

typedef struct
{
   /* Per-WAM State */
   btstatecontrol *active_bts;
   vect_n *jdest;
   vect_n *cdest;
   vect_n *active_pos;
   vect_n *active_dest;
   vect_n *active_trq;
   via_trj_array **vta;
   via_trj_array *vt_j;
   via_trj_array *vt_c;
   btthread wam_thd;
   btgeom_state pstate;
}
wamData_struct;

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
wam_struct *wam[4];
wamData_struct wamData[4];

/* Global State */
int mode;
int constraint;
int haptics;
int pauseCnt = 0;
int scr = SCREEN_MAIN;
pthread_mutex_t disp_mutex;
int entryLine;
int useGimbals      = FALSE;
int done            = FALSE;
int quiet           = FALSE;
//int prev_mode;
int cteach = 0;
int NoSafety;
int angular = 0;
int cplay = 0;

/* Global data */
btthread audio_thd;
btthread disp_thd;
int busCount;
char *command_help[100];
int num_commands;
PORT p; // Serial port

/* Event logger */
int eventIdx;
RTIME eventStart;
keyEventStruct keyEvent[500];

/* Other */
double vel = 0.5, acc = 2.0;
char active_file[250];
char *user_def = "User edited point list";
matr_3 *r_mat;
vect_3 *xyz, *RxRyRz;

/******* Haptics *******/
btgeom_plane planes[10];
bteffect_wall wall[10];
bteffect_wickedwall wickedwalls[10];
bthaptic_object objects[20];
btgeom_sphere spheres[10];
bteffect_bulletproofwall bpwall[10];
btgeom_box boxs[10];
//bteffect_global myglobal;
vect_3 *p1,*p2,*p3,*zero_v3;
bthaptic_scene bth;

SDL_Event event; /* Event structure */
//Screen attributes
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const int SCREEN_BPP = 32;
//The font
TTF_Font *font = NULL;

//The color of the font
SDL_Color textColor = { 255, 255, 255 };
SDL_Surface *screen = NULL;
SDL_Surface *text = NULL;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void RenderMAIN_SCREEN(void);
void RenderHELP_SCREEN(void);
void RenderJOINTSPACE_SCREEN(void);
void RenderCARTSPACE_SCREEN(void);
void ProcessInput(int c);
void Shutdown(void);
void DisplayThread(void);
void AudioThread(void);
void StartDisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void init_haptics(void);
void read_keys(char *filename);
int  WAMcallback(struct btwam_struct *wam);
void ProcessKey(Uint8 *keystates);

/*==============================*
 * Functions                    *
 *==============================*/

void clean_up()
{
   //Free the surfaces
   if(text)
      SDL_FreeSurface( text );

   //Close the font
   if(font)
      TTF_CloseFont( font );

   //Quit SDL_ttf
   TTF_Quit();

   //Quit SDL
   SDL_Quit();
}

void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination)
{
   //Holds offsets
   SDL_Rect offset;
   SDL_Rect* clip = NULL;

   //Get offsets
   offset.x = x;
   offset.y = y;

   //Blit
   SDL_BlitSurface( source, clip, destination, &offset );
}

void sdl_waitforkey()
{
   int quit = 0;
   while( !quit ) {
      SDL_Event event;
      if( SDL_PollEvent( &event ) ) {
         switch( event.type ) {
         case SDL_KEYDOWN: {
               //int key = event.key.keysym.sym;
               //if( key == SDLK_ESCAPE ) {
               quit = 1;
               //}
               break;
            }
         }
      }
   }
}

void mvprintw(int line, int col, char *str)
{
   text = TTF_RenderText_Solid( font, str, textColor );
   if( text == NULL ) {
      return;
   }
   apply_surface(col*10,line*16,text,screen);
   return;
}

/** Entry point for the application.
    Initializes the system and executes the main event loop.
*/
int main(int argc, char **argv)
{
   char     chr,cnt;
   int      err;
   int      i, nout;
   struct   sched_param mysched;
   char     robotName[128];

   /* Figure out what the keys do and print it on screen.
    * Parses this source file for lines containing "case '", because
    * that is how we manage keypresses in ProcessInput().
    */
   //system("grep \"case '\" btdiag.c | sed 's/[[:space:]]*case \\(.*\\)/\\1/' > keys.txt");
   //read_keys("keys.txt");

   /* Initialize the ncurses screen library */
   //init_ncurses();
   //atexit((void*)endwin);


   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);
   syslog(LOG_ERR,"...Starting btdiag program...");

   /* Initialize the display mutex */
   test_and_log(
      pthread_mutex_init(&(disp_mutex),NULL),
      "Could not initialize mutex for displays.");

   /* Look through the command line arguments for "-q" */
   //mvprintw(18,0,"argc=%d",argc);
   //for(i = 0; i < argc; i++) mvprintw(20+i,0,"%s",argv[i]);
   for(i = 1; i < argc; i++) {
      if(!strcmp(argv[i],"-q"))
         quiet = TRUE; // Flag to skip the startup walkthrough text
   }

   /* Do we want to bypass the safety circuit + pendant?
    * This is for diagnostics only and should not normally be used .
    */
   NoSafety = 0;
   for(i = 1; i < argc; i++) {
      if(!strcmp(argv[i],"-ns"))
         NoSafety = 1;
   }

   //Initialize all SDL subsystems
   if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 ) {
      clean_up();
      return 0;
   }
   //Set up the screen
   screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

   //If there was an error in setting up the screen
   if( screen == NULL ) {
      return 0;
   }

   //Initialize SDL_ttf
   if( TTF_Init() == -1 ) {
      return 0;
   }

   //Set the window caption
   SDL_WM_SetCaption( "Cartesian Flight", NULL );

   //Open the font
   font = TTF_OpenFont( "/usr/X11R6/lib/X11/fonts/TTF/VeraMono.ttf", 12 );
   //If there was an error in loading the font
   if( font == NULL ) {
      clean_up();
      printf("TTF_OpenFont: %s\n", SDL_GetError());
      return 0;
   }
   SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0x00, 0x00, 0x00 ) );

   if(!quiet) {
      /* Lead the user through a proper WAM startup */
      mvprintw(1,0,"Make sure the all WAM power and signal cables are securely");
      mvprintw(2,0,"fastened, then turn on the main power to WAM and press <Enter>");
      SDL_Flip( screen );
      sdl_waitforkey();
      mvprintw(4,0,"Make sure all E-STOPs are released, then press Shift-Idle");
      mvprintw(5,0,"on the control pendant. Then press <Enter>");
      SDL_Flip( screen );
      sdl_waitforkey();
      mvprintw(7,0,"Place WAM in its home (folded) position, then press <Enter>");
      SDL_Flip( screen );
      sdl_waitforkey();
   }
   clearScreen();

   /* Read the WAM configuration file */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig returned err = %d", err);
      exit(1);
   }

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned err = %d", err);
      exit(1);
   }

   /* Initialize and get a handle to the robot(s) */
   for(i = 0; i < busCount; i++) {
      if(!(wam[i] = OpenWAM("../../wam.conf", i)))
         exit(1);
   }

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Set the safety limits for each bus (WAM) */
   for(i = 0; i < busCount; i++) {
      if(!NoSafety) {
         /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
          * For now, the joint and tip velocities are ignored and
          * the elbow velocity provided is used for all three limits.
          */
         setSafetyLimits(i, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s

         /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical)
          * Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c) 
          * Note: btsystem.c bounds the outbound torque to 8191, so 9000
          * tells the safety system to never register a critical fault
          */
         setProperty(i, SAFETY_MODULE, TL2, FALSE, 9000); //4700);
         setProperty(i, SAFETY_MODULE, TL1, FALSE, 2000); //1800
      }

      /* Prepare the WAM data */
      wamData[i].jdest = new_vn(len_vn(wam[i]->Jpos));
      wamData[i].cdest = new_vn(len_vn(wam[i]->R6pos));
      //wv = new_vn(7);

      /* The WAM can be in either Joint mode or Cartesian mode.
       * We want a single set of variables (active_) to eliminate the need
       * for a whole bunch of if() statements.
       */
      wamData[i].active_bts = &(wam[i]->Jsc);
      setmode_bts(wamData[i].active_bts,SCMODE_IDLE);
      wamData[i].active_pos = wam[i]->Jpos;
      wamData[i].active_trq = wam[i]->Jtrq;
      wamData[i].active_dest = wamData[i].jdest;

      /* Create a new trajectory */
      wamData[i].vt_j = new_vta(len_vn(wam[i]->Jpos),50);
      wamData[i].vt_c = new_vta(len_vn(wam[i]->R6pos),50);
      wamData[i].vta = &wamData[i].vt_j;
      register_vta(wamData[i].active_bts,*wamData[i].vta);

      /* Initialize the control period */
      wamData[i].wam_thd.period = Ts;

      /* Register the control loop's local callback routine */
      registerWAMcallback(wam[i], WAMcallback);
   }
   r_mat = new_m3();
   xyz = new_v3();
   RxRyRz = new_v3();

   /* Initialize the haptic scene */
   init_haptics();

   /* Spin off the WAM control thread(s) */
   btthread_create(&wamData[0].wam_thd, 90, (void*)WAMControlThread1, (void*)wam[0]);
   //btthread_create(&wamData[1].wam_thd, 90, (void*)WAMControlThread2, (void*)wam[1]);

   /* Initialize the active teach filename (to blank) */
   active_file[0] = 0;

   /* Open serial port */
   if(err = serialOpen(&p, "/dev/ttyS0")) {
      syslog(LOG_ERR, "Error opening serial port: %d", err);
   }
   serialSetBaud(&p, 9600); // The BarrettHand defaults to 9600 baud

   /* Spin off the display thread */
   btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

   /* Spin off the audio thread */
   //btthread_create(&audio_thd,0,(void*)AudioThread,NULL);

   /* Main event loop, ~10Hz */
   while (!done) {
      /* Check the active trajectory for completion for each bus */
      for(i = 0; i < busCount; i++) {
         if (get_trjstate_bts(wamData[i].active_bts) == BTTRAJ_DONE && !wamData[i].active_bts->loop_trj) {  // BZ-16Nov2005
            stop_trj_bts(wamData[i].active_bts);
            //setmode_bts(active_bts,prev_mode);
            cplay = 0;
         }

         /* Handle the data logger */
         //evalDL(&(wam[i]->log));
      }
      /* Check and handle user keypress */
      //if ((chr = getch()) != ERR)
      //   ProcessInput(chr);
      //Get the keystates


      /* Check for events */
      //while(SDL_PollEvent(&event)) {  /* Loop until there are no events left on the queue */
      //}
      SDL_PumpEvents();
      Uint8 *keystates = SDL_GetKeyState( NULL );
      ProcessKey(keystates);

      /* If we are in playback mode, handle the BarrettHand teach commands */
      if(cplay) {
         if(keyEvent[eventIdx].c) {
            if((rt_get_cpu_time_ns() - eventStart) > keyEvent[eventIdx].t) {
               ProcessInput(keyEvent[eventIdx].c);
               ++eventIdx;
            }
         }
      }

      /* If the WAM has paused due to obstruction, wait */
      if(pauseCnt > 0) {
         pauseCnt--;
         if(pauseCnt == 0) {
            for(i = 0; i < busCount; i++) {
               unpause_trj_bts(wamData[i].active_bts,0.125);
            }
         }
      }

      /* If there is an obstruction, pause the WAM playback */
      for(i = 0; i < busCount; i++) {
         for(cnt=0;cnt<wam[i]->dof;cnt++) {
            if(fabs(getval_vn(wam[i]->Jtrq,cnt) - getval_vn(wam[i]->Gtrq,cnt)) >
                    getval_vn(wam[i]->torq_limit,cnt)) {
               pauseCnt = 50;
               pause_trj_bts(wamData[i].active_bts,5);
               break;
            }
         }
      }

      /* Sleep for 0.1s. This roughly defines the event loop frequency */
      usleep(100000);
   }

   /* Clean up and exit */
   for(i = 0; i < busCount; i++) {
      btthread_stop(&wamData[i].wam_thd); //Kill WAMControlThread
      //CloseDL(&(wam[i]->log));
   }
   //DecodeDL("datafile.dat","dat.csv",1);

   //Quit SDL
   clean_up();

   exit(1);
}

/* This function is called from the WAMControlThread() after the positions
 * have been received from the WAM (and after all the kinematics are calculated)
 * but before torques are sent to the WAM.
 */
int WAMcallback(struct btwam_struct *w)
{
   int i;
#if 0
   //R_to_q(q, w->robot.tool->origin);
   //q_to_R(r_mat, q);
   if(getmode_bts(w->active_sc) == SCMODE_POS && w->active_sc == &w->Csc)
   {
      // Reference
      //setrange_vn((vect_n*)xyz, w->R6ref, 0, 3, 3);
      //XYZftoR_m3(r_mat, xyz);
      //getcol_m3(ns, r_mat, 0);
      //getcol_m3(os, r_mat, 1);
      //getcol_m3(as, r_mat, 2);

      // Actual
      //setrange_vn((vect_n*)xyz, w->R6pos, 0, 3, 3);
      //XYZftoR_m3(r_mat, xyz);
      getcol_m3(n, w->robot.tool->origin, 0);
      getcol_m3(o, w->robot.tool->origin, 1);
      getcol_m3(a, w->robot.tool->origin, 2);

      setrange_vn(e, (vect_n*)/*matXvec_m3(w->robot.tool->origin,*/( scale_v3(0.5, add_v3(cross_v3(ns,n), add_v3(cross_v3(os,o), cross_v3(as,a))))), 3, 0, 3);
      set_vn(ed, scale_vn(1/Ts, add_vn(e, scale_vn(-1.0, last_e))));
      set_vn(f6, add_vn(matXvec_mn(kp, e, e->ret), matXvec_mn(kd, ed, ed->ret)));
      setrange_vn(w->R6force, f6, 3, 3, 3); // Just for show
      setrange_vn((vect_n*)t3, f6, 0, 3, 3);
      apply_tool_force_bot(&(w->robot), w->Cpoint, f3, t3);
      set_vn(last_e, e);
   }
#endif

   /* Handle haptic scene for the specified WAM */
   for(i = 0; i < busCount; i++)
   {
      if(wam[i] == w) {
         /* Filter the Cartesian endpoint position to generate the endpoint
          * velocity and acceleration.
          */
         eval_state_btg(&wamData[i].pstate, w->Cpos);

         /* Evaluate the haptic scene.
          * Uses the WAM's position and velocity along with the scene definition
          * to determine the required end-of-arm force vector.
          */
         eval_bthaptics(&bth, (vect_n*)w->Cpos, (vect_n*)wamData[i].pstate.vel, (vect_n*)zero_v3, (vect_n*)w->Cforce);

         /* Apply the calculated force vector to the robot */
         apply_tool_force_bot(&(w->robot), w->Cpoint, w->Cforce, w->Ctrq);
      }
   }

   return 0;
}

/* Parse the input file into an array of strings to be displayed on the help screen */
void read_keys(char *filename)
{
   FILE *inf;
   int done = 0;
   int cnt = 0;
   int len = 100;
   int ret;

   num_commands = 0;
   inf = fopen(filename,"r");
   ret = getline(&(command_help[num_commands]),&len,inf);
   if (inf != NULL) {
      while (!done) {
         command_help[num_commands] = (char*)btmalloc(100);
         ret = getline(&(command_help[num_commands]),&len,inf);
         if (ret == -1 || num_commands > 98)
            done = 1;
         else
            command_help[num_commands][ret-1] = 0;
         num_commands++;
      }
   }
   fclose(inf);
   inf = fopen("test.out","w");
   for(cnt = 0;cnt < num_commands;cnt++)
      fprintf(inf,"%s",command_help[cnt]);
   fclose(inf);
}

/* Initialize the haptic scene with various objects */
void init_haptics(void)
{
   int cnt, i;
   btreal xorig,yorig,zorig;
   int objectCount = 0;

   /* Define some variables to store point data */
   p1 = new_v3();
   p2 = new_v3();
   p3 = new_v3();

   /* Define the offset from the origin */
   xorig = 0.0;
   yorig = 0.0;
   zorig = 0.10;

   /* Allocate a scene with space for 10 objects */
   new_bthaptic_scene(&bth, 10);

   /* For each WAM (bus), initialize a structure to track velocity and accel, given position */
   for(i = 0; i < busCount; i++) {
      /* Initialize the position filter (btgeometry) */
      init_state_btg(&wamData[i].pstate, Ts, 30.0); // Update rate, filter cutoff Hz

      /* Define the Haptic interaction point with respect to the WAM tool frame */
      const_v3(wam[i]->Cpoint, 0.0, 0.0, 0.0); // Define the interaction point to be at the tool
   }

   /* To add a haptic object to the scene, you must:
    * 1) Define the object geometry
    * 2) Define a type of haptic interaction (method of force response)
    * 3) Tie the object geometry and haptic interaction together into a haptic object
    * 4) Add the new haptic object to the scene
    */

   /* Create workspace bounding box */
   init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
   init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
   init_normal_box_bth(&objects[objectCount],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
   addobject_bth(&bth,&objects[objectCount++]);

   /* Create nested spheres */
   init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0); // Inner sphere, outer wall
   init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1); // Inner sphere, inner wall
   init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0); // Outer sphere, outer wall
   init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1); // Outer sphere, inner wall
   /* Perform steps 2-4 (above) for each sphere */
   for(cnt = 0;cnt < 4;cnt++) {
      init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
      init_normal_sphere_bth(&objects[objectCount],&spheres[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
      addobject_bth(&bth,&objects[objectCount++]);
   }
}

/* Initialize the ncurses screen library */
void init_ncurses(void)
{
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
}

/** Traps the Ctrl-C signal.
    Quits the program gracefully when Ctrl-C is hit.
*/
void sigint_handler()
{
   clean_up();
   exit(1);
}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
   /* Clear the screen buffer */
   clear();

   /* Display the cleared screen */
   refresh();

   /* Loop forever, rendering the appropriate screen information */
   while (!done) {
      /* Try to obtain a mutex lock to refresh the screen.
       * The only time the mutex is unavailable is when the user is
       * typing an answer to an on-screen prompt.
       * See start_entry() and finish_entry()
       */
      test_and_log(
         pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");

      /* Render the appropriate screen, based on the "screen" variable */
      switch(scr) {
      case SCREEN_MAIN:
         RenderMAIN_SCREEN();
         break;
      case SCREEN_HELP:
         RenderHELP_SCREEN();
         break;
      }

      /* Release the mutex lock */
      pthread_mutex_unlock(&(disp_mutex));

      /* Slow this loop down to about 10Hz */
      usleep(100000);
   }

}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
   int err;
   test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
   move(entryLine, 1);
   echo();
   timeout(-1);
}

/** Unlocks the display mutex.
    Allows the computer to resume automatically updating the screen.
*/
void finish_entry()
{
   noecho();
   timeout(0);
   move(entryLine, 1);
   addstr("                                                                              ");
   refresh();
   pthread_mutex_unlock( &(disp_mutex) );
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderMAIN_SCREEN()
{
   int cnt, idx, Mid, cp;
   int line, line2;
   int cpt, nrows;
   double gimb[4],tacc,tvel;
   vectray* vr;
   char vect_buf1[2500];
   char txt[250];

   SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0x00, 0x00, 0x00 ) );

   /***** Display the interface text *****/
   line = 0;

   mvprintw(line , 0, "Barrett Technology - Diagnostic Application        Press 'h' for help");
   line+=2;

   // Show MODE
   if (wamData[0].active_bts == &(wam[0]->Jsc)) {
      mvprintw(line, 0, "Mode       : Joint Space    ");
   } else if (wamData[0].active_bts == &(wam[0]->Csc)) {
      mvprintw(line, 0, "Mode       : Cartesian Space");
   } else {
      mvprintw(line, 0, "Mode       : Undefined!!!   ");
   }
   ++line;

   // Show CONSTRAINT
   if (getmode_bts(wamData[0].active_bts)==SCMODE_IDLE)
      mvprintw(line, 0, "Constraint : IDLE      ");
   else if (getmode_bts(wamData[0].active_bts)==SCMODE_POS)
      mvprintw(line, 0, "Constraint : POSITION  ");
   else if (getmode_bts(wamData[0].active_bts)==SCMODE_TRJ)
      mvprintw(line, 0, "Constraint : TRAJECTORY");
   else
      mvprintw(line, 0, "Constraint : UNDEFINED!");
   ++line;

   // Show the state of the haptic scene
   if (bth.state) {
      mvprintw(line, 0, "Haptics    : ON    ");
   } else {
      mvprintw(line, 0, "Haptics    : OFF    ");
   }
   line+=1;

   // Show TRAJECTORY
   if (cteach)
      mvprintw(line, 0, "Trajectory : Teaching continuous trajectoy");
   else if (*wamData[0].vta == NULL)
      mvprintw(line, 0, "Trajectory : NONE                         ");
   else {
      sprintf(txt, "Trajectory : %s                           ",*active_file?active_file:"NONE");
      mvprintw(line, 0, txt);
   }
   ++line;
   ++line;

   for(cnt = 0; cnt < busCount; cnt++) {
      sprintf(txt, "Name       : %s", wam[cnt]->name);
      mvprintw(line, 0, txt);
      ++line;
      /*
      mvprintw(line, 0, "Velocity   : %+8.4f  ",vel);
      ++line;
      mvprintw(line, 0, "Accel      : %+8.4f  ",acc);
      ++line;
      */
      sprintf(txt, "Destination: %s ",sprint_vn(vect_buf1,wamData[cnt].active_dest));
      mvprintw(line, 0, txt);
      line+=1;
      sprintf(txt, "Position   : %s ", sprint_vn(vect_buf1,wamData[cnt].active_pos));
      mvprintw(line, 0, txt);
      ++line;
      sprintf(txt, "RzRyRz     : %s ", sprint_vn(vect_buf1,(vect_n*)RtoZYZf_m3(wam[cnt]->robot.tool->origin, RxRyRz)));
      mvprintw(line, 0, txt);
      ++line;
      sprintf(txt, "R6ref      : %s ", sprint_vn(vect_buf1,wam[cnt]->R6ref));
      mvprintw(line, 0, txt);
      ++line;
      /*
      mvprintw(line, 0, "origin     : \n%s ", sprint_mn(vect_buf1,(matr_mn*)wam[cnt]->robot.tool->origin));
      line+=5;

      mvprintw(line, 0, "q->R       : \n%s ", sprint_mn(vect_buf1,(matr_mn*)r_mat));
      line+=5;
      */
      //mvprintw(line, 0, "Target     : %s ", sprint_vn(vect_buf1,active_bts->qref));
      //++line;
      sprintf(txt, "%s     : %s ", (wamData[cnt].active_bts == &(wam[cnt]->Jsc)) ? "Torque" : "Force ", sprint_vn(vect_buf1,wamData[cnt].active_trq));
      mvprintw(line, 0, txt);
      line+=2;

      if (*wamData[cnt].vta != NULL) { // print current point
         vr = get_vr_vta(*wamData[cnt].vta);
         cpt = get_current_idx_vta(*wamData[cnt].vta);
         nrows = numrows_vr(vr);
         sprintf(txt, "Teach Point: %d of %d      ",cpt,nrows-1);
         mvprintw(line,0,txt);
         line++;

         mvprintw(line  , 0 , "Previous   :                                              ");
         mvprintw(line+1, 0 , "Current    :                                              ");
         mvprintw(line+2, 0 , "Next       :                                              ");

         // Previous
         if (nrows > 0 && cpt > 0) {
            sprintf(txt, "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));
            mvprintw(line, 13,txt);
         }
         // Current
         if (nrows > 0) {
            if (nrows != cpt) {
               sprintf(txt, "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
               mvprintw(line+1, 13 , txt);
            } else
               mvprintw(line+1, 13 , "END OF LIST");
         } else
            mvprintw(line+1, 13 , "EMPTY LIST");

         // Next
         if (nrows > 1) {
            if (cpt < nrows-1) {
               sprintf(txt, "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
               mvprintw(line+2, 13,txt);
            } else if (cpt == nrows-1)
               mvprintw(line+2, 13, "END OF LIST");
         }
         line += 3;
      } else {
         line++;
         line++;
         mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
         line += 2;
      }
      line += 2;
   }


   entryLine = line;
   //refresh();
   SDL_Flip( screen );
}

/* Show the available commands on-screen */
void RenderHELP_SCREEN()
{
   int cnt, line = 0;
   char txt[250];

   SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0x00, 0x00, 0x00 ) );

   mvprintw(line, 0, "Help Screen - (press 'h' to toggle)");
   line += 2;
   for (cnt = 0; cnt < num_commands;cnt++) {
      if (cnt % 2) {
         sprintf(txt, "%.39s",command_help[cnt]);
         mvprintw(line,40,txt);
         line += 1;
      } else {
         sprintf(txt, "%.39s",command_help[cnt]);
         mvprintw(line,0,txt);

      }
   }
   SDL_Flip( screen );
   //refresh();
}

/* Clear the screen while honoring the mutex lock */
void clearScreen(void)
{
   btmutex_lock(&(disp_mutex));
   //clear();
   SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0x00, 0x00, 0x00 ) );
   SDL_Flip( screen );
   btmutex_unlock(&(disp_mutex));
}

void ProcessKey(Uint8 *keystates)
{
   int cnt,elapsed = 0, i;
   double ftmp,tacc,tvel;
   int dtmp,status;

   char fn[250],fn2[250],chr;
   int ret;
   int done1;
   btreal zPos;

   if( keystates[ SDLK_x ] ) {
      done = 1;
      return;
   }
   if( keystates[ SDLK_EXCLAIM ] ) {
      if (NoSafety)
         EnergizeActuators();
   }
   if( keystates[ SDLK_AT ] ) {
      if (NoSafety)
         IdleActuators();
   }
   if( keystates[ SDLK_HASH ] ) {
      for(i = 0; i < busCount; i++) {
         DLon(&(wam[i]->log));
      }
   }

   if( keystates[ SDLK_g ] ) {
      start_entry();
      addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
      refresh();
      scanw("%lf\n",  &tvel);
      for(i = 0; i < busCount; i++) {
         SetGravityComp(wam[i],tvel);
      }
      finish_entry();
   }
#if 0
   if( keystates[ SDLK_0 ] ) {
      serialWriteString(&p, "\rHI\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_9 ] ) {
      serialWriteString(&p, "\rSM 1500\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_8 ] ) {
      serialWriteString(&p, "\rSM 1000\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_7 ] ) {
      serialWriteString(&p, "\rSO\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_6 ] ) {
      serialWriteString(&p, "\rSC\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_5 ] ) {
      serialWriteString(&p, "\rGO\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
   if( keystates[ SDLK_4 ] ) {
      serialWriteString(&p, "\rGC\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
   }
#endif
   if( keystates[ SDLK_3 ] ) {
      for(i = 0; i < busCount; i++) {
         DLoff(&(wam[i]->log));
      }
   }

   if( keystates[ SDLK_QUESTION ] ) {
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {
            moveparm_bts(wamData[i].active_bts,vel,acc);
            wamData[i].active_bts->loop_trj = 1;
            if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
               setmode_bts(wamData[i].active_bts,SCMODE_POS);
            start_trj_bts(wamData[i].active_bts);
         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
   }
   if( keystates[ SDLK_PERIOD ] ) {
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {

            cplay = 1;
            eventStart = rt_get_cpu_time_ns();
            eventIdx = 0;

            moveparm_bts(wamData[i].active_bts,vel,acc);
            wamData[i].active_bts->loop_trj = 0;
            if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
               setmode_bts(wamData[i].active_bts,SCMODE_POS);

            start_trj_bts(wamData[i].active_bts);

         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
   }
   if( keystates[ SDLK_p ] ) {
      for(i = 0; i < busCount; i++) {
         if (getmode_bts(wamData[i].active_bts)!=SCMODE_IDLE)
            setmode_bts(wamData[i].active_bts,SCMODE_IDLE);
         else
            setmode_bts(wamData[i].active_bts,SCMODE_POS);
      }
   }
   if( keystates[ SDLK_d ] ) {
      SDLMod mod = SDL_GetModState();
      if( mod & KMOD_SHIFT ) {
         bth.state = 1;
      } else {
         bth.state = 0;
      }
   }
   if( keystates[ SDLK_TAB ] ) {
      for(i = 0; i < busCount; i++) {
         destroy_vta(wamData[i].vta); //empty out the data if it was full
         setmode_bts(&(wam[i]->Jsc),SCMODE_IDLE);
         setmode_bts(&(wam[i]->Csc),SCMODE_IDLE);

         if (wamData[i].active_bts == &(wam[i]->Jsc)) { //switch to cartesian space mode.
            SetCartesianSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Csc);
            wamData[i].active_pos = wam[i]->R6pos;
            wamData[i].active_trq = wam[i]->R6force;
            wamData[i].active_dest = wamData[i].cdest;
            wamData[i].vta = &wamData[i].vt_c;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         } else {
            SetJointSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Jsc);
            wamData[i].active_pos = wam[i]->Jpos;
            wamData[i].active_trq = wam[i]->Jtrq;
            wamData[i].active_dest = wamData[i].jdest;
            wamData[i].vta = &wamData[i].vt_j;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      }
      clearScreen();
   }
   if( keystates[ SDLK_UNDERSCORE ] ) {
      clearScreen();
   }

   //case 'b'://Simulate loaded trajectory

   //sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
   //break;

   if( keystates[ SDLK_h ] ) {
      clearScreen();
      scr = !scr;
   }
   if( keystates[ SDLK_COMMA ] ) {
      for(i = 0; i < busCount; i++) {
         status =  movestatus_bts(wamData[i].active_bts);
         if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
            unpause_trj_bts(wamData[i].active_bts,2);
         else
            pause_trj_bts(wamData[i].active_bts,2);
      }
   }
   if( keystates[ SDLK_m ] ) {
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter comma seperated destination \".2,.4,...\": ");
         refresh();
         getstr( fn);
         strcat(fn,"\n");
         //syslog(LOG_ERR,"Moveto:%s",fn);
         finish_entry();
         fill_vn(wamData[0].active_dest,0.25);
         csvto_vn(wamData[0].active_dest,fn);
         if(wamData[0].active_bts == &(wam[cnt]->Csc)) {
            // Convert from RxRyRz to RotMatrix
            // active_dest[3] = Rx;
            // active_dest[4] = Ry;
            // active_dest[5] = Rz;
            setrange_vn((vect_n*)xyz, wamData[0].active_dest, 0, 3, 3);
            XYZftoR_m3(r_mat, xyz);
            getcol_m3(xyz, r_mat, 0);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 3, 0, 3);
            getcol_m3(xyz, r_mat, 1);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 6, 0, 3);
            getcol_m3(xyz, r_mat, 2);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 9, 0, 3);
         }

         moveparm_bts(wamData[0].active_bts,vel,acc);
         if (getmode_bts(wamData[0].active_bts) != SCMODE_POS)
            setmode_bts(wamData[0].active_bts,SCMODE_POS);

         if(moveto_bts(wamData[0].active_bts,wamData[0].active_dest))
            syslog(LOG_ERR,"Moveto Aborted");

      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
   }
   if( keystates[ SDLK_n ] ) {
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter the max number of points that will be in your trajectory: ");
         refresh();
         ret = scanw("%d", &dtmp);

         for(i = 0; i < busCount; i++) {
            destroy_vta(wamData[i].vta);
            strcpy(active_file, user_def);
            *wamData[i].vta = new_vta(len_vn(wamData[i].active_pos), dtmp);
            register_vta(wamData[i].active_bts, *wamData[i].vta);
         }
         active_file[0] = 0;
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
   }
   if( keystates[ SDLK_w ] ) {
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);
         for(i = 0; i < busCount; i++) {
            sprintf(fn, "%s_%d", active_file, i);
            if (*wamData[i].vta != NULL) {
               write_file_vta(*wamData[i].vta, fn);
            }
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
   }
   if( keystates[ SDLK_l ] ) {
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);

         for(i = 0; i < busCount; i++) {
            sprintf(fn, "%s_%d", active_file, i);
            destroy_vta(wamData[i].vta); //empty out the data if it was full
            *wamData[i].vta = read_file_vta(fn, 20);
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
   }
   if( keystates[ SDLK_y ] ) {
      SDLMod mod = SDL_GetModState();
      if( mod & KMOD_SHIFT ) {
         for(i = 0; i < busCount; i++) {
            sprintf(fn, "teachpath_%d", i);
            if (wamData[i].active_bts == &(wam[i]->Jsc))
               StartContinuousTeach(wam[i], 1, 25, fn);
            else
               StartContinuousTeach(wam[i], 0, 25, fn);
         }

         cteach = 1;
         eventIdx = 0;
         eventStart = rt_get_cpu_time_ns()-750000000L;
         keyEvent[eventIdx].c = 0;
      } else {
         for(i = 0; i < busCount; i++) {
            StopContinuousTeach(wam[i]);
            sprintf(fn, "teachpath_%d", i);
            sprintf(fn2, "teach_%d.csv", i);
            DecodeDL(fn, fn2, 0);
            cteach = 0;
            stop_trj_bts(wamData[i].active_bts);
            /** \internal \todo sleeps that are necessary might be forgotten. Can we eliminate the need?*/
            usleep(10000); //needed to give the command a chance to work.
            destroy_vta(wamData[i].vta); //empty out the data if it was full
            strcpy(active_file,"teach.csv");
            *wamData[i].vta = read_file_vta(fn2, 20);
            register_vta(wamData[i].active_bts, *wamData[i].vta);
         }
      }

   }
   if( keystates[ SDLK_SLASH ] ) {
      for(i = 0; i < busCount; i++) {
         stop_trj_bts(wamData[i].active_bts);
         //setmode_bts(active_bts,prev_mode);
         wamData[i].active_bts->loop_trj = 0;
      }
   }
#if 0
case 'N'://Toggle angular hold
   if(angular) {
      angular = 0;
      for(i = 0; i < busCount; i++) {
         stop_btPID(&(wam[i]->pid[3]));
      }
   } else {
      angular = 1;
      for(i = 0; i < busCount; i++) {
         set_q(wam[i]->qref, wam[i]->qact);
         start_btPID(&(wam[i]->pid[3]));
      }
   }
   break;


   //'m': Move to the presently selected trajectory point

case '<'://Select next trajectory point
   for(i = 0; i < busCount; i++) {
      prev_point_vta(*wamData[i].vta);
   }
   break;
case '>'://Select previous trajectory point
   for(i = 0; i < busCount; i++) {
      next_point_vta(*wamData[i].vta);
   }
   break;
case '+'://Insert a point in the trajectory
   for(i = 0; i < busCount; i++) {
      if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
         ins_point_vta(*wamData[i].vta, wamData[i].active_pos);
      }
   }
   break;
case '-'://Remove a point in the trajectory
   for(i = 0; i < busCount; i++) {
      if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
         del_point_vta(*wamData[i].vta);
      }
   }
   break;
case 's'://Adjust trj times using a velocity
   if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
      start_entry();
      addstr("Enter trajectory velocity: ");
      refresh();
      ret = scanw("%lf\n", &tvel);
      for(i = 0; i < busCount; i++) {
         if(*wamData[i].vta != NULL)
            dist_adjust_vta(*wamData[i].vta,tvel);
      }
      finish_entry();
   }
   break;
case 'S'://Scale trajectory in time
   if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
      start_entry();
      addstr("Enter scale factor: ");
      refresh();
      ret = scanw("%lf\n", &tvel);
      for(i = 0; i < busCount; i++) {
         if(wamData[i].vta != NULL)
            time_scale_vta(*wamData[i].vta,tvel);
      }
      finish_entry();
   }
   break;
case 'A'://Set the corner acceleration
   if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
      start_entry();
      addstr("Enter Corner Acceleration: ");
      refresh();
      ret = scanw("%lf\n", &tacc);
      for(i = 0; i < busCount; i++) {
         if(*wamData[i].vta != NULL)
            set_acc_vta(*wamData[i].vta, tacc);
      }
      finish_entry();
   }
   break;
case 'a'://Set the move acceleration
   if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
      start_entry();
      addstr("Enter Move Acceleration: ");
      refresh();
      ret = scanw("%lf\n", &acc);
      finish_entry();
   }
   break;
case 'v'://Set the move velocity
   if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
      start_entry();
      addstr("Enter Move Velocity: ");
      refresh();
      ret = scanw("%lf\n", &vel);
      finish_entry();
   }
   break;
#endif

}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
   int cnt,elapsed = 0, i;
   double ftmp,tacc,tvel;
   int dtmp,status;

   char fn[250],fn2[250],chr;
   int ret;
   int done1;
   btreal zPos;

   switch (c)
   {

   case 'x'://eXit
   case  'X'://eXit
      done = 1;
      break;
   case '!'://Set puck to mode TORQ
      if (NoSafety)
         EnergizeActuators();
      //setProperty(0, 1, MODE, FALSE, 2);
      break;
   case '@'://Set puck to mode IDLE
      if (NoSafety)
         IdleActuators();
      //setProperty(0, 1, MODE, FALSE, 0);
      break;
   case '#'://Data logging on
      for(i = 0; i < busCount; i++) {
         DLon(&(wam[i]->log));
      }
      break;
   case '3'://Data logging off
      for(i = 0; i < busCount; i++) {
         DLoff(&(wam[i]->log));
      }
      break;

   case '4'://BHand GC
      serialWriteString(&p, "\rGC\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '5'://BHand GO
      serialWriteString(&p, "\rGO\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         syslog(LOG_ERR, "keyEvent[%d].c = %d, keyEvent[%d].t = %lld", eventIdx, c, eventIdx, keyEvent[eventIdx].t);
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '6'://BHand SC
      serialWriteString(&p, "\rSC\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '7'://BHand SO
      serialWriteString(&p, "\rSO\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '8'://BHand SM 1000
      serialWriteString(&p, "\rSM 1000\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '9'://BHand SM 1500
      serialWriteString(&p, "\rSM 1500\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;
   case '0'://BHand HI
      serialWriteString(&p, "\rHI\r");
      if(cteach) {
         keyEvent[eventIdx].c = c;
         keyEvent[eventIdx].t = rt_get_cpu_time_ns() - eventStart;
         eventIdx++;
         keyEvent[eventIdx].c = 0;
      }
      break;

   case 'g'://Set gravity compensation
      //gravity = !gravity;
      start_entry();
      addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
      refresh();
      scanw("%lf\n",  &tvel);
      for(i = 0; i < busCount; i++) {
         SetGravityComp(wam[i],tvel);
      }
      finish_entry();
      break;
   case '_'://Refresh display
      clearScreen();
      break;
   case '\t'://Toggle jointspace and cartesian space
      for(i = 0; i < busCount; i++) {
         destroy_vta(wamData[i].vta); //empty out the data if it was full
         setmode_bts(&(wam[i]->Jsc),SCMODE_IDLE);
         setmode_bts(&(wam[i]->Csc),SCMODE_IDLE);

         if (wamData[i].active_bts == &(wam[i]->Jsc)) { //switch to cartesian space mode.
            SetCartesianSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Csc);
            wamData[i].active_pos = wam[i]->R6pos;
            wamData[i].active_trq = wam[i]->R6force;
            wamData[i].active_dest = wamData[i].cdest;
            wamData[i].vta = &wamData[i].vt_c;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         } else {
            SetJointSpace(wam[i]);
            wamData[i].active_bts = &(wam[i]->Jsc);
            wamData[i].active_pos = wam[i]->Jpos;
            wamData[i].active_trq = wam[i]->Jtrq;
            wamData[i].active_dest = wamData[i].jdest;
            wamData[i].vta = &wamData[i].vt_j;
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      }
      clearScreen();
      break;
   case 'D'://Haptics on
      bth.state = 1;
      break;
   case 'd'://Haptics off
      bth.state = 0;
      break;
   case 'p'://Turn on/off Constraint
      for(i = 0; i < busCount; i++) {
         if (getmode_bts(wamData[i].active_bts)!=SCMODE_IDLE)
            setmode_bts(wamData[i].active_bts,SCMODE_IDLE);
         else
            setmode_bts(wamData[i].active_bts,SCMODE_POS);
         /*
         setrange_vn((vect_n*)xyz, wam[i]->R6ref, 0, 3, 3);
         XYZftoR_m3(r_mat, xyz);
         getcol_m3(ns, r_mat, 0);
         getcol_m3(os, r_mat, 1);
         getcol_m3(as, r_mat, 2);
         //set_m3(r_mat, wam[i]->robot.tool->origin);
         */
      }


      break;
   case '.'://Play loaded trajectory
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {

            cplay = 1;
            eventStart = rt_get_cpu_time_ns();
            eventIdx = 0;

            moveparm_bts(wamData[i].active_bts,vel,acc);
            wamData[i].active_bts->loop_trj = 0;
            if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
               setmode_bts(wamData[i].active_bts,SCMODE_POS);

            start_trj_bts(wamData[i].active_bts);

         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
      break;

   case 'b'://Simulate loaded trajectory

      //sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
      break;

   case '?'://Loop loaded trajectory
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)!=SCMODE_TRJ) {
            moveparm_bts(wamData[i].active_bts,vel,acc);
            wamData[i].active_bts->loop_trj = 1;
            if (getmode_bts(wamData[i].active_bts) != SCMODE_POS)
               setmode_bts(wamData[i].active_bts,SCMODE_POS);
            start_trj_bts(wamData[i].active_bts);
         } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
         }
      }
      break;

   case '/'://Stop loaded trajectory
      for(i = 0; i < busCount; i++) {
         stop_trj_bts(wamData[i].active_bts);
         //setmode_bts(active_bts,prev_mode);
         wamData[i].active_bts->loop_trj = 0;
      }
      break;
   case 'Y'://Start continuous teach
      for(i = 0; i < busCount; i++) {
         sprintf(fn, "teachpath_%d", i);
         if (wamData[i].active_bts == &(wam[i]->Jsc))
            StartContinuousTeach(wam[i], 1, 25, fn);
         else
            StartContinuousTeach(wam[i], 0, 25, fn);
      }

      cteach = 1;
      eventIdx = 0;
      eventStart = rt_get_cpu_time_ns()-750000000L;
      keyEvent[eventIdx].c = 0;

      break;
   case 'y'://Stop continuous teach
      for(i = 0; i < busCount; i++) {
         StopContinuousTeach(wam[i]);
         sprintf(fn, "teachpath_%d", i);
         sprintf(fn2, "teach_%d.csv", i);
         DecodeDL(fn, fn2, 0);
         cteach = 0;
         stop_trj_bts(wamData[i].active_bts);
         /** \internal \todo sleeps that are necessary might be forgotten. Can we eliminate the need?*/
         usleep(10000); //needed to give the command a chance to work.
         destroy_vta(wamData[i].vta); //empty out the data if it was full
         strcpy(active_file,"teach.csv");
         *wamData[i].vta = read_file_vta(fn2, 20);
         register_vta(wamData[i].active_bts, *wamData[i].vta);
      }
      break;
   case 'l'://Load trajectory from file
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);

         for(i = 0; i < busCount; i++) {
            sprintf(fn, "%s_%d", active_file, i);
            destroy_vta(wamData[i].vta); //empty out the data if it was full
            *wamData[i].vta = read_file_vta(fn, 20);
            register_vta(wamData[i].active_bts,*wamData[i].vta);
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;
   case 'w'://Save trajectory to a file
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);
         for(i = 0; i < busCount; i++) {
            sprintf(fn, "%s_%d", active_file, i);
            if (*wamData[i].vta != NULL) {
               write_file_vta(*wamData[i].vta, fn);
            }
         }
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;
   case 'n'://Create a new trajectory
      start_entry();
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         addstr("Enter the max number of points that will be in your trajectory: ");
         refresh();
         ret = scanw("%d", &dtmp);

         for(i = 0; i < busCount; i++) {
            destroy_vta(wamData[i].vta);
            strcpy(active_file, user_def);
            *wamData[i].vta = new_vta(len_vn(wamData[i].active_pos), dtmp);
            register_vta(wamData[i].active_bts, *wamData[i].vta);
         }
         active_file[0] = 0;
      } else {
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
      }
      finish_entry();
      break;
   case 'N'://Toggle angular hold
      if(angular) {
         angular = 0;
         for(i = 0; i < busCount; i++) {
            stop_btPID(&(wam[i]->pid[3]));
         }
      } else {
         angular = 1;
         for(i = 0; i < busCount; i++) {
            set_q(wam[i]->qref, wam[i]->qact);
            start_btPID(&(wam[i]->pid[3]));
         }
      }
      break;

   case 'M'://Move to a location
      if(getmode_bts(wamData[0].active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter comma seperated destination \".2,.4,...\": ");
         refresh();
         getstr( fn);
         strcat(fn,"\n");
         //syslog(LOG_ERR,"Moveto:%s",fn);
         finish_entry();
         fill_vn(wamData[0].active_dest,0.25);
         csvto_vn(wamData[0].active_dest,fn);
         if(wamData[0].active_bts == &(wam[cnt]->Csc)) {
            // Convert from RxRyRz to RotMatrix
            // active_dest[3] = Rx;
            // active_dest[4] = Ry;
            // active_dest[5] = Rz;
            setrange_vn((vect_n*)xyz, wamData[0].active_dest, 0, 3, 3);
            XYZftoR_m3(r_mat, xyz);
            getcol_m3(xyz, r_mat, 0);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 3, 0, 3);
            getcol_m3(xyz, r_mat, 1);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 6, 0, 3);
            getcol_m3(xyz, r_mat, 2);
            setrange_vn(wamData[0].active_dest, (vect_n*)xyz, 9, 0, 3);
         }

         moveparm_bts(wamData[0].active_bts,vel,acc);
         if (getmode_bts(wamData[0].active_bts) != SCMODE_POS)
            setmode_bts(wamData[0].active_bts,SCMODE_POS);

         if(moveto_bts(wamData[0].active_bts,wamData[0].active_dest))
            syslog(LOG_ERR,"Moveto Aborted");

      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;

      //'m': Move to the presently selected trajectory point

   case '<'://Select next trajectory point
      for(i = 0; i < busCount; i++) {
         prev_point_vta(*wamData[i].vta);
      }
      break;
   case '>'://Select previous trajectory point
      for(i = 0; i < busCount; i++) {
         next_point_vta(*wamData[i].vta);
      }
      break;
   case '+'://Insert a point in the trajectory
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
            ins_point_vta(*wamData[i].vta, wamData[i].active_pos);
         }
      }
      break;
   case '-'://Remove a point in the trajectory
      for(i = 0; i < busCount; i++) {
         if(getmode_bts(wamData[i].active_bts)==SCMODE_IDLE) {
            del_point_vta(*wamData[i].vta);
         }
      }
      break;
   case 's'://Adjust trj times using a velocity
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter trajectory velocity: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         for(i = 0; i < busCount; i++) {
            if(*wamData[i].vta != NULL)
               dist_adjust_vta(*wamData[i].vta,tvel);
         }
         finish_entry();
      }
      break;
   case 'S'://Scale trajectory in time
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter scale factor: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         for(i = 0; i < busCount; i++) {
            if(wamData[i].vta != NULL)
               time_scale_vta(*wamData[i].vta,tvel);
         }
         finish_entry();
      }
      break;
   case 'A'://Set the corner acceleration
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Corner Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &tacc);
         for(i = 0; i < busCount; i++) {
            if(*wamData[i].vta != NULL)
               set_acc_vta(*wamData[i].vta, tacc);
         }
         finish_entry();
      }
      break;
   case 'a'://Set the move acceleration
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &acc);
         finish_entry();
      }
      break;
   case 'v'://Set the move velocity
      if(getmode_bts(wamData[0].active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Velocity: ");
         refresh();
         ret = scanw("%lf\n", &vel);
         finish_entry();
      }
      break;
   case ','://Pause/Unpause trajectory
      for(i = 0; i < busCount; i++) {
         status =  movestatus_bts(wamData[i].active_bts);
         if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
            unpause_trj_bts(wamData[i].active_bts,2);
         else
            pause_trj_bts(wamData[i].active_bts,2);
      }
      break;
   case 'h'://Toggle Help
      clearScreen();
      scr = !scr;
      break;
#if 0

   case 27://Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR) {
         if (chr == 91) {
            if ((chr = getch()) != ERR) {
               if (chr == 67) //Right arrow
               {
               } else if (chr == 68) //Left arrow
               {
               } else {
                  while(getch()!=ERR) {
                     // Do nothing
                  }
                  syslog(LOG_ERR,"Caught unknown keyhit 27-91-%d",c);
               }
            }
         } else {
            while(getch()!=ERR) {
               // Do nothing
            }
            syslog(LOG_ERR,"Caught unknown keyhit 27-%d",c);
         }
      }
      break;

   default:
      while(getch()!=ERR) {
         // Do nothing
      }
      syslog(LOG_ERR,"Caught unknown keyhit %d",c);

      break;
#endif

   }
}


