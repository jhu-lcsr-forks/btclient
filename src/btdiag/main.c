/*======================================================================*
 *  Module .............btdiag
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......18 Mar 2003
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  18 Mar 2003 - TH
 *    File created
 *  25 Mar 2003 - BZ
 *    File documented
 *  18 May 2004 - BZ
 *    Reworked interface
 *  17 Dec 2004 - TH
 *    Port to RTAI Linux
 *                                                                      *
 *======================================================================*/

/** \file main.c
    Provides a dignostic interface to a set of motor controllers.
 
 */

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

//th041217#include <sys/sched.h>
#ifdef USE_RTAI31
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif
#ifdef USE_FUSION
#include <rtai/mutex.h>
#include <rtai/task.h>
#include <rtai/timer.h>
#endif


/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/

//#include "btdriver.h"

#include "btjointcontrol.h"
#include "btsystem.h"
#include "gimbals.h"
#include "btwam.h"
#include "playlist.h"
#include "control_loop.h"
#include "btmath.h"
#include "btrobot.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define column_offset     (15)
#define column_width      (11)

enum {HELP_SCREEN, MAIN_SCREEN, GCOMP_SCREEN, HOMING_SCREEN, TIMING_SCREEN, PLAYLIST_SCREEN, ROBOT_SCREEN};

enum { NOOBJ, DEFINE_OBJ, WALLDEFINE_PT1, WALLDEFINE_PT2, WALLDEFINE_PT3, BLOCKDEFINE_PT1, BLOCKDEFINE_PT2, CONTAINERDEFINE_PT1, CONTAINERDEFINE_PT2, SPHEREDEFINE_CENTER, SPHEREDEFINE_RADIUS, FIELDDEFINE_PT1, FIELDDEFINE_PT2 };

static int mode = NOOBJ;
static double tempPt1[3];
static double tempPt2[3];
static double tempPt3[3];

static OBJECT hapticObjects[MAX_HAPTIC_OBJECTS];
static int hapticObjectIndices[MAX_HAPTIC_OBJECTS];
/* this is all to avoid using malloc */
/* right now we will just use corresponding specific object index - really quick, sorry */
static RESISTANCELINE resistanceLines[MAX_HAPTIC_OBJECTS];
static BLOCK blocks[MAX_HAPTIC_OBJECTS];
static CONTAINER containers[MAX_HAPTIC_OBJECTS];
static PLANE planes[MAX_HAPTIC_OBJECTS];
static SPHERE spheres[MAX_HAPTIC_OBJECTS];

static FIELD fields[MAX_HAPTIC_OBJECTS];

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
char divider[column_offset + (column_width * MAX_NODES)];
SC_move_list ml;
matr_h *myFrame;

int motor_position[MAX_NODES];
#ifdef USE_RTAI31
pthread_mutex_t disp_mutex;
#endif
#ifdef USE_FUSION
RT_MUTEX disp_mutex;
#endif


int active = 0;
int npucks;
int cpuck = 0;
int entryLine;


long saved_bus_error = 0;
int modes[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long int temperatures[MAX_NODES];
double commands[15];
double newpid[15];

int present_screen = MAIN_SCREEN;

int startup_stage   = FALSE; //Set to TRUE when main loop is started. Used to control sigint handler
int useGimbals      = FALSE;
int konstants       = FALSE;
int done            = FALSE;
int WACKYplay       = FALSE;
int showWhere       = FALSE;
int disp_puck       = FALSE;
int useTRC          = FALSE;
btrobot robot;
WAMdat wam;
//extern wam_struct WAM;  // this is a hack that will be resolved - sc
extern int isZeroed;
static RT_TASK *mainTask;
vect_3 *p,*o;
vect_n *t,*targ,*negtarg;
vect_n *Mpos, *Mtrq, *Jpos, *Jtrq, *wv;
matr_h *tmh;



/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void RenderScreen(void);
void RenderHelp(void);
void RenderRobot(void);
void ProcessInput(int c);
void Shutdown(void);
void DisplayThread(void);
void StartDisplayThread(void);
void WACKYeval(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);

/*==============================*
 * Functions                    *
 *==============================*/


/** Entry point for the application.
    Initializes the system and executes the main event loop.
 
*/
int main(int argc, char **argv)
{
  char    chr,cnt;
  int     err;
  int i;

  OBJECT          object1;
  BLOCK           block1;
  SPHERE          sphere1;
  RESISTANCELINE  line1;
  WAMLoc          point1, point2;
  struct sched_param mysched;
  p = new_v3();
  fill_v3(p,0.0);
  o = new_v3();
  fill_v3(o,0.0);
  t = new_vn(7);

  targ = new_vn(3);
  fill_vn(targ,0.3);
  negtarg = new_vn(3);
  const_vn(negtarg,0.3,-0.3,0.3);

  Mpos = new_vn(10);
  Mtrq = new_vn(10);
  Jpos = new_vn(10);
  Jtrq = new_vn(10);
  wv = new_vn(10);
  tmh = new_mh();
  const_vn(wv, 0.0, -2.017, -0.011, 0.88, 0.0, 0.0, 0.0);
  /* Initialize the ncurses screen library */
  init_ncurses();

  //* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  syslog(LOG_ERR, "syslog initalized by WAM application");
  //addstr("syslog initalized\n");
  //refresh();

  /* Initialize the display mutex */
  err = pthread_mutex_init(&(disp_mutex), NULL);
  if (err)
    syslog(LOG_ERR, "Could not initialize mutex for displays.");

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


  /* Initialize rtlinux subsystem */
  mysched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
  if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1)
  {
    syslog(LOG_ERR, "Error setting up linux (native) scheduler");
  }

  mainTask = rt_task_init(nam2num("main01"), 0, 0, 0); /* defaults */

  /* Output the WAM data structure */
  DumpData2Syslog();

  /* This is a hack for now, so references to global WAM don't
     have to be updated through all the driver code */
  wam.wamDriverData = GetWAM();

  for(i=0;i<MAX_HAPTIC_OBJECTS;i++)
  {

    hapticObjectIndices[i] = -1;
  }

  /* Set up the WAM data structure, init kinematics, dynamics, haptics */
  InitHapticWAM(&wam);


  setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous

  npucks = wam.wamDriverData->num_actuators; // Get the number of initialized actuators

  MLconstruct(&ml, wam.wamDriverData->sc, 10, 50); //Initialize the playlist

  start_control_threads(0, 2000000L, WAMControlThread, (void *)0);

  signal(SIGINT, sigint_handler); //register the interrupt handler
  startup_stage = 1;

  StartDisplayThread();

  fer(cnt, 3)  setgains_btPID(&(wam.wamDriverData->pid[cnt]), 5000, 10 , 1);

  while (!done)
  {
    WACKYeval();  //check to see whether we want to update to another random trajectory
    MLeval(&ml);  //Check to see whether the playlist needs attention

    if ((chr = getch()) != ERR) //Check buffer for keypress
      ProcessInput(chr);

    usleep(100000); // Sleep for 0.1s
  }
  Shutdown();
  syslog(LOG_ERR, "CloseSystem");
  CloseSystem();
  syslog(LOG_ERR, "end.");
}

/* Initialize the ncurses screen library */
void init_ncurses(void)
{
  initscr();
  addstr("Screen initalized\n");
  refresh();
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
  if (!startup_stage)
  {
    endwin();
    exit(1);
  }
  else
    done = 1;
}

/** Shut the application down gracefully.
    Stops the control threads, ncurses, syslog, and frees the movelist.
*/
void Shutdown()
{
  syslog(LOG_ERR, "stop_control_threads");
  stop_control_threads();
  syslog(LOG_ERR, "MLdestroy");
  MLdestroy(&ml);
  syslog(LOG_ERR, "closelog");
  closelog();
  syslog(LOG_ERR, "endwin");
  endwin();
  syslog(LOG_ERR, "rt_task_delete");
  rt_task_delete(mainTask);
}

/** Starts the display thread.
    Creates a new thread, sets its priority, and runs it as a display thread.
*/
void StartDisplayThread()
{
  pthread_t           display_thd_id;
  pthread_attr_t      display_attr;
  struct sched_param  display_param;

  pthread_attr_init(&display_attr);
  pthread_attr_setinheritsched(&display_attr, PTHREAD_EXPLICIT_SCHED);
  display_param.sched_priority = 10;
  pthread_attr_setschedparam(&display_attr, &display_param);

  pthread_create(&display_thd_id, &display_attr, (void*)DisplayThread, 0);
  sched_setscheduler(display_thd_id, SCHED_FIFO, &display_param);
  if (display_thd_id == -1)
  {
    syslog(LOG_ERR, "start_display_thread:Couldn't start display thread!");
    exit( -1);
  }
}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
  int cnt,err;
  RT_TASK *displayTask;

  displayTask = rt_task_init(nam2num("displa"), 0, 0, 0);

  /* Create the divider line */
  divider[0] = '\0';
  for(cnt = 0; cnt < column_offset + wam.wamDriverData->num_actuators * column_width; cnt++)
  {
    strcat(divider, "-");
  }

  clear();
  refresh();
  while (!done)
  {
    if ((err = pthread_mutex_lock( &(disp_mutex) )) != 0)
    {
      syslog(LOG_ERR, "Display mutex failed: %d", err);
    }

    switch (present_screen)
    {
    case HELP_SCREEN:
      RenderHelp();
      break;
    case MAIN_SCREEN:
      RenderScreen();
      break;
    case ROBOT_SCREEN:
      RenderRobot();
      break;
    }
    pthread_mutex_unlock( &(disp_mutex) );
    usleep(100000);
  }
  rt_task_delete(displayTask);

}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
  int err;
  if ((err = pthread_mutex_lock( &(disp_mutex) )) != 0)
  {
    syslog(LOG_ERR, "display mutex failed: %d", err);
  }
  move(entryLine, 1);
  echo();
  timeout( -1);
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

/** Show the help screen.
    Displays the on-screen help.
*/
void RenderHelp() //{{{
{
  //int val;
  //int cnt, idx;
  int line;

  line = 0;
  mvprintw(line , 0, "'a' Power wam");
  ++line;
  mvprintw(line , 0, "'d' Idle wam");
  ++line;
  mvprintw(line , 0, " ");
  ++line;
  mvprintw(line , 0, "'[' Select previous joint");
  ++line;
  mvprintw(line , 0, "']' Select next joint");
  ++line;

  refresh();
}

/** Show the help screen.
    Displays the on-screen help.
*/
void RenderRobot() //{{{
{

  int line,cnt;
  int col_offset;
  char buf[100];
  col_offset = 0;
  line = 0;

  mvprintw(line, col_offset, "CPos %s", sprint_v3(buf,wam.wamDriverData->Cpos));
  ++line;
  mvprintw(line, col_offset, "CForce %s", sprint_v3(buf,wam.wamDriverData->Cforce));
  ++line;
  mvprintw(line, col_offset, "CRef %s", sprint_v3(buf,wam.wamDriverData->Cref));
  ++line;
  set_mh(tmh,wam.wamDriverData->robot.links[4].origin);
  set_v3(o,wam.wamDriverData->Cforce);
  mvprintw(line, col_offset, "oForce %s", sprint_v3(buf,matTXvec_m3(tmh,o)));
  ++line;
  mvprintw(line, col_offset, "Joint Torques %s", sprint_vn(buf,wam.wamDriverData->robot.t));
  ++line;
  ++line;
  for (cnt = 0;cnt < 5; cnt++)
  {
    mvprintw(line, col_offset, "Link %d: F=%s T=%s tF=%s tT=%s",cnt,sprint_v3(buf,wam.wamDriverData->robot.links[cnt].lastforce.f),sprint_v3(buf,wam.wamDriverData->robot.links[cnt].lastforce.t),
             sprint_v3(buf,wam.wamDriverData->robot.links[cnt].lastforce.f->ret),sprint_v3(buf,wam.wamDriverData->robot.links[cnt].lastforce.t->ret));
    ++line;
  }
  for (cnt = 0;cnt < 5; cnt++)
  {
    set_v3(p,wam.wamDriverData->robot.links[cnt].cog);
    mvprintw(line, col_offset, "Link %d: F=%s T=%s Rp=%s",cnt,sprint_v3(buf,wam.wamDriverData->robot.links[cnt].f),sprint_v3(buf,wam.wamDriverData->robot.links[cnt].t),sprint_v3(buf,wam.wamDriverData->robot.links[cnt].Rp));
    ++line;
  }
  ++line;
  ++line;
  mvprintw(line, col_offset, "Arclength %f  ", arclength_pwl(&(wam.wamDriverData->pth)));
  ++line;
  mvprintw(line, col_offset, "State %d  ", wam.wamDriverData->trj.state);
  ++line;
  mvprintw(line, col_offset, "Trjcmd %f  ", wam.wamDriverData->F);
  ++line;
  mvprintw(line, col_offset, "Path Segment %d  ",get_segment_pwl(&(wam.wamDriverData->pth),wam.wamDriverData->F));
  ++line;
  mvprintw(line, col_offset, "End of Path %d  ",endof_vr(wam.wamDriverData->pth.vr));
  ++line;

  refresh();
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderScreen() //{{{
{
  //int val;
  int cnt, idx,cnt2;
  int line;
  double gimb[4];



  /***** Display the interface text *****/
  line = 0;
  mvprintw(line , 0, "Barrett Technology Diagnostic Application (btdiag)");
  ++line;

  if (disp_puck)
  {
    mvprintw(line , 0, "       PUCK: ");
    ++line;
    mvprintw(line , 0, "       Mode: ");
    ++line;
    mvprintw(line , 0, "        Pos: ");
    ++line;
    mvprintw(line , 0, "        Trq: ");
    ++line;
  }
  else
  {
    mvprintw(line , 0, "      JOINT: ");
    ++line;
    mvprintw(line , 0, "       Mode: ");
    ++line;
    mvprintw(line , 0, "      Angle: ");
    ++line;
    mvprintw(line , 0, "     Torque: ");
    ++line;
  }
  mvprintw(line , 0, "    PID Cmd: ");
  ++line;
  mvprintw(line , 0, " Trj Target: ");
  ++line;
  mvprintw(line , 0, "[G] Temp C : ");
  ++line;

  if(konstants)
  {
    mvprintw(line , 0, "[k] Kp     : ");
    ++line;
    mvprintw(line , 0, "    Kd     : ");
    ++line;
    mvprintw(line , 0, "    Ki     : ");
    ++line;
    mvprintw(line , 0, "[v] Vel    : ");
    ++line;
    mvprintw(line , 0, "    Acc    : ");
    ++line;
  }

  mvprintw(line, 0, "[c] Command: ");
  ++line;
  mvprintw(line, 0, "%s", divider);
  ++line;

  mvprintw(line, 0, "  Play List: ");
  ++line;
  mvprintw(line, 0, "     Status: ");
  ++line;
  mvprintw(line, 5, "[.]Stop  [,]One   [;]Play  [/]Loop  [>]Next");
  ++line;
  mvprintw(line, 5, "[<]Prev  [+]Add   [-]Del   [W]Write [l]Load");
  ++line;

  mvprintw(line, 0, "%s", divider);
  ++line;
  if(showWhere)
  {
    mvprintw(line, 0, "   WhereAmI:");
    ++line;
  }
  if(mode == DEFINE_OBJ)
  {
    mvprintw(line, 0, "Create Obj: [p]lane [c]ontainer [b]lock [s]phere [f]ield   ");
    ++line;

  }
  else if(mode == WALLDEFINE_PT1 || mode == BLOCKDEFINE_PT1 || mode == CONTAINERDEFINE_PT1)
  {
    mvprintw(line, 0, "Hit space at first point                          ");
    ++line;
  }
  else if(mode == WALLDEFINE_PT2 || mode == BLOCKDEFINE_PT2 || mode == CONTAINERDEFINE_PT2)
  {
    mvprintw(line, 0, "Hit space at the second point                     ");
    ++ line;

  }
  else if(mode == WALLDEFINE_PT3)
  {
    mvprintw(line, 0, "Hit space at the third point                      ");
    ++line;
  }
  else if (mode == SPHEREDEFINE_CENTER)
  {
    mvprintw(line, 0, "Hit space at the center of the sphere             ");
    ++line;
  }
  else if(mode == SPHEREDEFINE_RADIUS)
  {
    mvprintw(line, 0, "Hit space at the radius of the sphere             ");
    ++line;
  }
  else if(mode == FIELDDEFINE_PT1)
  {
    mvprintw(line, 0, "Hit space at the origin of the vector force             ");
    ++line;
  }
  else if(mode == FIELDDEFINE_PT2)
  {
    mvprintw(line, 0, "Hit space at the endpoint of the vector force           ");
    ++line;
  }
  else
  {
    mvprintw(line, 0, "                                                  ");
    ++line;
  }
  ++line;
  ++line;
  ++line;
  ++line;
  ++line;
  ++line;
  entryLine = line;

  mvprintw(line, 0, "%s", divider);
  ++line;

  mvprintw(line , 0, "[o]   Add Object   [y] Play file    [r/R] Record (Man/Auto) ");
  ++line;
  mvprintw(line , 0, "[i,I] Idle Mode    [x] Exit         [g] (%c) Gravity toggle ", wam.wamDriverData->Gcomp ? '*' : ' ');
  ++line;
  mvprintw(line , 0, "[t,T] Torque Mode  [L] Limit Vel    [b] (%c) Ripple toggle  ", useTRC ? '*' : ' ');
  ++line;
  mvprintw(line , 0, "[p,P] PID Mode     [h] Home WAM     [z,Z] ZeroWAM (Man/Auto)");
  ++line;
  mvprintw(line , 0, "[s,S] Start Trj    [ ] Joint/Puck   [k,K] Show/Edit Kp, etc.");
  ++line;

  /***** Display the data *****/
  getWAMjoints(Mpos, Mtrq, Jpos, Jtrq);
  getWAMmotor_position(motor_position);
  /******************** btRobot test *********************************/
  get_t_btrobot(&(wam.wamDriverData->robot),t);


  if(useGimbals) // Convert the gimbals Q4.12 position data into radians
  {
    getGimbalsAngles(gimb);
    setval_vn(Jpos,4,gimb[0]);
    setval_vn(Jpos,5,gimb[1]);
    setval_vn(Jpos,6,gimb[2]);

  }
  fer (cnt, wam.wamDriverData->num_actuators)
  {
    line = 1;
    idx = wam.wamDriverData->act[cnt].puck.ID;

    if(disp_puck)
    {
      mvprintw(line , column_offset + column_width*cnt, "   %2d   ", idx);
      ++line;
      if(modes[idx] == MODE_IDLE)
        mvprintw(line , column_offset + column_width*cnt, "IDLE  ");
      else if (modes[idx] == MODE_TORQUE)
        mvprintw(line , column_offset + column_width*cnt, "TORQUE");
      else if (modes[idx] == MODE_PID)
        mvprintw(line , column_offset + column_width*cnt, "PID   ");
      else
        mvprintw(line , column_offset + column_width*cnt, "UNK:%d ", modes[idx]);
      ++line;
      mvprintw(line , column_offset + column_width*cnt, "%8d ", wam.wamDriverData->act[cnt].puck.position);
      ++line;
      mvprintw(line , column_offset + column_width*cnt, "%8d ", wam.wamDriverData->act[cnt].puck.torque_cmd);
      ++line;
    }
    else
    {
      if (motor_position[cnt] == motor_position[cpuck])
        mvprintw(line , column_offset + column_width*cnt, " -[%2d]- ", motor_position[cnt]+1);
      else
        mvprintw(line , column_offset + column_width*cnt, "   %2d  ", motor_position[cnt]+1);
      ++line;

      if (wam.wamDriverData->sc[motor_position[cnt]].mode == MODE_IDLE)
        mvprintw(line , column_offset + column_width*cnt, "IDLE  ");
      else if (wam.wamDriverData->sc[motor_position[cnt]].mode == MODE_TORQUE)
        mvprintw(line , column_offset + column_width*cnt, "TORQUE");
      else if (wam.wamDriverData->sc[motor_position[cnt]].mode == MODE_PID)
        mvprintw(line , column_offset + column_width*cnt, "PID   ");
      else
        mvprintw(line , column_offset + column_width*cnt, "UNK:%d ", wam.wamDriverData->sc[motor_position[cnt]].mode);
      ++line;

      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jpos,motor_position[cnt]));
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jtrq,motor_position[cnt]));
      ++line;
    }


    mvprintw(line, column_offset + column_width*cnt, "%8d ", temperatures[wam.wamDriverData->act[cnt].puck.ID] );
    ++line;

    if(konstants)
    {
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[motor_position[cnt]].pid.Kp );
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[motor_position[cnt]].pid.Kd );
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[motor_position[cnt]].pid.Ki);
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[motor_position[cnt]].trj.vel);
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[motor_position[cnt]].trj.acc);
      ++line;
    }

    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", commands[cnt]);
    ++line;
    ++line;
  }
  cnt2 = line;
  fer (cnt, 10 ){
    line = cnt2;
    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[cnt].pid.yref);
    ++line;
    if (wam.wamDriverData->sc[cnt].trj.state == 0)
      mvprintw(line, column_offset + column_width*cnt, "STOPPED    ");
    else
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam.wamDriverData->sc[cnt].trj.end_point);
    ++line;
    if (ml.Nmoves)
      mvprintw(line, column_offset + column_width*cnt, "%8.4f ", ml.next[cnt].pos);
    else
      mvprintw(line, column_offset + column_width*cnt, " No Pts ");
    ++line;
    mvprintw(line, column_offset, "%s, point %d of %d          ", MLrunning(&ml) ? "RUNNING" : "STOPPED", ml.Cmove, ml.Nmoves);
    ++line;
  }
  ++line;
  ++line;
  ++line;

  //if(showWhere){
  mvprintw(line, column_offset, " X = %8.4f,  Y = %8.4f,  Z = %8.4f\t\t\t", wam.wamTipLoc.pos[0], wam.wamTipLoc.pos[1], wam.wamTipLoc.pos[2]);
  ++line;
  mvprintw(line, column_offset, " X = %8.4f,  Y = %8.4f,  Z = %8.4f\tbtCPos", getval_v3(wam.wamDriverData->Cpos,0), getval_v3(wam.wamDriverData->Cpos,1), getval_v3(wam.wamDriverData->Cpos,2));
  ++line;
  mvprintw(line, column_offset, " X = %8.4f,  Y = %8.4f,  Z = %8.4f\tbtCForce", getval_v3(wam.wamDriverData->Cforce,0), getval_v3(wam.wamDriverData->Cforce,1), getval_v3(wam.wamDriverData->Cforce,2));
  ++line;
  mvprintw(line, column_offset, " <%8.4f,  %8.4f,  %8.4f, %8.4f>\t Torque", getval_vn(t,0), getval_vn(t,1), getval_vn(t,2), getval_vn(t,3));
  //}
  mvprintw(line, 0, "");

  refresh();
} //}}}

void clearScreen(void)
{
  int err;

  if ((err = pthread_mutex_lock( &(disp_mutex) )) != 0)
  {
    syslog(LOG_ERR, "display mutex failed: %d", err);
  }
  clear();
  pthread_mutex_unlock( &(disp_mutex) );
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
  int tmp, ret;
  int cnt;
  char chr;
  double jVel, tVel, eVel;
  double ftmp;
  char fn[50];

  int i;
  switch (mode)
  {
  case DEFINE_OBJ:

    switch(c)
    {
    case 'p':
      mode = WALLDEFINE_PT1;
      break;
    case 'c':
      mode = CONTAINERDEFINE_PT1;
      break;
    case 'b':
      mode = BLOCKDEFINE_PT1;
      break;
    case 's':
      mode = SPHEREDEFINE_CENTER;
      break;
    case 'f':
      mode = FIELDDEFINE_PT1;
      break;

    }

    break;

  case WALLDEFINE_PT1:
    if(c == ' ')
    {
      tempPt1[0] = wam.wamTipLoc.pos[0];
      tempPt1[1] = wam.wamTipLoc.pos[1];
      tempPt1[2] = wam.wamTipLoc.pos[2];
      mode = WALLDEFINE_PT2;
    }
    break;

  case WALLDEFINE_PT2:
    if(c==' ')
    {
      tempPt2[0] = wam.wamTipLoc.pos[0];
      tempPt2[1] = wam.wamTipLoc.pos[1];
      tempPt2[2] = wam.wamTipLoc.pos[2];
      mode = WALLDEFINE_PT3;
    }
    break;

  case WALLDEFINE_PT3:
    if(c==' ')
    {
      tempPt3[0] = wam.wamTipLoc.pos[0];
      tempPt3[1] = wam.wamTipLoc.pos[1];
      tempPt3[2] = wam.wamTipLoc.pos[2];

      i=0;
      while(hapticObjectIndices[i] != -1)
      {
        i++;
      }
      createPlane(&hapticObjects[i], &planes[i], tempPt1, tempPt2, tempPt3);
      hapticObjectIndices[i] = addHapticObject(&hapticObjects[i]);

      mode = NOOBJ;
    }
    break;
  case BLOCKDEFINE_PT1:
    if(c==' ')
    {
      tempPt1[0] = wam.wamTipLoc.pos[0];
      tempPt1[1] = wam.wamTipLoc.pos[1];
      tempPt1[2] = wam.wamTipLoc.pos[2];
      mode = BLOCKDEFINE_PT2;
    }
    break;
  case BLOCKDEFINE_PT2:
    if(c==' ')
    {
      tempPt2[0] = wam.wamTipLoc.pos[0];
      tempPt2[1] = wam.wamTipLoc.pos[1];
      tempPt2[2] = wam.wamTipLoc.pos[2];
      i=0;
      while(hapticObjectIndices[i] != -1)
      {
        i++;
      }
      createBlock(&hapticObjects[i], &blocks[i], tempPt1, tempPt2,1.0);
      hapticObjectIndices[i] = addHapticObject(&hapticObjects[i]);
      mode = NOOBJ;
    }
    break;
  case CONTAINERDEFINE_PT1:
    if(c==' ')
    {
      tempPt1[0] = wam.wamTipLoc.pos[0];
      tempPt1[1] = wam.wamTipLoc.pos[1];
      tempPt1[2] = wam.wamTipLoc.pos[2];
      mode = CONTAINERDEFINE_PT2;
    }
    break;
  case CONTAINERDEFINE_PT2:
    if(c==' ')
    {
      tempPt2[0] = wam.wamTipLoc.pos[0];
      tempPt2[1] = wam.wamTipLoc.pos[1];
      tempPt2[2] = wam.wamTipLoc.pos[2];
      i=0;
      while(hapticObjectIndices[i] != -1)
      {
        i++;
      }
      createContainer(&hapticObjects[i], &containers[i], tempPt1, tempPt2);
      hapticObjectIndices[i] = addHapticObject(&hapticObjects[i]);
      mode = NOOBJ;
    }
    break;
  case SPHEREDEFINE_CENTER:
    if(c==' ')
    {
      tempPt1[0] = wam.wamTipLoc.pos[0];
      tempPt1[1] = wam.wamTipLoc.pos[1];
      tempPt1[2] = wam.wamTipLoc.pos[2];
      mode = SPHEREDEFINE_RADIUS;
    }
    break;
  case SPHEREDEFINE_RADIUS:
    if(c==' ')
    {
      tempPt2[0] = wam.wamTipLoc.pos[0];
      tempPt2[1] = wam.wamTipLoc.pos[1];
      tempPt2[2] = wam.wamTipLoc.pos[2];

      i=0;
      while(hapticObjectIndices[i] != -1)
      {
        i++;
      }
      createSphere(&hapticObjects[i], &spheres[i], tempPt1, tempPt2);
      hapticObjectIndices[i] = addHapticObject(&hapticObjects[i]);
      mode = NOOBJ;
    }
    break;
  case FIELDDEFINE_PT1:
    if(c==' ')
    {
      tempPt1[0] = wam.wamTipLoc.pos[0];
      tempPt1[1] = wam.wamTipLoc.pos[1];
      tempPt1[2] = wam.wamTipLoc.pos[2];
      mode = FIELDDEFINE_PT2;
    }
    break;
  case FIELDDEFINE_PT2:
    if(c==' ')
    {
      tempPt2[0] = wam.wamTipLoc.pos[0];
      tempPt2[1] = wam.wamTipLoc.pos[1];
      tempPt2[2] = wam.wamTipLoc.pos[2];
      i=0;
      while(hapticObjectIndices[i] != -1)
      {
        i++;
      }
      createField(&hapticObjects[i], &fields[i], tempPt1, tempPt2);
      hapticObjectIndices[i] = addHapticObject(&hapticObjects[i]);
      mode = NOOBJ;
    }
    break;


  case NOOBJ:
    switch (c)
    {
    case 'F': // Force zero
      isZeroed = TRUE;
      break;
    case 'f': // Eliminate torque fault
      setProperty(0,10,TL2,FALSE,8200);
      setProperty(0,10,VL2,FALSE,0); // Eliminate Velocity fault
      break;
    case 'o': // Define Object
      mode = DEFINE_OBJ;
      break;
    case 'a': // Activate
      fer(cnt,npucks) SetProp(cnt,MODE,2);
      break;
    case 'd': // Activate
      fer(cnt,npucks) SetProp(cnt,MODE,0);
      break;
    case 'i': // Set present joint controller to Idle mode (sends zero torque)
      SCsetmode(&(wam.wamDriverData->sc[motor_position[cpuck]]), MODE_IDLE);
      break;
    case 'I': // Set ALL joint controllers to Idle mode (sends zero torque)
      fer(cnt, npucks) SCsetmode(&(wam.wamDriverData->sc[motor_position[cnt]]), MODE_IDLE);
      break;
    case 't': // Set present joint controller to Torque mode
      SCsetmode(&(wam.wamDriverData->sc[motor_position[cpuck]]), MODE_TORQUE);
      break;
    case 'T': // Set ALL joint controllers to Torque mode
      fer(cnt, npucks) SCsetmode(&(wam.wamDriverData->sc[motor_position[cnt]]), MODE_TORQUE);
      break;
    case 'H': // Set ALL joint controllers to Torque mode
      fer(cnt, 3) {
        set_v3((wam.wamDriverData->Cref),(wam.wamDriverData->Cpos));
        start_btPID(&(wam.wamDriverData->pid[cnt]));
      }
      break;
    case 'h': // Set ALL joint controllers to Torque mode
      fer(cnt, 3) stop_btPID(&(wam.wamDriverData->pid[cnt]));
      break;
    case 'N':
      wam.wamDriverData->use_new = 0;
      break;
    case 'n':
      wam.wamDriverData->use_new = !wam.wamDriverData->use_new;
      break;
    case 'E': // Set ALL joint controllers to Torque mode
      start_entry();
      addstr("Enter cartesian pid constants \"Kp Kd Ki\": ");
      refresh();
      ret = scanw("%lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]));
      //if (ret == 3)
      fer(cnt, 3)
        setgains_btPID(&(wam.wamDriverData->pid[cnt]), newpid[0], newpid[1], newpid[2]);

      finish_entry();
      break;

    case 'p': // Set present joint controller to PID mode
      SCsetmode(&(wam.wamDriverData->sc[motor_position[cpuck]]), MODE_PID);
      break;
    case 'P': // Set ALL joint controllers to PID mode
      fer(cnt, npucks) SCsetmode(&(wam.wamDriverData->sc[motor_position[cnt]]), MODE_PID);
      break;
    case 's': // Start Trajectory control on present puck
      CartesianMoveWAM(negtarg, 0.5, 0.5);
      break;
    case 'S': // Start Trajectory control on all pucks
      CartesianMoveWAM(targ, 0.5, 0.5);
      break;
    case 'x':
    case 'X': /* eXit */
      done = 1;
      break;

    case ' ': /* Toggle puck/joint display */
      disp_puck = !disp_puck;
      clear();
      break;
    case '!': /* Toggle puck/joint display */
      present_screen = ROBOT_SCREEN;
      clear();
      break;
    case '@': /* Toggle puck/joint display */
      present_screen = MAIN_SCREEN;
      clear();
      break;
    case 'g': /* Toggle gravity compensation */
      toggleGcomp(wam.wamDriverData);
      refresh();
      break;
    case 'b': /* Toggle Torque Ripple Compensation */
      useTRC = !useTRC;
      SetTRC(useTRC);
      refresh();
      break;
    case 'Z': /* Zero WAM */
      const_vn(wv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      SetWAMpos(wv);
      //ZeroWAM();
      //useTRC = 1;
      //SetTRC(useTRC);
      break;
    case 'z': /* Send zero-position to WAM */
      // set_wam_vector(&wv, 0, 0, 0, 0, 0, 0, 0);
      //const_vn(wv, 0.0, -2.017, -0.011, 0.88, 0.0, 0.0, 0.0); //gimbals
      const_vn(wv, 0.0, -2.017, 0.0, 3.14, 0.0, 0.0, 0.0); //blanklink
      SetWAMpos(wv);
      break;
    case 'L': // Limit WAM velocity
      start_entry();
      addstr("Enter JointVel (rad/s), TipVel (m/s), ElbowVel (m/s): ");
      refresh();
      scanw("%lf,%lf,%lf", &jVel, &tVel, &eVel);
      finish_entry();
      setSafetyLimits(jVel, tVel, eVel);
      break;
    case 'W':   // Get whereAmI()
      showWhere = !showWhere;
      clearScreen();
      break;
    case '~':
      WACKYplay = !WACKYplay;
      break;
    case '[': /* Select previous puck */
      cpuck--;
      if (cpuck < 0)
        cpuck = npucks - 1;
      break;
    case ']': /* Select next puck */
      cpuck++;
      if (cpuck >= npucks)
        cpuck = 0;
      break;
    case 'c': /* Enter trajectory end position for present puck */
      start_entry();
      addstr("Enter next command: ");
      refresh();
      scanw("%lf", &(commands[cpuck]));
      finish_entry();
      break;
    case 'C': /* Enter trajectory end position for all pucks */
      start_entry();
      addstr("Enter next command for all pucks: ");
      refresh();
      scanw("%lf", &ftmp);
      fer (cnt, npucks) commands[cnt] = ftmp;
      finish_entry();
      break;
    case '_': /* Refresh display */
      clearScreen();
      break;
    case 'G':   /* Get temperatures */
      fer(cnt, npucks)
      {
        temperatures[wam.wamDriverData->act[cnt].puck.ID] = -1;
        tmp = GetProp(cnt, TEMP);
        temperatures[wam.wamDriverData->act[cnt].puck.ID] = tmp;
      }
      break;
    case 'k': /* Enter PID constants for present puck \bug Check to see if we are in control mode 0 and able to change gains*/
      if(konstants)
      {
        start_entry();
        addstr("Enter pid constants \"Kp Kd Ki\": ");
        refresh();
        ret = scanw("%lf %lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]), &(newpid[3]));
        //if (ret == 3)
        SCsetpid(&(wam.wamDriverData->sc[motor_position[cpuck]]), newpid[0], newpid[1], newpid[2], newpid[3]);
        finish_entry();
      }
      else
      {
        konstants = TRUE;
        clearScreen();
      }
      break;
    case 'K': /* Enter PID constants for all pucks \bug Check to see if we are in control mode 0 and able to change gains*/
      if(konstants)
      {
        start_entry();
        addstr("Enter pid constants \"Kp Kd Ki\": ");
        refresh();
        ret = scanw("%lf %lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]), &(newpid[3]));
        //if (ret == 3)
        fer(cnt, npucks)
        SCsetpid(&(wam.wamDriverData->sc[motor_position[cnt]]), newpid[0], newpid[1], newpid[2], newpid[3]);
        finish_entry();
      }
      else
      {
        konstants = TRUE;
        clearScreen();
      }
      break;
    case 'v': /* Enter trajectory constants for present puck */
      if(konstants)
      {
        start_entry();
        addstr("Enter trajectory constants \"Velocity Acceleration\": ");
        refresh();
        ret = scanw("%lf %lf", &(newpid[0]), &(newpid[1]));
        SCsettrjprof(&(wam.wamDriverData->sc[motor_position[cpuck]]), newpid[0], newpid[1]);
        finish_entry();
      }
      else
      {
        konstants = TRUE;
        clearScreen();
      }
      break;
    case 'V': /* Enter trajectory constants for all pucks */
      if(konstants)
      {
        start_entry();
        addstr("Enter trajectory constants \"Velocity Acceleration\": ");
        refresh();
        ret = scanw("%lf %lf", &(newpid[0]), &(newpid[1]));
        fer(cnt, npucks)
        SCsettrjprof(&(wam.wamDriverData->sc[motor_position[cnt]]), newpid[0], newpid[1]);
        finish_entry();
      }
      else
      {
        konstants = TRUE;
        clearScreen();
      }
      break;
    case ';':   /* Play through playlist once */
      MLplay(&ml);
      break;
    case ',': /* Execute present line in playlist */
      MLone(&ml);
      break;
    case '>': /* Show next line in playlist */
      MLnext(&ml);
      break;
    case '<': /* Show previous line in playlist */
      MLprev(&ml);
      break;
    case '/':   /* Loop through playlist */
      MLrepeatplay(&ml);
      break;
    case '.':   /* Stop playlist execution */
      MLstop(&ml);
      break;
    case 'l':   /* Load playlist from file */
      start_entry();
      addstr("Enter filename for playlist read: ");
      refresh();
      scanw("%s", fn);
      MLload(&ml, fn);
      finish_entry();
      break;
    case 'w':   /* Write current playlist to file */
      start_entry();
      addstr("Enter filename for playlist save: ");
      refresh();
      scanw("%s", fn);
      MLsave(&ml, fn);
      finish_entry();
      break;
    case '+':   /* Add to playlist */
      MLaddSC(&ml);
      break;
    case '-':   /* Delete from playlist */
      MLdel(&ml);
      break;
    case 'y':
      start_entry();
      addstr("Enter trajectory filename: ");
      refresh();
      scanw("%s", fn);
      finish_entry();
      playTrajectoryFile(fn, 1);
      break;
    case 'Y':
      start_entry();
      addstr("Enter trajectory filename: ");
      refresh();
      scanw("%s", fn);
      finish_entry();
      playViaTrajectoryFile(fn, .2);
      break;
    case 'r':
      start_entry();
      addstr("Enter trajectory filename: ");
      refresh();
      scanw("%s", fn);
      finish_entry();
      teachMoveManual(fn);
      break;
    case 'R':
      start_entry();
      addstr("Enter trajectory filename: ");
      refresh();
      scanw("%s", fn);
      finish_entry();
      teachMove(fn);
      break;
    case 27: //Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR)
      {
        if (chr == 91)
        {
          if ((chr = getch()) != ERR)
          {
            if (chr == 67) //Right arrow
            {
              cpuck++;
              if (cpuck >= npucks)
                cpuck = 0;
            }
            else if (chr == 68) //Left arrow
            {
              cpuck--;
              if (cpuck < 0)
                cpuck = npucks - 1;
            }
            else
            {
              while(getch()!=ERR)
              {
                // Do nothing
              }
            }
          }
        }
        else
        {
          while(getch()!=ERR)
          {
            // Do nothing
          }
        }
      }
      break;

    default:
      while(getch()!=ERR)
      {
        // Do nothing
      }
      //syslog(LOG_ERR,"Caught unknown keyhit %d",c);
      break;
    }
  }
}


/** Evaluate a playlist
    Checks to see if we have reached the next point and loads a new trajectory if we have.
*/
void WACKYeval()
{
  int ret,cnt;
  double pos,vel,acc,min,max;

  if (WACKYplay)
  {
    ret = 0;
    for (cnt = 0; cnt < npucks; cnt++)
    {
      if (wam.wamDriverData->sc[cnt].trj.state == 0)
      {
        switch(cnt)
        {
        case 0: // J1
          min = -2.45;
          max = +2.45;
          break;
        case 1: // J2
          min = -1.00;//-1.95;
          max = +1.00;//+1.95;
          break;
        case 2: // J3
          min = -2.78;
          max = +2.78;
          break;
        case 3: // J4
          min = -2.5;//-0.80;
          max = +0.5;//+2.00;
          break;
        case 4: // J5
          min = -4.50;
          max = +0.80;
          break;
        case 5: // J6
          min = -1.40;
          max = +1.40;
          break;
        case 6: // J7
          min = -2.50;
          max = +2.50;
          break;
        default:
          min = -1.40;
          max = +1.40;
          break;
        }
        pos = drand48() * (max - min) + min;
        //vel = (0.5-drand48()) * 1.4 * 2;
        SCstarttrj(&(wam.wamDriverData->sc[cnt]), pos);
      }
    }
  }
}
/*======================================================================*
 *                                                                      *
 *             Copyright (c) 2003 Barrett Technology, Inc.              *
 *                        139 Main Street                               *
 *                       Kendall/MIT Square                             *
 *                  Cambridge, MA  02142-1528  USA                      *
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
 *  ******************************************************************  *
 *
 * CVS info: $Id$
 * CVS automatic log (prune as desired):
 * $Log$
 *
 *======================================================================*/
