/*======================================================================*
 *  Module .............btdiag
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......29 Apr 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 * 050429 TH - Reworked for new library
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

#define fer(_x_,_y_) for (_x_ = 0;_x_ < _y_;_x_++)
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
wam_struct *wam;
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

  struct sched_param mysched;

  Mpos = new_vn(10);
  Mtrq = new_vn(10);
  Jpos = new_vn(10);
  Jtrq = new_vn(10);
  wv = new_vn(10);
  tmh = new_mh();
  const_vn(wv, 0.0, -2.017, -0.011, 0.88, 0.0, 0.0, 0.0);

  /* Initialize the ncurses screen library */
  init_ncurses();

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  syslog(LOG_ERR, "syslog initalized by WAM application");
  //addstr("syslog initalized\n");
  //refresh();

  /* Initialize the display mutex */
  test_and_log(
      pthread_mutex_init(&(disp_mutex),NULL),
      "Could not initialize mutex for displays.");
  
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


  /* This is a hack for now, so references to global WAM don't
     have to be updated through all the driver code */
  wam = GetWAM();

  /* Set up the WAM data structure, init kinematics, dynamics, haptics */
  err =  InitWAM(wam, "wam.dat");
  if(err)
    {
      CloseSystem();
      closelog();
      endwin();
      exit(1);

    }


  setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous

  npucks = wam->num_actuators; // Get the number of initialized actuators

  start_control_threads(0, 2000000L, WAMControlThread, (void *)0);

  signal(SIGINT, sigint_handler); //register the interrupt handler
  startup_stage = 1;

  StartDisplayThread();

  
  while (!done)
  {
    
    if ((chr = getch()) != ERR) //Check buffer for keypress
      ProcessInput(chr);

    usleep(100000); // Sleep for 0.1s
  }
  
  Shutdown();
  syslog(LOG_ERR, "CloseSystem");
  CloseSystem();
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
  for(cnt = 0; cnt < column_offset + wam->num_actuators * column_width; cnt++)
  {
    strcat(divider, "-");
  }

  clear();
  refresh();
  while (!done)
  {
    test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
    switch (present_screen)
    {
    case MAIN_SCREEN:
      RenderScreen();
      break;
    }
    pthread_mutex_unlock(&(disp_mutex));
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
  test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
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

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderScreen() //{{{
{
  //int val;
  int cnt, idx,cnt2 ,Mid;
  int line;
  double gimb[4];



  /***** Display the interface text *****/
  line = 0;
  mvprintw(line , 0, "Barrett Technology Diagnostic Application (btdiag)");
  ++line;

  if (disp_puck)
  {
    mvprintw(line , 0, "   ACTUATOR: ");
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
  mvprintw(line, 0, "[c] Command: ");
  ++line;
  mvprintw(line, 0, "%s", divider);
  ++line;
  ++line;
  entryLine = line;

  mvprintw(line, 0, "%s", divider);
  ++line;

  mvprintw(line , 0, "[ ]                [ ]              [   ]                   ");
  ++line;
  mvprintw(line , 0, "[i,I] Idle Mode    [x] Exit         [g] (%c) Gravity toggle ", wam->Gcomp ? '*' : ' ');
  ++line;
  mvprintw(line , 0, "[   ]              [ ]              [ ]                     ");
  ++line;
  mvprintw(line , 0, "[p,P] PID Mode     [ ]              [z,Z] ZeroWAM (Man/Auto)");
  ++line;
  mvprintw(line , 0, "[s,S] Start Trj    [ ] Joint/Puck   [   ]                   ");
  ++line;

  /***** Display the data *****/
  getWAMjoints(Mpos, Mtrq, Jpos, Jtrq);

  if(useGimbals) // Convert the gimbals Q4.12 position data into radians
  {
    getGimbalsAngles(gimb);
    setval_vn(Jpos,4,gimb[0]);
    setval_vn(Jpos,5,gimb[1]);
    setval_vn(Jpos,6,gimb[2]);
  }
  
  fer (cnt, wam->num_actuators)
  {
    line = 1;
    idx = wam->act[cnt].puck.ID;
    Mid = MotorID_From_ActIdx(cnt);
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
      mvprintw(line , column_offset + column_width*cnt, "%8d ", wam->act[cnt].puck.position);
      ++line;
      mvprintw(line , column_offset + column_width*cnt, "%8d ", wam->act[cnt].puck.torque_cmd);
      ++line;
    }
    else
    {
      if (Mid == MotorID_From_ActIdx(cpuck))
        mvprintw(line , column_offset + column_width*cnt, " -[%2d]- ", Mid+1);
      else
        mvprintw(line , column_offset + column_width*cnt, "   %2d  ", Mid+1);
      ++line;

      if (wam->sc[Mid].mode == SCMODE_IDLE)
        mvprintw(line , column_offset + column_width*cnt, "IDLE  ");
      else if (wam->sc[Mid].mode == SCMODE_TORQUE)
        mvprintw(line , column_offset + column_width*cnt, "TORQUE");
      else if (wam->sc[Mid].mode == SCMODE_PID)
        mvprintw(line , column_offset + column_width*cnt, "PID   ");
      else
        mvprintw(line , column_offset + column_width*cnt, "UNK:%d ", wam->sc[Mid].mode);
      ++line;

      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jpos,Mid));
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jtrq,Mid));
      ++line;
    }

    mvprintw(line, column_offset + column_width*cnt, "%8d ", temperatures[wam->act[cnt].puck.ID] );
    ++line;

    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", commands[cnt]);
    ++line;
    ++line;
  }
  cnt2 = line;
  fer (cnt, 10 ){
    line = cnt2;
    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam->sc[cnt].pid.yref);
    ++line;
    if (wam->sc[cnt].trj.state == 0)
      mvprintw(line, column_offset + column_width*cnt, "STOPPED    ");
    else
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam->sc[cnt].trj.end_point);
    ++line;
  }

  refresh();
} //}}}

void clearScreen(void)
{
  test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
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
  int cMid,Mid;
  
  cMid = MotorID_From_ActIdx(cpuck);
  switch (c)
  {
    case 'F': // Force zero
      isZeroed = TRUE;
      break;
    case 'f': // Eliminate torque fault
      setProperty(0,10,TL2,FALSE,8200);
      setProperty(0,10,VL2,FALSE,0); // Eliminate Velocity fault
      break;
    case 'i': // Set present joint controller to Idle mode (sends zero torque)
      SCsetmode(&(wam->sc[cMid]), SCMODE_IDLE);
      break;
    case 'I': // Set ALL joint controllers to Idle mode (sends zero torque)
      fer(cnt, npucks){
        Mid = MotorID_From_ActIdx(cnt);
        SCsetmode(&(wam->sc[Mid]), SCMODE_IDLE);
      }
      break;
    case 't': // Set present joint controller to Torque mode
      SCsetmode(&(wam->sc[cMid]), SCMODE_TORQUE);
      break;
    case 'T': // Set ALL joint controllers to Torque mode
      fer(cnt, npucks) {
        Mid = MotorID_From_ActIdx(cnt);
        SCsetmode(&(wam->sc[Mid]), SCMODE_TORQUE);
      }
      break;
    case 'p': // Set present joint controller to PID mode
      SCsetmode(&(wam->sc[cMid]), SCMODE_PID);
      break;
    case 'P': // Set ALL joint controllers to PID mode
      fer(cnt, npucks){
        Mid = MotorID_From_ActIdx(cnt);
        SCsetmode(&(wam->sc[Mid]), SCMODE_PID);
      }
      break;
    case 'x':
    case 'X': /* eXit */
      done = 1;
      break;
    case ' ': /* Toggle puck/joint display */
      disp_puck = !disp_puck;
      clear();
      break;
    case 'g': /* Toggle gravity compensation */
      toggleGcomp();
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
        temperatures[wam->act[cnt].puck.ID] = -1;
        tmp = GetProp(cnt, TEMP);
        temperatures[wam->act[cnt].puck.ID] = tmp;
      }
      break;
    case 'k': /* Enter PID constants for present puck \bug Check to see if we are in control mode 0 and able to change gains*/
        start_entry();
        addstr("Enter pid constants \"Kp Kd Ki\": ");
        refresh();
        ret = scanw("%lf %lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]), &(newpid[3]));
        //if (ret == 3)
        SCsetpid(&(wam->sc[cMid]), newpid[0], newpid[1], newpid[2], newpid[3]);
        finish_entry();
      break;
    case 'K': /* Enter PID constants for all pucks \bug Check to see if we are in control mode 0 and able to change gains*/
        start_entry();
        addstr("Enter pid constants \"Kp Kd Ki\": ");
        refresh();
        ret = scanw("%lf %lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]), &(newpid[3]));
        //if (ret == 3)
        fer(cnt, npucks){
          Mid = MotorID_From_ActIdx(cnt);
          SCsetpid(&(wam->sc[Mid]), newpid[0], newpid[1], newpid[2], newpid[3]);
        }
        finish_entry();
      break;
    case 'v': /* Enter trajectory constants for present puck */
        start_entry();
        addstr("Enter trajectory constants \"Velocity Acceleration\": ");
        refresh();
        ret = scanw("%lf %lf", &(newpid[0]), &(newpid[1]));
        SCsettrjprof(&(wam->sc[cMid]), newpid[0], newpid[1]);
        finish_entry();
      break;
    case 'V': /* Enter trajectory constants for all pucks */
         start_entry();
        addstr("Enter trajectory constants \"Velocity Acceleration\": ");
        refresh();
        ret = scanw("%lf %lf", &(newpid[0]), &(newpid[1]));
        fer(cnt, npucks){
          Mid = MotorID_From_ActIdx(cnt);
          SCsettrjprof(&(wam->sc[Mid]), newpid[0], newpid[1]);
        }
        finish_entry();
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
