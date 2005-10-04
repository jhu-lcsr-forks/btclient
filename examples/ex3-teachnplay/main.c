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
 \todo
 Cartesian playback
 
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

#include <rtai_lxrt.h>
#include <rtai_sem.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/

#include "btwam.h"


/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define column_offset     (15)
#define column_width      (11)

enum {MAIN_SCREEN = 0, JOINTSPACE_SCREEN, CARTSPACE_SCREEN, LAST_SCREEN};


/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define fer(_x_,_y_) for (_x_ = 0;_x_ < _y_;_x_++)
/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
char divider[column_offset + (column_width * MAX_NODES)];

pthread_mutex_t disp_mutex;

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
int done            = FALSE;

int gcompToggle = 0;

int trjisdirty = 0; //=1 if edits made to a trajectory data structure.
int trjidx;

btstatecontrol *active_bts;


vect_n* active_pos;
vect_n* active_trq;
vect_n *wv;
char active_file[250];
wam_struct *wam;
vectray *vr;
via_trj_array *vta = NULL,*vtb = NULL;


extern int isZeroed;
static RT_TASK *mainTask;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void RenderMAIN_SCREEN(void);
void RenderJOINTSPACE_SCREEN(void);
void RenderCARTSPACE_SCREEN(void);
void ProcessInput(int c);
void Shutdown(void);
void DisplayThread(void);
void StartDisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);

int WAMcallback(struct btwam_struct *wam);

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
  wv = new_vn(7);
  /* Initialize the ncurses screen library */
  init_ncurses(); atexit(endwin);
  

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

  /* Initialize the display mutex */
  test_and_log(
    pthread_mutex_init(&(disp_mutex),NULL),
    "Could not initialize mutex for displays.");


  /* Initialize rtlinux subsystem */
  mysched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
  if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1)
  {
    syslog(LOG_ERR, "Error setting up linux (native) scheduler");
  }

  mainTask = rt_task_init(nam2num("main01"), 0, 0, 0); /* defaults */

  if(test_and_log(
            InitializeSystem("actuators.dat","buses.dat","motors.dat","pucks.dat"),
            "Failed to initialize system"))
  {
    return -1;
  }

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

  /* Set up the WAM data structure, init kinematics, dynamics, haptics */
  err =  InitWAM("wam.dat");
  if(err)
  {
    CloseSystem();
    closelog();
    endwin();
    exit(1);

  }

  /* Obtain a pointer to the wam state object */
  wam = GetWAM();
  active_bts = &(wam->Jsc);
  signal(SIGINT, sigint_handler); //register the interrupt handler
  startup_stage = 1;

  /* Initialize our data logging structure */
  wam->logdivider = 1;
  PrepDL(&(wam->log),7);
  AddDataDL(&(wam->log),&(wam->log_time),sizeof(double),2,"Time");
  AddDataDL(&(wam->log),&(wam->loop_time),sizeof(RTIME),BTLOG_LONGLONG,"loop_time");
  AddDataDL(&(wam->log),&(wam->loop_period),sizeof(RTIME),BTLOG_LONGLONG,"loop_period");
  AddDataDL(&(wam->log),&(wam->readpos_time),sizeof(RTIME),BTLOG_LONGLONG,"readpos_time");
  AddDataDL(&(wam->log),&(wam->user_time),sizeof(RTIME),BTLOG_LONGLONG,"user_time");
  AddDataDL(&(wam->log),&(wam->Jsc_time),sizeof(RTIME),BTLOG_LONGLONG,"Jsc_time");

  InitDL(&(wam->log),1000,"datafile.dat");

  setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous

  npucks = wam->num_actuators; // Get the number of initialized actuators

  //prep modes
  setmode_bts(&(wam->Csc),SCMODE_IDLE);
  active_bts = &(wam->Jsc);
  active_pos = wam->Jpos;
  active_trq = wam->Jtrq;
  //new trajectory
  vta = new_vta(len_vn(active_pos),50);
  register_vta(active_bts,vta);
  active_file[0] = 0;


  start_control_threads(10, 0.002, WAMControlThread, (void *)0);

  StartDisplayThread();

  DLon(&(wam->log));
  while (!done)
  {

    if ((chr = getch()) != ERR) //Check buffer for keypress
      ProcessInput(chr);

    evalDL(&(wam->log));
    ServiceContinuousTeach();

    usleep(100000); // Sleep for 0.1s
  }
  DLon(&(wam->log));
  Shutdown();
  syslog(LOG_ERR, "CloseSystem");
  CloseSystem();
  CloseDL(&(wam->log));
  DecodeDL("datafile.dat","dat.csv",1);
  freebtptr();
}

int WAMcallback(struct btwam_struct *wam)
{

  return 0;
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
  int        err;

  pthread_attr_init(&display_attr);
  pthread_attr_setschedpolicy(&display_attr, SCHED_FIFO);
  pthread_attr_getschedparam(&display_attr, &display_param);
  display_param.sched_priority = 10;
  pthread_attr_setschedparam(&display_attr, &display_param);
  pthread_create(&display_thd_id, &display_attr, (void*)DisplayThread, 0);

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
      RenderMAIN_SCREEN();
      break;
    case JOINTSPACE_SCREEN:
      RenderMAIN_SCREEN();
      break;
    case CARTSPACE_SCREEN:
      RenderMAIN_SCREEN();
      break;
    default:
      RenderMAIN_SCREEN();
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
void RenderMAIN_SCREEN()
{
  //int val;
  int cnt, idx,Mid,cp;
  int line,line2;
  int cpt,nrows;
  double gimb[4];
  vectray* vr;
  char vect_buf1[250];

  //clear();
  /***** Display the interface text *****/
  line = 0;
  //mvprintw(line , 0, "012345678 1 2345678 2 2345678 3 2345678 4 2345678 5 2345678 6 2345678 7 2345678-8");++line;
  
  mvprintw(line , 0, "Barrett Technology Teach & Play Example");
  ++line;


  if (active_bts == &(wam->Jsc))
    mvprintw(line , 0, "Mode: Joint Space");
  else if (active_bts == &(wam->Csc))
    mvprintw(line , 0, "Mode: Cartesian Space");
  else
    mvprintw(line , 0, "Mode: Undefined!!!");

  if (vta == NULL)
    mvprintw(line , 50, "Trajectory: NONE");
  else
    mvprintw(line , 50, "Trajectory: %s",active_file);
  
  if (getmode_bts(active_bts)==SCMODE_IDLE)
    mvprintw(line , 24, "Constraint: IDLE");
  else if (getmode_bts(active_bts)==SCMODE_POS)
    mvprintw(line , 24, "Constraint: POSITION");
  else if (getmode_bts(active_bts)==SCMODE_TRJ)
    mvprintw(line , 24, "Constraint: TRAJECTORY");
  else
    mvprintw(line , 24, "Constraint: UNDEFINED!!!");

  ++line;++line;

  mvprintw(line, 0 , "Position :%s ", sprint_vn(vect_buf1,active_pos));
  ++line;
  /*
  mvprintw(line, 0 , "Pos :%s ", sprint_vn(vect_buf1,active_bts->q));
  ++line;
  mvprintw(line, 0 , "Targ Pos :%s ", sprint_vn(vect_buf1,active_bts->qref));
  ++line;
  mvprintw(line, 0 , "Time :%f ", active_bts->dt);
  ++line;*/
  mvprintw(line, 0 , "Force :%s ", sprint_vn(vect_buf1,active_trq));
  ++line;
  ++line;
  if (vta != NULL)
  {//print current point
    vr = get_vr_vta(vta);
    cpt = get_current_point_vta(vta);
    nrows = numrows_vr(vr);
    mvprintw(line,0,"Current Index:%d of %d    ",cpt,nrows-1);
    line++;
    
    mvprintw(line, 0 ,   "Previos Teach Point :                             ");
    mvprintw(line+1, 0 , "Current Teach Point :                             ");
    mvprintw(line+2, 0 , "   Next Teach Point :                             ");
    
    if (nrows > 0){
      if (nrows != cpt)
        mvprintw(line+1, 21 , "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
      else
        mvprintw(line+1, 21 , "END OF LIST                                      ");
    }
    else mvprintw(line+1, 21 , "EMPTY LIST                                      ");
    
    if (nrows >0 && cpt > 0)
      mvprintw(line, 21,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));
    
    if (nrows >1)
      if (cpt < nrows-1)
        mvprintw(line+2, 21,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
      else if (cpt == nrows-1)
        mvprintw(line+2, 21, "END OF LIST                                       ");
    line +=3;
  }
  else{
    line++;
    line++;
    mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
    line +=2;
  }
  line++;line++;
  mvprintw(line,0,"bts: state:%d",active_bts->mode);
  if(active_bts->btt.dat != NULL)mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
  entryLine = line + 2;
  refresh();
}
/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderJOINTSPACE_SCREEN()
{

  refresh();
}
/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderCARTSPACE_SCREEN()
{

  refresh();
}
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
  int cnt,elapsed = 0;
  double ftmp,tacc,tvel;
  int dtmp;
  int cMid,Mid;
  char fn[250],chr;
  int ret;
  int done1;

  cMid = MotorID_From_ActIdx(cpuck);
  switch (c)
  {
  case 'x':
  case 'X': /* eXit */
    done = 1;
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
  case 'z': /* Send zero-position to WAM */
    const_vn(wv, 0.0, -1.997, 0.0, +3.14, 0.0, 0.0, 0.0); //gimbals
    //const_vn(wv, 0.0, -2.017, 0.0, 3.14, 0.0, 0.0, 0.0); //blanklink
    SetWAMpos(wv);
    break;
  case 'g': /* Toggle gravity compensation */
    if(gcompToggle)
    {
      gcompToggle = 0;
      setGcomp(0.0);
    }
    else
    {
      gcompToggle = 1;
      setGcomp(1.0);
    }
    break;

    case '_': /* Refresh display */
    clearScreen();
    break;

  case '\t':  //Switch between jointspace and cartesian space trajectories
    if (vta != NULL)
      destroy_vta(&vta); //empty out the data if it was full

    if (active_bts == &(wam->Jsc))
    { //switch to cartesian space mode.
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      active_bts = &(wam->Csc);
      active_pos = wam->R6pos;
      active_trq = wam->R6force;
      clearScreen();
      test_and_log(pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
      //present_screen = CARTSPACE_SCREEN;
      pthread_mutex_unlock(&(disp_mutex));
    }
    else
    {
      setmode_bts(&(wam->Csc),SCMODE_IDLE);
      active_bts = &(wam->Jsc);
      active_pos = wam->Jpos;
      active_trq = wam->Jtrq;
      clearScreen();
      test_and_log(pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
      //present_screen = JOINTSPACE_SCREEN;
      pthread_mutex_unlock(&(disp_mutex));
    }
    break;
  case 'p':  //Constrain / free
    if (getmode_bts(active_bts)!=SCMODE_IDLE)
      setmode_bts(active_bts,SCMODE_IDLE);
    else
      setmode_bts(active_bts,SCMODE_POS);
    break;

  case 'T':
    setmode_bts(active_bts,SCMODE_TRJ);
    moveparm_bts(active_bts,0.5,0.5);
    prep_trj_bts(active_bts);
    elapsed = 0;
    done1 = 0;
    while (movestatus_bts(active_bts) == BTTRAJ_INPREP && !done1)
    {
      usleep(100000);
      elapsed ++;
      if (elapsed > 100) done1 = 1;
      
    }
    syslog(LOG_ERR,"Done with prep");
    //wam->Jsc.trj->state = BTTRAJ_READY;
    if (elapsed < 100)
      start_trj_bts(active_bts);
    else {
      setmode_bts(active_bts,SCMODE_IDLE);
      stop_trj_bts(active_bts);
    }
    break;
  case 't':
    stop_trj_bts(active_bts);
    break;

  case 'Y': //Start continuos teach
    StartContinuousTeach(1,50,"teachpath");
    break;
  case 'y': //Stop continuos teach
    StopContinuousTeach();
    DecodeDL("teachpath","teach.csv",0);

    break;
    //Free mode:
  case 'l':
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter filename for trajectory: ");
      refresh();
      scanw("%s", active_file);
      if (vta != NULL)
        destroy_vta(&vta); //empty out the data if it was full

      vta = read_file_vta(active_file,20);
      register_vta(active_bts,vta);
      finish_entry();
    }
    break;
  case 'L':
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter filename for trajectory: ");
      refresh();
      scanw("%s", active_file);
      finish_entry();
      
      if (vtb != NULL)
        destroy_vta(&vtb); //empty out the data if it was full

      vtb = read_file_vta(active_file,0);
      sim_vta(vtb,0.002,30.0,"sim.csv");
      
      
    }
    break;
    //  Save trajectory
  case 'w':
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter filename for trajectory: ");
      refresh();
      scanw("%s", fn);
      if (vta != NULL)
      {
        
        write_file_vta(vta,fn);
      }
      finish_entry();
    }
    break;
  case 'n': //New vta
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter the max number of points that will be in your trajectory: ");
      refresh();
      ret = scanw("%d", &dtmp);
      if (vta != NULL)
        destroy_vta(&vta);


      vta = new_vta(len_vn(active_pos),dtmp);
      register_vta(active_bts,vta);
      active_file[0] = 0;
      finish_entry();
    }
    break;
  case '<':
    prev_point_vta(vta);
    break;
  case '>':
    next_point_vta(vta);
    break;
  case '+': //Insert point
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      ins_point_vta(vta,active_pos);
    }
    break;
  case '-':
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      del_point_vta(vta);
    }
    break;
  case 's': //Scale vta
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter trajectory constants \"Velocity Acceleration\": ");
      refresh();
      ret = scanw("%lf %lf", &tvel, &tacc);
      scale_vta(vta,tvel,tacc);
      finish_entry();
    }
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
 *             Copyright (c) 2005 Barrett Technology, Inc.              *
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
