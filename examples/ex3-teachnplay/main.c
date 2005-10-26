/*======================================================================*
 *  Module .............ex3
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......06 Oct 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file ex3.c
    \brief An interactive teach and play demo.

    Read the code to see what you can do with it.
    
This program allows a user to test and interact with the teach and play
features of the WAM library.

The user can switch between cartesian space and joint space. The toggle 
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


pthread_mutex_t disp_mutex;

int entryLine;

int useGimbals      = FALSE;
int done            = FALSE;

int gcompToggle = 0;

btstatecontrol *active_bts;

vect_n* active_pos;
vect_n* active_trq;
vect_n *wv;

char active_file[250];
char *user_def = "User edited point list";
wam_struct *wam;

vectray *vr;
via_trj_array *vta = NULL,*vtb = NULL;
int cteach = 0;
extern int isZeroed;
static RT_TASK *mainTask;
btthread disp_thd,wam_thd;
double sample_rate;
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
  init_ncurses(); atexit((void*)endwin);
  

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  atexit((void*)closelog);
  
  /* Initialize the display mutex */
  test_and_log(
    pthread_mutex_init(&(disp_mutex),NULL),
    "Could not initialize mutex for displays.");


  mvprintw(1,0,"Make sure the all WAM power and signal cables are securely");
  mvprintw(2,0,"fastened, then turn on the main power to WAM and press <Enter>");
  while((chr=getch())==ERR) usleep(5000);
  mvprintw(4,0,"Make sure all E-STOPs are released, then press Shift-Idle");
  mvprintw(5,0,"on the control pendant. Then press <Enter>");
  while((chr=getch())==ERR) usleep(5000);

  mvprintw(7,0,"Place WAM in its home (folded) position, then press <Enter>");
  while((chr=getch())==ERR) usleep(5000);

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



  signal(SIGINT, sigint_handler); //register the interrupt handler

  setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous

  const_vn(wv, 0.0, -1.997, 0.0, +3.14, 0.0, 0.0, 0.0); //Blank link home pos
  DefineWAMpos(wam,wv);
  //prep modes
  active_bts = &(wam->Jsc);
  setmode_bts(active_bts,SCMODE_IDLE);
  active_pos = wam->Jpos;
  active_trq = wam->Jtrq;
  
  //new trajectory
  vta = new_vta(len_vn(active_pos),50);
  register_vta(active_bts,vta);
  active_file[0] = 0;

  wam_thd.period = 0.002;
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)&wam_thd);

  btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

  while (!done)
  {

    if ((chr = getch()) != ERR) //Check buffer for keypress
      ProcessInput(chr);

    evalDL(&(wam->log));
    ServiceContinuousTeach();

    usleep(100000); // Sleep for 0.1s
  }
  
  btthread_stop(&wam_thd); //Kill WAMControlThread 

  exit(1);
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
    exit(1);
}



/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
  int cnt,err;

  clear();
  refresh();
  while (!done)
  {
    test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");

    RenderMAIN_SCREEN();

    pthread_mutex_unlock(&(disp_mutex));
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

  mvprintw(line , 0, "Barrett Technology - BTdiag");

  if (active_bts == &(wam->Jsc)){
    mvprintw(line , 30, "Mode: Joint Space    ");
  }
  else if (active_bts == &(wam->Csc)){
    mvprintw(line , 30, "Mode: Cartesian Space");
  }
  else{
    mvprintw(line , 30, "Mode: Undefined!!!   ");
  }
  line+=2;
    
  if (cteach)
    mvprintw(line , 50, "Teaching continuous trajectoy");
  else if (vta == NULL)
    mvprintw(line , 50, "Trajectory: NONE             ");
  else
    mvprintw(line , 50, "Trajectory: %s",active_file);

  if (getmode_bts(active_bts)==SCMODE_IDLE)
    mvprintw(line , 24, "Constraint: IDLE      ");
  else if (getmode_bts(active_bts)==SCMODE_POS)
    mvprintw(line , 24, "Constraint: POSITION  ");
  else if (getmode_bts(active_bts)==SCMODE_TRJ)
    mvprintw(line , 24, "Constraint: TRAJECTORY");
  else
    mvprintw(line , 24, "Constraint: UNDEFINED!");

  ++line;
  mvprintw(line , 0, "Vel: %+8.4f Acc: %+8.4f  Dest:%s ",active_bts->vel,active_bts->acc,sprint_vn(vect_buf1,active_dest));

  line+=3;
  mvprintw(line, 0 , "Position :%s ", sprint_vn(vect_buf1,active_pos));
  ++line;
  mvprintw(line, 0 , "Target :%s ", sprint_vn(vect_buf1,active_bts->qref));
  ++line;
  mvprintw(line, 0 , "Force :%s ", sprint_vn(vect_buf1,active_trq));
  line+=3;
  if (vta != NULL)
  {//print current point
    vr = get_vr_vta(vta);
    cpt = get_current_point_vta(vta);
    nrows = numrows_vr(vr);
    mvprintw(line,0,"Current Index:%d of %d    ",cpt,nrows-1);
    line++;

    mvprintw(line, 0 ,   "Previos Teach Point :                                                       ");
    mvprintw(line+1, 0 , "Current Teach Point :                                                       ");
    mvprintw(line+2, 0 , "   Next Teach Point :                                                       ");

    if (nrows > 0)
    {
      if (nrows != cpt)
        mvprintw(line+1, 21 , "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
      else
        mvprintw(line+1, 21 , "END OF LIST                                      ");
    }
    else
      mvprintw(line+1, 21 , "EMPTY LIST                                      ");

    if (nrows >0 && cpt > 0)
      mvprintw(line, 21,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));

    if (nrows >1)
      if (cpt < nrows-1)
        mvprintw(line+2, 21,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
      else if (cpt == nrows-1)
        mvprintw(line+2, 21, "END OF LIST                                       ");
    line +=3;
  }
  else
  {
    line++;
    line++;
    mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
    line +=2;
  }
  line+=3;

  mvprintw(line,0,"bts: state:%d",active_bts->mode);
  //if(active_bts->btt.dat != NULL)
  mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
  entryLine = line + 2;
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

  char fn[250],chr;
  int ret;
  int done1;

  switch (c)
  {
  case 'x':  /* eXit */
  case 'X':  /* eXit */
    done = 1;
    break;
  //case 'z':  /* Send home-position to WAM */
  //  const_vn(wv, 0.0, -1.997, 0.0, +3.14, 0.0, 0.0, 0.0); //gimbals
  //  DefineWAMpos(wam,wv);
  //  break;
  case 'g':  /* Set gravity compensation */
    start_entry();
    addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
    refresh();
    scanw("%lf\n",  &tvel);
    SetGravityComp(wam,tvel);
    finish_entry();
    break;

  case '_':  /* Refresh display */
    clearScreen();
    break;

  case '\t': /* Switch between jointspace and cartesian space trajectories*/
    if (vta != NULL)
      destroy_vta(&vta); //empty out the data if it was full

    if (active_bts == &(wam->Jsc))
    { //switch to cartesian space mode.
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      setmode_bts(&(wam->Csc),SCMODE_IDLE);
      active_bts = &(wam->Csc);
      active_pos = wam->R6pos;
      active_trq = wam->R6force;
      active_dest = cdest;
      clearScreen();
      test_and_log(pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
      //present_screen = CARTSPACE_SCREEN;
      pthread_mutex_unlock(&(disp_mutex));
    }
    else
    {
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      setmode_bts(&(wam->Csc),SCMODE_IDLE);
      active_bts = &(wam->Jsc);
      active_pos = wam->Jpos;
      active_trq = wam->Jtrq;
      active_dest = jdest;
      clearScreen();
      test_and_log(pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
      //present_screen = JOINTSPACE_SCREEN;
      pthread_mutex_unlock(&(disp_mutex));
    }
    break;
  
  case 'p':  /* Turn on/off Constraint */
    if (getmode_bts(active_bts)!=SCMODE_IDLE)
      setmode_bts(active_bts,SCMODE_IDLE);
    else
      setmode_bts(active_bts,SCMODE_POS);
    break;

  case '.':  /* Play presontly loaded trajectory */
    setmode_bts(active_bts,SCMODE_TRJ);
    moveparm_bts(active_bts,vel,acc);
    active_bts->loop_trj = 0;
    prep_trj_bts(active_bts);
    elapsed = 0;
    done1 = 0;
    while (movestatus_bts(active_bts) == BTTRAJ_INPREP && !done1)
    {
      usleep(100000);
      elapsed ++;
      if (elapsed > 100)
        done1 = 1;

    }
    syslog(LOG_ERR,"Done with prep");

    if (elapsed < 100)
      start_trj_bts(active_bts);
    else
    {
      setmode_bts(active_bts,SCMODE_IDLE);
      stop_trj_bts(active_bts);
    }
    break;
    
    case ',':  /* Simulate presently loaded trajectory */
      sim_vta(vta,0.002,getval_vn(idx_vr(get_vr_vta(vta),numrows_vr(get_vr_vta(vta))-1),0),"sim.csv");
    break;
    case '?':  /* Play presontly loaded trajectory */
    setmode_bts(active_bts,SCMODE_TRJ);
    moveparm_bts(active_bts,vel,acc);
    active_bts->loop_trj = 1;
    prep_trj_bts(active_bts);
    elapsed = 0;
    done1 = 0;
    while (movestatus_bts(active_bts) == BTTRAJ_INPREP && !done1)
    {
      usleep(100000);
      elapsed ++;
      if (elapsed > 100)
        done1 = 1;

    }
    syslog(LOG_ERR,"Done with prep");

    if (elapsed < 100)
      start_trj_bts(active_bts);
    else
    {
      setmode_bts(active_bts,SCMODE_IDLE);
      stop_trj_bts(active_bts);
    }
    break;
  case '/':  /* Stop presontly loaded trajectory */
    stop_trj_bts(active_bts);
    break;

  case 'Y':  /* Start continuos teach */
    StartContinuousTeach(1,50,"teachpath");
    cteach = 1;
    break;
  case 'y': /*Stop continuos teach and load into memory*/
    StopContinuousTeach();
    DecodeDL("teachpath","teach.csv",0);
    cteach = 0;
    if (vta != NULL)
      destroy_vta(&vta); //empty out the data if it was full
    strcpy(active_file,"teach.csv");
    vta = read_file_vta(active_file,20);
    register_vta(active_bts,vta);
    break;
    //Free mode:
  case 'l':  /* Load trajectory file */
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
    
  case 'w':  /*  Save trajectory to a file */
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter filename for trajectory: ");
      refresh();
      scanw("%s", fn);
      if (vta != NULL)
      {
        write_file_vta(vta,fn);
        strcpy(active_file,fn);
      }
      finish_entry();
    }
    else {
       start_entry();
      addstr("You must be in idle mode!: ");
      refresh();
      sleep(1);
      finish_entry();
    }
    break;
  case 'n': /* Create a new trajectory*/
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter the max number of points that will be in your trajectory: ");
      refresh();
      ret = scanw("%d", &dtmp);
      if (vta != NULL)
        destroy_vta(&vta);

      strcpy(active_file,user_def);
      vta = new_vta(len_vn(active_pos),dtmp);
      register_vta(active_bts,vta);
      active_file[0] = 0;
      finish_entry();
    }
    break;
  case 'M':  /* Move to a location */
     start_entry();
      addstr("Enter comma seperated destination \".2,.4,...\": ");
      refresh();
      getstr( fn);
      strcat(fn,"\n");
      //syslog(LOG_ERR,"Moveto:%s",fn);
      finish_entry();
      if (getmode_bts(active_bts)!=SCMODE_TRJ){
        
        setmode_bts(active_bts,SCMODE_TRJ);
        fill_vn(active_dest,0.25);
        csvto_vn(active_dest,fn);
        moveparm_bts(active_bts,vel,acc);
        stop_trj_bts(active_bts);
        if(moveto_bts(active_bts,active_dest))
          syslog(LOG_ERR,"Moveto Died",fn);
      }
      break;
  case 'm':  /* Move to the presently selected trajectory point*/
  case '<':  /* Select next point in the trajectory */
    prev_point_vta(vta);
    break;
  case '>':  /* Select previous point in the trajectory */
    next_point_vta(vta);
    break;
  case '+':  /* Insert a point in the trajectory */
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      ins_point_vta(vta,active_pos);
    }
    break;
  case '-':  /* Remove a point in the trajectory */
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      del_point_vta(vta);
    }
    break;
  case 's':  /* Scale the present trajectory */
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter trajectory velocity: ");
      refresh();
      ret = scanw("%lf\n", &vel);
      if(vta != NULL)
        scale_vta(vta,vel,acc);
      finish_entry();
    }
    break;
  case 'S':  /* Set the corner acceleration */
    if(getmode_bts(active_bts)==SCMODE_IDLE)
    {
      start_entry();
      addstr("Enter Corner Acceleration: ");
      refresh();
      ret = scanw("%lf\n", &acc);
      if(vta != NULL)
        set_acc_vta(vta,acc);
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

          }
          else if (chr == 68) //Left arrow
          {

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
 *          Copyright (c) 2005 Barrett Technology, Inc.                 *
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
