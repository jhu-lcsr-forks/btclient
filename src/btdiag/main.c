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


#include <rtai_lxrt.h>
#include <rtai_sem.h>



/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "playlist.h"
#include "bthaptics.h"
#include "btwam.h"

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
 
matr_h *myFrame;



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
int konstants       = FALSE;
int done            = FALSE;
int WACKYplay       = FALSE;
int showWhere       = FALSE;
int disp_puck       = FALSE;
int useTRC          = FALSE;
int gcompToggle = 0;

wam_struct *wam;
SC_move_list ml;
ct_traj ct;
vectray *vr;

via_trj_array *vta = NULL;
//extern wam_struct WAM;  // this is a hack that will be resolved - sc
extern int isZeroed;
static RT_TASK *mainTask;
vect_3 *p,*o,*zero_v3,*destination;
vect_n *t,*targ,*negtarg;
vect_n *Mpos, *Mtrq, *Jpos, *Jtrq, *wv;

btgeom_plane myplane,plane2,planes[10];
bteffect_wall mywall,wall[10];
bteffect_wickedwall mywickedwall,mywickedwall2,wickedwalls[10];
bthaptic_object myobject,myobject2,objects[20];
btgeom_sphere mysphere,mysphere2,spheres[10];
bteffect_bulletproofwall mybpwall,bpwall[10];
btgeom_box boxs[10];
bteffect_global myglobal;
vect_3 *p1,*p2,*p3;
bthaptic_scene bth;
btgeom_state pstate;
//btlogger btlog;
btthread disp_thd,wam_thd;
double sample_rate;

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
void init_haptics(void);
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
  zero_v3 = new_v3();
  p = new_v3();
  p1 = new_v3();
  p2 = new_v3();
  p3 = new_v3();
  destination = new_v3();
  Mpos = new_vn(10);
  Mtrq = new_vn(10);
  Jpos = new_vn(10);
  Jtrq = new_vn(10);
  wv = new_vn(10);
  const_vn(wv, 0.0, -2.017, -0.011, 0.88, 0.0, 0.0, 0.0);

 /* Initialize the ncurses screen library */
  init_ncurses(); atexit((void*)endwin);
  

  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  atexit((void*)closelog);
  
  /* Initialize the display mutex */
  test_and_log(
    pthread_mutex_init(&(disp_mutex),NULL),
    "Could not initialize mutex for displays.");

  if(test_and_log(   
    InitializeSystem("wamConfig.txt"),
    "Failed to initialize system")){
    exit(-1);
  }
    
  atexit((void*)CloseSystem);//register CloseSystem for shutdown
  
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
    exit(1);
  }
  signal(SIGINT, sigint_handler); //register the interrupt handler
    
  /* Obtain a pointer to the wam state object */
  wam = GetWAM();
  
  
  /* Initialize our data logging structure */
  wam->logdivider = 1;
  PrepDL(&(wam->log),6);
  AddDataDL(&(wam->log),&(wam->log_time),sizeof(double),2,"Time");
  AddDataDL(&(wam->log),valptr_vn(wam->Jpos),sizeof(btreal)*7,4,"Jpos");
  AddDataDL(&(wam->log),valptr_vn(wam->Jref),sizeof(btreal)*7,4,"Jref");
  AddDataDL(&(wam->log),valptr_vn(wam->Jtrq),sizeof(btreal)*7,4,"Jtrq");
  InitDL(&(wam->log),1000,"datafile.dat");
  
  setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous

  npucks = wam->num_actuators; // Get the number of initialized actuators
  //*****************Haptics scene
  init_haptics();
  
  MLconstruct(&ml, wam->sc, 7, 50); //Initialize the playlist

  wam_thd.period = 0.002;
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)&wam_thd);

  btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

  
  while (!done)
  {
    WACKYeval();  //check to see whether we want to update to another random trajectory
  
    MLeval(&ml);  //Check to see whether the playlist needs attention

    if ((chr = getch()) != ERR) //Check buffer for keypress
      ProcessInput(chr);
    
    evalDL(&(wam->log));
    ServiceContinuousTeach();
    
    usleep(100000); // Sleep for 0.1s
  }
  btthread_stop(&wam_thd); //Kill WAMControlThread 
  MLdestroy(&ml);
  CloseDL(&(wam->log));
  DecodeDL("datafile.dat","dat.csv",1);

  exit(1);
}

int WAMcallback(struct btwam_struct *wam)
{
  eval_state_btg(&(pstate),wam->Cpos);
  eval_bthaptics(&bth,(vect_n*)wam->Cpos,(vect_n*)pstate.vel,(vect_n*)zero_v3,(vect_n*)wam->Cforce);
  apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce, wam->Ctrq);
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
void init_haptics(void)
{
  int cnt;
  new_bthaptic_scene(&bth,10);
  init_state_btg(&pstate,0.002,30.0);

  init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,0.0),const_v3(p2,0.4,0.0,0.0),0);
  init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,0.0),const_v3(p2,0.42,0.0,0.0),1);
  init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,0.0),const_v3(p2,0.3,0.0,0.0),0);
  init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,0.0),const_v3(p2,0.32,0.0,0.0),1);
  //init_wall(&mywall,0.0,10.0);
  for(cnt = 0;cnt < 6;cnt++){
    init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
  }
  init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
  init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,0.0),const_v3(p2,0.7,0.01,0.0),const_v3(p3,0.7,0.0,0.01),0.4,0.6,0.4,1);

  for(cnt = 0;cnt < 6;cnt++){
    init_normal_plane_bth(&objects[cnt],&boxs[0].side[cnt],(void*)&bpwall[0],bulletproofwall_nf);
    //init_normal_plane_bth(&objects[cnt],&boxs[0].side[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
    addobject_bth(&bth,&objects[cnt]);
  }
  
  for(cnt = 0;cnt < 4;cnt++){
    init_normal_sphere_bth(&objects[cnt+6],&spheres[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
    addobject_bth(&bth,&objects[cnt+6]);
  }
  /*
//for box demo
  init_bx_btg(&boxs[1],const_v3(p1,0.5,0.0,0.0),const_v3(p2,0.5,0.01,0.0),const_v3(p3,0.5,0.0,0.01),0.2,0.2,0.2,1);
 
  for(cnt = 0;cnt < 6;cnt++){
    init_bulletproofwall(&bpwall[cnt],0.002,3000.0,0.002,3000.0,50.0,20.0);
    //init_wall(&wall[cnt],6000.0,50.0);
    init_normal_plane_bth(&objects[cnt+10],&boxs[1].side[cnt],(void*)&bpwall[cnt],bulletproofwall_nf);
    addobject_bth(&bth,&objects[cnt+10]);
  }
  init_global_bth(&myobject2, &myglobal,60.0,C_v3(-0.0,0.0,0.0));
  addobject_bth(&bth,&myobject2);
 */
  const_v3(wam->Cpoint,0.0,-0.0,0.0);
  
  registerWAMcallback(WAMcallback);

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
    btmutex_lock(&disp_mutex);
    
      
    
      
    switch (present_screen)
    {
    case MAIN_SCREEN:
      RenderScreen();
      break;
    }
    btmutex_unlock(&disp_mutex);
    usleep(100000);
  }


}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
  int err;
  btmutex_lock(&disp_mutex);
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
  btmutex_unlock(&disp_mutex);
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderScreen() //{{{
{
  //int val;
  int cnt, idx,cnt2 ,Mid;
  int line,line2;
  double gimb[4],*dptr;
  char vect_buf1[80],vect_buf2[80];
  matr_h *mptr;



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
  mvprintw(line , 0, "Cart PIDCmd: ");
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

  mvprintw(line , 0, "[i,I] Idle Mode         [D,d] Haptics On,off   ");
  ++line;
  mvprintw(line , 0, "[p,P] PID Mode          [g  ] (%c) Gravity toggle ", wam->Gcomp ? '*' : ' ');
  ++line;
  mvprintw(line , 0, "[H,h] Cart Pos (on/off) [F  ] WAM is already zeroed ");
  ++line;
  mvprintw(line , 0, "[N,n] Cart Ang (on/off) [z,Z] ZeroWAM (Folded/Up)");
  ++line;
  mvprintw(line , 0, "[r,m] Cart Target, Move [x,X] Exit                ");
  ++line;++line;
  line2 = line;
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
      else if (wam->sc[Mid].mode == SCMODE_POS)
        mvprintw(line , column_offset + column_width*cnt, "POS   ");
      else
        mvprintw(line , column_offset + column_width*cnt, "UNK:%d ", wam->sc[Mid].mode);
      ++line;

      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jpos,Mid));
      ++line;
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", getval_vn(Jtrq,Mid));
      ++line;
    }
    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam->sc[cnt].pid.yref);
    ++line;
    if (wam->sc[cnt].trj.state == 0)
      mvprintw(line, column_offset + column_width*cnt, "STOPPED    ");
    else
      mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", wam->sc[cnt].trj.end_point);
    ++line;
    mvprintw(line, column_offset + column_width*cnt, "%8d ", temperatures[wam->act[cnt].puck.ID] );
    ++line;

    mvprintw(line, column_offset + column_width*cnt, "%+8.4f ", commands[cnt]);


  }

    line = line2;
    mvprintw(line, column_offset , "Jref:%s", sprint_vn(vect_buf1,(vect_n*)wam->Jref));
    ++line;
    mvprintw(line, column_offset , "Jpos:%s", sprint_vn(vect_buf1,(vect_n*)wam->Jpos));
    ++line;
     mvprintw(line, column_offset , "Jtrq:%s  ", sprint_vn(vect_buf1,(vect_n*)wam->Jtrq));
    ++line;
    mvprintw(line, column_offset , "Ctrq:%s qerr:%+8.4f ", sprint_vn(vect_buf1,(vect_n*)wam->Ctrq),wam->qerr);
    ++line;
    mvprintw(line, column_offset , "Cpos:%s ", sprint_vn(vect_buf1,(vect_n*)wam->Cpos));
    ++line;
    mvprintw(line, column_offset , "Cforce:%s ", sprint_vn(vect_buf1,(vect_n*)wam->Cforce));
    ++line;
    mvprintw(line, column_offset , "dist:%s ", sprint_vn(vect_buf1,(vect_n*)objects[6].Istate.pos));
    ++line;   
    mvprintw(line, column_offset , "vel:%s ", sprint_vn(vect_buf1,(vect_n*)pstate.vel));
    ++line;  
    mvprintw(line, column_offset , "time:%+8.4f ", wam->log_time);
    ++line;  
    mvprintw(line, column_offset , "dt:%+8.4f ", wam->dt);
    ++line;  
/*    if (vta != NULL)
    mvprintw(line, column_offset , "state:%d idx:%d n:%d acc:%+8.4f et:%+8.4f  cmd:%+8.4f q0:%+8.4f",
                vta[0].state,vta[0].idx,vta[0].n, vta[0].acc,vta[0].last_et, vta[0].last_cmd, vta[0].q0);
  */  ++line;  
    mvprintw(line, column_offset , "SCmode:%d TrjState:%d ", wam->Jsc.mode,wam->Jsc.btt.state);
    ++line; 
    mptr = T_to_W_trans_bot(&(wam->robot));
    
    
    dptr = mptr->q;
    mvprintw(line, column_offset , "Origin:%+8.4f %+8.4f %+8.4f %+8.4f", dptr[0],dptr[1],dptr[2],dptr[3]);++line;
    mvprintw(line, column_offset , "Origin:%+8.4f %+8.4f %+8.4f %+8.4f", dptr[4],dptr[5],dptr[6],dptr[7]);++line;
    mvprintw(line, column_offset , "Origin:%+8.4f %+8.4f %+8.4f %+8.4f", dptr[8],dptr[9],dptr[10],dptr[11]);++line;

    refresh();
} //}}}

void clearScreen(void)
{
  btmutex_lock(&disp_mutex);
  clear();
  btmutex_unlock(&disp_mutex);
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
  double A, B, C, D;
  FILE *outFile;
  
  cMid = MotorID_From_ActIdx(cpuck);
  switch (c)
  {
    /*
    case '1':
      bth.state = 0;
      init_bx_btg(&mybox,const_v3(p1,0.57,0.0,-0.35),const_v3(p2,0.6,0.0,-0.35),const_v3(p3,0.57,0.0,-0.3),0.01,0.06,0.1,1);
      for(cnt = 0;cnt < 6;cnt++){
        init_normal_plane_bth(&objects[cnt],&mybox.side[cnt],(void*)&bpwall[cnt],bulletproofwall_nf);
      }
      bth.state = 1;
      break;
    case '2':
      bth.state = 0;
      init_bx_btg(&mybox,const_v3(p1,0.573,-0.02,-0.35),const_v3(p2,0.573,0.0,-0.35),const_v3(p3,0.573,-0.02,-0.3),0.033,0.06,0.1,1);
      for(cnt = 0;cnt < 6;cnt++){
        init_normal_plane_bth(&objects[cnt],&mybox.side[cnt],(void*)&bpwall[cnt],bulletproofwall_nf);
      }
      bth.state = 1;
      break;   
    case '3':
      bth.state = 0;
      init_bx_btg(&mybox,const_v3(p1,0.58,-0.03937,-0.35),const_v3(p2,0.54,-0.03937,-0.35),const_v3(p3,0.58,-0.03937,-0.3),0.01,0.08,0.1,1);
      for(cnt = 0;cnt < 6;cnt++){
        init_normal_plane_bth(&objects[cnt],&mybox.side[cnt],(void*)&bpwall[cnt],bulletproofwall_nf);
      }
      bth.state = 1;
      break;      
    case 'u': //Lagrangian parameter measure
      getLagrangian4(&A, &B, &C, &D);
      outFile = fopen("gcomp.dat", "w");
      fprintf(outFile, "%f\n%f\n%f\n%f\n", A, B, C, D);
      fclose(outFile);
      break;*/
    case 'L': //Data logging on
      DLon(&(wam->log));
      break;
    case 'l': //Data logging off
      DLoff(&(wam->log));
      break; 
    case 'D': //Haptics on
      bth.state = 1;
      break;
    case 'd': //Haptics off
      bth.state = 0;
      break;    
    case 'F': // Force zero
      wam->isZeroed = TRUE;
      break;
    case 'f': // Eliminate torque fault
      setProperty(0,10,TL2,FALSE,8200);
      setProperty(0,10,VL2,FALSE,0); // Eliminate Velocity fault
      break;
    case 'i': // Set present joint controller to Idle mode (sends zero torque)
      //SCsetmode(&(wam->sc[cMid]), SCMODE_IDLE);
       setmode_bts(&(wam->Jsc),SCMODE_IDLE);
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
      /*
    case 'H': // Cartesian controller on
      set_v3(wam->Cref,wam->Cpos);
      fer(cnt, 3) {
        start_btPID(&(wam->pid[cnt]));
      }
      break;*/
    case 'h': // Cartesian controller off
      fer(cnt, 3) {
        stop_btPID(&(wam->pid[cnt]));
      }
      break;
    case 'N': // Angular controller on
      set_q(wam->qref,wam->qact);
      start_btPID(&(wam->pid[3]));
      break;
    case 'n': // angular controller off
        stop_btPID(&(wam->pid[3]));
      break;
    case 'p': // Set present joint controller to PID mode
      //SCsetmode(&(wam->sc[cMid]), SCMODE_POS);
      setmode_bts(&(wam->Jsc),SCMODE_POS);
      break;
    case 'P': // Set ALL joint controllers to PID mode
        setmode_bts(&(wam->Jsc),SCMODE_TRJ);
      //fer(cnt, npucks){
      //  Mid = MotorID_From_ActIdx(cnt);
      //  SCsetmode(&(wam->sc[Mid]), SCMODE_POS);
     // }
      break;
        case 's': // Start Trajectory control on present puck
            SCstarttrj(&(wam->sc[cMid]), commands[cpuck]);
            break;
        case 'S': // Start Trajectory control on all pucks
        fer(cnt, npucks) {
          Mid = MotorID_From_ActIdx(cnt);
          SCstarttrj(&(wam->sc[Mid]), commands[cnt]);
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
      if(gcompToggle){
        gcompToggle = 0;
        setGcomp(0.0);
      }
      else {
        gcompToggle = 1;
        setGcomp(1.0);
      }

      refresh();
      break;
    case 'Z': /* Zero WAM */
      const_vn(wv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      SetWAMpos(wv);
      break;
    case 'z': /* Send zero-position to WAM */
      const_vn(wv, 0.0, -1.997, 0.0, +3.14, 0.0, 0.0, 0.0); //gimbals
      //const_vn(wv, 0.0, -2.017, 0.0, 3.14, 0.0, 0.0, 0.0); //blanklink
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
    case 'r': //cartesian move destination entry
        start_entry();
        addstr("Enter destination \"x y z\": ");
        refresh();
        ret = scanw("%lf %lf %lf", &(newpid[0]), &(newpid[1]), &(newpid[2]));
        const_v3(destination,newpid[0],newpid[1],newpid[2]);
        finish_entry();
      break;
    case 'm': //cartesian move start
        CartesianMovePropsWAM(0.5,0.5);
        CartesianMoveWAM((vect_n*)destination);
        break;
    case 'j': //cartesian move start
        CartesianMovePropsWAM(0.5,0.5);
        CartesianMoveWAM((vect_n*)const_v3(destination,0.5,0.5,0.0));
        break;
     case 'J': //cartesian move start
        CartesianMovePropsWAM(0.5,0.5);
        CartesianMoveWAM((vect_n*)const_v3(destination,0.5,-0.5,0.0));
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
    case ';':   /* Play through playlist once */
            DLon(&(wam->log));
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
            DLoff(&(wam->log));
            MLstop(&ml);
            break;
        //case 'l':   /* Load playlist from file */
         /*   start_entry();
            addstr("Enter filename for playlist read: ");
            refresh();
            scanw("%s", fn);
            MLload(&ml, fn);
            finish_entry();
            break;*/
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
    case 'Y': //Start continuos teach
        StartContinuousTeach(1,50,"teachpath");
    break;
    case 'y': //Stop continuos teach
        StopContinuousTeach(); 
        DecodeDL("teachpath","teach.csv",0);
        
    break;
    case 'H':
        //readfile_ct(&ct,"teach.csv");
        //bttrajectory_interface_mapf_ct(&wam->Jsc,&ct);
        vta = read_file_vta("teach.csv",0);
        register_vta(&wam->Jsc,vta);
        break;
        
    case 'U':
        moveparm_bts(&wam->Jsc,0.5,0.5);
        prep_trj_bts(&wam->Jsc);
        while (wam->Jsc.btt.state == BTTRAJ_INPREP){
          usleep(100000);
        }
    
        //wam->Jsc.trj->state = BTTRAJ_READY;
        start_trj_bts(&wam->Jsc);
        
    break;
    case 'u':
        stop_trj_bts(&wam->Jsc);
    break;
    case '~':
            WACKYplay = !WACKYplay;
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
void WACKYeval()
{
    int ret,cnt;
    double pos,vel,acc,min,max;

    if (WACKYplay)
    {
        ret = 0;
        for (cnt = 0; cnt < npucks; cnt++)
        {
            if (wam->sc[cnt].trj.state == 0)
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
                        min = -0.80;
                        max = +2.00;
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
                SCstarttrj(&(wam->sc[cnt]), pos);
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
