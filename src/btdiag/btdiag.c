/*======================================================================*
 *  Module .............btdiag
 *  File ...............btdiag.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......14 Oct 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file btdiag.c
    \brief An interactive demo of wam capabilities.
 
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
#include "bthaptics.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
enum{SCREEN_MAIN, SCREEN_HELP};

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/

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

btstatecontrol *active_bts;

vect_n* active_pos;
vect_n* active_dest;
vect_n* jdest,*cdest;
vect_n* active_trq;
vect_n *wv;
double vel = 0.5,acc = 0.5;

int screen = SCREEN_MAIN;
int prev_mode;
char active_file[250];
char *user_def = "User edited point list";
wam_struct *wam;

vectray *vr;
via_trj_array **vta = NULL,*vt_j = NULL,*vt_c = NULL;
int cteach = 0;
extern int isZeroed;
static RT_TASK *mainTask;
btthread disp_thd,wam_thd;
double sample_rate;
/******* Haptics *******/
btgeom_plane myplane,plane2,planes[10];
bteffect_wall mywall,wall[10];
bteffect_wickedwall mywickedwall,mywickedwall2,wickedwalls[10];
bthaptic_object myobject,myobject2,objects[20];
btgeom_sphere mysphere,mysphere2,spheres[10];
bteffect_bulletproofwall mybpwall,bpwall[10];
btgeom_box boxs[10];
bteffect_global myglobal;
vect_3 *p1,*p2,*p3,*zero_v3;
bthaptic_scene bth;
btgeom_state pstate;
char *command_help[100];
int num_commands;

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
void StartDisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void init_haptics(void);
void read_keys(char *filename);

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
    char robotName[128];
    /* Figure out what the keys do and print it on screen */
    system("grep \"case '\" btdiag.c | sed 's/[[:space:]]*case \\(.*\\)/\\1/' > keys.txt");
    read_keys("keys.txt");
    
    /* Initialize the ncurses screen library */
    init_ncurses();
    atexit((void*)endwin);

    /* Initialize syslog */
    openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
    atexit((void*)closelog);

    /* Initialize the display mutex */
    test_and_log(
        pthread_mutex_init(&(disp_mutex),NULL),
        "Could not initialize mutex for displays.");

    /* Lead the user through a proper WAM startup */
    mvprintw(1,0,"Make sure the all WAM power and signal cables are securely");
    mvprintw(2,0,"fastened, then turn on the main power to WAM and press <Enter>");
    while((chr=getch())==ERR)
        usleep(5000);
    mvprintw(4,0,"Make sure all E-STOPs are released, then press Shift-Idle");
    mvprintw(5,0,"on the control pendant. Then press <Enter>");
    while((chr=getch())==ERR)
        usleep(5000);
    mvprintw(7,0,"Place WAM in its home (folded) position, then press <Enter>");
    while((chr=getch())==ERR)
        usleep(5000);

#ifndef BTOLDCONFIG
    err = ReadSystemFromConfig("wam.conf");
#else //BTOLDCONFIG
#endif //BTOLDCONFIG
    /* If the robot name was given on the command line, use it */
    *robotName = 0; 
    for(i = 1; i < argc-1; i++){
        if(!strcmp(argv[i],"-n"))
            strcpy(robotName, argv[i+1]);
    }
    
    /* Initialize and get a handle to the robot */
    if(!(wam = OpenWAM("wam.conf", robotName)))
        exit(1);

    /* Check and handle any additional command line arguments */
    for(i = 1; i < argc-1; i++){
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
    setSafetyLimits(2.0, 2.0, 2.0);  // ooh dangerous
    setProperty(0,10,TL2,FALSE,8200); //Eliminate torque faults in silly places

    
    /* Prepare MODE */
    jdest = new_vn(len_vn(wam->Jpos));
    cdest = new_vn(len_vn(wam->R6pos));
    
    active_bts = &(wam->Jsc);
    setmode_bts(active_bts,SCMODE_IDLE);
    active_pos = wam->Jpos;
    active_trq = wam->Jtrq;
    active_dest = jdest;
    prev_mode = SCMODE_IDLE;
    
    /* Create a new trajectory */
    vt_j = new_vta(len_vn(active_pos),50);
    vta = &vt_j;
    register_vta(active_bts,*vta);
    
    active_file[0] = 0;

    init_haptics();

    /* Spin off the WAM control thread */
    wam_thd.period = 0.002;
    btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)wam);

    /* Spin off the display thread */
    btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

    while (!done) {
        /* Check the active trajectory for completion */
        if (get_trjstate_bts(active_bts) == BTTRAJ_DONE) {
            stop_trj_bts(active_bts);
            setmode_bts(active_bts,prev_mode);
        }
        
        /* Check and handle user keypress */
        if ((chr = getch()) != ERR)
            ProcessInput(chr);

        usleep(100000); // Sleep for 0.1s
    }

    btthread_stop(&wam_thd); //Kill WAMControlThread

    exit(1);
}
int WAMcallback(struct btwam_struct *wam)
{
    eval_state_btg(&(pstate),wam->Cpos);
    eval_bthaptics(&bth,(vect_n*)wam->Cpos,(vect_n*)pstate.vel,(vect_n*)zero_v3,(vect_n*)wam->Cforce);
    apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce, wam->Ctrq);
    return 0;
}
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
  if (inf != NULL){
    while (!done){
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
void init_haptics(void)
{
    int cnt;
    btreal xorig,yorig,zorig;
    p1 = new_v3();
    p2 = new_v3();
    p3 = new_v3();
    xorig = 0.0;
    yorig = 0.0;
    zorig = 0.10;

    new_bthaptic_scene(&bth,10);
    init_state_btg(&pstate,0.002,30.0);

    init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
    init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1);
    init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0);
    init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1);

    //init_wall(&mywall,0.0,10.0);
    for(cnt = 0;cnt < 6;cnt++) {
        init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
    }
    init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
    init_bx_btg(&boxs[1],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
    init_bx_btg(&boxs[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.5,0.01,zorig+0.0),const_v3(p3,0.5,0.0,zorig+0.01),0.10,0.1,0.1,0);
    init_normal_box_bth(&objects[0],&boxs[1],(void*)&bpwall[0],bulletproofwall_nf);
    init_normal_box_bth(&objects[1],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
    addobject_bth(&bth,&objects[0]);
    addobject_bth(&bth,&objects[1]);
    /*
    for(cnt = 0;cnt < 6;cnt++) {
        init_normal_plane_bth(&objects[cnt],&boxs[0].side[cnt],(void*)&bpwall[0],bulletproofwall_nf);
        //init_normal_plane_bth(&objects[cnt],&boxs[0].side[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
        addobject_bth(&bth,&objects[cnt]);
    }*/

    for(cnt = 0;cnt < 4;cnt++) {
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

    registerWAMcallback(wam,WAMcallback);

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
    while (!done) {
        test_and_log(
            pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
	switch(screen){
	case SCREEN_MAIN:
        	RenderMAIN_SCREEN();
		break;
	case SCREEN_HELP:
		RenderHELP_SCREEN();
		break;
	}
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
    int cnt, idx, Mid, cp;
    int line, line2;
    int cpt, nrows;
    double gimb[4],tacc,tvel;
    vectray* vr;
    char vect_buf1[250];

    /***** Display the interface text *****/
    line = 0;

    mvprintw(line , 0, "Barrett Technology - Diagnostic Application\t\tPress 'h' for help");
    line+=2;

    // Show MODE
    if (active_bts == &(wam->Jsc)) {
        mvprintw(line, 0, "Mode       : Joint Space    ");
    } else if (active_bts == &(wam->Csc)) {
        mvprintw(line, 0, "Mode       : Cartesian Space");
    } else {
        mvprintw(line, 0, "Mode       : Undefined!!!   ");
    }
    ++line;

    // Show CONSTRAINT
    if (getmode_bts(active_bts)==SCMODE_IDLE)
        mvprintw(line, 0, "Constraint : IDLE      ");
    else if (getmode_bts(active_bts)==SCMODE_POS)
        mvprintw(line, 0, "Constraint : POSITION  ");
    else if (getmode_bts(active_bts)==SCMODE_TRJ)
        mvprintw(line, 0, "Constraint : TRAJECTORY");
    else
        mvprintw(line, 0, "Constraint : UNDEFINED!");
    ++line;
 
    if (bth.state) {
        mvprintw(line, 0, "Haptics    : ON    ");
    } else {
        mvprintw(line, 0, "Haptics    : OFF    ");
    }
    line+=2;
    
    // Show TRAJECTORY
    if (cteach)
        mvprintw(line, 0, "Trajectory : Teaching continuous trajectoy");
    else if (*vta == NULL)
        mvprintw(line, 0, "Trajectory : NONE                         ");
    else
        mvprintw(line, 0, "Trajectory : %s                           ",*active_file?active_file:"NONE");
    ++line;
    mvprintw(line, 0, "Velocity   : %+8.4f  ",vel);
    ++line;
    mvprintw(line, 0, "Accel      : %+8.4f  ",acc);
    ++line;
    mvprintw(line, 0, "Destination: %s ",sprint_vn(vect_buf1,active_dest));
    line+=2;
    
    mvprintw(line, 0, "Position   : %s ", sprint_vn(vect_buf1,active_pos));
    ++line;
    mvprintw(line, 0, "Target     : %s ", sprint_vn(vect_buf1,active_bts->qref));
    ++line;
    mvprintw(line, 0, "Force      : %s ", sprint_vn(vect_buf1,active_trq));
    line+=2;
    
    if (*vta != NULL) { // print current point
        vr = get_vr_vta(*vta);
        cpt = get_current_idx_vta(*vta);
        nrows = numrows_vr(vr);
        mvprintw(line,0,"Teach Point: %d of %d      ",cpt,nrows-1);
        line++;

        mvprintw(line  , 0 , "Previous   :\t\t\t\t\t\t\t\t\t\t");
        mvprintw(line+1, 0 , "Current    :\t\t\t\t\t\t\t\t\t\t");
        mvprintw(line+2, 0 , "Next       :\t\t\t\t\t\t\t\t\t\t");

        // Previous
        if (nrows > 0 && cpt > 0)
            mvprintw(line, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));
        
        // Current
        if (nrows > 0) {
            if (nrows != cpt)
                mvprintw(line+1, 13 , "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
            else
                mvprintw(line+1, 13 , "END OF LIST");
        } else
            mvprintw(line+1, 13 , "EMPTY LIST");

        // Next
        if (nrows > 1)
            if (cpt < nrows-1)
                mvprintw(line+2, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
            else if (cpt == nrows-1)
                mvprintw(line+2, 13, "END OF LIST");
        line += 3;
    } else {
        line++;
        line++;
        mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
        line += 2;
    }
  /*  line += 1;
    mvprintw(line,0,"bts: state:%d",active_bts->mode);
    mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
    line += 1;
 */   entryLine = line;
    refresh();
}

void RenderHELP_SCREEN(){
	int cnt, line = 0;

	mvprintw(line, 0, "Help Screen - (press 'h' to toggle)");	
    line += 2;
    for (cnt = 0; cnt < num_commands;cnt++){
      if (cnt % 2){
        mvprintw(line,40,"%.39s",command_help[cnt]);
        line += 1;
      }
      else {
        mvprintw(line,0,"%.39s",command_help[cnt]);

      }
    }
	refresh();
}
void clearScreen(void)
{
    btmutex_lock(&(disp_mutex));
    clear();
    btmutex_unlock(&(disp_mutex));
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
    int cnt,elapsed = 0;
    double ftmp,tacc,tvel;
    int dtmp,status;

    char fn[250],chr;
    int ret;
    int done1;

    switch (c) {
    case 'x'://eXit
    case  'X'://eXit
        done = 1;
        break;

    case 'g'://Set gravity compensation
        start_entry();
        addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
        refresh();
        scanw("%lf\n",  &tvel);
        SetGravityComp(wam,tvel);
        finish_entry();
        break;
    case '_'://Refresh display
        clearScreen();
        break;
    case '\t'://Toggle jointspace and cartesian space
        destroy_vta(vta); //empty out the data if it was full
        setmode_bts(&(wam->Jsc),SCMODE_IDLE);
        setmode_bts(&(wam->Csc),SCMODE_IDLE);

        if (active_bts == &(wam->Jsc)) { //switch to cartesian space mode.
            active_bts = &(wam->Csc);
            active_pos = wam->R6pos;
            active_trq = wam->R6force;
            active_dest = cdest;
            vta = &vt_c;
        } else {

            active_bts = &(wam->Jsc);
            active_pos = wam->Jpos;
            active_trq = wam->Jtrq;
            active_dest = jdest;
            vta = &vt_j;
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
        if (getmode_bts(active_bts)!=SCMODE_IDLE)
            setmode_bts(active_bts,SCMODE_IDLE);
        else
            setmode_bts(active_bts,SCMODE_POS);
        break;

    case '.'://Play loaded trajectory

        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
        moveparm_bts(active_bts,vel,acc);
        active_bts->loop_trj = 0;
        prev_mode = getmode_bts(active_bts);
        if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
        start_trj_bts(active_bts);
        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;

    case 'b'://Simulate loaded trajectory

        sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
        break;

    case '?'://Loop loaded trajectory

        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
        moveparm_bts(active_bts,vel,acc);
        active_bts->loop_trj = 1;
        prev_mode = getmode_bts(active_bts);
        if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
        start_trj_bts(active_bts);
        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;

    case '/'://Stop loaded trajectory

        stop_trj_bts(active_bts);
        setmode_bts(active_bts,prev_mode);
        break;
    case 'Y'://Start continuos teach
        if (active_bts == &(wam->Jsc))
            StartContinuousTeach(wam,1,25,"teachpath");
        else
            StartContinuousTeach(wam,0,25,"teachpath");
        cteach = 1;
        break;
    case 'y'://Stop continuos teach
        StopContinuousTeach(wam);
        DecodeDL("teachpath","teach.csv",0);
        cteach = 0;
        stop_trj_bts(active_bts);
        /** \internal \todo sleeps that are necessary might be forgotten. Can we eliminate the need?*/
        usleep(10000); //needed to give the command a chance to work.
        destroy_vta(vta); //empty out the data if it was full
        strcpy(active_file,"teach.csv");
        *vta = read_file_vta(active_file,20);
        register_vta(active_bts,*vta);
        break;
    case 'l'://Load trajectory from file
        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
            start_entry();
            addstr("Enter filename for trajectory: ");
            refresh();
            scanw("%s", active_file);
            destroy_vta(vta); //empty out the data if it was full

            *vta = read_file_vta(active_file,20);
            register_vta(active_bts,*vta);
            finish_entry();
        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;
    case 'w'://Save trajectory to a file
        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
            start_entry();
            addstr("Enter filename for trajectory: ");
            refresh();
            scanw("%s", fn);
            if (*vta != NULL) {
                write_file_vta(*vta,fn);
                strcpy(active_file,fn);
            }
            finish_entry();
        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;
    case 'n'://Create a new trajectory
        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
            start_entry();
            addstr("Enter the max number of points that will be in your trajectory: ");
            refresh();
            ret = scanw("%d", &dtmp);
            destroy_vta(vta);

            strcpy(active_file,user_def);
            *vta = new_vta(len_vn(active_pos),dtmp);
            register_vta(active_bts,*vta);
            active_file[0] = 0;
            finish_entry();
        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;
    case 'M'://Move to a location
        if(getmode_bts(active_bts)!=SCMODE_TRJ) {
            start_entry();
            addstr("Enter comma seperated destination \".2,.4,...\": ");
            refresh();
            getstr( fn);
            strcat(fn,"\n");
            //syslog(LOG_ERR,"Moveto:%s",fn);
            finish_entry();
            fill_vn(active_dest,0.25);
            csvto_vn(active_dest,fn);
            moveparm_bts(active_bts,vel,acc);
            prev_mode = getmode_bts(active_bts);
            if (prev_mode != SCMODE_POS)
                setmode_bts(active_bts,SCMODE_POS);

            if(moveto_bts(active_bts,active_dest))
                syslog(LOG_ERR,"Moveto Aported");

        } else {
            start_entry();
            addstr("You must stop the running trajectory first!: ");
            refresh();
            sleep(1);
            finish_entry();
        }
        break;
  /*'m': Move to the presently selected trajectory point*/

    case '<'://Select next trajectory point
        prev_point_vta(*vta);
        break;
    case '>'://Select previous trajectory point
        next_point_vta(*vta);
        break;
    case '+'://Insert a point in the trajectory
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            ins_point_vta(*vta,active_pos);
        }
        break;
    case '-'://Remove a point in the trajectory
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            del_point_vta(*vta);
        }
        break;
    case 's'://Adjust trj times using a velocity
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter trajectory velocity: ");
            refresh();
            ret = scanw("%lf\n", &tvel);
            if(*vta != NULL)
                dist_adjust_vta(*vta,tvel);
            finish_entry();
        }
        break;
    case 'S'://Scale trajectory in time
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter scale factor: ");
            refresh();
            ret = scanw("%lf\n", &tvel);
            if(vta != NULL)
                time_scale_vta(*vta,tvel);
            finish_entry();
        }
        break;
    case 'A'://Set the corner acceleration
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter Corner Acceleration: ");
            refresh();
            ret = scanw("%lf\n", &tacc);
            if(*vta != NULL)
                set_acc_vta(*vta,tacc);
            finish_entry();
        }
        break;
    case 'a'://Set the move acceleration
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter Move Acceleration: ");
            refresh();
            ret = scanw("%lf\n", &acc);
            finish_entry();
        }
         break;
    case 'v'://Set the move velocity
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter Move Velocity: ");
            refresh();
            ret = scanw("%lf\n", &vel);
            finish_entry();
        }        
        break;        
    case ','://Pause/Unpause trajectory
      status =  movestatus_bts(active_bts);
        if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
          unpause_trj_bts(active_bts,2);
        else
          pause_trj_bts(active_bts,2); 
      break;
       case 'h'://Toggle Help
        clearScreen();
	screen = !screen;
	break;
 
    case 27://Handle and discard extended keyboard characters (like arrows)
        if ((chr = getch()) != ERR) {
            if (chr == 91) {
                if ((chr = getch()) != ERR) {
                    if (chr == 67) //Right arrow
                    {
                    }
                    else if (chr == 68) //Left arrow
                    {
                    }
                    else {
                        while(getch()!=ERR) {
                            // Do nothing
                        }
                    }
                }
            }
            else {
                while(getch()!=ERR) {
                    // Do nothing
                }
            }
        }
        break;

    default:
        while(getch()!=ERR) {
            // Do nothing
        }
        //syslog(LOG_ERR,"Caught unknown keyhit %d",c);

        break;
    }
}

