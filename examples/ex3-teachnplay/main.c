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
    int     i;
    struct sched_param mysched;
    int     busCount;
    
    wv = new_vn(7);
    
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

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   err = ReadSystemFromConfig("wam.conf", &busCount);

   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned error: %d", err);
      exit(1);
   }

   /* Initialize and get a handle to the robot */
   if(!(wam = OpenWAM("wam.conf", 0)))
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
    setSafetyLimits(0, 1.5, 1.5, 1.5);  // bus, joint rad/s, elbow m/s, arm m/s

    // const_vn(wv, 0.0, -1.997, 0.0, +3.14, 0.0, 0.0, 0.0); //Blank link home pos
    // DefineWAMpos(wam, wv);
    
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

    /* Spin off the WAM control thread */
    wam_thd.period = 0.002;
    btthread_create(&wam_thd,90,(void*)WAMControlThread1,(void*)wam);

    /* Spin off the display thread */
    btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

    while(!done) {
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
    CloseWAM(wam);
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
    while (!done) {
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
    int cnt, idx, Mid, cp;
    int line, line2;
    int cpt, nrows;
    double gimb[4];
    vectray* vr;
    char vect_buf1[250];

    /***** Display the interface text *****/
    line = 0;

    mvprintw(line , 0, "Barrett Technology - Teach & Play");
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
    line += 3;

    mvprintw(line,0,"bts: state:%d",active_bts->mode);
    mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
    entryLine = line + 2;
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
    case 'x':  /* eXit */
    case 'X':  /* eXit */
        done = 1;
        break;
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
    case 'p':  /* Turn on/off Constraint */
        if (getmode_bts(active_bts)!=SCMODE_IDLE)
            setmode_bts(active_bts,SCMODE_IDLE);
        else
            setmode_bts(active_bts,SCMODE_POS);
        break;
    case '.':  /* Play presently loaded trajectory */
        moveparm_bts(active_bts,vel,acc);
        active_bts->loop_trj = 0;
        prev_mode = getmode_bts(active_bts);
        if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
        start_trj_bts(active_bts);
        break;
    case 'b':  /* Simulate presently loaded trajectory */
        sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
        break;
    case '?':  /* Play presently loaded trajectory */
        moveparm_bts(active_bts,vel,acc);
        active_bts->loop_trj = 1;
        prev_mode = getmode_bts(active_bts);
        if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
        start_trj_bts(active_bts);
        break;
    case '/':  /* Stop presently loaded trajectory */
        stop_trj_bts(active_bts);
        setmode_bts(active_bts,prev_mode);
        break;
    case 'Y':  /* Start continuos teach */
        if (active_bts == &(wam->Jsc))
            StartContinuousTeach(wam,1,25,"teachpath");
        else
            StartContinuousTeach(wam,0,25,"teachpath");
        cteach = 1;
        break;
    case 'y': /*Stop continuos teach */
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
    case 'l':  /* Load trajectory file */
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
    case 'w':  /*  Save trajectory to a file */
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
    case 'n': /* Create a new trajectory*/
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
    case 'M':  /* Move to a location */
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
    case 'm':  /* Move to the presently selected trajectory point*/
        break;
    case '<':  /* Select next point in the trajectory */
        prev_point_vta(*vta);
        break;
    case '>':  /* Select previous point in the trajectory */
        next_point_vta(*vta);
        break;
    case '+':  /* Insert a point in the trajectory */
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            ins_point_vta(*vta,active_pos);
        }
        break;
    case '-':  /* Remove a point in the trajectory */
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            del_point_vta(*vta);
        }
        break;
    case 's':  /* Set point times by defining a velocity */
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter trajectory velocity: ");
            refresh();
            ret = scanw("%lf\n", &vel);
            if(*vta != NULL)
                dist_adjust_vta(*vta,vel);
            finish_entry();
        }
        break;
    case 'S':  /* Scale the present trajectory in time*/
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter scale factor: ");
            refresh();
            ret = scanw("%lf\n", &vel);
            if(vta != NULL)
                time_scale_vta(*vta,vel);
            finish_entry();
        }
        break;
    case 'A':  /* Set the corner acceleration */
        if(getmode_bts(active_bts)==SCMODE_IDLE) {
            start_entry();
            addstr("Enter Corner Acceleration: ");
            refresh();
            ret = scanw("%lf\n", &acc);
            if(*vta != NULL)
                set_acc_vta(*vta,acc);
            finish_entry();
        }
        break;
    case ',': /* if PAUSING or PAUSED : Unpause*/
      status =  movestatus_bts(active_bts);
        if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
          unpause_trj_bts(active_bts,2);
        else
          pause_trj_bts(active_bts,2); 
      break;
        
    case 27: //Handle and discard extended keyboard characters (like arrows)
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

