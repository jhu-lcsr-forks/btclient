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
    if(!(wam = OpenWAM("wam.conf", robotName))){
        printf("\n\nThere was an error while initializing the robot.");
        printf("\nPlease check /var/log/syslog for details.\n\n");
        exit(1);
    }

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
    setProperty(0,10,TL2,FALSE,5400); //Eliminate torque faults in silly places
    setProperty(0,10,TL1,FALSE,1800); //Eliminate torque faults in silly places
    
    // Initialize a haptic scene
    init_haptics();

    /* Spin off the WAM control thread */
    wam_thd.period = 0.002;
    btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)wam);

    printf("\n\nPlease activate the WAM by pressing Shift-Activate on the");
    printf("\ncontrol pendant, then press <Enter>");
    mygetch();
    
    // Turn on gravity compensation
    SetGravityComp(wam, 1.1);
    
    // Turn on haptic scene
    bth.state = TRUE;
    
    printf("\n\nRunning -- Press Ctrl-C to quit\n");
    while (!done) {
        printf("\rForce (Nm): %s ", sprint_v3(vect_buf1,wam->Cforce));
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
    eval_bthaptics(&bth,(vect_n*)wam->Cpos,(vect_n*)pstate.vel,(vect_n*)zero_v3,(vect_n*)wam->Cforce);
    apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce, wam->Ctrq);
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
    init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
    init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
    init_normal_box_bth(&objects[objectCount],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
    addobject_bth(&bth,&objects[objectCount++]);
    
    // Create nested spheres
    init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
    init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1);
    init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0);
    init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1);
    for(cnt = 0;cnt < 4;cnt++) {
        init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
        init_normal_sphere_bth(&objects[objectCount],&spheres[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
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

