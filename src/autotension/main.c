/*======================================================================*
 *  Module .............autotension
 *  File ...............main.c
 *  Author .............Brian Zenowich
 *  Creation Date ......25 Jan 2006
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file main.c
    \brief Autotensioning application for the WAM.
    
    This program allows a user tension the WAM's mechanical cables.
 
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

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
int useGimbals  = FALSE;
int done        = FALSE;

wam_struct      *wam;
btthread        wam_thd;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
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
    vect_n* jdest;
    
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
        if(!strcmp(argv[i], "-g")) // If gimbals are being used
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
    setProperty(0, 10, TL2, FALSE, 5400); //Eliminate torque faults in silly places
    setProperty(0, 10, TL1, FALSE, 1800); //Eliminate torque faults in silly places
    
    /* Spin off the WAM control thread */
    wam_thd.period = 0.002;
    btthread_create(&wam_thd, 90, (void*)WAMControlThread, (void*)wam);

    printf("\n\nPlease activate the WAM by pressing Shift-Activate on the");
    printf("\ncontrol pendant, then press <Enter>");
    mygetch();
    
    // Turn on gravity compensation
    SetGravityComp(wam, 1.1);
    
    const_v3(wam->Cpoint, 0.0, -0.0, 0.05);
    registerWAMcallback(wam, WAMcallback);
    
    // Hold position
    setmode_bts(wam->active_sc,SCMODE_POS);
    
    // Allocate and zero a 7 element vector
    jdest = new_vn(7);
    
    // MoveWAM to zero position
    MoveSetup(wam, 0.5, 0.5)
    MoveWAM(wam, jdest);
    
    // Set low max torque
    
    // MoveWAM to initial tensioning position
    MoveWAM(wam, const_vn(jdest, -3.14, +2.2, -3.14, +3.5, 0.0, 0.0, 0.0));
    while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
    
    // MoveWAM to present position (stop pushing)
    MoveWAM(wam, wam->Jpos);
    while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
    
    // Back off J1 by the tension offset (TENSO) of M1
    
    // Back off J2 by the tension offset of M2
    
    // Back off J4 by the tension offset of M4
    
    // Activate solenoids M1, M2, M4
    setProperty(0, 1, TENSION, FALSE, 1);
    setProperty(0, 2, TENSION, FALSE, 1);
    setProperty(0, 4, TENSION, FALSE, 1);
    
    // Set tensioning max torque
    
    // MoveWAM to apply tension
    MoveWAM(wam, const_vn(jdest, -3.14, +2.2, <no change>, +3.5, 0.0, 0.0, 0.0));
    while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
    
    // Release solenoids M1, M2, M4
    setProperty(0, 1, TENSION, FALSE, 0);
    setProperty(0, 2, TENSION, FALSE, 0);
    setProperty(0, 4, TENSION, FALSE, 0);
    
    // Record tension totals
    
    // MoveWAM to present position (stop pushing)
    MoveWAM(wam, wam->Jpos);
    while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
    
    // Set normal max torque
    
    // Work the tension through
    for(i = 0; i < 8; i++){
        MoveWAM(wam, const_vn(jdest, +2.5, -1.8, +2.5, -0.7, 0.0, 0.0, 0.0));
        while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
        MoveWAM(wam, const_vn(jdest, -2.5, +1.8, -2.5, +3.0, 0.0, 0.0, 0.0));
        while(!MoveIsDone(wam))
            usleep(100000); // Sleep for 0.1s
    }
    
    printf("\n\nExiting...\n\n");
    btthread_stop(&wam_thd); //Kill WAMControlThread
    exit(1);
}

int WAMcallback(struct btwam_struct *wam)
{

    return 0;
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

