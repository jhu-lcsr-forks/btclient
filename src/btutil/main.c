/* main.c */

#define toupper(c)      ( ((c >= 'a') && (c <= 'z')) ? c - ('a' - 'A') : c )
#define HOLE            (0x0000)
#if 0
#define SMOOTH_CT       (5)


#define REVOLUTIONS     (2)
#define STEP            (1)
#define TR_SAMPLE_TIME  (0.005)
#define WAIT_TIME       (0.0025)
#endif

#define TR_GAIN         (0.8)
#define GAIN_WINDOW     (1)
#define pi              (3.14159)

#define TEST_TR         (0)
#define COLLECT_TR      (1)

#include <stdio.h>
#include <syslog.h>
#include <inttypes.h>
#include <malloc.h>
#include <math.h>
#include <semaphore.h>

#include "btcan.h"


//extern  local_info_t    priv_data;
int     REVOLUTIONS_F, REVOLUTIONS_R, SMOOTH_CT;
int     STEP;
double  TR_SAMPLE_TIME;
double  WAIT_TIME;
double  kp, kd, ki;
int     compTorque;





typedef struct{char s[20];}string20;



const    string20	statusTxt[]={
        {"STATUS_OFFLINE"},
    	{"STATUS_RESET"},
        {"STATUS_SHUTDOWN"},
        {"STATUS_INITIALIZE"},
        {"STATUS_ERR"},
        {"STATUS_READY"},
        {"STATUS_COMMUTATING"},
    };

void PuckControlThread(void *data);

void showMenu(){
    printf("\nMENU");
    printf("\n--------");
    printf("\nE)numerate bus status");
    printf("\nF)ind Motor offset");

    
    printf("\n\nYour Choice: ");
}

void handleMenu(char c){
    char        fname[32];
    int         node;
    int         err;
    uint64_t    time1, time2;
    float       delta;
    long        reply;
    long         status[MAX_NODES];
    int         i;
    double      cps;
    int         newID,dummy;
    long dat;
    
    
    switch(c){
    case 'E':
        printf("\n\nCAN bus enumeration (status)\n");
        getBusStatus(0, status);
        for(i = 0; i < MAX_NODES; i++)
        {
            if(i % 2)
                printf("\t\tNode %2d: %s", i, statusTxt[status[i]+1].s);
            else
                printf("\nNode %2d: %s", i, statusTxt[status[i]+1].s);
        }
        printf("\n");
        break;
    case 'F':
        printf("\n\nSet puck MOFST\n");
        printf("\nPuckID: ");
        scanf("%d", &newID);
        setProperty(0,newID,STAT,0,STATUS_READY);
        setProperty(0,newID,MODE,0,MODE_TORQUE);
        
        getProperty(0,newID,MOFST,&dat);
        printf("\n The old MOFST was:%d\n",dat);
        
        setProperty(0,newID,ADDR,0,32971);
        setProperty(0,newID,VALUE,0,1);
        printf("\nPress enter: ");
        scanf("%d", &dummy);
        setProperty(0,newID,ADDR,0,32970);
        getProperty(0,newID,VALUE,&dat);
        printf("\n The MOFST new is:%d\n",dat);
        setProperty(0,newID,MOFST,0,dat);
        setProperty(0,newID,SAVE,0,MOFST);
        printf("\nDone: ");
        printf("\n");
        break;
        
    default:
    
        break;
    }
}

int main( int argc, char **argv )
{
    char            c;
    int             err;
    
    /* Initialize syslogd */
    openlog("LOG_ERR",LOG_CONS | LOG_NDELAY, LOG_USER);
    syslog(LOG_ERR, "syslog initalized");
    
    /* Initialize CAN */
    if(err = initCAN(0))
    {
        syslog(LOG_ERR, "initCAN returned err=%d", err);
    }
    
    /* Show Menu */
    showMenu();
    
    /* Get Choice */
    c = getchar();
    //printf("\nYou pressed ASCII: %d", c);
    c = toupper(c);
    
    /* Handle Menu */
    handleMenu(c);
    
    return(0);
}

