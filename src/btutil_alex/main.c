/* ======================================================================== *
 *  Module ............. btutil
 *  File ............... main.c
 *  Creation Date ...... 15 Feb 2003
 *  Author ............. Brian Zenowich
 *  This file edited by Jessica Rucker for use with Cable Tester Rig
 *                                                                        *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file btutil.c

Puck utilities:

Bus enumeration - Prints out what is alive
Bus Enumeration (debug)- Broadcast request; Print CANid,Serial,Status

Bus enumeration and puck status -
  Prints out all interesting puck values.

Puck Find motor offsets

Puck - Load WAM enumeration information.

Puck - Load new firmware

*/

/** Usage:

btutil [-c configfile] command [detail]

where command is:

  enum - List what is on the can bus and what their state is.
  stat - List paramers of interest
    all - dump all parameters
    init - dump parameters that are important to initial startup

  moffst # - Find motor offset for puck id #
  writefirmware # filename - write the specified firmware file
  writewaminfo # - write default wam enumeration info to puck id #
  copyparameters filename - read all parameters and store them to a file
  writeparameters filename - write all parameters stored in a file



*/


//#define toupper(c)      ( ((c >= 'a') && (c <= 'z')) ? c - ('a' - 'A') : c )

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
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/io.h>

//Jessie added this section
#include <stdbool.h>
#include <string.h>
#include <time.h> 
//#include <conio.h>
//#include <iostream>
#include <sys/select.h>
#include <ctype.h>

#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#else
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btos.h"
#include "btmath.h"
#include "btcan.h"
#include "btserial.h"

#include "main.h"

enum{SCREEN_MAIN, SCREEN_HELP};
#define MAX_WATCH (20)
#define SET_SLEEP (1000)

int screen = SCREEN_MAIN;
int entryLine;
int done = FALSE;
PORT p;
int id = 1;
int arrow = 2;
int termX = 0, termY = 20;
int enumX = 39, enumY = 2;
int watchX = 39, watchY = 20;
int curses = FALSE;
int canport = 0;

btrt_thread_struct disp_thd;
btrt_mutex disp_mutex;
int startDone = FALSE;
btrt_thread_struct  StartupThread;

struct watchStruct watch[MAX_WATCH];
struct {int a; char **b;} args;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void RenderMAIN_SCREEN(void);
void ProcessInput(int c);
void DisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void handleMenu(int argc, char **argv);

void mountCable();
void wakePucks(); // wakes pucks and sets global variables

void enumeratePucks(void *data);
void activePuck(void *data);
void firmwarePuck(void *data); int firmwareDL(int id, char *fn);
void terminalMode(void *data);
void watchProperty(void *data);
void setDefaults(void *data); void paramDefaults(int newID,int targID);
void findOffset(void *data);
void exitProgram(void *data);

//declared these functions up here to prevent compiler warnings
int kbhit();
int canReadMsg();
int canClearMsg();
int canSendMsg();

float jVel = 0.5; // max velocity at joints, rad/s
float jAcc = 0.5; // max acceleration at joints, rad/s^2

int     idA = 2;//id of puck on Position Motor (1)
int     idB = 3;//id of puck on Torque Motor (2)

long    encoder = 4096; //4096 encoder counts per rev by default, gets set later
float   g = 10.0;//gearbox input:output ratio
float   d = 19.0;//diameter of pinion (on MB) in mm

int     MAminT = 4000;//didnt seem to matter what the low end was, so made it a large value
int     timestep = 1e4;//PID timestep - 1e3 was too quick sometimes? but 75us should be enough
int     maxErr = 200;//max allowable error on pinion (in terms of encoder counts, so 200/4000 ~5%)
int     torqueCap = 8000; // motors make bad noises at 5000 
int	  torqueMin = 0; // motors must be at least this to spin
int	  runRest = 70; // time in ms between position reversals 
int	  maxVel = 50;
int	  maxAcc = 75;
int	  maxPTurns = 26;
int     minPTurns = 3;
long    tens_torq = 2440;

// PID constants
long kpA = 2000;
long kdA = 8000;
long kiA = 30;


char    expFolder[16] = {"Experiments/"};//at least 12+1
char    dataFolder[16] = {"Data/"};//at least 5+1
char    temperatureFolder[16] = {"Temperature/"};//at least 12+1
char    ext[5] = {".txt"};
int     startupTravs = 20;//takes this number of first data points to null "startup" error
float   kp = 0.1;//tuning constants for PID

bool    pulley_left_of_pinion = true;//if false, directions change
bool    mount_clockwise = true;//if false, directions change

long mBgotTorq;
 


struct fcnStruct fcn[] = { //Jessie added {} around each entry to prevent compiler warnings
   {activePuck, "Change active puck (1-9)"},
   {enumeratePucks, "Enumerate bus"},
   {firmwarePuck, "Download firmware (tek/out)"},
   {terminalMode, "Go to terminal mode"},
   {watchProperty, "Add property to watchlist"},
   {setDefaults, "Set default properties"},
   {exitProgram, "Exit program"},

   {NULL}, {0}//Jessie changed "" to 0 to prevent compiler warning
};

/*==============================*
 * Functions                    *
 *==============================*/

//Jessie changed this to return void instead of int to prevent an error
void setPropertySlow(int bus, int id, int property, int verify, long value){
   setProperty(bus, id, property, verify, value);
	usleep(SET_SLEEP);
	// printf("\npassed ID=%d property=%d value=%ld",id,property,value);
	// long retVal;
	// getProperty(bus,id,property,&retVal);
	// printf("\ngotten ID=%d property=%d value=%ld",id,property,retVal);
   // usleep(SET_SLEEP);
}


////////////////////////////////////////////////////////////////////////////////////////////

//Alex wrote this
void wakePucks() {
	wakePuck(0, idA);
   wakePuck(0, idB);
	getProperty(0,idA,CTS,&encoder);
}

//Jessie wrote this
void saveToFile(char filename[200], char user[200], char dstamp[200], char tstamp[200], float pdiam, float pturns, long travs, long m2torq){

   char dest[64] = "";

   //create file destination
   strcat(dest,expFolder);
   strcat(dest,filename);
   strcat(dest,ext);

   FILE *file;
   file = fopen(dest,"a+");//append file (add text or create file)
	if (file == NULL) {  // ERROR!
		perror("Couldn't open file.\n");
		exit(1);
	}

   //write data to file
   fprintf(file,"%s", "User Name:         ");
   fprintf(file,"%s\n", user);

   fprintf(file,"%s", "Datestamp:         ");
   fprintf(file,"%s\n", dstamp );

   fprintf(file,"%s", "Timestamp:         ");
   fprintf(file,"%s\n", tstamp );

   fprintf(file,"%s", "Pulley Diameter:   ");
   fprintf(file,"%f\n",pdiam);

   fprintf(file,"%s", "Pinion Turns:      ");
   fprintf(file,"%f\n",pturns);

   fprintf(file,"%s", "Traverses:         ");
   fprintf(file,"%ld\n",travs);

   fprintf(file,"%s", "Motor 2 Torque:    ");
   fprintf(file,"%ld\n\n",m2torq);

   fprintf(file,"%s", "Time so far (hrs): ");
   fprintf(file,"%d\n",0);

   fprintf(file,"%s", "Traverses so far:  ");
   fprintf(file,"%d\n",0);

   //close file
   fclose(file);
   printf("\nSaved file.\n");
}


//Jessie wrote this
void newExp(char *filename, char *user, float *pdiam, float *pturns, long *numTravs, long *m2torq){
   char reply[200] ="";

   //get setup info from user
   printf("\nExperiment filename:   ");
   scanf("%s",filename);
   // gets(reply);
   // strcpy(filename, reply);

   printf("          User Name:   ");
   scanf("%s",user);
   // gets(reply);
   // strcpy(user, reply);
   // printf("user=%s\n",user);
	// printf("*user=%f\n",*user);
	// printf("*user=%s\n",*user);

   printf("Pulley Diameter(mm):   ");
   // gets(reply);
   // *pdiam = strtof(reply, NULL);
   scanf("%f",pdiam);
	// printf("*pdiam=%f\n",*pdiam);
	// printf("*pdiam=%f\n",*pdiam);

   *pturns = 0;
   while (*pturns < minPTurns || *pturns > maxPTurns){
      printf("Pinion turns (%d<=turns<=%d):   ", minPTurns, maxPTurns);
      scanf("%f",pturns);
	  //printf("reply=%f\n",reply);
      //*pturns = strtof(reply, NULL);
	  // printf("*pturns=%f\n",*pturns);
	  
      if (*pturns > maxPTurns || *pturns < minPTurns){
         printf("Please enter a valid number of turns\n");
      }
   }
   
   *numTravs = 0;
   do {
      printf("      # of Traverses:  ");
      scanf("%ld",numTravs);
      // gets(reply);
      // *numTravs = strtod(reply, NULL);

      if (*numTravs < 0){
         printf("Please enter a positive number of traverses\n");
      }
   }while ( *numTravs < 0 );
	// printf("*numTravs=%ld\n",*numTravs);

	do {
		printf("Motor B Torque (%d<T<%d):  ", torqueMin, torqueCap);
		scanf("%ld",m2torq);
		// gets(reply);
		// *m2torq = strtod(reply, NULL);
		// printf("*m2torq=%ld\n",*m2torq);
		if (*m2torq < torqueMin || *m2torq > torqueCap) {
			printf("Please enter a torque within the stated limits.\n");
		}
	} while (*m2torq < torqueMin || *m2torq > torqueCap);

}

void loadFile(char dest[200], char *user, char *dstamp, char *tstamp, float *pdiam, float *pturns, long *numTravs, long *m2torq, float *prTime, long *prTravs){
   int column = 19;
   char reply[200] ="";
	char tString[200] = "";
	double tempD;

   FILE *pFile;
   pFile = fopen(dest,"r");//open file for read only

   if (pFile == NULL) return;//maybe print out error??

   //user
   fgets(reply,sizeof(reply)-1,pFile);
   strcpy(user, &reply[column]);
   //datestamp
   fgets(reply,sizeof(reply)-1,pFile);
   strcpy(dstamp, &reply[column]);
   //timestamp
   fgets(reply,sizeof(reply)-1,pFile);
   strcpy(tstamp, &reply[column]);

   //pulley diameter
   fgets(reply,sizeof(reply)-1,pFile);
	// printf("\npdiam string=%s",reply[column]);
   *pdiam = strtod(&reply[column], NULL);
   //pinion turns
   fgets(reply,sizeof(reply)-1,pFile);
	// printf("\npturns string=%s",reply[column]);
   // tString = reply;
	// printf("\npturns string= %s",reply);
	tempD = strtod(&reply[column], NULL);
	strcpy(tString, &reply[column]);
	
	*pturns = strtod(tString, NULL);
	// printf("\npturns string= '%s'",tString);
	// printf("\npturns float= %f",pturns);
	// printf("\npturns double= %f",tempD);
	

   //traverses
   fgets(reply,sizeof(reply)-1,pFile);
	// printf("\nnumTravs string=%s",reply[column]);
	strcpy(tString, &reply[column]);
	tempD = strtod(&reply[column], NULL);
   *numTravs = strtod(&reply[column], NULL);
	// printf("\nnumTravs double= '%f'",tempD);	
	// printf("\nnumTravs native long= '%ld'",numTravs);	
	// printf("\nnumTravs string= '%s'",tString);
   //motor 2 torq
   fgets(reply,sizeof(reply)-1,pFile);
	// printf("\nm2torq string=%s",reply[column]);
   *m2torq = strtod(&reply[column], NULL);

   //data from testing so far
   fgets(reply,sizeof(reply)-1,pFile);//should be empty line

   while(!feof(pFile)){//need to cycle through to find last 3 lines
      //previous # of traverses
      fgets(reply,sizeof(reply)-1,pFile);
      if (sizeof(reply) == 0){ break;}//in case feof fails
   
		// printf("\nprTime string=%s",reply[column]);
      *prTime = strtod(&reply[column], NULL);

      //previous time count
      fgets(reply,sizeof(reply)-1,pFile);
      *prTravs = strtod(&reply[column], NULL);
      fgets(reply,sizeof(reply)-1,pFile);//should be empty line
   }

   fclose(pFile);


}

//Jessie wrote this
void isCableMounted(){
   char ok[10];
   char reply[200];
	
	wakePucks();

   printf("\nIs the cable already mounted [y/n]?    ");
   scanf("%s",ok);
   // gets(reply);
   // strcpy(ok, reply); ./

   if (strcmp(ok,"Y") != 0){
      if (strcmp(ok,"y") != 0){
         mountCable();
      }
   }
}

//Jessie wrote this
void dp_to_torq(int id, long dp, int minT, int maxT, float kp, int timestep){
   long torq;

   if ( fabs(maxT) > torqueCap){//limit on all motors
      maxT = copysign(torqueCap,maxT);
   }

   //calculate torque
   torq = kp*dp;
   long mBgotTorq;
   long mAgotTorq;
	getProperty(0,idB,42,&mBgotTorq);
	getProperty(0,idA,42,&mAgotTorq);
	// printf("\ntorq=%7ld  dp=%7ld  id=%4d  mB=%4d  mA=%4d",torq,dp,id,mBgotTorq,mAgotTorq);
   if (fabs(torq) < minT){
      torq = copysign(minT,torq);
   }
   else if (fabs(torq) > maxT){
      torq = copysign(maxT,torq);
   }

   //send torq command, let timestep pass
   setProperty(0,id,42,FALSE, torq);
   usleep(timestep);//shorter timesteps = more accurate position control
}


//Jessie wrote this
void mountCable(){
   int ch;
   long iposA, cposA, gposA, dpA;
   long iposB, cposB, pposB, gposB, dpB;
   bool problem = false;
   bool mounted = false;
   bool waiting;

   char ok[10];
   char reply[200];

   wakePucks();

   printf("\n\n               ___Cable Mounting Procedure___\n\n");

   //prevent the puck end condition 
   //check Motor A
   getProperty(0,idA,48,&iposA);
   if ( iposA > (2097152 - 100*encoder) || iposA < (-2097152 + 100*encoder) ){
      printf("\nNeed to reset Motor A position");
      printf("\n  Please make sure: 1. safety covers are in place 2. cable is completely off.");
      
      do {
         printf("\nReady to reset [y]?    ");
			scanf("%s",ok);
         // gets(reply);
         // strcpy(ok, reply);
   
         if ( strcmp(ok,"y") == 0 ){
            cposA = iposA;
            gposA = 0;
            dpA = gposA - cposA;
            setPropertySlow(0,idA,8,FALSE,2);
            while ( fabs(dpA) > maxErr*10 ){
               printf("%ld\n",dpA);
               getProperty(0,idA,48,&cposA);
               usleep(2e4);//after getProperty
               dpA = gposA - cposA;
               dp_to_torq(idA, dpA, 2000, 3000, 0.1, timestep);//minT, maxT, kp, timestep
            }
            setPropertySlow(0,idA,42,FALSE,0);
         }

      }while(strcmp(ok,"y") != 0);

   }
   
   //check Motor B
   getProperty(0,idB,48,&iposB);
   if ( iposB > (2097152 - 50*encoder) || iposB < (-2097152 + 50*encoder) ){
      printf("\nNeed to reset Motor B position");
      printf("\n  Please make sure the safety covers are in place and the cable is completely off.");

      do {
         printf("\nReady to reset [y]?    ");
			scanf("%s",ok);
			// gets(reply);
         // strcpy(ok, reply);
   
         if ( strcmp(ok,"y") == 0 ){
            cposB = iposB;
            gposB = 0;
            dpB = gposB - cposB;
            setPropertySlow(0,idB,8,FALSE,2);
            while ( fabs(dpB) > maxErr ){
               printf("%ld\n",dpB);
               getProperty(0,idB,48,&cposB);
               usleep(2e4);//after getProperty
               dpB = gposB - cposB;
               dp_to_torq(idB, dpB, 500, 700, 0.1, timestep);//minT, maxT, kp, timestep
            }
            setPropertySlow(0,idB,42,FALSE,0);
         }

      }while(strcmp(ok,"y") != 0);
   }



   while ( !mounted ){

      //Directions: 1st set
      printf("0. Remove Motor A (MA) safety shield. Lift primary safety shield.\n");
      printf("1. Put cable ball end into Motor B (MB) pinion mount.\n");
      printf("2. Wrap two rotations by hand on MB.\n");
      printf("3. Hold on to the cable loosely. If the cable is frayed use gloves.\n");
      printf("4. Type 's' and hit ENTER to spool\n  -->Pinion will wrap to end.\n");
   
      //wait for s input
      waiting = true;
      while( waiting ){
         ch = getchar();
         if ( ch == 's' ){
            waiting = false;
         }
         while ((ch = getchar()) != '\n' && ch != EOF);//puts("Flushing input");
      }

   
      //set MB to torque mode, get initial pos
      setPropertySlow(0,idB,8,FALSE,2);
      getProperty(0,idB,48,&iposB);
      usleep(2e4);//after getProperty
      cposB = iposB;
   
      //MB to goal position
      gposB = cposB + 33*encoder;
      dpB = gposB - cposB;

     while ( fabs(dpB) > maxErr ){
         getProperty(0,idB,48,&cposB);
         usleep(2e4);//after getProperty
         dpB = gposB - cposB;
         //limit low torq for human interaction - might need to up depending on motor
         //when I changed pucks 200 was a bit hard
         dp_to_torq(idB, dpB, 150, 300, 0.1, timestep);//minT, maxT, kp, timestep
			// getProperty(0,idB,42,&mBgotTorq);
			// printf("\nmotor B torque: %d",mBgotTorq);
      }

      //stop MB from spinning
      setPropertySlow(0,idB,42,FALSE,0);
         
      
      //Directions: 2nd set
      printf("\n5. Put other cable ball end into pulley mount and tape over ball\n");
      printf("   (you might need to pull some length off the pinion/wrap around the pulley,\n    both are OK).\n");
      printf("6. Rotate gearbox input to make cable taut, \n   close primary safety shield and put MA shield down.\n");
      printf("7. Type 's' and hit ENTER to spool to end.\n");
      printf("  --> Type 'l' (CCW) or ';' (CW) to turn the pulley to make it easier to mount.");
      
      //wait for s (s=spool) input, can also do 1/8th turns to make mounting easier
      waiting = true;
      while( waiting ){
         ch = getchar();
         if ( ch == 's' ){
            waiting = false;
         }
         else if (ch == 'l'){//"left turn" = CCW dir
            printf("rotate CCW\n");
            setPropertySlow(0,idA,8,FALSE,2);
            getProperty(0,idA,48,&iposA);
            usleep(2e4);//after getProperty
            cposA = iposA;
            gposA = iposA + encoder*g/8.0;//CCW = + dir
            dpA = gposA - cposA;

            while ( fabs(dpA) > maxErr*g ){
               dp_to_torq(idA, dpA, 100*g, torqueCap, kp, timestep);//1000 for human interaction
               //get positions and recalculate difference
               getProperty(0,idA,48,&cposA);
               usleep(2e4);//after getProperty
               dpA = gposA - cposA;
            }
            setPropertySlow(0,idA,42,FALSE,0);
         }
         else if (ch == ';'){//CW dir
            printf("rotate CW\n");
            setPropertySlow(0,idA,8,FALSE,2);
            getProperty(0,idA,48,&iposA);
            usleep(2e4);//after getProperty
            cposA = iposA;
            gposA = iposA - encoder*g/8.0;//CW = - dir
            dpA = gposA - cposA;

            while ( fabs(dpA) > maxErr*g ){
               dp_to_torq(idA, dpA, 100*g, torqueCap, kp, timestep);//1000 for human interaction
               //get positions and recalculate difference
               getProperty(0,idA,48,&cposA);
               usleep(2e4);//after getProperty
               dpA = gposA - cposA;
            }
            setPropertySlow(0,idA,42,FALSE,0);
         }

         while ((ch = getchar()) != '\n' && ch != EOF);//puts("Flushing input");
      }

      //MA will move MB via cable, so set goal on MB
      getProperty(0,idB,48,&cposB);
      usleep(2e4);//after getProperty
      dpB = cposB - iposB -encoder*4;
   
      //set MA to torque mode, give 600
      setPropertySlow(0, idA, 8, FALSE, 2);
      setPropertySlow(0,idA,42,FALSE,-600);//needs to turn CW (-) to mount cable under
      usleep(1e6);//wait a sec

      //check MB position
      while ( fabs(dpB) > maxErr && !problem){
         usleep(2e4);
         pposB = cposB;
         getProperty(0,idB,48,&cposB);
         dpB = cposB - iposB -encoder*5;
         if (fabs(pposB-cposB) < 1){
            problem = true;
         }
      }

      //immediately turn off MA
      setPropertySlow(0,idA,42,FALSE,0);
      usleep(SET_SLEEP);
      if(problem){
         setPropertySlow(0,idA,8,FALSE,0);
         setPropertySlow(0,idB,8,FALSE,0);
         printf("\nCable fell off, please try again\n\n");
         problem = false;
      }
     else{
         //set MA to hold and MB to tension cable
         setPropertySlow(0, idA, 42, FALSE, 0);
         setPropertySlow(0, idB, 42, FALSE, 200);
         printf("\nCable Mounted.\n");
         mounted = true;
      }
   }
}


//Jessie added this function
void cableTester(float pulleysize, double turns, long travs, long const_torq, char filename[64], float timeSoFar, long travsSoFar) {//timeSoFar in Hours!

   printf("\nEnter cabletester mode\n");
	
	long modeA, modeB; // motor modes, 2 = torque, 5 = trap
	long tA, tB; // reported motor torques
	long vA, mvA, vB, mvB; // velocities and max velocities

   long iposA, gposA, cposA/*, dpA*/;//initial, goal, current, /*d/dt*/ positions on Motor A
   long iposB, cposB, pposB;//initial, current, previous positions on Motor B
   long travel;//turns converted to position encoder distance on Motor A

   float km;//calc'd const - relates motor positions, MA(Pos):MB(Torq)

   long data[startupTravs];//record first # of errors so can record after find error average
	long tempData[startupTravs], dStampData[startupTravs], tStampData[startupTravs]; // temp and timestamp
	float lTimeData[startupTravs], tTimeData[startupTravs]; // loop & total time
   long startupErr = 0;//initial ipos2 error at startup of each set
   long mAerr, mBerr;//encoder error from PID control
   long calcdErr;//error on MB according to error on MA
   long strain;//theoretical strain in terms of encoder cts

   long temperatureB;//temperature of puck on Motor B

   bool printdata = false;//true will print info at end of traverse
   bool exit = false;//true will cause loops to break immediately
   bool pause = false;//true will cause loops to break at end of traverse
   int x = 0;//counter for traverses FOR THIS SET
   time_t start, end, now;//counting time FOR THIS SET, in SECONDS

	long currTime, currDate; // timestamp pieces
   float lTime; // time for this set in hours
	float actualTime;//this set PLUS previous, in HOURS
   long actualTravs;//this set PLUS previous

   char ch;//key stroke interrupts


   //save filename as most previous test
   FILE *previousFile; 
   previousFile = fopen("previous.txt","w+");//will write over previous filename stored
   fprintf(previousFile, "%s", filename );
   fclose(previousFile);

   //data file for writing
   char dataDest[64] = ""; 
   strcat(dataDest,expFolder);
   strcat(dataDest,dataFolder);
   strcat(dataDest,filename);
   strcat(dataDest,"_d");
   strcat(dataDest,ext);
   FILE *dFile;//data file

   //temperature file for writing
   char tempurDest[64] = ""; 
   strcat(tempurDest,expFolder);
   strcat(tempurDest,temperatureFolder);
   strcat(tempurDest,filename);
   strcat(tempurDest,"_t");
   strcat(tempurDest,ext);
   FILE *tFile;//temperature file


   //calculate constants
   km = g*d/pulleysize;//km is positive when cable is attached outside 
   //km is negative when cable is crossed over
   //ex: position motor moves km*5 turns, torque motor moves 5 turns
	// maxVel = km*13.69;
	
	
   wakePucks();

  	// Set initial conditions
	setPropertySlow(0, idA, KP, FALSE, kpA); // kp in PID
	setPropertySlow(0, idA, KD, FALSE, kdA); // kd in PID
	setPropertySlow(0, idA, KI, FALSE, kiA); // ki in PID
	setPropertySlow(0, idA, MT, FALSE, torqueCap); // max torques for both motors
	setPropertySlow(0, idA, HOLD, FALSE, 1);
	setPropertySlow(0,idA,ACCEL,FALSE,maxAcc); // set acceleration
	setPropertySlow(0,idA,MV,FALSE,maxVel); // set max velocity
	setPropertySlow(0,idB,MV,FALSE,maxVel); // set max velocity
  // setPropertySlow(0, idA, 8, FALSE, 2);	
	usleep(2e5);

 	setPropertySlow(0, idA, TSTOP, FALSE, runRest);

	//****DEBUGGING****
	// wait for s input
	// bool waiting = true;
	// while( waiting ){
		// ch = getchar();
		// if ( ch == 's' ){
			// waiting = false;
		// }
		// while ((ch = getchar()) != '\n' && ch != EOF);//puts("Flushing input");
	// }
	//****END DEBUGGING****

	//define goal position on MA
   getProperty(0,idA,48,&iposA);
   usleep(2e4);//after getProperty
   cposA = iposA;
   travel = km*turns*encoder;
   gposA = iposA + travel;

 
   //start traversing
   start = time(NULL);
   while (!exit && !pause){
   
   		if (!(x%10)) { // autotension every 10 runs
				//start MB pulling CW
				setPropertySlow(0, idA, MODE, FALSE, 3); // PID mode to hold position
				setPropertySlow(0, idB, MT, FALSE, tens_torq); // max T is desired T
				setPropertySlow(0, idB, V, FALSE, 10); // low velocity -> low cable impulse
				setPropertySlow(0, idB, MODE, FALSE, 4); // set to constant velocity mode
				printf("\ntensioning...\n");
				usleep(5e5);
				setPropertySlow(0, idB, V, FALSE, 500); // set an unattainable V so T is maintained
				usleep(1e6);
				// setPropertySlow(0, idB, MODE, FALSE, 3); // PID mode to hold position
	
				getProperty(0,idB,48,&iposB);//then define initial position on MB
				usleep(2e4);//after getProperty
				cposB = iposB;
				//define goal position on MA
				getProperty(0,idA,48,&iposA);
				usleep(2e4);//after getProperty
				cposA = iposA;
				travel = km*turns*encoder;
				gposA = iposA + travel;

				if (const_torq<tens_torq) {
					const_torq = const_torq + 100;
				}
				setPropertySlow(0, idB, MT, FALSE, const_torq); // max T is desired T
			
			}
		
      ///0. New loop
      x++;//increase traverses for this set
      actualTravs = travsSoFar + x;
      printf("\nTraverse %ld\n", actualTravs);
      pposB = cposB;//update previous position of MB


      ///1. MA go to Goal Position
      // dpA = gposA - cposA;

      //custom PID control
      // while ( fabs(dpA) > maxErr*g ){
         // dp_to_torq(idA, dpA, MAminT, torqueCap, kp, timestep);

         // //get positions and recalculate difference
         // getProperty(0,idA,48,&cposA);
         // usleep(2e4);//after getProperty
         // dpA = gposA - cposA;
		 // // getProperty(0,idB,42,&mBgotTorq);
		// // printf("\nmotor B torque: %d",mBgotTorq);

      // }

		if (exit) break;
		// new code 2012-01-03
		// setPropertySlow(0, idA, HOLD, FALSE, 0);
		// setPropertySlow(0, idA, TSTOP, FALSE, 500);
		setPropertySlow(0,idA,E,FALSE,gposA); // set goal position
		// setPropertySlow(0,idA,ACCEL,FALSE,500); // get max acceleration
		// setPropertySlow(0,idA,MV,FALSE,1000); // set max velocity
		setPropertySlow(0,idA,8,FALSE,5); // move with trapezoidal move
		
		usleep(2e3);
		getProperty(0,idA,8,&modeA);
		getProperty(0,idA,48,&cposA);
		getProperty(0,idA,T,&tA);
		getProperty(0,idB,T,&tB);
		getProperty(0,idA,V,&vA);
		getProperty(0,idA,MV,&mvA);
		getProperty(0,idB,V,&vB);
		getProperty(0,idB,MV,&mvB);
		getProperty(0,idB,8,&modeB);
		// printf("\nGOAL gposA=%ld cposA=%ld modeA=%ld modeB=%ld tA=%ld tB=%ld vA=%ld mvA=%ld vB=%ld mvB=%ld", gposA, cposA, modeA, modeB, tA, tB, vA, mvA, vB, mvB);

		while ( modeA==5 ) {
			usleep(2e4);
			getProperty(0,idA,8,&modeA);
			getProperty(0,idA,MV,&mvA);
			getProperty(0,idA,V,&vA);
			getProperty(0,idB,V,&vB);
			// printf("\nvA=%6ld vB=%6ld mvA=%6ld",vA,vB,mvA);
			getProperty(0,idA,48,&cposA);
			getProperty(0,idA,T,&tA);
			getProperty(0,idB,T,&tB);
			getProperty(0,idB,MV,&mvB);
			getProperty(0,idB,8,&modeB);
			// printf("\nGOAL gposA=%ld cposA=%ld modeA=%ld modeB=%ld tA=%ld tB=%ld vA=%ld mvA=%ld vB=%ld mvB=%ld", gposA, cposA, modeA, modeB, tA, tB, vA, mvA, vB, mvB);
         //check if user wants to pause experiment or print data
         if ( kbhit() ){            
            while ((ch = getchar()) != '\n' && ch != EOF);//flush buffer
            ch = getchar();
				// printf("--ch GOAL = %c", ch);
            if (ch == 'e'){//e for exit immediately
               exit = true;
					setProperty(0, idA, MODE, FALSE, 0);
					setProperty(0, idB, MODE, FALSE, 0);
               break;
            }
            else if (ch == 'p'){//p for pause
               pause = true;
            }
            else if (ch == 'd'){//d for data
               printdata = true;
            }
         }
		}

		
      ///2. MA go back to Initial Position
      // getProperty(0,idA,48,&cposA);
      // usleep(2e3);//after getProperty
      // dpA = iposA - cposA;

      //custom PID control
      // while ( fabs(dpA) > maxErr*g && !exit ){
         // dp_to_torq(idA, dpA, MAminT, torqueCap, kp, timestep);


         // //get positions and recalculate difference
         // getProperty(0,idA,48,&cposA);
         // usleep(2e4);//after getProperty
         // dpA = iposA - cposA;
      // }

		if (exit) break;
		
		
		
      //get final position for this round and calculate error
      getProperty(0,idA,48,&cposA);
      usleep(2e4);//after getProperty
      mAerr = cposA - iposA;
      calcdErr = mAerr/km;		
		
		// get properties to record
      getProperty(0,idB,48,&cposB);
      mBerr = cposB-iposB;       // get error of MB and compare to calcd error 
      strain = mBerr - calcdErr; // to get strain
      usleep(2e4);//after getProperty
		getProperty(0,idB,9,&temperatureB);
      usleep(SET_SLEEP);
		time_t rawtime;
		struct tm * timeinfo;
		int l = 18;
		char tStamp [l];
		char * tPtr;
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime (tStamp,l,"%Y%m%d %H%M%S",timeinfo);
		currDate = strtol(tStamp,&tPtr,0);
		currTime = strtol(tPtr,NULL,0);
		
		dFile = fopen(dataDest,"a+");//open for appending
		fprintf(dFile,"%1.1f,%1.1f,%ld,%ld,%f,%f,%ld,%ld,%ld,%ld\n", actualTravs-.5, x-.5, currDate, currTime, actualTime, lTime, calcdErr, mBerr, strain, temperatureB);
		fclose(dFile);

		// new code 2012-01-03
		// setPropertySlow(0, idA, HOLD, FALSE, 0);
		// setPropertySlow(0, idA, TSTOP, FALSE, 500);
		setPropertySlow(0,idA,E,FALSE,iposA); // set goal position
		// setPropertySlow(0,idA,ACCEL,FALSE,500); // get max acceleration
		// setPropertySlow(0,idA,MV,FALSE,1000); // set max velocity
		setPropertySlow(0,idA,8,FALSE,5); // move with trapezoidal move
			
		usleep(2e3);
		getProperty(0,idA,8,&modeA);
		getProperty(0,idA,48,&cposA);
		getProperty(0,idA,T,&tA);
		getProperty(0,idB,T,&tB);
		getProperty(0,idA,V,&vA);
		getProperty(0,idA,MV,&mvA);
		getProperty(0,idB,V,&vB);
		getProperty(0,idB,MV,&mvB);
		getProperty(0,idB,8,&modeB);
		// printf("\nINITIAL gposA=%ld cposA=%ld modeA=%ld modeB=%ld tA=%ld tB=%ld vA=%ld mvA=%ld vB=%ld mvB=%ld", gposA, cposA, modeA, modeB, tA, tB, vA, mvA, vB, mvB);

		while ( modeA==5 ) {
			usleep(2e4);
			getProperty(0,idA,8,&modeA);
			getProperty(0,idA,V,&vA);
			getProperty(0,idB,V,&vB);
			// printf("\nvA=%6ld vB=%6ld",vA,vB);
			getProperty(0,idA,48,&cposA);
			getProperty(0,idA,T,&tA);
			getProperty(0,idB,T,&tB);
			getProperty(0,idA,V,&vA);
			getProperty(0,idA,MV,&mvA);
			getProperty(0,idB,V,&vB);
			getProperty(0,idB,MV,&mvB);
			getProperty(0,idB,8,&modeB);
			// printf("\nINITIAL gposA=%ld cposA=%ld modeA=%ld modeB=%ld tA=%ld tB=%ld vA=%ld mvA=%ld vB=%ld mvB=%ld", gposA, cposA, modeA, modeB, tA, tB, vA, mvA, vB, mvB);
         //check if user wants to pause experiment or print data
         if (kbhit()){
				while ((ch = getchar()) != '\n' && ch != EOF);//flush buffer
            ch = getchar();
 				// printf("--ch INITIAL = %c", ch);
           if (ch == 'e'){
               exit = true;
					setProperty(0, idA, MODE, FALSE, 0);
					setProperty(0, idB, MODE, FALSE, 0);
               break;
            }
            else if (ch == 'p'){
               pause = true;
            }
            else if (ch == 'd'){//d for data
               printdata = true;
            }
         }
		}

      ///3. Calculate Error/Strain, Adjust Startup Values
      //stop torque on MA
      // setPropertySlow(0, idA, 42, FALSE, 0);

      //get final position for this round and calculate error
      getProperty(0,idA,48,&cposA);
      usleep(2e4);//after getProperty
      mAerr = cposA - iposA;
      calcdErr = mAerr/km;

		// get properties to record
      getProperty(0,idB,48,&cposB);
      mBerr = cposB-iposB;       // get error of MB and compare to calcd error 
      strain = mBerr - calcdErr; // to get strain
      usleep(2e4);//after getProperty
		getProperty(0,idB,9,&temperatureB);
      usleep(SET_SLEEP);
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime (tStamp,l,"%Y%m%d %H%M%S",timeinfo);
		currDate = strtol(tStamp,&tPtr,0);
		currTime = strtol(tPtr,NULL,0);

		// printf("\ncurrent time: %s",tStamp);
		
		lTime = difftime(rawtime,start)/3600.0;
		actualTime = lTime + timeSoFar;

      // //figure out and adjust startupError
      // if (x <= startupTravs){
         // startupErr = (startupErr*((double)x-1) + strain)/(double)x;
         // printf("startup err avg: %ld\n",startupErr);
			// if (x>startWindow) {
				// int i = startWindow;
				// for (i; i>0; i--) {
					
			// }
         // data[x-1] = strain;
			// tempData[x-1] = temperatureB;
			// dStampData[x-1] = currDate;
			// tStampData[x-1] = currTime;
			// lTimeData[x-1] = lTime;
			// tTimeData[x-1] = actualTime;
			// // printf("\ndata=%ld   temp=%ld   dStamp=%ld   tStamp=%ld   lTime=%f   tTime=%f",data[x-1],tempData[x-1],dStampData[x-1],tStampData[x-1],lTimeData[x-1],tTimeData[x-1]);
			// // printf("\ntStamp string=%s   tStamp long=%ld",tStamp, strtol(tStamp,NULL,0));
      // }
      // if (x == startupTravs){
         // iposB = iposB + startupErr;//modify ipos2 permanently to account for startup error
         // printf("New iposB is: %ld\n",iposB);
         // //record the first startupTravs points - startupErr
         // int a;
			// // printf("\nbreak 1\n");
         // for (a = 0; a < x; a++ ){
				// // printf("\nbreak 2\n");
            // dFile = fopen(dataDest,"a+");//open for appending
				// // printf("\nbreak 3 dataDest=%s\n",dataDest);
            // fprintf(dFile,"%ld,%ld,%ld,%ld,%f,%f,%ld\n", travsSoFar+a+1, a+1, dStampData[a], tStampData[a], tTimeData[a], lTimeData[a], data[a]-startupErr);//record adjusted data points
				// // printf("\nbreak 4\n");
            // fclose(dFile);
				// // printf("\nbreak 5\n");
				// // tFile = fopen(tempurDest,"a+");//open for appending
				// // printf("\nbreak A5\n");
				// // fprintf(tFile,"%ld,%ld,%ld,%ld,%f,%f,%ld\n", travsSoFar+a+1, a+1, dStampData[a], tStampData[a], tTimeData[a], lTimeData[a], tempData[a]);
				// // printf("\nbreak A6\n");
				// // fclose(tFile);
				// // printf("\nbreak A7\n");
        // }
      // }

      ///4. Check if completed this set of testing, record data
      if ( x >= travs && travs != 0){//if travs = 0 want to go until cable breaks
         printf("Finished %dth traverse\n",x);
         exit = true;
      }
      //if the cable broke, the pposB will be over 360deg from cposB and exit wont be triggered yet
      if ( fabs(pposB - cposB) > encoder && !exit){
         printf("Cable Broke!\n");
         exit = true;
      }
      //if not broken and ipos2 adjusted, record data!
      // else if (x > startupTravs){
		
		// printf("\nbreak A1\n");
		dFile = fopen(dataDest,"a+");//open for appending
		// printf("\nbreak A2\n");
		fprintf(dFile,"%ld,%ld,%ld,%ld,%f,%f,%ld,%ld,%ld,%ld\n", actualTravs, x, currDate, currTime, actualTime, lTime, calcdErr, mBerr, strain, temperatureB);
		fclose(dFile);
		// printf("\nbreak A4 tempurDest=%s\n", tempurDest);

      // }
		
		// tFile = fopen(tempurDest,"a+");//open for appending
		// // printf("\nbreak A5\n");
		// fprintf(tFile,"%ld,%ld,%ld,%ld,%f,%f,%ld\n", actualTravs, x, currDate, currTime, actualTime, lTime, temperatureB);
		// // printf("\nbreak A6\n");
		// fclose(tFile);
		// printf("\nbreak A7\n");

      
		//check if user wants to pause experiment or print data
		if (kbhit()){
			while ((ch = getchar()) != '\n' && ch != EOF);//flush buffer
			ch = getchar();
			if (ch == 'e'){
				exit = true;
				setProperty(0, idA, MODE, FALSE, 0);
				setProperty(0, idB, MODE, FALSE, 0);
				break;
			}
			else if (ch == 'p'){
				pause = true;
			}
			else if (ch == 'd'){//d for data
				printdata = true;
			}
		}

      printdata = true;//uncomment if you want it to print out every time
		
      if (printdata){
		
         printf("\nTime this loop so far is %f hours\n", lTime);
         printf("Total time so far is %f\n", actualTime);
         printf("Puck B Temp: %ld\n", temperatureB);
         printf("MB torque: %ld\n", const_torq);
         printf("MA initial position: %ld\n",iposA);
         printf("MA current position: %ld\n",cposA);
         printf("MA offset is:        %ld\n",mAerr);
         printf("Calc'd MB offset: %ld\n",calcdErr);
         printf("MB initial position: %ld\n",iposB);
         printf("MB current position: %ld\n",cposB);
         printf("MB prev. position:   %ld\n",pposB);
         printf("MB offset is:     %ld\n",mBerr);
         printf("\nEncoder 'strain' this loop is %ld\n",strain);
         printdata = false;
      }
		
		// Check that torque hasn't changed because we're approaching over-temp 
		getProperty(0, idB, MT, &tB);
		if (tB != const_torq) {
			printf("\nPaused test because of change in MB torque. Over-temp? MT=%ld\n",tB);
			pause = true;
		}
   }  

   ///5. End Stuff
   end = time(NULL);

   //stop torque on MB
   // setPropertySlow(0, idB, 42, FALSE, 0);
   // usleep(2e5);

   //exit program, set Mode to Idle
   setPropertySlow(0, idB, 8, FALSE, 0);
   setPropertySlow(0, idA, 8, FALSE, 0);
   usleep(2e5);

   printf("The loop used %f hours.\n", difftime(end,start)/3600.0 );
   actualTime = (difftime(end, start))/3600.0 + timeSoFar;
   printf("The total time is %f hours.\n", actualTime);
   if (pause && !exit){
      printf("'Strain' is %ld encoder counts",strain);
   }

   char paramDest[64] = "";
   strcat(paramDest,expFolder);
   strcat(paramDest,filename);
   strcat(paramDest,ext);
   FILE *pFile;//parameters file
   pFile = fopen(paramDest,"a+");//open for appending

   //record timeSoFar, travsSoFar
   fprintf(pFile, "\nTime so far (hrs): %f", actualTime);
   fprintf(pFile, "\nTraverses so far:  %ld\n", actualTravs);

   fclose(pFile);
}

//Jessie borrowed this from an online source
//kbhit determines if there is input from the keyboard in the buffer
int kbhit(void){
  struct timeval tv;
   fd_set read_fd;

   /* Do not wait at all, not even a microsecond */
   tv.tv_sec=0;
   tv.tv_usec=0;

   /* Must be done first to initialize read_fd */
   FD_ZERO(&read_fd);

   /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin */
   FD_SET(0,&read_fd);

   /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
   if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
      return 0; /* An error occured */

   /* read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */

   if(FD_ISSET(0,&read_fd)) {
		// printf("\n----FD_ISSET----\n");
		/* Character pending on stdin */
      return 1;
	}

   /* no characters were pending */
		// printf("FD NOT SET\n");
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void enumeratePucks(void *data){
   int id;
   int x;
   int y;
   long status[MAX_NODES];
   long monVers, mainVers;

   x = enumX; y = enumY;

   clearScreen();
   getBusStatus(0, status);
   usleep(500000);
   mvprintw(y++, x, "PUCK  MON MAIN");
   mvprintw(y++, x, "---- ---- ----");
   for(id = 0; id < MAX_NODES; id++) {
      monVers = mainVers = 0;
      switch(status[id]){
         case -1: // Offline
         break;
         case 0: // Reset
         getProperty(0, id, VERS, &monVers);
         wakePuck(0, id);
         getProperty(0, id, VERS, &mainVers);
         setPropertySlow(0, id, STAT, FALSE, 0); // Reset the puck
         mvprintw(y++, x, "%3d %4ld %4ld", id, monVers, mainVers);
         break;
         case 2: // Ready
         getProperty(0, id, VERS, &mainVers);
         if(id != SAFETY_MODULE){
            setPropertySlow(0, id, STAT, FALSE, 0); // Reset the puck
            usleep(500000);
            getProperty(0, id, VERS, &monVers);
         }
         mvprintw(y++, x, "%3d %4ld %4ld", id, monVers, mainVers);
         break;
         default: // Strange
         break;
      }
   }
}

void activePuck(void *data){
   start_entry();
   addstr("Change active puck to: ");
   refresh();
   scanw("%d\n",  &id);
   finish_entry();
}

void firmwarePuck(void *data){
   char fn[64];

   start_entry();
   addstr("Download firmware file: ");
   refresh();
   scanw("%s\n", fn);
   finish_entry();

   firmwareDL(id, fn);
   usleep(2000000);
   enumeratePucks(NULL);
}

void terminalMode(void *data){
   int prop;
   long val;

   start_entry();
   addstr("Which property: ");
   refresh();
   scanw("%d\n", &prop);
   finish_entry();

   start_entry();
   addstr("What value: ");
   refresh();
   scanw("%ld\n", &val);
   finish_entry();

   setPropertySlow(0, id, prop, FALSE, val);
}

void watchProperty(void *data){
   char prop[64];
   int i;

   start_entry();
   addstr("Which property (or \"clear\"): ");
   refresh();
   scanw("%s\n", prop);
   finish_entry();

   if(!strcmp(prop, "clear")){
      for(i = 0; i < MAX_WATCH; i++){
         watch[i].puckID = 0;
         watchY = 20;
      }
      clearScreen();
      return;
   }

   i = 0;
   while(i < MAX_WATCH){
      if(!watch[i].puckID){
         watch[i].puckID = id;
         watch[i].prop = atol(prop);
         if(i > 3)
            --watchY;
         return;
      }
      ++i;
   }
   return;
}

void setDefaults(void *data){
   int altID;
   char questionTxt[256];

   sprintf(questionTxt, "Set defaults for puck %d to typical defaults for puck [%d]: ", id, id);
   start_entry();
   addstr(questionTxt);
   refresh();
   scanw("%d\n",  &altID);
   finish_entry();

	printf("\n----in setDefaults----\n");
   paramDefaults(id, altID);

   mvprintw(entryLine, 1, "Done setting defaults                                          ");
}

void setMofst(int newID);

void findOffset(void *data){
   setMofst(id);
}

void exitProgram(void *data){
   done = 1;
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

void ProcessWatch(void){
   int i;
   long value;

   canClearMsg(0);
   //clearScreen();
   for(i = 0; i < MAX_WATCH; i++){
      if(watch[i].puckID){
         getProperty(0, watch[i].puckID, watch[i].prop, &value);
         mvprintw(watchY+i, watchX + 2, "ID=%d   PROP=%d   VAL=%ld\t\t",
            watch[i].puckID, watch[i].prop, value);
      }
   }
}

void Startup(void *thd){
   int argc;
   char **argv;
   char            chr;
   int             err;
   //int pID;//Jessie commented out this variable to prevent a compiler warning
   int i;
   long status[MAX_NODES];

   argc = args.a;
   argv = args.b;

   //printf("a=%d, s=%s\n", argc, argv[1]);

   /* Initialize CAN */
   if( (err = initCAN(0, canport)) ) {
      syslog(LOG_ERR, "initCAN returned err=%d", err);
   }

   /* Open serial port */
   if( (err = serialOpen(&p, "/dev/ttyS0")) ) {
      syslog(LOG_ERR, "Error opening serial port: %d", err);
   }
   serialSetBaud(&p, 9600);

   if(argc > 1){
      //getBusStatus(0, status);
      handleMenu(argc, argv);
   } else {
      /* Initialize the ncurses screen library */
      init_ncurses();
      atexit((void*)endwin);
      curses = TRUE;

      /* Initialize the display mutex */
      test_and_log(
         btrt_mutex_init(&(disp_mutex)),
         "Could not initialize mutex for displays.");

      /* Spin off the display thread */
      btrt_thread_create(&disp_thd, "DISP", 10, (void*)DisplayThread, NULL);
      //btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

      for(i = 0; i < MAX_WATCH; i++)
         watch[i].puckID = 0;
      //enumeratePucks(NULL);
      getBusStatus(0, status);

      while (!done) {
         /* Check and handle user keypress */
         if ((chr = getch()) != ERR)
            ProcessInput(chr);
         ProcessWatch();

         usleep(100000); // Sleep for 0.1s
      }
   }

   freeCAN(0);
   startDone = 1;

   btrt_thread_exit((btrt_thread_struct*)thd);
}

void Cleanup(){
   /* Exit the CANbus thread gracefully */
   StartupThread.done = 1;
   exit(0);
}


int main( int argc, char **argv )
{
   args.a = argc;
   args.b = argv;

   mlockall(MCL_CURRENT | MCL_FUTURE);

   if(fopen("port1", "r")){
      canport = 1;
   }else{
      canport = 0;
   }

   /* Initialize syslogd */
   openlog("PUCK", LOG_CONS | LOG_NDELAY, LOG_USER);
   syslog(LOG_ERR, "...Starting Puck Utility Program...");

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, Cleanup);

   /* RT task for setup of CAN Bus */
   btrt_thread_create(&StartupThread,"StTT", 45, (void*)Startup, NULL);
   while(!startDone)
      usleep(10000);
   return 0;
}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
   //int cnt,err;//Jessie commented out these variables to prevent compiler warnings

   clear();
   refresh();
   while (!done) {
      test_and_log(
         btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
      switch(screen) {
      case SCREEN_MAIN:
         RenderMAIN_SCREEN();
         break;
      }
      btrt_mutex_unlock(&(disp_mutex));
      usleep(100000);
   }
}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
   //int err;//Jessie commented out this variable to prevent a compiler warning
   test_and_log(
      btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
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
   btrt_mutex_unlock( &(disp_mutex) );
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderMAIN_SCREEN()
{
   int x, y, i;

   /***** Display the interface text *****/
   y = 0;
   x = 5;

   mvprintw(y, 0, "Barrett Technology - Puck Utility  *** Active puck = %d ***  ", id); y+=2;

   i = 0;
   while(fcn[i].f){
      mvprintw(y++, x, fcn[i].title);
      i++;
   }

   mvprintw(arrow, 1, "-->");

   y++;
   entryLine = y;

   mvprintw(termY-2, termX, " __________");
   mvprintw(termY-1, termX, "| TERMINAL \\____________________________");

   mvprintw(watchY-2, watchX, " __________");
   mvprintw(watchY-1, watchX, "| WATCH    \\____________________________");

   mvprintw(entryLine, 0, "");
   refresh();
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
   //int cnt,elapsed = 0;//Jessie commented out these variables to prevent compiler warnings
   //double ftmp,tacc,tvel;//Jessie commented out these variables to prevent compiler warnings
   //int dtmp,status;//Jessie commented out these variables to prevent compiler warnings

   char chr;//, fn[250];//Jessie commented out these variables to prevent compiler warnings
   //int ret;//Jessie commented out these variables to prevent compiler warnings
   //int done1;//Jessie commented out these variables to prevent compiler warnings
   //btreal zPos;//Jessie commented out these variables to prevent compiler warnings

   switch (c)
   {

   case 'x'://eXit
   case  'X'://eXit
      done = 1;
      break;
   case '1': id = 1; break;
   case '2': id = 2; break;
   case '3': id = 3; break;
   case '4': id = 4; break;
   case '5': id = 5; break;
   case '6': id = 6; break;
   case '7': id = 7; break;
   case '8': id = 8; break;
   case '9': id = 9; break;

   case 10: // <Enter Key>
      (fcn[arrow-2].f)(NULL);
      break;

   case '_'://Refresh display
      clearScreen();
      break;
   case 27://Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR) {
         if (chr == 91) {
            if ((chr = getch()) != ERR) {
               if (chr == 65) // Up arrow
               {
                  mvprintw(arrow, 1, "   ");
                  arrow--;
                  if(arrow < 2)
                     arrow = entryLine - 2;
               }
               else if(chr == 66) // Down arrow
               {
                  mvprintw(arrow, 1, "   ");
                  arrow++;
                  if(arrow > entryLine - 2)
                     arrow = 2;
               }
               else if (chr == 67) //Right arrow
               {

               }
               else if (chr == 68) //Left arrow
               {

               }
               else {
                  while(getch()!=ERR) {
                     // Do nothing
                  }
                  syslog(LOG_ERR,"Caught unknown keyhit 27-91-%d",chr);
                  //mvprintw(20,1,"Caught unknown keyhit 27-91-%d",chr);
               }
            }
         } else {
            while(getch()!=ERR) {
               // Do nothing
            }
            syslog(LOG_ERR,"Caught unknown keyhit 27-%d",chr);
            //mvprintw(20,1,"Caught unknown keyhit 27-%d",chr);
         }
      }
      break;

   default:
      while(getch()!=ERR) {
         // Do nothing
      }
      syslog(LOG_ERR,"Caught unknown keyhit %d",c);

      break;
   }
}

void clearScreen(void)
{
   btrt_mutex_lock(&(disp_mutex));
   clear();
   btrt_mutex_unlock(&(disp_mutex));
}

int mygetch( )
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

int firmwareDL(int id, char *fn)
{
   FILE *fp;
   int in_id;
   int in_len;
   unsigned char in_data[8];
   unsigned char out_Data[8];
   int i;
   int cnt = 1;
   char line[100];
   int rcpt;
   int lineTotal, lineCt;

   // Open the file
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }

   // Count the lines
   lineTotal = 0;
   while(fgets(line, 99, fp) != NULL)
      ++lineTotal;

   // Reset the file pointer
   fclose(fp);
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }

   setPropertySlow(0, id, 5, FALSE, 0L); // Reset
   usleep(1000000); // Wait a sec
   setPropertySlow(0, id, 0, FALSE, 0x000000AA);
   // For each line in the file
   //sendData[0] = 0x80 | VERS;
   //sendData[1] = 0x00;
   lineCt = 0;
   if(!curses) printf("\n\n");
   while(fgets(line, 99, fp) != NULL) {
      i = 1;
      ++lineCt;
      //printf("%s", line);
      if(curses){
      mvprintw(entryLine, 1, "Download progress: %d%%", (int)(100.0 * lineCt / lineTotal));

      }else{
         printf("\rDownload progress: %d%%   ", (int)(100.0 * lineCt /
               lineTotal));
         fflush(stdout);
      }
      while(line[i] >= '0') {
         // Wait for n "Get VERS"
         for(rcpt = 0; rcpt < cnt; rcpt++) {
            while(canReadMsg(0, &in_id, &in_len, in_data, 1))
               usleep(100);
         }
         // Send the byte
         out_Data[0] = line[i];
         canSendMsg(0, id, 1, out_Data, 1);
         ++i;
      }
   }
   fclose(fp);
   if(curses)
   mvprintw(entryLine, 1, "Download complete!               ");
   else
      printf("\nDownload complete!   ");

   return(0);
}

void checkTemp(int puckID){
   long dat;
   int err = 0;

   wakePuck(0, puckID);
   getProperty(0, puckID, TEMP, &dat);
   printf("\nTEMP = %ld", dat);
   if(dat < 15 || dat > 60){
      printf(" -- FAIL");
      err++;
   }

   printf("\nDone. ");
   printf("\n");
}

void paramDefaults(int newID,int targID)
{
	printf("\n----IN paramDefaults----\n");
   int i;
   long vers, role;

   wakePuck(0,newID);
   getProperty(0, newID, VERS, &vers);
   initPropertyDefs(vers);
   getProperty(0, newID, ROLE, &role);

   switch(role & 0x001F){
      case ROLE_TATER:
      case ROLE_BHAND:
      if(targID >= 1 && targID <= 7) { // WAM pucks
          for(i = 0; taterDefs[i].key; i++){
             setPropertySlow(0, newID, *taterDefs[i].key, 0, taterDefs[i].val);
          }
          setPropertySlow(0, newID, MT, 0, wamDefaultMT[targID-1]);
      }
      //if(role & 0x0100)
      //   setPropertySlow(0, newID, CTS, 0, 4096);

      if(targID <= 4) { //4DOF
         setPropertySlow(0,newID,IKCOR,0,1638);
         setPropertySlow(0,newID,IKP,0,8192);
         setPropertySlow(0,newID,IKI,0,3276);
         setPropertySlow(0,newID,IPNM,0,2700); //2755);
         //setPropertySlow(0,newID,IPNM,0,2562);//2755);
         setPropertySlow(0,newID,POLES,0,12);
         setPropertySlow(0,newID,GRPA,0,0);
         setPropertySlow(0,newID,GRPB,0,1);
         setPropertySlow(0,newID,GRPC,0,4);

      setPropertySlow(0,newID,JIDX,0,targID);
         setPropertySlow(0,newID,PIDX,0,targID);
         
     } else if(targID <= 7) { //Wrist
         setPropertySlow(0,newID,IKCOR,0,819);
         setPropertySlow(0,newID,IKP,0,4096);
         setPropertySlow(0,newID,IKI,0,819);
         setPropertySlow(0,newID,GRPA,0,0);
         setPropertySlow(0,newID,GRPB,0,2);
         setPropertySlow(0,newID,GRPC,0,4);
			setPropertySlow(0,newID,JIDX,0,targID);
         if(targID != 7) {
	//	 if(1) { // Temp fix for WK Song
            setPropertySlow(0,newID,IPNM,0,6500);
            //setPropertySlow(0,newID,IPNM,0,4961);
            setPropertySlow(0,newID,POLES,0,8);
         } else {
            setPropertySlow(0,newID,IPNM,0,17474);
            setPropertySlow(0,newID,POLES,0,6);
         }
         
         setPropertySlow(0,newID,PIDX,0,targID-4);
      }

      if(targID >= 11 && targID <= 14){ // BH8-280
          for(i = 0; bh8Defs[i].key; i++){
             setPropertySlow(0, newID, *bh8Defs[i].key, 0, bh8Defs[i].val);
          }
      setPropertySlow(0,newID,JIDX,0,targID-3);
          if(targID == 14) { // Spread on BH8-280
              setPropertySlow(0,newID,CT,0,35950);
              setPropertySlow(0,newID,DP,0,17975);
              setPropertySlow(0,newID,MV,0,50);
              setPropertySlow(0,newID,HSG,0,0);
              setPropertySlow(0,newID,LSG,0,0);
              setPropertySlow(0,newID,HOLD,0,1);
              setPropertySlow(0,newID,TSTOP,0,150);
              setPropertySlow(0,newID,KP,0,1000);
              setPropertySlow(0,newID,KD,0,10000);
            }
            
         setPropertySlow(0,newID,PIDX,0,targID-10);
         
         // set IHIT
         if (vers >= 175) {
         	if (targID == 14) {
	         	setPropertySlow(0,newID,108,0, 2200);  // spread
         	} else {
	         	setPropertySlow(0,newID,108,0, 1700);  // fingers
         	}
         }
      }

      break;

      case ROLE_SAFETY:
      for(i = 0; safetyDefs[i].key; i++){
         setPropertySlow(0, newID, *safetyDefs[i].key, 0, safetyDefs[i].val);

      }

      setPropertySlow(0, newID, SAFE, 0, 4);
      setPropertySlow(0, newID, SAFE, 0, 5);
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, FIND, 0, VBUS);
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, SAFE, 0, 0);

      break;

      case ROLE_WRAPTOR:
      for(i = 0; wraptorDefs[i].key; i++){
         setPropertySlow(0, newID, *wraptorDefs[i].key, 0, wraptorDefs[i].val);

      }

      if(targID < 4){
         // Set inner link parameters

      }else if(targID == 4){
         // Set spread puck parameters: KP=1500 KD=KI=0, ACCEL=10, MV=20, DP=18500, CT=37000, HOLD=1

      }else if(targID > 4){
         // Set outer link parameters

      }
      break;

      default:

      break;

   }

   setPropertySlow(0,newID,SAVE,0,-1); // Save All
}

/* changeID for Puck Monitor vers 5+ */
void changeID(int oldID, int newID, int role)
{
   int _LOCK, _SAVE;

   setPropertySlow(0, oldID, 5, 0, 0); /* RESET back to Monitor */
   usleep(2000000); /* Wait 2s */

   if(oldID == 10){ /* Safety puck's LOCK/SAVE commands are different */
      _LOCK = 13; _SAVE = 30;
   }else{
      _LOCK = 8; _SAVE = 9;
   }
   /* Unlock the ID/ROLE for writing */
   setPropertySlow(0, oldID, _LOCK, 0, 18384);
   setPropertySlow(0, oldID, _LOCK, 0, 23);
   setPropertySlow(0, oldID, _LOCK, 0, 3145);
   setPropertySlow(0, oldID, _LOCK, 0, 1024);
   setPropertySlow(0, oldID, _LOCK, 0, 1);

   /* Set the ROLE */
   if(role >= 0){
      setPropertySlow(0, oldID, 1, 0, role);
      setPropertySlow(0, oldID, _SAVE, 0, 1); usleep(2000000);
   }
   setPropertySlow(0, oldID, 3, 0, newID); /* Set the new ID */
   setPropertySlow(0, oldID, _SAVE, 0, 3); usleep(2000000); /* Save the new values to EEPROM */

   /* Reset to load the new ID */
   setPropertySlow(0, oldID, 5, 0, STATUS_RESET);
}

void setMofst(int newID)
{
   long dat, vers;
   int i, samples = 1024;//dummy - jessie commented out this variable to prevent a compiler warning

   long max, min;
   double sumX, sumX2, mean, stdev;
   int err = 0;

   wakePuck(0,newID);
   getProperty(0,newID,VERS,&vers);

   getProperty(0,newID,IOFST,&dat);
   printf("\nThe old IOFST was: %ld",dat);//Jessie changed %d to %ld to prevent a compiler warning

   // Get a valid IOFST
   #define IOFST_MIN (1800)
   #define IOFST_MAX (2230)
   #define IOFST_STDEV (15.0)

   // Collect stats
   sumX = sumX2 = 0;
   max = -2E9;
   min = +2E9;
   for(i = 0; i < samples; i++){
      setPropertySlow(0,newID,FIND,0,IOFST);
      getProperty(0,newID,IOFST,&dat);
      if(dat > max) max = dat;
      if(dat < min) min = dat;
      sumX += dat;
      sumX2 += dat * dat;
      usleep(1000000/samples);
   }
   mean = 1.0 * sumX / samples;
   stdev = sqrt((1.0 * samples * sumX2 - sumX * sumX) / (samples * samples - samples));
   printf("\nMIN IOFST = %ld", min);
   if(min < IOFST_MIN){
      printf(" -- FAIL");
      ++err;
   }
   printf("\nMAX IOFST = %ld", max);
   if(max > IOFST_MAX){
      printf(" -- FAIL");
      ++err;
   }
   printf("\nMEAN IOFST = %.2f", mean);
   printf("\nSTDEV IOFST = %.2f", stdev);
   if(stdev > IOFST_STDEV){
      printf(" -- FAIL");
      ++err;
   }
   setPropertySlow(0, newID, IOFST, 0, (long)mean);
   printf("\nThe new IOFST is:%d\n",(int)mean);

   if(!err){
      setPropertySlow(0,newID,MODE,0,MODE_TORQUE);
      getProperty(0,newID,MOFST,&dat);
      printf("\nThe old MOFST was:%ld\n",dat);//Jessie changed %d to %ld to prevent a compiler warning

      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32971);
         setPropertySlow(0,newID,VALUE,0,1);
      }else{
         setPropertySlow(0,newID,FIND,0,MOFST);
      }
      //printf("\nPress enter when the index pulse is found: ");
      //mygetch();
      //mygetch();
      printf("\nPlease wait (10 sec)...\n");
      usleep(10000000); // Sleep for 10 sec
      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32970);
         getProperty(0,newID,VALUE,&dat);
      }else{
         getProperty(0,newID,MOFST,&dat);
      }
      printf("\nThe new MOFST is:%ld\n",dat);//Jessie changed %d to %ld to prevent a compiler warning
      setPropertySlow(0,newID,MODE,0,MODE_IDLE);
      if(vers <= 39){
         setPropertySlow(0,newID,MOFST,0,dat);
         setPropertySlow(0,newID,SAVE,0,MOFST);
      }
   }

   printf("\nDone. ");
   printf("\n");
}

//Jessie changed this to return void instead of (nothing) to prevent an error
void getParams(int newID)
{
   long reply;
   //int cnt;//Jessie commented this out to prevent a warning during compile

   printf("\n...Puck %d...\n",newID);
   wakePuck(0,newID);

   /* General */
   getProperty(0,newID,SN,&reply);
   printf("SN = %ld\n",reply);
   getProperty(0,newID,VERS,&reply);
   printf("VERS = %ld\n",reply);
   getProperty(0,newID,ROLE,&reply);
   printf("ROLE = %ld (ROLE = %ld, OPT = 0x%02lX)\n",reply, reply & 0x001F, (reply >> 8) & 0x00FF);
   //from line above: Jessie changed %02X to %02lx and d to ld to prevent a warning
   getProperty(0,newID,JIDX,&reply);
   printf("JIDX = %ld\n",reply);
   getProperty(0,newID,PIDX,&reply);
   printf("PIDX = %ld\n",reply);

   /* Motor control */
   getProperty(0,newID,ACCEL,&reply);
   printf("ACCEL = %ld\n",reply);
   getProperty(0,newID,OT,&reply);
   printf("OT = %ld\n",reply);
   getProperty(0,newID,CT,&reply);
   printf("CT = %ld\n",reply);
   getProperty(0,newID,DP,&reply);
   printf("DP = %ld\n",reply);
   getProperty(0,newID,IPNM,&reply);
   printf("IPNM = %ld\n",reply);
   getProperty(0,newID,MT,&reply);
   printf("MT = %ld\n",reply);
   getProperty(0,newID,KP,&reply);
   printf("KP = %ld (0x%04lX)\n",reply, reply);//Jessie changed %04X to %04lx to prevent a warning
   getProperty(0,newID,KD,&reply);
   printf("KD = %ld (0x%04lX)\n",reply, reply);//Jessie changed %04X to %04lx to prevent a warning
   getProperty(0,newID,KI,&reply);
   printf("KI = %ld (0x%04lX)\n",reply, reply);//Jessie changed %04X to %04lx to prevent a warning
   getProperty(0,newID,TSTOP,&reply);
   printf("TSTOP = %ld\n",reply);
   getProperty(0,newID,HOLD,&reply);
   printf("HOLD = %ld\n",reply);


   /* Sensors */
   getProperty(0,newID,SG,&reply);
   printf("SG = %ld\n",reply);
   getProperty(0,newID,HSG,&reply);
   printf("HSG = %ld\n",reply);
   getProperty(0,newID,LSG,&reply);
   printf("LSG = %ld\n",reply);


   /* Active values */
   getProperty(0,newID,P,&reply);
   printf("P = %ld\n",reply);
   getProperty(0,newID,MECH,&reply);
   printf("MECH = %ld\n",reply);
   getProperty(0,newID,HALLS,&reply);
   printf("HALLS = 0x%04lX\n",reply);
   getProperty(0,newID,HALLH,&reply);
   printf("HALLH = %ld\n",reply);
   getProperty(0,newID,TEMP,&reply);
   printf("TEMP = %ld\n",reply);
   getProperty(0,newID,PTEMP,&reply);
   printf("PTEMP = %ld\n",reply);

   /* Commutation data */
   getProperty(0,newID,CTS,&reply);
   printf("CTS = %ld\n",reply);
   getProperty(0,newID,POLES,&reply);
   printf("POLES = %ld\n",reply);
   getProperty(0,newID,IKP,&reply);
   printf("IKP = %ld\n",reply);
   getProperty(0,newID,IKI,&reply);
   printf("IKI = %ld\n",reply);
   getProperty(0,newID,IKCOR,&reply);
   printf("IKCOR = %ld\n",reply);
   getProperty(0,newID,IOFST,&reply);
   printf("IOFST = %ld\n",reply);
   getProperty(0,newID,MOFST,&reply);
   printf("MOFST = %ld\n",reply);

   /* Communication */
   getProperty(0,newID,EN,&reply);
   printf("EN = 0x%08lX\n",reply);
   getProperty(0,newID,GRPA,&reply);
   printf("GRPA = %ld\n",reply);
   getProperty(0,newID,GRPB,&reply);
   printf("GRPB = %ld\n",reply);
   getProperty(0,newID,GRPC,&reply);
   printf("GRPC = %ld\n",reply);
   //getProperty(0,newID,TIE,&reply);
   //printf("TIE = %ld\n",reply);
}

/* Command enumeration. Append only. */
enum {
    CMD_LOAD, CMD_SAVE,    CMD_RESET, CMD_DEF, CMD_GET, CMD_FIND, CMD_SET, CMD_HOME,
    CMD_KEEP, CMD_LOOP, CMD_PASS, CMD_VERS, CMD_ERR, CMD_HI, CMD_IC, CMD_IO,
    CMD_TC, CMD_TO, CMD_C, CMD_M, CMD_O, CMD_T, CMD_HELP, CMD_END
};



void handleMenu(int argc, char **argv)
{
   long        status[MAX_NODES];
   int         i;
   int         arg1, arg2, arg3;
   long        vers;
   char        *c;
   char        fn[255];

   //Jessie added these
   char        reply[200] ="";
   char        ok[16] = "";

   char        filename[200] = "";
   char        user[200] = "";
   float       pdiam;
   float       pturns;
   long        m2torq;
   long        numTravs;
   long        prevTravs;
   float       prevTime;

   bool        save;
   bool        start;

   time_t      now;
   struct      tm *tm_now;
   char        tstamp[200] ="";
   char        dstamp[200] = "";

   c = argv[1];
   while(*c == '-') c++;
   *c = toupper(*c);

   switch(*c) { /* Only getBusStatus if not BHandDL, PuckDL, PuckID, or Quit */
   case 'B': case 'D': case 'I': case 'Q':
   break;
   default:
      //canClearMsg(0);
      getBusStatus(0, status);
   }

   switch(*c) {

   //Jessie wrote this case statement
   case 'N': //n = NEW experiment

      printf("\n\n        ____New Experiment____\n");

      isCableMounted(); //mount cable first

		// printf("\nbefore newExp: m2torq=%ld",m2torq);
      newExp(filename, user, &pdiam, &pturns, &numTravs, &m2torq);
		// printf("\nafter newExp: m2torq=%ld",m2torq);
      
      //confirm test
		// printf("\nNEW\n");
      printf("\nReady to start [y/n]?    ");
      scanf("%s",ok);
	  // gets(reply);
      // strcpy(ok, reply);
		
		// printf("\nGot reply... ");
		// printf("\nstring ok: %s",ok);
		// printf("\nresult of strcmp 'y': %i",strcmp(ok,"y"));
		// printf("\nresult of strcmp 'Y': %i",strcmp(ok,"Y"));

      if (strcmp(ok,"y") == 0 || strcmp(ok,"Y") == 0) {
         printf("\nLet's get started!");
         save = TRUE;
         start = TRUE;
      }
      else {
         save = FALSE;
         start = FALSE;

         printf("Save experiment file [y/n]?   ");
			scanf("%s",ok);
         // gets(reply);
         // strcpy(ok, reply);
         if (strcmp(ok,"y") == 0 || strcmp(ok,"Y") == 0){
            save = TRUE;
         }
      }

      //create file & start
      if (save) {
         now = time ( NULL );
         tm_now = localtime ( &now );
         strftime ( dstamp, sizeof dstamp, "%m%d%y", tm_now );
         strftime ( tstamp, sizeof tstamp, "%H%M", tm_now );

         //add datestamp to filename
         strcat(filename,"_");
         strcat(filename,dstamp);
         strcat(filename,"-");
         strcat(filename,tstamp);

         saveToFile(filename, user, dstamp, tstamp, pdiam, pturns, numTravs, m2torq);
         if (start){
				// printf("\nin if(start): m2torq=%l",m2torq);
            cableTester(pdiam, pturns, numTravs, m2torq, filename, 0, 0);//timeSoFar and TravsSoFar are 0 for a new test
         }
         else{
            //save filename as most previous test for resuming later
            FILE *previousFile;
            previousFile = fopen("previous.txt","w+");//will write over previous filename stored
            fprintf(previousFile, "%s", filename );
            fclose(previousFile);
            printf("Exited.");
         }
      }
      else{
         printf("Exited.");
      }

      break;

   //Jessie wrote this case statement
   case 'R': //r = RESUME paused experiment

      printf("\n\n      ___Resume Paused Experiment____\n");

      char filename[200];
      char dest[200] = "";

      isCableMounted(); //mount cable first

      //determine previous filename
      FILE *previousFile;
      previousFile = fopen("previous.txt","r");
      fscanf(previousFile, "%s",filename);
      fclose(previousFile);

      printf("\nPrevious test is %s",filename);
      printf("\n  Load previous test [y/n]?  ");
		scanf("%s",ok);
      // gets(reply);
      // strcpy(ok, reply);
      if (strcmp(ok,"Y") != 0){
         if (strcmp(ok,"y") != 0){
            printf("\nEnter filename with datestamp & timestamp (but no extension): ");
				scanf("%s",filename);
            // gets(reply);
            // strcpy(filename, reply);
         }
      }


      //get info from previous file
      strcat(dest,expFolder);
      strcat(dest,filename);
      strcat(dest,ext);

      loadFile(dest, user, dstamp, tstamp, &pdiam, &pturns, &numTravs, &m2torq, &prevTime, &prevTravs);

      printf("\nUser is %s\n", user);
      printf("\nStart date was %s", dstamp);
      printf("\nStart time was %s", tstamp);
      printf("\nPdiam is %f", pdiam);
      printf("\nTurns are %f", pturns);
      printf("\nTraverses are %ld", numTravs);
      printf("\nConstant T is %ld\n", m2torq);
      
      printf("\nPrevious time (hours) total is %f", prevTime);
      printf("\nPrevious traverses are %ld\n", prevTravs);
      usleep(1e6);
		
		printf("\nDo you want to change number of traverses [y/n]?    ");
		scanf("%s",ok);
      if (strcmp(ok,"y") == 0 || strcmp(ok,"Y") == 0){
			do {
				printf("      # of Traverses:  ");
				scanf("%ld",&numTravs);
				// gets(reply);
				// *numTravs = strtod(reply, NULL);

				if (numTravs < 0){
					printf("Please enter a positive number of traverses\n");
				}
			} while ( numTravs < 0 );
		}
		
      //confirm test
      printf("\nReady to start [y/n]?    ");
		scanf("%s",ok);
      // gets(reply);
      // strcpy(ok, reply);

      if (strcmp(ok,"y") == 0 || strcmp(ok,"Y") == 0){
         printf("\nLet's get started!");
         cableTester(pdiam, pturns, numTravs, m2torq, filename, prevTime, prevTravs);
      }
      else {
         printf("\nExited.");
      }

      break;


   //Jessie wrote this case statement
   case 'M': //m = MOUNT cable
      mountCable();
      break;

/////////////////////////////////////////////////////////////////////////////////////////////////
   case 'W':
   printf("\n\nGet PPS data: \n");
   vers = 999;
   getProperty(0, 1, 34, &vers);
   printf("\nResponse = [0x%04lx]\n", vers);//Jessie changed %04x to %04lx to prevent a warning
   break;

   case 'I':
      printf("\n\nChange puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }

      if(argc >= 5){
         arg2 = atol(argv[4]);
         printf("\nNew ID: %d", arg2);
      }else{
         printf("\nNew ID: ");
         scanf("%d", &arg2);
      }

      arg3 = -1;
      if(argc >= 7){
         arg3 = atol(argv[6]);
         printf("\nSet ROLE: %d", arg3);
      }
      changeID(arg1, arg2, arg3);
      break;

   case 'E':
      printf("\n\nCAN bus enumeration (status)\n");
      //getBusStatus(0, status);
      for(i = 0; i < MAX_NODES; i++) {
         if(status[i]>=0) {
            //canClearMsg(0);
            getProperty(0,i,VERS,&vers);
            //Jessie changed 2d to ld to prevent a warning
            printf("\nNode %2d: %15s vers=%ld", i,
                   statusTxt[status[i]+1], vers);
         }
      }
      printf("\n");
      break;
   case 'F':
      printf("\n\nFind offsets for puck: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }

      setMofst(arg1);
      break;
   case 'P':
      printf("\n\nSet defaults for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }

      if(argc >= 5){
         arg2 = atol(argv[4]);
         printf("\nLike ID: %d", arg2);
      }else{
         printf("\nLike ID: ");
         scanf("%d", &arg2);
      }
		printf("\n----in case 'P'----\n");
      paramDefaults(arg1,arg2);
      break;

   case 'G':
      printf("\n\nGet params from puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      getParams(arg1);
      break;
   case 'D':
      printf("\n\nDownload firmware to puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      if(argc >= 5){
         //arg2 = atol(argv[4]);
         printf("\nFile: %s", argv[4]);
         firmwareDL(arg1, argv[4]);
      }else{
         printf("\nFile: ");
         scanf("%s", fn);
         firmwareDL(arg1, fn);
      }
      break;
   case 'S': // Temperature Sensors
      printf("\n\nCheck temperature sensors for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      checkTemp(arg1);
      break;
   case 'Q':
   //   printf("\n\n");
      done = TRUE;
      break;
   default:
      printf("\n\nExample usage:");
      printf("\nbtutil -n             New cabletest");
      printf("\nbtutil -r             Resume cabletest");
      printf("\nbtutil -m             Mount cable, resume cabletest");
      printf("\nbtutil -e             Enumerate bus");
      printf("\nbtutil -h 1           Hall check, motor 1");
      printf("\nbtutil -i 1 -n 2      Change ID 1 to new ID 2");
      printf("\nbtutil -f 1           Find MOFST, motor 1");
      printf("\nbtutil -g 1           Get parameters, motor 1");
      printf("\nbtutil -d 1 -f x.tek  Download x.tek firmware to puck 1");
      printf("\nbtutil -p 5 -l 1      Set puck 5 defaults to puck 1 defaults");
      printf("\nbtutil -t 1           Tension cable, motor 1");
      printf("\nbtutil -a 1           Autotensioner test, motor 1");
      printf("\nbtutil -s 1           Temperature sensor test, motor 1");
      printf("\n");
      break;
   }
   printf("\n\n");
}


int ReadSerial(char *buf, int bytesToRead)
{
   int bytesRead;
   int totalRead = 0;
   long msec = 0;

   /** Read data from the serial port */
   do {
      serialRead(&p, buf, bytesToRead, &bytesRead);
      buf += bytesRead;
      totalRead += bytesRead;
      if(bytesToRead == totalRead)
         break;
      usleep(2000);
      msec += 2;
      if(msec == 1000) {
         printf("ReadSerial timeout!\n");
         return(1);
      }
   } while(1)
      ;

   return(0);
}

//Jessie added an int so it returned a correct type to prevent a warning
int WriteSerial(char *buf, int bytesToWrite)
{
   
   /** Write data to the serial port */
   serialWrite(&p, buf, bytesToWrite);
   return(bytesToWrite);
}

// Read characters, check for errors
char Read(void)
{
   char buf[10];

   //read character
   if( ReadSerial( buf, 1 ) )
      return 0;

   return buf[0];
}


// Write characters, check for errors
int Write( char ch )
{
   char buf[2] = " ";
   buf[0] = ch;
   buf[1] = 0;

   //write character
   return( WriteSerial( buf, 1 ) );

   //return;
}


// Echo write



int EchoWrite(char ch)
{
   char test;
   int err;

   err = Write( ch );
   if (err!=1)
      printf("err:%d",err);
   //fflush(stdout);
   do {
      test = Read();
      if (test != ch)
         printf("\nSent:%2hhX Recvd:%2hhX\n",ch,test);
   } while( ch != test );
   if (test==ch)
      printf("%2hhX",ch);
   fflush(stdout);
   return(err);
}




