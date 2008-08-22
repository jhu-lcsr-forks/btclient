/*======================================================================*
 *  Module .............WAM autotension
 *  File ...............btTension.c
 *  Author .............Kevin Doo
 *  Creation Date ......12 Aug 2008
 *                                                                      *
 *======================================================================*/

#include "btTension.h"
#include "btcontrol.h"
#include <stdio.h>
#include <time.h>
#include <math.h>

/** Public Function Prototypes */

void initialize(wam_struct* wam, FILE * logFile); /* creates vectors and variables necessary
												 to use other functions in btTension */
int menu();															 /* menu for autotension program */
int getDOF(); 													    /* gets the degrees of freedom */
void setToHomePosition(); 								       /* sets WAM to home position */
void promptNextStep();         							    /* Prompts for next step */

/** Private Function Prototypes */

/* Prompts user for the DOF of the WAM */
void prepareMotor(int motorID, int direction); /* prepares motor for tensioning (all motors)*/
void engageMotor(int motorID, int direction);  /* engages and tensions motor (motors 1-4)   */
void engageWrist(int motorID, int direction);  /* engages and tensions motor (wrist motor)  */
void oscillateMotor(int motorID);     			  /* works tension through cable (all motors)  */
int promptDOF();										  /* find + set degrees of freedom on the WAM  */
void findTangStop(int motorID, int direction); /* helper function to engage tang on WAM     */
void findJointStop(int motorID, int direction);/* helper function to find joint stop on WAM */
vect_n* getJointPosition();                    /* gets all joint angles, returns a vect_n   */
vect_n* getMotorPosition();                    /* gets all motor angles, returns a vect_n   */
void setToNeutralPosition();                   /* sets WAM straight up with J4 folded in     
  									                       (allows easy transition to any position)  */
void setToPosition(vect_n* Pos);   /* sets a WAM to a given vector position 
                                      and waits until the WAM is has reached the position   */
void getTorques();                 /* reads and saves motor torques to currentTorque[6]     */

// if the difference in the average of the encoder counts is less than EPSILON, then 
// tensioning is sufficient and will stop (autotensioning will run a minimum of 2 times)
#define AUTOTENSION_EPSILON (long)(.005 * ((float) ctsPerRev)) 

/** Global Variables (extern variables in main.c) */
int findingJointStop = 0; // 0 = false, 1 = true
int tensioning = 0;       // 0 = false, 1 = true
int findingTang = 0;		  // 0 = false, 1 = true
int hasTool = 0;          // 0 = false, 1 = true
int currentDirection = 0; //  1  = Positive Direction
                          //  0  = No Direction
								  // -1  = Negative Direction		
int autotensioning;   	  //  0 = false, 1 = true

int currentJoint = 0;     //  Joints are from 0 - 6
float currentTorque[6] = {0.0,0.0,0.0,0.0,0.0,0.0};  // motor torques at a given instant
float radianInc = 0.02;                              // radian increment for finding the joint stop
float numerator;        // 0 to 100%
float denominator;

int DOF;                  //Degrees of Freedom

wam_struct* wam;
FILE * logFile;

vect_n* home;                					    //positions for WAM
vect_n* neutral;          
vect_n* jointStop[6][2];      				 	 //actual joint stop positions
vect_n* jointStopApprox[6][2];					 //approximated joint stop positions
long lastTensionedMotorValues[4] = {0,0,0,0}; //save tensioning values
long ctsPerRev;
 
/** Functions */

/* initialize:
	input: wam_struct* wam_p
	output: none
	function: initializes btTension for autotensioning */
void initialize(wam_struct* wam_p, FILE * logFile_p){
	
	logFile = logFile_p;
	wam = wam_p;
	
	//get the DOF for the WAM
	DOF = promptDOF();
	//set the maximum velocity and acceleration for the WAM
 	MoveSetup(wam,1.0,0.3);         
	//set the home position for the WAM
	home = getJointPosition();
	neutral = new_vn(7);
	const_vn(neutral, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0);

	int i;
	//allocate memory for all the various joint positions
	for(i = 0; i < 6; i++) {
		if(i < DOF){
			//make sure tangs are deactivated
			setProperty(0,i+1,TENSION,FALSE,0);
			usleep(2000);
		}
		jointStopApprox[i][0] = new_vn(7);
		jointStopApprox[i][1] = new_vn(7);
   }
	
	/** Joint Ranges (radians)
		   J1    2.6    -2.6
		   J2    2.0    -2.0
		   J3    2.8    -2.8
		   J4    3.1    -0.9
			J5    1.3    -4.8
			J6    1.6    -1.6
			J7    3.0    -3.0     */

	//set all values for approximate joint stop positions 
/*                                  J1 	 J2   J3		J4	  J5    J6   J7 */
	const_vn(jointStopApprox[0][0], -2.5,  0.0, 0.0,  3.0, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[0][1],  2.5,  0.0, 0.0,  3.0, 0.0,-1.52, 0.0);	
	const_vn(jointStopApprox[1][0],  0.0, 1.80,-2.6,  2.0, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[1][1],  0.0,-1.80, 2.6, -0.8, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[2][0],  0.0,-1.80,-2.6, -0.8, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[2][1],  0.0, 1.80, 2.6,  2.0, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[3][0], 0.00,-1.20,0.00, -0.8, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[3][1], 0.00,-1.20,0.00,  3.0, 0.0,-1.52, 0.0);
	const_vn(jointStopApprox[4][0], 0.00,-1.80,0.00,  2.0, 0.0,  0.0, 0.0);
	const_vn(jointStopApprox[4][1], 0.00,-1.80,0.00,  2.0, 0.0,  0.0, 0.0);
	const_vn(jointStopApprox[5][0], 0.00,-1.80,0.00,  2.0, 0.0,  0.0, 0.0);
	const_vn(jointStopApprox[5][1], 0.00,-1.80,0.00,  2.0, 0.0,  0.0, 0.0);
	
	//deactivate and activate wam PID control to reset PID and torques of motor
	//(needed to change the gains of the joints)
	setmode_bts(wam->active_sc,SCMODE_IDLE);

	double kp,kd,ki;
	getgains_btPID(&(((btPID_array)wam->JposControl).pid[1]), &kp, &kd, &ki);
	
	//printf("Gains before: %f %f %f\n", kp, kd, ki);
	
	setgains_btPID(&(((btPID_array)wam->JposControl).pid[1]), 2500.0, 10.0, 0.0);
	setgains_btPID(&(((btPID_array)wam->JposControl).pid[3]), 500.0, 1.5, 0.0);
	
	
	
	getgains_btPID(&(((btPID_array)wam->JposControl).pid[1]), &kp, &kd, &ki);
	
	//printf("Gains after: %f %f %f\n", kp, kd, ki);
	//promptNextStep();
	
	if(DOF == 6){
		setgains_btPID(&(((btPID_array)wam->JposControl).pid[4]), 50.0, 0.5, 0.0);
		setgains_btPID(&(((btPID_array)wam->JposControl).pid[5]), 50.0, 0.5, 0.0);
		setgains_btPID(&(((btPID_array)wam->JposControl).pid[6]), 1.0, 0.0, 0.0);
	}
	
	//reacitvate wam PID control
	setmode_bts(wam->active_sc,SCMODE_POS);
}

/* promptDOF:
	input: none
	output: integer
   function: prompts user for DOF (degrees of freedom) to be tensioned*/
int promptDOF(){
	
	int choice = -1, choice2 = -1;
	
	//ask for input until 4 or 7 is entered for the DOF
	while(choice != 4 && choice != 7){
		printf("\nEnter the degrees of freedom (DOF) for the WAM (4 or 7): ");
		scanf("%d", &choice);
		 
		if(choice != 4 && choice != 7){
			printf("\nInvalid number of DOF.");
			while ((choice = getchar()) != '\n' && choice != EOF);
		}
	}
	
	//ask for input until 0 or 1 is entered for hasTool
	while(choice2 != 0 && choice2 != 1){
		printf("\nDo you currently have a tool equiped? (1 = Yes; 0 = No): ");
		scanf("%d", &choice2);
		 
		if(choice2 != 0 && choice2 != 1){
			printf("\nInvalid choice.");
			while ((choice2 = getchar()) != '\n' && choice2 != EOF);
		}
	}
	
	hasTool = choice2;

	if (choice == 7) { 
		return 6; 
	} else {
		return 4;
	}
}

/* menu:
	input: none
	output: 1 if autotension is to continue
	        0 if autotension is to quit
	functions: autotensions the selected motors */
int menu(){
	
	int choice = -1;
	
	//give user a selection of tensioning options
	while(choice < 0 || choice > 10){
		printf("\nEnter a selection: ");
		printf("\n1) Tension Motor 1: ");
		printf("\n2) Tension Motor 2: ");
		printf("\n3) Tension Motor 3: ");
		printf("\n4) Tension Motor 4: ");
		printf("\n5) Tension Motor 5 (if wrist is attached): ");
		printf("\n6) Tension Motor 6 (if wrist is attached): ");
		printf("\n7) Tension Motors 2 and 3: ");
		printf("\n8) Tension Motors 5 and 6 (if wrist is attached): ");
		printf("\n9) Tension Motors 1-4 (4 DOF motors): ");
		printf("\n10) Tension Motors 1-6 (all motors): ");
		
		printf("\n\n0) Quit Autotension Procedure: \n");
		
		printf("Enter a selection: ");
		
		scanf("%d", &choice);
		 
		if(choice < 0 || choice > 10){
			printf("\nInvalid selection.");
			while ((choice = getchar()) != '\n' && choice != EOF);
		}
	}

	int start = 0, end = -1;
	//determine start and end for motors to tension
	switch(choice){
		case 1: case 2: case 3: case 4: case 5: case 6:	
					start = choice; end = choice;
				   break;
		case 7:	start = 2; end = 3;
					break;
		case 8: 	start = 5; end = 6;
					break;
		case 9: 	start = 1; end = 4;
					break;
		case 10:	start = 1; end = 6;
					break;	
		case 0: 
		default: return 0;
					break;
	}
	
	if (start > DOF || end > DOF){
		printf("\nInvalid selection; Please attach wrist if you wish to tension motors 5 and 6.");
	}
	
	int i, j, k;
	
	printf("\nAbout to autotension from motor(s) %d to %d", start, end);
	printf("\nHit enter when ready...");
	promptNextStep();
	
	numerator = 0.0;
	denominator = fabs(end-start + 1.0) * 2.0;
	autotensioning = 1;
	//percentDone = 0.0;
	time_t timer = time(NULL);
	char * currentTime = ctime(&timer);
	
	fprintf(logFile, "------------------------------------------------------\n");
	fprintf(logFile, "Autotension Data Log - %s\n", currentTime);
	
	//autension from start to end
	for(i = start; i <= end; i++) { 
		
		int tensioned = 0;
		long tensionSum = 0;
		long previousSum = -1;
		
		prepareMotor(i, 1);
		
		for (k = 0; k < 5 && !tensioned; k++) {
			switch(i){		
				case 1: case 2: case 3: case 4:
					engageMotor(i, 1);
					break;
				case 5: case 6:
					engageWrist(i,1);
					break;
				default: break;
			}		
			oscillateMotor(i);
			
			printf("\rEncoder counts taken up for motor %d:                                       \n", i);
			fprintf(logFile, "Encoder counts taken up for motor %d:         \n", i);
			
			for(j = 0; j < 3; j++){
				tensionSum += abs(lastTensionedMotorValues[j]);
				printf("\nAt motor revolution %d: %ld encoder counts", (j+1), lastTensionedMotorValues[j]);
				fprintf(logFile, "\nAt motor revolution %d: %ld encoder counts", (j+1), lastTensionedMotorValues[j]);
			}
			
			long diff = abs(tensionSum - previousSum);
			
			printf("\nEPSILON: %ld DIFF: %ld\n", AUTOTENSION_EPSILON, diff);
			fprintf(logFile, "\n\n");
			
			if(previousSum != -1 && diff < AUTOTENSION_EPSILON){
				tensioned = 1;
			}
			
			numerator += 1.0;
			if(k >= 2){
				denominator += 1.0;
			}
			
			previousSum = tensionSum;
			tensionSum = 0;
		}
	}
	
	autotensioning = 0;
	
	printf("\n\n");
	fprintf(logFile, "------------------------------------------------------\n");
	return 1;
}

/*	sum_vn:
	input: vect_n* and vect_n* b
	output: vect_n*
	function: sums the two vectors and returns their sum as a new vector */
vect_n* sum_vn(vect_n* a, vect_n* b){
	
	vect_n* toReturn = new_vn(7);
	float a_values[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float b_values[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	int i;
	
	for(i = 0 ; i < DOF; i++){
		a_values[i] = a->q[i];
		b_values[i] = b->q[i];
		
		a_values[i] += b_values[i];
	}
	
	const_vn(toReturn, a_values[0], a_values[1], a_values[2],
							 a_values[3], a_values[4], a_values[5],
							 a_values[6]);
	
	return toReturn;
}

/* getJointPositon:
	input: none
	output: vect_n*
   function: Gets position of WAM in jointspace (in radians) and returns it as a vector*/
vect_n* getJointPosition(){
	int i;
	vect_n* toReturn = new_vn(7);
	float positions[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	for(i = 0; i < DOF; i++){
		positions[i] = ((vect_n*)wam->Jpos)->q[i];
		//printf("\njoint positions: positions[%d] is %f", i, positions[i]);
	}
	
	const_vn(toReturn, positions[0], positions[1], positions[2],
							 positions[3], positions[4], positions[5],
							 positions[6]);
	return toReturn;
}

/* getMotorPosition:
	input: none
	output: vect_n*
	function: Gets position of the individual motors (in radians) and returns them as a vector */
vect_n* getMotorPosition(){
	int i;
	vect_n* toReturn = new_vn(7);
	float positions[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	for(i = 0; i < DOF; i++){
		positions[i] = ((vect_n*)wam->Mpos)->q[i];
		//printf("\nmotor positions: positions[%d] is %f", i, positions[i]);
	}
	
	const_vn(toReturn, positions[0], positions[1], positions[2],
							 positions[3], positions[4], positions[5],
							 positions[6]);
	return toReturn;
}

/* motorToJointPosition:
	input: vect_n* motorPos (motor position)
	output: vect_n*
	function: takes a vect_n of motorPositions and uses and transforms it into Jointspace,
	          returning the result as a vector */
vect_n* motorToJointPosition(vect_n* motorPos){
	int i;
	
	//constant values
	float N1 = 42.0, N2 = 28.25, N3 = 28.25, n3 = 1.68, N4 = 18.0;
	float N5 = 9.70, N6 = 9.70, N7 = 14.93, n6 = 1.0;
	
	float JointPositions[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	// calculating the transformation from motors 1-7 to joints 1-7
	JointPositions[0] = (motorPos->q[0] * -1.0)/N1;
	JointPositions[1] = (motorPos->q[1])/(2.0 * N2) - (motorPos->q[2])/(2.0 * N2); 
	JointPositions[2] = (motorPos->q[1] * -1.0 * n3)/(2.0 * N2) + (motorPos->q[2] * -1.0 * n3)/(2.0 * N2);
	JointPositions[3] = (motorPos->q[3] * -1.0)/N4;
	
	if(DOF == 6){
		JointPositions[4] = (motorPos->q[4])/(2.0 * N5) + (motorPos->q[5])/(2.0 * N5);
		JointPositions[5] = (motorPos->q[4] * -1.0 * n6)/(2.0 * N5) + (motorPos->q[5] * n6)/(2.0 * N5);
		JointPositions[6] = (motorPos->q[6] * -1.0)/N7;
	}
	
	vect_n* toReturn = new_vn(7);
	
	const_vn(toReturn, JointPositions[0], JointPositions[1], JointPositions[2],
							 JointPositions[3], JointPositions[4], JointPositions[5],
							 JointPositions[6]);
	
	for(i = 0; i < 7; i++){
		//printf("\nJointPosition[%d]: %f", (i+1), toReturn->q[i]);
	}
						
	// return the new jointspace vector
	return toReturn;
}


/* setToNeutralPosition:
	input: none
	output: none
	function: sets the WAM to a neutral position in jointspace */
void setToNeutralPosition(){
	//printf("\nMoving WAM to a neutral position.\n");
	setToPosition(neutral);
	//printf("\nAt Neutral Position \n");
}

/* setToHomePosition:
	input: none
	output:none 
	function: sets the WAM to a home position in jointspace */
void setToHomePosition(){
	setToNeutralPosition();
	
	//printf("\nMoving WAM home.\n");
	setToPosition(home);
	//printf("\nAt Home \n");
}

/* setToPosition:
	input: vect_n* Pos (position)
	output: none 
	function: sets to a position in jointspace and returns when done*/
void setToPosition(vect_n* Pos){
		MoveWAM(wam,Pos);
		//printf("\nMovingwam \n");
		int stillMoving = 1;
		float ref[6], pos[6], diff[6];
		
		/* checks the difference between positions of all the joints every .15 seconds
		   and exits when the difference is less than .001 radians for each joint */
		while(stillMoving){
			int i;
			stillMoving = 0;
			
			usleep(150000);
			for(i = 0; i < DOF; i++){
				ref[i] = ((vect_n*)wam->Jpos)->q[i];
			}
			
			usleep(150000);
			for(i = 0; i < DOF; i++){
				pos[i] = ((vect_n*)wam->Jpos)->q[i];
				diff[i] = ref[i]-pos[i];
				//printf("\n pos is: %f ref is %f\n", pos[i], ref[i]);
				
				if(fabs(diff[i]) < .001 && stillMoving == 0){
					stillMoving = 0;
				} else {
					stillMoving = 1;
				}
			}
		}
		usleep(1000000);
}

/* promptNextStep:
	input: none
	output: none
	function: waits until enter is hit twice (wait command) */
void promptNextStep(){
	int choice2;
	
	//printf("\nPress enter to continue.");
	fflush(stdin);
	getchar();
	while ((choice2 = getchar()) != '\n' && choice2 != EOF);	
}

/* getDOF
	input: none
	output: returns DOF
	function: returns the degrees of freedom */
int getDOF() { 
	return DOF; 
}

/* findJointStop:
	input: int motorID, int direction
	output: none
	function: finds a joint stop given a motor and direction */
void findJointStop(int motorID, int direction) {
	
	float diff;
	
	if(direction >= 0){
		currentDirection = 1;	
	} else {
		currentDirection = -1;
	}
	
	currentJoint = motorID;
	findingJointStop = 1;
	usleep(150000);
	
	do { /*continue requesting position until told to stop
		    (WAMcallback will tell findJointStop when a joint stop is found*/
			usleep(100000);
			//printf("\nCurrent position is %f", ((vect_n*)wam->Jpos)->q[motorID-1]);
			//printf("\nJoint Torque is %f", ((vect_n*)wam->Jtrq)->q[motorID-1]);
	} while(findingJointStop);

	// backs off a little from the joint stop once it is found
	// to avoid direct contact with joint stop later with working tension through the motor
	vect_n* temp = getJointPosition();
	float tempAngle = temp->q[motorID-1];
	
	if(tempAngle > 0){
		tempAngle -= .08;
	} else if (tempAngle < 0) {
		tempAngle += .08;
	}
	
	temp->q[motorID-1] = tempAngle;
	// sets back to the new position found (see previous comment)
	
	setToPosition(temp);
	
	//printf("Found jointstop on motor %d\n", motorID); 
}

/* findTangStop:
	input: int motorID, int direction
	output: none
	function: finds a tang given a motors and a direction */
void findTangStop(int motorID, int direction){
	float diff, previousPosition;

	if(direction >= 0){
		currentDirection = 1;	
	} else {
		currentDirection = -1;
	}
	
	//currentJoint should be regarded as the current motor for all intents and purposes
	currentJoint = motorID;
	findingTang = 1;
	previousPosition = ((vect_n*)wam->Jpos)->q[motorID-1];
	usleep(150000);
	
	do { /*continue requesting position until difference in 
		    current joint position is less than 0.001 radians*/
			usleep(150000);
			//printf("\nPrevious position is %f", previousPosition);
			//printf("\nCurrent position is %f", ((vect_n*)wam->Jpos)->q[motorID-1]);
			//printf("\nJoint Torque is %f", ((vect_n*)wam->Jtrq)->q[motorID-1]);
			
			diff = previousPosition - ((vect_n*)wam->Jpos)->q[motorID-1];
			if (fabs(diff) < .001){
				break;
			}
			previousPosition = ((vect_n*)wam->Jpos)->q[motorID-1];
	} while(findingTang);
	
	usleep(1000000);
	
	// findingTang flag remains true even though tang is found so that torque is still being applied
	// (torque remains so tensioning is easier to do)
	
	//printf("\nFound Tang.\n");
}

/* getTorques:
	input: none
	output: none
	function: gets the torques of the motors and saves them to currentTorque[6] */
void getTorques(){
	int i;
	for(i = 0; i < DOF; i++){
		currentTorque[i] = ((vect_n*)wam->Mtrq)->q[i];
	}
}

/* prepareMotor:
	input: int motorID, int direction
	output: none
	function: finds both joint stops for specified motor */
void prepareMotor(int motorID, int direction){
	
	getProperty(0,motorID,CTS,&ctsPerRev);
	
	/** find the first jointstop */
	setToNeutralPosition();
	
	//printf("\nSetting to Approximated Joint Stop.");
	setToPosition(jointStopApprox[motorID-1][1]);
	
	switch(motorID){
		case 1: case 4: 
		        findJointStop(motorID, direction);
			     break;
		case 2: findJointStop(3, direction);
				  findJointStop(2, -direction);
				  break;
		case 3: findJointStop(3, direction);
				  findJointStop(2, direction);
				  break;
		case 5: findJointStop(5, -direction);
				  findJointStop(6, direction);
				  break;
		case 6: findJointStop(5, -direction);
		        findJointStop(6, -direction);
				  break;
		default: break;
	}
	
	//save the joint stop position
	//printf("\nFound the first jointstop of motor%d",motorID);
	usleep(2000000);
	jointStop[motorID-1][1] = getJointPosition();
	
	//percentDone += (10.0)/numMotorsToTension;
	
	/** find the second jointstop */
	setToNeutralPosition();
	
	//printf("\nSetting to Approximated Joint Stop.");
	setToPosition(jointStopApprox[motorID-1][0]);
	
	switch(motorID){
		case 1: case 4: 
				  findJointStop(motorID, -direction);
				  break;
		case 2: findJointStop(3, -direction);
				  findJointStop(2, direction);
				  break;
		case 3: findJointStop(3, -direction);
				  findJointStop(2, -direction);
				  break;
		case 5: findJointStop(5, direction);
				  findJointStop(6, -direction);
				  break;
		case 6: findJointStop(5, direction);;
				  findJointStop(6, direction);
				  break;
		default: break;
	}
	        
	//printf("\nFound the second jointstop of motor%d",motorID);
	//save the joint stop position
	usleep(2000000);
	jointStop[motorID-1][0] = getJointPosition();
	//percentDone += (10.0)/numMotorsToTension;
}          
           
/** Note on tensioning:

	Tensioning works by applying tension to the motor (with the tang engaged)
	on three different places of the motor.  The tension applied is not additive,
	so the cables will not become overtensioned from multiple runs of autotensioning.
*/

/* deactivateTorque:
	input: none
	output: none
	function: resets torque control on WAM and updates its position in PID control 
	          (internal use only) */
void deactivateTorque(){
	//deactivate and activate wam PID control to reset PID and torques of motor
	setmode_bts(wam->active_sc,SCMODE_IDLE);               
	//disable findingTang flag while wam mode is in idle
	findingTang = 0;
	//reacitvate wam PID control
	setmode_bts(wam->active_sc,SCMODE_POS);            
	usleep(1000000);
}

/* engageMotor:
	input: int motorID, int direction
	output: none
	function: engages the tang and tensions a motor given a motor and direction
				 (engageMotor is only for motors 1-4) */         
void engageMotor(int motorID, int direction){	
	int i;  
	long startPos,endPos,diff;
	float motorOffset[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	vect_n* tensionPos[4];
	vect_n* toAdd = new_vn(7);
	vect_n* motorPosition = getMotorPosition();
	        
	currentJoint = motorID;
	
	//find tangs at least one motor revolution back from the joint stop
	motorOffset[motorID-1] += (-2 * 3.14159);
	
	// find tension positions for tensionPos[4]
	for(i = 0; i < 4; i++){
		motorOffset[motorID-1] = (i+1) * (-2 * 3.14159);
		const_vn(toAdd, motorOffset[0], motorOffset[1], motorOffset[2],
		                motorOffset[3], motorOffset[4], motorOffset[5],
							 motorOffset[6]);
		tensionPos[i] = motorToJointPosition(sum_vn(toAdd, motorPosition));
	}
	
	//cycle through three tension positons.  4th is needed to disengage tang after third tensioning.
	for(i = 0; i < 3; i++){
		setToPosition(tensionPos[i]);
		//percentDone += (5.0)/numMotorsToTension;
		//printf("\nTension Position %d set\n", (i+1));
		//activate tang
		setProperty(0,motorID,TENSION,FALSE,1);
		//get the torques of all the motors
		getTorques();
		//find the tang of the current motor
		findTangStop(motorID, 1);
		//gets the torques of the motors again
		getTorques();
		//reads the current position of the motor
		getProperty(0,motorID,AP,&startPos);
		//tensions the motor with accounting for the current torques of all the motors
		tensioning = 1;
		//wait one second  
		usleep(1000000);   
		//read the end positon
		getProperty(0,motorID,AP,&endPos);
		//disable tensioning torque (there is still torque on motor from findingTang flag)
		tensioning = 0;
		diff = startPos - endPos;
		
		//output the net encoder counts taken up
		//printf("\nTook up %ld encoder cts of cable", abs(diff));
		//save encoder counts taken up at this step
		lastTensionedMotorValues[i] = abs(diff);
		
		deactivateTorque();
		//deactivate tang
		setProperty(0,motorID,TENSION,FALSE,0);                        
	}
	//disengage tang from motor
	setToPosition(tensionPos[3]);    
	//percentDone += (5.0)/numMotorsToTension;
}                                                         
	                                       
/*
	engageWrist:
	input: int motorID, int direction
	output: none
	function: engages and tensions a specified motor on a wrist
*/
void engageWrist(int motorID, int direction){
	
	//increment is 1/20th of a revolution
	float diff, tensionInc = ((3.14159)*2.0)/20.;
	float referencePos;
	long startPos, endPos, diffPos;
	int counter = 0, i, j, referenceMotor;
	
	vect_n* toAdd = new_vn(7);
	float motorOffset[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	//61 possible tension positions (position 61 is to disengage the tang)
	vect_n* tensionPos[61];
	vect_n* motorPosition = getMotorPosition();

	motorOffset[motorID-1] += (-2 * 3.14159);
	
	//calculate 61 possible locations of where the tang can be by rotating only the 
	//specified motor by tensionInc and saving the position (repeated 61 times)
	for(i = 0; i < 61; i++){
		if(i != 60 && i != 0) {
			motorOffset[motorID-1] += (-1 * tensionInc);
			//special cases for last position
		} else if (i == 60) {
			motorOffset[motorID-1] += (-2 * 3.14159);
		}
		//construct a new vector with the offsets and convert it from motor position to joint position
		const_vn(toAdd, motorOffset[0], motorOffset[1], motorOffset[2],
		                motorOffset[3], motorOffset[4], motorOffset[5],
							 motorOffset[6]);
		tensionPos[i] = motorToJointPosition(sum_vn(toAdd, motorPosition));
	}
	
	//for use in WAMmotorcallback
	currentJoint = motorID;
	
	//opposite motor on the wrist from the current one
	if(motorID == 5){
		referenceMotor = 6;
	} else if (motorID == 6){
		referenceMotor = 5;
	}
	
	//move to tension positions to find the tang
	for(i = 0; i < 60 && counter < 3; i++){
		//printf("\nAbout to set to position %d: ", i);
		setToPosition(tensionPos[i]);
		setProperty(0,5,TENSION,FALSE,1);
		//save the current motor position
		referencePos = ((vect_n*)wam->Mpos)->q[referenceMotor-1];
		getTorques();
		//try to engage the opposite motor
		findingTang = 1;
		usleep(2000000);
		
		//check to see if the opposite motor is engaged
		diff = ((vect_n*)wam->Mpos)->q[referenceMotor-1] - referencePos;	
		if(fabs(diff) < (2.0 * 3.14159)){
			deactivateTorque();
			getTorques();
			//printf("\nI think I found one tang on the wrist?\n");
			//if it is, try to engage the current motor
			
			referencePos = ((vect_n*)wam->Mpos)->q[motorID-1];
			findingTang = 2;
			usleep(1000000);
			diff = ((vect_n*)wam->Mpos)->q[motorID-1] - referencePos;
			
			if(abs(diff) < 3.14/5.0){
				//printf("\nI think I found the second tang and i'm about to tension");
				//deactivate tang (motors should still be engaged in tang at this point as they are stuck)
				setProperty(0,5,TENSION,FALSE,0);
				usleep(500000);
				getTorques();
				//save a current motor encoder count position
				getProperty(0,motorID,AP,&startPos);
				//apply tension to the motor
				tensioning = 1;
				usleep(1000000);
				//save the end motor encoder count position (after tensioning)
				getProperty(0,motorID,AP,&endPos);
				//deactivate tensioning (findingTang is still enabled)
				tensioning = 0;
				//calculate tension amount taken up and save it
				diffPos = startPos - endPos;
				lastTensionedMotorValues[counter] = abs(diffPos);
				//percentDone += (10.0)/numMotorsToTension;
				counter++;
				
				//go ahead 18/20ths of a revolution
				if(abs(diffPos) > 5){
					i += 18;
				}
			} 
		} 
		deactivateTorque();
		//make sure tang is deactivated
		setProperty(0,5,TENSION,FALSE,0);
		//percentDone += (10.0)/numMotorsToTension;
	}                                                    
	
	//disengage motor from tang if it is engaged
	setToPosition(tensionPos[60]);
}

/* oscillateMotor:
	input: int motorID
	output: none
	function: works tension through a motor (prepareMotor must be called on motor first 
				 in order for oscillateMotor to work) */
void oscillateMotor(int motorID){
	int i = 0;
	float val;
	
	//moves WAM to a safe position
	setToNeutralPosition();
	MoveSetup(wam,2.0,0.4);  	
	
	if(motorID != 5 && motorID != 6){
		val = 2.75;
	} else {
		val = 1.9;
	}
	
	//moves wam between the two jointstops
	for(i = 0; i < 10; i++){
		setToPosition(jointStop[motorID-1][1]);
		//percentDone += val/numMotorsToTension;
		setToPosition(jointStop[motorID-1][0]);
		//percentDone += val/numMotorsToTension;
	}
	
	//moves WAM back to a safe position
	MoveSetup(wam,1.0,0.3);

	if(motorID != 5 && motorID != 6){
		//percentDone += 5.0/numMotorsToTension;
	} else {
		//percentDone += 2.0/numMotorsToTension;
	}
}

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
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
