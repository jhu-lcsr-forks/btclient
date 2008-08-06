/** Included Header files */
#include "btwam.h"
#include "btcan.h"
#include "btmath.h"

/** Public Function Prototypes */

/** general */
void initialize(wam_struct* wam); /* creates vectors and variables necessary
												 to use other functions in btTension */
int menu();								 /* menu for autotension program */				
void promptNextStep();        	 /* waits until enter is hit twice */

/** get methods */
int getDOF(); /* gets the degrees of freedom */

/** position setters for the WAM */
void setToHomePosition(); /* sets WAM to home position */	
