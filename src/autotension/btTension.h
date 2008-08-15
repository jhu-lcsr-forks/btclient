/*======================================================================*
 *  Module .............WAM autotension
 *  File ...............btTension.h
 *  Author .............Kevin Doo
 *  Creation Date ......12 Aug 2008
 *                                                                      *
 *======================================================================*/
 
 
#ifndef _BTTENSION_H
#define _BTTENSION_H
 
/** Included Header files */
#include "btwam.h"
#include "btcan.h"

/** Public Function Prototypes */

/** general */
void initialize(wam_struct* wam, FILE * logFile); 
                                  /* creates vectors and variables necessary
												 to use other functions in btTension */
int menu();								 /* menu for autotension program */				
void promptNextStep();        	 /* waits until enter is hit twice */

/** get methods */
int getDOF(); /* gets the degrees of freedom */

/** position setters for the WAM */
void setToHomePosition(); /* sets WAM to home position */	

#endif

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
