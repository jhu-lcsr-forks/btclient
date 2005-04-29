/*
   Module .............libbt                                
   File ...............gimbals.h                                          
   Author .............Brian Zenowich                                 
   Creation Date ......Oct 31, 2003                                 

 ***********************************************************************  
 *                                                                      
 *  NOTES:                                                              
 *   Reads the gimbles angles, initializes the gimbals if necessary                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  031031 - BZ - File created                                          
 *                                                                      
 *======================================================================*/

/* Gimbals data structures
*/
#ifndef _GIMBALS_H
#define _GIMBALS_H

int getGimbalsAngles(double *gimbals);
int initGimbals(void);

#endif /* _GIMBALS_H*/

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003, 2004 Barrett Technology, Inc.           *
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
