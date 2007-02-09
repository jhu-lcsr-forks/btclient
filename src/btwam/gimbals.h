/*
   Module .............libbt                                
   File ...............gimbals.h                                          
   Author .............Brian Zenowich                                 
   Creation Date ......Oct 31, 2003                                 

 ***********************************************************************  
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:                                                              
 *   Reads the gimbals angles, initializes the gimbals if necessary                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  031031 - BZ - File created                                          
 *                                                                      
 *======================================================================*/

/* Gimbals data structures
*/
#ifndef _GIMBALS_H
#define _GIMBALS_H

int getGimbalsAngles(wam_struct *WAM,double *gimbals);
int initGimbals(wam_struct *WAM);

#endif /* _GIMBALS_H*/