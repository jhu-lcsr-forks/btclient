/*
   Module .............libbt                                
   File ...............gimbals.h                                          
   Author .............Brian Zenowich                                 
   Creation Date ......Oct 31, 2003                                 

 ***********************************************************************  
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation (version 2).
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
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

int getGimbalsAngles(double *gimbals);
int initGimbals(void);

#endif /* _GIMBALS_H*/