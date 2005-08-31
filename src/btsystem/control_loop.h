/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............control_loop.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Nov 23, 2002 
 *  Addtl Authors ......Brian Zenowich, Sam Clanton
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
 *   Infrastructure to simplify adding and using your own control threads. 
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  021123 - TH - File created                                          
 *                                                                      
 *======================================================================*/

 /* \file control_loop.h
    \brief Infrastructure to simplify adding and using your own control threads.
    
    This module adds code to start up timing threads that will call a user specified 
  control thread at regular intervals.
    
   
*/ 
#ifndef _CONTROL_H
#define _CONTROL_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
/*  Example code for passing parameters. See control_loop.c example.
#include "btsystem.h"
#include "btjointcontrol.h"

  
typedef struct 
{
  int num_act;
  actuator_struct *act;
  SimpleCtl *sc;
}control_thd_parms;
*/

void start_control_threads(int priority, double sample_period, void *function,void *args);
void stop_control_threads();
void TimerThread();
void ControlThread(void *data);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _CONTROL_H */


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
 

