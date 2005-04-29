                
/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btos.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 28, 2005 
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
 *   Operating system abstractions and helpers                       
 *                                                                      
 *  REVISION HISTORY:                                                   
 *                                                                      
 *======================================================================*/

 /* \file btos.h  
    \brief Operating system abstractions and helpers
    
    
*/ 
#ifndef _BTOS_H
#define _BTOS_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

#ifndef BTINLINE
#define BTINLINE inline
#endif

#include <pthread.h>
//Callback list

//Bailout function
/**
  During initialization files are opened and devices accessed. If something fails
  during one of the startup steps, we want to rollback; closing files and deallocating
  memory and shutting down GUI's.
  
  \bug Finish this code
*/
void register_btexit_function();
void btexit();

//mutex & threads
typedef pthread_mutex_t btmutex;





BTINLINE int test_and_log(int ret,const char *str);
















#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _SIMPLECONTROL_H */

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
