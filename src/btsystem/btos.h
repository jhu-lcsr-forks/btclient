                
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

//mutex & threads
/**
The intention of the thread functions is to provide a central point for
mutex error handling and debugging.

See the gnu_libc backtrace function for debugging output.
*/
typedef pthread_mutex_t btmutex;
int btmutex_init(btmutex* btm);
int btmutex_lock(btmutex* btm);
int btmutex_lock_msg(btmutex* btm,char *msg);
int btmutex_unlock(btmutex *btm);


#ifdef NULL_PTR_GUARD
  #define BTPTR_OK(x,y) btptr_ok((x),(y))
#else
  #define BTPTR_OK(x,y) 
#endif

#define LogErr(x) test_and_log((x),"")

int btptr_ok(void *ptr,char *str);
BTINLINE int test_and_log(int ret,const char *str);

//Memory
BTINLINE void * btmalloc(size_t size);
BTINLINE void btfree(void **ptr);













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
