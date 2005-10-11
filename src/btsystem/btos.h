                
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

/** 
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
#include <rtai_lxrt.h>
/*mutex & threads*/
/**
The intention of the thread functions is to provide a central point for
mutex error handling and debugging.

See the gnu_libc backtrace function for debugging output.
*/
typedef pthread_mutex_t btmutex;
int btmutex_init(btmutex* btm);
BTINLINE int btmutex_lock(btmutex* btm);
//int btmutex_lock_msg(btmutex* btm,char *msg);
BTINLINE int btmutex_unlock(btmutex *btm);

#ifdef BT_DEBUG_ON
  #define BT_DEBUG(x) (x)


#ifdef BT_NULL_PTR_GUARD
  #define BTPTR_OK(x,y) btptr_ok((x),(y));
#else
  #define BTPTR_OK(x,y) 
#endif
int btptr_ok(void *ptr,char *str);
int idx_bounds_ok(int idx,int max,char *str);

//void dump_backtrace_to_syslog();

BTINLINE int test_and_log(int ret,const char *str);

//Memory
BTINLINE void* btmalloc(size_t size);
BTINLINE void btfree(void **ptr);


//Threads
typedef struct {
  pthread_t thd_id;
  pthread_attr_t attr;
  struct sched_param param;
  
  int priority;
  int periodic;
  double period;
  int done;
  
  void* function;
  void* data;
  btmutex mutex;
  
  RTIME sampleCount;
}btthread;

btthread* new_btthread();
void free_btthread(btthread **thd);

int btthread_create(btthread *thd,int priority, void *function,void *args);
int btthread_done(btthread *thd); //ret !0 when time to kill
void btthread_stop(btthread *thd); //set done = 1;
void btthread_exit(btthread *thd);

int btperiodic_create(btthread *thd,int priority, double period, void *function,void *args);




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
