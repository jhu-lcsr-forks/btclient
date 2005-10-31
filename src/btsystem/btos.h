                
/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btos.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 28, 2005 
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:                                                              
 *   Operating system abstractions and helpers                       
 *                                                                      
 *  REVISION HISTORY:                                                   
 *                                                                      
 *======================================================================*/

/** \file btos.h
    \brief Operating system abstractions and helpers

A thin layer between barrett technologies code and the operating system.
Additionally, global defines can go here also. Virtually every Barrett library
source file will use this.

The mutex layer allows for ERRORCHECK mutexes to be compiled in if desired for 
debugging.

btmalloc() and btfree() provide error checking for memory access. Lack of memory 
is fatal. btfree() sets the calling variable to NULL 

test_and_log() provides a shorthand to replace return variable checks.

Additionally, btos.h has #defines for error checking:

-BTDEBUG (This is a long bitfield)
Global Values 0-32
  - 0 No debugging code compiled in
  - 1 Sanity warnings
  - 3 Sanity checks (fix on the fly if we can)
  - 7 Communications layer
  - 8 Realtime stuff
  - 9 Math stuff (NAN, div zero)
  - 10 Pointers & bounds
  - 20 Rediculous verbosity
Bits (by position starting with 0
  - 
*/
#define BTDEBUG_RANGE         0xff
// if (BTDEBUG & BTDEBUG_RANGE) > 3 ->sanity checks
#define BTDEBUG_PARSER        0x10
#define BTDEBUG_SYSTEM        0x20
#define BTDEBUG_MATH_VECT     0x40
#define BTDEBUG_MATH_MATR     0x80
#define BTDEBUG_MATH          0x100
#define BTDEBUG_STATECONTROL  0x200
#define BTDEBUG_CONTROL       0x400

/**  
-BT_NULL_PTR_GUARD //bt*.c functions will check incoming object pointers
to make sure they are not NULL and error if they are.

-BT_ARRAY_BOUNDS_CHECK //some sort of index sanity and bounds checking will 
be done on incoming functions

-BT_DUMMY_PROOF //extra code will be compiled in to protect the programmer
from thier own idiocy.

-BT_BACKTRACE //Dump backtrace info into bterrors.txt

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
//#include <rtai_lxrt.h>
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




#ifdef BT_NULL_PTR_GUARD
  #define BTPTR_OK(x,y) btptr_ok((x),(y));
#else
  #define BTPTR_OK(x,y) 
#endif
int btptr_ok(void *ptr,char *str);
int btptr_chk(void *ptr); //like btptr_ok but no syslog
int idx_bounds_ok(int idx,int max,char *str);

//void dump_backtrace_to_syslog();

BTINLINE int test_and_log(int ret,const char *str);

//Memory
BTINLINE void* btmalloc(size_t size);
BTINLINE void btfree(void **ptr);


//Threads
/** Convinience info for creation of threads

See new_btthread()
*/
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
  
  //RTIME sampleCount;
}btthread;

btthread* new_btthread();
void free_btthread(btthread **thd);

pthread_t* btthread_create(btthread *thd,int priority, void *function,void *args);
int btthread_done(btthread *thd); //ret !0 when time to kill
void btthread_stop(btthread *thd); //set done = 1;
void btthread_exit(btthread *thd);

int btperiodic_create(btthread *thd,int priority, double period, void *function,void *args);




#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _SIMPLECONTROL_H */


