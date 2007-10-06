
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
    \brief Operating system abstractions and helpers.
 
 
   
\section BTOS 
    
The btos module is a thin layer between barrett technologies code and the operating system.
Additionally, global defines can go here also. Virtually every Barrett library
source file will use this.
 
These are the submodules that btos implements:
 - Thread API: A simplified api for starting, stopping, and monitoring threads.
 - Mutex API: A wrapper around pthread_mutex_* that writes errors to syslog.
 - Malloc API: Wrappers for malloc & free that automate error handling.
 - Pointer & Array Sanity Checks: Used for debugging.
 - Convinience functions: Typing is such a chore. 
 
The mutex layer allows for ERRORCHECK mutexes to be compiled in if desired for 
debugging.
 
btmalloc() and btfree() provide error checking for memory access. Lack of memory 
is fatal. btfree() sets the calling variable to NULL 
 
 
 
Additionally, btos.h has #defines for error checking:
- BT_NULL_PTR_GUARD: bt*.c functions will check incoming object pointers
to make sure they are not NULL and error if they are.
- BT_ARRAY_BOUNDS_CHECK: some sort of index sanity and bounds checking will 
be done on incoming functions
- BT_DUMMY_PROOF: extra code will be compiled in to protect the programmer
from thier own idiocy.
- BT_BACKTRACE: Dump backtrace info into bterrors.txt
- BTDEBUG (This is a long bitfield)
*/
/**
\page  boc Barrett Object Conventions
 
Much of the btsystem and btwam libraries are written in an object oriented style.
OOP in C is less clean but easier to bind to other languages. 
To keep things sane, we follow certain conventions when working with the objects
used in our library. Since this is C and not C++; it's trickier. Treat the 
following functionname_obj() as reserved functions.
 
 - Pointers: Pointers to objects should be initialized to a NULL value.
             A NULL object pointer is treated as on empty pointer and is handled 
             gracefully. When an object is destroyed; It's referencing pointer is
             set to NULL.
             
 - Object memory allocation: 
   - malloc_obj() - Allocate memory for this object and sub-objects.
   - local_obj() - Macro for allocating this object on the local function stack. (It will be deallocated when the function exits)
   - sizeof_obj() - Size in bytes needed for an instance of this object
   - destroy_obj() - Free memory and set referencing pointer to NULL
   
 - Object initialization:
   - initval_obj() - Initialize the values in the object to sane values
   - initptr_obj() - Initialize the structure of the object (inter-object pointers)
   
 - Compound functions:
   - init_obj() - Initialize values and structure of the object.
   - new_obj() - Allocate memory for the object and initialization.
\internal
\todo Update all of btclient to use these conventions.
 
*/
#ifndef _BTOS_H
#define _BTOS_H

/*
Global Values 0-32
  - 0 No debugging code compiled in
  - 1 Sanity warnings
  - 3 Sanity checks (fix on the fly if we can)
  - 7 Communications layer
  - 8 Realtime stuff
  - 9 Math stuff (NAN, div zero)
  - 10 Pointers & bounds
  - 11 Rediculous verbosity
Bits (by position starting with 0
  - 
*/

#define BTDEBUG_RANGE         0x0f
// if (BTDEBUG & BTDEBUG_RANGE) > 3 ->sanity checks
#define BTDEBUG_PARSER        0x10
#define BTDEBUG_SYSTEM        0x20
#define BTDEBUG_MATH_VECT     0x40
#define BTDEBUG_MATH_MATR     0x80
#define BTDEBUG_MATH          0x100
#define BTDEBUG_STATECONTROL  0x200
#define BTDEBUG_CONTROL       0x400
#define BTDEBUG_COMM          0x800
#define BTDEBUG_BACKTRACE     0x1000
/*Compiler flags in use:
ifdefs:
  BT_BACKTRACE
  BT_ARRAY_BOUNDS_CHECK
  BT_NULL_PTR_GUARD
  
*/

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

#ifndef BTINLINE
#define BTINLINE inline
#endif

#include <pthread.h>
/** \todo Move periodic thread api to another include file to remove dependancy on rtai in this file */
//#include <rtai_lxrt.h>
#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>

#else
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif

/*mutex & threads*/
/** @name Mutex API */
//@{

/**
The intention of the thread functions is to provide a central point for
mutex error handling and debugging.
 
 
*/
typedef pthread_mutex_t btmutex;

#ifdef XENOMAI
typedef RT_MUTEX btrt_mutex;
#endif

#ifdef RTAI
typedef pthread_mutex_t btrt_mutex;
#endif

/* Realtime mutexes - must be used from realtime threads */
int btrt_mutex_init(btrt_mutex* btm);
int btrt_mutex_init(btrt_mutex* mutex);
int btrt_mutex_create(btrt_mutex* mutex);
BTINLINE int btrt_mutex_lock(btrt_mutex* btm);
BTINLINE int btrt_mutex_unlock(btrt_mutex *btm);

/* Non-realtime mutexes - must be used from non-realtime threads */
int btmutex_init(btmutex* btm);
int btmutex_init(btmutex* mutex);
int btmutex_create(btmutex* mutex);
BTINLINE int btmutex_lock(btmutex* btm);
BTINLINE int btmutex_unlock(btmutex *btm);

//int btrt_mutex_lock_msg(btrt_mutex* btm,char *msg);
//@}

/** @name Pointer and Array Sanity Checking */
//@{
#ifdef BT_NULL_PTR_GUARD
#define BTPTR_OK(x,y) btptr_ok((x),(y));
#else
#define BTPTR_OK(x,y)
#endif
int btptr_ok(void *ptr,char *str);
int btptr_chk(void *ptr); //like btptr_ok but no syslog
int idx_bounds_ok(int idx,int max,char *str);

//@}

/** @name Convinience functions */
//@{

BTINLINE int test_and_log(int ret,const char *str);
//@}

/** @name Malloc & Free wrappers */
//@{
//Memory
BTINLINE void* btmalloc(size_t size);
BTINLINE void btfree(void **ptr);


//Threads
//@}

/** @name Threads: */
//@{
/** Convenience info for creation of threads
    This is for non-realtime threads.
See new_btthread()
*/
typedef struct {
   pthread_t thd_id;
   pthread_attr_t attr;
   struct sched_param param;

   int priority; //Priority this thread was created at
   int periodic; //!0 = This thread is a periodic one
   double period; // The period we want this thread to be
   int done; //!< See btthread_done()
   void (*function)(void *data); //Pointer to the function this thread is running
   void* data;
   btmutex mutex;

   RTIME actual_period,proc_time;
}btthread;

// This is for realtime threads
// If not Xenomai, the structure will be the same as btthread
typedef struct {
#ifdef XENOMAI
   RT_TASK task;
#else
   pthread_t thd_id;
   pthread_attr_t attr;
   struct sched_param param;
#endif

   int priority; //Priority this thread was created at
   int periodic; //!0 = This thread is a periodic one
   double period; // The period we want this thread to be
   int done; //!< See btthread_done()
   void (*function)(void *data); //Pointer to the function this thread is running
   void* data;
   
   btrt_mutex mutex;

   RTIME actual_period,proc_time;

}btrt_thread_struct;

btthread* new_btthread();
btrt_thread_struct* new_btrt_thread();
void free_btthread(btthread **thd);

pthread_t* btthread_create(btthread *thd,int priority, void *function,void *args);
void btrt_thread_create(btrt_thread_struct *thd, const char *name, int priority, void *function, void *args);

int btthread_done(btthread *thd); //ret !0 when time to kill
int btrt_thread_done(btrt_thread_struct *thd);

void btthread_stop(btthread *thd); //set done = 1;
void btrt_thread_stop(btrt_thread_struct *thd);

void btthread_exit(btthread *thd);
void btrt_thread_exit(btrt_thread_struct *thd);

void btrt_thread_join(btrt_thread_struct *thd);

/** Create a periodic thread. Not yet implemented. */
pthread_t* btperiodic_create(btthread *thd,int priority, double period, void *function,void *args);
//@}

//RT Abstractions 
//@{
void btrt_set_mode_soft(void);
void btrt_set_mode_hard(void);
void btrt_set_mode_warn(void);
void btrt_task_wait_period(void);



#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _SIMPLECONTROL_H */


