/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btos.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 28, 2005
 *                                                                      *
 *  ******************************************************************  */
/** \file btos.c
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

#ifdef BT_NULL_PTR_GUARD //bt*.c functions will check incoming object pointers
to make sure they are not NULL and error if they are.

#ifdef BT_ARRAY_BOUNDS_CHECK //some sort of index sanity and bounds checking will 
be done on incoming functions

#ifdef BT_DUMMY_PROOF //extra code will be compiled in to protect the programmer
from thier own idiocy.

#ifdef BT_BACKTRACE //Dump backtrace info into bterrors.txt



*/
 
 
#include <syslog.h>
#include <stdlib.h>
#include "btos.h"


int btmutex_init(btmutex* btm){
  int ret;
  pthread_mutexattr_t mattr;
  pthread_mutexattr_init(&mattr);
  pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_TIMED_NP);
  //pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_ERRORCHECK_NP);

  ret = test_and_log(
    pthread_mutex_init(btm,&mattr),
    "Could not initialize mutex.");
  return ret;
}

BTINLINE int btmutex_lock(btmutex* btm)
{
  int ret;
  ret = pthread_mutex_lock(btm);
  if (ret != 0)
  {
    syslog(LOG_ERR, "Mutex lock failed: %d", ret);
  }
  return ret;
}
int btmutex_lock_msg(btmutex* btm,char *msg);
BTINLINE int btmutex_unlock(btmutex *btm)
{
  int ret;
  ret = pthread_mutex_unlock(btm);
  if (ret != 0)
  {
    syslog(LOG_ERR, "Mutex unlock failed: %d",  ret);
  }
  return ret;
}


/** Null pointer access flag */
int btptr_ok(void *ptr,char *str)
{
  if (ptr == NULL){
#ifdef BT_BACKTRACE 
/**\bug insert backtrace code here*/
#endif
    syslog(LOG_ERR,"bt ERROR: you tried to access a null pointer in %s",str);
    return 0;
  }
  return 1;
}
/** Prints an error if array index is out of range */
int idx_bounds_ok(int idx,int max,char *str)
{
  if ((idx < 0) || (idx > max)){
#ifdef BT_BACKTRACE 
/**\bug insert backtrace code here*/
#endif
    syslog(LOG_ERR,"bt ERROR: Your index is %d with limit %d in function %s",idx,max,str);
    return 0;
  }
  return 1;
}

BTINLINE int test_and_log(int ret, const char *str)
{
  if (ret != 0)
  {
    syslog(LOG_ERR, "%s: %d", str, ret);
    return ret;
  }
  else 
    return 0;
}

/**Memory allocation wrapper

  Causes an exit if we run out of memory.
*/
BTINLINE void * btmalloc(size_t size)
{
 void* vmem;

  //allocate mem for vector,return vector, and return structure
  if ((vmem = malloc(size)) == NULL) 
  {
    syslog(LOG_ERR,"btMalloc: memory allocation failed, size %d",size);
    exit(-1);
  }
  return vmem;
}
/**Memory deallocation wrapper

  free's memory at *ptr and then sets *ptr to NULL.
*/

BTINLINE void btfree(void **ptr)
{
#ifdef BT_NULL_PTR_GUARD
  if(btptr_ok(*ptr,"btfree"))
#endif
  free(*ptr);
  *ptr = NULL;
}


btthread* new_btthread()
{
  btthread* mem;
  mem = (btthread*)btmalloc(sizeof(btthread));
  return mem;
}

void free_btthread(btthread **thd)
{
  btfree((void**)thd);
}
/**  Create a new thread

We create a new posix thread with a schedpolicy of SCHED_FIFO. The thread_id
is returned.

\param
  thd The barrett thread structure; allocated before calling this function.
  priority The priority we wish to call this thread with. 0 = linux non-realtime priority. 99 = Max priority
  function Pointer to the function that represents the thread
  args Pointer to the arguments you want to pass.
\internal 

right now we kill the program if a thread create doesn't work. I'm not sure if this 
is reasonable.

*/
pthread_t* btthread_create(btthread *thd,int priority, void *function,void *args)
{
  pthread_attr_init(&(thd->attr));
  pthread_attr_setschedpolicy(&(thd->attr), SCHED_FIFO);
  pthread_attr_getschedparam(&(thd->attr),&(thd->param));
  thd->param.sched_priority = priority;
  pthread_attr_setschedparam(&(thd->attr), &(thd->param));
  
  thd->done = 0;
  thd->function = function;
  thd->data = args;
  thd->priority = priority;
  thd->periodic = 0;
  btmutex_init(&(thd->mutex));
  
  pthread_create(&(thd->thd_id), &(thd->attr), function, args);
  
  if (thd->thd_id == -1)
  {
    syslog(LOG_ERR,"btthread_create:Couldn't start control thread!");
    exit(-1);
  }
  return &(thd->thd_id);
}
BTINLINE int btthread_done(btthread *thd)
{
  int done;
  btmutex_lock(&(thd->mutex));
  done = thd->done;
  btmutex_unlock(&(thd->mutex));
  return done;
}

/** Stop a thread that is using btthread_done()

This function should only be called from outside the thread you want to stop.
The thread monitors thd->done using btthread_done()
\code
void mythread(void* args)
{
  btthread *mythd;
  mythd = (btthread*)args;
  
  while(!btthread_done(mythd))
  {
    //do something
  }
  pthread_exit(NULL);
}
\endcode

*/
BTINLINE void btthread_stop(btthread *thd)
{
  btmutex_lock(&(thd->mutex));
  thd->done = 1;
  btmutex_unlock(&(thd->mutex));
  pthread_join(thd->thd_id,NULL);
}

BTINLINE void btthread_exit(btthread *thd)
{
  pthread_exit(NULL);
}

void btperiodic_proto(void *args)
{
  
  
  
}
/*
#include <rtai_lxrt.h>

int btperiodic_create(btthread *thd,int priority, double period, void *function,void *args)
{

  RTIME rtime_period;
  
  rtime_period = (RTIME)(period * 1000000000.0);
  
  thd->sampleCount = nano2count(rtime_period);

  rt_set_periodic_mode();
  
  btthread_create(thd,priority,function,args);
  
  start_rt_timer(thd->sampleCount);
}
*/
/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2005 Barrett Technology, Inc.           *
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


