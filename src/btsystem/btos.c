/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btos.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 28, 2005
 *                                                                      *
 *  *********************************************************************
 *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>        *
 *
 *
 *  NOTES:
 *   
 *
 *  REVISION HISTORY:
 *  TH 051101 - Final pass. Needs testing. Esp. Periodic threads.
 *
 *======================================================================*/

#include <syslog.h>
#include <stdlib.h>
#include "btos.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
void *backtracearray[50];
char **backtracestrings;
int backtracesize;

/*==============================*
 * Internal use Functions       *
 *==============================*/
void syslog_backtrace(int size)
{
   int cnt;
   for (cnt = 0;cnt < size;cnt ++)
      syslog(LOG_ERR,"WAM:Backtrace:%s",backtracestrings[cnt]);
}

/*==============================*
 * Functions                    *
 *==============================*/
/** Initialize a mutex.
 
If pthread_mutex_init() fails, an error message is printed to syslog.
 
\return Result of pthread_mutex_init().
\exception Undefined if btm does not point to memory block meant for a btmutex.
\internal chk'd TH 051101
\todo Error checking mutexes enabled by compiler switch.
*/
int btmutex_init(btmutex* btm)
{
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

/** Lock a btmutex.
See pthread_mutex_lock() in pthread.h for more info.
This function calls pthread_mutex_lock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_lock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int btmutex_lock(btmutex* btm)
{
   int ret;
   ret = pthread_mutex_lock(btm);
   if (ret != 0) {
      syslog(LOG_ERR, "Mutex lock failed: %d", ret);
   }
   return ret;
}

/** Unlock a btmutex.
See pthread_mutex_unlock() in pthread.h for more info.
This function calls pthread_mutex_unlock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_unlock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int btmutex_unlock(btmutex *btm)
{
   int ret;
   ret = pthread_mutex_unlock(btm);
   if (ret != 0) {
      syslog(LOG_ERR, "Mutex unlock failed: %d",  ret);
   }
   return ret;
}

/** Check pointer for a NULL value.
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
\todo Eventually we should check for pointers outside of program heap instead of
just null pointers. See sbrk() [gnu libc] for finding end of data segment. 
 
*/
int btptr_ok(void *ptr,char *str)
{
   if (ptr == NULL) {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: you tried to access a null pointer in %s",str);
      return 0;
   }
   return 1;
}

/** Pointer out of range check.
 
Presently only checks for NULL.
Has same (backwards) return values as btptr_ok().
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
*/
int btptr_chk(void *ptr)
{
   if (ptr == NULL) {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      return 0;
   }
   return 1;
}

/** Prints an error if array index is out of range.
 
\retval 0 Array index is NOT valid.
\retval 1 Array index is valid.
 
\exception Undefined if str points to something not a string and index is not valid.
\internal chk'd TH 051101
 
*/
int idx_bounds_ok(int idx,int max,char *str)
{
   if ((idx < 0) || (idx > max)) {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: Your index is %d with max %d in function %s",idx,max,str);
      return 0;
   }
   return 1;
}

/** Provides a shorthand to replace return variable checks.
\internal chk'd TH 051101
*/
BTINLINE int test_and_log(int return_val, const char *str)
{
   if (return_val != 0) {
      syslog(LOG_ERR, "%s: %d", str, return_val);
      return return_val;
   } else
      return 0;
}

/**Memory allocation wrapper.
 
\return Pointer to allocated memory.
\exception If malloc returns NULL there is no memory left and we kill the process!
\internal chk'd TH 051101
 
*/
BTINLINE void * btmalloc(size_t size)
{
   void* vmem;
   if ((vmem = malloc(size)) == NULL) {
      syslog(LOG_ERR,"btMalloc: memory allocation failed, size %d",size);
      exit(-1);
   }
   return vmem;
}

/**Memory deallocation wrapper.
  Free's memory at *ptr and then sets *ptr to NULL.
\exception If *ptr points to a block of memory that was not allocated with btmalloc[malloc]
or if that block was previosly freed the results are undefined.
\internal chk'd TH 051101
*/
BTINLINE void btfree(void **ptr)
{
#ifdef BT_NULL_PTR_GUARD
   if(btptr_ok(*ptr,"btfree"))
#endif

      free(*ptr);
   *ptr = NULL;
}

/** Allocate memory for a btthread object.
\return Pointer to a newly allocated btthread object.
\internal chk'd TH 051101
*/
btthread* new_btthread()
{
   btthread* mem;
   mem = (btthread*)btmalloc(sizeof(btthread));
   return mem;
}

/** Free memory for a btthread object.
\internal chk'd TH 051101
*/
void free_btthread(btthread **thd)
{
   btfree((void**)thd);
}

/**  Create a new thread.
 
We create a new posix thread with a schedpolicy of SCHED_FIFO. The thread_id
is returned.
 
\param  thd The barrett thread structure; allocated before calling this function.
\param  priority The priority we wish to call this thread with. 0 = linux non-realtime priority. 99 = Max priority
\param  function Pointer to the function that represents the thread.
\param  args Pointer to the arguments you want to pass.
 
\internal chk'd TH 051101 
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

   pthread_create(&(thd->thd_id), &(thd->attr), function, thd);

   if (thd->thd_id == -1) {
      syslog(LOG_ERR,"btthread_create:Couldn't start control thread!");
      exit(-1);
   }
   return &(thd->thd_id);
}

/** See btthread_stop().
\internal chk'd TH 051101 
*/
BTINLINE int btthread_done(btthread *thd)
{
   int done;
   btmutex_lock(&(thd->mutex));
   done = thd->done;
   btmutex_unlock(&(thd->mutex));
   return done;
}

/** Stop a thread that is using btthread_done().
 
This function should only be called from outside the thread you want to stop.
The thread monitors thd->done using btthread_done().
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
\internal chk'd TH 051101 
*/
BTINLINE void btthread_stop(btthread *thd)
{
   btmutex_lock(&(thd->mutex));
   thd->done = 1;
   btmutex_unlock(&(thd->mutex));
   pthread_join(thd->thd_id,NULL);
}

/** Call pthread_exit() on this btthread object.
\internal chk'd TH 051101
*/
BTINLINE void btthread_exit(btthread *thd)
{
   pthread_exit(NULL);
}

/**
\internal 
\todo
  This function will set up periodic timing and then call the thd->function 
  once a period. It's meant to allow easy periodic threading.
 
\warning Untested!!!
 
*/
void btperiodic_proto(void *args)
{
   btthread* this_thd;
   RT_TASK *ThreadTask;

   double thisperiod;
   RTIME rtime_period;
   RTIME last_loop,loop_start,loop_end;

   this_thd = (btthread*)args;
   thisperiod = this_thd->period;
   rtime_period = (RTIME)(thisperiod * 1000000000.0);

   ThreadTask = rt_task_init(0, 0, 0, 0);
   rt_task_make_periodic_relative_ns(ThreadTask, rtime_period, rtime_period);
   while (!btthread_done(this_thd)) {
      rt_task_wait_period();
      loop_start = rt_get_cpu_time_ns(); //th prof
      this_thd->actual_period = loop_start - last_loop; //th prof

      (*this_thd->function)(this_thd->data);

      loop_end = rt_get_cpu_time_ns(); //th prof
      this_thd->proc_time = loop_end - loop_start; //th prof
      last_loop = loop_start; //th prof
   }
   rt_task_delete(ThreadTask);
   pthread_exit(NULL);
}

/**
\internal 
\todo
  This function will set up periodic timing and then call the thd->function 
  once a period. It's meant to allow easy periodic threading.
 
\warning Untested!!!
 
*/
pthread_t* btperiodic_create(btthread *thd,int priority, double period, void *function,void *args)
{
   thd->period = period;
   thd->function = function;
   return btthread_create(thd,priority,btperiodic_proto,args);

}
