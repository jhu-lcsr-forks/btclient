/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btos.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......Mar 28, 2006
 *                                                                      *
 *  ******************************************************************  */
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
    syslog(LOG_ERR,"bt ERROR: you tried to access a null pointer in %s",str);
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
#ifdef NULL_PTR_GUARD
  if(BTPTR_OK(*ptr,"btfree"))
#endif
  free(*ptr);
  *ptr = NULL;
}



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


