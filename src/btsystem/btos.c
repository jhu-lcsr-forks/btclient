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
    return -1;
  }
  else 
    return 0;
}


BTINLINE void * xmalloc(size_t size)
{
 void* vmem;

  //allocate mem for vector,return vector, and return structure
  if ((vmem = malloc(size)) == NULL) 
  {
    syslog(LOG_ERR,"xMalloc: memory allocation failed, size %d",size);
  }
  return vmem;
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


