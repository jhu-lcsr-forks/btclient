/*======================================================================*
 *  Module .............libbt
 *  File ...............btsystem_control_loop.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......16 Mar 2003
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 *======================================================================*/

#include <semaphore.h>

#include "btsystem.h"
#include "btjointcontrol.h"
#include "btsystem_control_loop.h"

extern sem_t timer_semaphore;
extern int shutdown_threads;

/** A canned controller for a set of pucks

*/
void BTsystemControlThread(void *data)
{
  int cnt;
  double sample_period;
  SimpleCtl *sc;
  actuator_struct *act;
  int num_actuators;

  sample_period = ((SCcontrol_thd_parms *)data)->sample_period;
  sc = ((SCcontrol_thd_parms *)data)->sc;
  act = ((SCcontrol_thd_parms *)data)->act;
  num_actuators = ((SCcontrol_thd_parms *)data)->num_act;

  while (!shutdown_threads)
  {
    sem_wait(&timer_semaphore);

    GetPositions(); //    getpos
    for (cnt = 0; cnt < num_actuators; cnt++) //    calc PID
    {
      act[cnt].torque = SCevaluate(&(sc[cnt]), act[cnt].angle, sample_period);
      
    }
    SetTorques(); //    set torques
  }
  pthread_exit(NULL);
}

/*======================================================================*
 *                                                                      *
 *             Copyright (c) 2003 Barrett Technology, Inc.              *
 *                        139 Main Street                               *
 *                       Kendall/MIT Square                             *
 *                  Cambridge, MA  02142-1528  USA                      *
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
 *  ******************************************************************  *
 *                                                                      
 * CVS info: $Id$
 * CVS automatic log (prune as desired):
 * $Log$
 *
 *======================================================================*/
