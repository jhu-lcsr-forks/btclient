/*======================================================================*
 *  Module .............libbt
 *  File ...............btsystem_control_loop.h
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
#ifndef _BTSYSTEM_CONTROL_LOOP_H
#define _BTSYSTEM_CONTROL_LOOP_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
#include "btsystem.h"

/** Defines a set of variables needed inside the control loop */
typedef struct 
{
  double sample_period; 
  SimpleCtl *sc; /*!< A pointer to an array of SimpleCtl[num_act] */
  int num_act; /*!< The number of actuators reported by btsystem*/
  actuator_struct *act; /*!< The pointer the the array of actuators */
}SCcontrol_thd_parms;

void BTsystemControlThread(void *data);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTSYSTEM_CONTROL_LOOP_H */

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
