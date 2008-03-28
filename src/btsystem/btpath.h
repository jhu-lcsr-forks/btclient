/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btpath.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......30 Mar 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  '051107 TH Minimal documentation in place.                          *
 *======================================================================*/
/** \file btpath.h
\brief Piecewize space curve geometry

The btpath_pwl object is a primitive meant for use either in trajectories or
haptic curve geometry. It's main property is that it paramaterizes a curve
by arc-length.

Presently this object is only used by the #btstatecontrol point-to-point move
code.

*/
#ifndef _BTPATH_H
#define _BTPATH_H


#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
 
#include "btmath.h"
/** Piecewize Linear Path

btpath_pwl is for N dimensional piecewize linear curves

 The parameter s is assumed to be monotonically increasing.

  Paths define geometry in space.
  Trajectories define a location and velocity as a function of time.
  Paths are stored as functions of pathlength


*/
typedef struct {
  vectray *vr; //size in vect_n size 
  btreal *s; //for each point we record the total arc length
  vect_n* proxy; //current point
  vect_n *tmp1,*tmp2; //current point
  btreal proxy_s;
  int segment;
  int startnode,endnode;
  int type; //!< 0 = cartesial, 1 = SE(3)
  int param; //0 = arclength, 1 = time
  
}btpath_pwl;

// Init / Destroy
int init_pwl(btpath_pwl *pth, int vect_size,int rows);
int init_pwl_from_vectray(btpath_pwl *pth,vectray *vr);

btpath_pwl * new_pwl();
void free_pwl(btpath_pwl *pth);
void new_param_by_arclen_pwl(btpath_pwl *pth, btpath_pwl *crv2); //creates a second curve of arclength vs time

// PWL Curve Data Manipulation
int add_arclen_point_pwl(btpath_pwl *pth, vect_n *p); //Add a point to the arc, parameterized by arclength, returns index to the point
int clear_pwl(btpath_pwl *pth); //Erase the present path
int add_point_pwl(btpath_pwl *pth, vect_n *p, btreal s); //Add a point to the arc, parameterized by time, returns index to the point
//void rem_point(btpath_pwl *pth, int idx); //Add a point to the arc
int add_vectray_pwl(btpath_pwl *pth, vectray *vr);//use the first column for the parameter
int add_arclen_vectray_pwl(btpath_pwl *pth, vectray *vr);

// PWL Curve Usage
vect_n * getval_pwl(btpath_pwl *pth, btreal s); //Given arclength , find location, set the proxy to it
vect_n* dsinit_pwl(btpath_pwl *pth, btreal s);
vect_n* ds_pwl(btpath_pwl *pth, btreal ds); //Shift our proxy by ds and return the position.

btreal getnodes_pwl(btpath_pwl *pth,int idx); //return arclength at node[idx]
btreal arclength_pwl(btpath_pwl *pth);

//int read_cvs_pwl(btpath_pwl *pth,FILE *infile);
//int write_cvs_pwl(btpath_pwl *pth,FILE *outfile);

/** Curve Trajectory Controller

  Curve control
    Reset curve to s = x;
    eval(ds)
    deval(ds)
    ddeval(ds)
    get_length
    loop
    
  Trajectory Ctl
    Reset to t = x
    eval(t)
    eval(dt)

*/








#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTMATH_H */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
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
 
