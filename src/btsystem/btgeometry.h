/*======================================================================*
 *  Module .............libbt
 *  File ...............bthaptics.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......29 Apr 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

#ifndef _BTHAPTICS_H
#define _BTHAPTICS_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
#include "btmath.h"

//Geometry
typedef struct {
  vect_3 *pos,*vel,*acc; //Normal of plane
  btfilter_vn velfilt,accfilt;
}btgeom_state;

init_state_btg(btgeom_state *bts,btreal samplerate,btreal cutoffHz);
eval_state_btg(btgeom_state *bts,vect_3* pos);


typedef struct {
  vect_3 *start,*end; //Normal of plane
}btgeom_lineseg;


typedef struct {
  vect_3 *normal; //Normal of plane
  btreal distance; //Distance from origin to plane in direction of normal
}btgeom_plane;

int init_pl_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
btreal D_Pt2Pl(vect_3 *dist,btgeom_plane *plane, vect_3 *point);
//vect_3* I_Li2Pl(

typedef struct {
  vect_3 center;
  double radius;
}btgeom_sphere;

int init_sp_btg(btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2);
btreal D_Pt2Sp(vect_3 *dist,btgeom_sphere *sp, vect_3 *pt);

typedef struct {
  vect_3 *center,*outside,*inside; //Center and size of box
  matr_3 *orient; //orientation of box
}btgeom_box;

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTHAPTICS_H */

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
