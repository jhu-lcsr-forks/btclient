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

typedef struct bthaptic_object_struct{
  int type;
  int (*interact)(struct bthaptic_object_struct *obj, vect_n *wamTipLoc, vect_n *resultForce);
  void *data;
  int idx;
  //spatial info
  btreal dist;
  btreal K,B;
  btfilter *filt;
}bthaptic_object;

typedef struct {
  bthaptic_object **list;
  int num_objects;
  int max_objects;
}bthaptic_scene;
//Geometry

typedef struct {
  vect_3 *normal; //Normal of plane
  btreal distance; //Distance from origin to plane in direction of normal
}bthaptic_plane;

typedef struct {
  vect_3 center;
  double radius;
}bthaptic_sphere;

vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *wamTipLoc, vect_n *tipforce);
int addobject_bth(bthaptic_scene *bth,bthaptic_object *object);
void removeobject_bth(bthaptic_scene *bth,int index);
int initplane_btg( bthaptic_plane *inputPlane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
int initplaneobj_bth(bthaptic_object *inputObj, bthaptic_plane *inputPlane, btreal K, btreal B);
int eval_haptic_plane_bth(bthaptic_object *obj, vect_n *wamTipLoc, vect_n *resultForce);


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
