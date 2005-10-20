/*======================================================================*
 *  Module .............libbt
 *  File ...............bthaptics.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......29 Apr 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation (version 2).
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/
/** \file btgeometry.h
    \brief Analytic geometry objects

 The objects in this file are primarily used by bthaptics.c

 */
#ifndef _BTGEOMETRY_H
#define _BTGEOMETRY_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
#include "btmath.h"

//Geometry
typedef struct {
  vect_3 *pos,*vel,*acc; //Normal of plane
  btfilter_vn *velfilt,*accfilt;
}btgeom_state;

void init_state_btg(btgeom_state *bts,btreal samplerate,btreal cutoffHz);
void eval_state_btg(btgeom_state *bts,vect_3* pos);


typedef struct {
  vect_n *start,*end; //Normal of plane
  vect_n *unit;
}btgeom_lineseg;

void new_Seg_btg(btgeom_lineseg *seg,int size);
void set_Seg_btg(btgeom_lineseg *seg,vect_n *p1,vect_n *p2);
btreal D_Ln2Pt(btgeom_lineseg *seg,vect_n *pt); 
btreal I_Ln2Ln(btgeom_lineseg *seg,vect_n *pt); 
btreal D_Seg2Pt(btgeom_lineseg *seg,vect_n *pt); //Distance
btreal Pt_Seg2Pt(vect_n *loc,btgeom_lineseg *seg,vect_n *pt); //Distance, return point on line
btreal D_Seg2Seg(btgeom_lineseg *seg1,btgeom_lineseg *seg2); //Distance
/**


*/
typedef struct {
  vect_3 *normal; //Normal of plane
  btreal distance; //Distance from origin to plane in direction of normal
}btgeom_plane;

btgeom_plane* new_pl_btg();
int init_pl_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
int init_3point_plane_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
int init_pointnormal_plane_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2);
btreal D_Pt2Pl(vect_3 *norm,btgeom_plane *plane, vect_3 *point);

//vect_3* I_Li2Pl(

typedef struct {
  vect_3 *center;
  double radius;
  int inside;
}btgeom_sphere;

int init_sp_btg( btgeom_sphere *sphere, vect_3 *pt1, vect_3 *pt2,int inside);
btreal D_Pt2Sp(vect_3 *norm,btgeom_sphere *sp, vect_3 *pt);

typedef struct {
  btgeom_plane side[6];
  int inside;
  vect_3 *center; //Center and size of box
  matr_3 *normals; //orientation of box
}btgeom_box;
/**
Define plane with 3 points
thk = dim of box in dir of plane normal
dir1 = dim of box in dir of pt1 to pt2
dir2 = dim of box in dir of pt3 normal to pt1,pt2 line
*/
int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3,btreal thk,btreal dir1,btreal dir2,int inside);
btreal D_Pt2Bx(vect_3 *dist,btgeom_box *bx, vect_3 *pt);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTHAPTICS_H */



