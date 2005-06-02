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
#include "btgeometry.h"

typedef struct bthaptic_object_struct{
  int type;
  int (*interact)(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
  btreal (*collide)(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *dist);
  int (*normalforce)(struct bthaptic_object_struct *obj, btreal depth, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force);
  void *geom,*norm_eff,*tang_eff;
  btgeom_state Istate;
  int idx;
}bthaptic_object;

typedef struct {
  bthaptic_object **list;
  int num_objects;
  int max_objects;
  int state;
}bthaptic_scene;
//Geometry
int new_bthaptic_scene(bthaptic_scene *bth, int size);
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
int addobject_bth(bthaptic_scene *bth,bthaptic_object *object);
void removeobject_bth(bthaptic_scene *bth,int index);

int eval_geom_normal_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);

int init_normal_plane_bth(bthaptic_object *obj, btgeom_plane *plane, void *nfobj,void *nffunc);
btreal plane_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *dist);

int init_normal_sphere_bth(bthaptic_object *obj, btgeom_sphere *sphere, void*nfobj,void*nffunc);
btreal sphere_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *dist);


typedef struct { 
  btreal K,B;
}bteffect_wall;

void init_wall(bteffect_wall *wall,btreal K, btreal B);
int wall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force);
 
typedef struct { 
  btreal Boffset; //Relative start of damping
  btreal K2; //second stage spring constant
  btreal K2offset; //Distance into wall second spring constant starts
  btreal K1; //first stage spring constant
  btreal Bin; //damping as you move into the wall
  btreal Bout; //damping as you move out of the wall
}bteffect_bulletproofwall;
void init_bulletproofwall(bteffect_bulletproofwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout);
int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force);

typedef struct { 
  int state; //outside, inside, brokethru
  btreal Boffset; //Relative start of damping
  btreal K1; //first stage spring constant
  btreal Bin; //damping as you move into the wall
  btreal Bout; //damping as you move out of the wall
  btreal Thk;
}bteffect_wickedwall;
void init_wickedwall(bteffect_wickedwall *wall,btreal K1, btreal Bin,btreal Bout,btreal Thk,btreal Boffset);
int wickedwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force);

typedef struct { 
  btreal B;
  vect_3* F; //bias force
}bteffect_global;
int eval_global_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
int init_global_bth(bthaptic_object *obj, bteffect_global *global,btreal B,vect_3 *F);


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
