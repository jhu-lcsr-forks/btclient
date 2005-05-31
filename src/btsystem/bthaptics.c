/*======================================================================*
 *  Module .............libbt
 *  File ...............bthaptics.c
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
#include <stdlib.h>
#include <stdio.h>
#include "btmath.h"
#include "bthaptics.h"
#include <syslog.h>

int new_bthaptic_scene(bthaptic_scene *bth, int size)
{
  bth->num_objects = 0;
  bth->max_objects = 0;
  bth->list = (bthaptic_object**)malloc(size * sizeof(void*));
  if (bth->list == NULL){
    syslog(LOG_ERR,"Can't allocate memory for haptic scene");
    return -1;
  }
  return 0;
}

/**
 each object adds it's effect to the tipforce
*/
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
  static int i;

  for(i=0; i < bth->num_objects; i++)
  {
    if(bth->list[i] != NULL)
    {
      (*(bth->list[i]->interact))(bth->list[i],pos,vel,acc,force);
    }

  }
  return tipforce;
}

int addobject_bth(bthaptic_scene *bth,bthaptic_object *object)
{
  int i;

  for(i=0;i<bth->num_objects;i++)
  {
    if((i < bth->num_objects) && (bth->list[i] == NULL))
    {
      bth->list[i] = object;
      return(i);
    }
  }
  bth->list[bth->num_objects] = object;
  return(bth->num_objects++);
}


void removeobject_bth(bthaptic_scene *bth,int index)
{
  bth->list[index] = NULL;
}



/**
\bug this uses a digital filter which assumes constant sample period and the sample rate and
cutoff freq are hard coded
*/
int init_normal_plane_bth(bthaptic_object *obj, btgeom_plane *plane, void*nfobj,void*nffunc)
{
  init_state_btg(&(obj->Istate),0.002,30);
  obj->interact = eval_geom_normal_interact_bth;
  obj->collide = D_Pt2Pl;
  obj->geom = (void *) plane;
  obj->normalforce = nffunc;
  obj->norm_eff = nfobj;
}

int eval_geom_normal_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
  (*(obj->collide))(obj,pos,obj->Istate.pos);
  //eval_state_btg(&(obj->Istate),obj->Istate.pos);
  (*(obj->normalforce))(obj,obj->Istate.pos,vel,acc,force);
}

void init_wall(bteffect_wall *wall,btreal K, btreal B)
{
  wall->K = K;
  wall->B = B;
}

int wall_nf(struct bthaptic_object_struct *obj, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal Dist,K,B;
  bteffect_wall *norm_eff;
  staticv3 udistmem;
  vect_3* udist;
  
  udist = init_staticv3(&udistmem);
  
  Dist = norm_v3((vect_3*)dist);
  set_v3(udist,unit_v3((vect_3*)dist));
  
  norm_eff = (bteffect_wall*)obj->norm_eff;
  if(Dist < 0.0) //we are on the "negative" side of the plane
    {
      set_v3((vect_3*)force,
             add_v3((vect_3*)force,
                     scale_v3(norm_eff->K*Dist + norm_eff->B*dot_v3(udist,(vect_3*)vel),udist)));

    }
}

int wickedwall_nf(struct bthaptic_object_struct *obj, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal Dist,K,B;
  bteffect_wickedwall *norm_eff;
  staticv3 udistmem;
  vect_3* udist;
  
  udist = init_staticv3(&udistmem);
  Dist = norm_v3((vect_3*)dist);
  set_v3(udist,unit_v3((vect_3*)dist));
  norm_eff = (bteffect_wickedwall*)obj->norm_eff;

  if (norm_eff->state = 0){//outside
    if (Dist < 0.0) norm_eff->state = 1;
  }
  if (norm_eff->state == 1){
    if (Dist < thisPlane->thk) state = 2;
    else if (Dist > 0.0) state = 0;
    else {
      WallStiff = -1.0*obj->K*Dist;
    }
  }
  if (state == 2){
    if (Dist > 0.0) state = 0;
  }
  
  if (Dist + obj->Boff < 0.0){
    if(Vel < 0.0) WallDamp = -1.0 * obj->Bin*Vel;
    else if (Vel > 0.0) WallDamp = obj->Bout*Vel;
  }
  set_v3((vect_3*)resultForce,
             add_v3((vect_3*)resultForce,
                     scale_v3(WallStiff + WallDamp,thisPlane->normal)));
}
