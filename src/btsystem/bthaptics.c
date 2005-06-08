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
  bth->max_objects = size;
  bth->list = (bthaptic_object**)malloc(size * sizeof(void*));
  if (bth->list == NULL){
    syslog(LOG_ERR,"Can't allocate memory for haptic scene");
    return -1;
  }
  bth->state = 0;
  return 0;
}

/**
 each object adds it's effect to the tipforce
*/
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
  static int i;
  if (bth->state){
  for(i=0; i < bth->num_objects; i++)
  {
    if(bth->list[i] != NULL)
    {
      (*(bth->list[i]->interact))(bth->list[i],pos,vel,acc,force);
    }
  }
  }
  return force;
}

int addobject_bth(bthaptic_scene *bth,bthaptic_object *object)
{
  int i;
  if(bth->num_objects >= bth->max_objects) return -1;
  bth->list[bth->num_objects] = object;
  object->idx = bth->num_objects;
  bth->num_objects++;
  return(bth->num_objects - 1);
}


void removeobject_bth(bthaptic_scene *bth,int index)
{
  int cnt;
  
  if ((index >= 0) && (index < bth->num_objects)){
    bth->list[index] = NULL;
    bth->num_objects--;
    for (cnt = index; cnt < bth->num_objects; cnt++) 
      bth->list[cnt] = bth->list[cnt + 1];
    bth->list[bth->num_objects] = NULL;
  }
}


int eval_geom_normal_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal depth;
  depth = (*(obj->collide))(obj,pos,(vect_n*)obj->Istate.pos);
  (*(obj->normalforce))(obj,depth,(vect_n*)obj->Istate.pos,vel,acc,force);
}

int eval_global_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
  bteffect_global *gp;
  gp = (bteffect_global *)obj->norm_eff;
  set_vn(force,add_vn(force,add_vn(scale_vn(-1.0*gp->B,vel),(vect_n*)gp->F)));

}

int init_global_bth(bthaptic_object *obj, bteffect_global *global,btreal B,vect_3 *F)
{
  
  global->F = new_v3();
  set_v3(global->F,F);
  global->B = B;
  init_state_btg(&(obj->Istate),0.002,30);
  obj->interact = eval_global_interact_bth;
  obj->norm_eff = global;
}

/**
\bug this uses a digital filter which assumes constant sample period and the sample rate and
cutoff freq are hard coded
*/
int init_normal_plane_bth(bthaptic_object *obj, btgeom_plane *plane, void*nfobj,void*nffunc)
{
  init_state_btg(&(obj->Istate),0.002,30);
  obj->interact = eval_geom_normal_interact_bth;
  obj->collide = plane_collide_bth;
  obj->geom = (void *) plane;
  obj->normalforce = nffunc;
  obj->norm_eff = nfobj;
}

btreal plane_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm)
{
  btreal res;
  res = D_Pt2Pl((vect_3*)norm,(btgeom_plane *)obj->geom,(vect_3*)pos);
  return res;
}

int init_normal_sphere_bth(bthaptic_object *obj, btgeom_sphere *sphere, void*nfobj,void*nffunc)
{
  init_state_btg(&(obj->Istate),0.002,30);
  obj->interact = eval_geom_normal_interact_bth;
  obj->collide = sphere_collide_bth;
  obj->geom = (void *) sphere;
  obj->normalforce = nffunc;
  obj->norm_eff = nfobj;
}

btreal sphere_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm)
{
  btreal res;
  res = D_Pt2Sp((vect_3*)norm,(btgeom_sphere *)obj->geom,(vect_3*)pos);
  //set_v3(obj->Istate.pos,scale_v3(res,(vect_3*)norm));
  return res;
}

void init_wall(bteffect_wall *wall,btreal K, btreal B)
{
  wall->K = K;
  wall->B = B;
}

int wall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal Dist,K,B;
  bteffect_wall *norm_eff;
  btreal WallStiff,WallDamp,Vel;
  
  WallStiff = 0.0;
  WallDamp = 0.0;
  
  norm_eff = (bteffect_wall*)obj->norm_eff;
  Vel = dot_v3((vect_3*)norm,(vect_3*)vel);
  
  if (depth < 0.0){
     WallStiff = -1.0*norm_eff->K*depth;
  }
  
  if (depth  < 0.0){
    if(Vel < 0.0) 
      WallDamp = -1.0*norm_eff->B*Vel;
  }
  set_v3((vect_3*)force,
             add_v3((vect_3*)force,
                     scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

int sheetwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal Dist,K,B;
  bteffect_wall *norm_eff;
  btreal WallStiff,WallDamp,Vel;
  
  WallStiff = 0.0;
  WallDamp = 0.0;
  
  norm_eff = (bteffect_wall*)obj->norm_eff;
  Vel = dot_v3((vect_3*)norm,(vect_3*)vel);
  
  WallStiff = -1.0*norm_eff->K*depth;
  
  WallDamp = -1.0*norm_eff->B*Vel;

  set_v3((vect_3*)force,
             add_v3((vect_3*)force,
                     scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}


void init_bulletproofwall(bteffect_bulletproofwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout)
{
  wall->K1 = K1;
  wall->K2 = K2;
  wall->K2offset = K2offset;
  wall->Bin = Bin;
  wall->Bout = Bout;
  wall->Boffset = Boffset;
}

int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal WallStiff,WallDamp,Vel;
  bteffect_bulletproofwall *norm_eff;

  
  WallStiff = 0.0;
  WallDamp = 0.0;

  norm_eff = (bteffect_bulletproofwall*)obj->norm_eff;
  Vel = dot_v3((vect_3*)norm,(vect_3*)vel);
  

  if (depth < 0.0){
     WallStiff = -1.0*norm_eff->K1*depth;
     if (depth < -1.0*norm_eff->K2offset)
       WallStiff += -1.0*norm_eff->K2*(depth+norm_eff->K2offset);
  }
  
  if (depth - norm_eff->Boffset < 0.0){
    if(Vel < 0.0) WallDamp = -1.0*norm_eff->Bin*Vel;
    else if (Vel > 0.0) WallDamp = -1.0*norm_eff->Bout*Vel;
  }
  set_v3((vect_3*)force,
             add_v3((vect_3*)force,
                     scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

void init_wickedwall(bteffect_wickedwall *wall,btreal K1, btreal Bin,btreal Bout,btreal Thk,btreal Boffset)
{
  wall->state = 1;
  wall->K1 = K1;
  wall->Bin = Bin;
  wall->Bout = Bout;
  wall->Thk = Thk;
  wall->Boffset = Boffset;
}

int wickedwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
  btreal WallStiff,WallDamp,Vel;
  bteffect_wickedwall *norm_eff;

  
  WallStiff = 0.0;
  WallDamp = 0.0;
 
  norm_eff = (bteffect_wickedwall*)obj->norm_eff;
  Vel = dot_v3((vect_3*)norm,(vect_3*)vel);
  
  if (norm_eff->state == 0){//outside
    if (depth < 0.0) norm_eff->state = 1;
  }
  if (norm_eff->state == 1){
    if (depth < -1.0*norm_eff->Thk) norm_eff->state = 2;
    else if (depth > 0.0) norm_eff->state = 0;
    else {
      WallStiff = -1.0*norm_eff->K1*depth;
    }
  }
  if (norm_eff->state == 2){
    if (depth > 0.0) norm_eff->state = 0;
  }
  
  if ((depth - norm_eff->Boffset < 0.0) && (norm_eff->state != 2)){
    if(Vel < 0.0) WallDamp = -1.0*norm_eff->Bin*Vel;
    else if (Vel > 0.0) WallDamp = -1.0*norm_eff->Bout*Vel;
  }
  set_v3((vect_3*)force,
             add_v3((vect_3*)force,
                     scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}
