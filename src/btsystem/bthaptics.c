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

int new_bthaptic_scene(bthaptic_scene *bth, int size)
{
  bth->num_objects = 0;
  bth->max_objects = 0;
  bth->list = (bthaptic_object*)malloc(size * sizeof(void*));
  if (bth->list == NULL){
    syslog(LOG_ERR,"Can't allocate memory for haptic scene");
    return -1;
  }
  return 0;
}

/**
 each object adds it's effect to the tipforce
*/
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *wamTipLoc, vect_n *tipforce)
{
  static int i;

  for(i=0; i < bth->num_objects; i++)
  {
    if(bth->list[i] != NULL)
    {
      (*(bth->list[i]->interact))(bth->list[i],wamTipLoc,tipforce);
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

int initplane_btg( bthaptic_plane *inputPlane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3)
{
  vect_3 * rel2;
  vect_3 * rel3;

  rel2 = new_v3();
  rel3 = new_v3();
  
  set_v3(rel2,sub_v3(pt2,pt1));
  set_v3(rel3,sub_v3(pt3,pt2));
  set_v3(inputPlane->normal,unit_v3(cross_v3(rel2,rel3)));
  inputPlane->distance = dot_v3(inputPlane->normal, pt1);
  
}
/**
\bug this uses a digital filter which assumes constant sample period and the sample rate and
cutoff freq are hard coded
*/
int initplaneobj_bth(bthaptic_object *inputObj, bthaptic_plane *inputPlane, btreal K, btreal B)
{
  
  inputObj->interact = eval_haptic_plane_bth;
  inputObj->data = (void *) inputPlane;
  inputObj->filt = new_btfilter(5);
  init_btfilter_butterworth_diff(inputObj->filt,.002,20);
  inputObj->K = K;
  inputObj->B = B;
}

int eval_haptic_plane_bth(bthaptic_object *obj, vect_n *wamTipLoc, vect_n *resultForce)
{

  double Dist;
  btreal Vel;
  bthaptic_plane *thisPlane;
  
  thisPlane = (bthaptic_plane *) (obj->data);
  
  Dist = dot_v3(thisPlane->normal,(vect_3*)wamTipLoc) - thisPlane->distance;
  Vel = eval_btfilter(obj->filt,Dist);
  
  if(Dist < 0.0) //we are on the "negative" side of the plane
    {
      /* push towards base, opposite normal (is this right?) */
      set_v3((vect_3*)resultForce,
             add_v3((vect_3*)resultForce,
                     scale_v3(obj->K*Dist + obj->B*Vel,thisPlane->normal)));

    }
}
