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
#include "btgeometry.h"
#include <syslog.h>

void init_state_btg(btgeom_state *bts,btreal samplerate,btreal cutoffHz)
{
  bts->pos = new_v3();
  bts->vel = new_v3();
  bts->acc = new_v3();
  bts->velfilt = new_btfilter_vn(5,3);
  bts->accfilt = new_btfilter_vn(5,3);
  init_btfilter_vn_butterworth_diff(bts->velfilt,2,samplerate,cutoffHz);
  init_btfilter_vn_butterworth_diff(bts->accfilt,2,samplerate,cutoffHz);
}

void eval_state_btg(btgeom_state *bts,vect_3* pos)
{
  set_v3(bts->pos,pos);
  set_v3(bts->vel,(vect_3*)eval_btfilter_vn(bts->velfilt, (vect_n*)bts->pos));
  set_v3(bts->acc,(vect_3*)eval_btfilter_vn(bts->accfilt, (vect_n*)bts->vel));
}

int init_pl_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3)
{
  vect_3 * rel2;
  vect_3 * rel3;

  rel2 = new_v3();
  rel3 = new_v3();
  plane->normal = new_v3();
  
  set_v3(rel2,sub_v3(pt2,pt1));
  set_v3(rel3,sub_v3(pt3,pt2));
  set_v3(plane->normal,unit_v3(cross_v3(rel2,rel3)));
  plane->distance = dot_v3(plane->normal, pt1);
  
}

btreal D_Pt2Pl(vect_3 *dist,btgeom_plane *plane, vect_3 *point)
{
  btreal Dist;
  Dist = dot_v3(plane->normal,point) - plane->distance;
  set_v3(dist,scale_v3(Dist,plane->normal));
  return Dist;
}
