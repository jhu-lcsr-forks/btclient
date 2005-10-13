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
  init_btfilter_vn_diff(bts->velfilt,2,samplerate,cutoffHz);
  init_btfilter_vn_diff(bts->accfilt,2,samplerate,cutoffHz);
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

btreal D_Pt2Pl(vect_3 *norm,btgeom_plane *plane, vect_3 *point)
{
  btreal Dist;
  Dist = dot_v3(plane->normal,point) - plane->distance;
  set_v3(norm,plane->normal); //dist from point to plane surface
  return Dist;
}

int init_sp_btg( btgeom_sphere *sphere, vect_3 *pt1, vect_3 *pt2,int inside)
{
  
  sphere->center = new_v3();
  
  set_v3(sphere->center,pt1);
  sphere->radius = norm_v3(sub_v3(pt1,pt2));
  sphere->inside = inside;
}

btreal D_Pt2Sp(vect_3 *norm,btgeom_sphere *sp, vect_3 *pt)
{
  btreal Dist;
  btreal tmp1,tmp2;
  
  set_v3(norm,sub_v3(pt,sp->center)); //dist from point to plane surface
  Dist = norm_v3(norm) - sp->radius;
  set_v3(norm,unit_v3(norm));
  
  if (sp->inside){
    Dist = -1.0 * Dist;
    set_v3(norm,neg_v3(norm));
  }
  
  return Dist;
}

int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3,btreal thk,btreal dir1,btreal dir2,int inside)
{
  vect_3* swap;
  staticv3 sp[10];
  vect_3* tp[10];
  int cnt;
  
  for(cnt = 0;cnt < 10;cnt ++)
    tp[cnt] = init_staticv3(&sp[cnt]);
  
  init_pl_btg(&box->side[0],pt1,pt2,pt3); //Plane0
  //Mirror to other side
  set_v3(tp[0],add_v3(pt1,scale_v3(-1.0*thk,box->side[0].normal)));
  set_v3(tp[1],add_v3(pt2,scale_v3(-1.0*thk,box->side[0].normal)));
  set_v3(tp[2],add_v3(pt3,scale_v3(-1.0*thk,box->side[0].normal)));
  
  init_pl_btg(&box->side[1],tp[2],tp[1],tp[0]); //Plane1
  
  //Create section plane 1 for translation
  init_pl_btg(&box->side[2],pt1,pt2,tp[0]); 
  
  //Translate in positive direction
  set_v3(tp[3],add_v3(pt1,scale_v3(0.5*dir2,box->side[2].normal)));
  set_v3(tp[4],add_v3(pt2,scale_v3(0.5*dir2,box->side[2].normal)));
  set_v3(tp[5],add_v3(tp[0],scale_v3(0.5*dir2,box->side[2].normal)));

  init_pl_btg(&box->side[2],tp[3],tp[4],tp[5]);  //Plane2

  //Translate in negative direction
  set_v3(tp[3],add_v3(pt1,scale_v3(-0.5*dir2,box->side[2].normal)));
  set_v3(tp[4],add_v3(pt2,scale_v3(-0.5*dir2,box->side[2].normal)));
  set_v3(tp[5],add_v3(tp[0],scale_v3(-0.5*dir2,box->side[2].normal)));  

  init_pl_btg(&box->side[3],tp[5],tp[4],tp[3]); //Plane3
  //---
  //Calculate point = vect normal to p1,p2 line added to p1
  set_v3(tp[6],unit_v3(sub_v3(pt2,pt1))); 
  set_v3(tp[7],sub_v3(pt3,scale_v3(-1.0*dot_v3(sub_v3(pt3,pt1),tp[6]),tp[6])));
  
  //Create section plane 2 for translation
  init_pl_btg(&box->side[4],pt1,tp[7],tp[0]); 
  
  //Translate in positive direction
  set_v3(tp[3],add_v3(pt1,scale_v3(0.5*dir1,box->side[4].normal)));
  set_v3(tp[4],add_v3(tp[7],scale_v3(0.5*dir1,box->side[4].normal)));
  set_v3(tp[5],add_v3(tp[0],scale_v3(0.5*dir1,box->side[4].normal)));  
  
  init_pl_btg(&box->side[4],tp[3],tp[4],tp[5]); //Plane4
  //Translate in negative direction
  set_v3(tp[3],add_v3(pt1,scale_v3(-0.5*dir1,box->side[4].normal)));
  set_v3(tp[4],add_v3(tp[7],scale_v3(-0.5*dir1,box->side[4].normal)));
  set_v3(tp[5],add_v3(tp[0],scale_v3(-0.5*dir1,box->side[4].normal)));  

  init_pl_btg(&box->side[5],tp[5],tp[4],tp[3]); //Plane5
  
  if (inside){ //if inside, flip normals and distances
    for (cnt = 0; cnt < 6; cnt ++){
      set_v3(box->side[cnt].normal,neg_v3(box->side[cnt].normal));
      box->side[cnt].distance *= -1.0;
    }
  }
  
}

btreal D_Pt2Bx(vect_3 *dist,btgeom_box *bx, vect_3 *pt)
{
  btreal Dist[6],Dmax,Dmin;
  staticv3 sp[7];
  vect_3* tp[7];
  int cnt,idx = 0,closest_in = -1,closest_out = -1,active_sides = 0;
  
  for(cnt = 0;cnt < 2;cnt ++)
    tp[cnt] = init_staticv3(&sp[cnt]);
  Dmax = -10.0e10;
  Dmin = 10.0e10;
  
  for (cnt = 0; cnt < 6; cnt ++){
    Dist[cnt] = D_Pt2Pl(tp[cnt],&bx->side[cnt],pt);
    
    
    if ((Dist[cnt] < 0.0) && (Dist[cnt] > Dmax)) {
      if (!bx->inside) active_sides++;
      Dmax = Dist[cnt];
      closest_in = cnt;
    }
    else if ((Dist[cnt] > 0.0) && (Dist[cnt] < Dmin)) {
      if (bx->inside) active_sides++;
      Dmin =  Dist[cnt];
      closest_out = cnt;
    }
  }
  if ((!bx->inside) && (active_sides == 6)){
    set_v3(dist,tp[closest_in]);
    return Dist[closest_in];
  }
  else if ((!bx->inside)){
    set_v3(dist,tp[closest_out]);
    return Dist[closest_out];
  }
  else if ((bx->inside) && (active_sides == 6)){
    set_v3(dist,tp[closest_out]);
    return Dist[closest_out];
  }
  else{ //if ((bx->inside)){
    set_v3(dist,tp[closest_in]);
    return Dist[closest_in];
  }
    
}

void new_Seg_btg(btgeom_lineseg *seg,int size)
{
  seg->start = new_vn(size);
  seg->end = new_vn(size);
}
void set_Seg_btg(btgeom_lineseg *seg,vect_n *p1,vect_n *p2)
{
  if (p1 != NULL) set_vn(seg->start,p1);
  if (p2 != NULL) set_vn(seg->end,p2);
  set_vn(seg->unit,unit_vn(sub_vn(seg->end,seg->start)));
}

/** Distance between a point and line

  
*/
btreal D_Ln2Pt(btgeom_lineseg *seg,vect_n *pt)
{
  vect_n *ul,*np;
  
  np = sub_vn(pt,seg->start);
  return norm_vn(sub_vn(np,scale_vn(dot_vn(np,seg->unit),seg->unit)));
}





