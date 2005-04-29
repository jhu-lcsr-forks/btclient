/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btpath.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......30 Mar 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include "btpath.h"
/**
  Paths define geometry in space.
  Trajectories define a location and velocity as a function of time.
  Paths are stored as functions of pathlength
*/  
  


/**
piecewize linear path
piecewize linear trajectory (s(t) - arc pos as a function of time)



*/

int init_pwl(btpath_pwl *pth, int vect_size,int rows)
{
  void* vmem;

  int cnt;
  //allocate mem for vector,return vector, and return structure
  if ((vmem = malloc(rows*sizeof(btreal))) == NULL) 
  {
    syslog(LOG_ERR,"btmath: init_pwl memory allocation failed, size %d",rows);
    return -1;
  }
  
  pth->s = (btreal*)vmem;
  pth->vr = new_vr(vect_size,rows);
  pth->proxy = new_vn(vect_size);
  pth->tmp1 = new_vn(vect_size);
  pth->tmp2 = new_vn(vect_size);
  pth->proxy_s = 0.0;
  pth->segment = 0;
  pth->startnode = 0;
  pth->endnode = 0;
}

/** Free the memory allocated during initialization of a btpath_pwl structure
*/
void free_pwl(btpath_pwl *pth)
{
  destroy_vr(pth->vr);
  free_vn(pth->proxy);
  free_vn(pth->tmp1);
  free_vn(pth->tmp2);
  free(pth->s);
}
/** Break a curve paramaterized by t (usually time) into x(s) and s(t) | s = arclength. 


*/
void new_param_by_arclen_pwl(btpath_pwl *pth, btpath_pwl *crv2)
{
  btreal t,s;
  vect_n *a;
  int cnt;
  
  a = new_vn(1);
  
  init_pwl(crv2,1,size_vr(pth->vr)); //allocate vectray with vector size 1
  
  //dont assume the parameterization starts at 0.0
  t = getnodes_pwl(pth,0);
  setval_vn(a,0,0.0);
  add_point_pwl(crv2, a, t);
  
  s = 0.0; //but arclength does start at 0.0
  
  for(cnt = 1;cnt <= endof_vr(pth->vr);cnt++){ 
    t = getnodes_pwl(pth,cnt);
    s += norm_vn(sub_vn(idxa_pwl(pth,cnt),idxb_pwl(pth,cnt-1)));
    pth->s[cnt] = s;
    
    setval_vn(a,0,s);
    add_point_pwl(crv2, a, t);
  }
  
}


vect_n * idxa_pwl(btpath_pwl *pth,int idx) //internal
{ return getvn_vr(pth->tmp1,pth->vr,idx);}

vect_n * idxb_pwl(btpath_pwl *pth,int idx) //internal: 
{ return getvn_vr(pth->tmp2,pth->vr,idx);}
/** Adds a point to a path and computes the arclength which it uses as the
parameter


*/
int add_arclen_point_pwl(btpath_pwl *pth, vect_n *p)
{
  int idx;
  append_vr(pth->vr,p);
  idx = endof_vr(pth->vr);
  if (idx==0)
    pth->s[0] = 0.0;
  else
    pth->s[idx] = pth->s[idx-1] + norm_vn(sub_vn(idxa_pwl(pth,idx),idxb_pwl(pth,idx-1)));
  
  return idx;
}
/** Adds a point to a path and computes the arclength


*/
int add_point_pwl(btpath_pwl *pth, vect_n *p, btreal s)
{
  int idx;
  idx = endof_vr(pth->vr);
  if (pth->s[idx] > s) return -1; //time points must be added to the path as monotonically increasing
  append_vr(pth->vr,p);
  idx = endof_vr(pth->vr);
  if (idx==0)
    pth->s[0] = 0.0;
  else
    pth->s[idx] = s;
  
  return idx;
}
int clear_pwl(btpath_pwl *pth)
{
  int idx;
  
  clear_vr(pth->vr);
  set_vn(pth->proxy,idx_vr(pth->vr,0));
  pth->proxy_s = pth->s[0];
  pth->segment = 0;
  
}
/** Find the segment that contains arclength s
\param s if s is negative we look for the segment that contains (smax - s)
\return the segment number that contains s, starting with 1
*/
int get_segment_pwl(btpath_pwl *pth,btreal s)
{
  int cnt, end;
  btreal send;
  
  end = endof_vr(pth->vr);
  if (end < 0) return -2;
  
  if (fabs(s) > pth->s[end]) return end;
  
  if (s > 0.0){
   for (cnt = 1; cnt < end; cnt ++){
      if (s < pth->s[cnt]) return cnt;
   }
   return end;
  }
  else {
    send = pth->s[end] - s;
    for (cnt = end; cnt < 1; cnt --){
      if (send > pth->s[cnt]) return cnt;
    }
    return 1;
   }
  return -1;
}
btreal getnodes_pwl(btpath_pwl *pth,int idx)
{
  return pth->s[idx];
  
}
/** Find the point location at arc-distance s into the curve

*/
vect_n * getval_pwl(btpath_pwl *pth, btreal s)
{
  int idx;
  
  idx = get_segment_pwl(pth,s);
  if (idx <= 0){ //If there are no points in the path, return the start of the path
    set_vn(pth->proxy,idx_vr(pth->vr,0));
    pth->proxy_s = pth->s[0];
    pth->segment = 0;
    return pth->proxy;
  }

  set_vn(pth->proxy,interp_vn(idxa_pwl(pth,idx-1),idxb_pwl(pth,idx),s - pth->s[idx-1]));
  
  pth->segment = idx;
  pth->proxy_s = s;  
  
  return pth->proxy;
}

/** Increment the proxy_s and return point at the new location


*/
vect_n* ds_pwl(btpath_pwl *pth, btreal ds)
{
  int idx, end;
  btreal s;
 
  end = endof_vr(pth->vr);
  if (end < 0){ //If there are no points in the path, return the start of the path
    set_vn(pth->proxy,idx_vr(pth->vr,0));
    pth->proxy_s = pth->s[0];
    pth->segment = 0;
    return pth->proxy;
  }
  
  s = ds + pth->proxy_s;
  idx = pth->segment;
  if (s > pth->s[idx]){
    if (idx >= end){
      set_vn(pth->proxy,idx_vr(pth->vr,end));
      pth->proxy_s = pth->s[end];
      pth->segment = end;
      return pth->proxy;
    }
    else {
      pth->segment += 1;
      idx = pth->segment;
    }
  }
  
  set_vn(pth->proxy,interp_vn(idxa_pwl(pth,idx),idxb_pwl(pth,idx-1),s - pth->s[idx-1]));
  pth->proxy_s = s;  
  
  return pth->proxy;

}

vect_n* dsinit_pwl(btpath_pwl *pth, btreal s)
{
  return getval_pwl(pth,s);
}

btreal arclength_pwl(btpath_pwl *pth)
{ int idx;
  idx = endof_vr(pth->vr);
  if (idx < 0){ //If there are no points in the path, return the start of the path
     return 0.0;
  }
  return pth->s[idx];
  
}
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

