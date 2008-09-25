/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btseg.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......Oct 20, 2005
 *                                                                      *
 *  ******************************************************************  *
 *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *   
 *
 *  REVISION HISTORY:
 *
 *======================================================================*/
 
#ifndef _BTSEG_H
#define _BTSEG_H

#include "btmath.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */



/** A parabolic function object.

This object stores individual segments of the piecewise 
parabolic function (pararray) and implements the mathematics for these segments.

Given final values for position, velocity, acceleration and time (sf, spf, sppf,
and tf respectively) calculate position and/or velocity for any point in time 
prior to tf.

As time approaches tf, s & sp approach sf & spf;
sppf is constant;

Symbols:
s: length (dependant) parameter
t: time (independent) parameter
0: initial value (t0 = initial time)
f: final value
p: prime (d/dt) (velocity)
pp: (d/dt^2) (acceleration)

f_of_t: f(t) (s_of_t=>s(t))


*/
typedef struct parabolic_s{
  btreal sf,spf,sppf,tf;
  unsigned int type; //0 = normal, 1 = pause here, 2 = looping
  struct parabolic_s *next;
}parabolic;

void dump_pb(parabolic *p,FILE *out);

//Evaluation functions
btreal sp_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pbl(parabolic **pin, btreal t);

//Definition & Initial Calcs from various combinations of boundary conditions.
btreal boundary_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf);
btreal blend_pb(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t);
btreal s0sfspftf_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal spf,btreal tf);

/** Parabolic segment trajectory controller Design Ideas: (not necessarily implemented)
Circular buffer.
Adding a segment to an empty list when it is "on" starts playing that segment using
the present time as t0.
When the last segment of the list is reached it pauses waiting for the next one.

You can turn it "off" to pre fill the buffer
You can loop
You can pause

Time starts at 0.0 from reset.
*/

/** One dimensional piecewise parabolic function.
This object stores a piecewise parabolic function (in one dimension), evaluates the
function relative to independent variable t, and keeps state information for t.

reset_pa() will initialize t to 0.
eval_pa() will increment t by dt until t > tf of the final segment after which it 
will return sf of the final segment. 

Symbols:
tF: tf of the last segment in the list.
*/
typedef struct pararray_s{
  parabolic *pb;
  parabolic *iter;
  double t; //time state variable
  int cnt,max; //Length of array,Max available memory
  int state;
}pararray;

pararray* new_pa(int max);
void destroy_pa(pararray** pa);

void clear_pa(pararray* pa);
btreal add_bseg_pa(pararray* pa,parabolic* p);

btreal  reset_pa(pararray* pa);
btreal eval_pa(pararray* pa,btreal dt);

/** Multi-dimensional piecewise parabolic functions.
A multi-dimensional version of pararray.

Inputs and outputs are vect_n's.

getstate_pavn() returns BTTRAJ_RUN until all t > tF for all dimensions.
*/
typedef struct pararray_vns{
  pararray **pa;
  int elements; //Length of array,Max available memory
  vect_n* result;
  int state;
}pararray_vn;

pararray_vn* new_pavn(int max,int elements);
void destroy_pavn(pararray_vn** pavn);
void clear_pavn(pararray_vn* pavn);
vect_n*  add_bseg_pavn(pararray_vn* pavn,vect_n* t0,vect_n* s0,vect_n* sf,vect_n* sp0,vect_n* spf);
vect_n*  reset_pavn(pararray_vn* pavn);
vect_n* eval_pavn(pararray_vn* pavn,btreal dt);
int getstate_pavn(pararray_vn* pavn);
pararray_vn* vr2pararray(vectray* vr,btreal acc);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTSEG_H */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
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
 
