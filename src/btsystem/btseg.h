#ifndef _BTSEG_H
#define _BTSEG_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

#include "btmath.h"

/**
A parabolic function.
Starts at time 0, ends at time tf;
As time approaches tf, s & sp approach sf & spf;
sppf is constant;

*/
typedef struct parabolic_s{
  btreal sf,spf,sppf,tf;
  unsigned int type; //0 = normal, 1 = pause here, 2 = looping
  struct parabolic_s *next;
}parabolic;

void dump_pb(parabolic *p,FILE *out);
btreal sp_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pbl(parabolic **pin, btreal t);
btreal boundary_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf);
btreal blend_pb(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t);
btreal s0sfspftf_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal spf,btreal tf);

/** Parabolic segment trajectory controller Ideas:
Circular buffer.
Adding a segment to an empty list when it is "on" starts playing that segment using
the present time as t0.
When the last segment of the list is reached it pauses waiting for the next one.

You can turn it "off" to pre fill the buffer
You can loop
You can pause

Time starts at 0.0 from reset.
*/
/** Array of parabolic segments


*/
typedef struct pararray_s{
  parabolic *pb;
  parabolic *iter;
  double t; //time state variable
  int cnt,max; //Length of array,Max available memory
  int state;
}pararray;

pararray* new_pa(int max);
void destroy_pa(pararray* pa);

void clear_pa(pararray* pa);
btreal add_bseg_pa(pararray* pa,parabolic* p);

btreal  reset_pa(pararray* pa);
btreal eval_pa(pararray* pa,btreal dt);

/** A group of parabolic lists.
Inputs and outputs are vect_n's
*/
typedef struct pararray_vns{
  pararray **pa;
  int elements; //Length of array,Max available memory
  vect_n* result;
  int state;
}pararray_vn;

pararray_vn* new_pavn(int max,int elements);
void destroy_pavn(pararray_vn* pavn);
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

