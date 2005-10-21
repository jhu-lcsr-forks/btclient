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

btreal sp_of_t_para(parabolic *p, btreal t);
btreal s_of_t_para(parabolic *p, btreal t);
btreal s_of_t_paral(parabolic **pin, btreal t);
btreal boundary_para(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf);
btreal blend_para(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t);
void velocity_para(parabolic *p,btreal s0,btreal sf,btreal t0,btreal tf); //Constant velocity
void velocity2_para(parabolic *p,btreal s0,btreal v0,btreal t0,btreal tf); //Constant velocity
//void acc_para(parabolic *p,btreal s0,btreal sf,btreal spf,btreal a,btreal t0); //trapezoidal trajectory

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
  parabolic *pa;
  parabolic *iter;
  int cnt,max; //Length of array,Max available memory
}pararray;

pararray* new_pa(int max);


void clear_pa(pararray* pa);
btreal add_bseg_pa(pararray* pa,btreal t0,btreal s0,btreal sf,btreal sp0,btreal spf);


btreal  reset_pa(pararray* pa);
btreal eval_pa(pararray* pa,btreal t);


typedef struct pararray_vns{
  pararray **pa;
  int elements; //Length of array,Max available memory
  vect_n* result;
}pararray_vn;

pararray_vn* new_pavn(int max,int elements);
void clear_pavn(pararray* pavn);
vect_n*  add_bseg_pavn(pararray* pavn,vect_n* t0,vect_n* s0,vect_n* sf,vect_n* sp0,vect_n* spf);
vect_n*  reset_pavn(pararray* pavn);
vect_n* eval_pavn(pararray* pavn,btreal t);

pararray_vn* vr2pararray(vectray* vr);


