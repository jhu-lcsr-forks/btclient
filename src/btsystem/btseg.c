
#include "btseg.h"
#include <stdio.h>

void dump_para(parabolic *p,FILE *out)
{
  fprintf(out,"tf: %f, sf: %f, spf: %f, sppf: %f\n",p->tf,p->sf,p->spf,p->sppf);
}

btreal sp_of_t_para(parabolic *p, btreal t)
{
  btreal difft;
  if (t > p->tf)
    return p->spf;
  difft = t - p->tf;
  return p->spf + p->sppf * difft;
}
btreal s_of_t_para(parabolic *p, btreal t)
{
  btreal difft;
  if (t > p->tf){ //End of segment
      return p->sf;
  }
  difft = t - p->tf;
  return p->sf + p->spf * difft + 0.5*p->sppf * difft * difft;
}
btreal s_of_t_paral(parabolic **pin, btreal t)
{
  btreal difft;
  parabolic *p;
  
  p = *pin;
  while (t > p->tf){ //Next segment
    p = p->next;
    if (p != NULL) {
      *pin = p;
    }
    else { //no more in the list
      p = *pin;  //backup to the end of the list
      break;
    }
  }
  return s_of_t_para(p,t);
}

/** Init bi object from boundary conditions
*/
btreal boundary_para(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf)
{
  
  btreal a,tf;
  //spf = sp0 + a*tf
  //sf = s0 + sp0*tf + 0.5*a*tf^2
  
  //spf = sp0 + a*tf
  //spf - sp0 = a*tf
  //sf = s0 + sp0*tf + 0.5*(spf - sp0)*tf
  //sf = s0 + sp0*tf + 0.5*spf*tf - 0.5*sp0*tf
  //sf = s0 + 0.5*sp0*tf + 0.5*spf*tf
  //(sf - s0)*2/(sp0 + spf) = tf
  
  tf = 2*(sf-s0)/(sp0+spf);
  a = (spf - sp0)/(tf);
  
  b->sf = sf;
  b->tf = tf + t0;
  b->sppf = a;
  b->spf = spf;
  return b->tf;
}
/** 

t = corner point time
st = corner point location

sp0 = starting velocity
spf = final velocity
t0 = starting time
*/
btreal blend_para(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t)
{
  btreal tf,a,s0,sf;

  tf = (t-t0)*2;
  //spf = sp0 + a*tf
  //a = (spf-sp0)/tf;
  //st = s0 + sp0*(t-t0);
  s0 = st - sp0*(t-t0);
  //sf = s0 + sp0*tf + 0.5*a*tf^2
  sf = st + spf*(tf-t);
  
  return boundary_para(b,t0,s0,sf,sp0,spf);
}

pararray* new_pa(int max)
{
  void* mem;
  pararray* pa;

  mem = btmalloc(sizeof(parabolic) * max + sizeof(pararray));
  pa = (pararray*)mem;
  pa->pa = (parabolic*)(mem + sizeof(pararray));
  pa->max = max;
  pa->cnt = 0;
  return pa;
}
void clear_pa(pararray* pa)
{
  pa->cnt = 0;
}

/** Adds to the end of the list */
btreal add_bseg_pa(pararray* pa,btreal t0,btreal s0,btreal sf,btreal sp0,btreal spf)
{
  int cnt; 
  btreal ret;
  if (pa->cnt > pa->max-1) ret = pa->pa[pa->cnt].tf;
  else{
  ret = boundary_para(&(pa->pa[pa->cnt]),t0,s0,sf,sp0,spf);
                                    
    //link with previous segment                             
    if (pa->cnt > 0){
      pa->pa[pa->cnt - 1].next = &(pa->pa[pa->cnt]);
    }
    pa->pa[pa->cnt].next = NULL;
  }
  dump_para(&(pa->pa[pa->cnt]),stdout);
  pa->cnt++;
  
  return ret;
}

btreal reset_pa(pararray* pa)
{
  int cnt;
  pa->iter = &(pa->pa[0]);
  return eval_pa(pa,0.0);
}

btreal eval_pa(pararray* pa,btreal t)
{
  int cnt; 
  return s_of_t_paral(&(pa->iter),t); 
}

pararray_vn* new_pavn(int max,int elements)
{
  void *mem;
  int cnt;
  pararray_vn* pavn;
  mem = btmalloc(sizeof(pararray_vn) + sizeof(pararray*) * elements);
  pavn = (pararray_vn*)mem;
  pavn->pa = (pararray**)(mem + sizeof(pararray_vn));
  pavn->elements = elements;
  for (cnt = 0; cnt < elements; cnt ++)
    pavn->pa[cnt] = new_pa(max);
  pavn->result = new_vn(elements);
  return pavn;
}
void clear_pavn(pararray_vn* pavn)
{
  int cnt;
  for (cnt = 0;cnt< pavn->elements; cnt ++)
    clear_pa(pavn->pa[cnt]);
  
}
vect_n*  add_bseg_pavn(pararray_vn* pavn,vect_n* t0,vect_n* s0,vect_n* sf,vect_n* sp0,vect_n* spf)
{
  int cnt;
  btreal ret;
  for (cnt = 0;cnt< pavn->elements; cnt ++)
    setval_vn(pavn->result,cnt,add_bseg_pa(pavn->pa[cnt],getval_vn(t0,cnt),getval_vn(s0,cnt),getval_vn(sf,cnt),getval_vn(sp0,cnt),getval_vn(spf,cnt)));
  return pavn->result;
}
vect_n*  reset_pavn(pararray_vn* pavn)
{
  int cnt;
  btreal ret;
  for (cnt = 0;cnt< pavn->elements; cnt ++)
    setval_vn(pavn->result,cnt,reset_pa(pavn->pa[cnt]));
  return pavn->result;
}
vect_n* eval_pavn(pararray_vn* pavn,btreal t)
{
  int cnt;
  btreal ret;
  for (cnt = 0;cnt< pavn->elements; cnt ++)
    setval_vn(pavn->result,cnt,eval_pa(pavn->pa[cnt],t));
  return pavn->result;
}
/**
Convert a vectray of time/points to a segment list

Time in the first value;

*/
pararray_vn* vr2pararray(vectray* vr,btreal acc)
{
  pararray_vn* pavn;
  int cnt,idx;
  btreal t1,t2,t3,x1,x2,x3;
  btreal v1,v2,v3,tacc,t1p2,t2p3,tmax;
  btreal tf,a,s0,sf;
  btreal sv0,svf,sa0,saf;
  btreal s0_prev,tf_prev;
  
  btreal dt,dx;
  
  //new pararray of size (Viapoints - 1)*2 + 1
  pavn = new_pavn(2*numrows_vr(vr)-1+5,numelements_vr(vr)-1);
  
  
  tmax = 0;
  for (cnt = 0;cnt < pavn->elements;cnt ++){

    /* First acceleration segment */
    t1 = getval_vn(idx_vr(vr,0),0);
    t2 = getval_vn(idx_vr(vr,1),0);
    x1 = getval_vn(idx_vr(vr,0),cnt+1);
    x2 = getval_vn(idx_vr(vr,1),cnt+1);    
    printf("cnt %d, t1: %f, t2: %f, x1: %f, x2: %f\n",cnt,t1,t2,x1,x2);
    dt = t2-t1;
    dx = x2-x1;
    v1 = 0.0;
    v2 = dx/dt;
    a = max_bt(acc,8*fabs(dx)/(3*dt*dt)); 
    tacc = dt - sqrt(dt*dt - 2*fabs(dx)/a);
    sa0 = x1;
    saf = x1 + v2*tacc;
    tf_prev = add_bseg_pa(pavn->pa[cnt],0.0,sa0,saf,v1,v2);  //acc seg starting at time 0.0
    printf(" tf_prev: %f\n",tf_prev);
    for(idx = 1; idx < numrows_vr(vr)-1; idx++){
      /* Extract info */
      t1 = getval_vn(idx_vr(vr,idx-1),0);
      t2 = getval_vn(idx_vr(vr,idx),0);
      t3 = getval_vn(idx_vr(vr,idx+1),0);
      x1 = getval_vn(idx_vr(vr,idx-1),cnt+1);
      x2 = getval_vn(idx_vr(vr,idx),cnt+1);
      x3 = getval_vn(idx_vr(vr,idx+1),cnt+1);
      printf("idx: %d, t1: %f, t2: %f, x1: %f, x2: %f\n",idx,t1,t2,x1,x2);
      /* Calc some useful values */
      v1 = (x2-x1)/(t2-t1);
      v2 = (x3-x2)/(t3-t2);
      t1p2 = (t1 + t2)/2;
      t2p3 = (t2 + t3)/2;
      /* Shrink acceleration if necessary */
      tmax = min_bt(t2-t1p2,t2p3-t2);
      //vf = v0 + at => t = (vf-v0)/a
      tacc = fabs(v2-v1)/acc;
      tacc = min_bt(tacc,tmax); //
      
      /* Calc : tf_prev & saf carry history from prev loops */
      sa0 = x2 - v1*(tacc/2); //acc start pos 
      printf(" tf_prev: %f, saf: %f, sa0: %f, v1: %f, tacc: %f\n",tf_prev,saf,sa0,v1,tacc);
      tf_prev = add_bseg_pa(pavn->pa[cnt],tf_prev,saf,sa0,v1,v1); //velocity seg
     
      saf = x2 + v2*(tacc/2); //acc end pos
      printf(" tf_prev: %f, saf: %f, sa0: %f, v1: %f, v2: %f\n",tf_prev,saf,sa0,v1,v2);
      tf_prev = add_bseg_pa(pavn->pa[cnt],tf_prev,sa0,saf,v1,v2);  //acc seg
      
    }
    printf("vr%d idx %d\n",numrows_vr(vr),idx);
    /* First acceleration segment */
    t1 = getval_vn(idx_vr(vr,idx-1),0);
    t2 = getval_vn(idx_vr(vr,idx),0);
    x1 = getval_vn(idx_vr(vr,idx-1),cnt+1);
    x2 = getval_vn(idx_vr(vr,idx),cnt+1);    
    
    dt = t2-t1;
    dx = x2-x1;
    v1 = dx/dt;
    v2 = 0.0;
    a = max_bt(acc,8*fabs(dx)/(3*dt*dt)); 
    tacc = dt - sqrt(dt*dt - 2*fabs(dx)/a);
    sa0 = x2 - v2*tacc;
    
    tf_prev = add_bseg_pa(pavn->pa[cnt],tf_prev,saf,sa0,v1,v1); //velocity seg
    saf = x2;
    tf_prev = add_bseg_pa(pavn->pa[cnt],tf_prev,sa0,saf,v1,v2);  //acc seg starting at time 0.0


  }
  
  
  
  return pavn;
}

int main()
{
  FILE *out;
  char buf[250];
  int cnt;
  pararray *pb;
  pararray_vn *pvn;
  vectray *vr;
  btreal t,s;
  pb = new_pa(10);
  s = 0.0;

  read_csv_file_vr("t2",&vr);
  write_csv_file_vr("t3",vr);
  pvn = vr2pararray(vr,1);
  out = fopen("curve.csv","w");
  s = 0;

  t = add_bseg_pa(pb,0.0,0.0,10,0,10);
  printf("Tf: %f  \n",t);
  t = add_bseg_pa(pb,t,10,20,10,-5);
  printf("Tf: %f  \n",t);
  t = add_bseg_pa(pb,t,20.0,0.0,-5.0,2.0);
  printf("Tf: %f  \n",t);
  reset_pa(pb);
  reset_pavn(pvn);
  t = 12;
  for(cnt = 0;cnt < 51;cnt++){
    
    //set_vn(res,calc_cseg(&t,s));
    s+=t/50;
    fprintf(out,"%f %s\n",s,sprint_plt_vn(buf,eval_pavn(pvn,s)));
    //fprintf(out,"%f %f\n",s,eval_pa(pb,s));
    //fprintf(out,"%f, %f\n",s,calc_tseg(&ms,s));
  }
  
  return 0;
}






