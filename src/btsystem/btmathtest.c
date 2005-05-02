

#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include "btmath.h"
#include "btrobot.h"

int main( int argc, char **argv )
{
  btrobot wam;
  int cnt;
  vect_n *t1,*t2,*t3;
  vect_n *q,*dq,*ddq;
  vect_3 *p,*o;
  btfilter *visFilt[4],*test;
   
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  test_btrobot();
   /*
  test = new_btfilter(4);
  for (cnt = 0; cnt < 3; cnt ++){
				 visFilt[cnt] = new_btfilter(5);
         init_btfilter_lowpass(visFilt[cnt], 0.002, 1);
  }

  for (cnt = 0; cnt < 3; cnt ++){
				 eval_btfilter(visFilt[cnt], 3.0);
  }*/
  //p = new_v3();
  //o = new_v3();

  //new_vn_group(3,4,&q,&dq,&ddq);
  
  //test_vr(1.0);
  //test_vn(1.0);
  
  //const_vn(q,0.0,1.5707963268,0.0,0.0);
  /*
  const_vn(q,0.0,1.5708,0.0,0.0);
  const_vn(dq,0.0,0.0,0.0,0.0);
  const_vn(ddq,0.0,0.0,0.0,0.0);

  fill_v3(p,0.0);
  
  
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  init_wam_btrobot(&wam);
  
  set_q_btrobot(&wam, q, dq, ddq);
  eval_fk_btrobot(&wam);
  eval_fd_btrobot(&wam);
  apply_force_btrobot(&wam,1,p,C_v3(0.0,0.0,0.0),C_v3(0.0,0.0,0.0));
  eval_bd_btrobot(&wam);
  
 
  printf("\nJoint Vars:");
  print_vn(wam.q);
  printf("\nJoint Torques:");
  print_vn(wam.t);
  */
  printf("\nend");
  freebtptr();
}





