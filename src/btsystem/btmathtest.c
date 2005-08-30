/** \file btmathtest.c
\brief btsystem internal testing template

*/
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include "btmath.h"
#include "btrobot.h"
#include "btpath.h"
#include "btcontrol.h"

int main( int argc, char **argv )
{
  btrobot wam;
  int cnt;
  char buff[355];
  vect_n *t1,*t2,*t3;
  vect_n *q,*dq,*ddq;
  vect_3 *p,*o;
  btfilter *visFilt[4],*test;
  ct_traj* ct;
  vectray *vr,*vr2;
  btreal s,dt;
   
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  q = new_vn(4);
  vr2 = new_vr(4,1000);
  ct = (ct_traj*)malloc(sizeof(ct_traj));
  read_csv_file_vr("teach.csv",&vr);

  create_ct(ct,vr);
  dt = 0.002;
  append_vr(vr2,dsinit_pwl(ct->pwl,0.0));
  for (cnt = 0;cnt < 1000; cnt++){
    append_vr(vr2,ds_pwl(ct->pwl,dt));
  }
  write_csv_file_vr("result.csv",vr2);
  freebtptr();
}





