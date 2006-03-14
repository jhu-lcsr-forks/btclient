/** \file btmathtest.c
\brief btsystem internal testing template

*/
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <time.h>
#include "btmath.h"



int main( int argc, char **argv )
{
  clock_t start,end;
  double time_used,c;

  long int cnt;
  char buff[355];
  vect_n *t1,*t2,*t3;
  vect_n *q,*dq,*ddq;

   
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  q = new_vn(6);
  dq = new_vn(6);
  ddq = new_vn(6);
  fill_vn(ddq,1.0);
  fill_vn(dq,1.0);
  start = clock();
  for (cnt = 0;cnt < 1000000L; cnt ++){
    set_vn(q,add_vn(q,add_vn(dq,scale_vn(2.0,ddq))));
  }
  end = clock();
  time_used = ((double) (end - start))/CLOCKS_PER_SEC;
  printf("\n");
  print_vn(q);
  printf(" : Time:%f\n",time_used);
}





