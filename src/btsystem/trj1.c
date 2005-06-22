
/** time-point list

A sequence of times and points are interpolated linearly

*/

/** Time - Point list trajectory info

For a time - point list the relationship between time and arc-length is 1 to 1.
So all we do is maintain the elapsed arc-length;

*/

typedef struct {
  btreal s;
}tpl_trj;

tpl_trj * new_tpl_trj()
{
   tpl_trj *bttrj;
  if ((bttrj = malloc(sizeof(tpl_trj))) == NULL)
  {
    syslog(LOG_ERR,"new_tpl_trj: memory allocation failed");
    return NULL;
  }
  return bttrj;
  
}
btreal tpl_init_T(void *dat,btreal t)
{
  tpl_trj *trj;
  trj = (tpl_trj*)dat;
  trj->s = t;
  return t;
}
btreal tpl_S_of_dt(void *dat,btreal dt)
{
  tpl_trj *trj;
  trj = (tpl_trj*)dat;
  trj->s += dt;
  return trj->s;
}
vect_n* tpl_init_S(void *dat,btreal s)
{
  btpath_pwl *pth;
  pth = (btpath_pwl*)dat;

  return dsinit_pwl(pth,s);
}

vect_n* tpl_Q_of_ds(void *dat,btreal ds)
{
  btpath_pwl *pth;
  pth = (btpath_pwl*)dat;
  
  return ds_pwl(pth,ds);
}

bttrajectory* tpl_load_n_register(char *filename)
{
  bttrajectory* trj;
  tpl_trj *crv_trj;
  
  
  trj = new_bttrajectory();
  crv_trj = new_tpl_trj();
  
  settraj_bttrj(trj,(void *)crv_trj, tpl_init_T, tpl_S_of_dt);
  setpath_bttrj(trj,void *crv_dat, void *initfunc, void *evalfunc);
  
  
  
  
  
  
}








