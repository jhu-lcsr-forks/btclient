/*======================================================================*
 *  Module .............libbt
 *  File ...............btmath.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......18 Feb 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/
/** \file btrobot.c
\brief serial robot kinematics & dynamics

example usage
\code
  btrobot robot;
  new_bot(&robot,2);

  link_geom_bot(&robot,0,0.0,0.0,0.0,-pi/2.0);
  link_geom_bot(&robot,1,0.0,0.0,0.0,pi/2.0);
  link_mass_bot(&robot,0,C_v3(0.0,0.1405,-0.0061),12.044);
  link_mass_bot(&robot,1,C_v3(0.0,-0.0166,0.0096),5.903);
  tool_geom_bot(&robot,0.0,0.356,0.0,0.0);
  tool_mass_bot(&robot,C_v3(0.0,0.0,0.025),2.50);
  
  while (1){
    set_q_bot(&robot,Jpos,Jvel,Jacc); 
    eval_fk_bot(&robot);  //forward kinematics
    eval_fd_bot(&robot);  //forward dynamics
  
    apply_tool_force_bot(&robot, Cpoint, Cforce, Ctrq);

    eval_bd_bot(&robot);  //backward dynamics

    get_t_bot(&robot,Jtrq);
  }
\endcode
*/
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>

#include "btrobot.h"
#include "btmath.h"
/** Internal: Allocates and initializes a new robot link

\param type 0=Revolute Joint, !0=Prismatic Joint
*/
int new_btlink(btlink* link,int type)
{
	link->trans = new_mh();
	link->origin = new_mh();
  link->w = new_v3();
	link->dw = new_v3();
  link->ae = new_v3();
  link->ac = new_v3();
  link->b = new_v3();
  link->o = new_v3();
  link->z = new_v3();
  const_v3(link->z,0.0,0.0,1.0);
  link->g = new_v3();
  
  
  link->cog = new_v3();
  link->I = new_m3();
  link->f = new_v3();
  link->fi = new_v3();
  link->t = new_v3();
  link->Rm = new_v3();
  link->Rl = new_v3();
  link->Rp = new_v3();
  const_v3(link->Rp,0.0,0.0,0.0);
  link->eforce.f = new_v3();
  link->eforce.t = new_v3();
  link->eforce.p = new_v3();
  link->lastforce.f = new_v3();
  link->lastforce.t = new_v3();
  link->lastforce.p = new_v3();
  link->tmpforce.f = new_v3();
  link->tmpforce.t = new_v3();
  link->tmpforce.p = new_v3();
  link->Gscale = 1;
  if (type) {link->type = 1; link->R = 0.0; link->P = 1.0;}
  else { link->type = 0; link->R = 1.0; link->P = 0.0;}
}
/** Allocate and initialize a new robot object.
\param nlinks Number of links the robot has not including the base link. (nlinks = number of joints)
*/
int new_bot(btrobot* robot, int nlinks)
{
  void* ptr;
  int cnt;
  
  ptr = btmalloc((nlinks+3)*sizeof(btlink));
  
  robot->num_links = nlinks;
  robot->user = (btlink*)ptr;
  robot->world = (btlink*)(ptr + sizeof(btlink));
  robot->links = (btlink*)(ptr + 2*sizeof(btlink));
  robot->tool = (btlink*)(ptr + (nlinks+2)*sizeof(btlink));
  
  new_btlink(robot->user,0);
  new_btlink(robot->world,0);
  for (cnt = 0;cnt < robot->num_links;cnt++)
    new_btlink(&(robot->links[cnt]),0);
  new_btlink(robot->tool,0);
  
  robot->q = new_vn(nlinks);
  robot->dq = new_vn(nlinks);
  robot->ddq = new_vn(nlinks);
  robot->t = new_vn(nlinks);
  robot->G = new_v3();
  const_v3(robot->G,0.0,0.0,-9.8);

  return 0;
}


/** Define the DH parameters of the specified link
  the value passed for the joint parameter (theta or d depending on revolute or prismatic joint)
  is an offset to allow specification of where the joint zero position is.
*/
void link_geom_bot(btrobot* robot, int link, double theta, double d, double a, double alpha)
{
  
  robot->links[link].Alpha = alpha;
  robot->links[link].Theta = theta;
  robot->links[link].D = d;
  robot->links[link].A = a;
  robot->links[link].sinAlpha = sin(alpha);
  robot->links[link].cosAlpha = cos(alpha);
  robot->links[link].sinTheta = sin(theta);
  robot->links[link].cosTheta = cos(theta);
  
  make_transform_btlink(&(robot->links[link]),0.0);
  getcol_mh(robot->links[link].Rl,robot->links[link].trans,3); 
  set_v3(robot->links[link].Rl,matTXvec_m3(robot->links[link].trans,robot->links[link].Rl));
}
/** Define the DH parameters of the tool
  the value passed for the joint parameter (theta or d depending on revolute or prismatic joint)
  is an offset to allow specification of where the joint zero position is.
*/
void tool_geom_bot(btrobot* robot,double theta, double d, double a, double alpha)
{
  
  robot->tool->Alpha = alpha;
  robot->tool->Theta = theta;
  robot->tool->D = d;
  robot->tool->A = a;
  robot->tool->sinAlpha = sin(alpha);
  robot->tool->cosAlpha = cos(alpha);
  robot->tool->sinTheta = sin(theta);
  robot->tool->cosTheta = cos(theta);
  
  make_transform_btlink(robot->tool,0.0);
  getcol_mh(robot->tool->Rl,robot->tool->trans,3); 
  set_v3(robot->tool->Rl,matTXvec_m3(robot->tool->trans,robot->tool->Rl));
}
/** get the transform from the base to the tool
*/
void tool_trans_bot(btrobot* robot, matr_h* trans)
{
  set_mh(robot->tool->trans,trans);
  
  getcol_mh(robot->tool->Rl,robot->tool->trans,3); 
  set_v3(robot->tool->Rl,matTXvec_m3(robot->tool->trans,robot->tool->Rl));
}
/** Set the mass parameters of the specified link

WARNING: you must call link_geom_bot() for this link first!

\param link The link index (Link 1 rotates around joint 1 (z axis of frame 0) and has index 0)
\param cog Location of the link center of mass in the link frame
\param m The mass of the link
*/
void link_mass_bot(btrobot* robot, int link, vect_3 *cog,double m)
{
  robot->links[link].m = m;
  set_v3(robot->links[link].cog,cog);
  set_v3(robot->links[link].Rm,add_v3(robot->links[link].Rl,robot->links[link].cog));
}
/** Set the mass parameters of the tool

WARNING: you must call tool_geom_bot() for this link first!

\param cog Location of the tool center of mass in the tool frame
\param m The mass of the link
*/
void tool_mass_bot(btrobot* robot, vect_3 *cog,double m)
{
  robot->tool->m = m;
  set_v3(robot->tool->cog,cog);
  set_v3(robot->tool->Rm,add_v3(robot->tool->Rl,robot->tool->cog));
}
/** Build a tranform matrix from joint parameters and link DH parameters.
\param q Joint parameters. The link will interpret q as an angle or distance depending on how it was initialized.
*/
void make_transform_btlink(btlink* link,double q) //set the transform matrix
{
  link->sinTheta = sin(q*link->R + link->Theta);
  link->cosTheta = cos(q*link->R + link->Theta);
  
  setrow_mh(link->trans,0, link->cosTheta, -1.0*link->sinTheta*link->cosAlpha,      link->sinTheta*link->sinAlpha, link->A*link->cosTheta);
  setrow_mh(link->trans,1, link->sinTheta,      link->cosTheta*link->cosAlpha, -1.0*link->cosTheta*link->sinAlpha, link->A*link->sinTheta);
  setrow_mh(link->trans,2,              0,                     link->sinAlpha,                     link->cosAlpha,    link->D + q*link->P);
}
/** Scale the effect of gravity by the factor Gscale.

This allows the user to ramp up the gravity compensation, or just turn it on or off. 
\param Gscale Gravity scaling factor, normally set between 0.0 and 1.0.
*/
void set_gravity_bot(btrobot* robot, double Gscale) //set value to scale effect of gravity
{
  int cnt;
  for (cnt = 0;cnt < robot->num_links + 1;cnt++)
    robot->links[cnt].Gscale = Gscale;
}
btreal get_gravity_bot(btrobot* robot)
{
  return robot->links[0].Gscale;
}
//void set_jacobian_scheme_bot(btrobot* robot,int scheme); //how to handle extra dof

/** Data access function to set the present joint angle state of the robot object.

Units: Angle = Radians, Time = Seconds
*/
void set_q_bot(btrobot* robot, vect_n* q, vect_n* dq, vect_n* ddq) //set generalized joit parameters
{
  set_vn(robot->q,q);
  set_vn(robot->dq,dq);
  set_vn(robot->ddq,ddq);
  
}

/** Data access function to get the present joint torques of the robot object.
*/
void get_t_bot(btrobot* robot,vect_n* t)
{
  set_vn(t,robot->t);
}

/** Calculates the forward kinematics for the robot.

  This function calculates the forward kinematics for the whole robot and then
  caches the "per link" results for later access by internal forward/reverse 
  dynamics calculations and by the user.
*/
void eval_fk_bot(btrobot* robot) //forward kinematics
{
  int cnt;
  for (cnt = 0;cnt < robot->num_links;cnt++){
    //calc tranform for present q
    make_transform_btlink(&(robot->links[cnt]),robot->q->q[cnt]);
    //calc transform to origin for present q
    set_mh(robot->links[cnt].origin,mul_mh(robot->links[cnt-1].origin,robot->links[cnt].trans));
    //get z axis vector for later calculations
    getcol_mh(robot->links[cnt].z,robot->links[cnt].origin,2);
    //get frame location in world frame coordinates for use in later calculations
    getcol_mh(robot->links[cnt].o,robot->links[cnt].origin,3);
  } 
  //repeat for the tool frame
    set_mh(robot->tool->origin,mul_mh(robot->links[robot->num_links-1].origin,robot->tool->trans));
    getcol_mh(robot->tool->z,robot->tool->origin,2);
    getcol_mh(robot->tool->o,robot->tool->origin,3);
}
void eval_fj_bot(btrobot* robot) //forward jacobian
{
  // J = [J1J2J3J4J5...]
  // Ji = z(i-1)X(On - O(i-1)]
  //    = z(i-1)

}
/** Transform a point in Link[n] frame to the World frame.

\param robot pointer to the robot object
\param link The link index (Link 1 rotates around joint 1 (z axis of frame 0) and has index 0)
\param position The coordinates expressed in the frame of Link[n] that you want to tranform
\return A pointer to the vector holding the World frame position
*/
vect_3* Ln_to_W_bot(btrobot* robot,int link, vect_3 *position)
{
  return matXvec_mh(robot->links[link].origin,position);
}

/** Transform a point in Tool frame to the World frame.

\param robot pointer to the robot object
\param position The coordinates expressed in the Tool frame that you want to tranform
\return A pointer to the vector holding the World frame position
*/
vect_3* T_to_W_bot(btrobot* robot, vect_3 *position) //find world position of location defined in tool frame
{
  return matXvec_mh(robot->tool->origin,position);
}

/** Returns a pointer to the transform matrix between the world frame and the
specified link frame
\bug It is not safe to perform calculations on this data structure that write 
any data to it. In most conditions it should be used in the form: 
  set_mh(mytrans,Ln_to_W_trans_bot(robot,2));
However, it is ok to perform read operations on it
*/
matr_h* Ln_to_W_trans_bot(btrobot* robot,int link) //return tranform matrix
{
  return robot->links[link].origin;
}
matr_h* T_to_W_trans_bot(btrobot* robot)
{
  return robot->tool->origin;
}
//void fk_btwam(btrobot* robot);  //forward kinematics optimized for the wam
//void fj_bot(btrobot* robot);
/** Calculate RNE forward dynamics for robot.

*/

void eval_fd_bot(btrobot* robot) //forward dynamics
{
  int cnt;
  for (cnt = 0;cnt < robot->num_links + 1;cnt++){
    set_v3(robot->links[cnt].lastforce.t,robot->links[cnt].eforce.t);
    set_v3(robot->links[cnt].lastforce.f,robot->links[cnt].eforce.f);
    const_v3(robot->links[cnt].eforce.t,0.0,0.0,0.0);
    const_v3(robot->links[cnt].eforce.f,0.0,0.0,0.0);
    //calc tranform for present q
    set_v3(robot->links[cnt].b,matTXvec_m3(robot->links[cnt].trans,C_v3(0.0,0.0,1.0)));
    //set_v3(robot->links[cnt].b,matTXvec_m3(robot->links[cnt].origin,C_v3(0.0,0.0,1.0)));

    /*printf("\nLink %d b: ",cnt);print_v3(robot->links[cnt].b);printf("\n");
    printf("\nLink %d Rl: ",cnt);print_v3(robot->links[cnt].Rl);printf("\n");
    printf("\nLink %d Rm: ",cnt);print_v3(robot->links[cnt].Rm);printf("\n");
    */
    set_v3(robot->links[cnt].w,
           add_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].w),
                  scale_v3(getval_vn(robot->dq,cnt),robot->links[cnt].b)));
    /*
    printf("\nLink %d w: ",cnt);
    print_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].w));printf(" + ");
    print_v3(scale_v3(getval_vn(robot->dq,cnt),robot->links[cnt].b));printf(" = ");
    print_v3(robot->links[cnt].w);
    printf("\n");
    */
    set_v3(robot->links[cnt].dw,
           add_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].dw),
           add_v3(scale_v3(getval_vn(robot->ddq,cnt),robot->links[cnt].b),
                  cross_v3(robot->links[cnt].w,
                        scale_v3(getval_vn(robot->dq,cnt),robot->links[cnt].b)))));
    /*
    printf("\nLink %d dw: ",cnt);
    print_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].dw));printf(" + ");
    print_v3(scale_v3(getval_vn(robot->ddq,cnt),robot->links[cnt].b));printf(" + ");
    print_v3(cross_v3(robot->links[cnt].w,scale_v3(getval_vn(robot->dq,cnt),robot->links[cnt].b)));printf(" = ");
    print_v3(robot->links[cnt].dw);
    printf("\n");
    */
    set_v3(robot->links[cnt].ae,
           add_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].ae),
           add_v3(cross_v3(robot->links[cnt].dw,robot->links[cnt].Rl),
                  cross_v3(robot->links[cnt].w,
                        cross_v3(robot->links[cnt].w,robot->links[cnt].Rl)))));
    /*
    printf("\nLink %d ae: ",cnt);
    print_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].ae));printf(" + ");
    print_v3(cross_v3(robot->links[cnt].dw,robot->links[cnt].Rl));printf(" + ");
    print_v3(cross_v3(robot->links[cnt].w, cross_v3(robot->links[cnt].w,robot->links[cnt].Rl)));printf(" = ");
    print_v3(robot->links[cnt].ae);
    printf("\n");  */                
   
    set_v3(robot->links[cnt].ac,
           add_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].ae),
           add_v3(cross_v3(robot->links[cnt].dw,robot->links[cnt].Rm),
                  cross_v3(robot->links[cnt].w,
                        cross_v3(robot->links[cnt].w,robot->links[cnt].Rm)))));
 /*   
    printf("\nLink %d ac: ",cnt);
    print_v3(matTXvec_m3(robot->links[cnt].trans,robot->links[cnt-1].ae));printf(" + ");
    print_v3(cross_v3(robot->links[cnt].dw,robot->links[cnt].Rm));printf(" + ");
    print_v3(cross_v3(robot->links[cnt].w,cross_v3(robot->links[cnt].w,robot->links[cnt].Rm)));printf(" = ");
    print_v3(robot->links[cnt].ac);
    printf("\n");*/      
  }  

}
/** Calculate RNE backward dynamics for robot.

*/
void eval_bd_bot(btrobot* robot) //backward kinematics
{
  int cnt;
  
  set_v3(robot->tool->g,matTXvec_m3(robot->tool->origin,scale_v3(robot->tool->Gscale,robot->G)));
  set_v3(robot->tool->f,
        sub_v3(scale_v3(robot->tool->m,robot->tool->ac),
               add_v3(neg_v3(robot->tool->eforce.f),scale_v3(robot->tool->m,robot->tool->g))));
               
  set_v3(robot->tool->t,
        add_v3(neg_v3(cross_v3(robot->tool->f,robot->tool->Rm)),
        add_v3(cross_v3(robot->tool->fi,robot->tool->cog),
        add_v3(neg_v3(robot->tool->eforce.t),
        add_v3(robot->tool->dw,
               cross_v3(robot->tool->w,
                        matXvec_m3(robot->tool->I,robot->tool->w)))))));

  
  for (cnt = robot->num_links-1;cnt >= 0;cnt--){
    
    set_v3(robot->links[cnt].g,matTXvec_m3(robot->links[cnt].origin,scale_v3(robot->links[cnt].Gscale,robot->G)));
    /*
    printf("\nLink %d g: ",cnt);print_v3(robot->links[cnt].g);printf("\n");
    printf("\nLink %d trans: ",cnt);print_mh(robot->links[cnt].trans);printf("\n");
    printf("\nLink %d origin: ",cnt);print_mh(robot->links[cnt].origin);printf("\n");
    */
    
   set_v3(robot->links[cnt].fi,matXvec_m3(robot->links[cnt+1].trans,robot->links[cnt+1].f));
     set_v3(robot->links[cnt].f,
        add_v3(robot->links[cnt].fi,
        sub_v3(scale_v3(robot->links[cnt].m,robot->links[cnt].ac),
               add_v3(neg_v3(robot->links[cnt].eforce.f),scale_v3(robot->links[cnt].m,robot->links[cnt].g)))));
     /*
     printf("\nLink %d force: ",cnt);
     print_v3(robot->links[cnt].fi);printf(" + ");
     print_v3(scale_v3(robot->links[cnt].m,robot->links[cnt].ac));printf(" - ( ");
     print_v3(robot->links[cnt].eforce.f);printf(" + ");
     print_v3(scale_v3(robot->links[cnt].m,robot->links[cnt].g));printf(" ) \n    = ");
     print_v3(robot->links[cnt].f);
     */

    set_v3(robot->links[cnt].t,
        add_v3(matXvec_m3(robot->links[cnt+1].trans,robot->links[cnt+1].t),
        add_v3(neg_v3(cross_v3(robot->links[cnt].f,robot->links[cnt].Rm)),
        add_v3(cross_v3(robot->links[cnt].fi,robot->links[cnt].cog),
        add_v3(neg_v3(robot->links[cnt].eforce.t),
        add_v3(robot->links[cnt].dw,
               cross_v3(robot->links[cnt].w,
                        matXvec_m3(robot->links[cnt].I,robot->links[cnt].w))))))));
     
     /*
     printf("\nLink %d torque: ",cnt);
     print_v3(matXvec_m3(robot->links[cnt+1].trans,robot->links[cnt+1].t));printf(" + ");
     print_v3(neg_v3(cross_v3(robot->links[cnt].f,robot->links[cnt].Rm)));printf(" - ( ");
     print_v3(cross_v3(robot->links[cnt].fi,robot->links[cnt].cog));printf(" + ");
     print_v3(robot->links[cnt].dw);printf(" + ");
     print_v3(cross_v3(robot->links[cnt].w,matXvec_m3(robot->links[cnt].I,robot->links[cnt].w)));printf(" + ");
     print_v3(robot->links[cnt].eforce.t);printf(" ) \n    = ");
     print_v3(robot->links[cnt].t);    
     printf("\nLink %d z: ",cnt);print_v3(robot->links[cnt].z);printf("b: ",cnt);print_v3(robot->links[cnt].b);printf("\n");
     */ 

    setval_vn(robot->t,cnt,dot_v3(robot->links[cnt].t,robot->links[cnt].b));
  }
        
  
}
/** Apply a force to a link.

*/
void apply_force_bot(btrobot* robot,int link, vect_3* pos, vect_3 *force, vect_3* torque)
{
  //pos + res = cog :=> cog - pos = res | res is vector from force point to cog.
  //The idea is to sum this force,torque with other force torques acting on the COG of this link.
  const_v3(robot->links[link].Rp,0.0,0.0,0.0);
  set_v3(robot->links[link].Rp,sub_v3(robot->links[link].cog,pos));

  set_v3(robot->links[link].eforce.t,
    add_v3(robot->links[link].eforce.t,
    add_v3(matTXvec_m3(robot->links[link].origin,torque),
    cross_v3(matTXvec_m3(robot->links[link].origin,force),robot->links[link].Rp))));
    
  set_v3(robot->links[link].eforce.f,
    add_v3(robot->links[link].eforce.f,matTXvec_m3(robot->links[link].origin,force)));
}
/** Apply a force to a tool.

*/
void apply_tool_force_bot(btrobot* robot, vect_3* pos, vect_3 *force, vect_3* torque)
{
  //pos + res = cog :=> cog - pos = res | res is vector from force point to cog.
  //The idea is to sum this force,torque with other force torques acting on the COG of this link.
  const_v3(robot->tool->Rp,0.0,0.0,0.0);
  set_v3(robot->tool->Rp,sub_v3(robot->tool->cog,pos));

  set_v3(robot->tool->eforce.t,
    add_v3(robot->tool->eforce.t,
    add_v3(matTXvec_m3(robot->tool->origin,torque),
    cross_v3(matTXvec_m3(robot->tool->origin,force),robot->tool->Rp))));
    
  set_v3(robot->tool->eforce.f,
    add_v3(robot->tool->eforce.f,matTXvec_m3(robot->tool->origin,force)));
}

void init_4dof_wam_bot(btrobot* robot)
{
  const double pi = 3.14159;
  new_bot(robot,4);
  
  link_geom_bot(robot,0,0.0,0.0,0.0,-pi/2.0);
  link_geom_bot(robot,1,0.0,0.0,0.0,pi/2.0);
  link_geom_bot(robot,2,0.0,0.550,0.045,-pi/2.0);
  link_geom_bot(robot,3,0.0,0.0,-0.045,pi/2.0);
  tool_geom_bot(robot,0.0,0.3574,0.0,0.0);
  
  link_mass_bot(robot,0,C_v3(0.0,0.1405,-0.0061),12.044);
  link_mass_bot(robot,1,C_v3(0.0,-0.0166,0.0096),5.903);
  link_mass_bot(robot,2,C_v3(-0.0443,0.2549,0.0),2.08);
  link_mass_bot(robot,3,C_v3(0.01465,0.0,0.1308),1.135);
  tool_mass_bot(robot,C_v3(0.0,0.0,0.0235),2.148);
}

void dumptofile_bot(btrobot* robot)
{
  int cnt;
  FILE *outf;
  char vect_buf1[100];
  
  outf = fopen("robotdump.txt","w");
  
  fprintf(outf,"Joint Parameters: %s",sprint_vn(vect_buf1,(vect_n*)robot->q));
  for (cnt = -2; cnt < robot->num_links + 1; cnt++){
    fprintf(outf,"Link %d \n",cnt);
    fprintf(outf,"Alpha %+8.4f, Theta %+8.4f, A %+8.4f, d %+8.4f \n",robot->links[cnt].Alpha,robot->links[cnt].Theta,robot->links[cnt].A,robot->links[cnt].D);
  }
  fclose(outf);
}


void test_btrobot()
{
  vect_3 *Cpos,*Cpoint;
  btrobot robot;
  const double pi = 3.14159;
  new_bot(&robot,4);
  
  Cpos = new_v3();
  Cpoint = new_v3();
  link_geom_bot(&robot,0,0.0,0.0,0.0,-pi/2.0);
  link_geom_bot(&robot,1,0.0,0.0,0.0,pi/2.0);
  link_geom_bot(&robot,2,0.0,0.550,0.045,-pi/2.0);
  link_geom_bot(&robot,3,0.0,0.0,-0.045,pi/2.0);
  link_geom_bot(&robot,4,0.0,0.3574,0.0,0.0);
  
  link_mass_bot(&robot,0,C_v3(0.0,0.1405,-0.0061),12.044);
  link_mass_bot(&robot,1,C_v3(0.0,-0.0166,0.0096),5.903);
  link_mass_bot(&robot,2,C_v3(-0.0443,0.2549,0.0),2.08);
  link_mass_bot(&robot,3,C_v3(0.01465,0.0,0.1308),1.135);
  link_mass_bot(&robot,4,C_v3(0.0,0.0,0.03),2.000);
  
  eval_fk_bot(&robot);
  eval_fd_bot(&robot);
  set_v3(Cpos,Ln_to_W_bot(&robot,4,Cpoint));
  print_vn((vect_n*)Cpos);
  
}


