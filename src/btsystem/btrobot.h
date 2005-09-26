/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btrobot.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......24 Mar 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

#ifndef _BTROBOT_H
#define _BTROBOT_H

#include "btmath.h"


#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
/** A force applied to a link. Used internally to btrobot.

*/
typedef struct {
  vect_3 *f;  //!< Force to apply (in the world frame)
  vect_3 *t;  //!< Torque to apply (in the world frame)
  vect_3 *p; //!< Location to apply the force (in the link frame)
}btlinkforce;

//matr_h is a homogeneous transform matrix
/** Robot link definition and state maintenance object

This object is used internally to btrobot. 
*/
typedef struct {
  //physical info
  double m,Gscale; //mass, percent of gravity to use
  vect_3 *cog; //Mass center, gravity vector in base frame
  matr_3 *I; //Inertial matrix
  
  //geometry info
  double Alpha,Theta,A,D;
  int type;
  double R,P; //Joint type multipliers to avoid decision
  
  //storage for dynamic info
  matr_h* trans; //last computed transform
  matr_h* origin; //transform to origin
  vect_3 *w,*dw,*ac,*ae; //forward dynamics results
  vect_3 *f,*t,*fi; //backward dynamics results
  vect_3 *b,*z,*o,*g; //for jacobian & forward dynamics
  vect_3 *Rl,*Rm; //backward dynamics results
  vect_3 *Rp; //vector from external force point to cog
  
  double sinAlpha,cosAlpha,sinTheta,cosTheta; //last computed sin and cos for alpha,theta
  
  //force info
  int nforces;
  //btlinkforce lforces[MAX_EXTERNAL_LINK_FORCES];
  btlinkforce eforce,lastforce,tmpforce;
  
  //link envelope info
  
}btlink;
/** Main robot description and interaction object



  Notes:
    We assume that for any configuration of joint angles we will have to evaluate 
    the math for all links. All intermediate results are cached for access and 
    use by the user. So; Rather that recalculate forward kinematics for each joint
    every time the user needs it, we calculate them all in one sweep and then 
    provide the results from our calculation cache.
    
    
*/
typedef struct {
 int num_links;  //Duh
 btlink* user;  //user frame
 btlink* world;  //!< World frame - gravity is referenced from this frame
 btlink* links;  //Pointer to the start of the array of links
 btlink* tool;   //Joint frame
 vect_n *q,*dq,*ddq; //Joint state inputs
 vect_n *t;      //Joint torque outputs
 vect_3 *G;      //Gravity vector
}btrobot;

//an array of links define a robot


int new_btlink(btlink* link,int type);
int new_bot(btrobot* robot, int nlinks);

void init_4dof_wam_bot(btrobot* robot);

void link_geom_bot(btrobot* robot, int link, double theta, double d, double a, double alpha);
void link_mass_bot(btrobot* robot, int link, vect_3 *cog,double m);
void tool_geom_bot(btrobot* robot, double theta, double d, double a, double alpha);
void tool_trans_bot(btrobot* robot,  matr_h* trans);
void tool_mass_bot(btrobot* robot, vect_3 *cog,double m);

void make_transform_btlink(btlink* link,double q); //set the transform matrix
void set_gravity_bot(btrobot* robot, double Gscale); //set percent effect of gravity
btreal get_gravity_bot(btrobot* robot);
//void set_jacobian_scheme_bot(btrobot* robot,int scheme); //how to handle extra dof


void set_q_bot(btrobot* robot, vect_n* q, vect_n* dq, vect_n* ddq); //set generalized joit parameters
void eval_fk_bot(btrobot* robot); //forward kinematics
void eval_fj_bot(btrobot* robot); //forward jacobian


vect_3* Ln_to_W_bot(btrobot* robot,int link, vect_3 *position); //find world position of position specified in frame[link] 
vect_3* T_to_W_bot(btrobot* robot, vect_3 *position); //find world position of location defined in tool frame
matr_h* Ln_to_W_trans_bot(btrobot* robot,int link); //return tranform matrix
matr_h* T_to_W_trans_bot(btrobot* robot);

void fk_btwam(btrobot* robot);  //forward kinematics optimized for the wam
void fj_bot(btrobot* robot);
void eval_fd_bot(btrobot* robot); //forward dynamics
void apply_force_bot(btrobot* robot,int link, vect_3* pos, vect_3 *force, vect_3* torque);
void apply_tool_force_bot(btrobot* robot, vect_3* pos, vect_3 *force, vect_3* torque);
void eval_bd_bot(btrobot* robot); //backward kinematics
void get_t_bot(btrobot* robot,vect_n* t);

int new_linkforce(btrobot* robot,btlinkforce *force); 
int release_linkforce(btrobot* robot, btlinkforce *force);

/** 
\todo
Create a frame from 3 points in space. 
origin is first point
line between first and second point is x axis
perpindicular between x axis and third point is y axis
*/
void test_btrobot();

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTROBOT_H */

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


