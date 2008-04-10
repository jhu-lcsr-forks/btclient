/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btrobot.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......24 Mar 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *   '051107 TH Minimal documentation in place.                         *
 *======================================================================*/
/** \file btrobot.h
\brief Simple serial robot kinematics & dynamics math

We use DH paramaterization to define the robot. The structure of the robot is 
stored as a list of link-frames. Additionally we also have User, World, and Tool 
frames integrated for convinience.

All intermediate results are stored in the link structure. As such, the forward
kinematics are calculated in a single pass, after which, the transform and origin 
matrices of each frame are available. (transform matrix: transform from previous frame to this frame.
origin frame: transform from this frame to the origin)

examples:
Initialize a robot object:
\code
  btrobot robot;
  new_bot(&robot,2);

  link_geom_bot(&robot,0,0.0,0.0,0.0,-pi/2.0);
  link_geom_bot(&robot,1,0.0,0.0,0.0,pi/2.0);
  link_mass_bot(&robot,0,C_v3(0.0,0.1405,-0.0061),12.044);
  link_mass_bot(&robot,1,C_v3(0.0,-0.0166,0.0096),5.903);
  tool_geom_bot(&robot,0.0,0.356,0.0,0.0);
  tool_mass_bot(&robot,C_v3(0.0,0.0,0.025),2.50);
\endcode
Get the position of the end-point
\code
  while (1){
    set_q_bot(&robot,Jpos,Jvel,Jacc); 
    eval_fk_bot(&robot);  //forward kinematics
    set_v3(Cpos,T_to_W_bot(&robot,C_v3(0,0,0));
  }
\endcode
Apply a cartesian force-torque to the robot.
\code
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
  double m; //!< Link mass (kg)
  double Gscale; //!< Percent of gravity to use (normally 0.0->1.0)
  vect_3 *cog; //!< Mass center, gravity vector in base frame
  matr_mn *I; //!< Inertial matrix for this link
  vect_3 *rotorI; //!< Rotor inertia, added to the link's inertial matrix in #OpenWAM()
  
  //geometry info
  double Alpha,Theta,A,D; //!< Denavit-Hartenberg link parameters
  int type;
  double R,P; //!< Revolute, Prismatic: Joint type multipliers to avoid decision
  
  //storage for dynamic info
  matr_h* trans; //last computed transform
  matr_h* origin; //transform to origin
  matr_mn* Ro; // True 3x3 rot matrix from link to origin
  
  //forward dynamics results
  vect_3 *w; //!< Angular velocity of link frame wrt frame 0
  vect_3 *dw; //!< Angular acceleration of link frame wrt frame 0
  vect_3 *ac; //!< Acceleration of the center of mass of link
  vect_3 *ae; //!< Acceleration of end of link (i.e. origin of next frame)
  
  //backward dynamics results
  vect_3 *f; //!< Force exerted by previous link on this link (f_i in Spong 7.146 pg 277)
  vect_3 *t; //!< Torque exerted by previous link on this link
  vect_3 *fi; //!< R^i_{i+1} * f_{i+1} (intermediate value used in Spong 7.146 pg 277)
  
  //for jacobian & forward dynamics
  vect_3 *b; //!< Axis of rotation of this link in this frame
  vect_3 *z;
  vect_3 *o;
  vect_3 *g; //for jacobian & forward dynamics
  vect_3 *Rl; //!< Vector from the origin of the first link to the COM of this link (Spong: ri+1,ci)
  vect_3 *Rm; //!< Vector from the origin of this link to the COM of this link (Spong: ri,ci)
  vect_3 *Rp; //vector from external force point to cog
  vect_n *J; //<! Full Jacobian at link frame
  vect_3 *como;
  matr_mn *Jcom; //<! Full Jacobian at link center-of-mass
  matr_mn *Jvcom; //<! Upper Jacobian at link center-of-mass
  matr_mn *Jwcom; //<! Lower Jacobian at link center-of-mass
  
  double sinAlpha,cosAlpha,sinTheta,cosTheta; //last computed sin and cos for alpha,theta
  
  //force info
  int nforces;
  //btlinkforce lforces[MAX_EXTERNAL_LINK_FORCES];
  btlinkforce eforce,lastforce,tmpforce;
  
  //link envelope info
  //Sanity check
  int has_geom;
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
 int num_links;  //<! Robot link count
 btlink* user;  //user frame (not implemented yet)
 btlink* world;  //!< World frame - gravity is referenced from this frame
 btlink* links;  //<! Pointer to the start of the array of links
 btlink* tool;   //<! Joint frame
 vect_n *q,*dq,*ddq; //<! Joint state inputs
 vect_n *t;      //<! Joint torque outputs
 vect_3 *G;      //<! Gravity vector
 matr_mn *J; //<! Full Jacobian at tool
 matr_mn *Jv; //<! Upper Jacobian at tool frame
 matr_mn *Jw; //<! Lower Jacobian at tool frame
 matr_mn *Jcom; //<! Full Jacobian at tool center-of-mass
 matr_mn *Jvcom; //<! Upper Jacobian at tool center-of-mass
 matr_mn *Jwcom; //<! Lower Jacobian at tool center-of-mass
 matr_mn *M; //<! Mass (inertial) matrix
 vect_n *vv; // Row interchange scaling vector for matrix inverse
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
void fj_bot(btrobot* robot); //forward jacobian
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
 
