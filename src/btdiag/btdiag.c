/*======================================================================*
 *  Module .............btdiag
 *  File ...............btdiag.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......14 Oct 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file btdiag.c
    \brief An interactive demo of wam capabilities.
 
    Read the code to see what you can do with it.
    
This program allows a user to test and interact with the teach and play
features of the WAM library.
 
The user can switch between cartesian space and joint space. The toggle 
variable is a pointer to the present btstatecontroller.
 
Note that if a trajectory is modified, you must call 's' scale trajectory on
it to properly establish the time values.
 
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>

#include <rtai_lxrt.h>
#include <rtai_sem.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"
#include "bthaptics.h"
#include "btserial.h"
//#include "dynfcn.h"
#include "aob.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
enum{SCREEN_MAIN, SCREEN_HELP};
#define Ts (0.002)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
pthread_mutex_t disp_mutex;

int entryLine;

int useGimbals      = FALSE;
int done            = FALSE;
int quiet           = FALSE;

btstatecontrol *active_bts;
int push = 0;
btreal newt = 1;

vect_n* active_pos;
vect_n* active_dest;
vect_n* jdest,*cdest;
vect_n* active_trq;
vect_n *wv;

// Rui vars
matr_mn *Jtemp, *Ftemp;
vect_n *vn1, *vn2;
matr_mn *Js, *Ms;
vect_n *Gs;
vect_n *fstar;
vect_n *fstar_n;

btreal Lc_x, m_x, Ko_x, K2_x;
btreal Lc_y, m_y, Ko_y, K2_y;
btreal Lc_z, m_z, Ko_z, K2_z;
AOB aob_x, aob_y, aob_z;
vect_n *yk_x, *yk_y, *yk_z;

btreal rk_x, startPos_x, inputX = 0.0;
btreal rk_y, startPos_y, inputY = 0.0;
btreal rk_z, startPos_z, inputZ = 0.0;

vect_n *qVel, *qLast;
btreal fval_x, fval_y, fval_z;
btreal fsc = 1.0;

matr_mn *Jp, *Jptemp, *N, *Fptemp, *J_pseudo;
btreal Lc_xn, m_xn, Ko_xn, K2_xn;
btreal Lc_yn, m_yn, Ko_yn, K2_yn;
AOB aob_xn, aob_yn;
vect_n *yk_xn, *yk_yn;
btreal rk_xn, rk_yn;
vect_3 *p_null, *p_tool, *p_trocar, *p_init;
btreal dtr;
vect_n *torq_t, *torq_p;
matr_mn *J5, *aux, *J5temp, *Jaux2, *I7;


int null = 0;
/*
int sd;
btreal Lc;
btreal mass;
btreal rkp_x, rk_x, startPos_x, inputX = 0.0;
btreal rkp_y, rk_y, startPos_y, inputY = 0.0;
btreal rkp_z, rk_z, startPos_z, inputZ = 0.0;
matr_mn *S_x, *S_y, *S_z;
matr_mn *yk_x, *yk_y, *yk_z;
matr_mn *yhk_x, *yhk_y, *yhk_z;
matr_mn *Fc, *Fo, *Pzero, *Qnoise, *Gama, *Ca, *Kk, *L_NAOB, *Rnoise;
vect_n *qVel, *qLast, *vLast;
matr_mn *Mscalar;
btreal Ko, K2;
btreal x_force, x_forcePrev, x_forceDeriv;
btreal y_force, y_forcePrev, y_forceDeriv;
btreal z_force, z_forcePrev, z_forceDeriv;
int nout;
btreal fsc = 1.0;
 
btreal rkp_x_f, rk_x_f;
btreal rkp_y_f, rk_y_f;
btreal rkp_z_f, rk_z_f;
matr_mn *S_x_f, *S_y_f, *S_z_f;
matr_mn *yk_x_f, *yk_y_f, *yk_z_f;
matr_mn *yhk_x_f, *yhk_y_f, *yhk_z_f;
matr_mn *Fc_f, *Fo_f, *Pzero_f, *Qnoise_f, *Gama_f, *Ca_f, *Kk_f, *L_NAOB_f, *Rnoise_f;
 
btreal fval_x, fval_y, fval_z;
btreal xa, ya, za, xaf, yaf, zaf;
*/

double vel = 3.0,acc = 2.0;

int screen = SCREEN_MAIN;
int prev_mode;
char active_file[250];
char *user_def = "User edited point list";
wam_struct *wam;

vectray *vr;
via_trj_array **vta = NULL,*vt_j = NULL,*vt_c = NULL;
int cteach = 0;
extern int isZeroed;
static RT_TASK *mainTask;
btthread disp_thd,wam_thd;
double sample_rate;
btreal Jpos_filt[7];
btfilter *j5,*j6,*j7;
btfilter *yk_xfilt, *yk_yfilt, *yk_zfilt;
matr_mn *Th;

/******* Haptics *******/
btgeom_plane myplane,plane2,planes[10];
bteffect_wall mywall,wall[10];
bteffect_wickedwall mywickedwall,mywickedwall2,wickedwalls[10];
bthaptic_object myobject,myobject2,objects[20];
btgeom_sphere mysphere,mysphere2,spheres[10];
bteffect_bulletproofwall mybpwall,bpwall[10];
btgeom_box boxs[10];
bteffect_global myglobal;
vect_3 *p1,*p2,*p3,*zero_v3;
bthaptic_scene bth;
btgeom_state pstate;
//vect_3 *f1;
//vect_n *jf;

char *command_help[100];
int num_commands;
//double wamdata[8];
int NoSafety;

// Flags
int gravity = 0;
int force = 0;
int angular = 0;
int zDown = 0;
int flip = 0;

// Serial port
PORT p;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void sigint_handler();
void RenderMAIN_SCREEN(void);
void RenderHELP_SCREEN(void);
void RenderJOINTSPACE_SCREEN(void);
void RenderCARTSPACE_SCREEN(void);
void ProcessInput(int c);
void Shutdown(void);
void DisplayThread(void);
void StartDisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void init_haptics(void);
void read_keys(char *filename);
int WAMcallback(struct btwam_struct *wam);

/*==============================*
 * Functions                    *
 *==============================*/

/** Entry point for the application.
    Initializes the system and executes the main event loop.
*/
int main(int argc, char **argv)
{
   char    chr,cnt;
   int     err;
   int i, nout;
   struct sched_param mysched;
   char robotName[128];

   //f1 = new_v3();
   //jf = new_vn(4);

   //readRuiParams("MedStiffKo50AOB.txt","FastForceSensorAOB.txt");
   //readRuiParams("StiffnessBarrettFast.txt","FastForceSensorAOB.txt");
   //readRuiParams("HighStiffKo50000AOB.txt","FastForceSensorAOB.txt");
   //readRuiParams("MedStiffKo500NoActive.txt","FastForceSensorAOB.txt");
   //readRuiParams("ImpedanceBarrett.txt");
   //readRuiParams("AccelerationBarrett.txt");
   //readRuiParams("HighStiffKo50000AOB.txt","ForceSensorAOB.txt");

   nout = new_aob("ImpedanceBarrett.txt", &aob_x, &Lc_x, &m_x, &Ko_x, &K2_x);
   yk_x = new_vn(nout);

   nout = new_aob("ImpedanceBarrett.txt", &aob_y, &Lc_y, &m_y, &Ko_y, &K2_y);
   yk_y = new_vn(nout);

   nout = new_aob("ImpedanceBarrett.txt", &aob_z, &Lc_z, &m_z, &Ko_z, &K2_z);
   yk_z = new_vn(nout);

   nout = new_aob("NullSpaceBarrett.txt", &aob_xn, &Lc_xn, &m_xn, &Ko_xn, &K2_xn);
   yk_xn = new_vn(nout);

   nout = new_aob("NullSpaceBarrett.txt", &aob_yn, &Lc_yn, &m_yn, &Ko_yn, &K2_yn);
   yk_yn = new_vn(nout);

#if 0
   //usleep(3000000);
   yk_xfilt = new_btfilter(5);
   yk_yfilt = new_btfilter(5);
   yk_zfilt = new_btfilter(5);
   init_btfilter_lowpass(yk_xfilt,0.002,50,0.8);
   init_btfilter_lowpass(yk_yfilt,0.002,50,0.8);
   init_btfilter_lowpass(yk_zfilt,0.002,50,0.8);



   j5 = new_btfilter(5);
   j6 = new_btfilter(5);
   j7 = new_btfilter(5);
   init_btfilter_lowpass(j5,0.002,30,0.8);
   init_btfilter_lowpass(j6,0.002,10,0.8);
   init_btfilter_lowpass(j7,0.002,1,0.8);
   syslog_filter(j5);
   syslog_filter(j6);
   syslog_filter(j7);
#endif
   /* Figure out what the keys do and print it on screen */
   system("grep \"case '\" btdiag.c | sed 's/[[:space:]]*case \\(.*\\)/\\1/' > keys.txt");
   read_keys("keys.txt");

   /* Initialize the ncurses screen library */
   init_ncurses();
   atexit((void*)endwin);

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);
   syslog(LOG_ERR,"...Starting btdiag program...");

   /* Initialize the display mutex */
   test_and_log(
      pthread_mutex_init(&(disp_mutex),NULL),
      "Could not initialize mutex for displays.");

   for(i = 1; i < argc-1; i++) {
      if(!strcmp(argv[i],"-q"))
         quiet = TRUE;
   }

   if(!quiet) {
      /* Lead the user through a proper WAM startup */
      mvprintw(1,0,"Make sure the all WAM power and signal cables are securely");
      mvprintw(2,0,"fastened, then turn on the main power to WAM and press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
      mvprintw(4,0,"Make sure all E-STOPs are released, then press Shift-Idle");
      mvprintw(5,0,"on the control pendant. Then press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
      mvprintw(7,0,"Place WAM in its home (folded) position, then press <Enter>");
      while((chr=getch())==ERR)
         usleep(5000);
   }
#ifndef BTOLDCONFIG

   err = ReadSystemFromConfig("wam.conf");
#else //BTOLDCONFIG
#endif //BTOLDCONFIG
   /* If the robot name was given on the command line, use it */
   *robotName = 0;
   for(i = 1; i < argc-1; i++) {
      if(!strcmp(argv[i],"-n"))
         strcpy(robotName, argv[i+1]);
   }

   /* Do we want to bypass the safety circuit + pendant? */
   NoSafety = 0;
   for(i = 1; i < argc-1; i++) {
      if(!strcmp(argv[i],"-ns"))
         NoSafety = 1;
   }

   /* Initialize and get a handle to the robot */
   if(!(wam = OpenWAM("wam.conf", robotName)))
      exit(1);

   //wam->Cvel = new_vn(3);
   Jtemp = new_mn(3,wam->dof);
   qVel = new_vn(wam->dof);
   qLast =  new_vn(wam->dof);
   //vLast = new_vn(3);
   Gs = new_vn(wam->dof);
   Ms = new_mn(wam->dof,wam->dof);


   Ftemp = new_mn(3,3);
   //Ftemp = new_mn(5,5);
   //Ftemp = new_mn(2,2);
   vn1 = new_vn(wam->dof);
   vn2 = new_vn(wam->dof);
   //fstar = new_vn(5);
   fstar = new_vn(3);
   aux = new_mn(wam->dof,wam->dof);
   //fstar = new_vn(2);
   //Js = new_mn(3,wam->dof);

   Th = new_mn(4,4);

   Jp = new_mn(2,wam->dof);
   Jptemp = new_mn(2,wam->dof);
   N = new_mn(wam->dof,wam->dof);
   Fptemp = new_mn(2,2);
   J_pseudo = new_mn(3,wam->dof);
   fstar_n = new_vn(2);
   p_null = new_v3();
   p_tool = wam->Cpos;
   p_init = new_v3();
   p_trocar = new_v3();
   torq_t = new_vn(wam->dof);
   torq_p = new_vn(wam->dof);

   J5 = new_mn_ptr(wam->robot.J, 3, wam->dof, 0);
   J5temp = new_mn(3,wam->dof);
   Jaux2 = new_mn(2,wam->dof);
   I7 = new_mn(wam->dof,wam->dof);
   ident_mn(I7);
   
   /* Check and handle any additional command line arguments */
   for(i = 1; i < argc-1; i++) {
      if(!strcmp(argv[i],"-g")) // If gimbals are being used
      {
         initGimbals(wam);
         useGimbals = TRUE;
         syslog(LOG_ERR, "Gimbals expected.");
      }
   }

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   wam->logdivider = 1;
   PrepDL(&(wam->log),35);
   AddDataDL(&(wam->log),&(wam->log_time),sizeof(double),2,"Time");
   AddDataDL(&(wam->log),valptr_vn(fstar),sizeof(btreal)*3,BTLOG_BTREAL,"Fstar");
   AddDataDL(&(wam->log),&(aob_x.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_x");
   AddDataDL(&(wam->log),&(aob_y.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_y");
   AddDataDL(&(wam->log),&(aob_z.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_z");
   //AddDataDL(&(wam->log),&(S_x_f->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"SensorAOB_State");
   AddDataDL(&(wam->log),valptr_vn((vect_n*)wam->Cpos),sizeof(btreal)*3,BTLOG_BTREAL,"Cpos");
   //AddDataDL(&(wam->log),valptr_vn((vect_n*)wam->Cvel),sizeof(btreal)*3,BTLOG_BTREAL,"Cvel");
   //AddDataDL(&(wam->log),&x_force,sizeof(btreal),BTLOG_BTREAL,"x_force");
   //AddDataDL(&(wam->log),&x_forceDeriv,sizeof(btreal),BTLOG_BTREAL,"x_forceDeriv");
   //AddDataDL(&(wam->log),&x_vel,sizeof(btreal),BTLOG_BTREAL,"Xvel");
   AddDataDL(&(wam->log),valptr_vn(wam->Jpos),sizeof(btreal)*7,BTLOG_BTREAL,"Jpos");

   InitDL(&(wam->log),1000,"datafile.dat");

   if(!NoSafety) {
      /* Set the safety limits */
      setSafetyLimits(1.5, 1.5, 1.5);  // Limit to 1.5 m/s
      setProperty(0,10,TL2,FALSE,4700);
      setProperty(0,10,TL1,FALSE,1800);
   }

   /* Prepare MODE */
   jdest = new_vn(len_vn(wam->Jpos));
   cdest = new_vn(len_vn(wam->R6pos));
   wv = new_vn(7);


   active_bts = &(wam->Jsc);
   setmode_bts(active_bts,SCMODE_IDLE);
   active_pos = wam->Jpos;
   active_trq = wam->Jtrq;
   active_dest = jdest;
   prev_mode = SCMODE_IDLE;

   /* Create a new trajectory */
   vt_j = new_vta(len_vn(active_pos),50);
   vta = &vt_j;
   register_vta(active_bts,*vta);

   active_file[0] = 0;

   init_haptics();



   /* Open serial port */
   if(err = serialOpen(&p, "/dev/ttyS0")) {
      syslog(LOG_ERR, "Error opening serial port: %d", err);
   }
   serialSetBaud(&p, 9600);

   /* Spin off the WAM control thread */
   wam_thd.period = Ts;
   registerWAMcallback(wam, WAMcallback);
   btthread_create(&wam_thd, 90, (void*)WAMControlThread, (void*)wam);

   /* Spin off the display thread */
   btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);

   while (!done) {
      /* Check the active trajectory for completion */
      if (get_trjstate_bts(active_bts) == BTTRAJ_DONE && !active_bts->loop_trj) {  // BZ-16Nov2005
         stop_trj_bts(active_bts);
         setmode_bts(active_bts,prev_mode);
      }

      /* Check and handle user keypress */
      if ((chr = getch()) != ERR)
         ProcessInput(chr);
      evalDL(&(wam->log));
      usleep(100000); // Sleep for 0.1s
   }

   btthread_stop(&wam_thd); //Kill WAMControlThread
   //syslog_filter(j5);
   CloseDL(&(wam->log));
   DecodeDL("datafile.dat","dat.csv",1);
   exit(1);
}

int WAMcallback(struct btwam_struct *wam)
{
   btreal rho;
   int cnt;

   //calcMatrix(wam->dof, wam->Jpos, Ms, Gs, Js);

   // qVel = (wam->Jpos - qLast) / Ts;
   set_vn(qVel, scale_vn(1.0/Ts, add_vn(wam->Jpos, scale_vn(-1.0, qLast))));

   // wam->Cvel = Js_v * qVel;
   matXvec_mn(wam->robot.Jv, qVel, wam->Cvel);
#if 1
   // Get the location of the frame where the tool is attached in world coordinates
   set_v3(p_init, Ln_to_W_bot(&wam->robot, wam->dof-1, const_v3(p_init, 0.0, 0.0, 0.0)));

   // Find the point (p_null) on the tool shaft closest to the trocar (hole)
   rho = -dot_v3(add_v3(p_tool, neg_v3(p_trocar)), add_v3(p_init, neg_v3(p_tool)))
         / (D_Pt2Pt(p_init, p_tool) * D_Pt2Pt(p_init, p_tool));
   set_v3(p_null, add_v3(p_tool, scale_v3(rho, add_v3(p_init, neg_v3(p_tool)))));
   //set_v3(p_null, p_tool);

   // Find the shortest distance between the trocar (hole)
   // and the null space control point on the tool shaft
   dtr = D_Pt2Pt(p_null, p_trocar);

   // Find the Jacobian at p_null
   for (cnt = 0;cnt < wam->robot.num_links;cnt++)
   {
      setcol_mn(Jp, (vect_n*)cross_v3(wam->robot.links[cnt-1].z,sub_v3(p_null, wam->robot.links[cnt-1].o)), cnt);
   }
#endif
   // Find task space outputs
   yk_x->q[0] = Ko_x * (wam->Cpos->q[0] - startPos_x);
   yk_y->q[0] = Ko_y * (wam->Cpos->q[1] - startPos_y);
   yk_z->q[0] = Ko_z * (wam->Cpos->q[2] - startPos_z);

   // Find null space outputs
   yk_xn->q[0] =  (p_null->q[0] - p_trocar->q[0]);
   yk_yn->q[0] =  (p_null->q[1] - p_trocar->q[1]);

   // Calc derivative terms for task space outputs
   yk_x->q[1] = (yk_x->q[0] - yk_x->ret->q[0]) / Ts;
   yk_y->q[1] = (yk_y->q[0] - yk_y->ret->q[0]) / Ts;
   yk_z->q[1] = (yk_z->q[0] - yk_z->ret->q[0]) / Ts;

   // Calc derivative terms for null space outputs
   yk_xn->q[1] = (yk_xn->q[0] - yk_xn->ret->q[0]) / Ts;
   yk_yn->q[1] = (yk_yn->q[0] - yk_yn->ret->q[0]) / Ts;

   if(force)
   {
      // ***************************************************
      // Control
      // ***************************************************

      // Calc x,y,z control force for task space
      fstar->q[0] = rk_x - eval_aob(&aob_x, rk_x, yk_x) - (K2_x * wam->Cvel->q[0]);
      fstar->q[1] = rk_y - eval_aob(&aob_y, rk_y, yk_y) - (K2_y * wam->Cvel->q[1]);
      fstar->q[2] = rk_z - eval_aob(&aob_z, rk_z, yk_z) - (K2_z * wam->Cvel->q[2]);
      //fstar->q[2] = 0.0;
      //fstar->q[3] = 0.0;
      //fstar->q[4] = 0.0;
      //const_vn(fstar, 10.0, 0.0, 0.0);

      // Calc x,y control force for null space
      fstar_n->q[0] = rk_xn - eval_aob(&aob_xn, rk_xn, yk_xn);
      fstar_n->q[1] = rk_yn - eval_aob(&aob_yn, rk_yn, yk_yn);
      //const_vn(fstar_n, 0.0, 0.0);
   } else
   {
      const_vn(fstar, 0.0, 0.0, 0.0);
      //const_vn(fstar, 0.0, 0.0, 0.0, 0.0, 0.0);
      const_vn(fstar_n, 0.0, 0.0);
   }

   //const_vn(wam->Jtrq, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   // Get joint torque due to task space control
   // Jtorq = T_mn(J) * (inv_mn(J * inv_mn(M) * T_mn(J)) * fstar)
/*   
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jtemp, wam->robot.Jv, wam->robot.M->ret); // A <= B * C
   T_mn(wam->robot.Jv); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jtemp, wam->robot.Jv->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(wam->robot.Jv->ret, fstar->ret, torq_t);  // A * B => C
*/ 
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jtemp, J5, wam->robot.M->ret); // A <= B * C
   T_mn(J5); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jtemp, J5->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(J5->ret, fstar->ret, torq_t);  // A * B => C
 /*  
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jptemp, Jp, wam->robot.M->ret); // A <= B * C
   T_mn(Jp); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jptemp, Jp->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(Jp->ret, fstar->ret, torq_t);  // A * B => C
 */ 

   set_vn(wam->Jtrq, add_vn(wam->Jtrq, torq_t)); // Add the task torque to the Joint Torque

   if(null)
   {
      // Get joint torque due to null space control
      // J_pseudo = inv(M) * T(Jv) * inv(Jv * inv(M) * T(Jv));
      // J_pseudo(dofx3) = wam->robot.M->ret(dofxdof) * wam->robot.Jv->ret(dofx3) * Ftemp->ret(3x3)
      
      // J5(5xdof) 
      // J5->ret(dofx5)
      // Ftemp->ret(5x5) (Lambda_t)
      // J5temp(dofx5)
      // wam->robot.M->ret(dofxdof) (Mass Inverse)
      // J_pseudo(dofx5)
      // J_pseudo->ret(5xdof)
      // aux(dofxdof)
      // I7(dofxdof)
      // aux->ret(dofxdof)
      // N(dofxdof)
      // Jp(2xdof)
      // Jptemp(2xdof)
      // Jp->ret(dofx2)
      // Fptemp(2x2)
      // Fptemp->ret(2x2) (Lambda_p)
      // N->ret(dofxdof)
      // Jptemp->ret(dofx2)
      // fstar_n(2x1)
      // fstar_n->ret(2x1)
      // torq_p(dofx1)
      
      mul_mn(J5temp, J5->ret, Ftemp->ret);
      mul_mn(J_pseudo, wam->robot.M->ret, J5temp);
      // N = ident(dof) - (T(Jv) * T(J_pseudo));
      // N(dofxdof) = ident(dof) - (wam->robot.Jv->ret(dofx3) * T(J_pseudo)(3xdof))
      T_mn(J_pseudo);
      mul_mn(aux, J5->ret, J_pseudo->ret);
      scale_mn(-1.0, aux);
      
      add_mn(N, I7, aux->ret);
      //set_mn(N, T_mn(N));
      //mul_mn(J5, J_pseudo->ret, T_mn(N));
      //ident_mn(N); //Test only
      // T_posture = T(Jp * N) * inv(Jp * inv(M) * T(N) * T(Jp)) * fstar_n;
      // T_posture = (dofx2) * inv(Jp(2xdof) * inv(M)(dofxdof) * T(N)(dofxdof) * T(Jp)(dofx2))(2x2) * fstar_n(2x1);
      mul_mn(Jptemp, Jp, wam->robot.M->ret); // A <= B * C
      //mul_mn(Jaux2, Jptemp, N); // A <= B * C
      T_mn(Jp);
      mul_mn(Fptemp, Jptemp, Jp->ret); // A <= B * C
      inv_mn(Fptemp, vn1, vn2); // A->ret <= inv(A)
      //T_mn(N);
      //mul_mn(Jptemp, Jp, N->ret);
      T_mn(Jp);
      mul_mn(Jptemp->ret, N, Jp->ret);
      //mul_mn(Jptemp, Jptemp->ret, Fptemp->ret);
      mul_mn(Jptemp, Jptemp->ret, Fptemp->ret); // A * B => C
      matXvec_mn(Jptemp, fstar_n, torq_p);
   }
   set_vn(wam->Jtrq, add_vn(wam->Jtrq, torq_p)); // Add the null torque to the Joint Torque

   // Save the present outputs for next time
   set_vn(qLast, wam->Jpos);
   set_vn(yk_x->ret, yk_x);
   set_vn(yk_y->ret, yk_y);
   set_vn(yk_z->ret, yk_z);
   set_vn(yk_xn->ret, yk_xn);
   set_vn(yk_yn->ret, yk_yn);

   // Handle haptic scene
   eval_state_btg(&(pstate),wam->Cpos);
   eval_bthaptics(&bth,(vect_n*)wam->Cpos,(vect_n*)pstate.vel,(vect_n*)zero_v3,(vect_n*)wam->Cforce);
   apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce, wam->Ctrq);
   
   return 0;
}

void read_keys(char *filename)
{
   FILE *inf;
   int done = 0;
   int cnt = 0;
   int len = 100;
   int ret;

   num_commands = 0;
   inf = fopen(filename,"r");
   ret = getline(&(command_help[num_commands]),&len,inf);
   if (inf != NULL) {
      while (!done) {
         command_help[num_commands] = (char*)btmalloc(100);
         ret = getline(&(command_help[num_commands]),&len,inf);
         if (ret == -1 || num_commands > 98)
            done = 1;
         else
            command_help[num_commands][ret-1] = 0;
         num_commands++;
      }
   }
   fclose(inf);
   inf = fopen("test.out","w");
   for(cnt = 0;cnt < num_commands;cnt++)
      fprintf(inf,"%s",command_help[cnt]);
   fclose(inf);

}
void init_haptics(void)
{
   int cnt;
   btreal xorig,yorig,zorig;
   int objectCount = 0;

   p1 = new_v3();
   p2 = new_v3();
   p3 = new_v3();
   xorig = 0.0;
   yorig = 0.0;
   zorig = 0.10;

   new_bthaptic_scene(&bth,10);
   init_state_btg(&pstate,0.002,30.0);

   // Create workspace bounding box
   init_bx_btg(&boxs[0],const_v3(p1,0.7,0.0,zorig+0.0),const_v3(p2,0.7,0.01,zorig+0.0),const_v3(p3,0.7,0.0,zorig+0.01),1.0,0.6,0.4,1);
   init_bulletproofwall(&bpwall[0],0.0,0.0,0.05,4000.0,10.0,10.0);
   init_normal_box_bth(&objects[objectCount],&boxs[0],(void*)&bpwall[0],bulletproofwall_nf);
   addobject_bth(&bth,&objects[objectCount++]);

   // Create nested spheres
   init_sp_btg( &spheres[0],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //  init_sp_btg( &spheres[1],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.42,0.0,zorig+0.0),1);
   init_sp_btg( &spheres[2],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.3,0.0,zorig+0.0),0);
   init_sp_btg( &spheres[3],const_v3(p1,0.5,0.0,zorig+0.0),const_v3(p2,0.32,0.0,zorig+0.0),1);
   init_wickedwall(&wickedwalls[0],500.0, 2.0,0.5,0.1,0.01);
   init_normal_sphere_bth(&objects[objectCount],&spheres[0],(void*)&wickedwalls[0],wickedwall_nf);
   addobject_bth(&bth,&objects[objectCount++]);
   for(cnt = 2;cnt < 4;cnt++) {
      init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);
      init_normal_sphere_bth(&objects[objectCount],&spheres[cnt],(void*)&wickedwalls[cnt],wickedwall_nf);
      addobject_bth(&bth,&objects[objectCount++]);
   }

   const_v3(wam->Cpoint, 0.0, 0.0, 0.0);
}

/* Initialize the ncurses screen library */
void init_ncurses(void)
{
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
}

/** Traps the Ctrl-C signal.
    Quits the program gracefully when Ctrl-C is hit.
*/
void sigint_handler()
{
   exit(1);
}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
   int cnt,err;

   clear();
   refresh();
   while (!done) {
      test_and_log(
         pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
      switch(screen) {
      case SCREEN_MAIN:
         RenderMAIN_SCREEN();
         break;
      case SCREEN_HELP:
         RenderHELP_SCREEN();
         break;
      }
      pthread_mutex_unlock(&(disp_mutex));
      usleep(100000);
   }

}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
   int err;
   test_and_log(
      pthread_mutex_lock(&(disp_mutex)),"Display mutex failed");
   move(entryLine, 1);
   echo();
   timeout(-1);
}

/** Unlocks the display mutex.
    Allows the computer to resume automatically updating the screen.
*/
void finish_entry()
{
   noecho();
   timeout(0);
   move(entryLine, 1);
   addstr("                                                                              ");
   refresh();
   pthread_mutex_unlock( &(disp_mutex) );
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderMAIN_SCREEN()
{
   int cnt, idx, Mid, cp;
   int line, line2;
   int cpt, nrows;
   double gimb[4],tacc,tvel;
   vectray* vr;
   char vect_buf1[2500];

   /***** Display the interface text *****/
   line = 0;

   mvprintw(line , 0, "Barrett Technology - Diagnostic Application\t\tPress 'h' for help");
   line+=2;

   // Show MODE
   if (active_bts == &(wam->Jsc)) {
      mvprintw(line, 0, "Mode       : Joint Space    ");
   } else if (active_bts == &(wam->Csc)) {
      mvprintw(line, 0, "Mode       : Cartesian Space");
   } else {
      mvprintw(line, 0, "Mode       : Undefined!!!   ");
   }
   ++line;

   // Show CONSTRAINT
   if (getmode_bts(active_bts)==SCMODE_IDLE)
      mvprintw(line, 0, "Constraint : IDLE      ");
   else if (getmode_bts(active_bts)==SCMODE_POS)
      mvprintw(line, 0, "Constraint : POSITION  ");
   else if (getmode_bts(active_bts)==SCMODE_TRJ)
      mvprintw(line, 0, "Constraint : TRAJECTORY");
   else
      mvprintw(line, 0, "Constraint : UNDEFINED!");
   ++line;

   if (bth.state) {
      mvprintw(line, 0, "Haptics    : ON    ");
   } else {
      mvprintw(line, 0, "Haptics    : OFF    ");
   }
   line+=2;

   // Show TRAJECTORY
   if (cteach)
      mvprintw(line, 0, "Trajectory : Teaching continuous trajectoy");
   else if (*vta == NULL)
      mvprintw(line, 0, "Trajectory : NONE                         ");
   else
      mvprintw(line, 0, "Trajectory : %s                           ",*active_file?active_file:"NONE");
   ++line;
   mvprintw(line, 0, "Velocity   : %+8.4f  ",vel);
   ++line;
   mvprintw(line, 0, "Accel      : %+8.4f  ",acc);
   ++line;
   mvprintw(line, 0, "Destination: %s ",sprint_vn(vect_buf1,active_dest));
   line+=2;

   mvprintw(line, 0, "Position   : %s ", sprint_vn(vect_buf1,active_pos));
   ++line;
   //mvprintw(line, 0, "Target     : %s ", sprint_vn(vect_buf1,active_bts->qref));
   //++line;
   mvprintw(line, 0, "%s     : %s ", (active_bts == &(wam->Jsc)) ? "Torque" : "Force ", sprint_vn(vect_buf1,active_trq));
   line+=2;

   if (*vta != NULL) { // print current point
      vr = get_vr_vta(*vta);
      cpt = get_current_idx_vta(*vta);
      nrows = numrows_vr(vr);
      mvprintw(line,0,"Teach Point: %d of %d      ",cpt,nrows-1);
      line++;

      mvprintw(line  , 0 , "Previous   :\t\t\t\t\t\t\t\t\t\t");
      mvprintw(line+1, 0 , "Current    :\t\t\t\t\t\t\t\t\t\t");
      mvprintw(line+2, 0 , "Next       :\t\t\t\t\t\t\t\t\t\t");

      // Previous
      if (nrows > 0 && cpt > 0)
         mvprintw(line, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));

      // Current
      if (nrows > 0) {
         if (nrows != cpt)
            mvprintw(line+1, 13 , "%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt)));
         else
            mvprintw(line+1, 13 , "END OF LIST");
      } else
         mvprintw(line+1, 13 , "EMPTY LIST");

      // Next
      if (nrows > 1)
         if (cpt < nrows-1)
            mvprintw(line+2, 13,"%s ", sprint_vn(vect_buf1,idx_vr(vr,cpt+1)));
         else if (cpt == nrows-1)
            mvprintw(line+2, 13, "END OF LIST");
      line += 3;
   } else {
      line++;
      line++;
      mvprintw(line, 0 ,   "No Playlist loaded. [l] to load one from a file, [n] to create a new one.");
      line += 2;
   }
  
   
/*
   mvprintw(line, 0, "p_tool:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_tool));
   line+= 1;

   mvprintw(line, 0, "p_trocar:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_trocar));
   line+= 1;

   mvprintw(line, 0, "p_init:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_init));
   line+= 1;

   mvprintw(line, 0, "p_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_null));
   line+= 1;

   mvprintw(line, 0, "dtr:");
   line++;
   mvprintw(line, 0, "%+8.4f", dtr);
   line+= 1;
   

   mvprintw(line, 0, "Jacobian @ p_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Jp));
   line+= 3;

   mvprintw(line, 0, "lam_p * fp:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar_n->ret));
   line+= 1;
   mvprintw(line, 0, "lam_t * ft:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar->ret));
   line+= 1;


   mvprintw(line, 0, "fstar_t:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar));
   line+= 1;

   mvprintw(line, 0, "fstar_n:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar_n));
   line+= 1;

   mvprintw(line, 0, "tau_task:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, torq_t));
   line+= 1;

   mvprintw(line, 0, "tau_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, torq_p));
   line+= 1;

   mvprintw(line, 0, "LNAOB:");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_xn.L_NAOB));
   line+= 1;

   mvprintw(line, 0, "yk_x");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, yk_x));
   line+= 1;

   mvprintw(line, 0, "State:x");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_x.S));
   line+= 4;

      mvprintw(line, 0, "L_NAOB * S:");
      line++;
      mvprintw(line, 0, "%+8.4f", aob_xn.Mscalar->q[0]);
      line+= 1;
   

      mvprintw(line, 0, "Lambda_task:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Ftemp->ret));
      line+= 5;
   
      mvprintw(line, 0, "Lambda_null:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Fptemp->ret));
      line+= 2;
      
      mvprintw(line, 0, "J5 * Jpseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aux));
      line+= 7;
      
      mvprintw(line, 0, "Jacobian Matrix:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J5->ret));
      line+= 7;
      
      mvprintw(line, 0, "Jacobian Pseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J_pseudo->ret));
      line+= 7;

      mvprintw(line, 0, "J_pseudoT * NT:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J5));
      line+= 3;

      mvprintw(line, 0, "N:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, N));
      line+= 7;

      mvprintw(line, 0, "J_pseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J_pseudo));
      line+= 3;
    

      mvprintw(line, 0, "Jacobian Matrix (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Js));
      line+= 4;
    
      mvprintw(line, 0, "Jacobian Matrix (com):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Jcom));
      line+= 7;
    
      mvprintw(line, 0, "Jacobian Matrix (vcom):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.Jvcom));
      line+= 4;
    
      mvprintw(line, 0, "Jacobian Matrix (w):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.Jw));
      line+= 4;
    
      mvprintw(line, 0, "fval_z:");
      line++;
      mvprintw(line, 0, "%+8.4f", fval_z);
      line+= 1;
    
      mvprintw(line, 0, "nout:");
      line++;
      mvprintw(line, 0, "%d", nout);
      line+= 1;
    
      mvprintw(line, 0, "mass:");
      line++;
      mvprintw(line, 0, "%+8.4f", mass);
      line+= 1;
    
      mvprintw(line, 0, "yk_z:");
      line++;
      mvprintw(line, 0, "%+8.4f", yk_z->q[0]);
      line+= 1;
    
        for(cnt = 0; cnt < wam->dof; cnt++){
              mvprintw(line, 0, "I(%d):", cnt+1);
              line++;
              mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[cnt].I));
              line+= 3;
              mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)wam->robot.links[cnt].rotorI));
              line++;
        }
      
      mvprintw(line, 0, "Ro(4):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Ro));
      line+= 4;
    
    
      mvprintw(line, 0, "Th (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Th));
      line +=5;
    
      mvprintw(line, 0, "Th:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Ro));
      line +=5;
      
      mvprintw(line, 0, "Mass Matrix:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.M->ret));
      line +=7;
       
      mvprintw(line, 0, "Mass Matrix (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Ms));
      line +=7;
      
      mvprintw(line, 0, "Mass Matrix (diff):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, add_mn(wam->robot.M->ret, wam->robot.M, scale_mn(-1.0, Ms))));
      line +=7;
     
      mvprintw(line, 0, "Kk_f:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Kk_f));
      line +=5;
      
      //mvprintw(line, 0, "yk_x:");
      //line++;
      //mvprintw(line, 0, "%s", sprint_mn(vect_buf1, yk_x));
      //line+= 4;
      
      mvprintw(line, 0, "vLast:");
      line++;
      mvprintw(line, 0, "%s", sprint_vn(vect_buf1, vLast));
      line+= 1;
    
      mvprintw(line, 0, "State Vector (S_x_f):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, S_x_f));
      line+= 5;
     
      mvprintw(line, 0, "State Vector (S_x):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_x.S));
      line+= 5;
     
      //mvprintw(line, 0, "L_NAOB*S (z):");
      //line++;
      //mvprintw(line, 0, "%+8.4f", Mscalar->q[0]);
      //line+= 1;
      
      mvprintw(line, 0, "fstar:");
      line++;
      mvprintw(line, 0, "%s", sprint_vn(vect_buf1, fstar));
      line+= 1;
    
      mvprintw(line, 0, "Cvel:");
      line++;
      mvprintw(line,0,"%s",sprint_vn(vect_buf1,wam->Cvel));
      line++;
    
    
      mvprintw(line, 0, "Gravity torques (symb):");
      line++;
      mvprintw(line,0,"%s",sprint_vn(vect_buf1,Gs));
      line++;
   */
   line++;
   mvprintw(line, 0, "Callback time (ms):");
   line++;
   mvprintw(line,0,"%.4f",wam->user_time/1000000.0);
   line+=2;
   /*
   mvprintw(line,0,"%s",sprint_vn(vect_buf1,jf));

   mvprintw(line, 0, "O Matrix:");
   line++;
   mvprintw(line, 0, "%s", sprint_v3(vect_buf1, wam->robot.links[3].o));

   line += 2;
   mvprintw(line, 0, "Z Matrix:");
   line++;
   mvprintw(line, 0, "%s", sprint_v3(vect_buf1, wam->robot.links[3].z));

   */
   /*  line += 1;
     mvprintw(line,0,"bts: state:%d",active_bts->mode);
     mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
     line += 1;
   */
   entryLine = line;
   refresh();
}

void RenderHELP_SCREEN()
{
   int cnt, line = 0;

   mvprintw(line, 0, "Help Screen - (press 'h' to toggle)");
   line += 2;
   for (cnt = 0; cnt < num_commands;cnt++) {
      if (cnt % 2) {
         mvprintw(line,40,"%.39s",command_help[cnt]);
         line += 1;
      } else {
         mvprintw(line,0,"%.39s",command_help[cnt]);

      }
   }
   refresh();
}
void clearScreen(void)
{
   btmutex_lock(&(disp_mutex));
   clear();
   btmutex_unlock(&(disp_mutex));
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
   int cnt,elapsed = 0;
   double ftmp,tacc,tvel;
   int dtmp,status;

   char fn[250],chr;
   int ret;
   int done1;
   btreal zPos;

   switch (c)
   {

   case 'x'://eXit
   case  'X'://eXit
      done = 1;
      break;
   case '!'://Set puck to mode TORQ
      if (NoSafety)
         EnergizeActuators();
      //setProperty(0, 1, MODE, FALSE, 2);
      break;
   case '@'://Set puck to mode IDLE
      if (NoSafety)
         IdleActuators();
      //setProperty(0, 1, MODE, FALSE, 0);
      break;
   case '#'://Data logging on
      DLon(&(wam->log));
      break;
   case '3'://Data logging off
      DLoff(&(wam->log));
      break;
 /*  case 'z'://Send home-position to WAM
      const_vn(wv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //gimbals
      DefineWAMpos(wam,wv);
      break;
  */ case '[':
      fsc *= 0.5;
      break;
   case ']':
      fsc *= 2.0;
      break;
   case 't':// Set trocar
      set_v3(p_trocar, wam->Cpos);
      null = 1;
      break;
   case 'z':// Toggle rk
      //if(inputZ < 0.001)
      //   inputZ = -2.5;
      //else
      //   inputZ = 0.0;
      inputZ += -1.0;

      rk_z = Lc_z * inputZ;

      break;
   case 'r':// Toggle rk
      //if(inputX < 0.001)
      //   inputX = 5.0;
      //else
      //   inputX = 0.0;
      inputX += 2.5;

      rk_x = Lc_x * inputX;

      break;
   case 'f'://Toggle force
      //if(!force)
      //   DLon(&(wam->log));
      //else
      //   DLoff(&(wam->log));

      //usleep(2000);

      // Set rk = rkp = x0
      rk_x = rk_y = rk_z = 0.0;
      rk_xn = rk_yn = 0.0;

      //rk_x_f = rkp_x_f = 0.0;
      //rk_y_f = rkp_y_f = 0.0;
      //rk_z_f = rkp_z_f = 0.0;

      // Set S->q[0] = rk
      zero_mn(aob_x.S);
      zero_mn(aob_y.S);
      zero_mn(aob_z.S);
      zero_mn(aob_xn.S);
      zero_mn(aob_yn.S);
      //zero_mn(S_y);
      //zero_mn(S_z);

      //zero_mn(S_x_f);
      //zero_mn(S_y_f);
      //zero_mn(S_z_f);

      startPos_x = wam->Cpos->q[0];
      startPos_y = wam->Cpos->q[1];
      startPos_z = wam->Cpos->q[2];

      yk_x->ret->q[0] = 0.0;
      yk_y->ret->q[0] = 0.0;
      yk_z->ret->q[0] = 0.0;
      yk_xn->ret->q[0] = 0.0;
      yk_yn->ret->q[0] = 0.0;

      /*
            x_forcePrev = 0.0;
            y_forcePrev = 0.0;
            z_forcePrev = 0.0;
            
            xaf = yaf = zaf = 0.0;
      */
      fval_x = fval_y = fval_z = 0.0;

      fsc = 1.0;

      force = !force;
      break;
   case 'g'://Set gravity compensation
      //gravity = !gravity;
      start_entry();
      addstr("Enter scale value for gravity (1.0 = 9.8m/s^2): ");
      refresh();
      scanw("%lf\n",  &tvel);
      SetGravityComp(wam,tvel);
      finish_entry();
      break;
   case '_'://Refresh display
      clearScreen();
      break;
   case '\t'://Toggle jointspace and cartesian space
      destroy_vta(vta); //empty out the data if it was full
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      setmode_bts(&(wam->Csc),SCMODE_IDLE);

      if (active_bts == &(wam->Jsc)) { //switch to cartesian space mode.
         SetCartesianSpace(wam);
         active_bts = &(wam->Csc);
         active_pos = wam->R6pos;
         active_trq = wam->R6force;
         active_dest = cdest;
         vta = &vt_c;
      } else {
         SetJointSpace(wam);
         active_bts = &(wam->Jsc);
         active_pos = wam->Jpos;
         active_trq = wam->Jtrq;
         active_dest = jdest;
         vta = &vt_j;
      }
      clearScreen();
      break;
   case 'D'://Haptics on
      bth.state = 1;
      break;
   case 'd'://Haptics off
      bth.state = 0;
      break;
   case 'p'://Turn on/off Constraint
      if (getmode_bts(active_bts)!=SCMODE_IDLE)
         setmode_bts(active_bts,SCMODE_IDLE);
      else
         setmode_bts(active_bts,SCMODE_POS);
      break;

   case '.'://Play loaded trajectory

      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         moveparm_bts(active_bts,vel,acc);
         active_bts->loop_trj = 0;
         prev_mode = getmode_bts(active_bts);
         if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
         start_trj_bts(active_bts);
      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;

   case 'b'://Simulate loaded trajectory

      sim_vta(*vta,0.002,getval_vn(idx_vr(get_vr_vta(*vta),numrows_vr(get_vr_vta(*vta))-1),0),"sim.csv");
      break;

   case '?'://Loop loaded trajectory

      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         moveparm_bts(active_bts,vel,acc);
         active_bts->loop_trj = 1;
         prev_mode = getmode_bts(active_bts);
         if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);
         start_trj_bts(active_bts);
      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;

   case '/'://Stop loaded trajectory

      stop_trj_bts(active_bts);
      setmode_bts(active_bts,prev_mode);
      active_bts->loop_trj = 0;
      break;
   case 'Y'://Start continuos teach
      if (active_bts == &(wam->Jsc))
         StartContinuousTeach(wam,1,25,"teachpath");
      else
         StartContinuousTeach(wam,0,25,"teachpath");
      cteach = 1;
      break;
   case 'y'://Stop continuos teach
      StopContinuousTeach(wam);
      DecodeDL("teachpath","teach.csv",0);
      cteach = 0;
      stop_trj_bts(active_bts);
      /** \internal \todo sleeps that are necessary might be forgotten. Can we eliminate the need?*/
      usleep(10000); //needed to give the command a chance to work.
      destroy_vta(vta); //empty out the data if it was full
      strcpy(active_file,"teach.csv");
      *vta = read_file_vta(active_file,20);
      register_vta(active_bts,*vta);
      break;
   case 'l'://Load trajectory from file
      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", active_file);
         destroy_vta(vta); //empty out the data if it was full

         *vta = read_file_vta(active_file,20);
         register_vta(active_bts,*vta);
         finish_entry();
      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;
   case 'w'://Save trajectory to a file
      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter filename for trajectory: ");
         refresh();
         scanw("%s", fn);
         if (*vta != NULL) {
            write_file_vta(*vta,fn);
            strcpy(active_file,fn);
         }
         finish_entry();
      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;
   case 'n'://Create a new trajectory
      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter the max number of points that will be in your trajectory: ");
         refresh();
         ret = scanw("%d", &dtmp);
         destroy_vta(vta);

         strcpy(active_file,user_def);
         *vta = new_vta(len_vn(active_pos),dtmp);
         register_vta(active_bts,*vta);
         active_file[0] = 0;
         finish_entry();
      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;
   case 'N'://Toggle angular hold
      if(angular) {
         angular = 0;
         stop_btPID(&(wam->pid[3]));
      } else {
         angular = 1;
         set_q(wam->qref,wam->qact);
         start_btPID(&(wam->pid[3]));
      }
      break;
   case 'M'://Move to a location
      if(getmode_bts(active_bts)!=SCMODE_TRJ) {
         start_entry();
         addstr("Enter comma seperated destination \".2,.4,...\": ");
         refresh();
         getstr( fn);
         strcat(fn,"\n");
         //syslog(LOG_ERR,"Moveto:%s",fn);
         finish_entry();
         fill_vn(active_dest,0.25);
         csvto_vn(active_dest,fn);
         moveparm_bts(active_bts,vel,acc);
         prev_mode = getmode_bts(active_bts);
         if (prev_mode != SCMODE_POS)
            setmode_bts(active_bts,SCMODE_POS);

         if(moveto_bts(active_bts,active_dest))
            syslog(LOG_ERR,"Moveto Aported");

      } else {
         start_entry();
         addstr("You must stop the running trajectory first!: ");
         refresh();
         sleep(1);
         finish_entry();
      }
      break;
      /*'m': Move to the presently selected trajectory point*/

   case '<'://Select next trajectory point
      prev_point_vta(*vta);
      break;
   case '>'://Select previous trajectory point
      next_point_vta(*vta);
      break;
   case '+'://Insert a point in the trajectory
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         ins_point_vta(*vta,active_pos);
      }
      break;
   case '-'://Remove a point in the trajectory
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         del_point_vta(*vta);
      }
      break;
   case 's'://Adjust trj times using a velocity
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter trajectory velocity: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         if(*vta != NULL)
            dist_adjust_vta(*vta,tvel);
         finish_entry();
      }
      break;
   case 'S'://Scale trajectory in time
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter scale factor: ");
         refresh();
         ret = scanw("%lf\n", &tvel);
         if(vta != NULL)
            time_scale_vta(*vta,tvel);
         finish_entry();
      }
      break;
   case 'A'://Set the corner acceleration
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Corner Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &tacc);
         if(*vta != NULL)
            set_acc_vta(*vta,tacc);
         finish_entry();
      }
      break;
   case 'a'://Set the move acceleration
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Acceleration: ");
         refresh();
         ret = scanw("%lf\n", &acc);
         finish_entry();
      }
      break;
   case 'v'://Set the move velocity
      if(getmode_bts(active_bts)==SCMODE_IDLE) {
         start_entry();
         addstr("Enter Move Velocity: ");
         refresh();
         ret = scanw("%lf\n", &vel);
         finish_entry();
      }
      break;
   case ','://Pause/Unpause trajectory
      status =  movestatus_bts(active_bts);
      if (status == BTTRAJ_PAUSING || status == BTTRAJ_PAUSED)
         unpause_trj_bts(active_bts,2);
      else
         pause_trj_bts(active_bts,2);
      break;
   case 'h'://Toggle Help
      clearScreen();
      screen = !screen;
      break;

   case 27://Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR) {
         if (chr == 91) {
            if ((chr = getch()) != ERR) {
               if (chr == 67) //Right arrow
               {
               } else if (chr == 68) //Left arrow
               {
               } else {
                  while(getch()!=ERR) {
                     // Do nothing
                  }
                  syslog(LOG_ERR,"Caught unknown keyhit 27-91-%d",c);
               }
            }
         } else {
            while(getch()!=ERR) {
               // Do nothing
            }
            syslog(LOG_ERR,"Caught unknown keyhit 27-%d",c);
         }
      }
      break;

   default:
      while(getch()!=ERR) {
         // Do nothing
      }
      syslog(LOG_ERR,"Caught unknown keyhit %d",c);

      break;
   }
}

#if 0
void readRuiParams(char *aobControl)
{
   FILE *inp;
   int i;
   char buf[2000];

   // Rui vars

   Ftemp = new_mn(3,3);
   vn1 = new_vn(7);
   vn2 = new_vn(7);
   fstar = new_vn(3);
   Js = new_mn(3,4);

   if((inp = fopen(aobControl, "r")) == NULL) {
      printf("\nCould not open file: %s\n", aobControl);
      exit(0);
   }
   fscanf(inp, "%*s%d %*s%d %*s%lf %*s%lf %*s%lf %*s%lf", &sd, &nout, &mass, &Lc, &Ko, &K2);

   // State vars
   S_x = new_mn(sd, 1);
   S_y = new_mn(sd, 1);
   S_z = new_mn(sd, 1);

   yk_x = new_mn(nout,1);
   yk_y = new_mn(nout,1);
   yk_z = new_mn(nout,1);
   yhk_x = new_mn(nout, 1);
   yhk_y = new_mn(nout, 1);
   yhk_z = new_mn(nout, 1);

   Mscalar = new_mn(1,1);

   Fc = new_mn(sd, sd);
   Fo = new_mn(sd, sd);
   Pzero =  new_mn(sd, sd);
   Qnoise =  new_mn(sd, sd);
   Gama = new_mn(sd, 1);
   Ca = new_mn(nout, sd);
   Kk = new_mn(sd, nout);
   L_NAOB = new_mn(1, sd);
   Rnoise = new_mn(nout,nout);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &Fo->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &Fc->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*nout); i++)
      fscanf(inp, "%lf", &Ca->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &Pzero->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &Qnoise->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (nout*nout); i++)
      fscanf(inp, "%lf", &Rnoise->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < sd; i++)
      fscanf(inp, "%lf", &L_NAOB->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < sd; i++)
      fscanf(inp, "%lf", &Gama->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*nout); i++)
      fscanf(inp, "%lf", &Kk->q[i]);

   fclose(inp);

   //usleep(10000000);
}
#endif

