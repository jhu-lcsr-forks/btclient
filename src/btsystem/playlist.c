/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............playlist.c
 *  Author .............Traveler Hauptman  
 *  Creation Date ......March 18, 2003 
 *  Addtl Authors ......Brian Zenowich
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES: 
 *                                                             
 *  REVISION HISTORY:                                                   
 *                                           
 *                                                                      
 *======================================================================*/

/**
    playlist.c provides functions for teach and play of a WAM being controlled
    using Barrett Technologies WAMControlThread.
    
*/

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <malloc.h>
#include <stdio.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "playlist.h"

/** Allocates memory and initializes the data structures for a playlist.
*/
int MLconstruct(SC_move_list *ml, SimpleCtl *sc, int n_sc, int max_moves)
{
   //malloc maxmoves*nsc*sizeof(moveinfo)
   if ((ml->list = (move_info *)malloc(max_moves * n_sc * sizeof(move_info))) == NULL) {
      syslog(LOG_ERR, "MLconstruct: Unable to allocate memory for %d moves. So I aborted.", max_moves * n_sc);
      return -1;
   }
   if ((ml->next = (move_info *)malloc(n_sc * sizeof(move_info))) == NULL) {
      syslog(LOG_ERR, "MLconstruct: Unable to allocate memory for next move. So I aborted.", max_moves * n_sc);
      free(ml->list);
      return -1;
   }
   ml->sc = sc;
   ml->Nsc = n_sc;
   ml->MaxMoves = max_moves;
   ml->Cmove = 0;
   ml->Nmoves = 0;
   ml->play = 0;
   ml->loop = 0;
   ml->normalize = 0;
}

/** Deallocates memory used by the playlist
*/
int MLdestroy(SC_move_list *ml)
{
   free(ml->list);
   free(ml->next);
}

/** (internal) Update "next" move with the selected move data
*/
void MLloadnext(SC_move_list *ml)
{
   int cnt;
   for (cnt = 0; cnt < ml->Nsc; cnt++) {
      ml->next[cnt].pos = ml->list[(ml->Cmove) * ml->Nsc + cnt].pos;
      ml->next[cnt].vel = ml->list[(ml->Cmove) * ml->Nsc + cnt].vel;
      ml->next[cnt].acc = ml->list[(ml->Cmove) * ml->Nsc + cnt].acc;
   }
}

/** Evaluate a playlist
  
  checks to see if we have reached the next point and loads a new trajectory if we have. 
*/
void MLeval(SC_move_list *ml)
{
   int ret,cnt;
   if (ml->play) {
      ret = 0;
      for (cnt = 0; cnt < ml->Nsc; cnt++) {
         ret += ml->sc[cnt].trj.state;
      }
      if (ret == 0) //all trajectories are done running... do another.
      {

         if (MLnext(ml))  //If there aren't anymore points
         {
            MLfirst(ml);
            if (!ml->loop)
               ml->play = 0;
         }

         if (ml->Nmoves > 0)//start next trajectory
         {
            if (ml->normalize)
               MLnormalize(ml);

            for (cnt = 0; cnt < ml->Nsc; cnt++)
            {
               SCsettrjprof(&(ml->sc[cnt]), ml->next[cnt].vel, ml->next[cnt].acc);
            }
            for (cnt = 0; cnt < ml->Nsc; cnt++)
            {
               SCstarttrj(&(ml->sc[cnt]), ml->next[cnt].pos);
            }
         }

      }
   }
}

/** Add a move to the playlist.
  points should be an array of at least Nsc in size.
*/
int MLadd(SC_move_list *ml, move_info *points) //points is move_info[Nsc] in size
{
   int cnt;
   if (ml->Nmoves >= (ml->MaxMoves - 1))
      return -1; //List is full
   ml->Nmoves++;
   ml->Cmove = ml->Nmoves - 1;
   for (cnt = 0; cnt < ml->Nsc; cnt++)
   {
      ml->list[(ml->Cmove)*ml->Nsc + cnt].pos = points[cnt].pos;
      ml->list[(ml->Cmove)*ml->Nsc + cnt].vel = points[cnt].vel;
      ml->list[(ml->Cmove)*ml->Nsc + cnt].acc = points[cnt].acc;
   }
   MLloadnext(ml);
   return 0;
}

/** Add the current position in SimpleCtl to the playlist
*/
int MLaddSC(SC_move_list *ml) //points is move_info[Nsc] in size
{
   int cnt;
   if (ml->Nmoves >= (ml->MaxMoves - 1))
      return -1; //List is full
   ml->Nmoves++;
   ml->Cmove = ml->Nmoves - 1;
   for (cnt = 0; cnt < ml->Nsc; cnt++)
   {
      ml->list[(ml->Cmove)*ml->Nsc + cnt].pos = ml->sc[cnt].position;
      ml->list[(ml->Cmove)*ml->Nsc + cnt].vel = ml->sc[cnt].trj.vel;
      ml->list[(ml->Cmove)*ml->Nsc + cnt].acc = ml->sc[cnt].trj.acc;
   }
   MLloadnext(ml);
   return 0;
}

/** Delete the selected point from the playlist.
*/
int MLdel(SC_move_list *ml)
{
   int idx,cnt;
   if (ml->Nmoves <= 0)
      return -1;

   for (idx = ml->Cmove; idx < (ml->Nmoves - 1); idx++)
      for (cnt = 0; cnt < ml->Nsc; cnt++) {
         ml->list[idx*ml->Nsc + cnt].pos = ml->list[(idx + 1) * ml->Nsc + cnt].pos;
         ml->list[idx*ml->Nsc + cnt].vel = ml->list[(idx + 1) * ml->Nsc + cnt].vel;
         ml->list[idx*ml->Nsc + cnt].acc = ml->list[(idx + 1) * ml->Nsc + cnt].acc;
      }
   ml->Nmoves--;
   if ((ml->Cmove == ml->Nmoves) && (ml->Cmove > 0))
      ml->Cmove--;
   MLloadnext(ml);
   return 0;
}

/** Select the first point in the playlist
*/
int MLfirst(SC_move_list *ml)
{
   int cnt;
   ml->Cmove = 0;
   MLloadnext(ml);
   return 0;

}

/** Select the next point in the playlist
*/
int MLnext(SC_move_list *ml)
{
   int cnt;
   if (ml->Cmove < (ml->Nmoves - 1)) {
      ml->Cmove++;
      MLloadnext(ml);
      return 0;
   }
   return -1;
}

/** Select the previous point in the playlist
*/
int MLprev(SC_move_list *ml)
{
   int cnt;
   if (ml->Cmove > 0) {
      ml->Cmove--;
      MLloadnext(ml);
   }
}

/** Load a playlist from a file
*/
int MLload(SC_move_list *ml, char *filename) //{{{
{
   FILE *in;
   int cnt;
   int scCt;
   int actuator;
   int nummoves;

   nummoves = 0;
   if ((in = fopen(filename, "r")) == NULL) {
      syslog(LOG_ERR, "load_playlist:Could not open file %s", filename);
      return -1;
   }

   fscanf(in, "%d", &(nummoves));
   fscanf(in, "%d", &(scCt));
   if ((nummoves > ml->MaxMoves) || (scCt > ml->Nsc)) {
      syslog(LOG_ERR, "load_playlist:MaxMoves not big enough for number of actuators in this file or too many controllers");
      fclose(in);
      return -1;
   }

   for (cnt = 0; cnt < nummoves; cnt++) {
      for (actuator = 0; actuator < ml->Nsc; actuator++) {
         fscanf(in, "%lf, %lf, %lf", &(ml->list[cnt*ml->Nsc + actuator].pos), &(ml->list[cnt*ml->Nsc + actuator].vel), &(ml->list[cnt*ml->Nsc + actuator].acc));
      }
   }

   fclose(in);
   ml->Nmoves = nummoves;
   MLfirst(ml);
} //}}}

/** Save the playlist to a file
*/
int MLsave(SC_move_list *ml, char *filename) //{{{
{
   FILE *in;
   int cnt;
   int actuator;

   if ((in = fopen(filename, "w")) == NULL) {
      syslog(LOG_ERR, "save_playlist:Could not open file %s", filename);
      return 0;
   }

   fprintf(in, "%d\n", ml->Nmoves);
   fprintf(in, "%d\n", ml->Nsc);

   for (cnt = 0; cnt < ml->Nmoves; cnt++) {
      for (actuator = 0; actuator < ml->Nsc; actuator++) {
         fprintf(in, "%lf, %lf, %lf\n", ml->list[cnt*ml->Nsc + actuator].pos, ml->list[cnt*ml->Nsc + actuator].vel, ml->list[cnt*ml->Nsc + actuator].acc);
      }
   }
   fclose(in);
   MLloadnext(ml);
   return 1;
} //}}}

//loop throught playlist
/** Move the WAM to the selected point in the playlist.
*/
void MLone(SC_move_list *ml) //play next point
{
   int cnt;

   if (ml->Nmoves <= 0)
      return;

   if (ml->play == 0)
   {
      if (ml->normalize)
         MLnormalize(ml);

      for (cnt = 0; cnt < ml->Nsc; cnt++)
         SCsettrjprof(&(ml->sc[cnt]), ml->next[cnt].vel, ml->next[cnt].acc);

      for (cnt = 0; cnt < ml->Nsc; cnt++)
         SCstarttrj(&(ml->sc[cnt]), ml->next[cnt].pos);

      MLnext(ml);
   }
}

/** Play through the playlist once
*/
void MLplay(SC_move_list *ml) //play through playlist once
{
   if (ml->Nmoves <= 0)
      return;

   ml->Cmove = 0;
   MLloadnext(ml);
   ml->loop = 0;
   ml->play = 1;
}

/** Play through the playlist repeatedly
*/
void MLrepeatplay(SC_move_list *ml) //loop through playlist
{
   if (ml->Nmoves <= 0)
      return;

   ml->Cmove = 0;
   MLloadnext(ml);
   ml->loop = 1;
   ml->play = 1;
}

/** Check to see if we are still playing through a list
*/
int MLrunning(SC_move_list *ml)
{
   return ml->play;
}

/** Stop playing a playlist
*/
void MLstop(SC_move_list *ml)
{
   int cnt;
   ml->play = 0;
   MLfirst(ml);
   for (cnt = 0; cnt < ml->Nsc; cnt++)
      SCsetmode(&(ml->sc[cnt]), 2);
}


void MLnormalize(SC_move_list *ml)
{
   int Max=0,cnt;
   double time,maxtime,ppos,vel,acc;
   maxtime = 0.0;
   //find trajectory that will take the most time
   for (cnt = 0; cnt < ml->Nsc; cnt++) {
      time = calc_traj_time(&(ml->sc[cnt].trj), ml->sc[cnt].position, ml->next[cnt].pos,ml->next[cnt].vel,ml->next[cnt].acc);
      if (time > maxtime) {
         maxtime = time;
         acc = ml->next[cnt].acc;
         vel = ml->next[cnt].vel;
         Max = cnt;
      }
   }

   for (cnt = 0; cnt < ml->Nsc; cnt++) {
      if (cnt != Max) {
         ml->next[cnt].vel = scale_vel(&(ml->sc[cnt].trj),ml->sc[cnt].position,ml->next[cnt].pos,vel,acc,maxtime);
         ml->next[cnt].acc = scale_acc(&(ml->sc[cnt].trj),ml->sc[cnt].position,ml->next[cnt].pos,vel,acc,maxtime);
      }
   }
}



