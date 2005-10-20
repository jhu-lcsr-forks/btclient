/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............playlist.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......March 18, 2003
 *  Addtl Authors ......Brian Zenowich
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation (version 2).
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
 *
 *  NOTES:   
 *                                                           
 *  REVISION HISTORY:                                                   
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *                                           
 *                                                                      
 *======================================================================*/

#ifndef _PLAYLIST_H
#define _PLAYLIST_H

#include "btjointcontrol.h"

/** Data that defines one leg of a trajectory through multiple points.
*/
typedef struct
{
  double pos;
  double vel;
  double acc;
}
move_info;
/** Data to maintain and process a playlist of points.

*/
typedef struct
{
  move_info *list;  /**< Array of move_info arrays */
  int Nmoves;       /**< Number of moves currently stored in the playlist */
  int MaxMoves;     /**< Maximum number of moves allowed for this playlist */
  int Cmove; /**< array index of the selected point starting with 0 */
  move_info *next; /**< A move_info array that contains the selected leg. */
  int loop; /**< 0: don't loop, 1: loop */
  int normalize; /**< if true, scale all trajectory accelerations and velocities to match slowest*/
  int play; /**< 0:stopped 1:play back trajectories */
  SimpleCtl *sc; /**< Pointer to the user created array of simple controllers */
  int Nsc;       /**< Number of simple controllers (number of motors or joints) */
}SC_move_list;

int MLcontruct(SC_move_list *ml, SimpleCtl *sc, int n_sc, int max_moves);
int MLdestroy(SC_move_list *ml);
void MLeval(SC_move_list *ml);
void MLloadnext(SC_move_list *ml);
int MLadd(SC_move_list *ml, move_info *points); //points is move_info[Nsc] in size
int MLaddSC(SC_move_list *ml); 
int MLdel(SC_move_list *ml);
int MLfirst(SC_move_list *ml);
int MLnext(SC_move_list *ml);
int MLprev(SC_move_list *ml);
int MLload(SC_move_list *ml, char *filename);
int MLsave(SC_move_list *ml, char *filename);
void MLone(SC_move_list *ml);
void MLplay(SC_move_list *ml); //play through playlist once
void MLrepeatplay(SC_move_list *ml); //loop through playlist
int MLrunning(SC_move_list *ml);
void MLstop(SC_move_list *ml);
void MLnormalize(SC_move_list *ml);


#endif /* _PLAYLIST_H */



