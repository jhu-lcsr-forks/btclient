/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............playlist.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......March 18, 2003
 *  Addtl Authors ......Brian Zenowich
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
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



/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003, 2004 Barrett Technology, Inc.           *
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
