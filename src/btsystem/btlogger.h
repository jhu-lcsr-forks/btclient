/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btlogger.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......18 Feb 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *   chk'd TH 051101                                                                   *
 *======================================================================*/

/** \file btlogger.h
    \brief Realtime data logging functionality

\section tmp Realtime data logging:

The btlogger object and member functions implement a double buffer data logger.
The double buffer mechanism allows data to be recorded at high speed into memory 
while writes to disk are done as efficient block operations in a low priority 
thread.


 - PrepDL(),AddDataDL(), and InitDL() are the setup functions.
 - DLon(), evalDL(), TriggerDL(), and DLoff() are the operation functions.
 - CloseDL() and DecodeDL() Are the shutdown and conversion functions.

 Typical usage is shown in the following pseudo code

\code
btlogger log;
double a,b,c[3];
void main()
{  
  //allocate fields
  PrepDL(&log,3);
  AddDataDL(&log,&a,sizeof(double),BTLOG_DOUBLE,"Arclength");
  AddDataDL(&log,&b,sizeof(double),BTLOG_DOUBLE,"Normal");
  AddDataDL(&log,c,sizeof(double)*3,BTLOG_DOUBLE,"Position");
  //initialize buffers
  InitDL(&log,1000,"logfile.dat");
  
  DLon(&log);
  while(!done){
    evalDL(&log);
  }
  DLoff(&log);
  CloseDL(&log);
}
void loop()
{
  while(1){
    a = a + b;
    c[2] = b/2;
    TriggerDL(&log);
  }
}
\endcode

*/
#ifndef _DOUBLEBUFFER_H
#define _DOUBLEBUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 

#include <stdio.h>

enum btlog_type {
  BTLOG_INT = 0,
  BTLOG_LONG,
  BTLOG_DOUBLE,
  BTLOG_LONGLONG,
  BTLOG_BTREAL
};
/** btlogger helper structure

  This structure holds info on each piece of data that the user wants to log.
  
*/
typedef struct
{
  size_t size; //!< size of the data being recorded. = sizeof(datatype)*arrayLength
  int type; //!< 0 = int, 1 = long, 2 = double 3 = long long 4 = btreal
  void *data; //!< pointer to the data start
  char name[50]; //!< null terminated string describing the data. No ','s allowed
}btdata_info;
/** Data logging object

btlogger is used for buffering data in a high speed thread and writing the data
to disk in a low speed thread. btlogger uses two buffers. It records data to one 
while the other is being written. The buffers need to be big enough that a buffer 
is completely written before the other is filled up. 



Terminology:
 - Field: One piece of data. Per field info is stored in a btdata_info structure
 - Record: One set of Fields. This represents all the data that needs to be recorded.
 
Function definitions are in btlogger.h.
\internal
\bug Add checks to make sure buffer size is large enough.
*/
typedef struct
{
  void *DL; //!< Pointer to current buffer
  void *DLbuffer1; //!< First data buffer
  void *DLbuffer2;  //!< Second data buffer
  btdata_info *data; //!< list of data information
  size_t data_size; //!< Total size of one block of data
  int fields; //!< Number of data_info pieces
  int maxfields; //!< Total number of data_info structures allocated.
  
  int DLidx; //!<
  int DLwritten;       //!< 0 all buffers clear, 1 buffer1 ready, 2 buffer2 ready
  int DLctr;
  int Log_Data;
  int Log_File_Open; 
  int buffersize;
  FILE *DLfile;
  pthread_mutex_t mutex; //!< Unused
}btlogger;

// public functions
int PrepDL(btlogger *db, unsigned int fields);
int AddDataDL(btlogger *db,void *data, int size, int type,char *name);
int InitDL(btlogger *db,int size, char *filename);

void DLon(btlogger *db);
void DLoff(btlogger *db);

void TriggerDL(btlogger *db);

void evalDL(btlogger *db);
void CloseDL(btlogger *db);

int DecodeDL(char *infile, char *outfile, int header);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _DOUBLEBUFFER_H */

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
 
