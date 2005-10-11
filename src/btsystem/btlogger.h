/*======================================================================*
 *  Module .............libbt
 *  File ...............btmath.h
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

/** Data logger data structures
There are two buffers and a pointer to the current one with
and index and a state variable to determine whether the
other buffer has been written.
*/
#ifndef _DOUBLEBUFFER_H
#define _DOUBLEBUFFER_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {BTLOG_INT = 0,BTLOG_LONG,BTLOG_DOUBLE,BTLOG_LONGLONG,BTLOG_BTREAL};
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

\bug Add checks to make sure buffer size is large enough.

Terminology:
 - Field: One piece of data. Per field info is stored in a btdata_info structure
 - Record: One set of Fields. This represents all the data that needs to be recorded.
 
Function definitions are in btlogger.c.
*/
typedef struct
{
  void *DL,*DLbuffer1,*DLbuffer2;  //!< Buffers and buffer pointer
  
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

// Private functions

void InitDataFileDL(btlogger *db);
int DataSizeDL(btlogger *db);
void UpdateDL(btlogger *db);
void flushDL(btlogger *db);
void DLwrite(btlogger *db);

// public functions
int PrepDL(btlogger *db, int fields);
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
