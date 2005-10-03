/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btlogger.c
 *  Author .............Traveler Hauptman   
 *  Creation Date ......April 3, 2003
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      
 *  NOTES:                                                              
 *   Uses some code & concepts developed at Nortwestern University 
 *   by Traveler Hauptman                                                                      
 *  REVISION HISTORY:                                                   
 *                                           
 *                                                                      
 *======================================================================*/

/** \file btlogger.c
    Realtime data logging functionality

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

\bug the code demo does not work yet
*/
#include <syslog.h>
#include "btlogger.h"
#include <stdio.h>
#include <stdlib.h>
#include "btmath.h"
/*****************************************************************************/
/** Allocate a specified number of fields
You can add fewer fields than specified by PrepDL() but you cannot AddDataDL() 
after calling InitDL().
 
\param db Pointer to the btlogger structure
\param fields The number of fields you want to record
*/
int PrepDL(btlogger *db, int fields)
{
  db->data = btmalloc(fields * sizeof(btdata_info));
  
  db->fields = 0;
  db->maxfields = fields;
  return 0;
}
/** Adds a field to the list of data to log
\bug Check to see if we have already initialized with InitDL()
 
\param size Size of data = Array_length * sizeof(type)
\return 0 = Success, -1 = no more fields available
*/
int AddDataDL(btlogger *db,void *data, int size, int type,char *name)
{
  int idx;
  idx = db->fields;
  if (idx <= db->maxfields)
  {
    db->data[idx].size = size;
    db->data[idx].data = data;
    db->data[idx].type = type;
    strcpy(db->data[idx].name,name);
    db->fields++;
    return 0;
  }
  return -1;
}
/** (internal use) Counts up how much data is being stored */
int DataSizeDL(btlogger *db)
{
  int total,cnt;
  total = 0;
  for(cnt = 0; cnt < db->maxfields;cnt++)
    total += db->data[cnt].size;
  return total;
}
/** Initialize the buffers and prepare for logging
 
Don't forget to call PrepDL() and AddDataDL() first to define what fields will
be recorded.
 
\param size The number of records the buffer will hold
\param filename The path and filename where you want to write to
\return 0 = Success, -1 = Could not open file, -2 = Could not allocate first buffer, -3 = Could not allocate second buffer
*/

int InitDL(btlogger *db,int size, char *filename)
{
  int datasize;

  db->Log_Data = 0;
  db->DLwritten = 0;
  db->Log_File_Open = 0;
  db->DLctr = 0;
  db->DLidx = 0;
  if ((db->DLfile = fopen(filename,"w"))==NULL)
  {
    syslog(LOG_ERR,"InitDL:Could not open datalogger file %s",filename);
    return -1;
  }
  db->Log_File_Open = 1;
  db->buffersize = size;
  datasize = DataSizeDL(db);
  db->data_size = datasize;
  db->DLbuffer1 = btmalloc(size * datasize);
  db->DLbuffer2 = btmalloc(size * datasize);
  
  InitDataFileDL(db);
  db->DL = db->DLbuffer1;
  return 0;
}
/** Turn the data logger on
The data logger will not record data until after DLon() is called
*/
void DLon(btlogger *db)
{
  db->Log_Data = 1;
}
/** Turn the data logger off
When the datalogger is turned off with DLoff(), data logging il paused until
logging is turned back on by DLon()
*/
void DLoff(btlogger *db)
{
  db->Log_Data = 0;
}
/** Close the logging file and free buffer memory
 
*/
void CloseDL(btlogger *db)
{
  flushDL(db);
  free(db->data);
  free(db->DLbuffer1);
  free(db->DLbuffer2);
  fclose(db->DLfile);
}

/** Copy all the data pointed to by the btdata_info array into the present buffer
 
 This function should be called every time you want to save a record to the buffer.
 Typically this is called in a high priority thread with a short loop period.
 
*/
void TriggerDL(btlogger *db)
{
  int tmp,cnt;
  void *start,*run;
  if (db->Log_Data) //If data logging is turned on
  {
    start = db->DL + db->DLidx * db->data_size; //point to the present location in the buffer

    for(cnt = 0;cnt < db->fields; cnt++)
    {
      memcpy(start,db->data[cnt].data,db->data[cnt].size); //copy user data to buffer
      start += db->data[cnt].size;
    }
    UpdateDL(db);
  }
}

/** (internal)Writes the header of a binary data file.
*/
void InitDataFileDL(btlogger *db)
{
  FILE *datfile;
  int tmp,cnt;
  long len;

  datfile = db->DLfile;

  fwrite(&(db->fields),sizeof(int),1,datfile);   //6 fields

  for(cnt = 0;cnt < db->fields; cnt++)
  {
    fwrite(&(db->data[cnt].type),sizeof(int),1,datfile);
    fwrite(&(db->data[cnt].size),sizeof(int),1,datfile); //block size = arraysize + sizeof(variable)
    fwrite(db->data[cnt].name,sizeof(char),50,datfile);
  }
}

/** (internal)UpdateDL increments the DL index and switches the pointer to the other buffer if necessary
*/
void UpdateDL(btlogger *db)
{
  db->DLidx++;                      //Increment current array index

  if (db->DLidx >= db->buffersize)            //If our index exceeds our array size
  {
    db->DLidx = 0;                  //reset our index to zero

    if (db->DL == db->DLbuffer1)        //If we are currently pointed to buffer 1
    {
      db->DL = db->DLbuffer2;           //Point to buffer 2
      db->DLwritten = 1;            //Indicate that buffer 1 is full
    }
    else                        //else
    {
      db->DL = db->DLbuffer1;           //Point to buffer 1
      db->DLwritten = 2;            //Indicate that buffer 2 is full
    }
  }
}

/** (internal)DLwrite checks to see if one of the buffers got filled and if so, writes it to a file
*/
void DLwrite(btlogger *db)
{
  void  *DLout;
  int Ridx;
  long Rlength;

  if (db->Log_Data) //If data logging is turned on
  {
    if (db->DLwritten)
    {               //If any buffer is full
      if (db->Log_File_Open)
      { //If our data logging files are open and everything is peachy

        if (db->DLwritten == 1)       //If buffer1 is full
          DLout = db->DLbuffer1;      //Set DLout to point the same place as DLbuffer1
        else if (db->DLwritten == 2)  //Else If buffer2 is full
          DLout = db->DLbuffer2;      //Set DLout to point the same place as DLbuffer1

        Ridx = db->DLctr;                           //Record index = full buffer counter
        fwrite(&Ridx,sizeof(int),1,db->DLfile);     //Write Record index as a binary integer
        Rlength = db->buffersize*db->data_size;         //Calculate the Record length
        fwrite(&Rlength,sizeof(long),1,db->DLfile);     //Write the Record length in bytes

        fwrite(DLout,db->data_size,db->buffersize,db->DLfile);  //Write all the data in binary form
        db->DLctr++;                                        //increment the record index
      }
      db->DLwritten=0;                                      //Reset full buffer indicator to zero
    }
  }
}
/** (internal use) Write what is remaining in our buffers before closing */
void flushDL(btlogger *db)
{
  void  *DLout;
  int Ridx;
  long Rlength;


  if (db->Log_File_Open)
  { //If our data logging files are open and everything is peachy

    DLout = db->DL;
    Ridx = db->DLctr;                           //Record index = full buffer counter
    fwrite(&Ridx,sizeof(int),1,db->DLfile);     //Write Record index as a binary integer
    Rlength = db->DLidx*db->data_size;         //Calculate the Record length
    fwrite(&Rlength,sizeof(long),1,db->DLfile);     //Write the Record length in bytes

    fwrite(DLout,db->data_size,db->DLidx,db->DLfile);  //Write all the data in binary form
    db->DLctr++;                                        //increment the record index
  }


}

/** Checks the buffers and writes them if necessary
*/
void evalDL(btlogger *db)
{
  if (db->DLwritten)
    DLwrite(db);
}
/*************************** binary file to text file converter ***********/

int DecodeDL(char *infile, char *outfile, int header)
{
  btlogger db; //use this just because it is convinient
  int numfields=0,raysize;
  FILE *inf,*outf;
  int fieldcnt;
  int current_index;
  long length;
  void *data,*dataidx;
  long cnt,idx,ridx;
  int array_len;

  int *intdata;
  double *doubledata;
  long *longdata;
  long long *exlongdata;

  syslog(LOG_ERR,"DecodeDL: Starting logfile decode from %s -> %s",infile,outfile);

  //open input file
  if ((inf = fopen(infile,"rb"))==NULL)
  {
    syslog(LOG_ERR,"DecodeDL:Unable to open input file: %s",infile);
    return -1;
  }

  //open output file
  if ((outf = fopen(outfile,"w"))==NULL)
  {
    syslog(LOG_ERR,"DecodeDL:Unable to open output file: %s\n",outfile);
    return -2;
  }

  //figure out fields
  //print fields
  fread(&fieldcnt,sizeof(int),1,inf);
  syslog(LOG_ERR,"DecodeDL:Fields %d",fieldcnt);
  PrepDL(&db,fieldcnt); //allocate memory for our field info

  // Read header info, write text header
  for (cnt = 0;cnt < fieldcnt;cnt++)
  {
    fread(&(db.data[cnt].type),sizeof(int),1,inf);
    fread(&(db.data[cnt].size),sizeof(int),1,inf);
    fread(db.data[cnt].name,sizeof(char),50,inf);

    switch (db.data[cnt].type)
    {
    case 0://integer
      array_len = db.data[cnt].size / sizeof(int);
      break;
    case 1://long
      array_len = db.data[cnt].size / sizeof(long);
      break;
    case 2://double
      array_len = db.data[cnt].size / sizeof(double);
      break;
    case 3://long long
      array_len = db.data[cnt].size / sizeof(long long);
      break;
    case 4://btreal
      array_len = db.data[cnt].size / sizeof(btreal);
      break;
    }
    if (header) {
    if (array_len > 1)
    {
      for (ridx = 0; ridx < array_len; ridx++)
      {
        fprintf(outf,"%s[%d]",db.data[cnt].name,ridx);
        if ((ridx < array_len - 1) || (cnt < fieldcnt - 1))
          fprintf(outf,",");
      }
    }
    else
    {
      fprintf(outf,"%s",db.data[cnt].name);
      if (cnt < fieldcnt - 1)
        fprintf(outf,",");
    }
    }
    syslog(LOG_ERR,"DecodeDL:Field %d - type: %d size: %d, name: %s",cnt,db.data[cnt].type,db.data[cnt].size,db.data[cnt].name);
  }
  if (header) fprintf(outf,"\n");

  while(!feof(inf))
  {
    if(fread(&current_index,sizeof(int),1,inf)==0)
      break;
    fread(&length,sizeof(long),1,inf);
    syslog(LOG_ERR,"DecodeDL:record index: %d length: %ld",current_index,length);
    data = btmalloc(length);
    

    fread(data,sizeof(char),length,inf);

    dataidx = data;
    cnt = 0;
    while(cnt<length)
    {

      for (idx = 0;idx < fieldcnt;idx++)
      {
        switch (db.data[idx].type)
        {
        case 0://integer
          array_len = db.data[idx].size / sizeof(int);
          if ((db.data[idx].size % sizeof(int)) != 0)
            syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
          for (ridx = 0; ridx < array_len; ridx++)
          {
            intdata = (int *)dataidx;
            fprintf(outf," %d ",*intdata);
            if (ridx < array_len - 1)
              fprintf(outf,",");
            dataidx +=  sizeof(int);
          }
          cnt+=db.data[idx].size;
          break;
        case 1://long
          array_len = db.data[idx].size / sizeof(long);
          if ((db.data[idx].size % sizeof(long)) != 0)
            syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
          for (ridx = 0; ridx < array_len; ridx++)
          {
            longdata = (long *)dataidx;
            fprintf(outf," %ld ",*longdata);
            if (ridx < array_len - 1)
              fprintf(outf,",");
            dataidx +=  sizeof(long);
          }
          cnt+=db.data[idx].size;
          break;
        case 2://double
          array_len = db.data[idx].size / sizeof(double);
          if ((db.data[idx].size % sizeof(double)) != 0)
            syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
          for (ridx = 0; ridx < array_len; ridx++)
          {
            doubledata = (double *)dataidx;
            fprintf(outf," %f ",*doubledata);
            if (ridx < array_len - 1)
              fprintf(outf,",");
            dataidx +=  sizeof(double);
          }
          cnt+=db.data[idx].size;
          break;
        case 3://long long
          array_len = db.data[idx].size / sizeof(long long);
          if ((db.data[idx].size % sizeof(long long)) != 0)
            syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
          for (ridx = 0; ridx < array_len; ridx++)
          {
            exlongdata = (long long *)dataidx;
            fprintf(outf," %lld ",*exlongdata);
            if (ridx < array_len - 1)
              fprintf(outf,",");
            dataidx +=  sizeof(long long);
          }
          cnt+=db.data[idx].size;
          break;
        case 4://btreal
          array_len = db.data[idx].size / sizeof(btreal);
          if ((db.data[idx].size % sizeof(btreal)) != 0)
            syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
          for (ridx = 0; ridx < array_len; ridx++)
          {
            doubledata = (btreal *)dataidx;
            fprintf(outf," %f ",*doubledata);
            if (ridx < array_len - 1)
              fprintf(outf,",");
            dataidx +=  sizeof(btreal);
          }
          cnt+=db.data[idx].size;
          break;
        }
        if (idx < fieldcnt-1)
          fprintf(outf,",");

      }
      fprintf(outf,"\n");
    }

    free(data);
  }
  free(db.data);
  fclose(inf);
  fclose(outf);
}
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
