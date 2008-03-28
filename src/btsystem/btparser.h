/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btparser.h
 *  Author .............Brian Zenowich
 *  Creation Date ......15 Feb 2005
 *  Addtl Authors ......Traveler Hauptman
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>        *
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  051031:TH:Finalized for WAM library.
 *                                                                      *
 *======================================================================*/


/** \file btparser.h
    \brief Config file parsing

The functions in btparser.c allow an application to read a structured
    configuration file and extract values from it.

\e myconfigfile 
\verbatim
# Example config file. Comments start with an octothorpe.
system{
    busCount = 1
    bus[]{
        type = "CAN"                # Type of bus
        address = 0                 # Address of robot
    }
    bus[]{
        world = <<1,2,3,3>,<1,2,3,3>,<1,2,3,3>> # World -> Base frame transform
        N[] = 35.87, 28.21, 28.21, 17.77, 10.27, 10.27, 14.93
    }
}
\endverbatim

The values in the above config file could be accessed by:
\code
{
  btparser parse;
  char bus_type[50];
  char buff[100],buff2[20];
  int address,Nvals[15],buscnt,cnt;
  matr_n werld;
  
  werld = new_mn(4,4);
  
  btParseFile(&parse,"myconfigfile");
  btParseGetVal(&parse,INT,"system.busCount",(void*)&buscnt);
  btParseGetVal(&parse,INT,"system.bus[0].address",(void*)&address);
  btParseGetVal(&parse,STRING,"system.bus[0].type",(void*)bus_type);
  for (cnt = 0;cnt < 7;cnt ++){
    sprintf(buff,"system.bus[1].N[%d]",cnt);
    btParseGetVal(&parse,INT,buff,(void*)Nvals);
  }
  btParseGetVal(&parse,INT,"system.bus[1].world",(void*)werld);
}
\endcode

Setting the BTDEBUG_PARSER bit of the BTDEBUG define will dump additional debug
info to syslog.

\todo Matrix Parsing

*/

#ifndef _PARSER_H
#define _PARSER_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

typedef struct {
  char filename[255];
  char tempfile[255];
  char flatfile[255];
}btparser;

#define BTPARSE_INIT {"wam.conf","",""}

int btParseFile(btparser *parse_obj,char *filename);
int btParseGetVal(btparser *parse_obj,int type, char *str, void *loc);
void btParseClose(btparser *parse_obj);

int parseFile(char *fn);
int parseGetVal(int type, char *str, void *loc);
enum parsetypes {INT = 0,LONG,DOUBLE,STRING,VECTOR,MATRIX};

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /* _PARSER_H */

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
 
