/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btparser.h
 *  Author .............Brian Zenowich
 *  Creation Date ......15 Feb 2003
 *  Addtl Authors ......
 *                                                                      *
 *  ******************************************************************  */
/** \file btparser.h
    The functions in btparser.c allow an application to read a structured
    configuration file and extract values from it.

\verbatim
# Example config file

...blah

\endverbatim

*/

#ifndef _PARSER_H
#define _PARSER_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

int parseFile(char *fn);
int parseGetVal(int type, char *str, void *loc);
enum{INT,LONG,DOUBLE,STRING,VECTOR};

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /* _PARSER_H */
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
