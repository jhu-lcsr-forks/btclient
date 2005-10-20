/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btserial.h
 *  Author .............Brian Zenowich
 *  Creation Date ......20 Oct 2005
 *  Addtl Authors ......
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
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
 *                                                                      *
 *======================================================================*/

/* Serial port routines */
  
typedef struct { 
    FILE   *isp, *osp;                   /*stream pointers to the serial port*/ 
    int     ifd, ofd;                   /*file descriptors for the stream pointers*/ 
    //char    obuf[BUFLEN], ibuf[BUFLEN]; /*buffers for handling the streams*/ 
} PORT;

/** Open serial port */
int serialOpen(PORT *port,char *portlocation);

/** Close serial port */
int serialClose(PORT *port);

/** Read data from the serial port */
int serialRead(PORT *port, char *buf, int bytesToRead, int *bytesRead);

/** Read data from serial port until termination char or timeout */
int serialReadLine(PORT *port, char *buf, int *lineLen, int term, long ms);

/** Write data to the serial port */
int serialWrite(PORT *port, char *buf, int bytesToWrite);

/** Write a string to the serial port */
int serialWriteString(PORT *port, char *buf);

/** Look for data present in the serial port */
int serialLook(PORT *port, int *bytesPresent);

/** Set the baud rate */
int serialSetBaud(PORT *port, long baud);

