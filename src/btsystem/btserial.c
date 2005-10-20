/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btserial.c
 *  Author .............Brian Zenowich
 *  Creation Date ......Oct 20, 2005
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
 *
 *  REVISION HISTORY:
 *
 *======================================================================*/

/* Serial port routines */
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include "btserial.h"

/** Open serial port */
int serialOpen(PORT *port,char *portlocation)
{ 
    // Open port for reading
//	port->ifd = open(port, 0_RDRW | 0_NOCTTY | 0_NONBLOCK);
    if ( ( port->isp=fopen(portlocation,"r") ) != NULL ) 
    { 
        port->ifd=fileno(port->isp); 
    }else
    {
        //printf("\nUnable to open port %s for input!\n", portlocation); 
        return(1); 
    }
    
    // Open port for writing
    if ( ( port->osp=fopen(portlocation,"w") ) != NULL ) 
    {
        port->ofd=fileno(port->osp); 
    }else
    {
        //printf("\nUnable to open port %s for output!\n", portlocation);
        return(2); 
    }
    
    // Set port to no delay on read
    fcntl(port->ifd, F_SETFL, FNDELAY);
    
    return 0; 
}

/** Close serial port */
int serialClose(PORT *port)
{
    int err=0;
    
    close(port->ofd);
    close(port->ifd);
    
    return err; 
}

/** Read data from the serial port */
int serialRead(PORT *port, char *buf, int bytesToRead, int *bytesRead)
{
    int err=0;
    
    *bytesRead = read(port->ifd, buf, bytesToRead);
    
    if(*bytesRead < 0) 
    {
        *bytesRead = 0;
        return 1;
    }
    
    buf[*bytesRead] = '\0'; // Null terminate incoming string
    
    return err;
}

int serialReadLine(PORT *port, char *buf, int *lineLen, int term, long ms)
{
    int bytesRead;
    int err;
    
    *lineLen = 0;
    //printf("term:%d\n",term);
    while(1)
    {
        err = serialRead(port, buf, 1, &bytesRead);
        //printf("bytesRead:%d\n", bytesRead);
        *lineLen += bytesRead;
        buf[bytesRead] = '\0'; // Null terminate
        //printf("serialRead:%s\n", buf);
        if(*buf == term) // If termination character is found
            return(0);
        //printf(".");    
        buf += bytesRead;
        usleep(20000); // Sleep for 20ms
        ms -= 20;
        if(ms < 0)
            return(1);
    }
}

/** Write data to the serial port */
int serialWrite(PORT *port, char *buf, int bytesToWrite)
{
    int err=0;
    
    err = write(port->ofd, buf, bytesToWrite);
    if(err >= 0)
        err = 0;
        
    return err;
}

int serialWriteString(PORT *port, char *buf)
{
    //printf("serialWriteString:%s\n", buf);
    return( serialWrite(port, buf, strlen(buf)) );
}

/** Look for data present in the serial port */
int serialLook(PORT *port, int *bytesPresent)
{
    int err=0;

    ioctl(port->ifd, FIONREAD, bytesPresent);
    
    return err;    
}

int serialSetBaud(PORT *port, long baud)
{
    struct termios options;

    /*
     * Get the current options for the port...
     */

    tcgetattr(port->ofd, &options);

    /*
     * Set the baud rates...
     */
    switch(baud)
    {
        case 1200:
            cfsetispeed(&options, B1200);
            cfsetospeed(&options, B1200);        
        break;
        case 2400:
            cfsetispeed(&options, B2400);
            cfsetospeed(&options, B2400);        
        break;
        case 4800:
            cfsetispeed(&options, B4800);
            cfsetospeed(&options, B4800);        
        break;
        case 9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);        
        break;
        case 19200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
        break;
        case 38400:
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
        break;
        case 57600:
            cfsetispeed(&options, B57600);
            cfsetospeed(&options, B57600);
        break;
        case 115200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
        break;
        default:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
        break;
    }
    /* * Enable the receiver and set local mode...  */ 
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_lflag = 0; 
    
    /* * Set the new options for the port...  */ 
    tcsetattr(port->ifd, TCSANOW, &options); 
    tcsetattr(port->ofd, TCSANOW, &options); return 0; 
}
