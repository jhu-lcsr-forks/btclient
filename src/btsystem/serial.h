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