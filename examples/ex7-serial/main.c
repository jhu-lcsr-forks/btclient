#include <stdio.h>
#include "btserial.h"

PORT p;

int main(int argc, void **argv)
{
   char input[255];
   char command[20];
   int err, len;

   if(err = serialOpen(&p, "/dev/ttyS0")) {
      printf("Error opening port: %d", err);
      exit(0);
   }
   serialSetBaud(&p,9600);

   /* Write a string to the port, followed by a carriage return */
   serialWriteString(&p, "HI\r");
   while(1) {
      /* Read chars from the port into the input buffer until
        the termination character '>' is received or 30 seconds has
        elapsed, whichever comes first.
       */
      serialReadLine(&p, input, &len, '>', 30000);

      serialWriteString(&p, "GC\r");
      serialReadLine(&p, input, &len, '>', 30000);
      serialWriteString(&p, "GO\r");
   }

   serialClose(&p);
   exit(0);
}
