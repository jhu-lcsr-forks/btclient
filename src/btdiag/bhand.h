/* bhand.h */
#ifndef _BHAND_H
#define _BHAND_H

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/

/*==============================*
 * PUBLIC DEFINED constants     *
 *==============================*/
#define MAX_CMD_LEN (32)

/*==============================*
 * PUBLIC MACRO definitions     *
 *==============================*/

/*==============================*
 * PUBLIC typedefs and structs  *
 *==============================*/
typedef struct
{
    char *ptr;
    char command[MAX_CMD_LEN];
}
COMMAND;

/*==============================*
 * PUBLIC Function Prototypes   *
 *==============================*/
void DemoThread(void);
void parseInput(COMMAND *c);
void runBatch(char *filename);

#endif
