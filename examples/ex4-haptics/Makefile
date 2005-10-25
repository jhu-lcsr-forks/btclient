OBJS = main.o
TARG = ex4
CC = gcc 
PATH_INCLUDE = -I/usr/realtime/include -I../../include

#set BTOLDCFG to -DBTOLDCONFIG for RIC,MIT,Ualberta,BPW
#ex: #export BTOLDCFG=-DBTOLDCONFIG
CFLAGS =  -g -DUSE_RTAI31  ${PATH_INCLUDE} ${BTOLDCFG}
#Replace -lrtai with -llxrt
LDFLAGS = -L/usr/realtime/lib -L/usr/lib -L../../lib -lpthread -llxrt \
          -lncurses -lm -lbtwam -lbtsystem -lntcan


default: ex1

ex1: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}

clean: 
	rm *.o ${TARG} 



