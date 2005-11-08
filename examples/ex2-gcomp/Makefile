OBJS = main.o
TARG = ex2
CC = gcc 
PATH_INCLUDE = -I/usr/realtime/include -I../../include

#Replace -DUSE_FUSION with -DUSE_RTAI31
CFLAGS =  -g -DUSE_RTAI31 -static ${PATH_INCLUDE} ${BTOLDCFG}
#Replace -lrtai with -llxrt
LDFLAGS = -L/usr/realtime/lib -L/usr/lib -L../../lib -lpthread -llxrt \
          -lncurses -lm -lbtwam -lbtsystem -lntcan


default: ex2

ex2: ${OBJS} 
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}

clean: 
	rm *.o ${TARG} 



