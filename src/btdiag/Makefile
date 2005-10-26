OBJS = btdiag.o
TARG = btdiag
CC = gcc 
PATH_INCLUDE = -I/usr/realtime/include -I../../include


#set BTOLDCFG to -DBTOLDCONFIG for RIC,MIT,Ualberta,BPW
#ex: #export BTOLDCFG=-DBTOLDCONFIG
CFLAGS =  -g -DUSE_RTAI31  ${PATH_INCLUDE} ${BTOLDCFG}
LDFLAGS = -L/usr/realtime/lib -L/usr/lib -L/usr/local/lib -L../../lib -lpthread -llxrt \
          -lncurses -lm -lbtwam -lbtsystem -lntcan


default: btdiag

btdiag: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}
  

clean: 
	rm *.o ${TARG} 

system: 	
	$(MAKE) -C ../btsystem

wam:
	$(MAKE) -C ../btwam

all: system wam btdiag


