OBJS = main.o
TARG = ex3
CC = gcc 
PATH_INCLUDE = -I/usr/realtime/include -I../../include

CFLAGS =  -g -DUSE_RTAI31 -static ${PATH_INCLUDE}
#CFLAGS =  -g -DUSE_RTAI31 -static ${PATH_INCLUDE} -DBTOLDCONFIG

LDFLAGS = -L/usr/realtime/lib -L/usr/lib -L../../lib -lpthread -llxrt \
          -lncurses -lm -lbtwam -lbtsystem -lntcan


default: ex3

ex3: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}

clean: 
	rm *.o ${TARG} 



