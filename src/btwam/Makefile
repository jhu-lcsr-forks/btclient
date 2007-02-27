LIB_OBJS =  btwam.o gimbals.o 
INCL_FILES = btwam.h gimbals.h
LIB_TARG = ./libbtwam.a
SLIB_TARG = ./libbtwam.so
CC = gcc
PATH_INCLUDE = -I./include -I/usr/realtime/include -I../btsystem
PATH_INSTALL = ../..

#set BTOLDCFG to -DBTOLDCONFIG for RIC,MIT,Ualberta,BPW
#ex: #export BTOLDCFG=-DBTOLDCONFIG
CFLAGS += -g -DUSE_RTAI31  ${PATH_INCLUDE} ${BTOLDCFG}
#CFLAGS += -g -DUSE_RTAI31  ${PATH_INCLUDE} -DBTOLDCONFIG

#Replace -lrtai with -llxrt
LDFLAGS += -L/usr/realtime/lib -L/usr/lib -L/usr/local/lib -L./lib 
LDFLAGS += -lm -lpthread -llxrt -lntcan -lgsl

#Make *.o files rebuild on any change to *.h
$(LIB_OBJS) : $(INCL_FILES)

default: lib

clean: 
	rm *.o ${LIB_TARG} $(PATH_INSTALL)/${LIB_TARG}

small:
	rm *.o *~ *.bak

lib: ${LIB_OBJS}
	$(AR) rcs $(LIB_TARG) ${LIB_OBJS}
	$(RM) $*.o

slib: ${LIB_OBJS}
	${CC} -g ${LDFLAGS} -shared -o ${SLIB_TARG} ${LIB_OBJS}

install:
	cp $(LIB_TARG) $(PATH_INSTALL)/lib
	cp $(INCL_FILES) $(PATH_INSTALL)/include  

