LIB_OBJS =  btsystem.o  btjointcontrol.o playlist.o \
            btlogger.o btcan_esd.o btrobot.o btmath.o \
            btos.o btpath.o btcontrol.o btserial.o bthaptics.o btstatecontrol.o \
            btgeometry.o btparser.o btseg.o
INCL_FILES = btcan.h btsystem.h  btlogger.h btjointcontrol.h \
            btmath.h btrobot.h btos.h btpath.h btcontrol.h btserial.h bthaptics.h \
            btstatecontrol.h playlist.h btgeometry.h btparser.h btseg.h
LIB_TARG = ./libbtsystem.a
SLIB_TARG = ./libbtsystem.so
CC = gcc
PATH_INCLUDE = -I./include -I/usr/realtime/include
PATH_INSTALL = ../..

#set BTOLDCFG to -DBTOLDCONFIG for RIC,MIT,Ualberta,BPW
#ex: #export BTOLDCFG=-DBTOLDCONFIG
CFLAGS += -DUSE_RTAI31  ${PATH_INCLUDE} ${BTOLDCFG}
#Debugging - Comment out line below for more speed
CFLAGS += -g -DBT_NULL_PTR_GUARD -DBT_ARRAY_BOUNDS_CHECK -DBTDEBUG  

LDFLAGS += -L/usr/realtime/lib -L/usr/lib -L./lib -lm -lpthread -llxrt -lntcan

$(LIB_OBJS) : $(INCL_FILES)

default: lib

clean: 
	rm *.o ${LIB_TARG} 

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
#-lm -Llib -lcan -lmitop
