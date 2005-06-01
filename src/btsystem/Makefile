LIB_OBJS =  btsystem.o control_loop.o btjointcontrol.o playlist.o \
            btsystem_control_loop.o btlogger.o btcan_esd.o btrobot.o btmath.o \
            btos.o btpath.o btcontrol.o serial.o bthaptics.o btcontrol_virt.o \
            btgeometry.o
INCL_FILES = btcan.h btsystem.h control_loop.h btlogger.h btjointcontrol.h \
            btmath.h btrobot.h btos.h btpath.h btcontrol.h serial.h bthaptics.h \
            btcontrol_virt.h playlist.h btgeometry.h
LIB_TARG = ./libbtsystem.a
SLIB_TARG = ./libbtsystem.so
CC = gcc
PATH_INCLUDE = -I./include -I/usr/realtime/include
PATH_INSTALL = ../..
#Replace -DUSE_FUSION with -DUSE_RTAI31
CFLAGS +=  -g -DUSE_RTAI31 -static ${PATH_INCLUDE} 
#CFLAGS +=  -DVECT_N_DEBUG -DVECT_N_BOUNDS
#Replace -lrtai with -llxrt
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
