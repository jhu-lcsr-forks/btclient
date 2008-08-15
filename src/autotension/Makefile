### Makefile for Barrett Software (examples)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

OBJS = main.o
TARG = autotension

# Done with defines, now for the real work
default: autotension

autotension: ${OBJS} btTension.o
	$(CC) ${CFLAGS} -o ${TARG} ${OBJS} btTension.o ${LDFLAGS}

btTension.o: btTension.c btTension.h
	$(CC)  ${CFLAGS} -c btTension.c 

clean:
	rm *.o

system:
	$(MAKE) -C ../btsystem lib
	$(MAKE) -C ../btsystem install

wam:
	$(MAKE) -C ../btwam lib
	$(MAKE) -C ../btwam install
   
all: system wam autotension

