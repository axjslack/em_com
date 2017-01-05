CC=$(CROSS_COMPILE)gcc 

SRCDIR=src
HEADER=inc

SERVER=emcom_server


LDFLAGS = -lem_com

CFLAGS += -I $(HEADER) -I ../libem_com/$(HEADER) -L ../libem_com -g -Wall   


SR_SRC := $(SRCDIR)/pinserver.c $(SRCDIR)/confparser.c
SR_OBJS := pinserver.o  confparser.o


all : $(SERVER)

$(SR_OBJS) : $(SR_SRC)
	$(CC) $(CFLAGS)  -c $^

$(SERVER): $(SR_OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean: 
	rm -rfv *.o
	rm -rfv  $(SERVER)

