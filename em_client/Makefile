CLIENT_CC=gcc

SRCDIR=src
HEADER=inc

CLIENT=em_client


CFLAGS += -I $(HEADER) -I ../libem_com/$(HEADER) -L . -g -Wall   


CL_SRC = pinclient.c 
CL_OBJS= pinclient.o 


all : $(CLIENT)


$(CL_OBJS) : $(SRCDIR)/$(CL_SRC)
	$(CLIENT_CC) $(CFLAGS) -c $< -o $@


$(CLIENT): $(CL_OBJS) 
	$(CLIENT_CC) $(CFLAGS)  $^ -o $@ 
		

clean: 
	rm -rfv *.o
	rm -rfv $(CLIENT)

