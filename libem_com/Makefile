CC=$(CROSS_COMPILE)gcc

SRCDIR=src
HEADER=inc

LIB=em_com

LIBFLAGS= -fPIC -shared -Wl,-soname,lib$(LIB).so

CFLAGS += -I $(HEADER) -L . -g -Wall   


LIB_SRC := $(SRCDIR)/gpiocontrol.c \
$(SRCDIR)/pinmesg.c \
$(SRCDIR)/serial.c \
$(SRCDIR)/spidev.c \
$(SRCDIR)/i2c.c \
$(SRCDIR)/rs232.c \
$(SRCDIR)/mesgmgt.c
#$(SRCDIR)/confparser.c \

LIB_OBJ := gpiocontrol.o confparser.o pinmesg.o serial.o spidev.o i2c.o rs232.o mesgmgt.o 

all : $(LIB)

$(LIB) : $(LIB_SRC)
	$(CC) $(CFLAGS) $(LIBFLAGS) $^ -o lib$@.so
	ln -s lib$(LIB).so lib$(LIB).so.1



clean: 
	rm -rfv *.o
	rm -rfv lib$(LIB).*

