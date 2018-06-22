#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>


#include "libem_com.h"


int serial_setup(int fd, struct termios *ourserial)
{

	tcgetattr(fd, ourserial);

	cfsetispeed(ourserial, B38400);
	cfsetospeed(ourserial, B38400);

	ourserial->c_cflag |= (CLOCAL | CREAD);
	// CREAD: enable reciever 
	// CLOCAL: don't change the owner

	//Parity 8n1 (copied)
	ourserial->c_cflag &= ~PARENB;
	ourserial->c_cflag &= ~CSTOPB;
	ourserial->c_cflag &= ~CSIZE;
	ourserial->c_cflag |= CS8;

	//Disable Hardware flow control
	//ourserial->c_cflag &= ~CNEW_RTSCTS;

	//Raw input instead of Canonical input
	ourserial->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Disable software flow control
	ourserial->c_iflag &= ~(IXON | IXOFF | IXANY);

	// Selecting Raw output
	ourserial->c_oflag &= ~OPOST;

	ourserial->c_cc[VMIN]  = 0;
	ourserial->c_cc[VTIME] = 10;

	//Setting serial
	tcsetattr(fd, TCSANOW, ourserial);

	return 0;

}


int send_buffer_serial(int fd, char *msg_to_write, size_t count)
{
	int status;	

	status=write(fd, msg_to_write, count);
	if(status<0)
	{
		fprintf(stderr,"Impossible to write buffer\n");
		return 1;
	}
	return 0; 
}
	

int receive_buffer_serial(int fd, char *answbuf, size_t count)
{
	char *tmpbufptr;
	int status, i=0;	
	
	tmpbufptr=answbuf;
	while (i<count)
	{
		status=read(fd,tmpbufptr, 1);
		//fprintf(stderr,"%c",*tmpbufptr);
		if(status<0)
		{
	
			fprintf(stderr, "%s: %s\n",__func__ , strerror(errno));
			return status;
		}


		i++;
		tmpbufptr++;
	}
	//printf("Result : %s \n", answbuf);
	return 0;

}







