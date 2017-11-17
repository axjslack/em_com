/*
 *
 * In this step I will consider only "atomic" SPI transaction. But this section is really improvable
 *
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "libem_com.h"

#define SPI_SINGLE_8 8
#define SPI_SINGLE_16 16

/* Ok, is not fair to setup a delay, but I'm having problem with mutliple transmissions*/
#define CS_DELAY 150


//delay, n.bit word


int spi_single_send(char *spidev, uint32_t spifreq, uint64_t *tx, uint8_t bitword, uint8_t spimode, uint16_t delay)
{
	int fd;
	struct spi_ioc_transfer xfer;
	int status;
	
	xfer.tx_buf = (unsigned long)tx;
	xfer.cs_change = 0;
	xfer.len = 1;
	xfer.bits_per_word=bitword;
	xfer.speed_hz = spifreq;
	xfer.delay_usecs=CS_DELAY;
	
	fd = open(spidev, O_RDWR);
	status=ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bitword);
	if(status < 0 )
	{
		printf("Unable to set %d WR bits per word\n", bitword);
	}
	
	status = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spifreq);
	
	ioctl(fd, SPI_IOC_WR_MODE, &spimode);
	if(status < 0 )
	{
		printf("Unable to set RD mode %d\n" , spimode);
	}
	
	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) 
	{
		perror("SPI_IOC_MESSAGE");
		return -34;
	}
	
	return 0;
}


int spi_simple_send_receive(char *spidev, uint32_t spifreq, uint64_t *tx, uint64_t *rx)

{



	int fd;
	struct spi_ioc_transfer xfer[2];
	int status;
	//uint8_t spimode;
	//spimode=3;


	debug_print("Selected Device %s : Speed is %u Hz\n",spidev, spifreq);

	
	fd = open(spidev, O_RDWR);
	if (fd < 0) 
	{
		error_print("Unable to open %s : please, check if the device exist and you have the permission to use it\n", spidev);
		return 1;
	}

	
	memset(xfer, 0, sizeof (xfer));
	memset(rx, 0, sizeof(*rx));


	xfer[0].tx_buf = (unsigned long)tx;
	xfer[0].cs_change = 0;
	xfer[0].len = 1;
	xfer[0].bits_per_word=SPI_SINGLE_8;
	xfer[0].speed_hz = spifreq;
	xfer[0].delay_usecs=CS_DELAY;

	xfer[1].rx_buf = (unsigned long)rx;
	xfer[1].cs_change = 0;
	xfer[1].len = 1;
	xfer[1].bits_per_word=SPI_SINGLE_8;
	xfer[1].speed_hz = spifreq;
	xfer[1].delay_usecs=CS_DELAY;


	// Integration of spimode is work in progress
	//ioctl(fd, SPI_IOC_WR_MODE, &spimode);
	//ioctl(fd, SPI_IOC_RD_MODE, &spimode);
	

	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) 
	{
		error_print("SPI_IOC_MESSAGE");
		return -1;
	}
	
	


	return status;
}



