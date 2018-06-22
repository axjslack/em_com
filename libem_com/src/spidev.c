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



/* Ok, is not fair to setup a delay, but I'm having problem with mutliple transmissions*/

/*
* char *name = "/dev/spidev1.1";
*/
/* 
 * At this moment, I still have problem with messages with 0-lenght answer
 * You cauld set udelay=0, but not safely

*/



int SPIreadwrite(char *name, uint32_t spifreq, uint8_t spimode, uint8_t bitflag, void *mesg, uint32_t udelay) 
{

//	char *name;
	int fd;

	struct spi_ioc_transfer xfer[2];

	int status;

	spi_mesg8_t *p8=NULL;	
	spi_mesg16_t *p16=NULL;

	debug_print("Selected %s SPI device\n", name);
	debug_print("Selected Speed is %u Hz\n", spifreq);
	debug_print("Selected Mode is %d :\n", spimode );
	debug_print("Selected %d bit package\n",bitflag);
	debug_print("Selected %u udelay \n",udelay);

	switch(bitflag)
	{
	case 8: { p8=(spi_mesg8_t*)mesg; } break;
	case 16: { p16=(spi_mesg16_t*)mesg; } break;
	default: 
		error_print("%d bit mode is not supported\n", bitflag); break;
	}

	fd = open(name, O_RDWR);
	if (fd < 0) 
	{
		error_print("Failed to open spidev\n");
		return 1;
	}

	memset(xfer, 0, sizeof xfer);

	if(bitflag == 8)
	{
		xfer[0].tx_buf = (unsigned long)p8->rawmsg;
		xfer[0].len = p8->size;
		xfer[1].rx_buf = (unsigned long)p8->rawansw;
		xfer[1].len = p8->answ_size;
	}
	if(bitflag == 16)
	{
		xfer[0].tx_buf = (unsigned long)p16->rawmsg;
		xfer[0].len = p16->size*2;
		xfer[1].rx_buf = (unsigned long)p16->rawansw;
		xfer[1].len = p16->answ_size*2;	
	}
	xfer[0].cs_change = 1;	
	xfer[0].bits_per_word=bitflag;
	xfer[0].delay_usecs=udelay;
	xfer[0].speed_hz = spifreq;
	
	xfer[1].cs_change = 1;
	xfer[1].bits_per_word=bitflag;
	xfer[1].delay_usecs=udelay;
	xfer[1].speed_hz = spifreq;


	status=ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bitflag);
	if(status < 0 )
	{
		error_print("Unable to set %d WR bits per word\n",bitflag );
	}
	
	status = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spifreq);
	
	status = ioctl(fd, SPI_IOC_WR_MODE, &spimode);
	if(status < 0 )
	{
		error_print("Unable to set RD mode %d\n" , spimode);
	}

	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) 
	{
		error_print("SPI_IOC_MESSAGE\n");
		return 32;
	}

	close(fd);

	return 0;
}





