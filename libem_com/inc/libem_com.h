#ifndef _LIBEM_COM_H
#define _LIBEM_COM_H

#include <stdint.h>
#include <stdio.h>
#include <termios.h>

// GPIOCONTROL define

#define PIN_SEL 8
#define PIN_SET_DIM 32
#define MAX_PIN 255
#define PATH_BUF 128
#define LED_TIME 5
#define GPIO_KEY 44
#define GPIO_LED 45

// PINMESG define


#define MAX_OPXMESG 32
#define END_OF_LIST 255

/* Proposal % Bit MSB 3 -7 Type, 0-2 Command
 * 0x00 - 0x07 Basic Command
 * 0x08 - 0x0F GPIO Command
 * 0x10 - 0x17 Serial Command
 * 0x20 - 0x27 I2C Command
 * 0x40 - 0x47 SPI Command
 * 0x80 - 0x87 Other Commands
 */

#define CMD_MASK 		0x80
#define BASIC_CMD 		0x00
#define GPIO_CMD		0x08
#define SERIAL_CMD		0x10
#define I2C_CMD			0x20
#define SPI_CMD			0x40
#define OTHER_CMD		0x80


//Old mode
#define NULL_CMD	0x00	//Null command, use TBD
#define SC_CMD 		0x01 	//Normal Set Clear CMD
#define READ_CMD	0x02	//Read pin status
#define RPIN_CMD	0x03	//Read pin mapping
#define SRV_CMD		0x08	//Special server command

//Define device

#define SPI_1_0 "/dev/spidev1.0"
#define SPI_1_1 "/dev/spidev1.1"
#define SPI_2_0 "/dev/spidev2.0"
#define SPI_2_1 "/dev/spidev2.1"

#define I2C0 "/dev/i2c-0"
#define I2C1 "/dev/i2c-1"
#define I2C2 "/dev/i2c-2"
#define I2C3 "/dev/i2c-3"
#define I2C4 "/dev/i2c-4"


// COMMON define

#define error_print(...) fprintf(stderr, __VA_ARGS__);
#define DBG
#ifdef DBG
#define debug_print(...) fprintf(stderr, __VA_ARGS__);
#else
#define debug_print(...)
#endif
#define DEFAULT_PORT 1337


// GPIOCONTROL data type

typedef enum gpiodirection {
	IN,
	OUT,
	ERR
} gpd;

typedef enum gpiovalue {
	OFF=0,
	ON=1
} gpv;

typedef uint8_t gpn;


typedef enum gpio_availability {
	OK,
	KO
} gpio_av; 


typedef struct single_pin {
	uint8_t gpion;
	gpd direction;
	gpio_av available;
} pin;

typedef struct pin_selector {
	pin pin_n[PIN_SET_DIM];
} selector;

#ifndef MAIN
extern selector sel; 
#endif

typedef struct direct_pin {
		gpn gpion;
		gpd gpdir;
		gpv gpiov;	
} dpin;



// PINMESG data type

typedef struct pinmessage {
	uint8_t slt; //selector
	uint32_t sbit; //Set bit
	uint32_t cbit; //clear bit
} pinmsg;


//MESG Data type

struct std_message_s {
	uint8_t command;
	uint64_t mesgdata;
};

typedef struct std_message_s std_msg_t; 



// COMMON data type

typedef struct serial_port {
	int ttyn;
	int bdrate;
	char mode[4];
} serial_port_t;

typedef enum eConn_type {
	serial,
	network,
	undef
} Conn_type_t; 

typedef enum eIP_type {
	ipv4,
	ipv6,
	unspec
} IP_type_t;


// SPI Data types

enum spimode_e {
	MODE0=0,
	MODE1=1,
	MODE2=2,
	MODE3=3

};

enum bitword_e {
	BIT8=0,
	BIT16=1,
	BIT24=2,
	BIT32=3
};

typedef enum bitword_e bitword_t;
typedef enum spimode_e spimode_t;



//This would require --short-enums in compile options
// Bitfileds are quite unsafe, and have to be tested on multiple arch.

struct spihead_s {
	spimode_t spim :2;
	bitword_t bitw :2;
	uint8_t delay_ms :4;
};

typedef struct spihead_s spihead_t;

struct __attribute__((__packed__)) spimessage_s {
	uint8_t slt;
	uint32_t spifrhz :24;
	spihead_t head;
	uint32_t spidata;

};

typedef struct spimessage_s spimsg_t;


//New SPI message (8 and 16 bit)

typedef struct spi_mesg16_str{
	uint16_t *rawmsg;
	uint16_t *rawansw;
	uint16_t size;
	uint16_t answ_size;
} spi_mesg16_t;

typedef struct spi_mesg8_str{
	uint8_t *rawmsg;
	uint8_t *rawansw;
	uint16_t size;
	uint16_t answ_size;
} spi_mesg8_t;




// End of SPI Data types 



// GPIOCONTROL Functions


int reserve_gpio(gpn n);
int set_direction(gpn n, gpd d);
gpv read_gpio(gpn n);
int write_gpio(gpn n, gpv v);
int poll_gpio(gpn n);
int led_pulse(gpn n);

uint8_t init_localpin(selector *localpin);
uint8_t multiled_kr_7p(selector *localpin);
void set_pin_cycle(selector *localpin, uint8_t *oplist_static, gpv op);
void setdirection_localpin(selector *localpin, gpd direction);
void init_confpin(selector *confpin, int pos);
void setdirection_confpin(selector *confpin, int pos);


// PINMESG Functions

void gen_oplist_static(uint32_t pinconf, uint8_t *oplist_static);
void read_oplist_static(uint8_t *oplist_static);
void decode_mesg(selector *sel, pinmsg *mesg);
void print_message(pinmsg *to_read);


//MESG Functions 


void null_function();
void gpio_decode_mesg(selector *sel, pinmsg *mesg);
int decode_message(std_msg_t recv);

// COMMON Functions

// Serial Functions
int serial_setup(int fd, struct termios *ourserial);
int send_buffer_serial(int fd, char *msg_to_write, size_t count);
int receive_buffer_serial(int fd, char *answbuf, size_t count);




// SPI Functions
int SPIreadwrite(char *name, uint32_t spifreq, uint8_t spimode, uint8_t bitflag, void *mesg, uint32_t udelay);


// I2C Functions

int i2c_write_byte(char *i2cdev, uint16_t slave_address, uint8_t data);
int i2c_read_byte(char *i2cdev, uint16_t slave_address, uint8_t *data);
int i2c_write_register(char *i2cdev, uint16_t address, uint8_t reg_address, uint8_t value);
int i2c_read_register(char *i2cdev, uint16_t address, uint8_t reg_address, uint8_t *data);
int i2c_write(char *i2cdev, uint16_t slave_address, const uint8_t *buffer, uint32_t count);
int i2c_read(char *i2cdev, uint16_t slave_address, uint8_t *buffer, uint32_t count);


#endif // _LIBEM_COM_H
