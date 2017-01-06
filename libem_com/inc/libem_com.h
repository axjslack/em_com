#ifndef _LIBEM_COM_H
#define _LIBEM_COM_H

#include <stdint.h>
#include <stdio.h>

// GPIOCONTROL define

#define PIN_SEL 8
#define PIN_SET_DIM 32
#define MAX_PIN 255
#define PATH_BUF 128
#define LED_TIME 5
#define GPIO_KEY 44
#define GPIO_LED 45

// PINMESG define

#define CMD_MASK 0x80
#define MAX_OPXMESG 32
#define END_OF_LIST 255

#define NULL_CMD	0x00	//Null command, use TBD
#define SC_CMD 		0x01 	//Normal Set Clear CMD
#define READ_CMD	0x02	//Read pin status
#define RPIN_CMD	0x04	//Read pin mapping
#define SRV_CMD		0x08	//Special server command


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

// COMMON Functions

int select_serial(char *serport);

#endif // _LIBEM_COM_H
