

#include "libem_com.h"

void null_function()
{
	//Here only for debug purpose
}


void gpio_decode_mesg(selector *sel, pinmsg *mesg)
{
	uint8_t set_oplist_static[MAX_OPXMESG];
	uint8_t clear_oplist_static[MAX_OPXMESG];
	
	gen_oplist_static(mesg->sbit, set_oplist_static);
	error_print("Printing the set oplist\n");
	read_oplist_static(set_oplist_static);
	gen_oplist_static(mesg->cbit, clear_oplist_static);
	error_print("Printing the clear oplist\n");
	read_oplist_static(clear_oplist_static);
	
	error_print("Running clear bit\n");
	set_pin_cycle(sel, clear_oplist_static, OFF);
	error_print("Running set bit\n");
	set_pin_cycle(sel, set_oplist_static, ON);
	
}


int decode_message(std_msg_t recv)
{
	uint8_t ccom;
	
	
	ccom= recv.command && CMD_MASK;
	switch(ccom) {
		case BASIC_CMD : null_function(); break;
		case GPIO_CMD : gpio_decode_mesg(&sel, (pinmsg*)&recv); break;
		case SERIAL_CMD : null_function(); break;
		case I2C_CMD : null_function(); break;
		case SPI_CMD : null_function(); break;
		case OTHER_CMD : null_function(); break;
		default : null_function();
	}
	return 0;

}


	
