#ifndef __INBUS_INCLUDED__
#define __INBUS_INCLUDED__

#define INBUS_MASTER_ID		0x000
#define INBUS_FRAME_SIZE	3
#define INBUS_DEVICE_ID		0x002

extern char is_param_updated;
extern char	is_transmit_ready;

void inbus_init( void );
void inbus_process( void );

#endif // __INBUS_INCLUDED__

