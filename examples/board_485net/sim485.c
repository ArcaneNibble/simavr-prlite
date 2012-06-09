#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>

#include "sim_avr.h"
#include "avr_eeprom.h"
#include "avr_uart.h"
#include "avr_ioport.h"
#include "sim_gdb.h"

avr_t * avr = NULL;
char *flfile, *eefile;
int outfifo, infifo;
pthread_mutex_t auxfifomutex = PTHREAD_MUTEX_INITIALIZER;

#define RED		"\x1b[31m"
#define GREEN	"\x1b[32m"
#define BLUE	"\x1b[34m"
#define RESET	"\x1b[0m"

#define EESIZE	0x400

void avr_special_init(avr_t *avr)
{
	FILE *f;
	size_t readbytes;
	unsigned char *eebuf;
	
	f = fopen(flfile, "rb");
	if(f == NULL)
	{
		printf(RED "Could not open flash file!\n");
		exit(1);
	}
	
	readbytes = fread(avr->flash, 1, avr->flashend+1, f);
	if(readbytes != avr->flashend+1)
	{
		printf(RED "Wrong number of bytes read (read %ld expecting %d)!\n", readbytes, avr->flashend+1);
		exit(1);
	}
	
	fclose(f);
	
	//////
	
	eebuf = malloc(EESIZE);
	if(eebuf == NULL)
	{
		printf(RED "Could not allocate eeprom buf!\n");
		exit(1);
	}
	
	f = fopen(eefile, "rb");
	if(f == NULL)
	{
		printf(RED "Could not open eeprom file!\n");
		exit(1);
	}
	
	readbytes = fread(eebuf, 1, EESIZE, f);
	if(readbytes != EESIZE)
	{
		printf(RED "Wrong number of bytes read (read %ld expecting %d)!\n", readbytes, EESIZE);
		exit(1);
	}
	
	fclose(f);
	
	////
	
	avr_eeprom_desc_t d = { .ee = eebuf, .offset = 0, .size = EESIZE };
	avr_ioctl(avr, AVR_IOCTL_EEPROM_SET, &d);
	free(eebuf);
	
	printf(GREEN "Firmware loading done!\n");
}

void avr_special_deinit(avr_t *avr)
{
	FILE *f;
	size_t wrotebytes;
	unsigned char *eebuf;
	
	printf(GREEN "Deinitializing: saving state\n");
	
	f = fopen(flfile, "wb");
	if(f == NULL)
	{
		printf(RED "Could not open flash file!\n");
		exit(1);
	}
	
	wrotebytes = fwrite(avr->flash, 1, avr->flashend+1, f);
	if(wrotebytes != avr->flashend+1)
	{
		printf(RED "Wrong number of bytes written (wrote %ld expecting %d)!\n", wrotebytes, avr->flashend+1);
		exit(1);
	}
	
	fclose(f);
	
	//////
	
	eebuf = malloc(EESIZE);
	if(eebuf == NULL)
	{
		printf(RED "Could not allocate eeprom buf!\n");
		exit(1);
	}
	
	avr_eeprom_desc_t d = { .ee = eebuf, .offset = 0, .size = EESIZE };
	avr_ioctl(avr, AVR_IOCTL_EEPROM_GET, &d);
	
	f = fopen(eefile, "wb");
	if(f == NULL)
	{
		printf(RED "Could not open eeprom file!\n");
		exit(1);
	}
	
	wrotebytes = fwrite(eebuf, 1, EESIZE, f);
	if(wrotebytes != EESIZE)
	{
		printf(RED "Wrong number of bytes written (wrote %ld expecting %d)!\n", wrotebytes, EESIZE);
		exit(1);
	}
	
	fclose(f);
	free(eebuf);
	
	////
	
	printf(GREEN "Firmware saving done!\n");
}

enum {
	IRQ_485NET_TXE,
	IRQ_485NET_RXE,
	IRQ_485NET_RXBYTE,
	IRQ_485NET_TXBYTE,
	IRQ_485NET_XON,
	IRQ_485NET_XOFF,
	IRQ_485NET_COUNT
};

static const char * irq_names[IRQ_485NET_COUNT] = 
{
	[IRQ_485NET_TXE] = "<485net.txe",
	[IRQ_485NET_RXE] = "<485net.rxe",
	[IRQ_485NET_TXBYTE] = "8<485net.in",
	[IRQ_485NET_RXBYTE] = "8>485net.out",
};

typedef struct net_485net_t
{
	avr_irq_t *irq;
	avr_t *avr;
	
	unsigned char bufout[64];
	unsigned char bufout_fill_to;
	unsigned char bufout_resent_to;
	unsigned char bufin_ext[64*16];
	unsigned char bufin_ext_used[16];
	unsigned char bufin_ext_sent_to;
	int bufcount;
	int tx_is_on;
	int rx_is_on;
	int xon;
} net_485net_t;

static void txe_changed(struct avr_irq_t * irq, uint32_t value, void *param)
{
	//printf(BLUE "txe changed to %d\n" RESET, value);
	net_485net_t *a = (net_485net_t *)(param);
	
	if(value == 1 && !a->tx_is_on)
	{
		printf(BLUE "txe was ENABLED\n" RESET);
		a->tx_is_on = 1;
	}
	if(value == 0 && a->tx_is_on)
	{
		printf(BLUE "txe was DISABLED\n" RESET);
		a->tx_is_on = 0;
		
		printf(BLUE "--%d bytes were output\n", a->bufout_fill_to);
		
		printf("--");
		
		for(int i = 0; i < a->bufout_fill_to; i++)
		{
			printf("%02X ", a->bufout[i]);
			if(i % 16 == 0 && i != 0)
				printf("\n--");
		}
		printf("\n" RESET);
		
		char l = a->bufout_fill_to;
		write(outfifo, &l, 1);	//length
		write(outfifo, a->bufout, l);	//save
		
		a->bufout_fill_to = 0;
		a->bufout_resent_to = 0;
		//this should be safe under most cases
		//controller waits until all bytes rx-ed
		//if tx is off, hw will not re-echo other bytes
	}
}

void advance_bufin(net_485net_t *a)
{
	int i;
	
	a->bufin_ext_sent_to = 0;
	
	for(i=0;i<15;i++)
		a->bufin_ext_used[i] = a->bufin_ext_used[i+1];
	a->bufin_ext_used[i] = 0;
	
	memmove(a->bufin_ext, a->bufin_ext + 64, 64*15);
}

static void rxe_changed(struct avr_irq_t * irq, uint32_t value, void *param)
{
	//printf(BLUE "rxe changed to %d\n" RESET, value);
	net_485net_t *a = (net_485net_t *)(param);
	
	if(value == 0 && !a->rx_is_on)
	{
		printf(BLUE "rxe was ENABLED\n" RESET);
		a->rx_is_on = 1;
	}
	if(value == 1 && a->rx_is_on)
	{
		printf(BLUE "rxe was DISABLED\n" RESET);
		a->rx_is_on = 0;
		
	
		pthread_mutex_lock(&auxfifomutex);
		
		if(a->bufout_resent_to == a->bufout_fill_to)
		{
			//all the "immediate out" stuff is done, so we must have been sending something from the buffer
	
			if(a->bufin_ext_sent_to == 0)
				return;	//nothing has been sent yet
				
			if(a->bufin_ext_sent_to != a->bufin_ext_used[0])
			{
				printf(BLUE "--of the %d bytes, only %d were read\n" RESET, a->bufin_ext_used[0], a->bufin_ext_sent_to);
			}
	
			advance_bufin(a);
		}
		else
		{
			printf(BLUE "--of the %d bytes, only %d were read\n" RESET, a->bufout_fill_to, a->bufout_resent_to);
			a->bufout_resent_to = 0;
		}
		
		pthread_mutex_unlock(&auxfifomutex);
	}
}

static void byte_output(struct avr_irq_t * irq, uint32_t value, void *param)
{
	printf(BLUE "the val %x was output\n" RESET, value);
	net_485net_t *a = (net_485net_t *)(param);
	
	if(!a->tx_is_on)
	{
		printf(BLUE "the byte %02X was lost because tx was off\n" RESET, value);
	}
	else
	{
		if(a->bufout_fill_to == 64)
		{
			printf(BLUE "the byte %02X was lost because it is too long\n" RESET, value);
		}
		else
		{
			a->bufout[a->bufout_fill_to++] = value;
		}
	}
}

int more_bytes_to_send(net_485net_t *a)
{
	if(a->bufout_resent_to != a->bufout_fill_to)
		return 1;
	if(a->bufin_ext_used[0] && (a->bufin_ext_used[0] != a->bufin_ext_sent_to))
		return 1;
	return 0;
}

static void uart_xon(struct avr_irq_t * irq, uint32_t value, void *param)
{
	//printf(RED "xon\n" RESET);
	net_485net_t *a = (net_485net_t *)(param);
	
	a->xon = 1;
	
	pthread_mutex_lock(&auxfifomutex);
	
	while(a->xon && a->rx_is_on && more_bytes_to_send(a))
	{
		if(a->bufout_resent_to != a->bufout_fill_to)
		{
			unsigned char c = a->bufout[a->bufout_resent_to++];
			printf(BLUE "sending from own output the val %02X\n" RESET, c);
			avr_raise_irq(a->irq + IRQ_485NET_RXBYTE, c);
		}
		
		
		if(a->bufin_ext_used[0])
		{
			unsigned char c = a->bufin_ext[a->bufin_ext_sent_to++];
			printf(BLUE "sending from aux queue the val %02X\n" RESET, c);
			avr_raise_irq(a->irq + IRQ_485NET_RXBYTE, c);
			
			if(a->bufin_ext_sent_to == a->bufin_ext_used[0])
				advance_bufin(a);
		}
	}
	
	pthread_mutex_unlock(&auxfifomutex);
}

static void uart_xoff(struct avr_irq_t * irq, uint32_t value, void *param)
{
	printf(GREEN "xoff\n" RESET);
	net_485net_t *a = (net_485net_t *)(param);
	
	a->xon = 0;
}

void init_485hw(avr_t *avr, net_485net_t *a)
{
	memset(a, 0, sizeof(*a));
	a->avr = avr;
	a->irq = avr_alloc_irq(&avr->irq_pool, 0, IRQ_485NET_COUNT, irq_names);
	avr_irq_register_notify(a->irq + IRQ_485NET_TXE, txe_changed, a);
	avr_irq_register_notify(a->irq + IRQ_485NET_RXE, rxe_changed, a);
	avr_irq_register_notify(a->irq + IRQ_485NET_TXBYTE, byte_output, a);
	avr_irq_register_notify(a->irq + IRQ_485NET_XON, uart_xon, a);
	avr_irq_register_notify(a->irq + IRQ_485NET_XOFF, uart_xoff, a);
	avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 0), a->irq + IRQ_485NET_TXE);
	avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 1), a->irq + IRQ_485NET_RXE);
	avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), a->irq + IRQ_485NET_TXBYTE);
	avr_connect_irq(a->irq + IRQ_485NET_RXBYTE, avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT));
	avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XON), a->irq + IRQ_485NET_XON);
	avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XOFF), a->irq + IRQ_485NET_XOFF);
	
	a->rx_is_on = 1;	//default
	
	uint32_t f = 0;
	avr_ioctl(a->avr, AVR_IOCTL_UART_GET_FLAGS('0'), &f);
	f &= ~AVR_UART_FLAG_STDIO;
	avr_ioctl(a->avr, AVR_IOCTL_UART_SET_FLAGS('0'), &f);
}

void fromrosthread(void *a_)
{
	struct pollfd pfd;
	net_485net_t *a = (net_485net_t *)(a_);
	
	pfd.fd = infifo;
	pfd.events = POLLIN|POLLPRI;

	while(1)
	{
		poll(&pfd, 1, -1);
		unsigned char c;
		unsigned char buf[64];
		read(infifo, &c, 1);
		read(infifo, &buf[0], c);
	
		pthread_mutex_lock(&auxfifomutex);
		
		for(int i = 0; i < 16; i++)
		{
			if(a->bufin_ext_used[i] == 0)
			{
				printf("Adding ext data to slot %d\n", i);
				a->bufin_ext_used[i] = c;
				memcpy(&a->bufin_ext[i*64], buf, c);
				break;
			}
		}
		
		pthread_mutex_unlock(&auxfifomutex);
	}
}

int main(int argc, char *argv[])
{
	net_485net_t netdev;
	pthread_t fromrost;
	
	if(argc < 5)
	{
		printf("Usage: %s <persistent flash> <persistent eeprom> <to ros pipe> <from ros pipe>\n", argv[0]);
		return 1;
	}
	
	flfile = argv[1];
	eefile = argv[2];
	
	outfifo = open(argv[3], O_WRONLY);
	infifo = open(argv[4], O_RDONLY);
	
	if(outfifo == -1 || infifo == -1)
	{
		printf("Could not open fifos!\n");
		return 1;
	}
	
	printf(GREEN "flash = %s, eeprom = %s\n", flfile, eefile);

	avr = avr_make_mcu_by_name("atmega328p");
	if (!avr)
	{
		printf(RED "Could not init AVR core!\n");
		return 1;
	}
	
	avr->special_init = avr_special_init;
	avr->special_deinit = avr_special_deinit;
	
	avr_init(avr);
	avr->frequency = 16000000;
	avr->pc = 0x7000;
	avr->blsection = 0x7000;
	
	init_485hw(avr, &netdev);
	
	printf(GREEN "AVR init done!\n" RESET);

	// even if not setup at startup, activate gdb if crashing
	avr->gdb_port = 1234;
	if (0) {
		printf("Waiting for gdb...\n");
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}
	
	pthread_create(&fromrost, NULL, fromrosthread, &netdev);

	int state = cpu_Running;
	while ((state != cpu_Done) && (state != cpu_Crashed))
		state = avr_run(avr);
}
