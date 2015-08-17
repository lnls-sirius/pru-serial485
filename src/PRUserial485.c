#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include <unistd.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM         		1
#define OFFSET_SHRAM_WRITE		0x64		// 100 general purpose bytes
#define OFFSET_SHRAM_READ		0x1800		//



/* PRU SHARED MEMORY (12kB) - MAPPING
 *
 *
 * SHRAM[0]~SHRAM[49] - General Purpose
 *
 * prudata[0] = MAX3107 version (0x1a)
 * prudata[1] = data ready to be sent (0xFF) / Data ready to be read (0x00)
 * prudata[2] = Baudrate (BRGCONFIG)
 * prudata[3] = Baudrate (DIVLSB)
 * prudata[4] = Baudrate (DIVMSB)
 * prudata[5] = procedimento sincrono: START (0xFF) ou STOP (0x00)
 *
 *
 *
 * SHRAM[50]~SHRAM[99] - Sync Operation
 * prudata[50] = data size
 * prudata[51..] = data
 *
 *
 *
 * SHRAM[100] ~ SHRAM[6k-1] - Sending Data
 *
 * prudata[100..103] = size of vector
 * prudata[104..] = vector
 *
 *
 *
 * SHRAM[6k] ~ SHRAM[12k-1] - Receiving Data
 *
 * prudata[6k..6k+3] = size of vector
 * prudata[6k+4..] = vector
 */



volatile uint8_t* prudata;
size_t i;



void set_sync_start_PRU(){
	prudata[5] = 0xff;
}


void set_sync_stop_PRU(){
	prudata[5] = 0x00;
}



void close_PRU(){
	// Disable PRU and close memory mapping //
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit ();
}


int init_start_PRU(int baudrate){

	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	// Initialize the PRU //
	prussdrv_init();

	// Open PRU Interrupt and get it initialized //
	if (prussdrv_open(PRU_EVTOUT_1)) {
		printf("prussdrv_open open failed\n");
		return -1;
	}
	prussdrv_pruintc_init(&pruss_intc_initdata);


	// Shared RAM mapping //
	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&prudata);

	// SHRAM - General Purpose = 0x00
	for(i=0; i<100; i++)
		prudata[i] = 0x00;

	// Initialization: sync proc disabled //
	set_sync_stop_PRU();



	// Baudrate config
	switch (baudrate){
		case 6:
			prudata[2] = 0x28;	// BRGCONFIG
		    prudata[3] = 0x02;	// DIVLSB
		    prudata[4] = 0x00;	// DIVMSB
		    break;

		case 10:
		    prudata[2] = 0x28;	// BRGCONFIG
		    prudata[3] = 0x01;	// DIVLSB
		    prudata[4] = 0x00;	// DIVMSB
		    break;

		case 12:
		    prudata[2] = 0x24;	// BRGCONFIG
		    prudata[3] = 0x01;	// DIVLSB
		    prudata[4] = 0x00;	// DIVMSB
		    break;

		case 9600:
			prudata[2] = 0x0a;	// BRGCONFIG
			prudata[3] = 0x86;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			break;

		case 14400:
			prudata[2] = 0x07;	// BRGCONFIG
			prudata[3] = 0x04;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			break;

		case 19200:
			prudata[2] = 0x05;	// BRGCONFIG
			prudata[3] = 0xc3;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			break;

		case 38400:
			prudata[2] = 0x15;	// BRGCONFIG
			prudata[3] = 0xc3;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			break;

		case 57600:
			prudata[2] = 0x27;	// BRGCONFIG
			prudata[3] = 0x04;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			break;

		case 115200:
			prudata[2] = 0x09;	// BRGCONFIG
			prudata[3] = 0x20;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			break;

		default:
			close_PRU();		// If requested baudrate is not defined
			return -1;
	}

	// Sync - Instruction
	prudata[50] = 0x06;				// Size
	prudata[51] = 0xff;				// |
	prudata[52] = 0x50;				// |
	prudata[53] = 0x00;				// | Command to be sent
	prudata[54] = 0x01;				// | after a sync pulse
	prudata[55] = 0x05;				// |
	prudata[56] = 0xab;				// |

	// Execute code on PRU //
	prussdrv_exec_program (PRU_NUM, "/usr/bin/485-BBB.bin");

	return 0;
}


int send_data_PRU(uint8_t *data, uint32_t *tamanho){

	prudata[OFFSET_SHRAM_WRITE] = *tamanho;				// LSByte do tamanho dos dados [7..0]
	prudata[OFFSET_SHRAM_WRITE+1] = *tamanho >> 8;		// MSByte do tamanho dos dados [15..8]
	prudata[OFFSET_SHRAM_WRITE+2] = *tamanho >> 16;		// MSByte do tamanho dos dados [23..16]
	prudata[OFFSET_SHRAM_WRITE+3] = *tamanho >> 24;		// MSByte do tamanho dos dados [31..24]

	// Insere na memoria - Dados a enviar
	for(i=0; i<*tamanho; i++)
		prudata[OFFSET_SHRAM_WRITE+4+i] = data[i];


	// Data is ready to be sent
	prudata[1] = 0xff;

	// Wait until PRU1 has finished execution //
	prussdrv_pru_wait_event (PRU_EVTOUT_1);

	while(prudata[1] != 0x00){
	}

	// Clear event
	prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);

	return 0;
}


int recv_data_PRU(uint8_t *data, uint32_t *tamanho){

	*tamanho = 0;

	// Data length
	for(i=0; i<4; i++)
		*tamanho += prudata[OFFSET_SHRAM_READ+i] << i*8;

	// Data
	for(i=0; i<*tamanho; i++)
		data[i] = prudata[OFFSET_SHRAM_READ+4+i];

	return 0;
}
