#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <unistd.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include <fcntl.h>
#include <sys/mman.h>



#define PRU_NUM         		1
#define OFFSET_SHRAM_WRITE		0x64		// 100 general purpose bytes
#define OFFSET_SHRAM_READ		0x1800		//

#define MENSAGEM_ANTIGA			0x55
#define	MENSAGEM_RECEBIDA_NOVA		0x00
#define	MENSAGEM_PARA_ENVIAR		0xff


#define GPIO_DATAOUT 0x13C
#define GPIO_DATAIN 0x138
#define GPIO0_ADDR 0x44E07000
#define GPIO1_ADDR 0x4804C000
#define GPIO2_ADDR 0x481AC000
#define GPIO3_ADDR 0x481AF000
#define	CM_PER_ADDR 0x44E00000
#define	CM_GPIO1	0xAC
#define	CM_GPIO2	0xB0
#define	CM_GPIO3	0xB4


/* PRU SHARED MEMORY (12kB) - MAPPING
 *
 *
 * SHRAM[0]~SHRAM[49] - General Purpose
 *
 * prudata[0] = versao MAX3107 (0x1a)
 * prudata[1] = Status: dados para enviar (0xFF) / dados para ler (0x00) / dados antigos (0x55)
 * prudata[2] = Baudrate (BRGCONFIG)
 * prudata[3] = Baudrate (DIVLSB)
 * prudata[4] = Baudrate (DIVMSB)
 * prudata[5] = procedimento sincrono: START (0xFF) ou STOP (0x00)
 * prudata[6..9] = Timeout
 *
 * prudata[25] = Master/Slave ('M'/'S')
  * prudata[26..28] = 1 Serial Byte length (ns)
 * prudata[29..31] = Delay Sync-Normal command (ns)
 *
 * SHRAM[50]~SHRAM[99] - Sync Operation
 * prudata[50] = data size
 * prudata[51..] = data
 * prudata[80..81] = Pulse counting
 *
 *
 * SHRAM[100] ~ SHRAM[6k-1] - Sending Data
 *
 * prudata[100..103] = Tamanho do vetor de dados
 * prudata[104..] = vetor de dados
 *
 *
 *
 * SHRAM[6k] ~ SHRAM[12k-1] - Receiving Data
 *
 * prudata[6k..6k+3] = Tamanho do vetor de dados
 * prudata[6k+4..] = vetor de dados
 */



volatile uint8_t* prudata;
size_t i;



uint8_t hardware_address_serialPRU(){
	uint8_t endereco = 0x00;

	int fd = open("/dev/mem",O_RDWR | O_SYNC);

	// Habilita GPIO2
	ulong* clk_mngr = (ulong*) mmap(NULL, 0x4000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, CM_PER_ADDR);
	clk_mngr[CM_GPIO2/4] = (clk_mngr[CM_GPIO2/4] & 0xFC) | 0x02;


	ulong* pinconf0 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO0_ADDR);
	ulong* pinconf2 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO2_ADDR);

	if(pinconf0[GPIO_DATAIN/4] & (1 << 10))
		endereco |= 0b00000001;

	if(pinconf0[GPIO_DATAIN/4] & (1 << 11))
		endereco |= 0b00000010;

	if(pinconf0[GPIO_DATAIN/4] & (1 << 9))
		endereco |= 0b00000100;

	if(pinconf2[GPIO_DATAIN/4] & (1 << 17))
		endereco |= 0b00001000;

	if(pinconf0[GPIO_DATAIN/4] & (1 << 8))
		endereco |= 0b00010000;

	close(fd);

	return endereco;
}



int clear_pulse_count_sync(){
        if(prudata[25]=='M' && prudata[5]==0x00){       // Sync disabled. Clear pulse counting
                prudata[80] = 0;
                prudata[81] = 0;
                return 0;
        }
        else 
                return 1;                               // Error: not possible to clear pulse counting
                                                        // while sync operation is enabled.
}




uint16_t read_pulse_count_sync(){
	uint16_t counting = 0;
        if(prudata[25] == 'M') 
                counting = (prudata[81] << 8) + prudata[80];
	return counting;
}




void set_sync_start_PRU(uint32_t delay_ns){
	if(prudata[25]=='M'){
		clear_pulse_count_sync();

		// Delay entre comando de sincronismo e requisicao qualquer
		// ----- Calculo do delay
		for(i=0; i<3; i++)
			delay_ns += prudata[26+i] << i*8;

		// ---- Numero de loops = delay / 10 ns
		delay_ns = delay_ns/10;

		// ----- Armazena numero de instrucoes
		for(i=0; i<3; i++)
			prudata[29+i] = delay_ns >> i*8;
		
		// Inicio modo sincrono
		prudata[5] = 0xff;
	}
}


void set_sync_stop_PRU(){
	if(prudata[25]=='M')
		prudata[5] = 0x00;
}


void close_PRU(){
	// ----- Desabilita PRU e fecha mapeamento da shared RAM
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit();
}


int init_start_PRU(int baudrate, char mode){

	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	// ----- Inicializacao da PRU
	prussdrv_init();

	// ----- Inicializacao da interrupcao PRU
	if (prussdrv_open(PRU_EVTOUT_1)){
		printf("prussdrv_open open failed\n");
		return -1;
	}
	prussdrv_pruintc_init(&pruss_intc_initdata);


	// ----- Mapeamento Shared RAM
	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&prudata);


	// ----- SHRAM - General Purpose = 0x00
	for(i=0; i<100; i++)
		prudata[i] = 0x00;


	// ----- MODO DE OPERACAO: Master/Slave
	if(mode=='M' || mode=='S')
		prudata[25] = mode;
	// Modo nao existente
	else{
		close_PRU();		
		printf("Requested mode does not exist.\n");
		return -1;
	}


	// ----- Inicializacao MASTER: procedimento sincrono desabilitado
	if(prudata[25]=='M')
		set_sync_stop_PRU();

	// ----- Inicializacao SLAVE: nenhuma mensagem nova na serial
	if(prudata[25]=='S')
		prudata[1]=MENSAGEM_ANTIGA;


	// Endereco de Hardware
	prudata[26] = hardware_address_serialPRU();



	// ----- Configuracao Baudrate
	switch (baudrate){
		case 6:
		    prudata[2] = 0x28;		// BRGCONFIG
		    prudata[3] = 0x02;		// DIVLSB
		    prudata[4] = 0x00;		// DIVMSB
		    one_byte_length_ns = (int) 10000/6;
		    break;

		case 10:
		    prudata[2] = 0x28;		// BRGCONFIG
		    prudata[3] = 0x01;		// DIVLSB
		    prudata[4] = 0x00;		// DIVMSB
		    one_byte_length_ns = (int) 10000/10;
		    break;

		case 12:
		    prudata[2] = 0x24;		// BRGCONFIG
		    prudata[3] = 0x01;		// DIVLSB
		    prudata[4] = 0x00;		// DIVMSB
		    one_byte_length_ns = (int) 10000/12;
		    break;

		case 9600:
			prudata[2] = 0x0a;	// BRGCONFIG
			prudata[3] = 0x86;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			one_byte_length_ns = (int) 100000000/96;
			break;

		case 14400:
			prudata[2] = 0x07;	// BRGCONFIG
			prudata[3] = 0x04;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			one_byte_length_ns = (int) 100000000/144;
			break;

		case 19200:
			prudata[2] = 0x05;	// BRGCONFIG
			prudata[3] = 0xc3;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			one_byte_length_ns = (int) 100000000/192;
			break;

		case 38400:
			prudata[2] = 0x15;	// BRGCONFIG
			prudata[3] = 0xc3;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			one_byte_length_ns = (int) 100000000/384;
			break;

		case 57600:
			prudata[2] = 0x27;	// BRGCONFIG
			prudata[3] = 0x04;	// DIVLSB
			prudata[4] = 0x01;	// DIVMSB
			one_byte_length_ns = (int) 100000000/576;
			break;

		case 115200:
			prudata[2] = 0x09;	// BRGCONFIG
			prudata[3] = 0x20;	// DIVLSB
			prudata[4] = 0x00;	// DIVMSB
			one_byte_length_ns = (int) 100000000/1152;
			break;

		default:
			close_PRU();		// Nao definido
			printf("Baudrate not defined.\n");
			return -1;
	}
	prudata[26] = one_byte_length_ns;
	prudata[27] = one_byte_length_ns >> 8;
	prudata[28] = one_byte_length_ns >>16;
	
	

	// ----- Instrucao - Passo Sync
	prudata[50] = 0x06;				// Tamanho
	prudata[51] = 0xff;				// |
	prudata[52] = 0x50;				// |
	prudata[53] = 0x00;				// | Comando a ser enviado
	prudata[54] = 0x01;				// | apos pulso de sync
	prudata[55] = 0x05;				// |
	prudata[56] = 0xab;				// |


	// ----- Executar codigo na PRU
	prussdrv_exec_program (PRU_NUM, "/usr/bin/PRUserial485.bin");

	return 0;
}


int send_data_PRU(uint8_t *data, uint32_t *tamanho, float timeout_ms){

	uint32_t timeout_instructions = 0;

	timeout_instructions = (int) timeout_ms*66600;


	// ----- MASTER: Configuracao do Timeout
	if(prudata[25]=='M'){
		prudata[6] = timeout_instructions;			// LSByte do timeout_instructions [7..0]
		prudata[7] = timeout_instructions >> 8;			// Byte do timeout_instructions [15..8]
		prudata[8] = timeout_instructions >> 16;		// Byte do timeout_instructions [23..16]
		prudata[9] = timeout_instructions >> 24;		// MSByte do timeout_instructions [31..24]
	}

	// ----- Tamanho dos dados
	prudata[OFFSET_SHRAM_WRITE] = *tamanho;				// LSByte do tamanho dos dados [7..0]
	prudata[OFFSET_SHRAM_WRITE+1] = *tamanho >> 8;		// MSByte do tamanho dos dados [15..8]
	prudata[OFFSET_SHRAM_WRITE+2] = *tamanho >> 16;		// MSByte do tamanho dos dados [23..16]
	prudata[OFFSET_SHRAM_WRITE+3] = *tamanho >> 24;		// MSByte do tamanho dos dados [31..24]


	// ----- Insere na memoria - Dados a enviar
	for(i=0; i<*tamanho; i++)
		prudata[OFFSET_SHRAM_WRITE+4+i] = data[i];

	// ----- Dados prontos para envio
	prudata[1] = MENSAGEM_PARA_ENVIAR;

	// ----- Aguarda sinal de finalizacao do ciclo
	prussdrv_pru_wait_event(PRU_EVTOUT_1);

	// ----- Clear evento
	prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);

	// ----- SLAVE: Aguarda fim de envio
	if(prudata[25]=='S')
		while(prudata[1] != 0x55);

	return 0;
}


int recv_data_PRU(uint8_t *data, uint32_t *tamanho){
	// ---------- MASTER MODE ----------
	if(prudata[25]=='M'){
		// Aguarda dados prontos na Shared RAM
		while(prudata[1] != 0x00){
		}

		*tamanho = 0;

		// ----- Copia dos dados recebidos
		// Tamanho
		for(i=0; i<4; i++)
			*tamanho += prudata[OFFSET_SHRAM_READ+i] << i*8;
		// Dados
		for(i=0; i<*tamanho; i++)
			data[i] = prudata[OFFSET_SHRAM_READ+4+i];

		return 0;
	}


	// ---------- SLAVE MODE ----------
	if(prudata[25]=='S'){

		// ----- Nova mensagem recebida !
		if(prudata[1] == MENSAGEM_RECEBIDA_NOVA){ 
			*tamanho = 0;

			// ----- Copia dos dados recebidos
			// Tamanho
			for(i=0; i<4; i++)
				*tamanho += prudata[OFFSET_SHRAM_READ+i] << i*8;
			// Dados
			for(i=0; i<*tamanho; i++)
				data[i] = prudata[OFFSET_SHRAM_READ+4+i];

			// ----- Sinaliza mensagem antiga
			prudata[1] = MENSAGEM_ANTIGA;

			return 0;
		}


		// ----- Mensagem antiga no buffer
		if(prudata[1] == MENSAGEM_ANTIGA)
			return -1;
	}
}