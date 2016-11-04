// ----- PRUserial485 -----

#define OWN_RAM              		0x000
#define OTHER_RAM           		0x020
#define SHARED_RAM          	 	0x100

#define OFFSET_SHRAM_READ			0x64		// 100 general purpose bytes
#define OFFSET_SHRAM_WRITE			r19			// 6kB (0x1800) offset to be set
#define OFFSET_SHRAM_SYNC			0x32		// 50 - Start of sync message
#define OFFSET_SHRAM_SYNC_COUNT			0x50		// 80 - Start of sync pulse counting
#define OFFSET_SHRAM_SYNC_DELAY		29		    // Time between sync and normal command
#define OFFSET_SHRAM_SYNC_OK			0x54		// 84 - Sync OK
#define OFFSET_SHRAM_SYNC_MODE			0x55		// 85 - Sync Mode

#define OFFSET_SEND_COMMAND			0x69



#define CLK							r30.t0		// P8.45
#define MISO						r31.t1		// P8.46
#define MOSI						r30.t4		// P8.41
#define CHIP_SELECT					r30.t2		// P8.43
#define IRQ							r31.t6		// P8.39
#define SYNC						r31.t8		// P8.27
#define	LED_READ					r30.t7		// P8.40
#define LED_WRITE					r30.t9		// P8.29
#define LED_IDLE					r30.t10		// P8.28


#define DDR_POINTER						r12
#define DDR_BASE						r9
#define CURVE						r11
#define CURVE_SIZE					r8
#define CHECKSUM_POINT						r10
#define A							r16
#define B							r17
#define	K							r18
#define BUFFER_SPI_IN				r20
#define BUFFER_SPI_OUT				r21
#define TAMANHO						r22
#define WOFFSET						r23
#define BLOCKS						r24
#define I							r25
#define FIFO_LENGHT					r26
#define	TIMEOUT_VALUE				r27
#define	J							r28
#define OPERATION_MODE				r15
#define ENDERECO_HARDWARE			r14
#define PISCA_LED				r13


#define UART_MODES_ADDRESS			0x89	// UART_WRITE_END
#define MODE_1						0x90	// MODE 1 ----
#define MODE_2						0x80	// MODE 2 ---- EchoSuppr
#define LCR							0x03	// LCR
#define RXTIMEOUT					0x13	// RxTimeOut -- 2 words
#define	HDPLXDELAY					0x11	// HDplxDelay -- 1 bit

#define CLOCK_CONFIG_ADDRESS		0x9a	// CLOCK_WRITE_END
#define PLLCONFIG					0xa0	// PLLConfig
#define BRGCONFIG					0x28	// BRGConfig	- Available in shared RAM (User defined)
#define DIVLSB						0x02	// DIVLSB		- Available in shared RAM (User defined)
#define DIVMSB						0x00	// DIVMSB
#define CLKSOURCE					0x14	// CLKSource

#define ISR_REGISTER_ADDRESS		0x02

#define INTERRUPTS_IRQEN_ADDRESS	0x81	// INTERRUPT_IRQEn_WRITE_END
#define IRQEN						0x01	// IRQEn -- LSRErrEn

#define INTERRUPTS_LSREN_ADDRESS	0x83	// INTERRUPT_LSRIntEn_WRITE_END
#define LSREN						0x03	// LSRIntEn -- RTimeoutEn


#define MENSAGEM_ANTIGA					0x55
#define	MENSAGEM_RECEBIDA_NOVA			0x00
#define	MENSAGEM_PARA_ENVIAR			0xff
#define SYNC_OK                   0xFF
#define SYNC_NOK                  0x00
#define SYNC_SINGLE               0x50
#define SYNC_CONTINUOUS           0xC0
#define SYNC_END                  0x0E
#define SYNC_INTERCAL             0x01
#define SYNC_BROADCAST							0x0B
#define ENABLED_SYNC               0xFF
#define DISABLED_SYNC              0x00



.setcallreg r29.w0					// PC saving register (CALL instructions)
.origin 0							// start of program in PRU memory
.entrypoint START					// program entry point (for a debugger)

#include "PRUserial485.hp"
#define AM33XX


START:

// ----- Initial Configuration --------------------------------------------------------------------

	CLR LED_READ
	CLR LED_WRITE
	CLR LED_IDLE


// OFFSET_SHRAM_WRITE
	ZERO	&OFFSET_SHRAM_WRITE, 4
	MOV		OFFSET_SHRAM_WRITE, 0x18
	LSL		OFFSET_SHRAM_WRITE, OFFSET_SHRAM_WRITE, 8	// SHRAM_WRITE offset = 0x1800

// DDR_BASE e DDR_POINTER
	LBCO	DDR_BASE,SHRAM_BASE,15,4
        ZERO    &DDR_POINTER, 4



//  Shared Memory Config
	SHRAM_CONFIG

	RESET_UART



// Wait for UART to start
	WBS		IRQ

	READ_ISR
	READ_LSR



// ~~~~~ MAX3107 Configuration Registers ~~~~~

	CS_DOWN
	SEND_SPI UART_MODES_ADDRESS, 8
	SEND_SPI MODE_1, 8
	SEND_SPI MODE_2, 8
	SEND_SPI LCR, 8
    LBCO	I, SHRAM_BASE, 32, 1								// RXTIMEOUT
	SEND_SPI I, 8											    //
	//SEND_SPI RXTIMEOUT, 8
	SEND_SPI HDPLXDELAY, 8
	CS_UP

	CS_DOWN
	SEND_SPI CLOCK_CONFIG_ADDRESS, 8
	SEND_SPI PLLCONFIG, 8
	LBCO	I, SHRAM_BASE, 2, 1								// BRGCONFIG
	SEND_SPI I, 8											//
	LBCO	I, SHRAM_BASE, 3, 1								// DIVLSB
	SEND_SPI I, 8											//
	LBCO	I, SHRAM_BASE, 4, 1								// DIVMSB
	SEND_SPI I, 8											//
	SEND_SPI CLKSOURCE, 8
	CS_UP


	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x00, 8
	CS_UP

	CS_DOWN
	SEND_SPI INTERRUPTS_LSREN_ADDRESS, 8
	SEND_SPI LSREN, 8
	CS_UP

	// MAX3107 RevID
	CS_DOWN
	SEND_SPI 0x1f, 8
	RECEIVE_SPI 8
	SBCO BUFFER_SPI_IN, SHRAM_BASE, 0, 1
	CS_UP

// ------------------------------------------------------------------------------------------------




// ~~~~~ ENDERECO EM HARDWARE ~~~~~~~~~~~~~~
	ZERO	&ENDERECO_HARDWARE, 4			// Define endereco fisico da placa no no
	LBCO	ENDERECO_HARDWARE, SHRAM_BASE, 24, 1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ~~~~~ BASE PARA CURVA ~~~~~~~~~~~~~~~~~~~
	LBCO    DDR_BASE,SHRAM_BASE,15,4
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Not waiting for sync ~~~~~~~~~~~~~~
  ZERO 	&I, 4
  ADD I, I, SYNC_NOK
  SBCO 	I,SHRAM_BASE,OFFSET_SHRAM_SYNC_OK,1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ MASTER/SLAVE MODE ~~~~~~~~~~~~~~~~~
	ZERO	&OPERATION_MODE, 4				// Verifica se opera em modo Master ou Slave
	LBCO	OPERATION_MODE, SHRAM_BASE, 25, 1		// OPERATION_MODE <- shram[25] ('M' ou 'S')

	QBEQ	PROCEDURE_START_MASTER, OPERATION_MODE, 0x4d 	// 0x4D = 'M'
	QBEQ	PROCEDURE_START_SLAVE, OPERATION_MODE, 0x53 	// 0x53 = 'S'
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::: MASTER MODE :::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


PROCEDURE_START_MASTER:

	NOP

// ~~~~~ Verifica modo de operacao Master ~~
OPERATION_MODE_MASTER:

	ZERO	&I, 4
	LBCO	I, SHRAM_BASE, 5, 1								// 0xFF : Modo Sincrono
	QBNE	WAIT_FOR_DATA, I, ENABLED_SYNC			// 0x00 : Modo Normal
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ----- PROCEDIMENTO SINCRONO --------------------------------------------------------------------
//	Prepara comando de sincronismo
	ZERO 	&BLOCKS, 4
	LBCO	BLOCKS, SHRAM_BASE, OFFSET_SHRAM_SYNC, 1	// Tamanho do comando

	ZERO 	&I, 4
	MOV	I,OFFSET_SHRAM_SYNC				// I: ponteiro para início dos dados - SHRAM[50]
	ADD 	BLOCKS,BLOCKS,I					// Blocks: endereco do ultimo byte do comando


// VERIFICA SE BROADCAST - Se sim, pula para "WAIT_SYNC"
	ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0x0F
	XOR     I, I, SYNC_BROADCAST
  QBEQ    WAIT_SYNC, I, 0x00



//	Prepara tamanho da curva sincronismo
	ZERO 	&CURVE_SIZE, 4
	LBCO	CURVE_SIZE, SHRAM_BASE, 20, 4			// Tamanho de bytes da curva


// 	Calcula checksum inicial
	ZERO 	&I, 4
	MOV	I,OFFSET_SHRAM_SYNC				// I: ponteiro para início dos dados - SHRAM[50]
	ZERO 	&CHECKSUM_POINT, 4
	ZERO	&A,4
CS:	ADD	I,I,1
	LBCO	A, SHRAM_BASE, I, 1
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,A
	QBNE	CS,I,55						// Se I == ENDERECO DO ULTIMO CABECALHO, termina loop


WAIT_SYNC:
	READ_ISR
	READ_LSR

	CS_DOWN
	SEND_SPI 0x80,8						// Comando Envio (MAX3107)


// Says it is ready for sync
  ZERO 	&I, 4
  ADD I, I, SYNC_OK
  SBCO 	I,SHRAM_BASE,OFFSET_SHRAM_SYNC_OK,1


// Wait for sync pulse: Apenas borda de subida
	WBC SYNC
	WBS SYNC

// VERIFICA SE BROADCAST - Se sim, pula para "SEND_SYNC"
	ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0x0F
	XOR     I, I, SYNC_BROADCAST
  QBEQ    SEND_SYNC, I, 0x00



READ_DATA_POINTER:
	// DDR_BASE e DDR_POINTER
	LBCO	DDR_POINTER,SHRAM_BASE,10,4
	// Verifica se esta no inicio da curva. Se SIM, seleciona bloco
	ZERO	&I, 4
	QBNE	POINTS_CURVE, DDR_POINTER, I
	LBCO	DDR_BASE,SHRAM_BASE,15,4

POINTS_CURVE:
	LBBO	BUFFER_SPI_OUT, DDR_BASE, DDR_POINTER, 4
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b0
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b1
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b2
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b3
	SBCO 	BUFFER_SPI_OUT,SHRAM_BASE,56,4
	ADD	DDR_POINTER,DDR_POINTER,4

	LBBO	BUFFER_SPI_OUT, DDR_BASE, DDR_POINTER, 4
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b0
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b1
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b2
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b3
	SBCO 	BUFFER_SPI_OUT,SHRAM_BASE,60,4
	ADD	DDR_POINTER,DDR_POINTER,4

	LBBO	BUFFER_SPI_OUT, DDR_BASE, DDR_POINTER, 4
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b0
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b1
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b2
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b3
	SBCO 	BUFFER_SPI_OUT,SHRAM_BASE,64,4
	ADD	DDR_POINTER,DDR_POINTER,4

	LBBO	BUFFER_SPI_OUT, DDR_BASE, DDR_POINTER, 4
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b0
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b1
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b2
	SUB	CHECKSUM_POINT,CHECKSUM_POINT,BUFFER_SPI_OUT.b3
	SBCO 	BUFFER_SPI_OUT,SHRAM_BASE,68,4
	ADD	DDR_POINTER,DDR_POINTER,4

	SBCO	CHECKSUM_POINT,SHRAM_BASE,72,1


//	Envia comando de sincronismo
SEND_SYNC:
	ZERO 	&I, 4
	MOV	I,OFFSET_SHRAM_SYNC				// I: ponteiro para início dos dados - SHRAM[50]


LOAD_FROM_MEMORY_SYNC:
	ADD	I,I,1
	LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1		// Carrega byte da shram
	SEND_SPI BUFFER_SPI_OUT,8				// Envia byte
	QBNE	LOAD_FROM_MEMORY_SYNC,I,BLOCKS			// Se I == NUMERO DE BYTES, termina loop

	CS_UP


// Le TX FIFO Level - Aguarda ate final do envio
WAIT_TX_ZERO:
	CS_DOWN
	SEND_SPI 0x11, 8
	RECEIVE_SPI  8
	CS_UP

	QBLT	WAIT_TX_ZERO, BUFFER_SPI_IN, 0x08	// Aguarda Buffer TX < 8

// VERIFICA SE BROADCAST - Se sim, pula para "UPDATE_PULSE_COUNTING"
	ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0x0F
	XOR     I, I, SYNC_BROADCAST
  QBEQ    UPDATE_PULSE_COUNTING, I, 0x00



// ---- Reseta ponteiro se está no final da curva
	QBNE	UPDATE_DATA_POINTER,DDR_POINTER,CURVE_SIZE
	ZERO	&DDR_POINTER,4


UPDATE_DATA_POINTER:
	SBCO	DDR_POINTER,SHRAM_BASE,10,4

UPDATE_PULSE_COUNTING:
        ZERO    &I, 4
        LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_COUNT,4   //Load current pulse count
	ADD	I,I,1
	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_SYNC_COUNT,4		// Store pulse count++



// VERIFICA SE BROADCAST - Se sim, pula para "END_SINGLE_SEQ"
	ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0x0F
	XOR     I, I, SYNC_BROADCAST
  QBEQ    END_SINGLE_SEQ, I, 0x00




// VERIFICACAO DE MODO SINCRONO
// Verifica se esta no final ou no meio da curva
  ZERO    &I, 4 // I = 0
  QBEQ	END_OF_CURVE, DDR_POINTER, I
  JMP   MIDDLE_CURVE


// MEIO DA CURVA
// Verifica se mensagens sao enviadas intecaladas ou apenas no final
MIDDLE_CURVE:
  ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0x0F
	XOR     I, I, SYNC_INTERCAL
MESSAGE_INTERCAL:
  QBEQ    DELAY_CONFIG, I, 0x00 // Se I == 0,Sync_Intercal
MESSAGE_END:
  JMP     OPERATION_MODE_MASTER


// FIM DA CURVA
// Verifica se e modo single ou continuous
END_OF_CURVE:
  ZERO    &I, 4
  LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
	AND			I, I, 0xF0
	XOR     I, I, SYNC_CONTINUOUS
  QBEQ    END_CONTINUOUS_SEQ, I, 0x00  // Se I == 0,Sync_Continuous

END_SINGLE_SEQ:
// Disable sync mode
  ZERO    &I, 4
  ADD     I, I, DISABLED_SYNC
  SBCO    I, SHRAM_BASE, 5, 1

// Says it wont wait for sync pulse
  ZERO 	&I, 4
  ADD I, I, SYNC_NOK
  SBCO 	I,SHRAM_BASE,OFFSET_SHRAM_SYNC_OK,1

  JMP     DELAY_CONFIG

END_CONTINUOUS_SEQ:
  JMP     DELAY_CONFIG







// ------------------------------------------------------------------------------------------------
// Delay: aguarda antes de enviar requisicao normal
DELAY_CONFIG:
	ZERO	&I, 4
	ZERO	&TIMEOUT_VALUE, 4
	LBCO	TIMEOUT_VALUE, SHRAM_BASE, OFFSET_SHRAM_SYNC_DELAY, 3
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

LOOP_DELAY:
	ADD	I,I,1
	QBNE	LOOP_DELAY, I, TIMEOUT_VALUE

// ------------------------------------------------------------------------------------------------



// ----- PROCEDIMENTO CONSTANTE -------------------------------------------------------------------
// Verifica se os dados já estão prontos para serem enviados: SHRAM[1] = 0xFF
WAIT_FOR_DATA:

	LBCO	I, SHRAM_BASE, 1, 1
	QBNE	OPERATION_MODE_MASTER, I.b0, 0xff	// Se nao, volta a verificar condicao de sincronismo



 // ----- ENVIAR DADOS ----------------------------------------------------------------------------
	RESET_FIFO_UART
	CALL SEND_DATA_UART

 // Aguardar resposta
 // -----------------------------------------------------------------------------------------------


 // ----- RECEBER DADOS ---------------------------------------------------------------------------
 RECEIVE_DATA:

// ~~~~~ Requer resposta? ~~~~~~~~~~~~~~~~~~
        LBCO    TIMEOUT_VALUE, SHRAM_BASE, 6, 4         // Se Timeout == 0, nao aguardar resposta.
        QBNE    RECEIVE_DATA_OK, TIMEOUT_VALUE, 0x00

        JMP     TIMEOUT_AND_NORESPONSE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 RECEIVE_DATA_OK:

 // ~~~~~ Int por RxFIFO nao-vazio ~~~~~~~~~
	CS_DOWN
	SEND_SPI 0x8a, 8				// Mode2 Address
	SEND_SPI 0x88, 8				// Mode2 = RxEmtyEn & EchoSuprs
	CS_UP

	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x40, 8				// RxEmtyEn
	CS_UP
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Configuracao Timeout ~~~~~~~~~~~~~~
	LBCO	TIMEOUT_VALUE, SHRAM_BASE, 6, 4
	ZERO	&I, 4
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Loop: Resposta ou timeout ? ~~~~~~~
WAIT_RECEIVED:

	QBEQ	TIMEOUT_AND_NORESPONSE, I, TIMEOUT_VALUE	// Timeout atingido: finalizar ciclo
	ADD	I,I,1

	QBBS	WAIT_RECEIVED, IRQ				// Dados sendo recebidos: quebra loop e continua
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Habilita Interrupcao RxTimeout
	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x01, 8				// Apenas LSRErrEn
	CS_UP

// Desabilita RxEmptyInv
	CS_DOWN
	SEND_SPI 0x8a, 8
	SEND_SPI 0x80, 8				// Apenas EchoSuprs
	CS_UP

	READ_ISR
	READ_LSR

// Configura pointer para SHRAM_WRITE
	ZERO	&I, 4
	ADD	I,OFFSET_SHRAM_WRITE, 3			// I: ponteiro para início dos dados -  SHRAM[0x1800 + 3]



// Le RxLevel até ser >= 64 e verifica IRQ (RxTimeout). Se Timeout_FimMensagem, ARMAZENA BYTES RESTANTES
RXLEVEL_AND_TIMEOUT:

	QBBC 	STORE_LEFTBYTES, IRQ			// RxTimeout Interrupt. Fim de mensagem.

	// Le Rx FIFO Level
	CS_DOWN
	SEND_SPI 0x12, 8
	RECEIVE_SPI  8
	CS_UP

	LSR BUFFER_SPI_IN, BUFFER_SPI_IN, 6
	QBEQ	RXLEVEL_AND_TIMEOUT, BUFFER_SPI_IN, 0	// Aguarda Buffer TX >= 0x0100 0000 = 64


 // Armazena 64 bytes na SHRAM
STORE_64BYTES:
	ZERO	&J,4					// J: contador do loop
	CS_DOWN
	SEND_SPI 0x00,8					// Comando Read

STORE_64_MEMORY:
	ADD	I,I,1					// Inicio em SHRAM[0x1800 + 4]
	RECEIVE_SPI 8					// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1		// Armazena no byte I da shram
	ADD	J,J,1

	QBNE	STORE_64_MEMORY,J,0x40			// Se J == 64, termina loop
	CS_UP

	JMP 	RXLEVEL_AND_TIMEOUT
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 // Armazena bytes restantes - Interrupcao RxTimeout
STORE_LEFTBYTES:
 // Le Rx FIFO Level
	CS_DOWN
	SEND_SPI 0x12, 8
	RECEIVE_SPI  8
	CS_UP

	ZERO	&J,4
	MOV	J,BUFFER_SPI_IN				// J = quantia de bytes no RxFIFO (MAX3107)

	CS_DOWN
	SEND_SPI 0x00,8					// Comando Read

STORE_LAST64_MEMORY:
	ADD	I,I,1					// SHRAM[0x1800 + 4]
	RECEIVE_SPI 8						// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1		// Armazena no byte I da shram
	SUB		J,J,1

	QBNE	STORE_LAST64_MEMORY,J,0x00		// Se J == 0, termina loop
	CS_UP


 // ~~~~~ TAMANHO DOS DADOS ~~~~~~~~~~~~~~~~
	ZERO	&J,4
	ADD		J,OFFSET_SHRAM_WRITE,3		// J = 0x1803
	SUB		I, I, J				// I = I - J = tamanho

	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_WRITE, 4 	// Armazena tamanho nos primeiros bytes
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	READ_ISR
	READ_LSR

// Procedimento concluido
 	JMP	DATA_READY
// ------------------------------------------------------------------------------------------------





// ----- TIMEOUT e SEM RESPOSTA - Tratamento de dados ---------------------------------------------
TIMEOUT_AND_NORESPONSE:

	ZERO	&I, 4
	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_WRITE, 4	// Tamanho = 0x00
	JMP	DATA_READY				// Finaliza execucao do comando
// ------------------------------------------------------------------------------------------------



// ----- FINALIZACAO - Finaliza e aguarda por novo envio de dados  --------------------------------
DATA_READY:

	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x00, 8				// Desabilita interrupcoes
	CS_UP

    MOV     I, MENSAGEM_RECEBIDA_NOVA
	SBCO	I, SHRAM_BASE, 1, 1			// Confirma Dados Recebidos prudata[1]=0x00


	MOV 	r31.b0, PRU1_ARM_INTERRUPT+16

	JMP	PROCEDURE_START_MASTER
// ------------------------------------------------------------------------------------------------
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::










// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::: SLAVE MODE ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

PROCEDURE_START_SLAVE:
	ZERO    &J, 4


START_SLAVE:
    NOP
	//READ_ISR
	//READ_LSR


 // ----- RECEBER DADOS ---------------------------------------------------------------------------
 RECEIVE_DATA_SLAVE:

// Reset FIFO UART
	//RESET_FIFO_UART

 // ~~~~~ Int por RxFIFO nao-vazio ~~~~~~~~~
	CS_DOWN
	SEND_SPI 0x8a, 8				// Mode2 Address
	SEND_SPI 0x88, 8				// Mode2 = RxEmtyInv & EchoSuprs
	CS_UP


	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x40, 8				// RxEmtyEn
	CS_UP
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	READ_ISR
    CLR LED_IDLE

//WAIT_BUFFER_READ:
// ----- Verifica se buffer de leitura foi lido  -----
//    LBCO	I, SHRAM_BASE, 1, 1
//    QBNE	WAIT_BUFFER_READ, I.b0, 0x55		// 0x55 : Mensagem antiga
// -------------------------------------------

// ~~~~~ Verifica recepcao de dados ~~~~~~~~
WAIT_RECEIVED_SLAVE:

// ----- Verifica se há algo para enviar -----
// Prontos para serem enviados: SHRAM[1] = 0xFF
	LBCO	I, SHRAM_BASE, 1, 1			// 0xFF : Dados a enviar
	QBEQ	SEND_DATA_SLAVE, I.b0, 0xff		// 0x55 : Nada a enviar
// -------------------------------------------

	QBBS	WAIT_RECEIVED_SLAVE, IRQ		// Data not received, loop continues
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// Habilita Interrupcao RxTimeout
	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x01, 8				// Apenas LSRErrEn
	CS_UP

// Desabilita RxEmptyInv
	CS_DOWN
	SEND_SPI 0x8a, 8
	SEND_SPI 0x80, 8				// Apenas EchoSuprs
	CS_UP

	READ_ISR
	READ_LSR

// Configura ponteiro para SHRAM_WRITE
	ZERO	&I, 4
	ADD	I,OFFSET_SHRAM_WRITE, 3			// I: ponteiro para início dos dados -  SHRAM[0x1800 + 3]



// Le RxLevel até ser >= 32 e verifica IRQ (RxTimeout). Se Timeout_FimMensagem, ARMAZENA BYTES RESTANTES
RXLEVEL_AND_TIMEOUT_SLAVE:

	QBBC 	STORE_LEFTBYTES_SLAVE, IRQ		// RxTimeout Interrupt. Fim de mensagem

	// Le Rx FIFO Level
	CS_DOWN
	SEND_SPI 0x12, 8
	RECEIVE_SPI  8
	CS_UP

	LSR BUFFER_SPI_IN, BUFFER_SPI_IN, 5
	QBEQ	RXLEVEL_AND_TIMEOUT_SLAVE, BUFFER_SPI_IN, 0	// Aguarda Buffer TX >= 0x0001 0000 = 32


 // Armazena 16 bytes na SHRAM
STORE_16BYTES_SLAVE:
	ZERO	&J,4						// J: contador do loop
	CS_DOWN
	SEND_SPI 0x00,8						// Comando Read

STORE_16_MEMORY_SLAVE:
	ADD	I,I,1						// SHRAM[0x1800 + 4]
	RECEIVE_SPI 8						// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1			// Armazena no byte I da shram
	ADD		J,J,1

	QBNE	STORE_16_MEMORY_SLAVE,J,0x20			// Se J == 32, termina loop
	CS_UP

	JMP 	RXLEVEL_AND_TIMEOUT_SLAVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 // Armazena bytes restantes - Interrupcao RxTimeout
STORE_LEFTBYTES_SLAVE:
    SET LED_IDLE
 // Le Rx FIFO Level
	CS_DOWN
	SEND_SPI 0x12, 8
	RECEIVE_SPI  8
	CS_UP

	ZERO	&J,4
	MOV		J,BUFFER_SPI_IN				// J = Quantia de bytes no RxFIFO (MAX3107)

	CS_DOWN
	SEND_SPI 0x00,8						// Comando Read

STORE_LAST16_MEMORY_SLAVE:
	ADD		I,I,1					    // Proximo endereco da SHRAM
	RECEIVE_SPI 8						// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1			// Armazena no byte I da shram
	SUB		J,J,1

	QBNE	STORE_LAST16_MEMORY_SLAVE,J,0x00		// Se J == 0, termina loop
	CS_UP


 // ~~~~~ TAMANHO DOS DADOS ~~~~~~~~~~~~~~~~
	ZERO	&J,4
	ADD		J,OFFSET_SHRAM_WRITE,3			// J = 0x1803
	SUB		I, I, J					// I = I - J = tamanho

	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_WRITE, 4 		// Armazena tamanho nos primeiros bytes
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//READ_ISR
	//READ_LSR

	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x00, 8					// Desabilita interrupcoes
	CS_UP


// ************* VERIFICA MENSAGEM DE SINCRONISMO
    QBNE DATA_READY_SLAVE, I, 0x06

	// Configura pointer para SHRAM_WRITE
        ZERO    &I, 4
        ADD     I,OFFSET_SHRAM_WRITE, 4                           // I pointer to the start of valid data SHRAM[0x1804]

	// Dado[0] -- 0xFF
	ZERO	&J, 4
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0xff
	// Dado[1] -- 0x50
	ZERO	&J, 4
	ADD	I,I,1
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0x50
	// Dado[2] -- 0x00
	ZERO	&J, 4
	ADD	I,I,1
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0x00
	// Dado[3] -- 0x01
	ZERO	&J, 4
	ADD	I,I,1
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0x01
	// Dado[4] -- 0x0C
	ZERO	&J, 4
	ADD	I,I,1
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0x0c
	// Dado[5] -- 0xa4
	ZERO	&J, 4
	ADD	I,I,1
	LBCO	J, SHRAM_BASE, I, 1
	QBNE	DATA_READY_SLAVE, J, 0xa4

	// Mensagem integra - PISCA LED

UPDATE_MSG_COUNTING:
    ZERO    &I, 4
    LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_COUNT, 2   //Load current count
	ADD	I,I,1
	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_SYNC_COUNT,2	// Store count++
// *******************








// Procedimento concluido
 	JMP	DATA_READY_SLAVE
// ------------------------------------------------------------------------------------------------


// ----- ENVIO DE DADOS - MODO SLAVE --------------------------------------------------------------
 SEND_DATA_SLAVE:

 READ_ISR
 READ_LSR
 	CALL SEND_DATA_UART

 	MOV	I, MENSAGEM_ANTIGA
 	SBCO	I, SHRAM_BASE, 1, 1	 	// Dados já enviados e nao ha dado recebido no buffer

 // Sinaliza fim de envio
 	MOV 	r31.b0, PRU1_ARM_INTERRUPT+16

	JMP		START_SLAVE
 // ------------------------------------------------------------------------------------------------




// ----- FINALIZACAO - Finaliza e aguarda por novo envio de dados  --------------------------------
DATA_READY_SLAVE:


//	CLR	LED_READ


    MOV     I, MENSAGEM_RECEBIDA_NOVA       // Confirma Dados Recebidos prudata[1]=0x00
	SBCO	I, SHRAM_BASE, 1, 1
    MOV 	r31.b0, PRU0_ARM_INTERRUPT+16

	JMP		START_SLAVE
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::



// Never reached
END:
	HALT






// -----------------------------------------------------------------------
// --------------------- Transmissao UART - MAX3107 ----------------------
// -----------------------------------------------------------------------
SEND_DATA_UART:

	SET LED_WRITE


// ~~~~~ Armazena valor do endereco de retorno
	MOV		K, r29
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// ~~~~~ Configs ~~~~~~~~~~~~~~~~~~~~~~~~~~
	ZERO 	&BLOCKS, 4
	LBCO	BLOCKS, SHRAM_BASE, OFFSET_SHRAM_READ, 4		// Tamanho do bloco

	MOV	I,OFFSET_SHRAM_READ
	ADD	I,I,4							// I: ponteiro para início dos dados - SHRAM[100+4]

	ZERO	&A, 4
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Total de bytes = A*100 + B ~~~~~~~~
	MVID	B, BLOCKS
	QBGT	LAST_BYTES, BLOCKS, 0x64				// Jump se tamanho < 100

QTDD_BLOCOS:
		SUB		B, B, 0x64				// bytes finais -= 100
		ADD		A, A, 1
	QBLE	QTDD_BLOCOS, B, 0x63					// Repete loop se bytes finais > 100
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Enviar A blocos de 100 bytes ~~~~~~
LOOP_COUNT:
	ZERO &J, 4

	CS_DOWN
	SEND_SPI 0x80,8
	LOOP_COUNT_DATA:
		LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1		// Carrega bloco I da shram
		SEND_SPI BUFFER_SPI_OUT,8				// Envia bloco
		ADD I, I, 1
		ADD J, J, 1
		QBNE LOOP_COUNT_DATA, J, 0x64				// Encerra com J == 100

	CS_UP

	WAIT_BUFFER_TX:
		// Le Tx FIFO Level
		CS_DOWN
		SEND_SPI 0x11, 8
		RECEIVE_SPI  8
		CS_UP

		LSR BUFFER_SPI_IN, BUFFER_SPI_IN, 3
		QBNE	WAIT_BUFFER_TX, BUFFER_SPI_IN, 0		// Aguarda Buffer TX < 8

	SUB A, A, 1
	QBNE LOOP_COUNT, A, 0						// Finaliza envio de A blocos
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~ Enviar B bytes ~~~~~~~~~~~~~~~~~~~~
LAST_BYTES:


	QBEQ	FINALIZANDO, B, 0

	CS_DOWN
	SEND_SPI 0x80,8
	LOOP_COUNT_DATA_FINAL:
		LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1		// Carrega bloco I da shram
		SEND_SPI BUFFER_SPI_OUT,8				// Envia bloco
		ADD I, I, 1
		SUB B, B, 1
		QBNE LOOP_COUNT_DATA_FINAL, B, 0			// Encerra
	CS_UP

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


FINALIZANDO:
	READ_ISR
	READ_LSR
	READ_STS

WAIT_TX_EMPTY:
	// Le Tx FIFO Level
	CS_DOWN
	SEND_SPI 0x11, 8
	RECEIVE_SPI  8
	CS_UP

	QBNE	WAIT_TX_EMPTY, BUFFER_SPI_IN, 0			// Aguarda Buffer TX estar vazio


// ~~~~~ Devolve valor do endereco de retorno
	MOV		r29, K
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	CLR LED_WRITE


	RET
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------







// -----------------------------------------------------------------------
// --------------------- Transmissao SPI - Mode 0 ------------------------
// -----------------------------------------------------------------------
SPI_SEND:

	MOV WOFFSET,32
	SUB WOFFSET,WOFFSET,TAMANHO
	LSL BUFFER_SPI_OUT,BUFFER_SPI_OUT,WOFFSET


ENVIA:
	CLR	CLK	           	// clock DOWN
	QBBS	BITSET,BUFFER_SPI_OUT.t31		// Jump se bit 31 é 1. Se nao, continua pq eh zero

BITCLR:
	CLR	MOSI			// reseta dados
	QBA	PRONTO

BITSET:
	SET	MOSI			// set linha dados

PRONTO:
	LSL	BUFFER_SPI_OUT,BUFFER_SPI_OUT,1
	MOV	r5,0
	SET	CLK			// clock UP
	MOV	r5,0

	SUB	TAMANHO,TAMANHO,1

	QBNE	ENVIA,TAMANHO,0		// Volta pro loop se a palavra nao foi toda transmitida

	CLR	CLK


	RET
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------





// -----------------------------------------------------------------------
// ------------------ Recebimento SPI - Mode 0 ---------------------------
// -----------------------------------------------------------------------
SPI_RECEIVE:

	ZERO	&BUFFER_SPI_IN, 4		// Zera 4 bytes do registrador indicado

RECEBE:
	CLR	CLK	           	// clock DOWN
	NOP
	NOP
	NOP
	NOP
	NOP
	SUB	TAMANHO,TAMANHO,1

	SET	CLK			// clock UP
	NOP
	NOP

// . . . . . . . Amostrar sinal do MISO em SPI_IN_REG . . . . . . . .
	QBBC	INPUTZERO,MISO
	ADD		BUFFER_SPI_IN,BUFFER_SPI_IN,1
	QBEQ	FINAL,TAMANHO,0

INPUTZERO:
	QBEQ	FINAL,TAMANHO,0
	LSL	BUFFER_SPI_IN,BUFFER_SPI_IN,1
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

	QBNE	RECEBE,TAMANHO,0		// Volta pro loop se a palavra nao foi toda transmitida

FINAL:
	CLR	CLK

	RET
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
