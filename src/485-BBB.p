#define OWN_RAM              		0x000
#define OTHER_RAM           		0x020
#define SHARED_RAM          	 	0x100

#define OFFSET_SHRAM_READ			0x64		// 100 general purpose bytes
#define OFFSET_SHRAM_WRITE			r19			// 6kB (0x1800) offset to be set
#define OFFSET_SHRAM_SYNC			0x32		// 50 - Start of sync message

#define OFFSET_SEND_COMMAND			0x69


#define CLK							r30.t0		// P8.45
#define MISO						r31.t1		// P8.46
#define MOSI						r30.t4		// P8.41
#define CHIP_SELECT					r30.t2		// P8.43
#define IRQ							r31.t6		// P8.39
#define SYNC						r31.t8		// P8.27


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

 
#define UART_MODES_ADDRESS			0x89	// UART_WRITE_END
#define MODE_1						0x90	// MODE 1 ----
#define MODE_2						0x80	// MODE 2 ---- EchoSuppr
#define LCR							0x03	// LCR
#define RXTIMEOUT					0x02	// RxTimeOut -- 2 words
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




.setcallreg r29.w0					// PC saving register (CALL instructions)				
.origin 0							// start of program in PRU memory
.entrypoint START					// program entry point (for a debugger)

#include "485-BBB.hp"
#define AM33XX


START:	 
// ----- Initial Configuration --------------------------------------------------------------------

// OFFSET_SHRAM_WRITE
	ZERO	&OFFSET_SHRAM_WRITE, 4    
	MOV		OFFSET_SHRAM_WRITE, 0x18
	LSL		OFFSET_SHRAM_WRITE, OFFSET_SHRAM_WRITE, 8	// SHRAM_WRITE offset = 0x1800    
 
//  Shared Memory Config
	SHRAM_CONFIG

// Wait for UART to start
	WBS		IRQ  
		
	READ_ISR	
	
// ~~~~~ MAX3107 Configuration Registers ~~~~~

	CS_DOWN
	SEND_SPI UART_MODES_ADDRESS, 8	
	SEND_SPI MODE_1, 8	
	SEND_SPI MODE_2, 8	
	SEND_SPI LCR, 8
	SEND_SPI RXTIMEOUT, 8
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
	
//	CS_DOWN
//	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8	
//	SEND_SPI IRQEN, 8
//	CS_UP

	CS_DOWN
	SEND_SPI INTERRUPTS_LSREN_ADDRESS, 8	
	SEND_SPI LSREN, 8
	CS_UP

	// RevID Request and Response
	CS_DOWN
	SEND_SPI 0x1f, 8
	RECEIVE_SPI 8
	SBCO BUFFER_SPI_IN, SHRAM_BASE, OFFSET_SHRAM_WRITE, 1
	CS_UP

// ------------------------------------------------------------------------------------------------



PROCEDURE_START:

	READ_ISR
	READ_LSR	

// ~~~~~ Verifica modo de operacao ~~~~~~~~~
OPERATION_MODE:
	ZERO	&I, 4
	LBCO	I, SHRAM_BASE, 5, 1								// 0xFF : Modo Sincrono
	QBNE	WAIT_FOR_DATA, I, 0xff 							// 0x00 : Modo Normal
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	

	
	
// ----- PROCEDIMENTO SINCRONO --------------------------------------------------------------------	
// Wait for sync pulse: Apenas borda de subida
	WBS SYNC
//	WBC	SYNC
	
//	Envia comando de sincronismo
	ZERO 	&BLOCKS, 4
	LBCO	BLOCKS, SHRAM_BASE, OFFSET_SHRAM_SYNC, 1		// Tamanho do bloco
	
	MOV		I,OFFSET_SHRAM_SYNC
//	ADD		I,I,1											// I pointer to the start of valid data SHRAM[50+1]
	ADD 	BLOCKS,BLOCKS,I									// Blocks: address of the last byte
	
	CS_DOWN
	SEND_SPI 0x80,8											// Comando Envio

LOAD_FROM_MEMORY_SYNC:	
	ADD		I,I,1
	LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1				// Carrega bloco I da shram
	SEND_SPI BUFFER_SPI_OUT,8								// Envia bloco
									
	QBNE	LOAD_FROM_MEMORY_SYNC,I,BLOCKS					// Se I == NUMERO DE BLOCOS, termina loop
	 
	CS_UP
	
DELAY_CONFIG:
	ZERO	&I, 4
	ZERO	&TIMEOUT_VALUE, 4
	ADD		TIMEOUT_VALUE, TIMEOUT_VALUE, 0xe2
	LSL		TIMEOUT_VALUE, TIMEOUT_VALUE, 4						

LOOP_DELAY:	
	ADD		I,I,1											
	QBNE	LOOP_DELAY, I, TIMEOUT_VALUE					// Wait before verifying normal data
// ------------------------------------------------------------------------------------------------	
	
	
	
	
	
// ----- PROCEDIMENTO CONSTANTE -------------------------------------------------------------------
// Verifica se os dados já estão prontos para serem enviados: SHRAM[1] = 0xFF
WAIT_FOR_DATA:

	LBCO	I, SHRAM_BASE, 1, 1
	QBNE	OPERATION_MODE, I.b0, 0xff						// Se not ready, verifica condicao de sincronismo
	

// Reset FIFO UART
	RESET_FIFO_UART
	
	
	
	JMP SEND_DATA

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	

// ------------------------------------------------------------------------------------------------
 
 
 
 
 
 
 
 
 
 // ----- ENVIAR DADOS ----------------------------------------------------------------------------
 SEND_DATA:
 
 // ~~~~~ Configs ~~~~~~~~~~~~~~~~~~~~~~~~~~
	ZERO 	&BLOCKS, 4
	LBCO	BLOCKS, SHRAM_BASE, OFFSET_SHRAM_READ, 4		// Tamanho do bloco
	
	MOV		I,OFFSET_SHRAM_READ
	ADD		I,I,4											// I pointer to the start of valid data SHRAM[100+4]
		
	ZERO	&A, 4
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


		
		
// ~~~~~ Total de bytes = A*100 + B ~~~~~~~~
	MVID	B, BLOCKS
	QBGT	LAST_BYTES, BLOCKS, 0x64						// Jump se tamanho < 100

QTDD_BLOCOS:
		SUB		B, B, 0x64									// bytes finais -= 100
		ADD		A, A, 1
	QBLE	QTDD_BLOCOS, B, 0x63							// Repete loop se bytes finais > 100	
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	




// ~~~~~ Enviar A blocos de 100 bytes ~~~~~~			
LOOP_COUNT:
	ZERO &J, 4

	CS_DOWN
	SEND_SPI 0x80,8
	LOOP_COUNT_DATA:
		LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1			// Carrega bloco I da shram
		SEND_SPI BUFFER_SPI_OUT,8							// Envia bloco
		ADD I, I, 1
		ADD J, J, 1
		QBNE LOOP_COUNT_DATA, J, 0x64						// Encerra com J == 100
		
	CS_UP
	 
	WAIT_BUFFER_TX:
		// Le Tx FIFO Level
		CS_DOWN
		SEND_SPI 0x11, 8
		RECEIVE_SPI  8
		CS_UP

		LSR BUFFER_SPI_IN, BUFFER_SPI_IN, 3
		QBNE	WAIT_BUFFER_TX, BUFFER_SPI_IN, 0			// Aguarda Buffer TX < 8

	SUB A, A, 1
	QBNE LOOP_COUNT, A, 0									// Finaliza envio de A blocos
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	
	

// ~~~~~ Enviar B bytes ~~~~~~~~~~~~~~~~~~~~
LAST_BYTES:
	QBEQ	FINALIZANDO, B, 0
	
	CS_DOWN
	SEND_SPI 0x80,8
	LOOP_COUNT_DATA_FINAL:
		LBCO	BUFFER_SPI_OUT, SHRAM_BASE, I, 1			// Carrega bloco I da shram
		SEND_SPI BUFFER_SPI_OUT,8							// Envia bloco
		ADD I, I, 1
		SUB B, B, 1
		QBNE LOOP_COUNT_DATA_FINAL, B, 0					// Encerra
	CS_UP	
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	


FINALIZANDO:
	READ_ISR
	READ_LSR		
	READ_STS
 
 
 
 // Aguardar resposta 
 //	JMP	RECEIVE_DATA
 // -----------------------------------------------------------------------------------------------
 
 
 
 
 
 
 
 
 // ----- RECEBER DADOS ---------------------------------------------------------------------------
 RECEIVE_DATA:
 
 // ~~~~~ Response required? ~~~~~~~~~~~~~~~~
	LBCO	I, SHRAM_BASE, OFFSET_SEND_COMMAND, 1			// Verify if command == 0x50. Already loaded from memory on previous instruction
	QBNE	RECEIVE_DATA_OK, I.b0, 0x50		
	LBCO	I, SHRAM_BASE, 0x6c, 1							// Offset "ID" in an 0x50 instruction
	QBNE	RECEIVE_DATA_OK, I.b0, 0x00						// Reset ID = 0x00
	
	JMP		TIMEOUT_AND_NORESPONSE							// Reset function. No response required.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~				// Response required: continue on next block

 
 RECEIVE_DATA_OK:
 
 // ~~~~~ Int por RxFIFO nao-vazio ~~~~~~~~~
	CS_DOWN
	SEND_SPI 0x8a, 8										// Mode2 Address
	SEND_SPI 0x88, 8										// Mode2 = RxEmtyEn & EchoSuprs
	CS_UP

	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x40, 8										// RxEmtyEn
	CS_UP
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ~~~~~ Timeout Config~~~~~~~~~~~~~~~~~~~~~
	LBCO	TIMEOUT_VALUE, SHRAM_BASE, 6, 4
	ZERO	&I, 4
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ~~~~~ Response or timeout ? ~~~~~~~~~~~~~
WAIT_RECEIVED:

	QBEQ	TIMEOUT_AND_NORESPONSE, I, TIMEOUT_VALUE		// Timeout reached: jump
	ADD		I,I,1											// Increment timeout register

	QBBS	WAIT_RECEIVED, IRQ								// Data not received, loop continues
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	



	

// Habilita Interrupcao RxTimeout 
	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x01, 8										// Apenas LSRErrEn
	CS_UP
	
// Desabilita RxEmptyInv
	CS_DOWN
	SEND_SPI 0x8a, 8
	SEND_SPI 0x80, 8										// Apenas EchoSuprs
	CS_UP
	
	READ_ISR
	READ_LSR
		
// Configura pointer para SHRAM_WRITE
	ZERO	&I, 4
	ADD		I,OFFSET_SHRAM_WRITE, 3							// I pointer to the start of valid data SHRAM[0x1800 + 3]



// Le RxLevel até ser >= 64 e verifica IRQ (RxTimeout). Se Timeout_FimMensagem, ARMAZENA BYTES RESTANTES
RXLEVEL_AND_TIMEOUT:

	QBBC 	STORE_LEFTBYTES, IRQ							// RxTimeout Interrupt. End of message reached
	
	// Le Rx FIFO Level
	CS_DOWN
	SEND_SPI 0x12, 8 
	RECEIVE_SPI  8
	CS_UP

	LSR BUFFER_SPI_IN, BUFFER_SPI_IN, 6
	QBEQ	RXLEVEL_AND_TIMEOUT, BUFFER_SPI_IN, 0			// Aguarda Buffer TX >= 0x0100 0000 = 64

 
 // Armazena 64 bytes na SHRAM
STORE_64BYTES:
	ZERO	&J,4											// J: contador do loop
	CS_DOWN
	SEND_SPI 0x00,8											// Comando Read

STORE_64_MEMORY:	
	ADD		I,I,1											// Starting at SHRAM[0x1800 + 4]
	RECEIVE_SPI 8											// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1					// Armazena no byte I da shram
	ADD		J,J,1											
									
	QBNE	STORE_64_MEMORY,J,0x40							// Se J == 64, termina loop
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
	MOV		J,BUFFER_SPI_IN									// J = N.of words in RxFIFO
	
	CS_DOWN
	SEND_SPI 0x00,8											// Comando Read

STORE_LAST64_MEMORY:	
	ADD		I,I,1											// Starting at SHRAM[0x1800 + 4]
	RECEIVE_SPI 8											// Recebe byte bloco
	SBCO	BUFFER_SPI_IN, SHRAM_BASE, I, 1					// Armazena no byte I da shram
	SUB		J,J,1											
									
	QBNE	STORE_LAST64_MEMORY,J,0x00						// Se J == 0, termina loop
	CS_UP	
 
 
 
 // ~~~~~ TAMANHO DOS DADOS ~~~~~~~~~~~~~~~~
	ZERO	&J,4
	ADD		J,OFFSET_SHRAM_WRITE,3							// J = 0x1803
	SUB		I, I, J											// I = I - J = tamanho
	
	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_WRITE, 4 			// Armazena tamanho nos primeiros bytes
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	READ_ISR
	READ_LSR
	
 // Procedimento concluido
 	JMP	DATA_READY
 // ------------------------------------------------------------------------------------------------
 
 
 
 
 
 
 
 
 
 
 
// ----- TIMEOUT e NO RESPONSE - Tratamento de dados ----------------------------------------------
TIMEOUT_AND_NORESPONSE:
	
	ZERO	&I, 4
	SBCO	I, SHRAM_BASE, OFFSET_SHRAM_WRITE, 4			// Tamanho = 0x00
	JMP		DATA_READY										// Finaliza execucao do comando
// ------------------------------------------------------------------------------------------------
  
    


// ----- DONE - Wait for new data send request ----------------------------------------------------
DATA_READY:

	CS_DOWN
	SEND_SPI INTERRUPTS_IRQEN_ADDRESS, 8
	SEND_SPI 0x00, 8										// Desabilita interrupcoes
	CS_UP

	SBCO	0x00, SHRAM_BASE, 1, 1							// Apaga data ready e confirma dados ok
	
	

	MOV 	r31.b0, PRU1_ARM_INTERRUPT+16
	
	JMP		PROCEDURE_START
// ------------------------------------------------------------------------------------------------	


	
// Never reached	
END:
	HALT
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
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
