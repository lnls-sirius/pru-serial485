// ----- PRUserial485 - Timing - PRU0-----

#define OWN_RAM              		0x000
#define OTHER_RAM           		0x020
#define SHARED_RAM          	 	0x100

#define OFFSET_SHRAM_OPMODE         0x05
#define OFFSET_SHRAM_DDR_POINTER         0x0a        // 10..13 - DDR POINTER
#define OFFSET_SHRAM_SYNC			0x32		// 50 - Start of sync message
#define OFFSET_SHRAM_SYNC_COUNT			0x50		// 80 - Start of sync pulse counting
#define OFFSET_SHRAM_SYNC_DELAY		29		    // Time between sync and normal command
#define OFFSET_SHRAM_SYNC_OK			0x54		// 84 - Sync OK
#define OFFSET_SHRAM_SYNC_MODE			0x55		// 85 - Sync Mode
#define OFFSET_SHRAM_PS_TYPE			0x56		// 86 - PS type (FBP or FAC)
#define OFFSET_SHRAM_SYNC_TIMEOUT       0x57        // 87 - Enable sync timeout
#define OFFSET_SHRAM_SYNC_RECEIVED      0x58        // 88 - Sync Pulse Received


#define A							r16
#define B							r17
#define	K							r18
#define I							r25
#define	TIMEOUT_VALUE				r27
#define	J							r28
#define OPERATION_MODE				r15
#define ENDERECO_HARDWARE			r14
#define SYNC_SINGLE               0x50
#define SYNC_CONTINUOUS           0xC0
#define ENABLED_SYNC               0xFF
#define DISABLED_SYNC              0x00
#define ENABLED_SYNC_TIMEOUT       0xFF
#define DISABLED_SYNC_TIMEOUT      0x00
#define SYNC_PULSE_RECEIVED        0xFF
#define SYNC_PULSE_NOT_RECEIVED     0x00

#define PIN r30.t2




.setcallreg r29.w0					// PC saving register (CALL instructions)
.origin 0							// start of program in PRU memory
.entrypoint START					// program entry point (for a debugger)

#include "timing.hp"
#define AM33XX


START:

    CLR PIN

// ----- Initial Configuration --------------------------------------------------------------------
//  Shared Memory Config
	SHRAM_CONFIG
// ------------------------------------------------------------------------------------------------


// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::: SYNC TIMEOUT ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


// Verify if sync mode is set
CHECK_OPERATION_MODE:
	ZERO	&I, 4
	LBCO	I, SHRAM_BASE, OFFSET_SHRAM_OPMODE, 1			// 0xFF : Modo Sincrono
	QBNE	CHECK_OPERATION_MODE, I, ENABLED_SYNC			// 0x00 : Modo Normal


// Verify if sync mode is single or continuous
CHECK_SYNC_CONTINUOUS_MODE:
    ZERO    &I, 4
    LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_MODE, 1
    AND		I, I, 0xF0
    XOR     I, I, SYNC_CONTINUOUS
    QBNE    CHECK_OPERATION_MODE, I, 0x00  // If i !=0, single sequence - no timeout needed


CHECK_TIMEOUT_ENABLED:
	ZERO	&I, 4
	LBCO	I, SHRAM_BASE, OFFSET_SHRAM_SYNC_TIMEOUT, 1		// 0xFF : Timeout
	QBNE	CHECK_OPERATION_MODE, I, ENABLED_SYNC_TIMEOUT	// 0x00 : No timeout


// ----- PROCEDIMENTO SINCRONO - COM TIMEOUT -------------------------------

// Aguarda recebimento de um pulso de Sincronismo
WAIT_SYNC:
	ZERO    &I, 4
    LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_RECEIVED, 1
    QBEQ    WAIT_SYNC, I, SYNC_PULSE_NOT_RECEIVED


// Pulse received - Clear register
    ZERO    &I, 4
    SBCO    I.b0, SHRAM_BASE, OFFSET_SHRAM_SYNC_RECEIVED, 1




// Wait delay
DELAY_CONFIG:
CLR PIN

	ZERO	&I, 4
	ZERO	&TIMEOUT_VALUE, 4
	ADD    TIMEOUT_VALUE, TIMEOUT_VALUE, 0x01
    LSL     TIMEOUT_VALUE, TIMEOUT_VALUE, 8
    ADD    TIMEOUT_VALUE, TIMEOUT_VALUE, 0x86
    LSL     TIMEOUT_VALUE, TIMEOUT_VALUE, 8
    ADD    TIMEOUT_VALUE, TIMEOUT_VALUE, 0xa0



LOOP_DELAY:

	ADD	I,I,1
	QBNE	LOOP_DELAY, I, TIMEOUT_VALUE


// Verify if another pulse was Received
    ZERO    &I, 4
    LBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_RECEIVED, 1
    QBEQ    PULSE_NOT_RECEIVED, I, SYNC_PULSE_NOT_RECEIVED


// Pulse received:
PULSE_RECEIVED:
    // Pulse received - Clear register
    ZERO    &I, 4
    SBCO    I, SHRAM_BASE, OFFSET_SHRAM_SYNC_RECEIVED, 1

    // Verify if sync mode is still set - If so, wait new delay. If not, wait sync mode
	ZERO	&I, 4
	LBCO	I, SHRAM_BASE, OFFSET_SHRAM_OPMODE, 1			// 0xFF : Modo Sincrono
	QBNE	CHECK_OPERATION_MODE, I, ENABLED_SYNC			// 0x00 : Modo Normal
    JMP DELAY_CONFIG


// Pulse not received:
PULSE_NOT_RECEIVED:
    SET PIN
    // Clear curve pointer - Next point of curve will be the first one
    ZERO    &I, 4
    SBCO    I, SHRAM_BASE, OFFSET_SHRAM_DDR_POINTER, 4
    JMP     CHECK_OPERATION_MODE



// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
