// ----- Kyma Feed-Forward -----
// June 23 2020

#define OWN_RAM                         0x000
#define OTHER_RAM                       0x020
#define SHARED_RAM                      0x100

#define OFFSET_SHRAM_SYNC               0x32                    // 50 - Start of sync message
#define OFFSET_SHRAM_SYNC_COUNT         0x50                    // 80 - Start of sync pulse counting
#define OFFSET_SHRAM_SYNC_DELAY         29                      // Time between sync and normal command
#define OFFSET_SHRAM_SYNC_OK            0x54                    // 84 - Sync OK
#define OFFSET_SHRAM_SYNC_MODE          0x55                    // 85 - Sync Mode
#define OFFSET_FF_ENABLED               0x56                    // 86 - Feedforward status (0xff if enabled)
#define OFFSET_FF_STEP                  0x57                    // 87 - FeedForward step




#define KYMA_PRIMARY_INPUT              r31.t15                 // P8.15
#define KYMA_SECONDARY_INPUT            r31.t14                 // P8.16



#define DDR_POINTER                     r12
#define DDR_BASE                        r9
#define CURVE                           r11
#define CURVE_SIZE                      r8
#define FF_STEP                         r16
#define B                               r17
#define K                               r18
#define I                               r25
#define J                               r28
#define OPERATION_MODE                  r15

#define ENABLED_SYNC                    0xFF
#define DISABLED_SYNC                   0x00

#define FF_MODE                         0xFF



.setcallreg r29.w0                                              // PC saving register (CALL instructions)
.origin 0                                                       // start of program in PRU memory
.entrypoint START                                               // program entry point (for a debugger)

#include "Kyma-FF.hp"
#define AM33XX


START:

// ----- Initial Configuration --------------------------------------------------------------------
    NOP

//  Shared Memory Config
    SHRAM_CONFIG

// ~~~~~ MASTER/SLAVE MODE ~~~~~~~~~~~~~~~~~
    ZERO        &OPERATION_MODE, 4                              // Verifica se opera em modo Master ou Slave
    LBCO        OPERATION_MODE, SHRAM_BASE, 25, 1               // OPERATION_MODE <- shram[25] ('M' ou 'S')

    QBEQ        KYMA_START, OPERATION_MODE, 0x4d                // 0x4D = 'M'
    HALT                                                        // 0x53 = 'S'
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::: KYMA FEED-FORWARD :::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


// ~~~~~ Verifica modo de operacao Master ~~
KYMA_START:
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_FF_ENABLED, 1  // 0xFF : Modo FeedForward
    QBNE        KYMA_START, I, FF_MODE               // 0x00 : Modo Normal
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// Wait Trigger from primary input (from Kyma Encoder)
    WBC         KYMA_PRIMARY_INPUT
    WBS         KYMA_PRIMARY_INPUT

// Got Triggered! Check Secondary Input
    QBBS        FORWARD_MOVE, KYMA_SECONDARY_INPUT // If 1, straight movement
    QBBC        STRAIGHT_MOVE, KYMA_SECONDARY_INPUT  // If 0, forward movement


FORWARD_MOVE:
    LBCO        I, SHRAM_BASE, OFFSET_FF_STEP,4        // Load current step
    SUB         I,I,1
    SBCO        I, SHRAM_BASE, OFFSET_FF_STEP,4        // Store step++
    MOV         r31.b0, PRU0_ARM_INTERRUPT+16
    JMP         KYMA_START


STRAIGHT_MOVE:
    LBCO        I, SHRAM_BASE, OFFSET_FF_STEP,4        // Load current step
    ADD         I,I,1
    SBCO        I, SHRAM_BASE, OFFSET_FF_STEP,4        // Store step--
    MOV         r31.b0, PRU0_ARM_INTERRUPT+16
    JMP         KYMA_START
