// ----- Kyma Encoder -----
// July 06 2020

#define OWN_RAM                         0x000
#define OTHER_RAM                       0x020
#define SHARED_RAM                      0x100

#define OFFSET_ENCODER_ENABLED               0x56                    // 86 - KymaEncoder status (0xff if enabled)
#define OFFSET_ENCODER_STEP                  0x57                    // 87 - KymaEncoder step
#define OFFSET_ENCODER_CURRENT_STATE         0x5B                    // 91 - KymaEncoder Current State

#define KYMA_A_INPUT                    r31.t15                 // P8.15
#define KYMA_B_INPUT                    r31.t14                 // P8.16

#define DDR_POINTER                     r12
#define DDR_BASE                        r9
#define ENCODER_STEP                         r16
#define I                               r25
#define OPERATION_MODE                  r15
#define ENCODER_MODE                         0xFF


.setcallreg r29.w0                                              // PC saving register (CALL instructions)
.origin 0                                                       // start of program in PRU memory
.entrypoint START                                               // program entry point (for a debugger)

#include "Kyma-Encoder.hp"
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
// :::::::::::::::::::::::::: KYMA ENCODER ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//        Quadrature Encoder State Machine
//
//       ---------        ++      ---------
//      | STATE 1 |------------->| STATE 2 |
//      |         |              |         |
//      |   A B   |       --     |   A B   |
//      |   0 0   |<-------------|   0 1   |
//       ---------                ---------
//        ^     |                  ^     |
//        |     |                  |     |
//     ++ |     | --            -- |     | ++
//        |     |                  |     |
//        |     V                  |     V
//       ---------        --      ---------
//      | STATE 3 |------------->| STATE 4 |
//      |         |              |         |
//      |   A B   |       ++     |   A B   |
//      |   1 0   |<-------------|   1 1   |
//       ---------                ---------
//

// ~~~~~ Verifica modo de operacao Master ~~
KYMA_START:
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  // 0x00 : Modo Normal (No encoder reading)
    QBEQ        KYMA_START, I, 0x00                  

// Verifica Estado Inicial
    ZERO        &I, 4
VERIFY_A: 
    QBBC        VERIFY_B, KYMA_A_INPUT
    ADD         I, I, 2
VERIFY_B:
    QBBC        VERIFY_INIT_STATE, KYMA_B_INPUT
    ADD         I, I, 1

VERIFY_INIT_STATE:
    QBEQ        STATE_1, I, 0x00
    QBEQ        STATE_2, I, 0x01
    QBEQ        STATE_3, I, 0x03
    QBEQ        STATE_4, I, 0x02
    JMP         KYMA_START
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ----------------------------------------------------------------------------
// STATE 1
// 00 -> 01 (+)     ||     00 -> 10 (-)
// ----------------------------------------------------------------------------
STATE_1:
    ZERO        &I, 4
    ADD         I, I, 1
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_CURRENT_STATE, 1       // Store current Machine State

STATE_1_WAIT:
    QBBS        STATE_1_POS, KYMA_B_INPUT
    QBBS        STATE_1_NEG, KYMA_A_INPUT
    JMP         STATE_1_WAIT

STATE_1_POS:
    CALL        STRAIGHT_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_2

STATE_1_NEG:
    CALL        FORWARD_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_4


// ----------------------------------------------------------------------------
// STATE 2
// 01 -> 11 (+)     ||     01 -> 00 (-)
// ----------------------------------------------------------------------------
STATE_2:
    ZERO        &I, 4
    ADD         I, I, 2
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_CURRENT_STATE, 1       // Store current Machine State

STATE_2_WAIT:
    QBBS        STATE_2_POS, KYMA_A_INPUT
    QBBC        STATE_2_NEG, KYMA_B_INPUT
    JMP         STATE_2_WAIT


STATE_2_POS:
    CALL        STRAIGHT_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_3

STATE_2_NEG:
    CALL        FORWARD_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_1


// ----------------------------------------------------------------------------
// STATE 3
// 11 -> 10 (+)     ||     11 -> 01 (-)
// ----------------------------------------------------------------------------
STATE_3:
    ZERO        &I, 4
    ADD         I, I, 3
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_CURRENT_STATE, 1       // Store current Machine State

STATE_3_WAIT:
    QBBC        STATE_3_POS, KYMA_B_INPUT
    QBBC        STATE_3_NEG, KYMA_A_INPUT
    JMP         STATE_3_WAIT

STATE_3_POS:
    CALL        STRAIGHT_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_4

STATE_3_NEG:
    CALL        FORWARD_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_2

 
// ----------------------------------------------------------------------------
// STATE 4
// 10 -> 00 (+)     ||     10 -> 11 (-)
// ----------------------------------------------------------------------------
STATE_4:
    ZERO        &I, 4
    ADD         I, I, 4
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_CURRENT_STATE, 1       // Store current Machine State

STATE_4_WAIT:
    QBBC        STATE_4_POS, KYMA_A_INPUT
    QBBS        STATE_4_NEG, KYMA_B_INPUT
    JMP         STATE_4_WAIT

STATE_4_POS:
    CALL        STRAIGHT_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_1

STATE_4_NEG:
    CALL        FORWARD_MOVE
    ZERO        &I, 4
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_ENABLED, 1  
    QBEQ        KYMA_START, I, 0x00                  // 0x00 : Modo Normal (No encoder reading)
    JMP         STATE_3


// .............................................................................
// SUBROUTINES TO TREAT STEPS
// .............................................................................
FORWARD_MOVE:
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_STEP,4        // Load current step
    SUB         I,I,1
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_STEP,4        // Store step++
    MOV         r31.b0, PRU0_ARM_INTERRUPT+16
    RET

STRAIGHT_MOVE:
    LBCO        I, SHRAM_BASE, OFFSET_ENCODER_STEP,4        // Load current step
    ADD         I,I,1
    SBCO        I, SHRAM_BASE, OFFSET_ENCODER_STEP,4        // Store step--
    MOV         r31.b0, PRU0_ARM_INTERRUPT+16
    RET