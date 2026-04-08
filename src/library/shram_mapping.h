/*
shram_mapping.h


--------------------------------------------------------------------------------
Shared RAM Memory Mapping for PRUserial485 application
--------------------------------------------------------------------------------
Interfaces with SERIALxxCON Hardware (v2-3)

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group | Electronic Instrumentation Group

Author: Patricia HENRIQUES NALLIN
Date: October/2024
*/



// ----- Shared RAM Mapping -----
/* OFFSET   |   Reserved
 * 0            MAX3107 IC version (0x1A)    
 * 1            RS485 status: dados para enviar (0xFF) / dados para ler (0x00) / dados antigos (0x55)
 * 2            MAX3107: Baudrate (BRGCONFIG)
 * 3            MAX3107: Baudrate (DIVLSB)
 * 4            MAX3107: Baudrate (DIVMSB)
 * 5            Sync Mode: START (0xFF) or STOP (0x00)
 * 6..9         RS485: Timeout for receiving a reply
 * 10..13       Sync Mode: Ponteiro para próximo ponto da curva a ser executado
 * 14           FeedForward Mode: FF Enable
 * 15..18       Sync Mode: Endereco absoluto do bloco alocado na memoria DDR (armazenamento de curvas)
 * 19
 * 20..23       Sync Mode: Tamanho total em bytes das quatro curvas
 * 24           Hardware: Board hardware address
 * 25           RS485: Master/Slave ('M'/'S')
 * 26..28       RS485: 1 Serial Byte length [ns]
 * 29..31       Sync Mode: Delay Sync-Normal command (x10ns)
 * 32           MAX3107: RXTIMEOUT
 * 33           FeedForward Mode: Mutex for memory data
 * 34           RS485 & FeedForward Mode: Mutex acquired for RS485 line
 * 35..36       FeedForward Mode: Current FF table pointer
 * 37           FeedForward Mode: Current FF table
 * 38..41       FeedForward Mode: Endereco absoluto do bloco alocado na memoria DDR (armazenamento de tabelas)
 * 42           FeedForward Mode: Number of tables/modes
 * 43           FeedForward Mode: ID Type
 * 44..47       FeedForward Mode: ID maximum movement range
 * 48           FeedForward Mode: PRU#2 Mutex request for RS485 line
 * 49           RS485: ARM Mutex request for RS485 line
 * 50           Sync Mode: Data size
 * 51..         Sync Mode: Data
 * 
 * 80..83       Sync Mode: Pulse counting
 * 84           Sync Mode: Sync Ok (0x00 not ok / 0xFF waiting for trigger)
 * 85           Sync Mode: Sync mode config - 0x51 (Single sequence & Intercalated messages), 0x5E (Single sequence & messages at End of curve)
 *                                            0xC1 (Continuous sequence & Intercalated messages), 0xCE (Continuous sequence & messages at End of curve)
 *                                            0x5B (Single Sequence, Single Broadcast Function command)
 * 
 * 
 * 100..103     Sending Data: Data size
 * 104..6143    Sending Data: Data
 * 
 * 6144..6147   Receiving Data: Data size
 * 6148..10999  Receiving Data: Data
 * 
 * 11000        FeedForward Mode: From CLP: Current mode
 * 11001..11004 FeedForward Mode: From CLP: Current gap/phase position
 * 11005..11499 FeedForward Mode: Reserved to FF Data from CLP
 * 11500..11999 FeedForward Mode: Reserved to FF Pre-Data from CLP
 * 
 * 12000        FeedForward Mode: Warning flags


*/

#define SHRAM_OFFSET_485_MODE               25
#define SHRAM_OFFSET_485_TIMEOUT            6
#define SHRAM_OFFSET_485_BYTE_LENGTH_NS     26
#define SHRAM_OFFSET_MAX3107_BRGCONFIG      2
#define SHRAM_OFFSET_MAX3107_DIVLSB         3
#define SHRAM_OFFSET_MAX3107_DIVMSB         4
#define SHRAM_OFFSET_MAX3107_RXTIMEOUT      32
#define SHRAM_OFFSET_DATA_STATUS            1


#define SHRAM_OFFSET_SYNC_STATUS              5
#define SHRAM_OFFSET_SYNC_POINTER           10
#define SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR    15
#define SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES 20
#define SHRAM_OFFSET_SYNC_DELAY             29
#define SHRAM_OFFSET_SYNC_DATA_SIZE         50
#define SHRAM_OFFSET_SYNC_PAYLOAD           51
#define SHRAM_OFFSET_SYNC_PULSE_COUNTING    80
#define SHRAM_OFFSET_SYNC_TRIGGER_WAITING   84
#define SHRAM_OFFSET_SYNC_MODE              85

#define SHRAM_OFFSET_WRITE                  100
#define SHRAM_OFFSET_READ                   0x1800


#define SHRAM_OFFSET_MUTEX_PRU2_ARM         34
#define SHRAM_OFFSET_MUTEX_PRU2_REQUEST     48
#define SHRAM_OFFSET_MUTEX_ARM_REQUEST      49



#define SHRAM_OFFSET_FF_ENABLED             14
#define SHRAM_OFFSET_FF_N_TABLES            42
#define SHRAM_OFFSET_FF_ID_TYPE             43
#define SHRAM_OFFSET_FF_MAX_RANGE           44
#define SHRAM_OFFSET_FF_POINTER             35
#define SHRAM_OFFSET_FF_TABLE               37
#define SHRAM_OFFSET_FF_TABLE_ABS_ADDR      38
#define SHRAM_OFFSET_FF_POSITION            11001
#define SHRAM_OFFSET_FF_FLAGS               12000