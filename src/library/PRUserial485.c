/*
PRUserial485.c


--------------------------------------------------------------------------------
RS-485 communication via PRU
--------------------------------------------------------------------------------
Interfaces with SERIALxxCON Hardware (v2-3)

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group | Electronic Instrumentation Group

Author: Patricia HENRIQUES NALLIN
Date: October/2024
*/

#include "PRUserial485.h"
#include "shram_mapping.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>


// --- PRU mapping
#define PRU_485_NUM                         1 // Primary PRU
#define PRU_485_BINARY                      "/usr/bin/PRUserial485.bin"


// --- ARM Registers
#define GPIO_DATAOUT                        0x13C
#define GPIO_DATAIN                         0x138
#define GPIO0_ADDR                          0x44E07000
#define GPIO1_ADDR                          0x4804C000
#define GPIO2_ADDR                          0x481AC000
#define GPIO3_ADDR                          0x481AF000
#define CM_PER_ADDR                         0x44E00000
#define CM_GPIO1                            0xAC
#define CM_GPIO2                            0xB0
#define CM_GPIO3                            0xB4


// --- Memory mapping
#define MMAP0_LOC                           "/sys/class/uio/uio0/maps/map0/"
#define MMAP1_LOC                           "/sys/class/uio/uio0/maps/map1/"
#define MAP_SIZE                            0x0FFFFFFF
#define MAP_MASK                            (MAP_SIZE)


// --- Parameters for RS485 flow control
#define INCOMING_BUFF_SIZE                  100000
#define OLD_DATA                            0x55
#define NEW_INCOMING_DATA                   0x00
#define NEW_SENDING_DATA                    0xff


// --- Mutex for RS485 (PRU1) sharing
#define MUTEX_485_FREE                      0
#define MUTEX_485_PRU2_ACQUIRED             1
#define MUTEX_485_ARM_ACQUIRED              2


// --- Curves for Sync Mode
#define CURVE_MAX_BLOCKS                    4
#define CURVE_BYTES_PER_BLOCK               100000
#define CURVE_TOTAL_RESERVED_BYTES          CURVE_MAX_BLOCKS*CURVE_BYTES_PER_BLOCK
#define CURVE_MAX_POINTS_PER_BLOCK          CURVE_BYTES_PER_BLOCK/16




// --- Shared RAM pointer
volatile uint8_t* prudata;


// --- Managing/threading RS485 incoming data in circular buffer
pthread_mutex_t lock;
volatile pthread_t tid = 0;
volatile uint8_t thread_control = 0;
volatile uint32_t pru_pointer = 0, read_pointer = 0;
volatile uint8_t receive_buffer[INCOMING_BUFF_SIZE];


// --- FeedForward config variables
volatile int bytes_per_table = 0, max_points_per_table = 0;


// --- General Purpose
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


int load_ddr(unsigned int ddr_offset, uint32_t table_points, float *curve1, float *curve2, float *curve3, float *curve4){
    // ----- DDR address
    unsigned int DDR_address[2];

    FILE* fp;
    fp = fopen(MMAP1_LOC "addr", "rt");
    fscanf(fp, "%x", &DDR_address[0]);
    fclose(fp);

    fp = fopen(MMAP1_LOC "size", "rt");
    fscanf(fp, "%x", &DDR_address[1]);
    fclose(fp);

    int fd;
    void *map_base, *virt_addr;
    off_t target = DDR_address[0];

    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1){
        return ERR_LD_CURVE_MOPEN;
    }

    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    if(map_base == (void *) -1) {
        return ERR_LD_CURVE_MMAP;
    }

    uint32_t value1, value2, value3, value4 = 0;
    float read1, read2, read3, read4 = 0;
    int i;

    target += ddr_offset;

    for(i=0; i<table_points; i++){

        read1 = curve1[i];
        read2 = curve2[i];
        read3 = curve3[i];
        read4 = curve4[i];

        value1 = (uint32_t)(*(uint32_t*)&read1);
        value2 = (uint32_t)(*(uint32_t*)&read2);
        value3 = (uint32_t)(*(uint32_t*)&read3);
        value4 = (uint32_t)(*(uint32_t*)&read4);

        // ----- CURVA 1
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value1 >> 0;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value1 >> 8;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value1 >> 16;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value1 >> 24;

        target++;


        // ----- CURVA 2
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value2 >> 0;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value2 >> 8;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value2 >> 16;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value2 >> 24;

        target++;


        // ----- CURVA 3
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value3 >> 0;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value3 >> 8;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value3 >> 16;

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value3 >> 24;

        target++;


        // ----- CURVA 4
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value4 >> 0;
        while((*((uint8_t *) virt_addr)) != ((value4 >> 0)&0xff)){
        }

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value4 >> 8;
        while((*((uint8_t *) virt_addr)) != ((value4 >> 8)&0xff)){
        }

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value4 >> 16;
        while((*((uint8_t *) virt_addr)) != ((value4 >> 16)&0xff)){
        }

        target++;
        virt_addr = map_base + (target & MAP_MASK);
        *((uint8_t *) virt_addr) = value4 >> 24;
        while((*((uint8_t *) virt_addr)) != ((value4 >> 24)&0xff)){
        }

        target++;
    }

    if(munmap(map_base, MAP_SIZE) == -1) {
        return ERR_LD_CURVE_UMMAP;
    }

    close(fd);
    return OK;
}


uint32_t read_ddr(unsigned int ddr_offset, uint32_t table_points, float *curve1, float *curve2, float *curve3, float *curve4){
    // ----- DDR address
    unsigned int DDR_address[2];

    FILE* fp;
    fp = fopen(MMAP1_LOC "addr", "rt");
    fscanf(fp, "%x", &DDR_address[0]);
    fclose(fp);

    int fd;
    void *map_base, *virt_addr;
    off_t target = DDR_address[0];

    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1){
        return ERR_LD_CURVE_MOPEN;
    }

    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    if(map_base == (void *) -1) {
        return ERR_LD_CURVE_MMAP;
    }

    uint32_t value, vals[4];
    int i,j;

    target += ddr_offset -1;

    for(i=0; i<table_points; i++){
    
        // ----- CURVA 1
        value = 0;
        for(j=0;j<4;j++){
            target++;
            virt_addr = map_base + (target & MAP_MASK);
            vals[j] = *((uint8_t *) virt_addr);
        }
        value = (vals[3]<<24) + (vals[2]<<16) + (vals[1]<<8) + (vals[0]);
        curve1[i] = *(float *)&value;

        // ----- CURVA 2
        value = 0;
        for(j=0;j<4;j++){
            target++;
            virt_addr = map_base + (target & MAP_MASK);
            vals[j] = *((uint8_t *) virt_addr);
        }
        value = (vals[3]<<24) + (vals[2]<<16) + (vals[1]<<8) + (vals[0]);
        curve2[i] = *(float *)&value;

        // ----- CURVA 3
        value = 0;
        for(j=0;j<4;j++){
            target++;
            virt_addr = map_base + (target & MAP_MASK);
            vals[j] = *((uint8_t *) virt_addr);
        }
        value = (vals[3]<<24) + (vals[2]<<16) + (vals[1]<<8) + (vals[0]);
        curve3[i] = *(float *)&value;

        // ----- CURVA 4
        value = 0;
        for(j=0;j<4;j++){
            target++;
            virt_addr = map_base + (target & MAP_MASK);
            vals[j] = *((uint8_t *) virt_addr);
        }
        value = (vals[3]<<24) + (vals[2]<<16) + (vals[1]<<8) + (vals[0]);
        curve4[i] = *(float *)&value;
    }

    if(munmap(map_base, MAP_SIZE) == -1) {
        return ERR_LD_CURVE_UMMAP;
    }

    close(fd);
    return table_points;
}


int clear_pulse_count_sync(){
    if(prudata[SHRAM_OFFSET_SYNC_STATUS]==0x00){     // Sync disabled. Clear pulse counting
        for(i=0; i<3; i++){
            prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+i] = 0;
        }
        return OK;
    }
    else{
        return ERR_CLEAR_PULSE;  // Error: not possible to clear pulse counting
    }// while sync operation is enabled.
}


uint32_t read_pulse_count_sync(){
    uint32_t counting = 0;
    counting =  (prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+3] << 24) + \
                (prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+2] << 16) + \
                (prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+1] << 8)  + \
                 prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING];
    return counting;
}


uint8_t read_shram(uint16_t offset){
    return prudata[offset];
}


void write_shram(uint16_t offset, uint8_t value){
    prudata[offset] = value;
}


void set_curve_pointer(uint32_t new_pointer){
    if(16*new_pointer >= ((prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+3]<<24) + \
                          (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+2]<<16) + \
                          (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+1]<<8)  + \
                           prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES])){
        return;
    }

    for(i=0; i<3; i++){
        prudata[SHRAM_OFFSET_SYNC_POINTER+i] = (16*new_pointer) >> i*8;
    }

    while (((prudata[SHRAM_OFFSET_SYNC_POINTER+3]<<24) + \
            (prudata[SHRAM_OFFSET_SYNC_POINTER+2]<<16) + \
            (prudata[SHRAM_OFFSET_SYNC_POINTER+1]<<8)  + \
             prudata[SHRAM_OFFSET_SYNC_POINTER]) != 16*new_pointer){
    }
}


uint32_t read_curve_pointer(){
    uint32_t pointer = 0;
    pointer = ((prudata[SHRAM_OFFSET_SYNC_POINTER+3] << 24) + \
               (prudata[SHRAM_OFFSET_SYNC_POINTER+2] << 16) + \
               (prudata[SHRAM_OFFSET_SYNC_POINTER+1] << 8)  + \
                prudata[SHRAM_OFFSET_SYNC_POINTER])/16;
    return pointer;
}


int sync_status(){
    // Sync trigger not waiting
    if(prudata[SHRAM_OFFSET_SYNC_TRIGGER_WAITING] == 0x00){
        return SYNC_OFF;
    }
    // Sync trigger waiting
    if(prudata[SHRAM_OFFSET_SYNC_TRIGGER_WAITING] == 0xff){
        return SYNC_ON;
    }
}


void set_sync_start_PRU(uint8_t sync_mode, uint32_t delay_us, uint8_t sync_address){
    if(prudata[SHRAM_OFFSET_485_MODE]=='M'){
        clear_pulse_count_sync();
        set_curve_pointer(0);

        uint32_t delay_ns = delay_us * 1000;

        // ----- Instrucao - Sync - Broadcast function
        if(sync_mode == 0x5B){
            prudata[SHRAM_OFFSET_SYNC_DATA_SIZE] = 0x06;       // Tamanho
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD]   = 0xff;       // | Endereco broadcast
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+1] = 0x50;       // |
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+2] = 0x00;       // | Comando a ser enviado
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+3] = 0x01;       // | apos pulso de sync
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+4] = 0x0f;       // |
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+5] = 0xa1;       // |
        }
        // ----- Instrucao - Sync - SetIx4
        else {
            prudata[SHRAM_OFFSET_SYNC_DATA_SIZE] = 0x16;          // Tamanho
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD]   = sync_address;  // | Endereco do controlador que respondera ao sync
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+1] = 0x50;          // |
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+2] = 0x00;          // | Comando a ser enviado
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+3] = 0x11;          // | apos pulso de sync
            prudata[SHRAM_OFFSET_SYNC_PAYLOAD+4] = 0x11;          // |
        }

        // Delay entre comando de sincronismo e requisicao qualquer
        // ----- Calculo do delay
        for(i=0; i<3; i++){
            delay_ns += prudata[SHRAM_OFFSET_485_BYTE_LENGTH_NS+i] << i*8;
        }

        // ---- Numero de loops = delay / 10 ns. Se delay = 0, apenas 1 loop.
        delay_ns = delay_ns/10;
        if (delay_us == 0){
            delay_ns = 1;
        }
        // ----- Armazena numero de instrucoes
        for(i=0; i<3; i++){
            prudata[SHRAM_OFFSET_SYNC_DELAY+i] = delay_ns >> i*8;
        }

        // Modo de Sincronismo
        // 0x51 - Single curve sequence & Intercalated read messages
        // 0x5E - Single curve sequence & Read messages at End of curve
        // 0xC1 - Continuous curve sequence & Intercalated read messages
        // 0xCE - Continuous curve sequence & Read messages at End of curve
        prudata[SHRAM_OFFSET_SYNC_MODE] = sync_mode;

        // Inicio modo sincrono
        prudata[SHRAM_OFFSET_SYNC_STATUS] = 0xff;

        // Aguarda início efetivo
        while(sync_status() == 0){
        }
    }
}


void set_sync_stop_PRU(){
    if(prudata[SHRAM_OFFSET_485_MODE]=='M'){
        prudata[SHRAM_OFFSET_SYNC_STATUS] = 0x00;
        prussdrv_exec_program(PRU_485_NUM, PRU_485_BINARY);
    }
}


void close_PRU(){
    // ----- Desabilita PRU e fecha mapeamento da shared RAM
    thread_control = 0;
    tid = 0;
    prussdrv_pru_disable(PRU_485_NUM);
    prussdrv_exit();
}


int loadCurve(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t CurvePoints, uint8_t block){

    if(block >= CURVE_MAX_BLOCKS){
        return ERR_CURVE_OVER_BLOCK;
    }
    if(CurvePoints > CURVE_MAX_POINTS_PER_BLOCK){
        return ERR_CURVE_OVER_POINTS;
    }

    load_ddr(block*CURVE_BYTES_PER_BLOCK, CurvePoints, curve1, curve2, curve3, curve4);

    
    if(((prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+3]<<24) + \
        (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+2]<<16) + \
        (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+1]<<8)  + \
         prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES]) != (16*CurvePoints)){

        // Tamanho em bytes
        for(i=0; i<3; i++){
            prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+i] = (16*CurvePoints) >> i*8;
        }
        
        while(((prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+3]<<24) + \
               (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+2]<<16) + \
               (prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES+1]<<8)  + \
                prudata[SHRAM_OFFSET_SYNC_TOTAL_CURVE_BYTES]) != (16*CurvePoints)){
        }
    }

    return OK;
}


void set_curve_block(uint8_t block){

    if(block >= CURVE_MAX_BLOCKS){
        return;
    }

    unsigned int DDR_address[2];

    FILE* fp;
    fp = fopen(MMAP1_LOC "addr", "rt");
    fscanf(fp, "%x", &DDR_address[0]);
    fclose(fp);

    DDR_address[0] += block * (CURVE_BYTES_PER_BLOCK);

    // Endereco do bloco na DDR
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR]   = (DDR_address[0]) >> 0;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+1] = (DDR_address[0]) >> 8;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+2] = (DDR_address[0]) >> 16;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+3] = (DDR_address[0]) >> 24;

}


uint8_t read_curve_block(){
    unsigned int DDR_address[2], address;
    uint8_t block = 0;

    FILE* fp;
    fp = fopen(MMAP1_LOC "addr", "rt");
    fscanf(fp, "%x", &DDR_address[0]);
    fclose(fp);

    // Endereco do bloco na DDR
    address = (prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+3] << 24) + \
              (prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+2] << 16) + \
              (prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+1] << 8)  + \
               prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR];
    block = (address - DDR_address[0])/CURVE_BYTES_PER_BLOCK;

    return block;
}


void *monitorRecvBuffer(void *arg){
    // ----- Copia dos dados recebidos
    uint32_t pru_recv_pointer = SHRAM_OFFSET_READ + 3;
    uint32_t os_recv_pointer = SHRAM_OFFSET_READ + 3;
    uint32_t tamanho;
    uint32_t BUFF_RECV_START = 0x1803;
    uint32_t BUFF_RECV_STOP = 0x2800;
    uint32_t idx;

    while(thread_control){
        // ----- Aguarda sinal de finalizacao do ciclo
        tamanho = 0;

        if(prudata[SHRAM_OFFSET_485_MODE] == 'M'){
            prussdrv_pru_wait_event(PRU_EVTOUT_0);
            prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
            // ----- Nova mensagem recebida !
            while(prudata[SHRAM_OFFSET_DATA_STATUS] != NEW_INCOMING_DATA){
            }

            // ----- Copia dos dados recebidos
            // Tamanho
            for(idx=0; idx<4; idx++){
                tamanho += prudata[SHRAM_OFFSET_READ+idx] << idx*8;
            }
            // Dados
            for(idx=0; idx<tamanho; idx++){
                receive_buffer[pru_pointer] = prudata[SHRAM_OFFSET_READ+4+idx];
                pru_pointer++;
                // Reset pru_pointer
                if(pru_pointer == INCOMING_BUFF_SIZE){
                    pru_pointer = 0;
                }
            }
            // ----- Sinaliza mensagem antiga
            prudata[SHRAM_OFFSET_DATA_STATUS] = OLD_DATA;
        }

        else{
            prussdrv_pru_wait_event(PRU_EVTOUT_0);
            prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

            // ----- Nova mensagem recebida !
            // Aguarda dado salvo na SHRAM
            while(prudata[SHRAM_OFFSET_DATA_STATUS] != NEW_INCOMING_DATA){
            }
            // ----- Sinaliza mensagem antiga
            prudata[SHRAM_OFFSET_DATA_STATUS] = OLD_DATA;

            // Le ponteiro da PRU
            pru_recv_pointer = 0;
            for(idx=0; idx<4; idx++){
                pru_recv_pointer += prudata[SHRAM_OFFSET_READ+idx] << idx*8;
            }

            while(pru_recv_pointer == os_recv_pointer){
                // Atualiza pru_recv_pointer
                pru_recv_pointer = 0;
                for(idx=0; idx<4; idx++){
                    pru_recv_pointer += prudata[SHRAM_OFFSET_READ+idx] << idx*8;
                }
            }

            // Ha DADOS
            if (pru_recv_pointer > os_recv_pointer){
                tamanho = pru_recv_pointer - os_recv_pointer;
            }
            else{
                tamanho = (BUFF_RECV_STOP - BUFF_RECV_START) - (os_recv_pointer - pru_recv_pointer);
            }



            pthread_mutex_lock(&lock);

            for(idx=0; idx<tamanho; idx++){
                os_recv_pointer++;
                receive_buffer[pru_pointer] = prudata[os_recv_pointer];
                pru_pointer++;
                // Reset pru_pointer
                if(pru_pointer == INCOMING_BUFF_SIZE){
                    pru_pointer = 0;
                }
                // Reset recv pointer
                if(os_recv_pointer == BUFF_RECV_STOP){
                    os_recv_pointer = BUFF_RECV_START;
                }
            }
            pthread_mutex_unlock(&lock);

        }
    }
}


int init_start_PRU(int baudrate, char mode){

    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    // ----- Inicializacao da PRU
    prussdrv_init();

    // ----- Inicializacao da interrupcao PRU
    if (prussdrv_open(PRU_EVTOUT_1)){
        return ERR_INIT_PRU_SSDRV;
    }
    if (prussdrv_open(PRU_EVTOUT_0)){
        return ERR_INIT_PRU_SSDRV;
    }
    prussdrv_pruintc_init(&pruss_intc_initdata);


    // ----- Mapeamento Shared RAM
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&prudata);


    // ----- SHRAM - General Purpose = 0x00
    for(i=0; i<100; i++)
    prudata[i] = 0x00;


    // ----- MODO DE OPERACAO: Master/Slave
    if(mode=='M' || mode=='S')
    prudata[SHRAM_OFFSET_485_MODE] = mode;
    else{
        // Modo nao existente
        close_PRU();
        return ERR_INIT_PRU_MODE;
    }

    // ----- Inicializacao: contador de sincronismo zerado
    prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING]   = 0;
    prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+1] = 0;
    prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+2] = 0;
    prudata[SHRAM_OFFSET_SYNC_PULSE_COUNTING+3] = 0;

    // ----- Inicializacao MASTER: procedimento sincrono desabilitado e RxTimeOut = 2 bytes
    if(prudata[SHRAM_OFFSET_485_MODE]=='M'){
        set_sync_stop_PRU();
        prudata[SHRAM_OFFSET_MAX3107_RXTIMEOUT] = 0x02;
    }

    // ----- Inicializacao SLAVE: nenhuma mensagem nova na serial e RxTimeOut = 18 bytes
    if(prudata[SHRAM_OFFSET_485_MODE]=='S'){
        prudata[SHRAM_OFFSET_DATA_STATUS]=OLD_DATA;
        prudata[SHRAM_OFFSET_MAX3107_RXTIMEOUT] = 0x02;
    }

    // Endereco de Hardware
    prudata[24] = hardware_address_serialPRU();

    // ----- Configuracao Baudrate
    uint32_t one_byte_length_ns = 0;
    switch (baudrate){
        case 1:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x20;
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x0f;
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 10000/1;
        break;

        case 6:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x28; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x02; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 10000/6;
        break;

        case 10:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x28; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x01; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 10000/10;
        break;

        case 12:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x24; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x01; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 10000/12;
        break;

        case 9600:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x0a;   
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x86;   
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x01;   
        one_byte_length_ns = (int) 100000000/96;
        break;

        case 14400:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x07; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x04; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x01; 
        one_byte_length_ns = (int) 100000000/144;
        break;

        case 19200:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x05; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0xc3; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 100000000/192;
        break;

        case 38400:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x15; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0xc3; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 100000000/384;
        break;

        case 57600:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x27; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x04; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x01; 
        one_byte_length_ns = (int) 100000000/576;
        break;

        case 115200:
        prudata[SHRAM_OFFSET_MAX3107_BRGCONFIG] = 0x09; 
        prudata[SHRAM_OFFSET_MAX3107_DIVLSB]    = 0x20; 
        prudata[SHRAM_OFFSET_MAX3107_DIVMSB]    = 0x00; 
        one_byte_length_ns = (int) 100000000/1152;
        break;

        default:
        close_PRU();    // BR nao definido
        return ERR_INIT_PRU_BAUDR;
    }

    prudata[SHRAM_OFFSET_485_BYTE_LENGTH_NS] = one_byte_length_ns;
    prudata[SHRAM_OFFSET_485_BYTE_LENGTH_NS+1] = one_byte_length_ns >> 8;
    prudata[SHRAM_OFFSET_485_BYTE_LENGTH_NS+2] = one_byte_length_ns >>16;
    prudata[SHRAM_OFFSET_485_BYTE_LENGTH_NS+3] = one_byte_length_ns >>24;

    // ----- DDR address
    unsigned int DDR_address[2];

    FILE* fp;
    fp = fopen(MMAP1_LOC "addr", "rt");
    fscanf(fp, "%x", &DDR_address[0]);
    fclose(fp);

    fp = fopen(MMAP1_LOC "size", "rt");
    fscanf(fp, "%x", &DDR_address[1]);
    fclose(fp);

    // Endereco da DDR - Sync via PRUserial485
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR]   = (DDR_address[0]) >> 0;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+1] = (DDR_address[0]) >> 8;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+2] = (DDR_address[0]) >> 16;
    prudata[SHRAM_OFFSET_SYNC_BLOCK_ABS_ADDR+3] = (DDR_address[0]) >> 24;

    // Endereco da DDR - FeedForward operations
    prudata[SHRAM_OFFSET_FF_TABLE_ABS_ADDR]   = (DDR_address[0]) >> 0;
    prudata[SHRAM_OFFSET_FF_TABLE_ABS_ADDR+1] = (DDR_address[0]) >> 8;
    prudata[SHRAM_OFFSET_FF_TABLE_ABS_ADDR+2] = (DDR_address[0]) >> 16;
    prudata[SHRAM_OFFSET_FF_TABLE_ABS_ADDR+3] = (DDR_address[0]) >> 24;


    // ----- Executar codigo na PRU
    prussdrv_exec_program (PRU_485_NUM, PRU_485_BINARY);

    // ----- Lanca thread para monitorar recebimento de dados
    //        e reinicializa ponteiros
    pru_pointer = 0;
    read_pointer = 0;
    if(tid == 0){
        pthread_attr_t attr;
        struct sched_param param;

        pthread_attr_init (&attr);
        pthread_attr_getschedparam (&attr, &param);
        (param.sched_priority)++;
        pthread_attr_setschedparam (&attr, &param);

        thread_control = 1;
        pthread_t thread_id;
        pthread_create(&thread_id, &attr, monitorRecvBuffer, NULL);
        tid = thread_id;
    }
    return OK;
}


int send_data_PRU(uint8_t *data, uint32_t *tamanho, float timeout_ms){

    uint32_t timeout_instructions;
    float timeout_inst;

    timeout_inst = timeout_ms*66600;
    timeout_instructions = (int)timeout_inst;


    prudata[SHRAM_OFFSET_MUTEX_ARM_REQUEST] = 1;
    while(prudata[SHRAM_OFFSET_MUTEX_PRU2_ARM] != MUTEX_485_ARM_ACQUIRED){
    }
    


    // ----- MASTER: Configuracao do Timeout
    if(prudata[SHRAM_OFFSET_485_MODE]=='M'){
        prudata[SHRAM_OFFSET_485_TIMEOUT]   = timeout_instructions;
        prudata[SHRAM_OFFSET_485_TIMEOUT+1] = timeout_instructions >> 8;
        prudata[SHRAM_OFFSET_485_TIMEOUT+2] = timeout_instructions >> 16;
        prudata[SHRAM_OFFSET_485_TIMEOUT+3] = timeout_instructions >> 24;
    }

    // ----- Tamanho dos dados
    prudata[SHRAM_OFFSET_WRITE]   = *tamanho;
    prudata[SHRAM_OFFSET_WRITE+1] = *tamanho >> 8;
    prudata[SHRAM_OFFSET_WRITE+2] = *tamanho >> 16;
    prudata[SHRAM_OFFSET_WRITE+3] = *tamanho >> 24;


    // ----- Insere na memoria - Dados a enviar
    for(i=0; i<*tamanho; i++){
        prudata[SHRAM_OFFSET_WRITE+4+i] = data[i];
    }

    // ----- Dados prontos para envio
    prudata[SHRAM_OFFSET_DATA_STATUS] = NEW_SENDING_DATA;

    // ----- Aguarda sinal de finalizacao do ciclo
    prussdrv_pru_wait_event(PRU_EVTOUT_1);
    prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);


    // Aguarda dados prontos na Shared RAM (M) ou fim do envio (S)
    if(prudata[SHRAM_OFFSET_485_MODE] == 'M'){
        while(prudata[SHRAM_OFFSET_DATA_STATUS] != OLD_DATA); 
    }

    // ----- SLAVE: Aguarda fim de envio
    if(prudata[SHRAM_OFFSET_485_MODE]=='S'){
        while(prudata[SHRAM_OFFSET_DATA_STATUS] != OLD_DATA);
    }

    while(prudata[SHRAM_OFFSET_MUTEX_ARM_REQUEST] != MUTEX_485_FREE){
    }

    return OK;
}


int recv_data_PRU(uint8_t *data, uint32_t *tamanho_recv, uint32_t bytes2read){
    uint32_t index;
    pthread_mutex_lock(&lock);
    if(pru_pointer == read_pointer){
        *tamanho_recv = 0;
        pthread_mutex_unlock(&lock);
        return OK;
    }

    if (pru_pointer > read_pointer){
        *tamanho_recv = pru_pointer - read_pointer;
    }
    else{
        *tamanho_recv = INCOMING_BUFF_SIZE - (read_pointer - pru_pointer);
    }

    if((bytes2read != 0) & (*tamanho_recv > bytes2read)){
        *tamanho_recv = bytes2read;
    }

    for(index=0; index<*tamanho_recv; index++){
        data[index] = receive_buffer[read_pointer];
        read_pointer++;
        // Reset pointer
        if(read_pointer == INCOMING_BUFF_SIZE){
            read_pointer = 0;
        }
    }
    pthread_mutex_unlock(&lock);
    return OK;
}


int recv_flush(){
    pthread_mutex_lock(&lock);
    if(pru_pointer != read_pointer){
        read_pointer = pru_pointer;
    }
    pthread_mutex_unlock(&lock);
    return OK;
}
