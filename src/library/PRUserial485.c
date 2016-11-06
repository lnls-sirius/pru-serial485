/*
PRUserial485.c


--------------------------------------------------------------------------------
RS-485 communication via PRU
--------------------------------------------------------------------------------
Interfaces with SERIALxxCON Hardware (v2-3)

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group

Author: Patricia HENRIQUES NALLIN
Date: April/2018
*/

#include "PRUserial485.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>



#define PRU_NUM             1
#define PRU_BINARY          "/usr/bin/PRUserial485.bin"
#define OFFSET_SHRAM_WRITE  0x64    // 100 general purpose bytes
#define OFFSET_SHRAM_READ   0x1800  //

#define MENSAGEM_ANTIGA         0x55
#define MENSAGEM_RECEBIDA_NOVA  0x00
#define MENSAGEM_PARA_ENVIAR    0xff

#define GPIO_DATAOUT 0x13C
#define GPIO_DATAIN  0x138
#define GPIO0_ADDR   0x44E07000
#define GPIO1_ADDR   0x4804C000
#define GPIO2_ADDR   0x481AC000
#define GPIO3_ADDR   0x481AF000
#define CM_PER_ADDR  0x44E00000
#define CM_GPIO1     0xAC
#define CM_GPIO2     0xB0
#define CM_GPIO3     0xB4

#define MMAP0_LOC   "/sys/class/uio/uio0/maps/map0/"
#define MMAP1_LOC   "/sys/class/uio/uio0/maps/map1/"

#define CURVE_BYTES_PER_BLOCK      100000
#define CURVE_MAX_POINTS_PER_BLOCK CURVE_BYTES_PER_BLOCK/16

#define MAP_SIZE 0x0FFFFFFF
#define MAP_MASK (MAP_SIZE)



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
 * prudata[10..13] = Ponteiro para próximo ponto da curva a ser executado
 * prudata[15..18] = Endereco absoluto do bloco alocado na memoria DDR (armazenamento de curvas)
 * prudata[20..23] = Tamanho total em bytes das quatro curvas
 *
 * prudata[24] = Board hardware address
 * prudata[25] = Master/Slave ('M'/'S')
 * prudata[26..28] = 1 Serial Byte length (ns)
 * prudata[29..31] = Delay Sync-Normal command (x10ns)
 *
 * prudata[32] = MAX3107 RXTIMEOUT
 *
 * SHRAM[50]~SHRAM[99] - Sync Operation
 * prudata[50] = data size
 * prudata[51..] = data
 * prudata[80..83] = Pulse counting
 * prudata[84] = Sync_Ok (0x00 not ok / 0xFF waiting for trigger)
 * prudata[85] = Sync_Mode
          | 0x51 - Single curve sequence & Intercalated read messages
 *        | 0x5E - Single curve sequence & Read messages at End of curve
 *        | 0xC1 - Continuous curve sequence & Intercalated read messages
 *        | 0xCE - Continuous curve sequence & Read messages at End of curve
 *        | 0x5B - Single Sequence - Single Broadcast Function command
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


 #define BUFF_SIZE 10000

 volatile uint8_t* prudata;
 volatile uint8_t receive_buffer[BUFF_SIZE];
 volatile uint16_t pru_pointer = 0, read_pointer = 0;
 volatile pthread_t tid = 0;
 volatile uint8_t thread_control = 0;
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
  if(prudata[5]==0x00){     // Sync disabled. Clear pulse counting
    prudata[80] = 0;
    prudata[81] = 0;
    prudata[82] = 0;
    prudata[83] = 0;
    return OK;
  }
  else
    return ERR_CLEAR_PULSE;  // Error: not possible to clear pulse counting
                             // while sync operation is enabled.
}



uint32_t read_pulse_count_sync(){
  uint32_t counting = 0;
  counting = (prudata[83] << 24) + (prudata[82] << 16) + (prudata[81] << 8) + prudata[80];
  return counting;
}



void set_curve_pointer(uint32_t new_pointer){
  prudata[10] = (16*new_pointer);        // LSByte do new_pointer [7..0]
  prudata[11] = (16*new_pointer) >> 8;   // Byte do new_pointer [15..8]
  prudata[12] = (16*new_pointer) >> 16;  // Byte do new_pointer [23..16]
  prudata[13] = (16*new_pointer) >> 24;  // MSByte do new_pointer [31..24]

  while (((prudata[13]<<24) + (prudata[12]<<16) + (prudata[11]<<8) + prudata[10]) != 16*new_pointer){
  }
}



uint32_t read_curve_pointer(){
  uint32_t pointer = 0;
  pointer = ((prudata[13] << 24) + (prudata[12] << 16) + (prudata[11] << 8) + prudata[10])/16;
  return pointer;
}



int sync_status(){
  // Sync trigger not waiting
  if(prudata[84] == 0x00){
    return SYNC_OFF;
  }
  // Sync trigger waiting
  if(prudata[84] == 0xff){
    return SYNC_ON;
  }
}



void set_sync_start_PRU(uint8_t sync_mode, uint32_t delay_us, uint8_t sync_address){
  if(prudata[25]=='M'){
    clear_pulse_count_sync();
    set_curve_pointer(0);

    uint32_t delay_ns = delay_us * 1000;


    // ----- Instrucao - Sync - Broadcast function
    if(sync_mode == 0x5B){
      prudata[50] = 0x06;       // Tamanho
      prudata[51] = 0xff;       // | Endereco broadcast
      prudata[52] = 0x50;       // |
      prudata[53] = 0x00;       // | Comando a ser enviado
      prudata[54] = 0x01;       // | apos pulso de sync
      prudata[55] = 0x0f;       // |
      prudata[56] = 0xa1;       // |
    }
    // ----- Instrucao - Sync - SetIx4
    else {
      prudata[50] = 0x16;          // Tamanho
      prudata[51] = sync_address;  // | Endereco do controlador que respondera ao sync
      prudata[52] = 0x50;          // |
      prudata[53] = 0x00;          // | Comando a ser enviado
      prudata[54] = 0x11;          // | apos pulso de sync
      prudata[55] = 0x11;          // |
    }


    // Delay entre comando de sincronismo e requisicao qualquer
    // ----- Calculo do delay
    for(i=0; i<3; i++)
      delay_ns += prudata[26+i] << i*8;

    // ---- Numero de loops = delay / 10 ns. Se delay = 0, apenas 1 loop.
    delay_ns = delay_ns/10;
    if (delay_us == 0)
      delay_ns = 1;

    // ----- Armazena numero de instrucoes
    for(i=0; i<3; i++)
      prudata[29+i] = delay_ns >> i*8;

    // Modo de Sincronismo
    // 0x51 - Single curve sequence & Intercalated read messages
    // 0x5E - Single curve sequence & Read messages at End of curve
    // 0xC1 - Continuous curve sequence & Intercalated read messages
    // 0xCE - Continuous curve sequence & Read messages at End of curve
    prudata[85] = sync_mode;

    // Inicio modo sincrono
    prudata[5] = 0xff;

    // Aguarda início efetivo
    while(sync_status() == 0);

  }  // if(prudata[25]=='M'){

}



void set_sync_stop_PRU(){
  if(prudata[25]=='M')
    prudata[5] = 0x00;
    prussdrv_exec_program(PRU_NUM, PRU_BINARY);
}



void close_PRU(){
  // ----- Desabilita PRU e fecha mapeamento da shared RAM
  thread_control = 0;
  pthread_join(tid,NULL);
  tid = 0;
  prussdrv_pru_disable(PRU_NUM);
  prussdrv_exit();
}



int loadCurve(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t CurvePoints, uint8_t block){

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
    // printf("Failed to open memory!");
    return ERR_LD_CURVE_MOPEN;
  }

  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
  if(map_base == (void *) -1) {
    // printf("Failed to map base address");
    return ERR_LD_CURVE_MMAP;
  }

  uint32_t value1, value2, value3, value4 = 0;
  float read1, read2, read3, read4 = 0;
  int i;

  target += block * (CURVE_BYTES_PER_BLOCK);

  for(i=0; i<CurvePoints; i++){

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

  if (((prudata[23]<<24) + (prudata[22]<<16) + (prudata[21]<<8) + prudata[20]) != (16*CurvePoints)){
    // Tamanho em bytes
    prudata[20] = (16*CurvePoints);         // LSByte do tamanho curva [7..0]
    prudata[21] = (16*CurvePoints) >> 8;    // Byte do tamanho curva [15..8]
    prudata[22] = (16*CurvePoints) >> 16;   // Byte do tamanho curva [23..16]
    prudata[23] = (16*CurvePoints) >> 24;   // MSByte do tamanho curva [31..24]

    while(((prudata[23]<<24) + (prudata[22]<<16) + (prudata[21]<<8) + prudata[20]) != (16*CurvePoints)){
    }
  }

    if(munmap(map_base, MAP_SIZE) == -1) {
      // printf("Failed to unmap memory");
      return ERR_LD_CURVE_UMMAP;
    }

  close(fd);

  // printf("%d-point curve successfully loaded.\n", CurvePoints);

  return OK;
}



void set_curve_block(uint8_t block){
  unsigned int DDR_address[2];

  FILE* fp;
  fp = fopen(MMAP1_LOC "addr", "rt");
  fscanf(fp, "%x", &DDR_address[0]);
  fclose(fp);

  DDR_address[0] += block * (CURVE_BYTES_PER_BLOCK);

  // Endereco do bloco na DDR
  prudata[15] = (DDR_address[0]) >> 0;    // LSByte [7..0]
  prudata[16] = (DDR_address[0]) >> 8;    // Byte [15..8]
  prudata[17] = (DDR_address[0]) >> 16;   // Byte [23..16]
  prudata[18] = (DDR_address[0]) >> 24;   // MSByte [31..24]

}



uint8_t read_curve_block(){
  unsigned int DDR_address[2], address;
  uint8_t block = 0;

  FILE* fp;
  fp = fopen(MMAP1_LOC "addr", "rt");
  fscanf(fp, "%x", &DDR_address[0]);
  fclose(fp);

  // Endereco do bloco na DDR
  address = (prudata[18] << 24) + (prudata[17] << 16) +(prudata[16] << 8) +(prudata[15] << 0);
  block = (address - DDR_address[0])/CURVE_BYTES_PER_BLOCK;

  return block;
}



void *monitorRecvBuffer(void *arg){
    // ----- Copia dos dados recebidos
    uint32_t tamanho;

    while(thread_control){
        // ----- Aguarda sinal de finalizacao do ciclo
        tamanho = 0;

        if(prudata[25] == "M"){
            prussdrv_pru_wait_event(PRU_EVTOUT_1);
            prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
        }
        else{
            prussdrv_pru_wait_event(PRU_EVTOUT_0);
            prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
        }

        // ----- Nova mensagem recebida !
        while(prudata[1] != MENSAGEM_RECEBIDA_NOVA){}


        // ----- Copia dos dados recebidos
        // Tamanho
        for(i=0; i<4; i++)
        tamanho += prudata[OFFSET_SHRAM_READ+i] << i*8;
        // Dados
        for(i=0; i<tamanho; i++){
            receive_buffer[pru_pointer] = prudata[OFFSET_SHRAM_READ+4+i];
            pru_pointer++;
            // Reset pru_pointer
            if(pru_pointer == BUFF_SIZE){
                pru_pointer = 0;
            }
        }
        // ----- Sinaliza mensagem antiga
        prudata[1] = MENSAGEM_ANTIGA;
    }
}



int init_start_PRU(int baudrate, char mode){


  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

  // ----- Inicializacao da PRU
  prussdrv_init();

  // ----- Inicializacao da interrupcao PRU
  if (prussdrv_open(PRU_EVTOUT_1)){
    // printf("prussdrv_open open failed\n");
    return ERR_INIT_PRU_SSDRV;
  }
  if (prussdrv_open(PRU_EVTOUT_0)){
    // printf("prussdrv_open open failed\n");
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
    prudata[25] = mode;
  else{
    // Modo nao existente
    close_PRU();
    // printf("Requested mode does not exist.\n");
    return ERR_INIT_PRU_MODE;
  }

  // ----- Inicializacao: contador de sincronismo zerado
  prudata[80] = 0;
  prudata[81] = 0;

  // ----- Inicializacao MASTER: procedimento sincrono desabilitado e RxTimeOut = 2 bytes
  if(prudata[25]=='M')
    set_sync_stop_PRU();
    prudata[32] = 0x02;

  // ----- Inicializacao SLAVE: nenhuma mensagem nova na serial e RxTimeOut = 18 bytes
  if(prudata[25]=='S')
    prudata[1]=MENSAGEM_ANTIGA;
    prudata[32] = 0x10;

  // Endereco de Hardware
  prudata[24] = hardware_address_serialPRU();

  // ----- Configuracao Baudrate
  uint32_t one_byte_length_ns = 0;
  switch (baudrate){
    case 6:
        prudata[2] = 0x28;    // BRGCONFIG
        prudata[3] = 0x02;    // DIVLSB
        prudata[4] = 0x00;    // DIVMSB
        one_byte_length_ns = (int) 10000/6;
        break;

    case 10:
        prudata[2] = 0x28;    // BRGCONFIG
        prudata[3] = 0x01;    // DIVLSB
        prudata[4] = 0x00;    // DIVMSB
        one_byte_length_ns = (int) 10000/10;
        break;

    case 12:
        prudata[2] = 0x24;    // BRGCONFIG
        prudata[3] = 0x01;    // DIVLSB
        prudata[4] = 0x00;    // DIVMSB
        one_byte_length_ns = (int) 10000/12;
        break;

    case 9600:
      prudata[2] = 0x0a;      // BRGCONFIG
      prudata[3] = 0x86;      // DIVLSB
      prudata[4] = 0x01;      // DIVMSB
      one_byte_length_ns = (int) 100000000/96;
      break;

    case 14400:
      prudata[2] = 0x07;    // BRGCONFIG
      prudata[3] = 0x04;    // DIVLSB
      prudata[4] = 0x01;    // DIVMSB
      one_byte_length_ns = (int) 100000000/144;
      break;

    case 19200:
      prudata[2] = 0x05;    // BRGCONFIG
      prudata[3] = 0xc3;    // DIVLSB
      prudata[4] = 0x00;    // DIVMSB
      one_byte_length_ns = (int) 100000000/192;
      break;

    case 38400:
      prudata[2] = 0x15;    // BRGCONFIG
      prudata[3] = 0xc3;    // DIVLSB
      prudata[4] = 0x00;    // DIVMSB
      one_byte_length_ns = (int) 100000000/384;
      break;

    case 57600:
      prudata[2] = 0x27;    // BRGCONFIG
      prudata[3] = 0x04;    // DIVLSB
      prudata[4] = 0x01;    // DIVMSB
      one_byte_length_ns = (int) 100000000/576;
      break;

    case 115200:
      prudata[2] = 0x09;    // BRGCONFIG
      prudata[3] = 0x20;    // DIVLSB
      prudata[4] = 0x00;    // DIVMSB
      one_byte_length_ns = (int) 100000000/1152;
      break;

    default:
      close_PRU();    // BR nao definido
      // printf("Baudrate not defined.\n");
      return ERR_INIT_PRU_BAUDR;
  }
  prudata[26] = one_byte_length_ns;
  prudata[27] = one_byte_length_ns >> 8;
  prudata[28] = one_byte_length_ns >>16;

  // ----- DDR address
  unsigned int DDR_address[2];

  FILE* fp;
  fp = fopen(MMAP1_LOC "addr", "rt");
  fscanf(fp, "%x", &DDR_address[0]);
  fclose(fp);

  fp = fopen(MMAP1_LOC "size", "rt");
  fscanf(fp, "%x", &DDR_address[1]);
  fclose(fp);

  // Endereco da DDR
  prudata[15] = (DDR_address[0]) >> 0;    // LSByte [7..0]
  prudata[16] = (DDR_address[0]) >> 8;    // Byte [15..8]
  prudata[17] = (DDR_address[0]) >> 16;   // Byte [23..16]
  prudata[18] = (DDR_address[0]) >> 24;   // MSByte [31..24]

  // ----- Executar codigo na PRU
  prussdrv_exec_program (PRU_NUM, PRU_BINARY);

  // ----- Lanca thread para monitorar recebimento de dados
  //        e reinicializa ponteiros
    pru_pointer = 0;
    read_pointer = 0;
      if(tid == 0){
          thread_control = 1;
          pthread_create(&tid, NULL, monitorRecvBuffer, NULL);
      }
  return OK;
}



int send_data_PRU(uint8_t *data, uint32_t *tamanho, float timeout_ms){

  uint32_t timeout_instructions;
  float timeout_inst;

  timeout_inst = timeout_ms*66600;
  timeout_instructions = (int)timeout_inst;

  // ----- MASTER: Configuracao do Timeout
  if(prudata[25]=='M'){
    prudata[6] = timeout_instructions;        // LSByte do timeout_instructions [7..0]
    prudata[7] = timeout_instructions >> 8;   // Byte do timeout_instructions [15..8]
    prudata[8] = timeout_instructions >> 16;  // Byte do timeout_instructions [23..16]
    prudata[9] = timeout_instructions >> 24;  // MSByte do timeout_instructions [31..24]
  }

  // ----- Tamanho dos dados
  prudata[OFFSET_SHRAM_WRITE] = *tamanho;           // LSByte do tamanho dos dados [7..0]
  prudata[OFFSET_SHRAM_WRITE+1] = *tamanho >> 8;    // Byte do tamanho dos dados [15..8]
  prudata[OFFSET_SHRAM_WRITE+2] = *tamanho >> 16;   // Byte do tamanho dos dados [23..16]
  prudata[OFFSET_SHRAM_WRITE+3] = *tamanho >> 24;   // MSByte do tamanho dos dados [31..24]


  // ----- Insere na memoria - Dados a enviar
  for(i=0; i<*tamanho; i++)
    prudata[OFFSET_SHRAM_WRITE+4+i] = data[i];

  // ----- Dados prontos para envio
  prudata[1] = MENSAGEM_PARA_ENVIAR;

  // ----- Aguarda sinal de finalizacao do ciclo
  prussdrv_pru_wait_event(PRU_EVTOUT_1);
  prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);


  // Aguarda dados prontos na Shared RAM (M) ou fim do envio (S)
  if(prudata[25]=='M'){
      while(prudata[1] != MENSAGEM_RECEBIDA_NOVA);
      /*
      uint32_t tamanho = 0;

      // ----- Copia dos dados recebidos
      // Tamanho
      for(i=0; i<4; i++)
        tamanho += prudata[OFFSET_SHRAM_READ+i] << i*8;

      // Dados
      for(i=0; i<tamanho; i++){
        receive_buffer[pru_pointer] = prudata[OFFSET_SHRAM_READ+4+i];
        pru_pointer++;
        // Reset pru_pointer
        if(pru_pointer == BUFF_SIZE){
            pru_pointer = 0;
        }
      }
      */
  }

  // ----- SLAVE: Aguarda fim de envio
  if(prudata[25]=='S'){
    while(prudata[1] != 0x55);
  }
  return OK;
}



int recv_data_PRU(uint8_t *data, uint32_t *tamanho, uint32_t bytes2read){
    // ---------- MASTER MODE ----------
    //  if(prudata[25]=='M'){
    if(pru_pointer == read_pointer){
        *tamanho = 0;
        return OK;
    }

    if (pru_pointer > read_pointer){
        *tamanho = pru_pointer - read_pointer;
    }
    else{
        *tamanho = BUFF_SIZE - (read_pointer - pru_pointer);
    }

    if(bytes2read != 0 & *tamanho > bytes2read)
    {
        *tamanho = bytes2read;
    }

    for(i=0; i<*tamanho; i++){
        data[i] = receive_buffer[read_pointer];
        read_pointer++;
        // Reset pointer
        if(read_pointer == BUFF_SIZE){
            read_pointer = 0;
        }

    }
    return OK;
}


    /*
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
*/




  // ---------- SLAVE MODE ----------
  /*
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

      return OK;
    }


    // ----- Mensagem antiga no buffer
    if(prudata[1] == MENSAGEM_ANTIGA)
      return ERR_RECV_DATA_OLDMSG;
  }

}*/
