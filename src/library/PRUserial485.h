/*
PRUserial485.h


--------------------------------------------------------------------------------
RS-485 communication via PRU
--------------------------------------------------------------------------------
Interfaces with SERIALxxCON Hardware (v2-3)

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group

Author: Patricia HENRIQUES NALLIN
Date: July/2024
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <unistd.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>





/* Return codes */

#define SYNC_OFF 0                 // sync_status
#define SYNC_ON 1                  // sync_status
#define OK 0
#define ERR_CLEAR_PULSE 1          // clear_pulse_count_sync
#define ERR_LD_CURVE_MOPEN 2       // loadCurve
#define ERR_LD_CURVE_MMAP 3        // loadCurve
#define ERR_LD_CURVE_UMMAP 4       // loadCurve
#define ERR_CURVE_OVER_BLOCK 11    // loadCurve + setCurveBlock
#define ERR_CURVE_OVER_POINTS 12   // loadCurve + setCurvePointer
#define ERR_INIT_PRU_SSDRV 5       // init_start_PRU
#define ERR_INIT_PRU_MODE 6        // init_start_PRU
#define ERR_INIT_PRU_BAUDR 7       // init_start_PRU
#define ERR_RECV_DATA_OLDMSG 8     // recv_data_PRU
#define ERR_FF_TABLE_NUMBER 9      // ff_configure
#define ERR_FF_IDTYPE 10           // ff_configure
#define STS_FF_ENABLED 1
#define STS_FF_DISABLED 0




#ifdef __cplusplus
extern "C" {
#endif



/* ENDERECO DA PLACA
 * --Retorno--
 * Retorna o endereco fisico da placa, selecionado em hardware.
 * Util apenas no modo SLAVE
 * 
*/
uint8_t hardware_address_serialPRU();



int load_ddr(unsigned int ddr_offset, uint32_t table_points, float *curve1, float *curve2, float *curve3, float *curve4);



uint32_t read_ddr(unsigned int ddr_offset, uint32_t table_points, float *curve1, float *curve2, float *curve3, float *curve4);



/* ZERA CONTADOR DE PULSOS - SINCRONISMO
 *
 *
 * Se o sincronismo estiver desabilitado, zera contador de pulsos
 *
 * --Retorno--
 *  OK              : apos zerar contador
 *  ERR_CLEAR_PULSE : Caso o modo sincrono esteja habilitado no modo master
 * 
*/
int clear_pulse_count_sync();



/* LEITURA CONTADOR DE PULSOS - SINCRONISMO
 *
 *
 * --Retorno--
 * Valor do contador de pulsos
 * (Contagem de pulsos físicos no modo MASTER --- Contagem de mensagem de passo sync no modo SLAVE)
 * 
*/
uint32_t read_pulse_count_sync();



/* USO GERAL - LEITURA DE BYTE DA MEMORIA COMPARTILHADA (SHARED RAM)
 *
 * --Parametro--
 * offset: index do byte a ser lido
 * --Retorno--
 * Valor do byte
 * 
*/
uint8_t read_shram(uint16_t offset);



/* USO GERAL - ESCRITA EM BYTE DA MEMORIA COMPARTILHADA (SHARED RAM)
 *
 *
 * --Parametro--
 * offset: index do byte a ser escrito
 * value: valor a ser escrito
 * 
*/
void write_shram(uint16_t offset, uint8_t value);



/* PROCEDIMENTO SINCRONO - ESCOLHA DO PROXIMO PONTO A SER EXECUTADO
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * new_pointer: ponto de onde a curva sera executada a partir do proximo pulso de sincronismo. Parametro incrementado
 * automaticamente apos cada pulso.
 * 
*/
void set_curve_pointer(uint32_t new_pointer);



/* PROCEDIMENTO SINCRONO - ESCOLHA DO PROXIMO PONTO A SER EXECUTADO
 *
* --Retorno--
 * MODO MASTER:
 * A funcao retorna o indice do proximo ponto que sera executado, apos o pulso de sincronismo.
 * 
*/
uint32_t read_curve_pointer();



/* PROCEDIMENTO SINCRONO - STATUS DE PRONTO
 *
* --Retorno--
 * MODO MASTER:
 * SYNC_ON : pronto
 * SYNC_OFF: desarmado
 * 
*/
int sync_status();



/* PROCEDIMENTO SINCRONO - START
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * Sync_Mode:  | 0x51 - Single curve sequence & Intercalated read messages
 *             | 0x5E - Single curve sequence & Read messages at End of curve
 *             | 0xC1 - Continuous curve sequence & Intercalated read messages
 *             | 0xCE - Continuous curve sequence & Read messages at End of curve
 *             | 0x5B - Single Sequence - Single Broadcast Function command
 * delay: tempo aproximado entre o fim da mensagem de sincronismo e o inicio de uma mensagem normal de requisicao. Unidade: microssegundos.
 * sync_address: endereco do controlador que recebera os comandos de SetIx4 (setpoints da curva) -> Caso modo != 0x5B
 * Sinaliza o inicio do procedimento sincrono via PRU
 * 
*/
void set_sync_start_PRU(uint8_t sync_mode, uint32_t delay_us, uint8_t sync_address);



/* PROCEDIMENTO SINCRONO - STOP
 *
 * SOMENTE MODO MASTER
 *
 * Sinaliza o encerramento do procedimento sincrono via PRU
 * 
*/
void set_sync_stop_PRU();



/* DESABILITA PRU
 * Encerra atividade da PRU e fecha o mapeamento da memoria compartilhada
 *
*/
void close_PRU();



/* PROCEDIMENTO SINCRONO - CARREGAMENTO DOS PONTOS DA CURVA
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * curveX: vetor dos pontos da curva, correnspondente a cada fonte FBP do bastidor, em ponto flutuante
 * CurvePoints: numero de pontos da curva. Todas as curvas devem ter o MESMO tamanho.
 * blocks: numero do bloco no qual a curva sera escrita.
 * --Retorno--
 * OK
 * ERR_LD_CURVE_MOPEN : failed to open memory.
 * ERR_LD_CURVE_MMAP  : failed to map base address.
 * ERR_LD_CURVE_UMMAP : failed to unmap memory.
 *
*/
int loadCurve(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t CurvePoints, uint8_t block);



/* PROCEDIMENTO SINCRONO - SELECAO DE BLOCO DE CURVAS
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * block: Numero do bloco a ser executado a partir do proximo ciclo. Inicializacao no bloco 0.
 *
*/
void set_curve_block(uint8_t block);



/* PROCEDIMENTO SINCRONO - LEITURA DO BLOCO DE CURVAS
 *
* --Retorno--
 * MODO MASTER:
 * Numero do bloco que sera executado no proximo ciclo
 *
*/
uint8_t read_curve_block();



/* INICIALIZACAO DA PRU
 * --Parametros--
 * baudrate: velocidade de comunicacao RS485
 * mode: modo de operacao. 'M' para master e 'S' para slave
 *
 * Velocidades disponiveis:
 * (1)     Mbps    |
 * (6)     Mbps    |  (19200)  bps
 * (10)    Mbps    |  (38400)  bps
 * (12)    Mbps    |  (57600)  bps
 * (9600)  bps     |  (115200) bps
 * (14400) bps     |
 *
 * --Retorno--
 * OK
 * ERR_INIT_PRU_SSDRV : prussdrv_open open failed
 * ERR_INIT_PRU_MODE  : Requested mode does not exist.
 * ERR_INIT_PRU_BAUDR : Baudrate not defined.
*/
int init_start_PRU(int baudrate, char mode);



/* ENVIO DE DADOS
 * --Parametros--
 * data: valores a serem enviados
 * tamanho: quantidade de bytes a serem enviados (tamanho util do vetor data)
 * timeout_ms: tempo maximo de espera para comecar a receber uma resposta (em ms). Minimo: 15ns / Maximo: 64s. Se 0, nao aguarda resposta.
 *  ##### ATENCAO: no MODO SLAVE o parametro TIMEOUT_MS e ignorado. #####
 *
 *
 * --Retorno--
 * MODO MASTER:
 * A funcao retorna OK SOMENTE apos o recebimento de uma resposta do destino
 * (resposta válida, ignorada ou timeout)
 *
 * MODO SLAVE:
 * A funcao retorna OK apos o envio dos dados
 * 
*/
int send_data_PRU(uint8_t *data, uint32_t *tamanho, float timeout_ms);



/* RECEBIMENTO DE DADOS
 * --Parametros--
 * data: local para armazenamento de dados recebidos
 * tamanho: quantidade de bytes recebidos (tamanho util do vetor data)
 *
 * Obs: Se tamanho == 0:
 * - Timeout
 * - Comando enviado nao requis resposta
 *
 *
 *
 * * --Retorno--
 * MODO MASTER:
 * 0:    apos a copia da resposta recebida (resposta válida, ignorada ou
 *     timeout) nos enderecos indicados pelos parametros da funcao.
 *
 *
 * MODO SLAVE:
 * OK                   : apos a copia dos novos dados recebidos nos enderecos
 *                        indicados pelos parametros da funcao.
 * ERR_RECV_DATA_OLDMSG : caso nao exista novos dados de recepcao no buffer
 * 
*/
int recv_data_PRU(uint8_t *data, uint32_t *tamanho, uint32_t bytes2read);



/* RECEBIMENTO DE DADOS
 *
 * Flush na FIFO de recebimento da PRUserial485.
 * --Retorno--
 * OK
 *
*/
int recv_flush();



/* FF - CONFIGURACOES INICIAIS E INICIALIZACAO DA PRU
 *
 * --Parametros--
 * idType: tipo de ID (0: Delta | 1: IVU | 2: VPU)
 * nTables: quantidade de polarizacoes/modos/tabelas a serem pré-configuradas
 * max_range: posicao maxima do ID (gap/fase), em micrometros.
 * --Retorno--
 * OK
 * ERR
 *
*/
int ff_configure(uint8_t idType, uint8_t nTables, float max_range);



/* FF - HABILITAR
 * Habilita loop feed-forward: aquisição de dados de posição + envio de setpoints de corrente, que devem ser previamente configurados (tabelas)
 *
*/
void ff_enable();



/* FF - DESABILITAR
 * Habilita loop feed-forward: aquisição de dados de posição + envio de setpoints de corrente, que devem ser previamente configurados (tabelas)
 *
*/
void ff_disable();



/* FF - STATUS DE OPERAÇÃO
 *
 * Estado do loop de controle da operação de ajustes de corrente.
 * --Retorno--
 * 0: desabilitado
 * Diferente de 0: habilitado
 *
*/
int ff_get_status();



/* FF - TAMANHO DA TABELA DE CORRECAO
 *
 * --Retorno--
 * Tamanho, em quantidade de pontos, de cada tabela referente à uma fonte.
 *
*/
int ff_get_table_size();



/* FF - CARREGAMENTO DOS PONTOS DAS TABELAS
 *
 * --Parametro--
 * curveX: vetor dos pontos da curva, correnspondente a cada fonte FBP do bastidor, em ponto flutuante
 * table_points: numero de pontos de cada tabela/curva. Todas as tabelas devem ter o MESMO tamanho.
 * table: numero referencial da tabela na qual a curva sera armazenada.
 * --Retorno--
 * OK
 * ERR...
 *
*/
int ff_load_table(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t table_points, uint8_t table);



/* FF - LEITURA DOS PONTOS DAS TABELAS
 *
 * --Parametros--
 * curveX: vetor para armazenamento dos pontos da curva, correnspondente a cada fonte FBP do bastidor, em ponto flutuante
 * table: numero referencial das tabelas a serem consultadas.
 * --Retorno--
 * Quantidade de pontos de cada curva, correspondente a cada fonte FBP
 *
*/
uint32_t ff_read_table(float *curve1, float *curve2, float *curve3, float *curve4, uint8_t table);



/* FF - TABELA EM USO
 *
 * --Retorno--
 * Numero da tabela em uso, correspondente ao modo/polarizacao do ID
 *
*/
uint8_t ff_read_current_table();



/* FF - PONTEIRO DA TABELA DE CORRECAO
 *
 * --Retorno--
 * Indice/Ponteiro relativo ao último ponto aplicado da tabela de correcao
 *
*/
uint16_t ff_read_current_pointer();



/* FF - GAP/FASE
 *
 * --Retorno--
 * Valor do gap/fase atual recebido
 *
*/
int ff_read_current_position();



/* FF - FLAGS DE SINALIZACAO
 *
 * --Retorno--
 * Byte [b7..b0] com os flags.
 *
*/
uint8_t ff_read_flags();



/* FF - LIMPA FLAGS
 * Reinicializa flags de sinalização da operação feed-forward
 *
*/
void ff_clear_flags();




#ifdef __cplusplus
}
#endif
