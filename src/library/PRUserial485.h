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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <unistd.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


#ifdef __cplusplus
extern "C" {
#endif




/* PROCEDIMENTO SINCRONO - CARREGAMENTO DOS PONTOS DA CURVA
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * curveX: vetor dos pontos da curva, correnspondente a cada fonte FBP do bastidor, em ponto flutuante
 * CurvePoints: numero de pontos da curva. Todas as curvas devem ter o MESMO tamanho.
 * blocks: numero do bloco no qual a curva sera escrita.
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


/* PROCEDIMENTO SINCRONO - STATUS DE PRONTO
 *
* --Retorno--
 * MODO MASTER:
 * 1: pronto
 * 0: desarmado
 *
*/
int sync_status();


/* PROCEDIMENTO SINCRONO - ESCOLHA DO PROXIMO PONTO A SER EXECUTADO
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * new_pointer: ponto de onde a curva sera executada a partir do proximo pulso de sincronismo. Parametro incrementado
 * automaticamente apos cada pulso.
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




/* INICIALIZACAO DA PRU
 * --Parametros--
 * baudrate: velocidade de comunicacao RS485
 * mode: modo de operacao. 'M' para master e 'S' para slave
 *
 * Velocidades disponiveis:
 * (6)     Mbps    |  (19200)  bps
 * (10)    Mbps    |  (38400)  bps
 * (12)    Mbps    |  (57600)  bps
 * (9600)  bps     |  (115200) bps
 * (14400) bps     |
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
 * A funcao retorna SOMENTE apos o recebimento de uma resposta do destino
 * (resposta válida, ignorada ou timeout)
 *
 * MODO SLAVE:
 * A funcao retorna apos o envio dos dados
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
 * 0:    apos a copia dos novos dados recebidos nos enderecos indicados
 *     pelos parametros da funcao.
 *-1:  caso nao exista novos dados de recepcao no buffer
*/
int recv_data_PRU(uint8_t *data, uint32_t *tamanho);




/* PROCEDIMENTO SINCRONO - START
 *
 * SOMENTE MODO MASTER
 * --Parametro--
 * PROCEDIMENTO SINCRONO - START
 * Sync_Mode   | 0x51 - Single curve sequence & Intercalated read messages
 *             | 0x5E - Single curve sequence & Read messages at End of curve
 *             | 0xC1 - Continuous curve sequence & Intercalated read messages
 *             | 0xCE - Continuous curve sequence & Read messages at End of curve
 *             | 0x5C - Single Sequence - Single CYCLING COMMAND
 * delay: tempo aproximado entre o fim da mensagem de sincronismo e o inicio de uma mensagem normal de requisicao. Unidade: microssegundos.
 * sync_address: endereco do controlador que recebera os comandos de SetIx4 (setpoints da curva) -> Caso modo != 0x5C
 * Sinaliza o inicio do procedimento sincrono via PRU
*/
void set_sync_start_PRU(uint8_t sync_mode, uint32_t delay_us, uint8_t sync_address);




/* PROCEDIMENTO SINCRONO - STOP
 *
 * SOMENTE MODO MASTER
 *
 * Sinaliza o encerramento do procedimento sincrono via PRU
*/
void set_sync_stop_PRU();



/* ENDERECO DA PLACA
 * Retorna o endereco fisico da placa, selecionado em hardware.
 * Util apenas no modo SLAVE
*/
uint8_t hardware_address_serialPRU();



/* ZERA CONTADOR DE PULSOS - SINCRONISMO
 *
 *
 * Se o sincronismo estiver desabilitado, zera contador de pulsos
 *
 * --Retorno--
 *  0: apos zerar contador
 * -1: Caso o modo sincrono esteja habilitado no modo master
*/
int clear_pulse_count_sync();



/* LEITURA CONTADOR DE PULSOS - SINCRONISMO
 *
 *
 * --Retorno--
 * Valor do contador de pulsos
 * (Contagem de pulsos físicos no modo MASTER --- Contagem de mensagem de passo sync no modo SLAVE)
*/
uint32_t read_pulse_count_sync();



/* DESABILITA PRU
 * Encerra atividade da PRU e fecha o mapeamento da memoria compartilhada
*/
void close_PRU();

#ifdef __cplusplus
}
#endif
