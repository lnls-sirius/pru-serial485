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


/* INICIALIZACAO DA PRU
 * --Parametros--
 * baudrate: velocidade de comunicacao RS485
 * mode: modo de operacao. 'M' para master e 'S' para slave
 *
 * Velocidades disponiveis:
 * (6)     Mbps		|	(19200)  bps
 * (10)    Mbps		|	(38400)  bps
 * (12)    Mbps		|	(57600)  bps
 * (9600)  bps		|	(115200) bps
 * (14400) bps		|
*/
int init_start_PRU(int baudrate, char mode);




/* ENVIO DE DADOS
 * --Parametros--
 * data: valores a serem enviados
 * tamanho: quantidade de bytes a serem enviados (tamanho util do vetor data)
 * timeout_ms: tempo maximo de espera para receber uma resposta (em ms). Se 0, nao aguarda resposta.
 *  ##### ATENCAO: no MODO SLAVE o parametro TIMEOUT e ignorado. #####
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
 * 0:  	apos a copia da resposta recebida (resposta válida, ignorada ou
 * 		timeout) nos enderecos indicados pelos parametros da funcao.
 *
 *
 * MODO SLAVE:
 * 0:  	apos a copia dos novos dados recebidos nos enderecos indicados
 * 		pelos parametros da funcao.
 *-1:	caso nao exista novos dados de recepcao no buffer
*/
int recv_data_PRU(uint8_t *data, uint32_t *tamanho);




/* PROCEDIMENTO SINCRONO - START
 *
 * SOMENTE MODO MASTER
 *
 * Sinaliza o inicio do procedimento sincrono via PRU
*/
void set_sync_start_PRU();




/* PROCEDIMENTO SINCRONO - STOP
 *
 * SOMENTE MODO MASTER
 *
 * Sinaliza o encerramento do procedimento sincrono via PRU
*/
void set_sync_stop_PRU();




/* DESABILITA PRU
 * Encerra atividade da PRU e fecha o mapeamento da memoria compartilhada
*/
void close_PRU();

#ifdef __cplusplus
}
#endif
