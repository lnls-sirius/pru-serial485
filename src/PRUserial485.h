#ifdef __cplusplus
extern "C" {
#endif


/* INICIALIZACAO DA PRU
 * Parametro 'baudrate': velocidade de comunicacao RS485
 *
 * Velocidades disponiveis:
 * (6)     Mbps		|	(19200)  bps
 * (10)    Mbps		|	(38400)  bps
 * (12)    Mbps		|	(57600)  bps
 * (9600)  bps		|	(115200) bps
 * (14400) bps		|
*/
int init_start_PRU(int baudrate);



/* ENVIO DE DADOS
 * --Parametros--
 * data: valores a serem enviados
 * tamanho: quantidade de bytes a serem enviados (tamanho util do vetor data)
 *
 * --Retorno--
 * A funcao retorna SOMENTE após o recebimento de uma resposta do destino
 * (resposta válida ou timeout)
*/
int send_data_PRU(uint8_t *data, uint32_t *tamanho);



/* RECEBIMENTO DE DADOS
 * --Parametros--
 * data: local para armazenamento de dados recebidos
 * tamanho: quantidade de bytes recebidos (tamanho util do vetor data)
 *
 *
 * Obs: Se tamanho == 0:
 * - Timeout
 * - Comando enviado nao requis resposta
*/
int recv_data_PRU(uint8_t *data, uint32_t *tamanho);



/* PROCEDIMENTO SINCRONO - START
 * Sinaliza o inicio do procedimento sincrono via PRU
*/
void set_sync_start_PRU();



/* PROCEDIMENTO SINCRONO - STOP
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
