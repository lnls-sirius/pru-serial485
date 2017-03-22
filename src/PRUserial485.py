#!/usr/bin/python
# -*- coding: utf-8 -*-

####################################################################################################
#
# Simples interface em Python 2 (através do módulo ctypes) para comunicação serial RS-485 usando o
# hardware já desenvolvido e a biblioteca libPRUserial485.
#
####################################################################################################

# Este arquivo é um módulo Python, e não deve ser executado
if (__name__ == "__main__"):
    exit()

# Importa o módulo ctypes
import ctypes

# Carrega as bibliotecas dinâmicas das quais a biblioteca libPRUserial485 depende (bibliotecas da
# PRU)
ctypes.CDLL("libprussdrv.so", mode = ctypes.RTLD_GLOBAL)
ctypes.CDLL("libprussdrvd.so", mode = ctypes.RTLD_GLOBAL)

# Carrega a biblioteca libPRUserial485
libPRUserial485 = ctypes.CDLL("libPRUserial485.so", mode = ctypes.RTLD_GLOBAL)

# Buffer de 8 kB para o envio e recebimento de dados
data_buffer = (ctypes.c_uint8 * 8192)()

# Variável que armazena o tamanho da última mensagem trocada (em bytes)
data_size = ctypes.c_uint32(0)


# Procedimento de inicialização da PRU
def PRUserial485_open(baudrate, mode):
    libPRUserial485.init_start_PRU(baudrate, ctypes.c_char(mode))


# Envia dados através da interface serial
def PRUserial485_write(request, reply_timeout):
    if (len(request) == 0):
        return
    i = 0
    while (i < len(request)):
        data_buffer[i] = ord(request[i])
        i += 1
    data_size.value = len(request)
    libPRUserial485.send_data_PRU(ctypes.byref(data_buffer), ctypes.byref(data_size), ctypes.c_float(reply_timeout))


# Recebe dados através da interface serial.
def PRUserial485_read():
    recv_ok = libPRUserial485.recv_data_PRU(ctypes.byref(data_buffer), ctypes.byref(data_size))
    answer = []
    i = 0
    while (i < data_size.value):
        answer.append(chr(data_buffer[i]))
        i += 1
    # Resposta antiga no buffer (recv_ok = -1)
    if(revc_ok):
        return []
    else:
        return(answer)


# Inicia operação em modo síncrono
def PRUserial485_sync_start(delay):
    libPRUserial485.set_sync_start_PRU(delay)


# Finaliza a operação em modo síncrono
def PRUserial485_sync_stop():
    libPRUserial485.set_sync_stop_PRU()


# Zera contador de pulsos - Sync
def PRUserial485_clear_pulse_count_sync():
    return(libPRUserial485.clear_pulse_count_sync())


# Leitura do contador de pulsos - Sync
def PRUserial485_read_pulse_count_sync():
    return(libPRUserial485.read_pulse_count_sync())


# Retorna endereco fisico da placa
def PRUserial485_address():
    return(libPRUserial485.hardware_address_serialPRU())


# Encerra a PRU
def PRUserial485_close():
    libPRUserial485.close_PRU()