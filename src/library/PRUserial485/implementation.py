#!/usr/bin/env python-sirius
# -*- coding: utf-8 -*-

"""
PRUserial485.

Simples interface em Python 2 (através do módulo ctypes) para comunicação
serial RS-485 usando o hardware já desenvolvido e a biblioteca
libPRUserial485.
"""

# Importa o módulo ctypes
if __name__ != '__main__':
    import ctypes
else:
    # Este arquivo é um módulo Python, e não deve ser executado
    exit()

# Carrega bibliotecas
try:
    # Carrega as bibliotecas dinâmicas das quais a biblioteca libPRUserial485
    # depende (bibliotecas da PRU)
    ctypes.CDLL("libprussdrv.so", mode=ctypes.RTLD_GLOBAL)
    ctypes.CDLL("libprussdrvd.so", mode=ctypes.RTLD_GLOBAL)
    # Carrega a biblioteca libPRUserial485
    libPRUserial485 = ctypes.CDLL(
        "libPRUserial485.so", mode=ctypes.RTLD_GLOBAL)
    libraries_loaded = True
except OSError:
    libraries_loaded = False

# Buffer de 8 kB para o envio e recebimento de dados
data_buffer = (ctypes.c_uint8 * 8192*2)()

# Buffer para armazenamento dos pontos de curva
C1_buffer = (ctypes.c_float * 262144)()
C2_buffer = (ctypes.c_float * 262144)()
C3_buffer = (ctypes.c_float * 262144)()
C4_buffer = (ctypes.c_float * 262144)()


# Variável que armazena o tamanho da última mensagem trocada (em bytes)
data_size = ctypes.c_uint32(0)


class ConstReturn:
    """Namespace for return constants."""

    SYNC_OFF = 0  # sync_status
    SYNC_ON = 1  # sync_status
    OK = 0
    ERR_CLEAR_PULSE = 1  # clear_pulse_count_sync
    ERR_LD_CURVE_MOPEN = 2  # loadCurve
    ERR_LD_CURVE_MMAP = 3  # loadCurve
    ERR_LD_CURVE_UMMAP = 4  # loadCurve
    ERR_INIT_PRU_SSDRV = 5  # init_start_PRU
    ERR_INIT_PRU_MODE = 6  # init_start_PRU
    ERR_INIT_PRU_BAUDR = 7  # init_start_PRU
    ERR_RECV_DATA_OLDMSG = 8  # recv_data_PRU


class ConstSyncMode:
    """Namespace for PRU sync modes."""

    MIGINT = 0x51  # Single curve sequence & Read msgs at End of curve
    MIGEND = 0x5E  # Single curve sequence & Read msgs at End of curve
    RMPINT = 0xC1  # Contin. curve sequence & Intercalated read messages
    RMPEND = 0xCE  # Contin. curve sequence & Read msgs at End of curve
    BRDCST = 0x5B  # Single Sequence - Single Broadcast Function command
    ALL = (MIGINT, MIGEND, RMPINT, RMPEND, BRDCST)


def PRUserial485_open(baudrate, mode):
    """Procedimento de inicialização da PRU."""
    ret = libPRUserial485.init_start_PRU(baudrate, ctypes.c_char(mode))
    return ret


def PRUserial485_curve(curve1, curve2, curve3, curve4, block):
    """Carregamento de curva."""
    if len(curve1) == len(curve2) == len(curve3) == len(curve4):
        for i in range(0, len(curve1)):
            C1_buffer[i] = curve1[i]
            C2_buffer[i] = curve2[i]
            C3_buffer[i] = curve3[i]
            C4_buffer[i] = curve4[i]
        ret = libPRUserial485.loadCurve(ctypes.byref(C1_buffer),
                                        ctypes.byref(C2_buffer),
                                        ctypes.byref(C3_buffer),
                                        ctypes.byref(C4_buffer),
                                        len(curve1), block)
        return ret
    else:
        raise ValueError("Erro: Curvas nao tem o mesmo tamanho!")


def PRUserial485_set_curve_block(block):
    """Selecao de bloco de curva a ser realizado."""
    libPRUserial485.set_curve_block(block)


def PRUserial485_read_curve_block():
    """Leitura do bloco de curva que sera realizado."""
    return(libPRUserial485.read_curve_block())


def PRUserial485_set_curve_pointer(next_point):
    """Ajusta ponteiro para proximo ponto a ser executado (curva)."""
    libPRUserial485.set_curve_pointer(next_point)


def PRUserial485_read_curve_pointer():
    """Leitura do ponteiro de curva (proximo ponto que sera executado)."""
    return(libPRUserial485.read_curve_pointer())


def PRUserial485_write(request, reply_timeout):
    """Envia dados através da interface serial."""
    if (len(request) == 0):
        return
    i = 0
    while (i < len(request)):
        data_buffer[i] = ord(request[i])
        i += 1
    data_size.value = len(request)
    ret = libPRUserial485.send_data_PRU(ctypes.byref(data_buffer),
                                        ctypes.byref(data_size),
                                        ctypes.c_float(reply_timeout))
    return ret


def PRUserial485_read(bytes2read=0):
    """Recebe dados através da interface serial."""
    recv_ok = libPRUserial485.recv_data_PRU(ctypes.byref(data_buffer),
                                            ctypes.byref(data_size),
                                            bytes2read)
    answer = []
    i = 0
    while (i < data_size.value):
        answer.append(chr(data_buffer[i]))
        i += 1
    # Resposta antiga no buffer (recv_ok = -1)
    if(recv_ok):
        return []
    else:
        return(answer)


def PRUserial485_sync_start(sync_mode, delay, sync_address=0x00):
    """Inicia operação em modo síncrono."""
    libPRUserial485.set_sync_start_PRU(sync_mode, delay, sync_address)


def PRUserial485_sync_stop():
    """Finaliza a operação em modo síncrono."""
    libPRUserial485.set_sync_stop_PRU()


def PRUserial485_sync_status():
    """Verifica se sincronismo via PRU está aguardando pulso."""
    if libPRUserial485.sync_status():
        return True
    else:
        return False


def PRUserial485_clear_pulse_count_sync():
    """Zera contador de pulsos - Sync."""
    return(libPRUserial485.clear_pulse_count_sync())


def PRUserial485_read_pulse_count_sync():
    """Leitura do contador de pulsos - Sync."""
    return(libPRUserial485.read_pulse_count_sync())


def PRUserial485_address():
    """Retorna endereco fisico da placa."""
    return(libPRUserial485.hardware_address_serialPRU())


def PRUserial485_close():
    """Encerra a PRU."""
    libPRUserial485.close_PRU()
