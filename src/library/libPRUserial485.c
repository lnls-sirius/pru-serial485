// ------------------------------------------------------------------------------------------------
// Python Module for PRUserial485 Interface
//
// Mapping .c functions into Python methods in order to ease code development without losing
// performance.
//
// CONTROLS GROUP
// Author: Patricia Nallin (patricia.nallin@lnls.br)
// Release Date: Jan 13, 2020
//
// ------------------------------------------------------------------------------------------------

#include <Python.h>
#include <stdio.h>
#include <prussdrv.h>
#include <PRUserial485.h>

// ------------------------------------------------------------------------------------------------
// int PRUserial485_open(int baudrate, char mode) -> int init_start_PRU(int baudrate, char mode)
// ------------------------------------------------------------------------------------------------
PyObject* pru_open(PyObject* self, PyObject *args)
{
 	int br;
 	char mode;

 	if (!PyArg_ParseTuple(args, "ic", &br, &mode))
    {
 		return NULL;
 	}

    return Py_BuildValue("i", init_start_PRU(br,mode));
}

// ------------------------------------------------------------------------------------------------
// void PRUserial485_close() -> void close_PRU()
// ------------------------------------------------------------------------------------------------
PyObject* pru_close(PyObject* self, PyObject *args)
{
    close_PRU();
     return Py_BuildValue("s", NULL);
}

// ------------------------------------------------------------------------------------------------
// int PRUserial485_write(bytes/bytearray, float timeout) -> int send_data_PRU(char array, uint32 length, float timeout)
// ------------------------------------------------------------------------------------------------
PyObject* pru_send(PyObject* self, PyObject *args)
{
    float timeout;
    uint32_t data_size;
    const uint8_t * str_input;
    uint8_t * data;

    if (!PyArg_ParseTuple(args, "s#f", &str_input, &data_size, &timeout))
    {
        PyErr_SetString(PyExc_TypeError, "data must be 'bytes' and timeout a positive 'float' value");
        return NULL;
    }

    data = (uint8_t *)malloc(data_size);

    for(uint32_t i=0; i<data_size; i++)
    {
        data[i]=str_input[i];
    }

    int res = send_data_PRU(data, &data_size, timeout);
    free(data);

    return Py_BuildValue("i", res);
}

// ------------------------------------------------------------------------------------------------
// bytes PRUserial485_read() -> int recv_data_PRU(char array, uint32 length)
// ------------------------------------------------------------------------------------------------
PyObject* pru_recv(PyObject* self, PyObject *args)
{
    uint32_t data_size;
    uint8_t data[2048];

    recv_data_PRU(data, &data_size);

    return Py_BuildValue("y#", data, data_size);
}

// ------------------------------------------------------------------------------------------------
// int PRUserial485_address() -> int hardware_address_serialPRU()
// ------------------------------------------------------------------------------------------------
static PyObject* pru_address(PyObject* self, PyObject *args)
{
        return Py_BuildValue("i", hardware_address_serialPRU());
}

// ------------------------------------------------------------------------------------------------
// str __version__()
// ------------------------------------------------------------------------------------------------
static PyObject* pru_version(PyObject* self, PyObject *args)
{
    return Py_BuildValue("s", VERSIONHASH);
}




// ------------------------------------------------------------------------------------------------
// Python Module definitions, methods and initialization
// ------------------------------------------------------------------------------------------------

static PyMethodDef pruserial485_funcs[] = {
     {"PRUserial485_open", (PyCFunction)pru_open,      METH_VARARGS, NULL},
     {"PRUserial485_close", (PyCFunction)pru_close,      METH_VARARGS, NULL},
     {"PRUserial485_address", (PyCFunction)pru_address,      METH_VARARGS, NULL},
     {"PRUserial485_write", (PyCFunction)pru_send,      METH_VARARGS, NULL},
     {"PRUserial485_read", (PyCFunction)pru_recv,      METH_VARARGS, NULL},
     {"__version__", (PyCFunction)pru_version,      METH_VARARGS, NULL},
     {NULL}
};

static struct PyModuleDef cModPyDem = {
    PyModuleDef_HEAD_INIT,
    "PRUserial485",
    "",          /* module documentation */
    -1,
    pruserial485_funcs
};

PyMODINIT_FUNC PyInit_PRUserial485(void)
{
    return PyModule_Create(&cModPyDem);
}
