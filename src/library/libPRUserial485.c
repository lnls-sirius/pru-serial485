#include <Python.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <prussdrv.h>
#include <PRUserial485.h>

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

PyObject* pru_close(PyObject* self, PyObject *args)
{
    close_PRU();
     return Py_BuildValue("s", NULL);
}

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


PyObject* pru_recv(PyObject* self, PyObject *args)
{
    uint32_t data_size;
    uint8_t data[2048];

    recv_data_PRU(data, &data_size);

    return Py_BuildValue("y#", data, data_size);
}


static PyObject* pru_address(PyObject* self, PyObject *args)
{
        return Py_BuildValue("i", hardware_address_serialPRU());
}


static PyObject* pru_version(PyObject* self, PyObject *args)
{
    return Py_BuildValue("s", VERSION-HASH);
}



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
    "PRUserial485", /* name of module */
    "",          /* module documentation, may be NULL */
    -1,          /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    pruserial485_funcs
};

PyMODINIT_FUNC PyInit_PRUserial485(void)
{
    return PyModule_Create(&cModPyDem);
}
