// ------------------------------------------------------------------------------------------------
// Python Module for PRUserial485 Interface
//
// Mapping .c functions into Python methods in order to ease code development without losing
// performance.
//
// CONTROLS GROUP
// Author: Patricia Nallin (patricia.nallin@lnls.br)
// Release Date: May 18, 2020
//
// ------------------------------------------------------------------------------------------------

#include <Python.h>
#include <stdio.h>
#include <prussdrv.h>
#include <PRUserial485.h>


// ------------------------------------------------------------------------------------------------
// int PRUserial485_address() -> int hardware_address_serialPRU()
// ------------------------------------------------------------------------------------------------
static PyObject* pru_address(PyObject* self, PyObject *args)
{
        return Py_BuildValue("i", hardware_address_serialPRU());
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_clear_pulse_count_sync() -> void clear_pulse_count_sync()
// ------------------------------------------------------------------------------------------------
PyObject* pru_clear_pulse_count_sync(PyObject* self, PyObject *args)
{
    clear_pulse_count_sync();
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// uint32_t PRUserial485_read_pulse_count_sync() -> void read_pulse_count_sync()
// ------------------------------------------------------------------------------------------------
PyObject* pru_read_pulse_count_sync(PyObject* self, PyObject *args)
{
    return Py_BuildValue("l", read_pulse_count_sync());
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_set_curve_pointer(point) -> void set_curve_pointer(uint32_t point)
// ------------------------------------------------------------------------------------------------
PyObject* pru_set_curve_pointer(PyObject* self, PyObject *args)
{
    uint32_t pointer;

 	if (!PyArg_ParseTuple(args, "l", &pointer))
    {
 		return NULL;
 	}
    set_curve_pointer(pointer);
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_read_curve_pointer() -> uint32_t read_curve_pointer()
// ------------------------------------------------------------------------------------------------
PyObject* pru_read_curve_pointer(PyObject* self, PyObject *args)
{
    uint32_t pointer = read_curve_pointer();
    return Py_BuildValue("l", pointer);
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_sync_status() -> int sync_status()
// ------------------------------------------------------------------------------------------------
PyObject* pru_sync_status(PyObject* self, PyObject *args)
{
     return Py_BuildValue("i", sync_status());
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_sync_stop() -> void set_sync_stop_PRU()
// ------------------------------------------------------------------------------------------------
PyObject* pru_sync_stop(PyObject* self, PyObject *args)
{
    set_sync_stop_PRU();
     return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_sync_start(mode, delay, addr) -> void set_sync_start_PRU(uint8_t mode, uint32_t delay, uint8_t addr)
// ------------------------------------------------------------------------------------------------
PyObject* pru_sync_start(PyObject* self, PyObject *args)
{
    uint8_t mode, address;
    uint32_t delay;

 	if (!PyArg_ParseTuple(args, "ili", &mode, &delay, &address))
    {
 		return NULL;
 	}
    set_sync_start_PRU(mode, delay, address);
    return Py_BuildValue("s", NULL);
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
// int PRUserial485_curve(int block, [[floats1],[floats2],[floats3],[floats4]]) ->
// int loadCurve(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t CurvePoints, uint8_t block)
// ------------------------------------------------------------------------------------------------
PyObject* pru_load_curve_block(PyObject* self, PyObject *args)
{
    uint8_t block = 0;
    uint32_t curve_size = 0;
    float curves[4][6250];
    PyObject* lists;

    if (!PyArg_ParseTuple(args, "iO!", &block, &PyList_Type, &lists))
    return NULL;

    Py_ssize_t list_size = PyList_Size(lists);

    for (Py_ssize_t i = 0; i < list_size; i++)
    {
        PyObject* sublist = PyList_GetItem(lists, i);

        if (!PyList_Check(sublist))
        {
            PyErr_SetString(PyExc_TypeError, "List must contain lists");
            return NULL;
        }

        Py_ssize_t sublist_size = PyList_Size(sublist);
        curve_size = sublist_size;

        for (Py_ssize_t j = 0; j < sublist_size; j++)
        {
            curves[i][j] = PyFloat_AsDouble(PyList_GetItem(sublist, j));
        }
    }

    return Py_BuildValue("i", loadCurve(curves[0], curves[1], curves[2], curves[3], curve_size, block));
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_set_curve_block(block) -> void set_curve_block(uint8_t block)
// ------------------------------------------------------------------------------------------------
PyObject* pru_set_curve_block(PyObject* self, PyObject *args)
{
    int block;

 	if (!PyArg_ParseTuple(args, "i", &block))
    {
 		return NULL;
 	}
    set_curve_block(block);
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_read_curve_block() -> uint8_t read_curve_block()
// ------------------------------------------------------------------------------------------------
PyObject* pru_read_curve_block(PyObject* self, PyObject *args)
{
    uint8_t block = read_curve_block();
    return Py_BuildValue("i", block);
}



// ---------------------------------------------------------------------------$
// int PRUserial485_shram(int offset) -> int read_shram(uint16_t offset)
// ---------------------------------------------------------------------------$
PyObject* pru_read_shram(PyObject* self, PyObject *args)
{
        int offst;

        if (!PyArg_ParseTuple(args, "i", &offst))
    {
                return NULL;
        }

    return Py_BuildValue("i", read_shram(offst));
}



// ---------------------------------------------------------------------------$
// void PRUserial485_shram_write(int offset, uint8_t value) -> void write_shram(uint16_t offset, uint8_t value)
// ---------------------------------------------------------------------------$
PyObject* pru_write_shram(PyObject* self, PyObject *args)
{
        int offst;
        int value = 0;

        if (!PyArg_ParseTuple(args, "ii", &offst, &value)){
                return NULL;
        }
        write_shram(offst, value);

    return Py_BuildValue("s", NULL);
}






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
// bytes PRUserial485_read(uint32_t bytes2read = 0) -> int recv_data_PRU(char array, uint32 length, uint32_t bytes2read)
// ------------------------------------------------------------------------------------------------
PyObject* pru_recv(PyObject* self, PyObject *args)
{
    uint32_t data_size;
    uint8_t data[100000];
    uint32_t bytes2read = 0;

    if (!PyArg_ParseTuple(args, "|l", &bytes2read))
    {
        PyErr_SetString(PyExc_TypeError, "Bytes to read must be a positive int value");
        return NULL;
    }
    

    recv_data_PRU(data, &data_size, bytes2read);

    return Py_BuildValue("y#", data, data_size);
}



// ------------------------------------------------------------------------------------------------
// int PRUserial485_read_flush() -> int recv_flush()
// ------------------------------------------------------------------------------------------------
PyObject* pru_recv_flush(PyObject* self, PyObject *args)
{
     return Py_BuildValue("i", recv_flush());
}



// ------------------------------------------------------------------------------------------------
// str __version__()
// ------------------------------------------------------------------------------------------------
static PyObject* pru_version(PyObject* self, PyObject *args)
{
    return Py_BuildValue("s", VERSIONHASH);
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_ff_configure(id_type, n_tables, max_range) -> int ff_configure(uint8_t id_type, uint8_t n_tables, float max_range)
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_configure(PyObject* self, PyObject *args)
{
    uint8_t id_type, n_tables;
    float max_range;

 	if (!PyArg_ParseTuple(args, "iif", &id_type, &n_tables, &max_range))
    {
 		return NULL;
 	}
    int ret = ff_configure(id_type, n_tables, max_range);
    
    return Py_BuildValue("i", ret);
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_ff_enable() -> void ff_enable()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_enable(PyObject* self, PyObject *args)
{
    ff_enable();
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_ff_disable() -> void ff_disable()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_disable(PyObject* self, PyObject *args)
{
    ff_disable();
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_ff_status() -> int ff_get_status()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_status(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_get_status());
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_ff_get_table_size() -> int ff_get_table_size()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_get_table_size(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_get_table_size());
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_ff_load_table(int table, [[floats1],[floats2],[floats3],[floats4]]) ->
// int ff_load_table(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t table_points, uint8_t table)
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_load_table(PyObject* self, PyObject *args)
{
    uint8_t table = 0;
    uint32_t table_points = 0;
    float curves[4][37500];
    PyObject* lists;

    if (!PyArg_ParseTuple(args, "iO!", &table, &PyList_Type, &lists))
    return NULL;

    Py_ssize_t list_size = PyList_Size(lists);

    for (Py_ssize_t i = 0; i < list_size; i++)
    {
        PyObject* sublist = PyList_GetItem(lists, i);

        if (!PyList_Check(sublist))
        {
            PyErr_SetString(PyExc_TypeError, "List must contain lists");
            return NULL;
        }

        Py_ssize_t sublist_size = PyList_Size(sublist);
        table_points = sublist_size;

        for (Py_ssize_t j = 0; j < sublist_size; j++)
        {
            curves[i][j] = PyFloat_AsDouble(PyList_GetItem(sublist, j));
        }
    }

    return Py_BuildValue("i", ff_load_table(curves[0], curves[1], curves[2], curves[3], table_points, table));
}


// ------------------------------------------------------------------------------------------------
// list(float) PRUserial485_ff_read_table(int table) ->
// int ff_read_table(float *curve1, float *curve2, float *curve3, float *curve4, uint32_t table_points, uint8_t table)
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_read_table(PyObject* self, PyObject *args)
{
    uint8_t table = 0;
    uint32_t table_points = 0;
    float curves[4][37500];
    PyObject* return_list = PyList_New(4);

    if (!PyArg_ParseTuple(args, "i", &table))
        return NULL;

    table_points = ff_read_table(curves[0], curves[1], curves[2], curves[3], table);
    if(table_points == ERR_CURVE_OVER_BLOCK)
        return NULL;

    PyObject* curve = PyList_New(table_points);

    for(int j=0; j<4; j++ ){
        for(int i=0; i<table_points; i++)
        {
            PyList_SetItem(curve, i, Py_BuildValue("f", curves[j][i]));
        }
        PyList_SetItem(return_list, j, curve);
    }

    return return_list;
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_ff_read_current_table() -> uint8_t ff_read_current_table()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_read_current_table(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_read_current_table());
}


// ------------------------------------------------------------------------------------------------
// int pru_ff_read_current_pointer() -> uint8_t ff_read_current_pointer()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_read_current_pointer(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_read_current_pointer());
}

// ------------------------------------------------------------------------------------------------
// int pru_ff_read_current_position() -> int ff_read_current_position()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_read_current_position(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_read_current_position());
}


// ------------------------------------------------------------------------------------------------
// int PRUserial485_ff_read_flags -> uint8_t ff_read_flags()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_read_flags(PyObject* self, PyObject *args)
{
    return Py_BuildValue("i", ff_read_flags());
}


// ------------------------------------------------------------------------------------------------
// void PRUserial485_ff_clear_flags() -> void ff_clear_flags()
// ------------------------------------------------------------------------------------------------
PyObject* pru_ff_clear_flags(PyObject* self, PyObject *args)
{
    ff_clear_flags();
    return Py_BuildValue("s", NULL);
}


// ------------------------------------------------------------------------------------------------
// Python Module definitions, methods and initialization
// ------------------------------------------------------------------------------------------------
static PyMethodDef pruserial485_funcs[] = {
    {"PRUserial485_address",                 (PyCFunction)pru_address,                  METH_VARARGS, NULL},
    {"PRUserial485_clear_pulse_count_sync",  (PyCFunction)pru_clear_pulse_count_sync,   METH_VARARGS, NULL},
    {"PRUserial485_read_pulse_count_sync",   (PyCFunction)pru_read_pulse_count_sync,    METH_VARARGS, NULL},
    {"PRUserial485_set_curve_pointer",       (PyCFunction)pru_set_curve_pointer,        METH_VARARGS, NULL},
    {"PRUserial485_read_curve_pointer",      (PyCFunction)pru_read_curve_pointer,       METH_VARARGS, NULL},
    {"PRUserial485_sync_status",             (PyCFunction)pru_sync_status,              METH_VARARGS, NULL},
    {"PRUserial485_sync_start",              (PyCFunction)pru_sync_start,               METH_VARARGS, NULL},
    {"PRUserial485_sync_stop",               (PyCFunction)pru_sync_stop,                METH_VARARGS, NULL},
    {"PRUserial485_close",                   (PyCFunction)pru_close,                    METH_VARARGS, NULL},
    {"PRUserial485_curve",                   (PyCFunction)pru_load_curve_block,         METH_VARARGS, NULL},
    {"PRUserial485_set_curve_block",         (PyCFunction)pru_set_curve_block,          METH_VARARGS, NULL},
    {"PRUserial485_read_curve_block",        (PyCFunction)pru_read_curve_block,         METH_VARARGS, NULL},
    {"PRUserial485_open",                    (PyCFunction)pru_open,                     METH_VARARGS, NULL},
    {"PRUserial485_write",                   (PyCFunction)pru_send,                     METH_VARARGS, NULL},
    {"PRUserial485_read",                    (PyCFunction)pru_recv,                     METH_VARARGS, NULL},
    {"PRUserial485_read_flush",              (PyCFunction)pru_recv_flush,               METH_VARARGS, NULL},
    {"PRUserial485_shram", 		             (PyCFunction)pru_read_shram,               METH_VARARGS, NULL},
    {"PRUserial485_write_shram",	         (PyCFunction)pru_write_shram,              METH_VARARGS, NULL},
    {"PRUserial485_ff_configure",	         (PyCFunction)pru_ff_configure,             METH_VARARGS, NULL},
    {"PRUserial485_ff_enable",	             (PyCFunction)pru_ff_enable,                METH_VARARGS, NULL},
    {"PRUserial485_ff_disable",	             (PyCFunction)pru_ff_disable,               METH_VARARGS, NULL},
    {"PRUserial485_ff_status",	             (PyCFunction)pru_ff_status,                METH_VARARGS, NULL},
    {"PRUserial485_ff_get_table_size",	     (PyCFunction)pru_ff_get_table_size,        METH_VARARGS, NULL},
    {"PRUserial485_ff_load_table",	         (PyCFunction)pru_ff_load_table,            METH_VARARGS, NULL},
    {"PRUserial485_ff_read_table",	         (PyCFunction)pru_ff_read_table,            METH_VARARGS, NULL},
    {"PRUserial485_ff_read_current_table",	 (PyCFunction)pru_ff_read_current_table,    METH_VARARGS, NULL},
    {"PRUserial485_ff_read_current_pointer", (PyCFunction)pru_ff_read_current_pointer,  METH_VARARGS, NULL},
    {"PRUserial485_ff_read_current_position",(PyCFunction)pru_ff_read_current_position, METH_VARARGS, NULL},
    {"PRUserial485_ff_read_flags",           (PyCFunction)pru_ff_read_flags,            METH_VARARGS, NULL},
    {"PRUserial485_ff_clear_flags",          (PyCFunction)pru_ff_clear_flags,           METH_VARARGS, NULL},
    {"__version__",                          (PyCFunction)pru_version,                  METH_VARARGS, NULL},
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
