#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef enum pond_parameter_type
{
    POND_PARAMETER_ERROR,
    POND_PARAMETER_STRING,
    POND_PARAMETER_STRING_ARRAY,
    POND_PARAMETER_INT,
    POND_PARAMETER_INT_ARRAY,
    POND_PARAMETER_DOUBLE,
    POND_PARAMETER_DOUBLE_ARRAY,
    POND_PARAMETER_BOOL,
    POND_PARAMETER_BOOL_ARRAY
} pond_parameter_type;

static const char* pond_parameter_type_to_string(pond_parameter_type type)
{
    switch (type)
    {
        case POND_PARAMETER_STRING: return "POND_PARAMETER_STRING";
        case POND_PARAMETER_STRING_ARRAY: return "POND_PARAMETER_STRING_ARRAY";
        case POND_PARAMETER_INT: return "POND_PARAMETER_INT";
        case POND_PARAMETER_INT_ARRAY: return "POND_PARAMETER_INT_ARRAY";
        case POND_PARAMETER_DOUBLE: return "POND_PARAMETER_DOUBLE";
        case POND_PARAMETER_DOUBLE_ARRAY: return "POND_PARAMETER_DOUBLE_ARRAY";
        case POND_PARAMETER_BOOL: return "POND_PARAMETER_BOOL";
        case POND_PARAMETER_BOOL_ARRAY: return "POND_PARAMETER_BOOL_ARRAY";
        default: return "ERROR";
    }
    
}

typedef struct pond_parameter
{
    pond_parameter_type type;
    uint32_t array_length;
    union
    {
        uint8_t* String;
        uint8_t** StringArray;
        int Int;
        int* IntArray;
        double Double;
        double* DoubleArray;
        bool Bool;
        bool* BoolArray;
    } value;
} pond_parameter;

static pond_parameter* pond_malloc_parameter_int(int32_t value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_INT;
    p->value.Int = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_double(double value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_DOUBLE;
    p->value.Double = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_bool(bool value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_BOOL;
    p->value.Bool = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_string(uint8_t* value)
{
    size_t size = strlen((char*)value) + 1;
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + size);
    p->type = POND_PARAMETER_STRING;
    p->value.String = (uint8_t*)p + sizeof(pond_parameter);
    memcpy(p->value.String, value, size);
    return p;
}

static pond_parameter* pond_malloc_parameter_int_array(int32_t* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(int32_t) * length);
    p->type = POND_PARAMETER_INT_ARRAY;
    p->array_length = length;
    p->value.IntArray = (int32_t*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.IntArray, value, sizeof(int32_t) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_double_array(double* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(double) * length);
    p->type = POND_PARAMETER_DOUBLE_ARRAY;
    p->array_length = length;
    p->value.DoubleArray = (double*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.DoubleArray, value, sizeof(double) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_bool_array(bool* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(bool) * length);
    p->type = POND_PARAMETER_BOOL_ARRAY;
    p->array_length = length;
    p->value.BoolArray = (bool*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.BoolArray, value, sizeof(bool) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_string_array(uint8_t** value, uint32_t length)
{
    size_t total_size = 0;
    for (uint32_t i = 0; i < length; i++) total_size += strlen((char*)value[i]) + 1;

    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(uint8_t*) * length + total_size);
    p->type = POND_PARAMETER_STRING_ARRAY;
    p->array_length = length;
    p->value.StringArray = (uint8_t**)((uint8_t*)p + sizeof(pond_parameter));

    for (uint32_t i = 0, offset = 0; i < length; i++)
    {
        p->value.StringArray[i] = (uint8_t*)((uint8_t*)p + sizeof(pond_parameter) + sizeof(uint8_t*) * length + offset);
        size_t len = strlen((char*)value[i])+1;
        memcpy(p->value.StringArray[i], value[i], len);
        offset += len;
    }

    return p;
}