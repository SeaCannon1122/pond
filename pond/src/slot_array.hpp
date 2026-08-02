#pragma once
#include <cstdlib>
#include <stdint.h>
#include <stdlib.h>

template <typename T>
struct _SlotArrayElement
{
    bool in_use;
    T element;
};

template <typename T>
class SlotArray
{
public:
    void create(uint32_t cap)
    {
        capacity = cap;
        elements = (_SlotArrayElement<T>*)malloc(sizeof(_SlotArrayElement<T>) * cap);
        for (uint32_t i = 0; i < capacity; i++) elements[i].in_use = false;
        free_index = 0;
    }

    void destroy()
    {
        free(elements);
    }

    T& operator[](uint32_t index)
    {
        return elements[index].element;
    }

    uint32_t get_slot()
    {
        if (free_index == capacity)
        {
            capacity *= 2;
            elements = realloc(elements, capacity);
            for (uint32_t i = free_index; i < capacity; i++) elements[i].in_use = false;
        }

        uint32_t index = free_index;
        elements[free_index].in_use = true;
        while (elements[free_index].in_use && free_index < capacity) free_index++;

        return index;
    }

    void release_slot(uint32_t slot)
    {
        elements[slot].in_use = false;
        if (free_index > slot) free_index = slot;
    }

    uint32_t get_length() { return capacity; }

    bool is_used(uint32_t slot) { return elements[slot].in_use;}

private:
    uint32_t capacity;  
    _SlotArrayElement<T>* elements;
    uint32_t free_index;
};