//
// Created by fraser on 19/02/18.
//
#include "packet_constructor.h"
#include "packet_structure.h"

void reverse_array(unsigned char* a, int array_length) {
    int i = array_length - 1;
    int j = 0;
    int temp = 0;

    while(i > j)
    {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
        i--;
        j++;
    }
}