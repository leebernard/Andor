#ifndef FITSIMAGE_H

#include "andorproto.h"

/* structure to hold a #define macro value and its stringified name */
struct numandstring {
    int         num;
    char *      string;
};

// macro to create value of a struct numandstring using value and stringification
#define makenumandstring(hashdefine)    { hashdefine, (char *)#hashdefine}

#define FITSIMAGE_H
#endif /* FITSIMAGE_H */
