// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#ifndef MYTIME_H
#define MYTIME_H

#ifdef __linux__
#include <sys/time.h>
#else

#define WIN32_BARE_BONES
#include <Windows.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64 

int gettimeofday(struct timeval * tp, struct timezone * tzp);
#endif

#endif
