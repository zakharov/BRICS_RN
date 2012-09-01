/* 
 * File:   logger.h
 * Author: alexey
 *
 * Created on August 29, 2012, 3:20 PM
 */

#ifndef LOGGER_H
#define	LOGGER_H

#include <cstdio>

#ifdef DEBUG
#define LOG(format, args...) { fprintf (stderr, format, ##args); fprintf (stderr, "\n"); }
#else
#define LOG(format, args...);
#endif

#endif	/* LOGGER_H */

