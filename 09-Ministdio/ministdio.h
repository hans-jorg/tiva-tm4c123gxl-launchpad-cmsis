#ifndef MINISTIO_H
#define MINISTIO_H
/**
 * @file  ministdio.h
 * @brief Routines for minimal stdio support
 *
 * @note  It uses getchar and putchar functions defined by application
 *
 **/

int printf(const char *fmt, ...);
int puts(const char *s);
int fputs(const char *s, void *ignored);
char *fgets(char *s, int n, void *ignored);

#define stdin  0
#define stdout 0
#define stderr 0

#endif
