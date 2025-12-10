#ifndef PANIC_H
#define PANIC_H

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PANIC(messageFormat, ...) panic(__FILENAME__, __LINE__, messageFormat, ##__VA_ARGS__)

extern void panic(const char *file, int line, const char *messageFormat, ...);

#endif