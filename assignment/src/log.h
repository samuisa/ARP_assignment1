// log.h
#ifndef LOG_H
#define LOG_H

// Scrive un messaggio nel file di log specificato (in append) con timestamp
void logMessage(const char *filename, const char *format, ...);

#endif
