// log.c
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

void logMessage(const char *filename, const char *format, ...) {
    // Assicura che la directory logs/ esista
    mkdir("logs", 0777);

    FILE *fp = fopen(filename, "a");
    if (!fp) return;

    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    char timebuf[26];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", tm_info);

    pid_t pid = getpid();

    // Scrivi solo nel file
    fprintf(fp, "[%s] (PID %d) ", timebuf, pid);

    va_list args;
    va_start(args, format);
    vfprintf(fp, format, args);
    va_end(args);

    fprintf(fp, "\n");

    fclose(fp);
}
