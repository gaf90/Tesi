#ifndef WINBACKTRACE_H
#define WINBACKTRACE_H

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
    void *address;
    char *func_name;
    char *file_name;
    int line;
} backtrace_entry_t;

typedef struct {
    int count;
    backtrace_entry_t *entries;
} backtrace_result_t;

void backtrace(backtrace_result_t *result, int depth);
void backtrace_with_context(backtrace_result_t *result, int depth, LPCONTEXT context);
void backtrace_free_result(backtrace_result_t *result);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* WINBACKTRACE_H */
