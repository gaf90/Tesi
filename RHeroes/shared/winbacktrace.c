/*
   Based on:
    http://code.google.com/p/backtrace-mingw/

----------------------------------------------------------------------------------------------------

    Copyright (c) 2010 ,
         Cloud Wu . All rights reserved.
 
         http://www.codingnow.com
 
     Use, modification and distribution are subject to the "New BSD License"
     as listed at <url: http://www.opensource.org/licenses/bsd-license.php >.
 
   filename: backtrace.c

   compiler: gcc 3.4.5 (mingw-win32)

   build command: gcc -O2 -shared -Wall -o backtrace.dll backtrace.c -lbfd -liberty -limagehlp 

   how to use: Call LoadLibraryA("backtrace.dll"); at beginning of your program .

  */

#include "winbacktrace.h"
#include <excpt.h>
#include <imagehlp.h>
#include <bfd.h>
#include <psapi.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#define MAX_NAME_BUFFER 256

struct bfd_ctx {
    bfd * handle;
    asymbol ** symbol;
};

struct bfd_set {
    char * name;
    struct bfd_ctx * bc;
    struct bfd_set *next;
};

struct find_info {
    asymbol **symbol;
    bfd_vma counter;
    const char *file;
    const char *func;
    unsigned line;
};

static void lookup_section(bfd *abfd, asection *sec, void *opaque_data)
{
    struct find_info *data = opaque_data;

    if (data->func)
        return;

    if (!(bfd_get_section_flags(abfd, sec) & SEC_ALLOC))
        return;

    bfd_vma vma = bfd_get_section_vma(abfd, sec);
    if (data->counter < vma || vma + bfd_get_section_size(sec) <= data->counter)
        return;

    bfd_find_nearest_line(
                abfd, sec, data->symbol, data->counter - vma,
                &(data->file), &(data->func), &(data->line));
}

static void find(
        struct bfd_ctx * b, DWORD offset, const char **file, const char **func, int *line)
{
    struct find_info data;
    data.func = NULL;
    data.symbol = b->symbol;
    data.counter = offset;
    data.file = NULL;
    data.func = NULL;
    data.line = 0;

    bfd_map_over_sections(b->handle, &lookup_section, &data);
    if (file) {
        *file = data.file;
    }
    if (func) {
        *func = data.func;
    }
    if (line) {
        *line = data.line;
    }
}

static int init_bfd_ctx(struct bfd_ctx *bc, const char * procname)
{
    bc->handle = NULL;
    bc->symbol = NULL;

    bfd *b = bfd_openr(procname, 0);
    if (!b) return 1;

    int r1 = bfd_check_format(b, bfd_object);
    int r2 = bfd_check_format_matches(b, bfd_object, NULL);
    int r3 = bfd_get_file_flags(b) & HAS_SYMS;

    if (!(r1 && r2 && r3)) {
        bfd_close(b);
        return 1;
    }

    void *symbol_table;

    unsigned dummy = 0;
    if (bfd_read_minisymbols(b, FALSE, &symbol_table, &dummy) == 0) {
        if (bfd_read_minisymbols(b, TRUE, &symbol_table, &dummy) < 0) {
            free(symbol_table);
            bfd_close(b);
            return 1;
        }
    }

    bc->handle = b;
    bc->symbol = symbol_table;

    return 0;
}

static void close_bfd_ctx(struct bfd_ctx *bc)
{
    if (bc) {
        if (bc->symbol) {
            free(bc->symbol);
        }
        if (bc->handle) {
            bfd_close(bc->handle);
        }
    }
}

static struct bfd_ctx *get_bc(struct bfd_set *set , const char *procname)
{
    while(set->name) {
        if (strcmp(set->name, procname) == 0) {
            return set->bc;
        }
        set = set->next;
    }

    struct bfd_ctx bc;
    if (init_bfd_ctx(&bc, procname)) {
        return NULL;
    }
    set->next = calloc(1, sizeof(*set));
    set->bc = malloc(sizeof(struct bfd_ctx));
    memcpy(set->bc, &bc, sizeof(bc));
    set->name = strdup(procname);

    return set->bc;
}

static void release_set(struct bfd_set *set)
{
    while(set) {
        struct bfd_set * temp = set->next;
        free(set->name);
        close_bfd_ctx(set->bc);
        free(set);
        set = temp;
    }
}

void backtrace_with_context(backtrace_result_t *result, int depth, LPCONTEXT context)
{
    if (!SymInitialize(GetCurrentProcess(), 0, TRUE)) {
        result->count = 0;
    } else {
        bfd_init();
        struct bfd_set *set = calloc(1, sizeof(struct bfd_set));
        struct bfd_ctx *bc = NULL;

        STACKFRAME frame;
        memset(&frame, 0, sizeof(frame));

        result->entries = malloc(depth * sizeof(backtrace_entry_t));

        frame.AddrPC.Offset = context->Eip;
        frame.AddrPC.Mode = AddrModeFlat;
        frame.AddrStack.Offset = context->Esp;
        frame.AddrStack.Mode = AddrModeFlat;
        frame.AddrFrame.Offset = context->Ebp;
        frame.AddrFrame.Mode = AddrModeFlat;

        HANDLE process = GetCurrentProcess();
        HANDLE thread = GetCurrentThread();

        char symbol_buffer[sizeof(IMAGEHLP_SYMBOL) + MAX_NAME_BUFFER];
        char module_name[MAX_PATH];
        int count = 0;

        while(StackWalk(
                  IMAGE_FILE_MACHINE_I386, process, thread, &frame, context, 0,
                  SymFunctionTableAccess, SymGetModuleBase, 0) && count < depth) {

            IMAGEHLP_SYMBOL *symbol = (IMAGEHLP_SYMBOL *) symbol_buffer;
            symbol->SizeOfStruct = (sizeof *symbol) + MAX_NAME_BUFFER;
            symbol->MaxNameLength = MAX_NAME_BUFFER - 1;

            DWORD module_base = SymGetModuleBase(process, frame.AddrPC.Offset);

            if(module_base &&
                GetModuleFileNameA((HINSTANCE) module_base, module_name, MAX_PATH)) {
                bc = get_bc(set, module_name);
            }

            const char * file = NULL;
            const char * func = NULL;
            int line = 0;

            if(bc) {
                find(bc, frame.AddrPC.Offset, &file, &func, &line);
            }

            if(func == NULL) {
                DWORD dummy = 0;
                if(SymGetSymFromAddr(process, frame.AddrPC.Offset, &dummy, symbol)) {
                    func = symbol->Name;
                }
            }

            result->entries[count].address = (void *) frame.AddrPC.Offset;
            result->entries[count].line = line;
            if(file != NULL) {
                result->entries[count].file_name = strdup(file);
            } else {
                result->entries[count].file_name = NULL;
            }
            if(func != NULL) {
                result->entries[count].func_name = strdup(func);
            } else {
                result->entries[count].func_name = NULL;
            }

            count++;
        }

        result->count = count;
        release_set(set);

        SymCleanup(GetCurrentProcess());
    }
}

void backtrace(backtrace_result_t *result, int depth)
{
    CONTEXT context;
    memset(&context, 0, sizeof(CONTEXT));
    context.ContextFlags = CONTEXT_FULL;
    GetThreadContext(GetCurrentThread(), &context);
    backtrace_with_context(result, depth, &context);
}

void backtrace_free_result(backtrace_result_t *result)
{
    int i;
    for(i = 0; i < result->count; i++) {
        if(result->entries[i].func_name)
            free(result->entries[i].func_name);
        if(result->entries[i].file_name)
            free(result->entries[i].file_name);
    }
    free(result->entries);
}
