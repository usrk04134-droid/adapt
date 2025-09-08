/*****************************************************************************/
/*  Copyright (C) 2024 Siemens Aktiengesellschaft. All rights reserved.      */
/*****************************************************************************/
/*  This program is protected by German copyright law and international      */
/*  treaties. The use of this software including but not limited to its      */
/*  Source Code is subject to restrictions as agreed in the license          */
/*  agreement between you and Siemens.                                       */
/*  Copying or distribution is not allowed unless expressly permitted        */
/*  according to your license agreement with Siemens.                        */
/*****************************************************************************/
/*                                                                           */
/*  P r o j e c t         &P: PROFINET IO Runtime Software              :P&  */
/*                                                                           */
/*  P a c k a g e         &W: PROFINET IO Runtime Software              :W&  */
/*                                                                           */
/*  C o m p o n e n t     &C: PnDriver                                  :C&  */
/*                                                                           */
/*  F i l e               &F: pnd_trace_util.c                          :F&  */
/*                                                                           */
/*  V e r s i o n         &V: BC_PNDRIVER_V03.01.00.00_00.03.00.29      :V&  */
/*                                                                           */
/*  D a t e  (YYYY-MM-DD) &D: 2024-03-19                                :D&  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/*  D e s c r i p t i o n :                                                  */
/*                                                                           */
/* Trace logging helper functions for the Example Applications for PNDriver  */
/*                                                                           */
/*****************************************************************************/

// configure pntrc 
#define LTRC_ACT_MODUL_ID   0000
#define PND_MODULE_ID       0000

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pnd/common/pniobase.h>

#include "pnd_pntrc.h"

// only one of the mode (PNDTEST_USE_CIRCLE_TRACE or PNDTEST_USE_MULTI_TRACE_FILES) can be activated
#if defined PNDTEST_USE_CIRCLE_TRACE && defined PNDTEST_USE_MULTI_TRACE_FILES
#error use only one of the trace methods at a time
#endif

#if (defined PLF_PNDRIVER_LINUX)
#include <signal.h>
#include <pthread.h>      // used for locking
#elif defined PLF_PNDRIVER_WIN
#include <crtdbg.h>
#include <windows.h>      // used for locking and error handling
#endif


// Defines 
#define ONE_KBYTE (1024L)        // bytes
#define ONE_MBYTE (1048576L)     // (1024 * ONE_KBYTE)
#define ONE_GBYTE (1073741824L)  // (1024 * ONE_MBYTE)

// ATTENTION: The size of TRACE_CIRCLE_BUFFER_SIZE must be the same as TRACE_AREA_SIZE (in pntrc_circ_trace.h) minus 40 Bytes!
#define TRACE_CIRCLE_BUFFER_SIZE        (ONE_MBYTE + 40/*Bytes*/)   // 40Bytes for Circle Buffer Header
#define MAX_SIZE_OF_PND_TRACE_FILE       ONE_GBYTE 
#define MAX_NUM_OF_PND_TRACE_FILES       10


// Global Varibales
#if defined PLF_PNDRIVER_WIN
static const char *pnd_fatal_log_name = "pnd_fatal_log.txt";
#endif

#if defined PNDTEST_USE_CIRCLE_TRACE
PNIO_UINT8 trace_buffer[TRACE_CIRCLE_BUFFER_SIZE] = { 0 };  // RAM trace buffer
static const char  *pnd_trace_file_name_only = "pnd_trace";
#endif

#if defined PNDTEST_USE_MULTI_TRACE_FILES
static const char *pnd_trace_file_name = "pnd_trace.bin";
static FILE* PNTRC_fp = 0;
#endif

// extern declarations
extern PNIO_UINT32 g_ApplHandle;

/*---------------------------------------------------------------------------*/
/* Function implementations                                                  */
/*---------------------------------------------------------------------------*/
#if defined PNDTEST_USE_CIRCLE_TRACE // Circle Trace

void pnd_test_write_trace_buffer_to_filesystem(PNIO_UINT8 bIsFatal)
{
    static PNIO_UINT32 file_count = 0;
    char  file_name[2048];
    FILE* PNTRC_file;
    PNIO_UINT32 result = PNIO_OK;

    if (bIsFatal)
    {
        sprintf(file_name, "%s_%s%s", pnd_trace_file_name_only, "fatal", ".bin");
    }
    else
    {
        sprintf(file_name, "%s_%04d%s", pnd_trace_file_name_only, file_count, ".bin");
    }

    result = SERV_CP_set_trace_buffer((PNIO_UINT8*)&trace_buffer); 
    if (result != PNIO_OK)  
    {
       printf("\nCould not update tracebuffer \n");    
    }
    else
    {
       // open trace buffer file
       PNTRC_file = fopen(file_name, "wb");
       if (PNTRC_file != 0)
       {
           fwrite(trace_buffer, 1, TRACE_CIRCLE_BUFFER_SIZE, PNTRC_file);
           fclose(PNTRC_file);
           printf("\nTracefile '%s' generated\n\n", file_name);
           file_count++;
       }
       else
       {
           printf("\nCould not write trace file\n\n");
       }
    }
}

#elif defined PNDTEST_USE_MULTI_TRACE_FILES // start of ring buffer implemantation
/*---------------------------------------------------------------------------*/
long int get_file_size(FILE* fp)
{
    long int prev   = 0;
    long int retval = 0;

    prev = ftell(fp);
    fseek(fp, 0L, SEEK_END);
    retval = ftell(fp);
    fseek(fp, prev, SEEK_SET); // go back to where we were

    return retval;
}

/*---------------------------------------------------------------------------*/
int file_exists(const char* filename)
{
    // try to open file to read
    FILE* fp = PNIO_NULL;

    fp = fopen(filename, "r");
    if (fp == PNIO_NULL)
    {
        return 0;
    }

    fclose(fp);
    return 1;
}

/*---------------------------------------------------------------------------*/
void shift_pnd_trace_files(const char* filename)
{
    int i = 0;
    int k = 0;
    char file_name[2048];
    char file_new_name[2048];

    // check the last number of the log file exist
    for (i = 0; i < MAX_NUM_OF_PND_TRACE_FILES; i++)
    {
        sprintf(file_name, "%s_%02d", filename, (i + 1));

        // check if the file exist   
        if (file_exists(file_name))
        {
            continue; // File exists, chec next file
        }
        else
        {
            break;
        }
    }

    // shift the files
    for (k = i; k > 0; k--)
    {
        // construct current file name to be shift
        sprintf(file_name, "%s_%02d", filename, k);

        if (k == MAX_NUM_OF_PND_TRACE_FILES)
        {
            // remove the file
            remove(file_name);
            continue;
        }

        // construct new file name 
        sprintf(file_new_name, "%s_%02d", filename, (k + 1));
        rename(file_name, file_new_name);
    }

    sprintf(file_new_name, "%s_%02d", filename, 1);    
    rename(filename, file_new_name);

    // create an empty file
    {
        FILE* fp = PNIO_NULL;

        fp = fopen(filename, "wb");
        if (fp != PNIO_NULL)
        {
            fclose(fp);
        }
    }
}
#endif // end of ring buffer implementation

#if defined PLF_PNDRIVER_WIN
/*---------------------------------------------------------------------------*/
void pnd_create_win_fatal_log(char* fatal_reason)
{
    FILE* fp;
    fp = fopen(pnd_fatal_log_name, "w+");
    if(fp != PNIO_NULL)
    {
         fprintf(fp, "PND Fatal occured with unhandled exception...\n");
         fputs(fatal_reason, fp);
         fclose(fp);
    }
    else
    {
         printf("Error: File could not be opened");
         return;
    }
}

// Unhandled exception handler
LONG WINAPI pnd_unhandled_exception_filter(struct _EXCEPTION_POINTERS* ep)
{
    PNIO_UINT8 isFatalOccured = FALSE;
    if ((!isFatalOccured))   // First one ?
    {
        isFatalOccured = TRUE;
        char reason[2048];
        sprintf(reason, "Unhandled exception - Code(0x%08x) - Address(0x%08x)",
                ep->ExceptionRecord->ExceptionCode, (PNIO_UINT32)ep->ExceptionRecord->ExceptionAddress);

        printf("PND Fatal occured with unhandled exception...\n");
        printf("%s", reason);

        pnd_create_win_fatal_log(reason);

#if defined PNDTEST_USE_CIRCLE_TRACE
        pnd_test_write_trace_buffer_to_filesystem(isFatalOccured);
#endif // defined PNDTEST_USE_CIRCLE_TRACE
    }
    return (EXCEPTION_EXECUTE_HANDLER);  // exception handled only here !
}
#endif // This function only works in Windows

/*---------------------------------------------------------------------------*/
void pnd_test_buffer_full(const PNIO_UINT8* pBuffer, PNIO_UINT32 BufferSize)
{
#if defined PNDTEST_USE_MULTI_TRACE_FILES
    long int file_size = 0;

    if (PNTRC_fp != 0)
    {
        fwrite(pBuffer, 1, BufferSize, PNTRC_fp);
        fflush(PNTRC_fp);

        if (MAX_NUM_OF_PND_TRACE_FILES != 0 && MAX_NUM_OF_PND_TRACE_FILES != 1) {

            file_size = get_file_size(PNTRC_fp);
            if (file_size >= MAX_SIZE_OF_PND_TRACE_FILE)
            {
                fclose(PNTRC_fp);
                PNTRC_fp = PNIO_NULL;

                /*
                 * MAX_NUM_OF_PND_TRACE_FILES defines maximum number of "pnd_trace.bin" files that will be preserved for instance if 
                 * MAX_NUM_OF_PND_TRACE_FILES defined to be as 3, in a specific moment of time there will 4 instance of log files that
                 * is, "pnd_trace.bin", "pnd_trace.bin_01", "pnd_trace.bin_02" and "pnd_trace.bin_03"  in this configuration the current
                 * file where the traces are logged will be file "pnd_trace.bin", and the oldest trace file will  be "pnd_trace.bin_03" 
                 * when the size of file "pnd_trace.bin" exceeds MAX_SIZE_OF_PND_TRACE_FILE, file "pnd_trace.bin_03" will be deleted, file
                 * "pnd_trace.bin_02" will be renamed as "pnd_trace.bin_03" file "pnd_trace.bin_01" will be renamed as "pnd_trace.bin_02",
                 * file "pnd_trace.bin" will be renamed as "pnd_trace.bin_01". file "pnd_trace.bin" will be reopened with size 0 and new 
                 * traceses will continue to be logged to "pnd_trace.bin"
                 */
                shift_pnd_trace_files(pnd_trace_file_name);

                //open trace buffer file
                PNTRC_fp = fopen(pnd_trace_file_name, "wb");
            }
        }
    }
#endif
}

/*---------------------------------------------------------------------------*/
void InitTraceParams(PNIO_DEBUG_SETTINGS_TYPE* deb_set_ptr)
{
    memset(deb_set_ptr, 0, sizeof(PNIO_DEBUG_SETTINGS_TYPE));

#if defined PLF_PNDRIVER_WIN
    SetUnhandledExceptionFilter(pnd_unhandled_exception_filter);
#endif

#if defined PNDTEST_USE_CIRCLE_TRACE

    debSetPtr->CbfPntrcWriteBuffer = pnd_test_write_trace_buffer_to_filesystem;
#elif defined PNDTEST_USE_MULTI_TRACE_FILES
    //open trace buffer file
    PNTRC_fp = fopen(pnd_trace_file_name, "wb");
    deb_set_ptr->CbfPntrcBufferFull = pnd_test_buffer_full;
#endif
}

/*---------------------------------------------------------------------------*/
void ReleaseTraceResources(void)
{
#if defined PNDTEST_USE_MULTI_TRACE_FILES
    if (PNTRC_fp)
    {
        fclose(PNTRC_fp);
    }
#endif
}

/*****************************************************************************/
/*  Copyright (C) 2024 Siemens Aktiengesellschaft. All rights reserved.      */
/*****************************************************************************/
