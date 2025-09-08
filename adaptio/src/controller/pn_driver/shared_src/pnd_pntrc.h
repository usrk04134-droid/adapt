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
/*  F i l e               &F: pnd_pntrc.h                               :F&  */
/*                                                                           */
/*  V e r s i o n         &V: BC_PNDRIVER_V03.01.00.00_00.03.00.29      :V&  */
/*                                                                           */
/*  D a t e  (YYYY-MM-DD) &D: 2024-03-19                                :D&  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/*  D e s c r i p t i o n :                                                  */
/*                                                                           */
/*  PNTrace functions declarations for the Example Applications for PNDriver */
/*                                                                           */
/*****************************************************************************/

#include <pnd/common/pniobase.h>
#include <pnd/common/servusrx.h>

#define PND_TEST_UNUSED_ARG(arg_) \
  {                               \
    (void)(arg_);                 \
  }

/**************************** TRACING ****************************************/
/* use only on of the trace methods at the same time                         */
/* use circle trace: trace will be held in RAM, until user writes it with menu to disk or a FATAL/exception occurs  */
/* #define PNDTEST_USE_CIRCLE_TRACE                                                                                 */
/* use multi trace files: trace will be written to files as PNRUN buffer get full, when the size of the file exceed */
/* the specified size a new file will be opened                                                                     */
#define PNDTEST_USE_MULTI_TRACE_FILES

void PndTestBufferFull(const PNIO_UINT8* p_buffer, PNIO_UINT32 buffer_size);

#ifdef __cplusplus
extern "C" {
#endif

// trace helper functions
void InitTraceParams(PNIO_DEBUG_SETTINGS_TYPE* deb_set_ptr);
void ReleaseTraceResources(void);
#ifdef PNDTEST_USE_CIRCLE_TRACE
void pnd_test_write_trace_buffer_to_filesystem(PNIO_UINT8 bIsFatal);
#endif

#ifdef __cplusplus
}
#endif

/*****************************************************************************/
/*  Copyright (C) 2024 Siemens Aktiengesellschaft. All rights reserved.      */
/*****************************************************************************/
