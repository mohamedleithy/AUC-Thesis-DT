/*******************************************************************************
 * (c)Copyright 2019 by Siemens Industry Software and Services B.V.
 * All rights reserved.
 *
 * Simcenter Tire(r) has been developed at Siemens Industry Software and 
 * Services B.V.
 *
 * This document contains proprietary and confidential information of Siemens.
 * The contents of this document may not be disclosed to third parties, copied 
 * or duplicated in any form, in whole or in part, without prior written
 * permission of Siemens.
 *
 * The terms and conditions governing the license of Simcenter Tire(r)
 * software consist solely of those set forth in the written contracts between
 * Siemens or Siemens authorized third parties and its customers. The software 
 * may only be used or copied in accordance with the terms of these contracts.
 *******************************************************************************/
#ifndef MFS_API_H
#define MFS_API_H

#if defined(EXPORT_MFS_API)
  #if (defined(_WIN32))
    #define MFS_API __declspec(dllexport)
  #elif (defined(__linux__) && (__GNUC__ >= 4))
    #define MFS_API __attribute__ ((visibility ("default")))
  #else
    #define MFS_API
  #endif
#elif defined(IMPORT_MFS_API)
  #if (defined(_WIN32))
    #define MFS_API __declspec(dllimport)
  #else
    #define MFS_API
  #endif
#else
  #define MFS_API
#endif

#if defined(_WIN32)
  #define MFS_CALL __cdecl
#else
  #define MFS_CALL
#endif

#endif /* MFS_API_H */
