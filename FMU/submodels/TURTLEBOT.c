/*
 * Simcenter Amesim submodel used to import an FMU for co-simulation 2.0
 *
 * Copyright (c) 2015-2018 Siemens Industry Software NV
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ameutils.h"


#define _SUBMODELNAME_ "TURTLEBOT"

/* >>>>>>>>>>>>Insert Private Code Here. */
/* FMI for co-simulation  version   : 2.0
   Model name                       : AGV_turtle_electric_drive_Prescan_export
   Model identifier                 : AGV_turtle_electric_drive_Prescan_export
   GUID                             : 237474235
   Description                      : Simcenter Amesim model with FMI for co-simulation interface
   Generation tool                  : Simcenter Amesim 20211..
   Date of external model creation  : 2022-01-31T18:07:22Z
   Date of submodel generation      : 2023-01-24 17:14:36
*/


#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <float.h>
#ifdef WIN32
#define _WIN32_WINNT 0x0502 
#include <windows.h> 
#endif

static void *fmu_handle = NULL;  /* Handle to the FMU DLL */
static int fmu_nb_instance = 0;  /* Number of instantiate slave */

/* Type definitions of variables passed as arguments
   Version "default" means:

   fmi2Component           : an opaque object pointer
   fmi2ComponentEnvironment: an opaque object pointer
   fmi2FMUstate            : an opaque object pointer
   fmi2ValueReference      : handle to the value of a variable
   fmi2Real                : double precision floating-point data type
   fmi2Integer             : basic signed integer data type
   fmi2Boolean             : basic signed integer data type
   fmi2Char                : character data type
   fmi2String              : a pointer to a vector of fmi2Char characters
                             ('\0' terminated, UTF8 encoded)
   fmi2Byte                : smallest addressable unit of the machine, typically one byte.
*/
typedef void*           fmi2Component;               /* Pointer to FMU instance       */
typedef void*           fmi2ComponentEnvironment;    /* Pointer to FMU environment    */
typedef void*           fmi2FMUstate;                /* Pointer to internal FMU state */
typedef unsigned int    fmi2ValueReference;
typedef double          fmi2Real   ;
typedef int             fmi2Integer;
typedef int             fmi2Boolean;
typedef char            fmi2Char;
typedef const fmi2Char* fmi2String;
typedef char            fmi2Byte;

/* Values for fmi2Boolean  */
#define fmi2True  1
#define fmi2False 0

/* make sure all compiler use the same alignment policies for structures */
#ifdef WIN32
#pragma pack(push,8)
#endif

/* Include stddef.h, in order that size_t etc. is defined */
#include <stddef.h>


/* Type definitions */
typedef enum {
    fmi2OK,
    fmi2Warning,
    fmi2Discard,
    fmi2Error,
    fmi2Fatal,
    fmi2Pending
} fmi2Status;

typedef enum {
    fmi2ModelExchange,
    fmi2CoSimulation
} fmi2Type;

typedef enum {
    fmi2DoStepStatus,
    fmi2PendingStatus,
    fmi2LastSuccessfulTime,
    fmi2Terminated
} fmi2StatusKind;

typedef void      (*fmi2CallbackLogger)        (fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
typedef void*     (*fmi2CallbackAllocateMemory)(size_t, size_t);
typedef void      (*fmi2CallbackFreeMemory)    (void*);
typedef void      (*fmi2StepFinished)          (fmi2ComponentEnvironment, fmi2Status);

typedef struct {
   fmi2CallbackLogger         logger;
   fmi2CallbackAllocateMemory allocateMemory;
   fmi2CallbackFreeMemory     freeMemory;
   fmi2StepFinished           stepFinished;
   fmi2ComponentEnvironment   componentEnvironment;
} fmi2CallbackFunctions;

typedef struct {
   fmi2Boolean newDiscreteStatesNeeded;
   fmi2Boolean terminateSimulation;
   fmi2Boolean nominalsOfContinuousStatesChanged;
   fmi2Boolean valuesOfContinuousStatesChanged;
   fmi2Boolean nextEventTimeDefined;
   fmi2Real    nextEventTime;
} fmi2EventInfo;

/* Main structure that holds the state of master */
typedef struct
{
   fmi2Component  fmuInstanceId;
   fmi2Status     fmuInstanceStatus;
   double         commStepSize;
   double         currentExchangeTime;
   double         stopTime;
   char*          instanceName;
   fmi2CallbackFunctions cbfunctions;
   double         tunable_rparams[0+1];
   int            tunable_iparams[0+1];
} masterDataStruct;

#define fmuCallStatus lastStatus
#define masterData ((masterDataStruct*)ps[0])

/* reset alignment policy to the one set before reading this file */
#ifdef WIN32
#pragma pack(pop)
#endif

/* Define fmi2 function pointer types to simplify dynamic loading */

/***************************************************
Types for Common Functions
****************************************************/

/* Inquire version numbers of header files and setting logging status */
typedef const char* (*fmi2GetTypesPlatformTYPE)(void);
typedef const char* (*fmi2GetVersionTYPE)(void);
typedef fmi2Status  (*fmi2SetDebugLoggingTYPE)(fmi2Component, fmi2Boolean, size_t, const fmi2String[]);

/* Creation and destruction of FMU instances and setting debug status */
typedef fmi2Component (*fmi2InstantiateTYPE) (fmi2String, fmi2Type, fmi2String, fmi2String, const fmi2CallbackFunctions*, fmi2Boolean, fmi2Boolean);
typedef void          (*fmi2FreeInstanceTYPE)(fmi2Component);

/* Enter and exit initialization mode, terminate and reset */
typedef fmi2Status (*fmi2SetupExperimentTYPE)        (fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real);
typedef fmi2Status (*fmi2EnterInitializationModeTYPE)(fmi2Component);
typedef fmi2Status (*fmi2ExitInitializationModeTYPE) (fmi2Component);
typedef fmi2Status (*fmi2TerminateTYPE)              (fmi2Component);
typedef fmi2Status (*fmi2ResetTYPE)                  (fmi2Component);

/* Getting and setting variable values */
typedef fmi2Status (*fmi2GetRealTYPE)   (fmi2Component, const fmi2ValueReference[], size_t, fmi2Real   []);
typedef fmi2Status (*fmi2GetIntegerTYPE)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Integer[]);
typedef fmi2Status (*fmi2GetBooleanTYPE)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Boolean[]);
typedef fmi2Status (*fmi2GetStringTYPE) (fmi2Component, const fmi2ValueReference[], size_t, fmi2String []);

typedef fmi2Status (*fmi2SetRealTYPE)   (fmi2Component, const fmi2ValueReference[], size_t, const fmi2Real   []);
typedef fmi2Status (*fmi2SetIntegerTYPE)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[]);
typedef fmi2Status (*fmi2SetBooleanTYPE)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Boolean[]);
typedef fmi2Status (*fmi2SetStringTYPE) (fmi2Component, const fmi2ValueReference[], size_t, const fmi2String []);

/* Getting and setting the internal FMU state */
typedef fmi2Status (*fmi2GetFMUstateTYPE)           (fmi2Component, fmi2FMUstate*);
typedef fmi2Status (*fmi2SetFMUstateTYPE)           (fmi2Component, fmi2FMUstate);
typedef fmi2Status (*fmi2FreeFMUstateTYPE)          (fmi2Component, fmi2FMUstate*);
typedef fmi2Status (*fmi2SerializedFMUstateSizeTYPE)(fmi2Component, fmi2FMUstate, size_t*);
typedef fmi2Status (*fmi2SerializeFMUstateTYPE)     (fmi2Component, fmi2FMUstate, fmi2Byte[], size_t);
typedef fmi2Status (*fmi2DeSerializeFMUstateTYPE)   (fmi2Component, const fmi2Byte[], size_t, fmi2FMUstate*);

/* Getting partial derivatives */
typedef fmi2Status (*fmi2GetDirectionalDerivativeTYPE)(fmi2Component, const fmi2ValueReference[], size_t,
                                                                const fmi2ValueReference[], size_t,
                                                                const fmi2Real[], fmi2Real[]);

/***************************************************
Types for Functions for FMI2 for Co-Simulation
****************************************************/

/* Simulating the slave */
typedef fmi2Status (*fmi2SetRealInputDerivativesTYPE) (fmi2Component, const fmi2ValueReference [], size_t, const fmi2Integer [], const fmi2Real []);
typedef fmi2Status (*fmi2GetRealOutputDerivativesTYPE)(fmi2Component, const fmi2ValueReference [], size_t, const fmi2Integer [], fmi2Real []);

typedef fmi2Status (*fmi2DoStepTYPE)     (fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);
typedef fmi2Status (*fmi2CancelStepTYPE) (fmi2Component);

/* Inquire slave status */
typedef fmi2Status (*fmi2GetStatusTYPE)       (fmi2Component, const fmi2StatusKind, fmi2Status* );
typedef fmi2Status (*fmi2GetRealStatusTYPE)   (fmi2Component, const fmi2StatusKind, fmi2Real*   );
typedef fmi2Status (*fmi2GetIntegerStatusTYPE)(fmi2Component, const fmi2StatusKind, fmi2Integer*);
typedef fmi2Status (*fmi2GetBooleanStatusTYPE)(fmi2Component, const fmi2StatusKind, fmi2Boolean*);
typedef fmi2Status (*fmi2GetStringStatusTYPE) (fmi2Component, const fmi2StatusKind, fmi2String* );

/* Macros to construct the real function name
   (prepend function name by FMI2_FUNCTION_PREFIX) */
#if defined(FMI2_FUNCTION_PREFIX)
  #define fmi2Paste(a,b)     a ## b
  #define fmi2PasteB(a,b)    fmi2Paste(a,b)
  #define fmi2FullName(name) fmi2PasteB(FMI2_FUNCTION_PREFIX, name)
#else
  #define fmi2FullName(name) name
#endif

/***************************************************
Common Functions
****************************************************/
#define fmi2GetTypesPlatform         fmi2FullName(fmi2GetTypesPlatform)
#define fmi2GetVersion               fmi2FullName(fmi2GetVersion)
#define fmi2SetDebugLogging          fmi2FullName(fmi2SetDebugLogging)
#define fmi2Instantiate              fmi2FullName(fmi2Instantiate)
#define fmi2FreeInstance             fmi2FullName(fmi2FreeInstance)
#define fmi2SetupExperiment          fmi2FullName(fmi2SetupExperiment)
#define fmi2EnterInitializationMode  fmi2FullName(fmi2EnterInitializationMode)
#define fmi2ExitInitializationMode   fmi2FullName(fmi2ExitInitializationMode)
#define fmi2Terminate                fmi2FullName(fmi2Terminate)
#define fmi2Reset                    fmi2FullName(fmi2Reset)
#define fmi2GetReal                  fmi2FullName(fmi2GetReal)
#define fmi2GetInteger               fmi2FullName(fmi2GetInteger)
#define fmi2GetBoolean               fmi2FullName(fmi2GetBoolean)
#define fmi2GetString                fmi2FullName(fmi2GetString)
#define fmi2SetReal                  fmi2FullName(fmi2SetReal)
#define fmi2SetInteger               fmi2FullName(fmi2SetInteger)
#define fmi2SetBoolean               fmi2FullName(fmi2SetBoolean)
#define fmi2SetString                fmi2FullName(fmi2SetString)
#define fmi2GetFMUstate              fmi2FullName(fmi2GetFMUstate)
#define fmi2SetFMUstate              fmi2FullName(fmi2SetFMUstate)
#define fmi2FreeFMUstate             fmi2FullName(fmi2FreeFMUstate)
#define fmi2SerializedFMUstateSize   fmi2FullName(fmi2SerializedFMUstateSize)
#define fmi2SerializeFMUstate        fmi2FullName(fmi2SerializeFMUstate)
#define fmi2DeSerializeFMUstate      fmi2FullName(fmi2DeSerializeFMUstate)
#define fmi2GetDirectionalDerivative fmi2FullName(fmi2GetDirectionalDerivative)

/***************************************************
Functions for FMI2 for Co-Simulation
****************************************************/
#define fmi2SetRealInputDerivatives      fmi2FullName(fmi2SetRealInputDerivatives)
#define fmi2GetRealOutputDerivatives     fmi2FullName(fmi2GetRealOutputDerivatives)
#define fmi2DoStep                       fmi2FullName(fmi2DoStep)
#define fmi2CancelStep                   fmi2FullName(fmi2CancelStep)
#define fmi2GetStatus                    fmi2FullName(fmi2GetStatus)
#define fmi2GetRealStatus                fmi2FullName(fmi2GetRealStatus)
#define fmi2GetIntegerStatus             fmi2FullName(fmi2GetIntegerStatus)
#define fmi2GetBooleanStatus             fmi2FullName(fmi2GetBooleanStatus)
#define fmi2GetStringStatus              fmi2FullName(fmi2GetStringStatus)

/* Version number */
#define fmi2Version "2.0"

/***************************************************
Common Functions
****************************************************/

/* Inquire version numbers of header files */
static fmi2GetTypesPlatformTYPE fmi2GetTypesPlatform;
static fmi2GetVersionTYPE       fmi2GetVersion;
static fmi2SetDebugLoggingTYPE  fmi2SetDebugLogging;

/* Creation and destruction of FMU instances */
static fmi2InstantiateTYPE  fmi2Instantiate;
static fmi2FreeInstanceTYPE fmi2FreeInstance;

/* Enter and exit initialization mode, terminate and reset */
static fmi2SetupExperimentTYPE         fmi2SetupExperiment;
static fmi2EnterInitializationModeTYPE fmi2EnterInitializationMode;
static fmi2ExitInitializationModeTYPE  fmi2ExitInitializationMode;
static fmi2TerminateTYPE               fmi2Terminate;
static fmi2ResetTYPE                   fmi2Reset;

/* Getting and setting variables values */
static fmi2GetRealTYPE    fmi2GetReal;
static fmi2GetIntegerTYPE fmi2GetInteger;
static fmi2GetBooleanTYPE fmi2GetBoolean;
static fmi2GetStringTYPE  fmi2GetString;

static fmi2SetRealTYPE    fmi2SetReal;
static fmi2SetIntegerTYPE fmi2SetInteger;
static fmi2SetBooleanTYPE fmi2SetBoolean;
static fmi2SetStringTYPE  fmi2SetString;

/* Getting and setting the internal FMU state */
static fmi2GetFMUstateTYPE            fmi2GetFMUstate;
static fmi2SetFMUstateTYPE            fmi2SetFMUstate;
static fmi2FreeFMUstateTYPE           fmi2FreeFMUstate;
static fmi2SerializedFMUstateSizeTYPE fmi2SerializedFMUstateSize;
static fmi2SerializeFMUstateTYPE      fmi2SerializeFMUstate;
static fmi2DeSerializeFMUstateTYPE    fmi2DeSerializeFMUstate;

/* Getting partial derivatives */
static fmi2GetDirectionalDerivativeTYPE fmi2GetDirectionalDerivative;

/***************************************************
Functions for FMI2 for Co-Simulation
****************************************************/

/* Simulating the slave */
static fmi2SetRealInputDerivativesTYPE  fmi2SetRealInputDerivatives;
static fmi2GetRealOutputDerivativesTYPE fmi2GetRealOutputDerivatives;

static fmi2DoStepTYPE     fmi2DoStep;
static fmi2CancelStepTYPE fmi2CancelStep;

/* Inquire slave status */
static fmi2GetStatusTYPE        fmi2GetStatus;
static fmi2GetRealStatusTYPE    fmi2GetRealStatus;
static fmi2GetIntegerStatusTYPE fmi2GetIntegerStatus;
static fmi2GetBooleanStatusTYPE fmi2GetBooleanStatus;
static fmi2GetStringStatusTYPE  fmi2GetStringStatus;

/***************************************************
Functions for Co-Simulation Master
****************************************************/

/* Return the original variable name, given the base type and value reference */
static char *lookup_varname_from_valref(char btype, fmi2ValueReference vr, char *b)
{
   switch(btype)
   {
      case 'r':
         switch(vr)
         {
            case 134217728u: return "expseu_.SetSpeed_mps";
            case 134217729u: return "expseu_.PosError_m";
            case 134217730u: return "expseu_.HeadingError_rad";
            case 268435456u: return "expseu_.PositionX_m";
            case 268435457u: return "expseu_.PositionY_m";
            case 268435458u: return "expseu_.PositionZ_m";
            case 268435459u: return "expseu_.Roll_deg";
            case 268435460u: return "expseu_.Pitch_deg";
            case 268435461u: return "expseu_.Yaw_deg";
            case 268435462u: return "expseu_.WheelSpeedLeft_RPM";
            case 268435463u: return "expseu_.WheelSpeedRight_RPM";
            case 536870912u: return "run_parameters.maxTimeStep";
            case 536870913u: return "run_parameters.tolerance";
            case 536870914u: return "run_parameters.fixedStep";
            case 671088651u: return "Ky";
            case 671088652u: return "Kh";
            case 671088658u: return "Mload";
            case 671088666u: return "Xg_ini";
            case 671088669u: return "Yg_ini";
            case 671088671u: return "Zg_ini";
            default:
               sprintf(b, "unknown real value reference %d", vr);
               return b;
         }
         break;
      case 'i':
         switch(vr)
         {
            case 536870912u: return "run_parameters.integratorType";
            case 536870913u: return "run_parameters.errorType";
            case 536870914u: return "run_parameters.solverType";
            case 536870915u: return "run_parameters.integrationMethod";
            case 536870916u: return "run_parameters.fixedOrder";
            default:
               sprintf(b, "unknown integer value reference %d", vr);
               return b;
         }
         break;
      case 'b':
         switch(vr)
         {

            default:
               sprintf(b, "unknown boolean value reference %d", vr);
               return b;
         }
         break;
      case 's':
         switch(vr)
         {

            default:
               sprintf(b, "unknown string value reference %d", vr);
               return b;
         }
         break;
   } /* switch(btype) */
   sprintf(b, "unknown value reference %d", vr);
   return b;
}  /* lookup_varname_from_valref */

static void DynamicStrnCat(char **source, const char *end, unsigned int n)
{
   unsigned int new_length;

   if (n == 0)
      return;
   if (*source == NULL)
   {
      new_length = n + 1;
      *source = malloc(new_length*sizeof(char));
      strncpy(*source, end, n);
   }
   else if (end != NULL)
   {
      new_length = (unsigned int)strlen(*source) + n + 1;
      *source = (char *)realloc(*source, new_length*sizeof(char));
      strncat(*source, end, n);
   }
   (*source)[new_length-1] = '\0';
}  /* DynamicStrnCat */

/* Convert UTF8 to ANSI or to Latin1 */
static char* replace_utf8(const char* utf8buff, int toLatin1)
{
   typedef enum {
      UTF8_read1,
      UTF8_2_from_2,
      UTF8_2_from_3,
      UTF8_2_from_4,
      UTF8_3_from_3,
      UTF8_3_from_4,
      UTF8_4_from_4,
      UTF8_end,
      UTF8_error,
      UTF8_getcar,
      LATIN1_a0,
      LATIN1_c0
   } fsm_state;

   const char paddC = 0x3F; /* ? character */
   const char bom[3] = {0xEF,0xBB,0xBF};

   char* inIdx;
   char cMin,cMax,ctoGet;
   fsm_state fsm;
   size_t buffL;
   char* asciibuff = NULL;

   buffL = strlen(utf8buff);

   if (buffL>0) {
      asciibuff = (char*)calloc((buffL+1)*sizeof(char),1);
   }

   if (asciibuff) {
      const unsigned char* idxbuffin = utf8buff;
      char* idxbuffout = asciibuff;
      fsm = UTF8_read1;
      inIdx = 0;

      if (memcmp(utf8buff,bom,3) == 0)
         inIdx += 3;

      while((fsm != UTF8_end)&&(fsm != UTF8_error)) {
         switch(fsm) {
            case UTF8_read1:
               if ( *idxbuffin == 0x00 ) {
                  fsm = UTF8_end;
               }
               else if ( *idxbuffin <= 0x7F ) {
                  ctoGet = *idxbuffin; fsm = UTF8_getcar;
               }
               else if ((*idxbuffin == 0xC2)&&(toLatin1)) {
                  cMin = 0x80; cMax = 0xBF; fsm = LATIN1_a0;
               }
               else if ((*idxbuffin == 0xC3)&&(toLatin1)) {
                  cMin = 0x80; cMax = 0xBF; fsm = LATIN1_c0;
               }
               else if ((*idxbuffin >= 0xC2) && (*idxbuffin <= 0xDF)) {
                  cMin = 0x80; cMax = 0xBF; fsm = UTF8_2_from_2;
               }
               else if ( *idxbuffin == 0xE0 ) {
                  cMin = 0xA0; cMax = 0xBF; fsm = UTF8_2_from_3;
               }
               else if ( *idxbuffin == 0xED ) {
                  cMin = 0x80; cMax = 0x9F; fsm = UTF8_2_from_3;
               }
               else if ( (*idxbuffin >= 0xE1) && (*idxbuffin <= 0xEF) )
               {
                  cMin = 0x80; cMax = 0xBF; fsm = UTF8_2_from_3;
               }
               else if ( *idxbuffin == 0xF0 ) {
                  cMin = 0x90; cMax = 0xBF; fsm = UTF8_2_from_4;
               }
               else if ((*idxbuffin >= 0xF1) && (*idxbuffin <= 0xF3)) {
                  cMin = 0x80; cMax = 0xBF; fsm = UTF8_2_from_4;
               }
               else if ( *idxbuffin == 0xF4 ) {
                  cMin = 0x80; cMax = 0x8F; fsm = UTF8_2_from_4;
               }
               else {
                  fsm = UTF8_error;
               }
               break;
            case LATIN1_a0:
               if ((*idxbuffin >= 0xA0) && (*idxbuffin <= 0xBF)) {
                  ctoGet = *idxbuffin; fsm = UTF8_getcar;
               }
               else if ((*idxbuffin >= cMin) && (*idxbuffin <= cMax)) {
                  ctoGet = paddC; fsm = UTF8_getcar;
               }
               else
                  fsm = UTF8_error;
               break;
            case LATIN1_c0:
               if ((*idxbuffin >= 0x80) && (*idxbuffin <= 0xBF)) {
                  ctoGet = (*idxbuffin)+0x40; fsm = UTF8_getcar;
               }
               else if ((*idxbuffin >= cMin) && (*idxbuffin <= cMax)) {
                  ctoGet = paddC; fsm = UTF8_getcar;
               }
               else
                  fsm = UTF8_error;
               break;
            case UTF8_2_from_2:
               if ((*idxbuffin >= cMin) && (*idxbuffin <= cMax)) {
                  ctoGet = paddC; fsm = UTF8_getcar;
                }
               else
                  fsm = UTF8_error;
               break;
            case UTF8_2_from_3:
               if ((*idxbuffin >= cMin) && (*idxbuffin <= cMax))
                  fsm = UTF8_3_from_3;
               else
                  fsm = UTF8_error;
               break;
            case UTF8_2_from_4:
               if ((*idxbuffin >= cMin) && (*idxbuffin <= cMax))
                  fsm = UTF8_3_from_4;
               else
                  fsm = UTF8_error;
               break;
            case UTF8_3_from_4:
               if ((*idxbuffin >= 0x80) && (*idxbuffin <= 0xBF))
                  fsm = UTF8_4_from_4;
               else
                  fsm = UTF8_error;
               break;
            case UTF8_3_from_3:
            case UTF8_4_from_4:
               if ((*idxbuffin >= 0x80) && (*idxbuffin <= 0xBF)) {
                  ctoGet = paddC; fsm = UTF8_getcar;
               }
               else
                  fsm = UTF8_error;
               break;
            default:
               fsm = UTF8_error;
               break;
         }

         if (fsm == UTF8_getcar) {
            *idxbuffout = ctoGet;
            idxbuffout++;
            fsm = UTF8_read1;
         }
         idxbuffin++;
      }
      if (fsm != UTF8_end) {
         free(asciibuff);
         asciibuff = NULL;
      }
      else {
         *idxbuffout = 0x00;
         idxbuffout++;
         asciibuff = (char*)realloc(asciibuff,idxbuffout-asciibuff);
      }
   }

   return asciibuff;
}  /* replace_utf8 */

/* Look for value references between hash marks, like in #i987#
 * and replace it by the original variable name
 */
static void replace_variable_references(char *inmsg, char **outmsg)
{
   enum fsm_states { Regular, HashMark, ValueRef };
   size_t current = 0,   /* position of current character */
          s = 0,         /* start of buffer to be copied */
          p = 0;         /* current position in copy buffer */
   enum fsm_states fsm = Regular; /* finite state machine */

   /* scan the inmsg string character by character */
   *outmsg = NULL;
   while (current < strlen(inmsg))
   {
      char c = inmsg[current];
      switch (fsm)
      {
         case Regular:  /* FSM state: regular character */
            if (c != '#')
               p++;  /* ok, get next character */
            else
            {
               /* copy the content of buffer read so far */
               DynamicStrnCat(outmsg, inmsg+s, (unsigned int)p-(unsigned int)s);
               /* reset copy pointer */
               p = s = current + 1;
               fsm = HashMark;
            }
            break;
         case HashMark: /* FSM state: hash mark character */
            if (c == '#')
            {
               /* append a single hash mark */
               DynamicStrnCat(outmsg, inmsg+s, 1);
               /* reset copy pointer */
               p = s = current + 1;
               fsm = Regular;
            }
            else if (c == 'r' || c == 'i' || c == 'b' || c == 's')
            {
               /* ignore the type specifier */
               /* reset copy pointer */
               p = s = current + 1;
               fsm = ValueRef;
            }
            else
            {  /* invalid character */
               s--;  /* move back to hash mark */
               goto Error;
            }
            break;
         case ValueRef: /* FSM state: value reference */
            if (isdigit(c))
               p++;  /* ok, get next character */
            else if (c == '#')
            {
               fmi2ValueReference vref;
               char *varname, b[128];
               /* convert the value reference number from string to int */
               /* if value is invalid, it yields 0 which is an unknown variable */
               vref = (fmi2ValueReference)atoi(inmsg+s);
               /* get the variable name from the value reference number */
               varname = lookup_varname_from_valref(*(inmsg+s-1), vref, b);
               /* and append the name */
               DynamicStrnCat(outmsg, varname, (unsigned int)strlen(varname));
               /* reset copy pointer */
               p = s = current + 1;
               fsm = Regular;
            }
            else
            {  /* invalid character */
               s -= 2;  /* move back to hash mark */
               goto Error;
            }
            break;
      }
      current++;
   }
   /* append trailing characters, if any */
   DynamicStrnCat(outmsg, inmsg+s, (unsigned int)p-(unsigned int)s);
   return;

Error:
   /* Error: output the remaining part of inmsg without parsing */
   DynamicStrnCat(outmsg, inmsg+s, (unsigned int)strlen(inmsg+s));
}  /* replace_variable_references */

/* Logging function, of type fmiCallbackLogger */
static void fmulogger(fmi2ComponentEnvironment c,
                      fmi2String    instanceName,
                      fmi2Status    status,
                      fmi2String    category,
                      fmi2String    message,
                      ...)
{
   char buf[1024], *final_message, *status_msg;
   va_list arg;

   char* utf8buff;

   /* apply the variable list of arguments to the format string */
   va_start(arg, message);
   vsprintf(buf, message, arg);
   va_end(arg);

   /* convert UTF8 to LATIN1 */
   utf8buff =  replace_utf8(buf,1);

   if (utf8buff) {
      /* substitute the value references between hash marks by the variable name */
      replace_variable_references(utf8buff, &final_message);
      free(utf8buff);
   }
   else {
      final_message = (char*)calloc(50, sizeof(char));
      if (final_message)
         sprintf(final_message, "logger string format error.");
   }

   /* status message */
   switch (status)
   {
      case fmi2OK:
         status_msg = "OK";
         break;
      case fmi2Warning:
         status_msg = "Warning";
         break;
      case fmi2Discard:
         status_msg = "Discard";
         break;
      case fmi2Error:
         status_msg = "Error";
         break;
      case fmi2Fatal:
         status_msg = "Fatal";
         break;
      case fmi2Pending:
         status_msg = "Pending";
         break;
   }

   /* output using Amesim error channel */
   if (category)
      amefprintf(status == fmi2OK ? stdout : stderr, "%s: %s (%s) %s\n", instanceName, status_msg, category, final_message);
   else
      amefprintf(status == fmi2OK ? stdout : stderr, "%s: %s %s\n", instanceName, status_msg, final_message);
   free(final_message);
}  /* fmulogger */

static void *fmuAllocateMemory(size_t nobj, size_t size)
{
  return calloc(nobj, size);
}  /* fmuAllocateMemory */

static void fmuFreeMemory(void* obj)
{
   free(obj);
}  /* fmuFreeMemory */

static char* GetDLLfullPath(const char* fmuPath)
{
   char *binaryLocation = NULL, *actualFmuPath;
   size_t aSize;
   fmi2Boolean freeActualFmuPath = fmi2False;

#if defined(WIN32) && defined(WIN64)
#define EXT      ".dll"
#define PLATFORM "win64"
#elif defined(WIN32)
#define EXT      ".dll"
#define PLATFORM "win32"
#elif defined(LINUX64)
#define EXT      ".so"
#define PLATFORM "linux64"
#elif defined(LINUX)
#define EXT      ".so"
#define PLATFORM "linux32"
#endif

#ifdef WIN32
   if (!(actualFmuPath = _fullpath(NULL, fmuPath, _MAX_PATH+1)))
   {
      amefprintf(stderr, "Error unable to get absolute path for %s\n", fmuPath);
      actualFmuPath = fmuPath;  /* fallback */
   }
   else
      freeActualFmuPath = fmi2True;
#else
   actualFmuPath = fmuPath;
#endif
   aSize = strlen(actualFmuPath);
   aSize += sizeof("binaries");
   aSize += sizeof(PLATFORM);
   aSize += sizeof("AGV_turtle_electric_drive_Prescan_export");
   aSize += sizeof(EXT);
   aSize += 4;

   binaryLocation = (char*)malloc(aSize*sizeof(char));

   if(binaryLocation)
   {
#ifdef WIN32
      sprintf(binaryLocation, "%s/binaries/%s", actualFmuPath, PLATFORM);
      SubstituteChars(binaryLocation, '\\', '/');
      if (SetDllDirectory(binaryLocation))
         amefprintf(stdout, "Added %s to DLL search path\n", binaryLocation);
      else
         amefprintf(stderr, "Error while Adding %s to DLL search path\n", binaryLocation);
#endif
      sprintf(binaryLocation, "%s/binaries/%s/AGV_turtle_electric_drive_Prescan_export%s",
                              actualFmuPath, PLATFORM, EXT);
      SubstituteChars(binaryLocation, '\\', '/');
   }
   if (freeActualFmuPath == fmi2True)
      free(actualFmuPath);  /* previously allocated by _fullpath() */
   return binaryLocation;
} /* GetDLLfullPath */

/* <<<<<<<<<<<<End of Private Code. */

/*
 * Submodel initialization
 */
void turtlebotin_(int *n,
   double rp[10],
   int ip[8],
   char *tp[1],
   void *ps[1],
   double *expseuPositionXm /* PositionX_m - expseu instance 1 PositionX_m */,
   double *expseuPositionYm /* PositionY_m - expseu instance 1 PositionY_m */,
   double *expseuPositionZm /* PositionZ_m - expseu instance 1 PositionZ_m */,
   double *expseuRolldeg /* Roll_deg - expseu instance 1 Roll_deg */,
   double *expseuPitchdeg /* Pitch_deg - expseu instance 1 Pitch_deg */,
   double *expseuYawdeg /* Yaw_deg - expseu instance 1 Yaw_deg */,
   double *expseuWheelSpeedLeftRPM /* WheelSpeedLeft_RPM - expseu instance 1 WheelSpeedLeft_RPM */,
   double *expseuWheelSpeedRightRPM /* WheelSpeedRight_RPM - expseu instance 1 WheelSpeedRight_RPM */,
   double *Xgini /* Xg_ini - Initial X position of the COG (ground frame) */,
   double *Ygini /* Yg_ini - Initial Y position of the COG (ground frame) */,
   double *Zgini /* Zg_ini - Initial Z position of the COG (ground frame) */)

{
/* >>>>>>>>>>>>Extra Initialization Function Declarations Here. */
   char *instanceName = NULL;
   fmi2Status lastStatus = fmi2OK;
   fmi2Component fmuInstance;
   char *uri_fmulocation;
/* <<<<<<<<<<<<End of Extra Initialization declarations. */

   double comStepSize; /* co-simulation step size */
   double runparametersmaxTimeStep; /* maxTimeStep - maximum internal time step */
   double runparameterstolerance; /* tolerance - tolerance */
   double runparametersfixedStep; /* fixedStep - fixed step size */
   double Ky; /* Ky - Y-position control gain */
   double Kh; /* Kh - Heading control gain */
   double Mload; /* Mload - load additioal weight */
   double startexpseuSetSpeedmps; /* SetSpeed_mps - expseu instance 1 SetSpeed_mps - start value */
   double startexpseuPosErrorm; /* PosError_m - expseu instance 1 PosError_m - start value */
   double startexpseuHeadingErrorrad; /* HeadingError_rad - expseu instance 1 HeadingError_rad - start value */
   int enableLogging; /* enable logging */
   int comType; /* co-simulation step specification */
   int comNbStepSize; /* multiple of master fixed step size */
   int runparametersintegratorType; /* integratorType - integrator type: standard (1) or fixed step (2) */
   int runparameterserrorType; /* errorType - error control: mixed (1), relative (2), or absolute (3) */
   int runparameterssolverType; /* solverType - solver type: regular (1) or cautious (2) */
   int runparametersintegrationMethod; /* integrationMethod - fixed step integration method: Forward Euler (1), Adams-Bashforth (2) or Runge-Kutta (3) */
   int runparametersfixedOrder; /* fixedOrder - order of fixed step integration method */
   char *fmuLocation; /* path to the unzipped FMU root */

   comStepSize = rp[0];
   runparametersmaxTimeStep = rp[1];
   runparameterstolerance = rp[2];
   runparametersfixedStep = rp[3];
   Ky = rp[4];
   Kh = rp[5];
   Mload = rp[6];
   startexpseuSetSpeedmps = rp[7];
   startexpseuPosErrorm = rp[8];
   startexpseuHeadingErrorrad = rp[9];
   enableLogging = ip[0];
   comType = ip[1];
   comNbStepSize = ip[2];
   runparametersintegratorType = ip[3];
   runparameterserrorType = ip[4];
   runparameterssolverType = ip[5];
   runparametersintegrationMethod = ip[6];
   runparametersfixedOrder = ip[7];
   fmuLocation = tp[0];


/* >>>>>>>>>>>>Initialization Function Check Statements. */
   ps[0] = (masterDataStruct*)malloc(sizeof(masterDataStruct));

   if (!masterData)
   {
      amefprintf(stderr,"Error in %s (instance %d): memory allocation failed.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }

   masterData->fmuInstanceId                    = NULL;
   masterData->instanceName                     = NULL;
   masterData->cbfunctions.logger               = fmulogger;
   masterData->cbfunctions.allocateMemory       = fmuAllocateMemory;
   masterData->cbfunctions.freeMemory           = fmuFreeMemory;
   masterData->cbfunctions.stepFinished         = NULL;
   masterData->cbfunctions.componentEnvironment = NULL;

   if (comType == 1)
   {
      if (isfixedstepsolver_() != 0)
      {
         amefprintf(stderr,"Error in %s (instance %d): time step size cannot be specified with fixed-step solver.\n", _SUBMODELNAME_, *n);
         AmeExit(1);
      }

      masterData->commStepSize = comStepSize;
   }
   else
   {
      if (isfixedstepsolver_() == 0)
      {
         amefprintf(stderr,"Error in %s (instance %d): number of fixed step size cannot be specified with variable-step solver.\n", _SUBMODELNAME_, *n);
         AmeExit(1);
      }
      if (comNbStepSize < 1)
      {
         amefprintf(stderr,"Error in %s (instance %d): communication time must be a mutiple of the fixed-step solver step.\n", _SUBMODELNAME_, *n);
         AmeExit(1);
      }
      masterData->commStepSize = getfixedtimestep_();
      masterData->commStepSize *= (double)comNbStepSize;
   }

   /* Test multiple instance capability */
   if (fmi2True && (*n != 1))
   {
      amefprintf(stderr,"Error in %s (instance %d): cannot instantiate multiple fmu AGV_turtle_electric_drive_Prescan_export.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }

   /* Load library if needed */
   if (!fmu_handle)
   {
      char *binaryLocation;

      binaryLocation = GetDLLfullPath(fmuLocation);

      if (!binaryLocation)
      {
         amefprintf(stderr, "Error in %s (instance %d): memory allocation failed.\n", _SUBMODELNAME_, *n);
         AmeExit(1);
      }

      if (!(fmu_handle = OpenDynLib(binaryLocation)))
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot load %s.\n", _SUBMODELNAME_, *n, binaryLocation);
         free(binaryLocation);
         AmeExit(1);
      }
      free(binaryLocation);
   }

   lastStatus = fmi2OK;
   if (FindInLib(fmu_handle, "fmi2GetTypesPlatform", (void **)&fmi2GetTypesPlatform) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetTypesPlatform\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetVersion", (void **)&fmi2GetVersion) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetVersion\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetDebugLogging", (void **)&fmi2SetDebugLogging) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetDebugLogging\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2Instantiate", (void **)&fmi2Instantiate) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2Instantiate\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2FreeInstance", (void **)&fmi2FreeInstance) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2FreeInstance\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetupExperiment", (void **)&fmi2SetupExperiment) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetupExperiment\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2EnterInitializationMode", (void **)&fmi2EnterInitializationMode) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2EnterInitializationMode\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2ExitInitializationMode", (void **)&fmi2ExitInitializationMode) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2ExitInitializationMode\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2Terminate", (void **)&fmi2Terminate) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2Terminate\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2Reset", (void **)&fmi2Reset) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2Reset\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetReal", (void **)&fmi2GetReal) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetReal\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetInteger", (void **)&fmi2GetInteger) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetInteger\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetBoolean", (void **)&fmi2GetBoolean) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetBoolean\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetString", (void **)&fmi2GetString) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetString\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetReal", (void **)&fmi2SetReal) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetReal\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetInteger", (void **)&fmi2SetInteger) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetInteger\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetBoolean", (void **)&fmi2SetBoolean) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetBoolean\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetString", (void **)&fmi2SetString) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetString\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2SetRealInputDerivatives", (void **)&fmi2SetRealInputDerivatives) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2SetRealInputDerivatives\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetRealOutputDerivatives", (void **)&fmi2GetRealOutputDerivatives) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetRealOutputDerivatives\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2DoStep", (void **)&fmi2DoStep) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2DoStep\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2CancelStep", (void **)&fmi2CancelStep) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2CancelStep\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetStatus", (void **)&fmi2GetStatus) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetStatus\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetRealStatus", (void **)&fmi2GetRealStatus) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetRealStatus\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetIntegerStatus", (void **)&fmi2GetIntegerStatus) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetIntegerStatus\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetBooleanStatus", (void **)&fmi2GetBooleanStatus) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetBooleanStatus\n", *n);
      lastStatus = fmi2Fatal;
   }
   if (FindInLib(fmu_handle, "fmi2GetStringStatus", (void **)&fmi2GetStringStatus) != 0)
   {
      amefprintf(stderr, "Error in TURTLEBOT (instance %d): cannot find fmi2GetStringStatus\n", *n);
      lastStatus = fmi2Fatal;
   }


   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      AmeExit(1);
   }

/* <<<<<<<<<<<<End of Initialization Check Statements. */

/* >>>>>>>>>>>>Initialization Function Executable Statements. */
   instanceName = (char*)malloc(sizeof("AGV_turtle_electric_drive_Prescan_export")+7*sizeof(char));

   masterData->instanceName = instanceName;
   if (!instanceName)
   {
      amefprintf(stderr, "Error in %s (instance %d): memory allocation failed.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }
   sprintf(instanceName, "AGV_turtle_electric_drive_Prescan_export-%d", *n);

   if (!(uri_fmulocation = (char *)malloc(sizeof("file:////resources")+strlen(fmuLocation))))
   {
      amefprintf(stderr, "Error in %s (instance %d): memory allocation failed.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }

   /* Format fmuLocation according to rfc 3986 */
#ifdef WIN32
   if (strlen(fmuLocation) > 0 && fmuLocation[0] == '\\')
      sprintf(uri_fmulocation, "file:///%s/resources", &fmuLocation[1]);
   else if (strlen(fmuLocation) > 1 && fmuLocation[1] == ':')
      sprintf(uri_fmulocation, "file:///%s/resources", fmuLocation);
   else
#endif
   sprintf(uri_fmulocation, "file://%s/resources", fmuLocation);
   SubstituteChars(uri_fmulocation, '\\', '/');

   fmuInstance = fmi2Instantiate(
         instanceName,
         fmi2CoSimulation,
         "237474235",
         uri_fmulocation,
         &(masterData->cbfunctions),
         fmi2False /* visible */,
         enableLogging == 1 ? fmi2True : fmi2False);
   masterData->fmuInstanceId = fmuInstance;
   free(uri_fmulocation);

   if (!masterData->fmuInstanceId)
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot instantiate fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }
   masterData->fmuInstanceStatus = fmi2OK;
   fmu_nb_instance++;
   amefprintf(stdout, "%s successfully instantiated\n", instanceName);

   /* Assign start values */
   lastStatus = fmi2OK;
   {
      fmi2ValueReference vr[] = {536870912u, 536870913u, 536870914u, 671088651u, 671088652u, 671088658u};
      fmi2Real va[] = {runparametersmaxTimeStep, runparameterstolerance, runparametersfixedStep, Ky, Kh, Mload};
      lastStatus = fmi2SetReal(fmuInstance, vr, 6, va) == fmi2OK ? lastStatus : fmi2Error;

   }

   {
      fmi2ValueReference vr[] = {536870916u};
      fmi2Integer va[] = {runparametersfixedOrder};
      lastStatus = fmi2SetInteger(fmuInstance, vr, 1, va) == fmi2OK ? lastStatus : fmi2Error;

   }

   {
      fmi2ValueReference vr[] = {536870912u, 536870913u, 536870914u, 536870915u};
      fmi2Integer va[] = {runparametersintegratorType==1?1:2, runparameterserrorType==1?1:(runparameterserrorType==2?2:3), runparameterssolverType==1?1:2, runparametersintegrationMethod==1?1:(runparametersintegrationMethod==2?2:3)};
      lastStatus = fmi2SetInteger(fmuInstance, vr, 4, va) == fmi2OK ? lastStatus : fmi2Error;

   }

   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot set start values in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }

   /* Initialize experiment */
   fmuCallStatus = fmi2SetupExperiment(masterData->fmuInstanceId,
                                       fmi2False, 0.0,
                                       getstarttime_(), fmi2False, getfinaltime_());
   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot initialize experiment of fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }

   masterData->currentExchangeTime = getstarttime_();
   masterData->stopTime = getfinaltime_();

   /* Enter initialization mode */
   fmuCallStatus = fmi2EnterInitializationMode(masterData->fmuInstanceId);
   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): error entering initialization mode for fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }

   /* Assign initial values of inputs */
   lastStatus = fmi2OK;
   {
      fmi2ValueReference vr[] = {134217728u, 134217729u, 134217730u};
      fmi2Real va[] = {startexpseuSetSpeedmps, startexpseuPosErrorm, startexpseuHeadingErrorrad};
      lastStatus = fmi2SetReal(fmuInstance, vr, 3, va) == fmi2OK ? lastStatus : fmi2Error;

   }

   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot set start value of input variable(s) in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }

   /* Leave initialization mode */
   fmuCallStatus = fmi2ExitInitializationMode(masterData->fmuInstanceId);
   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): error leaving initialization mode for fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
      AmeExit(1);
   }


   /* Get values computed during initialization */
   lastStatus = fmi2OK;

   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot get value of fixed variables.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }

   lastStatus = fmi2OK;
   {
      fmi2ValueReference vr[] = {671088666u, 671088669u, 671088671u, 268435456u, 268435457u, 268435458u, 268435459u, 268435460u, 268435461u, 268435462u, 268435463u};
      fmi2Real va[11];
      lastStatus = fmi2GetReal(fmuInstance, vr, 11, va) == fmi2OK ? lastStatus : fmi2Error;
      *Xgini = va[0];
      *Ygini = va[1];
      *Zgini = va[2];
      *expseuPositionXm = va[3];
      *expseuPositionYm = va[4];
      *expseuPositionZm = va[5];
      *expseuRolldeg = va[6];
      *expseuPitchdeg = va[7];
      *expseuYawdeg = va[8];
      *expseuWheelSpeedLeftRPM = va[9];
      *expseuWheelSpeedRightRPM = va[10];
   }

   masterData->fmuInstanceStatus = fmuCallStatus;
   if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
   {
      amefprintf(stderr, "Error in %s (instance %d): cannot get value of modified parameters.\n", _SUBMODELNAME_, *n);
      AmeExit(1);
   }

   /* Store the tunable parameters into buffers */


   masterData->fmuInstanceStatus = fmi2OK;

   return;
   /* <<<<<<<<<<<<End of Initialization Executable Statements. */
}

/*
 * End function of submodel
 */
void turtlebotend_(int *n,
   double rp[10],
   int ip[8],
   char *tp[1],
   void *ps[1],
   double *expseuPositionXm /* PositionX_m - expseu instance 1 PositionX_m */,
   double *expseuPositionYm /* PositionY_m - expseu instance 1 PositionY_m */,
   double *expseuPositionZm /* PositionZ_m - expseu instance 1 PositionZ_m */,
   double *expseuRolldeg /* Roll_deg - expseu instance 1 Roll_deg */,
   double *expseuPitchdeg /* Pitch_deg - expseu instance 1 Pitch_deg */,
   double *expseuYawdeg /* Yaw_deg - expseu instance 1 Yaw_deg */,
   double *expseuWheelSpeedLeftRPM /* WheelSpeedLeft_RPM - expseu instance 1 WheelSpeedLeft_RPM */,
   double *expseuWheelSpeedRightRPM /* WheelSpeedRight_RPM - expseu instance 1 WheelSpeedRight_RPM */,
   double *Xgini /* Xg_ini - Initial X position of the COG (ground frame) */,
   double *Ygini /* Yg_ini - Initial Y position of the COG (ground frame) */,
   double *Zgini /* Zg_ini - Initial Z position of the COG (ground frame) */)

{
/* >>>>>>>>>>>>Extra Terminate Function Declarations Here. */
   fmi2Status lastStatus = fmi2OK;
/* <<<<<<<<<<<<End of Extra Terminate declarations. */

/* >>>>>>>>>>>>Terminate Function Executable Statements. */
   if (masterData)
   {
      if (fmu_handle)
      {
         if (masterData->fmuInstanceId)
         {
            if (masterData->fmuInstanceStatus == fmi2Pending)
            {
               if (fmi2CancelStep(masterData->fmuInstanceId) != fmi2Fatal)
                  fmi2FreeInstance(masterData->fmuInstanceId);
            }
            else if ((masterData->fmuInstanceStatus != fmi2Error) &&
                     (masterData->fmuInstanceStatus != fmi2Fatal))
            {
               if (fmi2Terminate(masterData->fmuInstanceId) != fmi2Fatal)
                  fmi2FreeInstance(masterData->fmuInstanceId);
            }
            else if (masterData->fmuInstanceStatus == fmi2Error)
            {
               fmi2FreeInstance(masterData->fmuInstanceId);
            }
            fmu_nb_instance--;
            amefprintf(stdout, "Instance %s terminated.\n", masterData->instanceName);
         }

         if (fmu_nb_instance == 0)
         {
            if (CloseDynLib(fmu_handle) != 0)
            {
               amefprintf(stderr, "Error in %s (instance %d): cannot unload library from memory!\n", _SUBMODELNAME_, *n);
            }
            else
            {
               amefprintf(stdout, "FMU library successfully unloaded.\n");
               fmu_handle = NULL;
#ifdef WIN32
               if (!SetDllDirectory(NULL))
                  amefprintf(stderr, "Error in %s (instance %d): Error while resetting the DLL search path\n", _SUBMODELNAME_, *n);
#endif
            }
         }
      }
      free(masterData->instanceName);

      free(masterData);
   }
   return;
/* <<<<<<<<<<<<End of Terminate Executable Statements. */
}

/*
 * Calculation function of submodel
 */
void turtlebot_(int *n,
   double *expseuPositionXm /* PositionX_m - expseu instance 1 PositionX_m */,
   double *expseuPositionYm /* PositionY_m - expseu instance 1 PositionY_m */,
   double *expseuPositionZm /* PositionZ_m - expseu instance 1 PositionZ_m */,
   double *expseuRolldeg /* Roll_deg - expseu instance 1 Roll_deg */,
   double *expseuPitchdeg /* Pitch_deg - expseu instance 1 Pitch_deg */,
   double *expseuYawdeg /* Yaw_deg - expseu instance 1 Yaw_deg */,
   double *expseuWheelSpeedLeftRPM /* WheelSpeedLeft_RPM - expseu instance 1 WheelSpeedLeft_RPM */,
   double *expseuWheelSpeedRightRPM /* WheelSpeedRight_RPM - expseu instance 1 WheelSpeedRight_RPM */,
   double *expseuSetSpeedmps /* SetSpeed_mps - expseu instance 1 SetSpeed_mps */,
   double *expseuPosErrorm /* PosError_m - expseu instance 1 PosError_m */,
   double *expseuHeadingErrorrad /* HeadingError_rad - expseu instance 1 HeadingError_rad */,
   double *Xgini /* Xg_ini - Initial X position of the COG (ground frame) */,
   double *Ygini /* Yg_ini - Initial Y position of the COG (ground frame) */,
   double *Zgini /* Zg_ini - Initial Z position of the COG (ground frame) */,
   double rp[10],
   int ip[8],
   char *tp[1],
   void *ps[1],
   int *flag,
   double *t)

{
/* >>>>>>>>>>>>Extra Calculation Function Declarations Here. */
   fmi2Status lastStatus = fmi2OK;
   double ct2_time, actual_commStepSize;
   char *instanceName = masterData->instanceName;
   fmi2Component fmuInstance = masterData->fmuInstanceId;
      

/* <<<<<<<<<<<<End of Extra Calculation declarations. */
/* >>>>>>>>>>>>Calculation Function Executable Statements. */

   if (firstc_())
   {
      if (ConstructionLevel_() != 0)
      {
         return; /* Do not start if in consolidation step */
      }
   }

   ct2_time = masterData->currentExchangeTime;

   if ((firstc_() || *flag == 0) && *t >= masterData->currentExchangeTime)
   {
      fmuCallStatus = masterData->fmuInstanceStatus;

      while(fmuCallStatus == fmi2Pending)
      {
         masterData->fmuInstanceStatus = fmi2GetStatus(masterData->fmuInstanceId,
                                                       fmi2DoStepStatus,
                                                       &fmuCallStatus);
         if (masterData->fmuInstanceStatus != fmi2OK)
         {
            fmuCallStatus = masterData->fmuInstanceStatus;
         }
      }
      if (fmuCallStatus != fmi2OK)
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot perform step to t=%25.16e s in fmu %s.\n", _SUBMODELNAME_, *n, masterData->currentExchangeTime, instanceName);
         AmeExit(1);
      }

      /* Retrieve outputs */
   lastStatus = fmi2OK;
   {
      fmi2ValueReference vr[] = {671088666u, 671088669u, 671088671u, 268435456u, 268435457u, 268435458u, 268435459u, 268435460u, 268435461u, 268435462u, 268435463u};
      fmi2Real va[11];
      lastStatus = fmi2GetReal(fmuInstance, vr, 11, va) == fmi2OK ? lastStatus : fmi2Error;
      *Xgini = va[0];
      *Ygini = va[1];
      *Zgini = va[2];
      *expseuPositionXm = va[3];
      *expseuPositionYm = va[4];
      *expseuPositionZm = va[5];
      *expseuRolldeg = va[6];
      *expseuPitchdeg = va[7];
      *expseuYawdeg = va[8];
      *expseuWheelSpeedLeftRPM = va[9];
      *expseuWheelSpeedRightRPM = va[10];
   }

      masterData->fmuInstanceStatus = fmuCallStatus;
      if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot retrieve outputs in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
         AmeExit(1);
      }

      /* Set inputs */
   lastStatus = fmi2OK;
   {
      fmi2ValueReference vr[] = {134217728u, 134217729u, 134217730u};
      fmi2Real va[] = {*expseuSetSpeedmps, *expseuPosErrorm, *expseuHeadingErrorrad};
      lastStatus = fmi2SetReal(fmuInstance, vr, 3, va) == fmi2OK ? lastStatus : fmi2Error;

   }

      masterData->fmuInstanceStatus = fmuCallStatus;
      if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot set inputs in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
         AmeExit(1);
      }

      /* Set tunable parameters */

   lastStatus = fmi2OK;

      masterData->fmuInstanceStatus = fmuCallStatus;
      if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot set tunable parameters in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
         AmeExit(1);
      }

      /* Store the tunable parameters into buffers */


      /* Copy the tunable parameters to keep their history */
   lastStatus = fmi2OK;

      masterData->fmuInstanceStatus = fmuCallStatus;
      if ( (fmuCallStatus == fmi2Error) || (fmuCallStatus == fmi2Fatal) )
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot copy tunable parameters to variable in fmu %s.\n", _SUBMODELNAME_, *n, instanceName);
         AmeExit(1);
      }

      /* Compute next exchange step time */
      if (fmi2True && ct2_time+masterData->commStepSize > masterData->stopTime)
         /* Prevent going beyond final time... */
         actual_commStepSize = masterData->stopTime-ct2_time;
      else
         actual_commStepSize = masterData->commStepSize;
      ct2_time += actual_commStepSize;

      fmuCallStatus = fmi2DoStep( masterData->fmuInstanceId,
                                  masterData->currentExchangeTime,
                                  actual_commStepSize,
                                  fmi2True);
      masterData->fmuInstanceStatus = fmuCallStatus;
      if ( (fmuCallStatus != fmi2OK) && (fmuCallStatus != fmi2Pending) )
      {
         amefprintf(stderr, "Error in %s (instance %d): cannot perform step to t=%25.16e s in fmu %s.\n", _SUBMODELNAME_, *n, ct2_time, instanceName);
         AmeExit(1);
      }
      /* Store new current step time */
      masterData->currentExchangeTime = ct2_time;
   }
   distim_(&ct2_time);

/* <<<<<<<<<<<<End of Calculation Executable Statements. */
} /* end of calculation function */
