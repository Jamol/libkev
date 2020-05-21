//
//  kmtrace.h
//  kev
//
//  Created by Fengping Bao <jamol@live.com> on 11/12/14.
//  Copyright (c) 2014-2019. All rights reserved.
//

#pragma once

#include "../../include/kmconf.h"

#include <sstream>
#include <assert.h>

namespace kev {

#define KM_TRACE(l, x) \
    do{ \
        if (l <= kev::getTraceLevel()) {\
            std::stringstream ss; \
            ss<<x; \
            kev::traceWrite(l, ss.str()); \
        }\
    }while(0)
    
#define KM_XTRACE(l, x) \
    do{ \
        if (l <= kev::getTraceLevel()) {\
            std::stringstream ss; \
            ss<<getObjKey()<<":: "<<x; \
            kev::traceWrite(l, ss.str()); \
        }\
    }while(0)
    
#define KM_INFOXTRACE(x)  KM_XTRACE(kev::TRACE_LEVEL_INFO, x)
#define KM_WARNXTRACE(x)  KM_XTRACE(kev::TRACE_LEVEL_WARN, x)
#define KM_ERRXTRACE(x)   KM_XTRACE(kev::TRACE_LEVEL_ERROR, x)
#define KM_DBGXTRACE(x)   KM_XTRACE(kev::TRACE_LEVEL_DEBUG, x)

#define KM_INFOTRACE(x)   KM_TRACE(kev::TRACE_LEVEL_INFO, x)
#define KM_WARNTRACE(x)   KM_TRACE(kev::TRACE_LEVEL_WARN, x)
#define KM_ERRTRACE(x)    KM_TRACE(kev::TRACE_LEVEL_ERROR, x)
#define KM_DBGTRACE(x)    KM_TRACE(kev::TRACE_LEVEL_DEBUG, x)

#define KM_INFOTRACE_THIS(x)   KM_TRACE(kev::TRACE_LEVEL_INFO, x<<", this="<<this)
#define KM_WARNTRACE_THIS(x)   KM_TRACE(kev::TRACE_LEVEL_WARN, x<<", this="<<this)
#define KM_ERRTRACE_THIS(x)    KM_TRACE(kev::TRACE_LEVEL_ERROR, x<<", this="<<this)
#define KM_DBGTRACE_THIS(x)    KM_TRACE(kev::TRACE_LEVEL_DEBUG, x<<", this="<<this)

#define KM_ASSERT(x) assert(x)

const int TRACE_LEVEL_ERROR  = 1;
const int TRACE_LEVEL_WARN   = 2;
const int TRACE_LEVEL_INFO   = 3;
const int TRACE_LEVEL_DEBUG  = 4;
const int TRACE_LEVEL_VERBOS = 5;
const int TRACE_LEVEL_MAX = TRACE_LEVEL_VERBOS;

void traceWrite(int level, const std::string &msg);
void traceWrite(int level, std::string &&msg);

// msg is null-terminated and msg_len doesn't include '\0'
using TraceFunc = void(*)(int level, const char* msg, size_t msg_len);
void setTraceFunc(TraceFunc func);
void setTraceLevel(int level);
int getTraceLevel();

} // namespace kev
