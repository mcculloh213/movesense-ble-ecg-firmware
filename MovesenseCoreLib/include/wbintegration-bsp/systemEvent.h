#pragma once
// Copyright (c) Suunto Oy 2015. All rights reserved.

#include "whiteboard/integration/port.h"

#include "platform/bsp/systemevent.h"

#define WB_SYSEVENT_API_ERROR        SYSEVENT_WBAPI_ERROR
#define WB_SYSEVENT_API_WARNING      SYSEVENT_WBAPI_WARNING
#define WB_SYSEVENT_API_EVENT        SYSEVENT_WBAPI_EVENT
#define WB_SYSEVENT_API_TRACE        SYSEVENT_WBAPI_TRACE

#define WB_SYSEVENT_COMM_ERROR       SYSEVENT_WBCOMM_ERROR
#define WB_SYSEVENT_COMM_WARNING     SYSEVENT_WBCOMM_WARNING
#define WB_SYSEVENT_COMM_EVENT       SYSEVENT_WBCOMM_EVENT
#define WB_SYSEVENT_COMM_TRACE       SYSEVENT_WBCOMM_TRACE

#define WB_SYSEVENT_DEVD_ERROR       SYSEVENT_WBDEVD_ERROR
#define WB_SYSEVENT_DEVD_WARNING     SYSEVENT_WBDEVD_WARNING
#define WB_SYSEVENT_DEVD_EVENT       SYSEVENT_WBDEVD_EVENT
#define WB_SYSEVENT_DEVD_TRACE       SYSEVENT_WBDEVD_TRACE

#define WB_SYSEVENT_MAIN_ERROR       SYSEVENT_WBMAIN_ERROR
#define WB_SYSEVENT_MAIN_WARNING     SYSEVENT_WBMAIN_WARNING
#define WB_SYSEVENT_MAIN_EVENT       SYSEVENT_WBMAIN_EVENT
#define WB_SYSEVENT_MAIN_TRACE       SYSEVENT_WBMAIN_TRACE

#define WB_SYSEVENT_PORT_ERROR       SYSEVENT_WBPORT_ERROR
#define WB_SYSEVENT_PORT_WARNING     SYSEVENT_WBPORT_WARNING
#define WB_SYSEVENT_PORT_EVENT       SYSEVENT_WBPORT_EVENT
#define WB_SYSEVENT_PORT_TRACE       SYSEVENT_WBPORT_TRACE

#define WB_SYSEVENT_ELAS_ERROR       SYSEVENT_WBELAS_ERROR
#define WB_SYSEVENT_ELAS_WARNING     SYSEVENT_WBELAS_WARNING
#define WB_SYSEVENT_ELAS_EVENT       SYSEVENT_WBELAS_EVENT
#define WB_SYSEVENT_ELAS_TRACE       SYSEVENT_WBELAS_TRACE

#define WB_SYSEVENT_JAVA_ERROR       SYSEVENT_WBJAVA_ERROR
#define WB_SYSEVENT_JAVA_WARNING     SYSEVENT_WBJAVA_WARNING
#define WB_SYSEVENT_JAVA_EVENT       SYSEVENT_WBJAVA_EVENT
#define WB_SYSEVENT_JAVA_TRACE       SYSEVENT_WBJAVA_TRACE