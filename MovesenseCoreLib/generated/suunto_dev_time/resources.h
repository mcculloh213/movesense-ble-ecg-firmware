#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"

#include "whiteboard/builtinTypes/Array.h"
#include "whiteboard/builtinTypes/ByteStream.h"
#include "whiteboard/builtinTypes/Date.h"
#include "whiteboard/builtinTypes/DateTime.h"
#include "whiteboard/builtinTypes/Optional.h"
#include "whiteboard/builtinTypes/Structures.h"
#include "whiteboard/builtinTypes/Time.h"
#include "whiteboard/builtinTypes/Timestamp.h"
#include "whiteboard/builtinTypes/TypedEnum.h"
#include "whiteboard/builtinTypes/Vector2D.h"
#include "whiteboard/builtinTypes/Vector3D.h"
#include "whiteboard/builtinTypes/WrapperFor32BitPointer.h"

#define WB_EXECUTION_CONTEXT_INSTANTION_REF(id) static_cast<whiteboard::ExecutionContextId>(id)
#define WB_RESOURCE_VALUE(whiteboardId, localResourceId, executionContextId) \
    static_cast<whiteboard::ResourceId::Value>( \
        (static_cast<uint32>(localResourceId) << 16) | \
        (static_cast<uint32>(whiteboardId) << 8) | \
        (static_cast<uint32>(executionContextId) << 4) | \
        (static_cast<uint32>(whiteboard::ID_INVALID_RESOURCE_INSTANCE)))

#define WB_CALLER_CONTEXT whiteboard::ID_INVALID_EXECUTION_CONTEXT


#include "../wb-resources/resources.h"
#include "../suunto_shared/resources.h"

#define WB_EXEC_CTX_PRIMARYSERVICES              WB_EXECUTION_CONTEXT_INSTANTION_REF(0)
#define WB_EXEC_CTX_APPLICATION                  WB_EXECUTION_CONTEXT_INSTANTION_REF(1)
#define WB_EXEC_CTX_MEAS                         WB_EXECUTION_CONTEXT_INSTANTION_REF(2)
#define WB_EXEC_CTX_UI                           WB_EXECUTION_CONTEXT_INSTANTION_REF(3)

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DEV;

struct DEV_TIME
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9984, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9984;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct PUT
	{
		struct Parameters
		{
			struct VALUE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int64 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/Time */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets VALUE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::VALUE::ConstReferenceType getValue() const
			{
				return mrParameterList[Parameters::VALUE::Index].convertTo<Parameters::VALUE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_DATECHANGE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9985, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9985;

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef uint32 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_DUALTIME
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9986, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9986;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_DUALTIME_MIN
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9987, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9987;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_HOUR
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9988, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9988;

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef bool NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_LOCALTIME
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9989, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9989;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_LOCALTIME_MIN
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9990, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9990;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_MIDNIGHT
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9991, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9991;

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef bool NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_TIME_MIN
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9992, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9992;

	struct GET
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef int64 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef int64 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};


} // namespace LOCAL

} // namespace WB_RES
