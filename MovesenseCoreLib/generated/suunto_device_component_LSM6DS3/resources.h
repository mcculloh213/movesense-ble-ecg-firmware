#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/

// Copyright (c) Suunto Oy 2014 - 2016. All rights reserved.

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

#define WB_EXECUTION_CONTEXT_INSTANTION_REF(id)					static_cast<whiteboard::ExecutionContextId>(id)
#define WB_RESOURCE_VALUE(whiteboardId, localResourceId, executionContextId) \
	static_cast<whiteboard::ResourceId::Value>( \
		(static_cast<uint32>(localResourceId) << 16) | \
		(static_cast<uint32>(whiteboardId) << 8) | \
		(static_cast<uint32>(executionContextId) << 4) | \
		(static_cast<uint32>(whiteboard::ID_INVALID_RESOURCE_INSTANCE)))

#define WB_CALLER_CONTEXT										whiteboard::ID_INVALID_EXECUTION_CONTEXT


#include "../wb-resources/resources.h"
#include "../suunto_shared/resources.h"

#define WB_EXEC_CTX_PRIMARYSERVICES              WB_EXECUTION_CONTEXT_INSTANTION_REF(0)
#define WB_EXEC_CTX_APPLICATION                  WB_EXECUTION_CONTEXT_INSTANTION_REF(1)
#define WB_EXEC_CTX_MEAS                         WB_EXECUTION_CONTEXT_INSTANTION_REF(2)
#define WB_EXEC_CTX_UI                           WB_EXECUTION_CONTEXT_INSTANTION_REF(3)

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct LSM6DS3OperationModeValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 3328;

	enum Type
	{
		PASSIVE = 0U,
		ACTIVE = 1U
	};
};
typedef whiteboard::TypedEnum<LSM6DS3OperationModeValues, LSM6DS3OperationModeValues::Type, uint8> LSM6DS3OperationMode;

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DEVICE;

struct DEVICE_COMPONENT;

struct DEVICE_COMPONENT_LSM6DS3;

struct DEVICE_COMPONENT_LSM6DS3_AUTOPASSIVEENABLED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 3328, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 3328;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

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

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Device/Component/LSM6DS3/AutoPassiveEnabled */
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
};

struct DEVICE_COMPONENT_LSM6DS3_MODE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 3329, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 3329;

	struct GET
	{
		typedef LSM6DS3OperationMode Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef LSM6DS3OperationMode Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef LSM6DS3OperationMode NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEVICE_COMPONENT_LSM6DS3_RAWENABLED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 3330, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 3330;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

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

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Device/Component/LSM6DS3/RawEnabled */
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
};

struct DEVICE_COMPONENT_LSM6DS3_TILTCOMPENSATIONENABLED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 3331, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 3331;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

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

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Device/Component/LSM6DS3/TiltCompensationEnabled */
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
};


} // namespace LOCAL

} // namespace WB_RES
