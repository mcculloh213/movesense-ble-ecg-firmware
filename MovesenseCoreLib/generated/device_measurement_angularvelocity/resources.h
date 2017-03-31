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

struct WB_STRUCT_PACKED FloatVector3DArray2;
struct WB_STRUCT_PACKED AngularVelocityValue;

struct WB_STRUCT_PACKED FloatVector3DArray2
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17152;
	static const whiteboard::StructureValueSerializer<FloatVector3DArray2> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<FloatVector3DArray2> cleaner;)

	WB_ALIGN(4) whiteboard::Array< whiteboard::FloatVector3D > data;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(data);
	}
};

struct WB_STRUCT_PACKED AngularVelocityValue
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 17153;
	static const whiteboard::StructureValueSerializer<AngularVelocityValue> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<AngularVelocityValue> cleaner;)

	WB_ALIGN(4) uint32 relativeTime;
	WB_ALIGN(4) whiteboard::Optional< FloatVector3DArray2 > measurement;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(relativeTime)
			.visit(measurement);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DEVICE;

struct DEVICE_MEASUREMENT;

struct DEVICE_MEASUREMENT_ANGULARVELOCITY
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17152, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17152;

	struct GET
	{
		typedef AngularVelocityValue Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
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
		typedef AngularVelocityValue NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEVICE_MEASUREMENT_ANGULARVELOCITY_HIGHSPEED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 17153, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 17153;

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef AngularVelocityValue NotificationType;
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
