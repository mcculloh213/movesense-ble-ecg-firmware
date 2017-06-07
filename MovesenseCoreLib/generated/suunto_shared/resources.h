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

#define WB_EXEC_CTX_PRIMARYSERVICES              WB_EXECUTION_CONTEXT_INSTANTION_REF(0)
#define WB_EXEC_CTX_APPLICATION                  WB_EXECUTION_CONTEXT_INSTANTION_REF(1)
#define WB_EXEC_CTX_MEAS                         WB_EXECUTION_CONTEXT_INSTANTION_REF(2)
#define WB_EXEC_CTX_UI                           WB_EXECUTION_CONTEXT_INSTANTION_REF(3)

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct HeartRateQualityValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15362;

	enum Type
	{
		OK = 0U,
		NOTRELIABLE = 1U
	};
};
typedef whiteboard::TypedEnum<HeartRateQualityValues, HeartRateQualityValues::Type, uint8> HeartRateQuality;

struct IntervalTrainingStateValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15363;

	enum Type
	{
		DISABLED = 0U,
		ENABLED = 1U,
		INTERVAL = 2U,
		RECOVERY = 3U,
		COMPLETED = 4U
	};
};
typedef whiteboard::TypedEnum<IntervalTrainingStateValues, IntervalTrainingStateValues::Type, uint8> IntervalTrainingState;

struct IntervalTrainingTypeValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15364;

	enum Type
	{
		DURATION = 0U,
		DISTANCE = 1U
	};
};
typedef whiteboard::TypedEnum<IntervalTrainingTypeValues, IntervalTrainingTypeValues::Type, uint8> IntervalTrainingType;

struct UpgradeStateValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15365;

	enum Type
	{
		STARTED = 0U,
		ONGOING = 1U,
		DONE = 2U,
		FAILURE = 3U
	};
};
typedef whiteboard::TypedEnum<UpgradeStateValues, UpgradeStateValues::Type, uint8> UpgradeState;

struct WB_STRUCT_PACKED VersionInfo;
struct WB_STRUCT_PACKED VersionInfoArray;
struct WB_STRUCT_PACKED UpgradeStatus;
struct WB_STRUCT_PACKED FloatVector3DArray;
struct WB_STRUCT_PACKED AddressInfo;
struct WB_STRUCT_PACKED AddressInfoArray;

struct WB_STRUCT_PACKED VersionInfo
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15360;
	static const whiteboard::StructureValueSerializer<VersionInfo> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<VersionInfo> cleaner;)

	WB_ALIGN(4) whiteboard::WrapperFor32BitPointer<const char> name;
	WB_ALIGN(4) whiteboard::WrapperFor32BitPointer<const char> version;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(name)
			.visit(version);
	}
};

struct WB_STRUCT_PACKED VersionInfoArray
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15361;
	static const whiteboard::StructureValueSerializer<VersionInfoArray> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<VersionInfoArray> cleaner;)

	WB_ALIGN(4) whiteboard::Array< VersionInfo > versionInfo;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(versionInfo);
	}
};

struct WB_STRUCT_PACKED UpgradeStatus
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15366;
	static const whiteboard::StructureValueSerializer<UpgradeStatus> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<UpgradeStatus> cleaner;)

	WB_ALIGN(1) UpgradeState state;
	WB_ALIGN(1) uint8 progress;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(state)
			.visit(progress);
	}
};

struct WB_STRUCT_PACKED FloatVector3DArray
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15367;
	static const whiteboard::StructureValueSerializer<FloatVector3DArray> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<FloatVector3DArray> cleaner;)

	WB_ALIGN(4) whiteboard::Array< whiteboard::FloatVector3D > data;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(data);
	}
};

struct WB_STRUCT_PACKED AddressInfo
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15368;
	static const whiteboard::StructureValueSerializer<AddressInfo> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<AddressInfo> cleaner;)

	WB_ALIGN(4) whiteboard::WrapperFor32BitPointer<const char> name;
	WB_ALIGN(4) whiteboard::WrapperFor32BitPointer<const char> address;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(name)
			.visit(address);
	}
};

struct WB_STRUCT_PACKED AddressInfoArray
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15369;
	static const whiteboard::StructureValueSerializer<AddressInfoArray> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<AddressInfoArray> cleaner;)

	WB_ALIGN(4) whiteboard::Array< AddressInfo > addressInfo;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(addressInfo);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;


} // namespace LOCAL

} // namespace WB_RES
