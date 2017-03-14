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

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct DataLoggerStateValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 16643;

	enum Type
	{
		DATALOGGER_INVALID = 1U,
		DATALOGGER_READY = 2U,
		DATALOGGER_LOGGING = 3U
	};
};
typedef whiteboard::TypedEnum<DataLoggerStateValues, DataLoggerStateValues::Type, uint8> DataLoggerState;

struct WB_STRUCT_PACKED DataEntry;
struct WB_STRUCT_PACKED DataEntryArray;
struct WB_STRUCT_PACKED DataLoggerConfig;

struct WB_STRUCT_PACKED DataEntry
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 16640;
	static const whiteboard::StructureValueSerializer<DataEntry> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<DataEntry> cleaner;)

	WB_ALIGN(4) whiteboard::WrapperFor32BitPointer<const char> path;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(path);
	}
};

struct WB_STRUCT_PACKED DataEntryArray
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 16641;
	static const whiteboard::StructureValueSerializer<DataEntryArray> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<DataEntryArray> cleaner;)

	WB_ALIGN(4) whiteboard::Array< DataEntry > dataEntry;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(dataEntry);
	}
};

struct WB_STRUCT_PACKED DataLoggerConfig
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 16642;
	static const whiteboard::StructureValueSerializer<DataLoggerConfig> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<DataLoggerConfig> cleaner;)

	WB_ALIGN(4) DataEntryArray dataEntries;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(dataEntries);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DATALOGGER;

struct DATALOGGER_CONFIG
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 16640, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 16640;

	struct GET
	{
		typedef DataLoggerConfig Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct PUT
	{
		struct Parameters
		{
			struct CONFIG
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef DataLoggerConfig Type;
				typedef const Type& ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /DataLogger/Config */
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

			/** Gets CONFIG parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::CONFIG::ConstReferenceType getConfig() const
			{
				return mrParameterList[Parameters::CONFIG::Index].convertTo<Parameters::CONFIG::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct DATALOGGER_STATE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 16641, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 16641;

	struct GET
	{
		typedef DataLoggerState Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct PUT
	{
		struct Parameters
		{
			struct NEWSTATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef DataLoggerState Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /DataLogger/State */
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

			/** Gets NEWSTATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::NEWSTATE::ConstReferenceType getNewState() const
			{
				return mrParameterList[Parameters::NEWSTATE::Index].convertTo<Parameters::NEWSTATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};


} // namespace LOCAL

} // namespace WB_RES
