#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"
#include "whiteboard/Result.h"
#include "whiteboard/ResourceClient.h"

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
#include "../movesense_types/resources.h"

#define WB_EXEC_CTX_PRIMARYSERVICES              WB_EXECUTION_CONTEXT_INSTANTION_REF(0)
#define WB_EXEC_CTX_APPLICATION                  WB_EXECUTION_CONTEXT_INSTANTION_REF(1)
#define WB_EXEC_CTX_MEAS                         WB_EXECUTION_CONTEXT_INSTANTION_REF(2)

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct WB_STRUCT_PACKED MagnInfo;
struct WB_STRUCT_PACKED MagnConfig;
struct WB_STRUCT_PACKED MagnData;

struct WB_STRUCT_PACKED MagnInfo
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15616;
	static const whiteboard::StructureValueSerializer<MagnInfo> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<MagnInfo> cleaner;)

	WB_ALIGN(4) whiteboard::Array< uint16 > sampleRates;
	WB_ALIGN(4) whiteboard::Array< uint16 > scale;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(sampleRates)
			.visit(scale);
	}
};

struct WB_STRUCT_PACKED MagnConfig
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15617;
	static const whiteboard::StructureValueSerializer<MagnConfig> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<MagnConfig> cleaner;)

	WB_ALIGN(2) uint16 scale;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(scale);
	}
};

struct WB_STRUCT_PACKED MagnData
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 15618;
	static const whiteboard::StructureValueSerializer<MagnData> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<MagnData> cleaner;)

	WB_ALIGN(4) uint32 timestamp;
	WB_ALIGN(4) whiteboard::Array< whiteboard::FloatVector3D > arrayMagn;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(timestamp)
			.visit(arrayMagn);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct MEAS;

struct MEAS_MAGN;

struct MEAS_MAGN_CONFIG
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 15616, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 15616;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const MagnConfig&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_SERVICE_UNAVAILABLE> HTTP_CODE_SERVICE_UNAVAILABLE;

		struct Parameters
		{
			struct CONFIG
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef MagnConfig Type;
				typedef const Type& ConstReferenceType;
			};

			typedef CONFIG Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/Magn/Config */
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

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::CONFIG::ConstReferenceType)
		{
		}
	};
};

struct MEAS_MAGN_INFO
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 15617, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 15617;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const MagnInfo&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct MEAS_MAGN_SAMPLERATE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 15618, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 15618;

	struct SUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_NOT_IMPLEMENTED> HTTP_CODE_NOT_IMPLEMENTED;

		struct Parameters
		{
			struct SAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef SAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/Magn/{SampleRate} */
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

			/** Gets SAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::SAMPLERATE::ConstReferenceType getSampleRate() const
			{
				return mrParameterList[Parameters::SAMPLERATE::Index].convertTo<Parameters::SAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::SAMPLERATE::ConstReferenceType)
		{
		}
	};

	struct EVENT
	{
		typedef MagnData NotificationType;
		typedef const NotificationType& ConstReferenceNotificationType;

		struct Parameters
		{
			struct SAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef SAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/Magn/{SampleRate} */
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

			/** Checks whether optional parameter SAMPLERATE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasSampleRate() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::SAMPLERATE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::SAMPLERATE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets SAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::SAMPLERATE::ConstReferenceType getSampleRate() const
			{
				return mrParameterList[Parameters::SAMPLERATE::Index].convertTo<Parameters::SAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			const whiteboard::Api::OptionalParameter<ConstReferenceNotificationType>&,
			const whiteboard::Api::OptionalParameter<Parameters::SAMPLERATE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};

	struct UNSUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			struct SAMPLERATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			typedef SAMPLERATE Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Meas/Magn/{SampleRate} */
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

			/** Gets SAMPLERATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::SAMPLERATE::ConstReferenceType getSampleRate() const
			{
				return mrParameterList[Parameters::SAMPLERATE::Index].convertTo<Parameters::SAMPLERATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::SAMPLERATE::ConstReferenceType)
		{
		}
	};
};


} // namespace LOCAL

} // namespace WB_RES
