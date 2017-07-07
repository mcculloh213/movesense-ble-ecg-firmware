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

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct ActiveProcessorValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 2048;

	enum Type
	{
		LP = 1U,
		AP = 2U
	};
};
typedef whiteboard::TypedEnum<ActiveProcessorValues, ActiveProcessorValues::Type, uint8> ActiveProcessor;

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DEV;

struct DEV_SYSTEM;

struct DEV_SYSTEM_DISPLAY;

struct DEV_SYSTEM_DISPLAY_RECORD
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2048, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2048;

	struct PUT
	{
		struct Parameters
		{
			struct START
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/Display/Record */
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

			/** Gets START parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::START::ConstReferenceType getStart() const
			{
				return mrParameterList[Parameters::START::Index].convertTo<Parameters::START::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct DEV_SYSTEM_DISPLAY_SAVE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2049, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2049;

	struct PUT
	{
		struct Parameters
		{
			struct FILENAME
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef const char* Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/Display/Save */
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

			/** Gets FILENAME parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::FILENAME::ConstReferenceType getFilename() const
			{
				return mrParameterList[Parameters::FILENAME::Index].convertTo<Parameters::FILENAME::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct DEV_SYSTEM_LPAPPVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2050, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2050;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_LPDALIVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2051, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2051;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_LPFILESYSTEMVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2052, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2052;

	struct GET
	{
		typedef const char* Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_LPUPDATEPROGRESS
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2053, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2053;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_MEMORY;

struct DEV_SYSTEM_MEMORY_FREE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2054, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2054;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_MEMORY_ISSUFFICIENT
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2055, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2055;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_MEMORY_LOWESTFREE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2056, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2056;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_MEMORY_SIZE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2057, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2057;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_MEMORY_USED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2058, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2058;

	struct GET
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct DEV_SYSTEM_UPDATELPAPP
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2059, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2059;

	struct PUT
	{
		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpApp */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct DEV_SYSTEM_UPDATELPDALI
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2060, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2060;

	struct PUT
	{
		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpDali */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct DEV_SYSTEM_UPDATELPFILESYSTEM
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2061, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2061;

	struct PUT
	{
		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpFilesystem */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};


} // namespace LOCAL

} // namespace WB_RES
