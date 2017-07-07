/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(GyroInfo) == 16, SizeOfStructure_GyroInfo_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(GyroInfo) == 4, AlignmentOfStructure_GyroInfo_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(GyroConfig) == 4, SizeOfStructure_GyroConfig_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(GyroConfig) == 4, AlignmentOfStructure_GyroConfig_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(GyroData) == 12, SizeOfStructure_GyroData_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(GyroData) == 4, AlignmentOfStructure_GyroData_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<GyroInfo> GyroInfo::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<GyroInfo> GyroInfo::cleaner;)
const whiteboard::StructureValueSerializer<GyroConfig> GyroConfig::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<GyroConfig> GyroConfig::cleaner;)
const whiteboard::StructureValueSerializer<GyroData> GyroData::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<GyroData> GyroData::cleaner;)

} // namespace WB_RES