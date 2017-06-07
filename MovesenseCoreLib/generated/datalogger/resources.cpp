/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(DataLoggerConfig) == 8, SizeOfStructure_DataLoggerConfig_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(DataLoggerConfig) == 4, AlignmentOfStructure_DataLoggerConfig_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(DataEntryArray) == 8, SizeOfStructure_DataEntryArray_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(DataEntryArray) == 4, AlignmentOfStructure_DataEntryArray_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(DataEntry) == 4, SizeOfStructure_DataEntry_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(DataEntry) == 4, AlignmentOfStructure_DataEntry_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<DataLoggerConfig> DataLoggerConfig::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<DataLoggerConfig> DataLoggerConfig::cleaner;)
const whiteboard::StructureValueSerializer<DataEntryArray> DataEntryArray::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<DataEntryArray> DataEntryArray::cleaner;)
const whiteboard::StructureValueSerializer<DataEntry> DataEntry::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<DataEntry> DataEntry::cleaner;)

} // namespace WB_RES
