/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(SystemEventLogEntryArray) == 8, SizeOfStructure_SystemEventLogEntryArray_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(SystemEventLogEntryArray) == 4, AlignmentOfStructure_SystemEventLogEntryArray_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(SystemEventLogEntry) == 20, SizeOfStructure_SystemEventLogEntry_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(SystemEventLogEntry) == 4, AlignmentOfStructure_SystemEventLogEntry_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<SystemEventLogEntryArray> SystemEventLogEntryArray::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<SystemEventLogEntryArray> SystemEventLogEntryArray::cleaner;)
const whiteboard::StructureValueSerializer<SystemEventLogEntry> SystemEventLogEntry::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<SystemEventLogEntry> SystemEventLogEntry::cleaner;)

} // namespace WB_RES
