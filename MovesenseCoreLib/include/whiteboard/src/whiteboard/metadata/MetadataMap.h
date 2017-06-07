#pragma once
// Copyright (c) Suunto Oy 2015. All rights reserved.

#include "whiteboard/integration/port.h"
#include "whiteboard/metadata/MetadataStructures.h"

namespace whiteboard
{

/** Map of read-only resource metadata */
class WB_API MetadataMap
{
public:
    /** Constructor 
     *
     * @param pMetadataBlob Metadata blob instanse
     */
    MetadataMap(const uint32* pMetadataBlob);

    /** Destructor */
    virtual ~MetadataMap();

    /** Checks validity of the metadata blob 
    *
    * @return A value indicating wether the metadata blob i valid
    */
    bool isValid() const;

    /// @return The metadata header
    inline const metadata::MetadataBlobHeader& getMetadataHeader() const
    {
        return *reinterpret_cast<const metadata::MetadataBlobHeader*>(mpMetadataBlob);
    }

    /// @return number of response lists in the const metadata map
    inline size_t getStringMapLength() const
    {
        return getMetadataHeader().stringMapLength;
    }

    /// @return Number of execution contexts
    inline size_t getNumberOfExecutionContexts() const
    {
        return getMetadataHeader().numberOfExecutionContexts;
    }

    /// @return Number of security tags in the resource tree
    inline size_t getNumberOfMountPoints() const
    {
        return getMetadataHeader().numberOfMountPoints;
    }

    /// @return number of strings in the const metadata map
    size_t countNumberOfStrings() const;

    /// @return number of properties in the const metadata map
    inline size_t getNumberOfProperties() const
    {
        return getMetadataHeader().numberOfProperties;
    }

    /// @return number of property lists in the const metadata map
    inline size_t getNumberOfPropertyLists() const
    {
        return getMetadataHeader().numberOfPropertyListMapEntries;
    }

    /// @return number of enumeration items in the const metadata map
    inline size_t getNumberOfEnumerationItems() const
    {
        return getMetadataHeader().numberOfEnumerationItems;
    }

    /// @return number of data types in the const metadata map
    inline size_t getNumberOfDataTypes() const
    {
        return getMetadataHeader().numberOfDataTypes;
    }

    /// @return Number of sparse map entries in the data type map
    inline size_t getNumberOfDataTypeSparseMapEntries() const
    {
        return getMetadataHeader().numberOfDataTypeSparseMapEntries;
    }

    /// @return number of parameters in the const metadata map
    inline size_t getNumberOfParameters() const
    {
        return getMetadataHeader().numberOfParameters;
    }

    /// @return number of parameter lists in the const metadata map
    inline size_t getNumberOfParameterLists() const
    {
        return getMetadataHeader().numberOfParameterListMapEntries;
    }

    /// @return number of responses in the const metadata map
    inline size_t getNumberOfResponses() const
    {
        return getMetadataHeader().numberOfResponses;
    }

    /// @return number of response lists in the const metadata map
    inline size_t getNumberOfResponseLists() const
    {
        return getMetadataHeader().numberOfResponseListMapEntries;
    }

    /// @return number of operations in the const metadata map
    inline size_t getNumberOfOperations() const
    {
        return getMetadataHeader().numberOfOperations;
    }

    /// @return number of operation lists in the const metadata map
    inline size_t getNumberOfOperationLists() const
    {
        return getMetadataHeader().numberOfOperationLists;
    }

    /// @return number of security tags in the const metadata map
    inline size_t getNumberOfSecurityTags() const
    {
        return getMetadataHeader().numberOfSecurityTags;
    }

    /// @return Number of resource tree nodes
    inline size_t getNumberOfResourceTreeNodes() const
    {
        return getMetadataHeader().numberOfResourceTreeNodes;
    }

    /// @return Number of sparse map entries in the resource tree
    inline size_t getNumberOfResourceTreeSparseMapEntries() const
    {
        return getMetadataHeader().numberOfResourceTreeSparseMapEntries;
    }

    /** @return array of strings */
    inline const char* getStringMap() const
    {
        return reinterpret_cast<const char*>(
            mpMetadataBlob + getMetadataHeader().offsetToStringMap);
    }

    /** @return array of execution contexts */
    inline const metadata::ExecutionContext* getExecutionContexts() const
    {
        return reinterpret_cast<const metadata::ExecutionContext*>(
            mpMetadataBlob + getMetadataHeader().offsetToExecutionContexts);
    }

    /** @return array of properties */
    inline const metadata::Property* getProperties() const
    {
        return reinterpret_cast<const metadata::Property*>(
            mpMetadataBlob + getMetadataHeader().offsetToProperties);
    }

    /** @return array of property lists */
    inline const metadata::PropertyId* getPropertyLists() const
    {
        return reinterpret_cast<const metadata::PropertyId*>(
            mpMetadataBlob + getMetadataHeader().offsetToPropertyLists);
    }

    /** @return array of enumeration items */
    inline const metadata::EnumerationItem* getEnumerationItems() const
    {
        return reinterpret_cast<const metadata::EnumerationItem*>(
            mpMetadataBlob + getMetadataHeader().offsetToEnumerationItems);
    }

    /** @return array of data types */
    inline const metadata::DataType* getDataTypes() const
    {
        return reinterpret_cast<const metadata::DataType*>(
            mpMetadataBlob + getMetadataHeader().offsetToDataTypes);
    }

    /** @return array of data type sparse map entries */
    inline const metadata::DataTypeId* getDataTypeSparseMap() const
    {
        return reinterpret_cast<const metadata::DataTypeId*>(
            mpMetadataBlob + getMetadataHeader().offsetToDataTypeSparseMap);
    }

    /** @return array of parameters */
    inline const metadata::Parameter* getParameters() const
    {
        return reinterpret_cast<const metadata::Parameter*>(
            mpMetadataBlob + getMetadataHeader().offsetToParameters);
    }

    /** @return array of parameter lists */
    inline const metadata::ParameterId* getParameterLists() const
    {
        return reinterpret_cast<const metadata::ParameterId*>(
            mpMetadataBlob + getMetadataHeader().offsetToParameterLists);
    }

    /** @return array of responses */
    inline const metadata::Response* getResponses() const
    {
        return reinterpret_cast<const metadata::Response*>(
            mpMetadataBlob + getMetadataHeader().offsetToResponses);
    }

    /** @return array of response lists */
    inline const metadata::ResponseId* getResponseLists() const
    {
        return reinterpret_cast<const metadata::ResponseId*>(
            mpMetadataBlob + getMetadataHeader().offsetToResponseLists);
    }

    /** @return array of operations */
    inline const metadata::Operation* getOperations() const
    {
        return reinterpret_cast<const metadata::Operation*>(
            mpMetadataBlob + getMetadataHeader().offsetToOperations);
    }

    /** @return array of operation lists */
    inline const metadata::OperationList* getOperationLists() const
    {
        return reinterpret_cast<const metadata::OperationList*>(
            mpMetadataBlob + getMetadataHeader().offsetToOperationLists);
    }

    /** @return array of security tags */
    inline const metadata::SecurityTag* getSecurityTags() const
    {
        return reinterpret_cast<const metadata::SecurityTag*>(
            mpMetadataBlob + getMetadataHeader().offsetToSecurityTags);
    }

    /** @return array of resource tree nodes in the resource tree */
    inline const metadata::ResourceTreeNode* getResourceTreeNodes() const
    {
        return reinterpret_cast<const metadata::ResourceTreeNode*>(
            mpMetadataBlob + getMetadataHeader().offsetToResourceTreeNodes);
    }

    /** @return array of resource tree node sparse map entries */
    inline const LocalResourceId* getResourceTreeSparseMap() const
    {
        return reinterpret_cast<const LocalResourceId*>(
            mpMetadataBlob + getMetadataHeader().offsetToResourceTreeSparseMap);
    }

    /** Returns a string by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const char* getStringById(metadata::StringId id) const;

    /** Returns a data type by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::DataType* getDataTypeById(metadata::DataTypeId id) const;

    /** Returns a data type by index
    *
    * @param index ID of the object
    * @param rDataTypeId On output contains ID of the data type
    * @return Object or NULL if invalid id is given
    */
    const metadata::DataType* getDataTypeByIndex(size_t index, metadata::DataTypeId& rDataTypeId) const;

    /** Returns a property by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::Property* getPropertyById(metadata::PropertyId id) const;

    /** Returns a property list by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::PropertyList* getPropertyListById(metadata::PropertyListId id) const;

    /** Returns a enumeration item list by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::EnumerationItemList* getEnumerationItemListById(metadata::EnumerationItemListId id) const;

    /** Returns a parameter by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::Parameter* getParameterById(metadata::ParameterId id) const;

    /** Returns a parameter list by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::ParameterList* getParameterListById(metadata::ParameterListId id) const;

    /** Returns a response by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::Response* getResponseById(metadata::ResponseId id) const;

    /** Returns a response list by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::ResponseList* getResponseListById(metadata::ResponseListId id) const;

    /** Returns a operation by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::Operation* getOperationById(metadata::OperationId id) const;

    /** Returns a operation list by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::OperationList* getOperationListById(metadata::OperationListId id) const;

    /** Returns a security tag by ID
    *
    * @param id ID of the object
    * @return Object or NULL if invalid id is given
    */
    const metadata::SecurityTag* getSecurityTagById(SecurityTagId id) const;

private:
    /** Metadata blob */
    const uint8* mpMetadataBlob;
};

} // namespace whiteboard
