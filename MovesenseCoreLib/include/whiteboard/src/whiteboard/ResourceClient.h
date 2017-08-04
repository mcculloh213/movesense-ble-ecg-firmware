#pragma once
// Copyright (c) Suunto Oy 2014. All rights reserved.

#include "whiteboard/ApiHelpers.h"
#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"
#include "whiteboard/Request.h"
#include "whiteboard/Result.h"
#include "whiteboard/Value.h"
#include "whiteboard/ValueStorage.h"

#if WB_UNITTEST_BUILD
#include "whiteboard/unittest/RequestHook.h"
#endif

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs" // Caused by the WB_STATIC_VERIFY calls below
#endif

WB_HEADER_CHECK_DEFINE(WB_HAVE_DEBUG_NAMES);
WB_HEADER_CHECK_DEFINE(WB_UNITTEST_BUILD);

namespace whiteboard
{

// Forward declarations
class Whiteboard;

/** Options for the operation can be given with this class. */
class WB_API ResourceClient_RequestOptions
{
public:
    /** Ctor
    @param timeoutMs [in] Request timeout in milliseconds (Timeout applies only to remote requests.
                          If zero, request won't timeout).
    */
    EXPLICIT ResourceClient_RequestOptions(size_t timeoutMs) : mTimeoutMs(timeoutMs) {}

    /** @return timeout in ms to wait until the call is cancelled */
    inline size_t getTimeout() const { return mTimeoutMs; }

    /// Empty request options, that can be used if no options required for the operation
    static const ResourceClient_RequestOptions Empty;

private:
    // Prevent use of default constructor
    ResourceClient_RequestOptions() DELETED;

private:
    /** Request timeout in milliseconds. Ignore in local Whiteboard queries. */
    size_t mTimeoutMs;
};

/** Options for the asynchronous operation can be given with this class. */
class WB_API ResourceClient_AsyncRequestOptions
{
public:
    /** Ctor
    @param pRequestId [out] Pointer to store the requestId of the operation
    @param timeoutMs [in] Request timeout in milliseconds (Timeout applies only to remote requests.
                          If zero, request won't timeout).
    @param forceAsync [in] Forces the request to be performed asynchronously, even if the resource is in the same context as the client.
    @param isCriticalSubscription [in] Subscriptions only; Criticality of the subscription; allow data to be lost in case of congestion (
    recipients event queues full or all message buffers in use. Usefull with subscriptions with high update frequency - e.g. measurement / 
    sensor data.

    In case of resource subscription of a path containing path parameters, the criticality flag affects ALL subscriptions of that path from
    that client.
    */
    ResourceClient_AsyncRequestOptions(RequestId* pRequestId, size_t timeoutMs = 0, bool forceAsync = false, bool isCriticalSubscription = true)
        : mpRequestId(pRequestId), 
          mTimeoutMs(timeoutMs), 
          mForceAsync(forceAsync ? 1 : 0), 
          mIsCriticalSub(isCriticalSubscription ? 1 : 0), 
          mReserved(0) 
    {
        WB_NOT_USED(mReserved);
    }

    /** @return pointer to the requestId */
    inline RequestId* getRequestId() const { return mpRequestId; }

    /** @return timeout in ms to wait until the call is cancelled */
    inline size_t getTimeout() const { return mTimeoutMs; }

    /**
    @return true if the operation should be explicitly performed asyncronously.
    By default whiteboard performs operation directly, if the requested resource is in the same context.
    */
    inline bool getForceAsync() const { return mForceAsync ? true : false; }

    /**
    @return true if notifications to subscriptions can be thrown away in case of congestion (client's event queue full or blocking object 
    pools full).
    */
    inline bool isCriticalSubscription() const { return mIsCriticalSub ? true : false; }

    /// Empty async request options, that can be used if no options required for the operation
    static const ResourceClient_AsyncRequestOptions Empty;

private:
    /** Prevent use of default constructor */
    ResourceClient_AsyncRequestOptions() DELETED;

private:
    /** Optional pointer to variable where ID of the request will be saved. */
    RequestId* mpRequestId;

    /** Request timeout in milliseconds. Ignore in local Whiteboard queries. */
    size_t mTimeoutMs;
    
    /** A value indicating whether request should be always performed asynchronously */
    uint8 mForceAsync : 1;
    
    /** A value indicating whether subscription request is considered critical */
    uint8 mIsCriticalSub : 1;

    /** Reserved for future use */
    uint8 mReserved : 6;
};

/**
Base class Whiteboard resource clients
*/
class WB_API ResourceClient
{
public:

    // This alias allows forward declaration of the options in internal classes
    typedef ResourceClient_RequestOptions RequestOptions;
    typedef ResourceClient_AsyncRequestOptions AsyncRequestOptions;

    /**
    *	Initializes a new instance of the ResourceClient class
    *
    *	@param pClientName Name of the client
    *	@param executionContextId ID of client's execution context
    */
    ResourceClient(const char* pClientName, ExecutionContextId executionContextId);

    /**
    *	Destructor
    */
    virtual ~ResourceClient();

    /**
    *	Gets name of the client
    */
    const char* getName() const;

    /**
    *	Gets ID of the client
    *
    *	@return ID of the registered client or ID_INVALID_CLIENT if client has not been registered
    */
    ClientId getId() const;

    /**
    *	Gets local client ID of the client
    *
    *	@return Local ID of the registered client or ID_INVALID_LOCAL_CLIENT if client has not been registered
    */
    LocalClientId getLocalClientId() const;

    /** Gets ID of client's execution context
     *
     * @return ID of client's execution context
     */
    ExecutionContextId getExecutionContextId() const;

    /**
    *	Resolves resource path. Blocks calling thread until request
    *	result has been received.
    *
    *	@param pFullPath Path of the resource
    *	@param rResourceId On successful output contains the resource identifier
    *	@param rOptions Optional. The options for the request
    *	@return Result of the operation
    */
    Result getResource(const char* pFullPath,
                       ResourceId& rResourceId,
                       const RequestOptions& rOptions = RequestOptions::Empty);

    /**
    *	Performs asynchronous resource resolving
    *
    *	@param pFullPath Path of the resource
    *	@param rOptions Optional. The options for the request
    *	@return Result of the operation
    */
    Result asyncGetResource(const char* pFullPath, const AsyncRequestOptions& rOptions = AsyncRequestOptions::Empty);

    /**
    *	Performs resource release. Release is intented to indicate to the provider's whiteboard, that this client does not use the
    *	resource anymore.
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@return Result of the operation
    */
    Result releaseResource(ResourceId resourceId, const RequestOptions& rOptions = RequestOptions::Empty);

    /**
    *	Performs asynchronous resource release. Release is intented to indicate to the provider's whiteboard, that this
    *   client does not use the resource anymore.
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@return Result of the operation
    */
    Result asyncReleaseResource(ResourceId resourceId, const AsyncRequestOptions& rOptions = AsyncRequestOptions::Empty);

    /**
    *	Performs asynchronous GET request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncGet(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    /**
    *	Performs asynchronous PUT request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncPut(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    /**
    *	Performs asynchronous POST request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncPost(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    /**
    *	Performs asynchronous DELETE request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncDelete(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    /**
    *	Performs asynchronous SUBSCRIBE request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncSubscribe(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    /**
    *	Performs asynchronous UNSUBSCRIBE request.
    *
    *	@tparam P1 .. P8 Optional. Request parameter native types
    *
    *	@param resourceId ID of the associated resource
    *	@param rOptions Optional. The options for the request
    *	@param rP1 .. rP8 Optional. List of request parameters
    *	@return Result of the operation
    */
    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    inline Result asyncUnsubscribe(
        ResourceId resourceId,
        const AsyncRequestOptions& rOptions,
        const P1& rP1,
        const P2& rP2,
        const P3& rP3,
        const P4& rP4,
        const P5& rP5,
        const P6& rP6,
        const P7& rP7,
        const P8& rP8);

    // Include overloads that have different number of parameters (we don't want to clutter this header)
    #include "ResourceClient_AsyncRequestOverloads.h"

    /**
    *	Starts a continuous timer with given period. Override whiteboard::ResourceClient::onTimer to handle timer notifications.
    *
    *   Note: If previous timer message has not been processed, for example due to execution context blocking on long operation -
    *   and short interval continuous timer is in use, the timer messages might be filtered out to prevent flooding the message 
    *   queues.
    *
    *   Note: Starting a timer from interrupt is not supported.
    *
    *	@param periodMs Period in milliseconds how often the timer should fire.
    *	@param isContinous If true, continuous timer, else single shot.
    *
    *	@return ID of the started timer or ID_INVALID_TIMER, if starting a new timer fails (WB_MAX_NUMBER_OF_CLIENT_TIMERS is
    *   exceeded).
    *
    *	@see whiteboard::ResourceClient::onTimer
    */
    TimerId startTimer(size_t periodMs, bool isContinuous = false);

    /**
    *   Reschedules the continuous timer with a new period; The existing timer is stopped and restarted (but not deallocated).
    *
    *   Note: A timer message with the previous timer interval might have been queued before processing the rescheduling.
    *   and hence the first time might fire in period less than the new period.
    *
    *   @param timerId Id of the timer that is running
    *	@param periodMs New period in milliseconds how often the timer should fire.
    *
    *	@see whiteboard::ResourceClient::onTimer
    */
    bool rescheduleTimer(TimerId timerId, size_t periodMs);

    /**
    *	Stops a timer started with startTimer. No notifications of the given timer are received after this call returns.
    *
    *   Note: In case of continuous timer, a notification might be received after the stop if the stop is called from
    *   another thread than the clients execution context. Hence using timer start / stop from another thread is discouraged.
    *
    *   Note: Stopping a timer from interrupt is not supported.
    *
    *	@param timerId Id of the timer to stop
    *
    *	@return true on success, false if timer was already stopped or this provider did not own the timer, or if id was illegal.
    *
    *	@see whiteboard::ResourceClient::startTimer
    */
    bool stopTimer(TimerId timerId);

    /**
    *   This unsafe method is only for adapting other interfaces on top of Whiteboard!
    *
    *	Performs asynchronous request.
    *
    *	@param resourceId ID of the associated resource
    *	@param requestType Type of the request
    *	@param rOptions Options for this async operation, @see whiteboard::ResourceClient::AsyncRequestOptions
    *	@param rParameters List of request parameters
    *	@return Result of the operation
    */
    WB_FORCE_INLINE
    Result WB_FORCE_INLINE_ATTRIBUTE asyncRequestVariant(
        ResourceId resourceId,
        RequestType requestType,
        const AsyncRequestOptions& rOptions,
        const ParameterList& rParameters)
    {
        return asyncRequestInternal(resourceId, requestType, rOptions, rParameters);
    }

#if WB_UNITTEST_BUILD
    /**
    *   Unit test API for setting response overriding for results supposed to be coming from Whiteboard
    *   to a specific client instance. When a request made by the client is matched to a specified hook
    *   it is intercepted and a response specified in the hook will be sent back to the client to enable
    *   client instance testing without going through Whiteboard itself.
    *   (HTTP_CODE_INVALID == pRequestHooks[i].mCallResult) inidcates end of table terminator.
    *
    *   @param rClientId ID of the client for which the hook table is intetended
    *   @param pRequestHooks Pointer to the hook table, NULL to disable hooks
    *   @param pWhiteboard Optional parameter to select mockup WB instance instead of default WB
    *   @return HTTP_CODE_NOT_FOUND returned if client not found, otherwise HTTP_CODE_OK
    */
    static Result setRequestHooks(const LocalClientId& rClientId,
                                  const RequestHookDef* pRequestHooks,
                                  const Whiteboard* pWhiteboard = NULL);

    /**
    *   Unit test API for setting response overriding for results supposed to be coming from Whiteboard
    *   to a specific client instance. When a request made by the client is matched to a specified hook
    *   it is intercepted and a response specified in the hook will be sent back to the client to enable
    *   client instance testing without going through Whiteboard itself.
    *   (HTTP_CODE_INVALID == pRequestHooks[i].mCallResult) inidcates end of table terminator.
    *
    *   @param clientName Name string of the client for which the hook table is intetended
    *   @param pRequestHooks Pointer to the hook table, NULL to disable hooks
    *   @param pWhiteboard Optional parameter to select mockup WB instance instead of default WB
    *   @return HTTP_CODE_NOT_FOUND returned if client not found, otherwise HTTP_CODE_OK
    */
    static Result setRequestHooks(const char* clientName,
                                  const RequestHookDef* pRequestHooks,
                                  const Whiteboard* pWhiteboard = NULL);

protected:
    /**
    *   Internal method for assigning the RequestHook table pointer to the specific client instance.
    *
    *   @param pRequestHooks Pointer to the hook table, NULL to disable hooks
    */
    void setRequestHooks(const RequestHookDef* pRequestHooks);

    /// Pointer to the optional request hook table given from unit test case
    const RequestHookDef* mpRequestHookTabRef;    // storage reference for filter table defined in test scope
    /// Counter to fake RequestId numbers
    RequestId mTestRequestCount;
    /// Storage for parameters to be passed in case of request hook override match
    RequestHookParams mTestReqParams;
#endif

protected:
    /** These functions are called only by Whiteboard */
    friend class Whiteboard;

#if WB_UNITTEST_BUILD
    /** Gets the associated whiteboard instance
    *
    * @return Whiteboard instance
    */
    virtual Whiteboard& getWhiteboard();

    /**
    *   Async callback function set for DPC to produce async call to pass overridden
    *   request response with paramteres to the RequestClient instance.
    */
    void responseAsyncWrapper();
#endif

    /**
    *	Callback for asynchronous resource requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    */
    virtual void onGetResourceResult(RequestId requestId, ResourceId resourceId, Result resultCode);

    /**
    *	Callback for asynchronous resource requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    */
    virtual void onReleaseResourceResult(RequestId requestId, ResourceId resourceId, Result resultCode);

    /**
    *	Callback for asynchronous resource GET requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onGetResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *	Callback for asynchronous resource PUT requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onPutResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *	Callback for asynchronous SUBSCRIBE requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onSubscribeResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *	Callback for asynchronous UNSUBSCRIBE requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onUnsubscribeResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *	Callback for resource notifications.
    *
    *	@param resourceId Resource id associated with the update
    *	@param rValue Current value of the resource
    *	@param rParameters Notification parameters
    */
    virtual void onNotify(ResourceId resourceId, const Value& rValue, const ParameterList& rParameters);

    /**
    *	Callback for timer notifications.
    *
    *	@param timerId ID of the timer that has expired.
    *
    *	@see whiteboard::ResourceClient::startTimer
    *	@see whiteboard::ResourceClient::stopTimer
    */
    virtual void onTimer(TimerId timerId);

    /***
    * Callback for POST operation result
    *
    * @param requestId ID of the request
    * @param resourceId Successful request contains ID of the resource
    * @param resultCode Result code of the request
    * @param rResultData Successful result contains the request result
    *
    * @see whiteboard::ResourceClient::asyncPost
    */
    virtual void onPostResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);
    
    /***
    * Callback for DELETE operation result
    *
    * @param requestId ID of the request
    * @param resourceId Successful request contains ID of the resource
    * @param resultCode Result code of the request
    * @param rResultData Successful result contains the request result
    *
    * @see whiteboard::ResourceClient::asyncDelete
    */
    virtual void onDeleteResult(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *  Remote whiteboard disconnect notification handler.
    *
    *  This can be used for example to cleanup possible subscription related state in the client-end. 
    *  Whiteboard automatically cleans subscriptions and path variable allocations related to the disconnected whiteboard,
    *  so the client will not get further notifications and does not need to call unsubscribe or releaseResource either.
    *
    *  @param whiteboardId ID of the whiteboard that has been disconnected.
    *
    *  @see whiteboard::ResourceProvider::onSubscribe
    *  @see whiteboard::ResourceProvider::onUnsubscribe
    */
    virtual void onRemoteWhiteboardDisconnected(WhiteboardId whiteboardId);

    /**
    * Local resource unavailability handler.
    *
    *  This can be used for example to cleanup possible subscription related state in the client-end.
    *  Whiteboard automatically cleans subscriptions and path variable allocations related to the removed resource,
    *  so the client will not get further notifications and does not need to call unsubscribe or releaseResource either.
    *
    *  @param resourceId ID of the resource that is no longer available
    *
    *  @see whiteboard::ResourceProvider::onSubscribe
    *  @see whiteboard::ResourceProvider::onUnsubscribe
    */
    virtual void onResourceUnavailable(ResourceId resourceId);

private:
    /**
    *	Sets ID of the client
    *
    *	@param clientId New ID for the client
    */
    void setLocalClientId(LocalClientId clientId);

    /**
    *   This unsafe method is for internal use only - or for porting Whiteboard to specific environment!
    *
    *	Performs asynchronous request.
    *
    *	@param resourceId ID of the associated resource
    *	@param requestType Type of the request
    *	@param rOptions Options for this async operation, @see whiteboard::ResourceClient::AsyncRequestOptions
    *	@param rParameters List of request parameters
    *	@return Result of the operation
    */
    Result asyncRequestInternal(
        ResourceId resourceId,
        RequestType requestType,
        const AsyncRequestOptions& rOptions,
        const ParameterList& rParameters);

    /**
    *	Performs asynchronous PUT of a stream.
    *
    *	@param rRequest Request information
    *	@param rParameters List of request parameters
    *	@param rOptions Options for this async operation, @see whiteboard::ResourceClient::AsyncRequestOptions
    *	@return Result of the operation
    */
    Result asyncPutInternalStream(
        Request& rRequest,
        const ParameterList& rParameters,
        const AsyncRequestOptions& rOptions);

    /**
    *	System callback for asynchronous PUT requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    void onPutResultSys(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

    /**
    *	System callback for asynchronous SUBSCRIBE requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    void onSubscribeResultSys(RequestId requestId, ResourceId resourceId, Result resultCode, const Value& rResultData);

private:
    // disallow C++ default constructor and copy constructor usage
    ResourceClient() DELETED;
    ResourceClient(ResourceClient&) DELETED;

#ifdef WB_HAVE_DEBUG_NAMES
    /**
    * Name of the client
    */
    const char* mpName;
#endif

    /**
    * ID of the client
    */
    LocalClientId mId;

    /**
    * ID of client's next request
    */
    RequestId mNextRequestId;

    /**
    * ID of client's execution context
    */
    ExecutionContextId mExecutionContextId;

    /** Flag to enable / disable broadcast events during unbind */
    bool mHasSubscribtions;

    /** Stream handle that is used for splitting large PUT streams into multiple smaller requests */
    StreamHandle mInternalStreamId;

    /** Flag that is used internally to piggy back information about compile time type check */
    static const uint8 TYPE_CHECKED = 0x80;
};

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncGet(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_GET, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncPut(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_PUT, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncPost(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_POST, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncDelete(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_DELETE, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncSubscribe(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_SUBSCRIBE, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

WB_FORCE_INLINE
template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
WB_FORCE_INLINE_ATTRIBUTE Result ResourceClient::asyncUnsubscribe(
    ResourceId resourceId,
    const ResourceClient::AsyncRequestOptions& rOptions,
    const P1& rP1,
    const P2& rP2,
    const P3& rP3,
    const P4& rP4,
    const P5& rP5,
    const P6& rP6,
    const P7& rP7,
    const P8& rP8)
{
    return asyncRequest(resourceId, REQUEST_UNSUBSCRIBE, rOptions,
        static_cast<typename Api::ParameterType<P1>::type>(rP1),
        static_cast<typename Api::ParameterType<P2>::type>(rP2),
        static_cast<typename Api::ParameterType<P3>::type>(rP3),
        static_cast<typename Api::ParameterType<P4>::type>(rP4),
        static_cast<typename Api::ParameterType<P5>::type>(rP5),
        static_cast<typename Api::ParameterType<P6>::type>(rP6),
        static_cast<typename Api::ParameterType<P7>::type>(rP7),
        static_cast<typename Api::ParameterType<P8>::type>(rP8));
}

} // namespace whiteboard

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
