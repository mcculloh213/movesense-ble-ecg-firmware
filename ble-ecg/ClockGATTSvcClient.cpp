#include "movesense.h"

#include "ClockGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"

const char* const ClockGATTSvcClient::LAUNCHABLE_NAME = "ClockSvc";
// // UUID: 0000XXXX-0000-1000-8000-00805F9B34FB
// constexpr uint8_t CLOCK_DATASERVICE_UUID[] = {
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 
//     0x00, 0x80, 
//     0x00, 0x10, 
//     0x00, 0x00, 
//     0xXX, 0xXX, 0x00, 0x00
// };
// UUID: 00004200-0000-1000-8000-00805F9B34FB
constexpr uint8_t CLOCK_DATASERVICE_UUID[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 
    0x00, 0x80, 
    0x00, 0x10, 
    0x00, 0x00, 
    0x00, 0x42, 0x00, 0x00
};
// UUID: 00000001-0000-1000-8000-00805F9B34FB
constexpr uint8_t SET_EPOCH_CHARACTERISTIC_UUID[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 
    0x00, 0x80, 
    0x00, 0x10, 
    0x00, 0x00, 
    0x01, 0x00, 0x00, 0x00
};
constexpr uint16_t SET_EPOCH_CHARACTERISTIC_UUID16 = 0x0001;
// UUID: 00000002-0000-1000-8000-00805F9B34FB
constexpr uint8_t GET_EPOCH_CHARACTERISTIC_UUID[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 
    0x00, 0x80, 
    0x00, 0x10, 
    0x00, 0x00, 
    0x02, 0x00, 0x00, 0x00
};
constexpr uint16_t GET_EPOCH_CHARACTERISTIC_UUID16 = 0x0002;

ClockGATTSvcClient::ClockGATTSvcClient() :
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mSetEpochCharResource(wb::ID_INVALID_RESOURCE),
    mGetEpochCharResource(wb::ID_INVALID_RESOURCE),
    mClockServiceHandle(0),
    mGetEpochCharHandle(0),
    mSetEpochCharHandle(0),
    mNotificationsEnabled(false)
{
}

ClockGATTSvcClient::~ClockGATTSvcClient()
{
}

bool ClockGATTSvcClient::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    DEBUGLOG("ClockGATTSvcClient: Initialized");
    return true;
}

void ClockGATTSvcClient::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
    DEBUGLOG("ClockGATTSvcClient: Uninitialized");
}

bool ClockGATTSvcClient::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;
    DEBUGLOG("ClockGATTSvcClient: Started");

    // follow BLE connection status
    asyncSubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());

    // configure service
    configureService();

    return true;
}

void ClockGATTSvcClient::stopModule()
{
    // unsubscribe resources
    asyncUnsubscribe(mSetEpochCharResource);
    asyncUnsubscribe(mGetEpochCharResource);

    // release resources
    releaseResource(mSetEpochCharResource);
    releaseResource(mGetEpochCharResource);

    // invalidate resources
    mSetEpochCharResource = wb::ID_INVALID_RESOURCE;
    mGetEpochCharResource = wb::ID_INVALID_RESOURCE;

    mModuleState = WB_RES::ModuleStateValues::STOPPED;
    DEBUGLOG("ClockGATTSvcClient: Stopped");
}

void ClockGATTSvcClient::configureService()
{
    WB_RES::GattSvc clockGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &setEpochChar = characteristics[0];
    WB_RES::GattChar &getEpochChar = characteristics[1];

    // specify characterictic properties
    WB_RES::GattProperty setEpochCharProp = WB_RES::GattProperty::WRITE;
    WB_RES::GattProperty getEpochCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::NOTIFY
    };

    // specify set epoch characteristic
    setEpochChar.props = wb::MakeArray<WB_RES::GattProperty>(&setEpochCharProp, 1);
    setEpochChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&SET_EPOCH_CHARACTERISTIC_UUID), sizeof(SET_EPOCH_CHARACTERISTIC_UUID));

    // specify get epoch characteristic
    getEpochChar.props = wb::MakeArray<WB_RES::GattProperty>(getEpochCharProps, 2);
    getEpochChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&GET_EPOCH_CHARACTERISTIC_UUID), sizeof(GET_EPOCH_CHARACTERISTIC_UUID));

    // specify clock service
    clockGattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&CLOCK_DATASERVICE_UUID), sizeof(CLOCK_DATASERVICE_UUID));
    clockGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 2);

    // create service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions(NULL, 0, true), clockGattSvc);
    DEBUGLOG("ClockGATTSvcClient: Created GATT service");
}

/** @see whiteboard::ResourceClient::onPostResult */
void ClockGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                      wb::ResourceId resourceId, 
                                      wb::Result resultCode, 
                                      const wb::Value& rResultData)
{
    DEBUGLOG("ClockGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // custom  service was created
        mClockServiceHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Custom Gatt service was created. handle: %d", mClockServiceHandle);
        
        // request more info about created svc so we get the char handles
        asyncGet(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(), AsyncRequestOptions(NULL, 0, true), mClockServiceHandle);
        // Note: The rest of the init is performed in onGetResult()
    }
}

/** @see whiteboard::ResourceClient::onGetResult */
void ClockGATTSvcClient::onGetResult(wb::RequestId requestId,
                                     wb::ResourceId resourceId,
                                     wb::Result resultCode,
                                     const wb::Value& rResultData)
{
    DEBUGLOG("ClockGATTSvcClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            // finalize service setup (triggered by code in onPostResult)
            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i=0; i < svc.chars.size(); i++)
            {
                // identify characteristic handles and store them for later use
                const WB_RES::GattChar &c = svc.chars[i];
                // extract 16-bit sub-uuid from full 128-bit uuid
                DEBUGLOG("c.uuid.size() %u", c.uuid.size());
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[12]));
                DEBUGLOG("char[%u] 128-bit UUID: 0x%032X", i, c.uuid);
                DEBUGLOG("char[%u] 16-bit UUID: 0x%04X", i, uuid16);

                switch(uuid16)
                {
                    case SET_EPOCH_CHARACTERISTIC_UUID16:
                        mSetEpochCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case GET_EPOCH_CHARACTERISTIC_UUID16:
                        mGetEpochCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    default:
                        DEBUGLOG("Unknown 16-bit UUID: 0x%04X", uuid16);
                        break;
                }
            }

            if (!mSetEpochCharHandle || !mGetEpochCharHandle)
            {
                DEBUGLOG("ERROR: Not all characteristics were configured!!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mClockServiceHandle, mSetEpochCharHandle);
            getResource(pathBuffer, mSetEpochCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mClockServiceHandle, mGetEpochCharHandle);
            getResource(pathBuffer, mGetEpochCharResource);

            // force subscriptions asynchronously to save stack (will have stack overflow if not) 
            asyncSubscribe(mSetEpochCharResource, AsyncRequestOptions(NULL, 0, true));
            asyncSubscribe(mGetEpochCharResource, AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

/** @see whiteboard::ResourceClient::onSubscribeResult */
void ClockGATTSvcClient::onSubscribeResult(wb::RequestId requestId,
                                           wb::ResourceId resourceId,
                                           wb::Result resultCode,
                                           const wb::Value& rResultData)
{
DEBUGLOG("onSubscribeResult() resourceId: %u, resultCode: %d", resourceId, resultCode);

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
            {
                DEBUGLOG("OnSubscribeResult: WB_RES::LOCAL::COMM_BLE_PEERS: %d", resultCode);
                return;
            }
            break;
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
            {
                DEBUGLOG("OnSubscribeResult: COMM_BLE_GATTSVC*: %d", resultCode);
                return;
            }
            break;
        default:
        {
            break;
        }
    }
}

/** @see whiteboard::ResourceClient::onNotify */
void ClockGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                  const wb::Value& rValue,
                                  const wb::ParameterList& rParameters)
{
    DEBUGLOG("ClockGATTSvcClient::onNotify");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
        {
            WB_RES::PeerChange peerChange = rValue.convertTo<WB_RES::PeerChange>();
            if (peerChange.state == peerChange.state.DISCONNECTED)
            {
                // if connection is dropped, unsubscribe all data streams so that sensor does not stay on for no reason
                // unsubscribeAllStreams();
                DEBUGLOG("Peer change state: Disconnected");
            }
        }

            break;
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mSetEpochCharHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();

                DEBUGLOG("onNotify: mSetEpochCharHandle: len: %d", charValue.bytes.size());

                // TODO: I think this is where I would set the UTC time.
                // handleIncomingCommand(charValue.bytes);
                return;
            }
            else if (parameterRef.getCharHandle() == mGetEpochCharHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                // Update the notification state so we know if to forward data to datapipe
                mNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mGetEpochCharHandle. mNotificationsEnabled: %d", mNotificationsEnabled);
            }
            break;
        }
    }
}
