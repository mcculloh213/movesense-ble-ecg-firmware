#include "movesense.h"

#include <meas_acc/resources.h>

#include "AccGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


// Accelerometer GATT Service implementations:

const char* const AccGATTSvcClient::LAUNCHABLE_NAME = "AccGattSvc";

AccGATTSvcClient::AccGATTSvcClient() :
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mAccSvcHandle(0),
    mAccCharHandle(0),
    mAccCharResource(wb::ID_INVALID_RESOURCE),
    mMeasurementIntervalCharHandle(0),
    mMeasurementIntervalCharResource(wb::ID_INVALID_RESOURCE),
    mObjectSizeCharHandle(0),
    mObjectSizeCharResource(wb::ID_INVALID_RESOURCE),
    measurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL),
    objectSize(DEFAULT_MOV_OBJECT_SIZE)
{
    this->accBuffer = new SeriesBuffer<acc_vec4_t>(this->objectSize, numberOfMovAccBuffers);
}

AccGATTSvcClient::~AccGATTSvcClient()
{
    delete this->accBuffer;
}

bool AccGATTSvcClient::initModule()
{
    DEBUGLOG("AccGATTSvcClient::initModule");

    this->mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void AccGATTSvcClient::deinitModule()
{
    DEBUGLOG("AccGATTSvcClient::deinitModule");

    this->mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool AccGATTSvcClient::startModule()
{
    DEBUGLOG("AccGATTSvcClient::startModule");

    this->mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Set object size and allocate buffer.
    this->setObjectSize(DEFAULT_MOV_OBJECT_SIZE);
    // Set measurement interval to compute Accelerometer sampling frequency.
    this->setMeasurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL);
    // Subscribe to Accelerometer samples with computed sampling frequency.
    this->subscribeToAccSamples();

    // Configure GATT Service
    this->configGattSvc();

    return true;
}

void AccGATTSvcClient::stopModule()
{
    DEBUGLOG("AccGATTSvcClient::stopModule");

    // Unsubscribe from Accelerometer samples
    this->unsubscribeFromAccSamples();
    this->mAccCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Measurement Interval GATT characteristic
    this->asyncUnsubscribe(this->mMeasurementIntervalCharResource);
    this->mMeasurementIntervalCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Object Size GATT characteristics.
    this->asyncUnsubscribe(this->mObjectSizeCharResource);
    this->mObjectSizeCharResource = wb::ID_INVALID_RESOURCE;

    this->mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void AccGATTSvcClient::onGetResult(wb::RequestId requestId,
                                   wb::ResourceId resourceId,
                                   wb::Result resultCode,
                                   const wb::Value& rResultData)
{
    DEBUGLOG("AccGATTSvcClient::onGetResult");

    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            DEBUGLOG("AccGATTSvcClient::onGetResult - COMM_BLE_GATTSVC_SVCHANDLE");

            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i = 0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));

                switch (uuid16)
                {
                    case accCharUUID16:
                        this->mAccCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case accMeasurementIntervalCharUUID16:
                        this->mMeasurementIntervalCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case accObjectSizeCharUUID16:
                        this->mObjectSizeCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                }
            }

            // Force subscriptions asynchronously to save stack (will have stack overflow if not) 

            // Subscribe to listen to Movement Acceleration Characteristics notifications (someone enables/disables the NOTIFY characteristic)
            this->asyncSubscribe(this->mAccCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Measurement Interval Characteristics notifications (someone writes new value to measurementIntervalChar) 
            this->asyncSubscribe(this->mMeasurementIntervalCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Object Size Characteristics notifications (someone writes new value to objectSizeChar)
            this->asyncSubscribe(this->mObjectSizeCharResource,  AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

void AccGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                    wb::ResourceId resourceId, 
                                    wb::Result resultCode, 
                                    const wb::Value& rResultData)
{
    DEBUGLOG("AccGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Accelerometer GATT service was created.
        this->mAccSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("MOV GATT service was created. Handle: %d", this->mAccSvcHandle);

        // Request more info about created GATT service so we get the characteristics handles.
        this->asyncGet(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(),
            AsyncRequestOptions::Empty,
            this->mAccSvcHandle
        );
    }
}

void AccGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                const wb::Value& value,
                                const wb::ParameterList& rParameters)
{
    DEBUGLOG("AccGATTSvcClient::onNotify");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::LID:
        {
            // Get Acceleration data
            auto accData = value.convertTo<const WB_RES::AccData&>();

            // Parse timestamp
            timestamp_t timestamp = (timestamp_t)accData.timestamp;

            // Parse samples and put them into sample buffer
            size_t numberOfSamples = accData.arrayAcc.getNumberOfItems();
            for (size_t i = 0; i < numberOfSamples; i++)
            {
                auto accSample = this->convertAccSample(accData.arrayAcc[i], timestamp);
                // Add converted sample to Acceleration buffer
                this->accBuffer->addSample(accSample);

                // If buffer is full, add timestamp and send samples.
                if (!this->accBuffer->canAddSample())
                {
                    // Compute timestamp.
                    timestamp_t t = timestamp - ((numberOfSamples - i - 1) * this->measurementInterval);
                    // Set timestamp to timestamp of last sample in buffer.
                    this->accBuffer->setTimestamp(t);
                    // Send samples.
                    bool result = this->sendAccBuffer();
                }
            }
            break;
        }
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            // Get the GATT characteristic which is handled now.
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            auto charHandle = parameterRef.getCharHandle();

            // Set the current GATT characteristic.
            if (charHandle == this->mMeasurementIntervalCharHandle)
            {
                // Set Measurement Interval GATT Characteristic:

                // Parse received Measurement Interval.
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic&>();
                uint16_t interval = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: MeasurementInterval: len: %d, new interval: %d", charValue.bytes.size(), interval);

                // Update the Measurement Interval.
                this->setMeasurementInterval(interval);
            }
            else if (charHandle == this->mObjectSizeCharHandle)
            {
                // Set Object Size GATT Characteristic:

                // Parse received Object Size.
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic&>();
                uint16_t size = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: objectSize: len: %d, new interval: %d", charValue.bytes.size(), size);

                // Update the Object Size.
                this->setObjectSize(size);
            }
            break;
        }
    }
}

void AccGATTSvcClient::configGattSvc()
{
    // Define Accelerometer GATT Service and its Characteristics.
    WB_RES::GattSvc accGattSvc;
    WB_RES::GattChar characteristics[3];
    WB_RES::GattChar &accChar = characteristics[0];
    WB_RES::GattChar &measurementIntervalChar = characteristics[1];
    WB_RES::GattChar &objectSizeChar = characteristics[2];

    // Specify Characteristics's properties.
    WB_RES::GattProperty accCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty measurementIntervalCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };
    WB_RES::GattProperty objectSizeCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };

    // Specify Acceleration Characteristic
    accChar.props = wb::MakeArray<WB_RES::GattProperty>(&accCharProp, 1);
    accChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&accCharUUID16), sizeof(uint16_t));

    // Specify Interval Characteristic
    measurementIntervalChar.props = wb::MakeArray<WB_RES::GattProperty>(measurementIntervalCharProps, 2);
    measurementIntervalChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&accMeasurementIntervalCharUUID16), sizeof(uint16_t));
    measurementIntervalChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->measurementInterval), sizeof(uint16_t));

    // Specify Size Characteristic
    objectSizeChar.props = wb::MakeArray<WB_RES::GattProperty>(objectSizeCharProps, 2);
    objectSizeChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&accObjectSizeCharUUID16), sizeof(uint16_t));
    objectSizeChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->objectSize), sizeof(uint16_t));

    // Combine Characteristics to Service
    accGattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&accSvcUUID16), sizeof(uint16_t));
    accGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 3);

    // Create custom GATT Service.
    this->asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, accGattSvc);
}

acc_vec4_t AccGATTSvcClient::convertAccSample(whiteboard::FloatVector3D accVector,
                                              uint32_t timestamp)
{
    acc_vec4_t value;
    value.x = (acc_t)accVector.x;
    value.y = (acc_t)accVector.y;
    value.z = (acc_t)accVector.z;
    value.timestamp = timestamp;
    return value;
}

bool AccGATTSvcClient::sendAccBuffer()
{
    // Get the current buffer and its size.
    size_t size = this->accBuffer->getSingleBufferSize();

    uint8_t* currentBuffer = this->accBuffer->getCurrentBuffer();

    // Move to next message buffer.
    this->accBuffer->switchBuffer();

    // Generate Acceleration Characteristics value to send.
    WB_RES::Characteristic movAccCharacteristic;
    movAccCharacteristic.bytes = wb::MakeArray<uint8_t>(currentBuffer, size);

    // Send Acceleration characteristics value.
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mAccSvcHandle,
        this->mAccCharHandle,
        movAccCharacteristic
    );
    return true;
}

uint32_t AccGATTSvcClient::getSampleRate()
{
    return this->toSampleRate(this->measurementInterval);
}

uint32_t AccGATTSvcClient::toSampleRate(uint16_t interval)
{
    switch (interval)
    {
        case 5:
            return 208;
        case 10:
            return 104;
        case 20:
            return 52;
        case 40:
            return 26;
        default:
            return 52;
    }
}

void AccGATTSvcClient::setMeasurementInterval(uint16_t value)
{
    // Unsubscribe from current Accelerometer subscription
    this->unsubscribeFromAccSamples();
    // Update measurement interval.
    this->measurementInterval = value;
    // Set measurement interval to GATT Characteristics value.
    WB_RES::Characteristic measurementIntervalChar;
    measurementIntervalChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->measurementInterval, sizeof(uint16_t));
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mAccSvcHandle,
        this->mMeasurementIntervalCharHandle,
        measurementIntervalChar
    );
    // Reset current Accelerometer buffers and start over.
    this->accBuffer->resetCurrentBuffer();
    // Subscribe to new Accelerometer subscriptions.
    this->subscribeToAccSamples();
}

void AccGATTSvcClient::subscribeToAccSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Subscribe to Acceleration samples with the desired Acceleration sample rate.
    this->asyncSubscribe(
        WB_RES::LOCAL::MEAS_ACC_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void AccGATTSvcClient::unsubscribeFromAccSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Unsubscribe from Acceleration samples with desired Acceleration sample rate.
    this->asyncUnsubscribe(
        WB_RES::LOCAL::MEAS_ACC_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void AccGATTSvcClient::setObjectSize(uint16_t value)
{
    // Set new object size.
    this->objectSize = value;
    // Change object size in buffers.
    this->accBuffer->setLength((size_t)value);
    // Set object size to GATT Characteristics value.
    if (this->mObjectSizeCharHandle != 0) {
        WB_RES::Characteristic objectSizeChar;
        objectSizeChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->objectSize, sizeof(uint16_t));
        asyncPut(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
            AsyncRequestOptions::Empty,
            this->mAccSvcHandle,
            this->mObjectSizeCharHandle,
            objectSizeChar
        );
    }
}
