#include "movesense.h"

#include <meas_acc/resources.h>

#include "AccelerometerGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


// Accelerometer GATT Service implementations:

const char* const AccelerometerGATTSvcClient::LAUNCHABLE_NAME = "AccGattSvc";

AccelerometerGATTSvcClient::AccelerometerGATTSvcClient() :
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mSvcHandle(0),
    mCharHandle(0),
    mCharResource(wb::ID_INVALID_RESOURCE),
    mSampleRateCharHandle(0),
    mSampleRateCharResource(wb::ID_INVALID_RESOURCE),
    mBufferSizeCharHandle(0),
    mBufferSizeCharResource(wb::ID_INVALID_RESOURCE),
    mMeasurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL),
    mBufferSize(DEFAULT_MOV_OBJECT_SIZE)
{
    this->pBuffer = new SeriesBuffer<acc_vec4_t>(this->mBufferSize, numberOfMovAccBuffers);
}

AccelerometerGATTSvcClient::~AccelerometerGATTSvcClient()
{
    delete this->pBuffer;
}

bool AccelerometerGATTSvcClient::initModule()
{
    DEBUGLOG("AccelerometerGATTSvcClient::initModule");

    this->mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void AccelerometerGATTSvcClient::deinitModule()
{
    DEBUGLOG("AccelerometerGATTSvcClient::deinitModule");

    this->mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool AccelerometerGATTSvcClient::startModule()
{
    DEBUGLOG("AccelerometerGATTSvcClient::startModule");

    this->mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Set object size and allocate buffer.
    this->setBufferSize(DEFAULT_MOV_OBJECT_SIZE);
    // Set measurement interval to compute Accelerometer sampling frequency.
    this->setMeasurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL);
    // Subscribe to Accelerometer samples with computed sampling frequency.
    this->subscribeToAccSamples();

    // Configure GATT Service
    this->configGattSvc();

    return true;
}

void AccelerometerGATTSvcClient::stopModule()
{
    DEBUGLOG("AccelerometerGATTSvcClient::stopModule");

    // Unsubscribe from Accelerometer samples
    this->unsubscribeFromAccSamples();
    this->mCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Measurement Interval GATT characteristic
    this->asyncUnsubscribe(this->mSampleRateCharResource);
    this->mSampleRateCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Object Size GATT characteristics.
    this->asyncUnsubscribe(this->mBufferSizeCharResource);
    this->mBufferSizeCharResource = wb::ID_INVALID_RESOURCE;

    this->mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void AccelerometerGATTSvcClient::onGetResult(wb::RequestId requestId,
                                   wb::ResourceId resourceId,
                                   wb::Result resultCode,
                                   const wb::Value& rResultData)
{
    DEBUGLOG("AccelerometerGATTSvcClient::onGetResult");

    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            DEBUGLOG("AccelerometerGATTSvcClient::onGetResult - COMM_BLE_GATTSVC_SVCHANDLE");

            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i = 0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));

                switch (uuid16)
                {
                    case ACCELEROMETER_CHARACTERISTIC_UUID16:
                        this->mCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case ACCELEROMETER_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16:
                        this->mSampleRateCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case ACCELEROMETER_OBJECT_SIZE_CHARACTERISTIC_UUID16:
                        this->mBufferSizeCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                }
            }

            // Force subscriptions asynchronously to save stack (will have stack overflow if not) 

            // Subscribe to listen to Movement Acceleration Characteristics notifications (someone enables/disables the NOTIFY characteristic)
            this->asyncSubscribe(this->mCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Measurement Interval Characteristics notifications (someone writes new value to mSampleRateCharResource) 
            this->asyncSubscribe(this->mSampleRateCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Object Size Characteristics notifications (someone writes new value to mBufferSizeCharResource)
            this->asyncSubscribe(this->mBufferSizeCharResource,  AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

void AccelerometerGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                    wb::ResourceId resourceId, 
                                    wb::Result resultCode, 
                                    const wb::Value& rResultData)
{
    DEBUGLOG("AccelerometerGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Accelerometer GATT service was created.
        this->mSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("ACC GATT service was created. Handle: %d", this->mSvcHandle);

        // Request more info about created GATT service so we get the characteristics handles.
        this->asyncGet(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle
        );
    }
}

void AccelerometerGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                const wb::Value& value,
                                const wb::ParameterList& rParameters)
{
    DEBUGLOG("AccelerometerGATTSvcClient::onNotify");

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
                this->pBuffer->addSample(accSample);

                // If buffer is full, add timestamp and send samples.
                if (!this->pBuffer->canAddSample())
                {
                    // Compute timestamp.
                    timestamp_t t = timestamp - ((numberOfSamples - i - 1) * this->mMeasurementInterval);
                    // Set timestamp to timestamp of last sample in buffer.
                    this->pBuffer->setTimestamp(t);
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
            if (charHandle == this->mSampleRateCharHandle)
            {
                // Set Measurement Interval GATT Characteristic:

                // Parse received Measurement Interval.
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic&>();
                uint16_t interval = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: MeasurementInterval: len: %d, new interval: %d", charValue.bytes.size(), interval);

                // Update the Measurement Interval.
                this->setMeasurementInterval(interval);
            }
            else if (charHandle == this->mBufferSizeCharHandle)
            {
                // Set Object Size GATT Characteristic:

                // Parse received Object Size.
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic&>();
                uint16_t size = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: mBufferSize: len: %d, new size: %d", charValue.bytes.size(), size);

                // Update the Object Size.
                this->setBufferSize(size);
            }
            break;
        }
    }
}

void AccelerometerGATTSvcClient::configGattSvc()
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
    accChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ACCELEROMETER_CHARACTERISTIC_UUID16), sizeof(uint16_t));

    // Specify Interval Characteristic
    measurementIntervalChar.props = wb::MakeArray<WB_RES::GattProperty>(measurementIntervalCharProps, 2);
    measurementIntervalChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ACCELEROMETER_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    measurementIntervalChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mMeasurementInterval), sizeof(uint16_t));

    // Specify Size Characteristic
    objectSizeChar.props = wb::MakeArray<WB_RES::GattProperty>(objectSizeCharProps, 2);
    objectSizeChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ACCELEROMETER_OBJECT_SIZE_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    objectSizeChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mBufferSize), sizeof(uint16_t));

    // Combine Characteristics to Service
    accGattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ACCELEROMETER_SERVICE_UUID16), sizeof(uint16_t));
    accGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 3);

    // Create custom GATT Service.
    this->asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, accGattSvc);
}

acc_vec4_t AccelerometerGATTSvcClient::convertAccSample(whiteboard::FloatVector3D accVector,
                                              uint32_t timestamp)
{
    float accX = accVector.x * ACCELEROMETER_SCALEING_FACTOR;
    float accY = accVector.y * ACCELEROMETER_SCALEING_FACTOR;
    float accZ = accVector.z * ACCELEROMETER_SCALEING_FACTOR;

    if (accX > MAX_ACC || accX < MIN_ACC)
    {
        accX = ERR_ACC;
    }
    if (accY > MAX_ACC || accY < MIN_ACC)
    {
        accY = ERR_ACC;
    }
    if (accZ > MAX_ACC || accZ < MIN_ACC)
    {
        accZ = ERR_ACC;
    }

    acc_vec4_t value;
    value.x = static_cast<acc_t>(accX);
    value.y = static_cast<acc_t>(accY);
    value.z = static_cast<acc_t>(accZ);
    value.timestamp = timestamp;
    return value;
}

bool AccelerometerGATTSvcClient::sendAccBuffer()
{
    // Get the current buffer and its size.
    size_t size = this->pBuffer->getSingleBufferSize();

    uint8_t* currentBuffer = this->pBuffer->getCurrentBuffer();

    // Move to next message buffer.
    this->pBuffer->switchBuffer();

    // Generate Acceleration Characteristics value to send.
    WB_RES::Characteristic accCharacteristic;
    accCharacteristic.bytes = wb::MakeArray<uint8_t>(currentBuffer, size);

    // Send Acceleration characteristics value.
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mCharHandle,
        accCharacteristic
    );
    return true;
}

uint32_t AccelerometerGATTSvcClient::getSampleRate()
{
    return this->toSampleRate(this->mMeasurementInterval);
}

uint32_t AccelerometerGATTSvcClient::toSampleRate(uint16_t interval)
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

void AccelerometerGATTSvcClient::setMeasurementInterval(uint16_t value)
{
    // Unsubscribe from current Accelerometer subscription
    this->unsubscribeFromAccSamples();
    // Update measurement interval.
    this->mMeasurementInterval = value;
    // Set measurement interval to GATT Characteristics value.
    WB_RES::Characteristic measurementIntervalChar;
    measurementIntervalChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->mMeasurementInterval, sizeof(uint16_t));
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mSampleRateCharHandle,
        measurementIntervalChar
    );
    // Reset current Accelerometer buffers and start over.
    this->pBuffer->resetCurrentBuffer();
    // Subscribe to new Accelerometer subscriptions.
    this->subscribeToAccSamples();
}

void AccelerometerGATTSvcClient::subscribeToAccSamples()
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

void AccelerometerGATTSvcClient::unsubscribeFromAccSamples()
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

void AccelerometerGATTSvcClient::setBufferSize(uint16_t value)
{
    // Set new object size.
    this->mBufferSize = value;
    // Change object size in buffers.
    this->pBuffer->setLength((size_t)value);
    // Set object size to GATT Characteristics value.
    if (this->mBufferSizeCharHandle != 0) {
        WB_RES::Characteristic objectSizeChar;
        objectSizeChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->mBufferSize, sizeof(uint16_t));
        asyncPut(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle,
            this->mBufferSizeCharHandle,
            objectSizeChar
        );
    }
}
