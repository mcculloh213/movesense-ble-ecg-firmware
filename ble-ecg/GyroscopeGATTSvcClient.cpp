#include "movesense.h"

#include <meas_gyro/resources.h>

#include "GyroscopeGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


// Gyroscope GATT Service implementations:

const char* const GyroscopeGATTSvcClient::LAUNCHABLE_NAME = "GyroGattSvc";

GyroscopeGATTSvcClient::GyroscopeGATTSvcClient() :
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mSvcHandle(0),
    mCharHandle(0),
    mCharResource(wb::ID_INVALID_RESOURCE),
    mSampleRateCharHandle(0),
    mSampleRateCharResource(wb::ID_INVALID_RESOURCE),
    mBufferSizeCharHandle(0),
    mBufferSizeCharResource(wb::ID_INVALID_RESOURCE),
    measurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL),
    mObjectSize(DEFAULT_MOV_OBJECT_SIZE)
{
    this->gyroBuffer = new SeriesBuffer<gyr_vec4_t>(this->mObjectSize, numberOfMovGyrBuffers);
}

GyroscopeGATTSvcClient::~GyroscopeGATTSvcClient()
{
    delete this->gyroBuffer;
}

bool GyroscopeGATTSvcClient::initModule()
{
    DEBUGLOG("GyroscopeGATTSvcClient::initModule");

    this->mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void GyroscopeGATTSvcClient::deinitModule()
{
    DEBUGLOG("GyroscopeGATTSvcClient::deinitModule");

    this->mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool GyroscopeGATTSvcClient::startModule()
{
    DEBUGLOG("GyroscopeGATTSvcClient::startModule");

    this->mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Set object size and allocate buffer.
    this->setObjectSize(DEFAULT_MOV_OBJECT_SIZE);
    // Set measurement interval to compute Gyroscope sampling frequency.
    this->setMeasurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL);
    // Subscribe to Gyroscope samples with computed sampling frequency.
    this->subscribeToGyroSamples();

    // Configure GATT Service
    this->configGattSvc();

    return true;
}

void GyroscopeGATTSvcClient::stopModule()
{
    DEBUGLOG("GyroscopeGATTSvcClient::stopModule");

    // Unsubscribe from Gyroscope samples
    this->unsubscribeFromGyroSamples();
    this->mCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Measurement Interval GATT characteristic
    this->asyncUnsubscribe(this->mSampleRateCharResource);
    this->mSampleRateCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Object Size GATT characteristics.
    this->asyncUnsubscribe(this->mBufferSizeCharResource);
    this->mBufferSizeCharResource = wb::ID_INVALID_RESOURCE;

    this->mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void GyroscopeGATTSvcClient::onGetResult(wb::RequestId requestId,
                                   wb::ResourceId resourceId,
                                   wb::Result resultCode,
                                   const wb::Value& rResultData)
{
    DEBUGLOG("GyroscopeGATTSvcClient::onGetResult");

    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            DEBUGLOG("GyroscopeGATTSvcClient::onGetResult - COMM_BLE_GATTSVC_SVCHANDLE");

            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i = 0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));

                switch (uuid16)
                {
                    case GYROSCOPE_CHARACTERISTIC_UUID16:
                        this->mCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case GYROSCOPE_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16:
                        this->mSampleRateCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case GYROSCOPE_OBJECT_SIZE_CHARACTERISTIC_UUID16:
                        this->mBufferSizeCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                }
            }

            // Force subscriptions asynchronously to save stack (will have stack overflow if not) 

            // Subscribe to listen to Movement Gyroscope Characteristics notifications (someone enables/disables the NOTIFY characteristic)
            this->asyncSubscribe(this->mCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Measurement Interval Characteristics notifications (someone writes new value to measurementIntervalChar) 
            this->asyncSubscribe(this->mSampleRateCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Object Size Characteristics notifications (someone writes new value to objectSizeChar)
            this->asyncSubscribe(this->mBufferSizeCharResource,  AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

void GyroscopeGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                    wb::ResourceId resourceId, 
                                    wb::Result resultCode, 
                                    const wb::Value& rResultData)
{
    DEBUGLOG("GyroscopeGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Gyroscope GATT service was created.
        this->mSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Gyro GATT service was created. Handle: %d", this->mSvcHandle);

        // Request more info about created GATT service so we get the characteristics handles.
        this->asyncGet(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle
        );
    }
}

void GyroscopeGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                const wb::Value& value,
                                const wb::ParameterList& rParameters)
{
    DEBUGLOG("GyroscopeGATTSvcClient::onNotify");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE::LID:
        {
            // Get Gyroscope data
            auto gyroData = value.convertTo<const WB_RES::GyroData&>();

            // Parse timestamp
            timestamp_t timestamp = (timestamp_t)gyroData.timestamp;

            // Parse samples and put them into sample buffer
            size_t numberOfSamples = gyroData.arrayGyro.getNumberOfItems();
            for (size_t i = 0; i < numberOfSamples; i++)
            {
                auto gyroSample = this->convertGyroSample(gyroData.arrayGyro[i], timestamp);
                // Add converted sample to Gyroscope buffer
                this->gyroBuffer->addSample(gyroSample);

                // If buffer is full, add timestamp and send samples.
                if (!this->gyroBuffer->canAddSample())
                {
                    // Compute timestamp.
                    timestamp_t t = timestamp - ((numberOfSamples - i - 1) * this->measurementInterval);
                    // Set timestamp to timestamp of last sample in buffer.
                    this->gyroBuffer->setTimestamp(t);
                    // Send samples.
                    bool result = this->sendGyroBuffer();
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

                DEBUGLOG("onNotify: mObjectSize: len: %d, new interval: %d", charValue.bytes.size(), size);

                // Update the Object Size.
                this->setObjectSize(size);
            }
            break;
        }
    }
}

void GyroscopeGATTSvcClient::configGattSvc()
{
    // Define Gyroscope GATT Service and its Characteristics.
    WB_RES::GattSvc gyroGattSvc;
    WB_RES::GattChar characteristics[3];
    WB_RES::GattChar &gyroChar = characteristics[0];
    WB_RES::GattChar &measurementIntervalChar = characteristics[1];
    WB_RES::GattChar &objectSizeChar = characteristics[2];

    // Specify Characteristics's properties.
    WB_RES::GattProperty gyroCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty measurementIntervalCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };
    WB_RES::GattProperty objectSizeCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };

    // Specify Gyroscope Characteristic
    gyroChar.props = wb::MakeArray<WB_RES::GattProperty>(&gyroCharProp, 1);
    gyroChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&GYROSCOPE_CHARACTERISTIC_UUID16), sizeof(uint16_t));

    // Specify Interval Characteristic
    measurementIntervalChar.props = wb::MakeArray<WB_RES::GattProperty>(measurementIntervalCharProps, 2);
    measurementIntervalChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&GYROSCOPE_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    measurementIntervalChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->measurementInterval), sizeof(uint16_t));

    // Specify Size Characteristic
    objectSizeChar.props = wb::MakeArray<WB_RES::GattProperty>(objectSizeCharProps, 2);
    objectSizeChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&GYROSCOPE_OBJECT_SIZE_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    objectSizeChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mObjectSize), sizeof(uint16_t));

    // Combine Characteristics to Service
    gyroGattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&GYROSCOPE_SERVICE_UUID16), sizeof(uint16_t));
    gyroGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 3);

    // Create custom GATT Service.
    this->asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, gyroGattSvc);
}

gyr_vec4_t GyroscopeGATTSvcClient::convertGyroSample(whiteboard::FloatVector3D gyroVector,
                                              uint32_t timestamp)
{
    float gyrX = gyroVector.x * GYROSCOPE_SCALING_FACTOR;
    float gyrY = gyroVector.y * GYROSCOPE_SCALING_FACTOR;
    float gyrZ = gyroVector.z * GYROSCOPE_SCALING_FACTOR;

    if (gyrX > MAX_GYR || gyrX < MIN_GYR)
    {
        gyrX = ERR_GYR;
    }
    if (gyrY > MAX_GYR || gyrY < MIN_GYR)
    {
        gyrY = ERR_GYR;
    }
    if (gyrZ > MAX_GYR || gyrZ < MIN_GYR)
    {
        gyrZ = ERR_GYR;
    }

    gyr_vec4_t value;
    value.x = static_cast<gyr_t>(gyrX);
    value.y = static_cast<gyr_t>(gyrY);
    value.z = static_cast<gyr_t>(gyrZ);
    value.timestamp = timestamp;
    return value;
}

bool GyroscopeGATTSvcClient::sendGyroBuffer()
{
    // Get the current buffer and its size.
    size_t size = this->gyroBuffer->getSingleBufferSize();

    uint8_t* currentBuffer = this->gyroBuffer->getCurrentBuffer();

    // Move to next message buffer.
    this->gyroBuffer->switchBuffer();

    // Generate Gyroscope Characteristics value to send.
    WB_RES::Characteristic gyroCharacteristic;
    gyroCharacteristic.bytes = wb::MakeArray<uint8_t>(currentBuffer, size);

    // Send Gyroscope characteristics value.
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mCharHandle,
        gyroCharacteristic
    );
    return true;
}

uint32_t GyroscopeGATTSvcClient::getSampleRate()
{
    return this->toSampleRate(this->measurementInterval);
}

uint32_t GyroscopeGATTSvcClient::toSampleRate(uint16_t interval)
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

void GyroscopeGATTSvcClient::setMeasurementInterval(uint16_t value)
{
    // Unsubscribe from current Gyroscope subscription
    this->unsubscribeFromGyroSamples();
    // Update measurement interval.
    this->measurementInterval = value;
    // Set measurement interval to GATT Characteristics value.
    WB_RES::Characteristic measurementIntervalChar;
    measurementIntervalChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->measurementInterval, sizeof(uint16_t));
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mSampleRateCharHandle,
        measurementIntervalChar
    );
    // Reset current Gyroscope buffers and start over.
    this->gyroBuffer->resetCurrentBuffer();
    // Subscribe to new Gyroscope subscriptions.
    this->subscribeToGyroSamples();
}

void GyroscopeGATTSvcClient::subscribeToGyroSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Subscribe to Gyroscope samples with the desired Gyroscope sample rate.
    this->asyncSubscribe(
        WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void GyroscopeGATTSvcClient::unsubscribeFromGyroSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Unsubscribe from Gyroscope samples with desired Gyroscope sample rate.
    this->asyncUnsubscribe(
        WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void GyroscopeGATTSvcClient::setObjectSize(uint16_t value)
{
    // Set new object size.
    this->mObjectSize = value;
    // Change object size in buffers.
    this->gyroBuffer->setLength((size_t)value);
    // Set object size to GATT Characteristics value.
    if (this->mBufferSizeCharHandle != 0) {
        WB_RES::Characteristic objectSizeChar;
        objectSizeChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->mObjectSize, sizeof(uint16_t));
        asyncPut(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle,
            this->mBufferSizeCharHandle,
            objectSizeChar
        );
    }
}
