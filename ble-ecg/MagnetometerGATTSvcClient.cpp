#include "movesense.h"

#include <meas_magn/resources.h>

#include "MagnetometerGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


// Magnetometer GATT Service implementations:

const char* const MagnetometerGATTSvcClient::LAUNCHABLE_NAME = "MagnGattSvc";

MagnetometerGATTSvcClient::MagnetometerGATTSvcClient() :
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
    this->magnBuffer = new SeriesBuffer<mag_vec4_t>(this->mObjectSize, numberOfMovMagBuffers);
}

MagnetometerGATTSvcClient::~MagnetometerGATTSvcClient()
{
    delete this->magnBuffer;
}

bool MagnetometerGATTSvcClient::initModule()
{
    DEBUGLOG("MagnetometerGATTSvcClient::initModule");

    this->mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void MagnetometerGATTSvcClient::deinitModule()
{
    DEBUGLOG("MagnetometerGATTSvcClient::deinitModule");

    this->mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool MagnetometerGATTSvcClient::startModule()
{
    DEBUGLOG("MagnetometerGATTSvcClient::startModule");

    this->mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Set object size and allocate buffer.
    this->setObjectSize(DEFAULT_MOV_OBJECT_SIZE);
    // Set measurement interval to compute Magnetometer sampling frequency.
    this->setMeasurementInterval(DEFAULT_MOV_MEASUREMENT_INTERVAL);
    // Subscribe to Magnetometer samples with computed sampling frequency.
    this->subscribeToMagnSamples();

    // Configure GATT Service
    this->configGattSvc();

    return true;
}

void MagnetometerGATTSvcClient::stopModule()
{
    DEBUGLOG("MagnetometerGATTSvcClient::stopModule");

    // Unsubscribe from Magnetometer samples
    this->unsubscribeFromMagnSamples();
    this->mCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Measurement Interval GATT characteristic
    this->asyncUnsubscribe(this->mSampleRateCharResource);
    this->mSampleRateCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Object Size GATT characteristics.
    this->asyncUnsubscribe(this->mBufferSizeCharResource);
    this->mBufferSizeCharResource = wb::ID_INVALID_RESOURCE;

    this->mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void MagnetometerGATTSvcClient::onGetResult(wb::RequestId requestId,
                                   wb::ResourceId resourceId,
                                   wb::Result resultCode,
                                   const wb::Value& rResultData)
{
    DEBUGLOG("MagnetometerGATTSvcClient::onGetResult");

    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            DEBUGLOG("MagnetometerGATTSvcClient::onGetResult - COMM_BLE_GATTSVC_SVCHANDLE");

            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i = 0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));

                switch (uuid16)
                {
                    case MAGNETOMETER_CHARACTERISTIC_UUID16:
                        this->mCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case MAGNETOMETER_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16:
                        this->mSampleRateCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case MAGNETOMETER_OBJECT_SIZE_CHARACTERISTIC_UUID16:
                        this->mBufferSizeCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                }
            }

            // Force subscriptions asynchronously to save stack (will have stack overflow if not) 

            // Subscribe to listen to Movement Magnetometer Characteristics notifications (someone enables/disables the NOTIFY characteristic)
            this->asyncSubscribe(this->mCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Measurement Interval Characteristics notifications (someone writes new value to measurementIntervalChar) 
            this->asyncSubscribe(this->mSampleRateCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Object Size Characteristics notifications (someone writes new value to objectSizeChar)
            this->asyncSubscribe(this->mBufferSizeCharResource,  AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

void MagnetometerGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                    wb::ResourceId resourceId, 
                                    wb::Result resultCode, 
                                    const wb::Value& rResultData)
{
    DEBUGLOG("MagnetometerGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Magnetometer GATT service was created.
        this->mSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Magn GATT service was created. Handle: %d", this->mSvcHandle);

        // Request more info about created GATT service so we get the characteristics handles.
        this->asyncGet(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle
        );
    }
}

void MagnetometerGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                const wb::Value& value,
                                const wb::ParameterList& rParameters)
{
    DEBUGLOG("MagnetometerGATTSvcClient::onNotify");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEAS_MAGN_SAMPLERATE::LID:
        {
            // Get Magnetometer data
            auto magnData = value.convertTo<const WB_RES::MagnData&>();

            // Parse timestamp
            timestamp_t timestamp = (timestamp_t)magnData.timestamp;

            // Parse samples and put them into sample buffer
            size_t numberOfSamples = magnData.arrayMagn.getNumberOfItems();
            for (size_t i = 0; i < numberOfSamples; i++)
            {
                auto magnSample = this->convertMagnSample(magnData.arrayMagn[i], timestamp);
                // Add converted sample to Magnetometer buffer
                this->magnBuffer->addSample(magnSample);

                // If buffer is full, add timestamp and send samples.
                if (!this->magnBuffer->canAddSample())
                {
                    // Compute timestamp.
                    timestamp_t t = timestamp - ((numberOfSamples - i - 1) * this->measurementInterval);
                    // Set timestamp to timestamp of last sample in buffer.
                    this->magnBuffer->setTimestamp(t);
                    // Send samples.
                    bool result = this->sendMagnBuffer();
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

void MagnetometerGATTSvcClient::configGattSvc()
{
    // Define Magnetometer GATT Service and its Characteristics.
    WB_RES::GattSvc magnGattSvc;
    WB_RES::GattChar characteristics[3];
    WB_RES::GattChar &magnChar = characteristics[0];
    WB_RES::GattChar &measurementIntervalChar = characteristics[1];
    WB_RES::GattChar &objectSizeChar = characteristics[2];

    // Specify Characteristics's properties.
    WB_RES::GattProperty magnCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty measurementIntervalCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };
    WB_RES::GattProperty objectSizeCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };

    // Specify Magnetometer Characteristic
    magnChar.props = wb::MakeArray<WB_RES::GattProperty>(&magnCharProp, 1);
    magnChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&MAGNETOMETER_CHARACTERISTIC_UUID16), sizeof(uint16_t));

    // Specify Interval Characteristic
    measurementIntervalChar.props = wb::MakeArray<WB_RES::GattProperty>(measurementIntervalCharProps, 2);
    measurementIntervalChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&MAGNETOMETER_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    measurementIntervalChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->measurementInterval), sizeof(uint16_t));

    // Specify Size Characteristic
    objectSizeChar.props = wb::MakeArray<WB_RES::GattProperty>(objectSizeCharProps, 2);
    objectSizeChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&MAGNETOMETER_OBJECT_SIZE_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    objectSizeChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mObjectSize), sizeof(uint16_t));

    // Combine Characteristics to Service
    magnGattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&MAGNETOMETER_SERVICE_UUID16), sizeof(uint16_t));
    magnGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 3);

    // Create custom GATT Service.
    this->asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, magnGattSvc);
}

mag_vec4_t MagnetometerGATTSvcClient::convertMagnSample(whiteboard::FloatVector3D magVector,
                                              uint32_t timestamp)
{
    float magX = magVector.x * MAGNETOMETER_SCALING_FACTOR;
    float magY = magVector.y * MAGNETOMETER_SCALING_FACTOR;
    float magZ = magVector.z * MAGNETOMETER_SCALING_FACTOR;

    if (magX > MAX_MAG || magX < MIN_MAG)
    {
        magX = ERR_MAG;
    }
    if (magY > MAX_MAG || magY < MIN_MAG)
    {
        magY = ERR_MAG;
    }
    if (magZ > MAX_MAG || magZ < MIN_MAG)
    {
        magZ = ERR_MAG;
    }

    mag_vec4_t value;
    value.x = static_cast<mag_t>(magX);
    value.y = static_cast<mag_t>(magY);
    value.z = static_cast<mag_t>(magZ);
    value.timestamp = timestamp;
    return value;
}

bool MagnetometerGATTSvcClient::sendMagnBuffer()
{
    // Get the current buffer and its size.
    size_t size = this->magnBuffer->getSingleBufferSize();

    uint8_t* currentBuffer = this->magnBuffer->getCurrentBuffer();

    // Move to next message buffer.
    this->magnBuffer->switchBuffer();

    // Generate Magnetometer Characteristics value to send.
    WB_RES::Characteristic magnCharacteristic;
    magnCharacteristic.bytes = wb::MakeArray<uint8_t>(currentBuffer, size);

    // Send Magnetometer characteristics value.
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mCharHandle,
        magnCharacteristic
    );
    return true;
}

uint32_t MagnetometerGATTSvcClient::getSampleRate()
{
    return this->toSampleRate(this->measurementInterval);
}

uint32_t MagnetometerGATTSvcClient::toSampleRate(uint16_t interval)
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

void MagnetometerGATTSvcClient::setMeasurementInterval(uint16_t value)
{
    // Unsubscribe from current Magnetometer subscription
    this->unsubscribeFromMagnSamples();
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
    // Reset current Magnetometer buffers and start over.
    this->magnBuffer->resetCurrentBuffer();
    // Subscribe to new Magnetometer subscriptions.
    this->subscribeToMagnSamples();
}

void MagnetometerGATTSvcClient::subscribeToMagnSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Subscribe to Magnetometer samples with the desired Magnetometer sample rate.
    this->asyncSubscribe(
        WB_RES::LOCAL::MEAS_MAGN_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void MagnetometerGATTSvcClient::unsubscribeFromMagnSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Unsubscribe from Magnetometer samples with desired Magnetometer sample rate.
    this->asyncUnsubscribe(
        WB_RES::LOCAL::MEAS_MAGN_SAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void MagnetometerGATTSvcClient::setObjectSize(uint16_t value)
{
    // Set new object size.
    this->mObjectSize = value;
    // Change object size in buffers.
    this->magnBuffer->setLength((size_t)value);
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
