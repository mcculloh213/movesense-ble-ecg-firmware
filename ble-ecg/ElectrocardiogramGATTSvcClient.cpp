#include "movesense.h"

#include <meas_ecg/resources.h>

#include "ElectrocardiogramGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


// Electrocargiogram GATT Service implementations:

const char* const ElectrocardiogramGATTSvcClient::LAUNCHABLE_NAME = "EcgGattSvc";

ElectrocardiogramGATTSvcClient::ElectrocardiogramGATTSvcClient() :
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mSvcHandle(0),
    mCharHandle(0),
    mCharResource(wb::ID_INVALID_RESOURCE),
    mSampleRateCharHandle(0),
    mSampleRateCharResource(wb::ID_INVALID_RESOURCE),
    mBufferSizeCharHandle(0),
    mBufferSizeCharResource(wb::ID_INVALID_RESOURCE),
    mSampleCounter(0),
    mSampleSkipCount(2),
    mSampleRate(DEFAULT_ECG_MEASUREMENT_INTERVAL),
    mBufferSize(DEFAULT_ECG_OBJECT_SIZE)
{
    this->pBuffer = new SeriesBuffer<ecg_t>(this->mBufferSize, numberOfEcgBuffers);
}

ElectrocardiogramGATTSvcClient::~ElectrocardiogramGATTSvcClient()
{
    delete this->pBuffer;
    this->pBuffer = nullptr;
}

bool ElectrocardiogramGATTSvcClient::initModule()
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::initModule");

    this->mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void ElectrocardiogramGATTSvcClient::deinitModule()
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::deinitModule");

    this->mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool ElectrocardiogramGATTSvcClient::startModule()
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::startModule");

    this->mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Set ECG measurement interval to compute ECG sampling frequency.
    this->setSampleRate(DEFAULT_ECG_MEASUREMENT_INTERVAL);
    // Subscribe to ECG samples with computed ECG sampling frequency.
    this->subscribeToEcgSamples();

    // Configure Activity GATT Service.
    this->configGattSvc();

    return true;
}

void ElectrocardiogramGATTSvcClient::stopModule()
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::stopModule");

    // Unsubscribe from ECG samples.
    this->unsubscribeFromEcgSamples();
    this->mCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear ECG Measurement Interval GATT characteristics.
    this->asyncUnsubscribe(this->mSampleRateCharResource);
    this->mSampleRateCharResource = wb::ID_INVALID_RESOURCE;

    // Unsubscribe and clear Object Size GATT characteristics.
    this->asyncUnsubscribe(this->mBufferSizeCharResource);
    this->mBufferSizeCharResource = wb::ID_INVALID_RESOURCE;

    this->mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void ElectrocardiogramGATTSvcClient::onGetResult(wb::RequestId requestId,
                                                 wb::ResourceId resourceId,
                                                 wb::Result resultCode,
                                                 const wb::Value& rResultData)
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::onGetResult");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            DEBUGLOG("ElectrocardiogramGATTSvcClient::onGetResult - COMM_BLE_GATTSVC_SVCHANDLE");

            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc&>();
            for (size_t i = 0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));

                switch (uuid16)
                {
                    case ELECTROCARDIOGRAM_CHARACTERISTIC_UUID16:
                        this->mCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case ELECTROCARDIOGRAM_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16:
                        this->mSampleRateCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                    case ELECTROCARDIOGRAM_OBJECT_SIZE_CHARACTERISTIC_UUID16:
                        this->mBufferSizeCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                        break;
                }
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mSvcHandle, mCharHandle);
            getResource(pathBuffer, mCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mSvcHandle, mSampleRateCharHandle);
            getResource(pathBuffer, mSampleRateCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mSvcHandle, mBufferSizeCharHandle);
            auto res = getResource(pathBuffer, mBufferSizeCharResource);

            // Force subscriptions asynchronously to save stack (will have stack overflow if not) 

            // Subscribe to listen to Electrocardiogram Characteristics notifications (someone enables/disables the NOTIFY characteristic)
            this->asyncSubscribe(this->mCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Electrocardiogram Measurement Interval Characteristics notifications (someone writes new value to mSampleRateChar) 
            this->asyncSubscribe(this->mSampleRateCharResource, AsyncRequestOptions(NULL, 0, true));
            // Subscribe to listen to Electrocardiogram Object Size Characteristic notifications (someone writes a new value to mBufferSizeCharResource)
            this->asyncSubscribe(this->mBufferSizeCharResource, AsyncRequestOptions(NULL, 0, true));
            break;
        }
    }
}

void ElectrocardiogramGATTSvcClient::onPostResult(wb::RequestId requestId,
                                                  wb::ResourceId resourceId,
                                                  wb::Result resultCode,
                                                  const wb::Value& rResultData)
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Electrocardiogram GATT service was created.
        this->mSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Electrocardiogram GATT service was created. Handle: %d", this->mSvcHandle);

        // Request more info about created GATT service so we get the characteristics handles.
        this->asyncGet(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle
        );
    }
}

void ElectrocardiogramGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                              const wb::Value& rValue,
                                              const wb::ParameterList& rParameters)
{
    DEBUGLOG("ElectrocardiogramGATTSvcClient::onNotify");

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEAS_ECG_REQUIREDSAMPLERATE::LID:
        {
            // Get ECG data.
            auto ecgData = rValue.convertTo<const WB_RES::ECGData&>();

            // Parse timestamp
            timestamp_t timestamp = (timestamp_t)ecgData.timestamp;

            // Parse samples and put them into sample buffer.
            size_t numberOfSamples = ecgData.samples.getNumberOfItems();
            size_t j = 0;
            for (size_t i = 0; i < numberOfSamples; i++)
            {
                // Update ECG sample counter.
                this->mSampleCounter = (this->mSampleCounter + 1) % this->mSampleSkipCount;
                if (this->mSampleCounter == 0) {
                    // Convert ECG sample.
                    auto ecgSample = this->convertEcgSample(ecgData.samples[j]);
                    // Add converted sample to ECG buffer.
                    this->pBuffer->addSample(ecgSample);

                    // If buffer is full, add timestamp and send samples.
                    if (!this->pBuffer->canAddSample())
                    {
                        // Compute timestamp.
                        timestamp_t t = timestamp - ((numberOfSamples - i - 1) * this->mSampleRate);
                        // Set timestamp to timestamp of last sample in buffer.
                        this->pBuffer->setTimestamp(t);
                        // Send samples.
                        this->sendEcgBuffer();
                    }

                    j++;
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
                // Set Electrocardiogram Sample Rate GATT Characteristic:

                // Parse received sample rate.
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic&>();
                uint16_t sampleRate = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: Sample Rate: len: %d, new: %d", charValue.bytes.size(), sampleRate);

                // Update Electrocardiogram sample rate.
                this->setSampleRate(sampleRate);
            }
            else if (charHandle == this->mBufferSizeCharHandle)
            {
                // Set Object Size GATT Characteristic:

                // Parse received Object Size.
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic&>();
                uint16_t size = *reinterpret_cast<const uint16_t*>(&charValue.bytes[0]);

                DEBUGLOG("onNotify: Object Size: len: %d, new size: %d", charValue.bytes.size(), size);

                // Update the Object Size.
                this->setBufferSize(size);
            }
            break;
        }
    }
}

void ElectrocardiogramGATTSvcClient::configGattSvc()
{
    // Define Electrocardiogram GATT Service and its Characteristics.
    WB_RES::GattSvc gattSvc;
    WB_RES::GattChar characteristics[3];
    WB_RES::GattChar &svcChar = characteristics[0];
    WB_RES::GattChar &sampleRateChar = characteristics[1];
    WB_RES::GattChar &bufferSizeChar = characteristics[2];

    // Specify Characteristics' properties.
    WB_RES::GattProperty svcCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty sampleRateCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };
    WB_RES::GattProperty bufferSizeCharProps[2] = {
        WB_RES::GattProperty::READ,
        WB_RES::GattProperty::WRITE
    };

    // Specify Service Characteristic
    svcChar.props = wb::MakeArray<WB_RES::GattProperty>(&svcCharProp, 1);
    svcChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ELECTROCARDIOGRAM_CHARACTERISTIC_UUID16), sizeof(uint16_t));

    // Specify Sample Rate Characteristic
    sampleRateChar.props = wb::MakeArray<WB_RES::GattProperty>(sampleRateCharProps, 2);
    sampleRateChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ELECTROCARDIOGRAM_MEASUREMENT_INTERVAL_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    sampleRateChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mSampleRate), sizeof(uint16_t));

    // Specify Buffer Size Characteristic
    bufferSizeChar.props = wb::MakeArray<WB_RES::GattProperty>(bufferSizeCharProps, 2);
    bufferSizeChar.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ELECTROCARDIOGRAM_OBJECT_SIZE_CHARACTERISTIC_UUID16), sizeof(uint16_t));
    bufferSizeChar.initial_value = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&this->mBufferSize), sizeof(uint16_t));

    // Combine Characteristics to Service
    gattSvc.uuid = wb::MakeArray<uint8_t>(reinterpret_cast<const uint8_t*>(&ELECTROCARDIOGRAM_SERVICE_UUID16), sizeof(uint16_t));
    gattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 3);

    // Create custom GATT Service.
    this->asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, gattSvc);
}

// TODO: Does this need to be an int32?
ecg_t ElectrocardiogramGATTSvcClient::convertEcgSample(int32 ecgValue)
{
    if (ecgValue > ECG_MAX_VALUE)
    {
        return ECG_INVALID_VALUE;
    }
    else if (ecgValue < ECG_MIN_VALUE)
    {
        return ECG_INVALID_VALUE;
    }
    else
    {
        return (ecg_t)ecgValue;
    }
}

bool ElectrocardiogramGATTSvcClient::sendEcgBuffer()
{
    // Get the current buffer and its size.
    size_t size = this->pBuffer->getSingleBufferSize();
    uint8_t* currentBuffer = this->pBuffer->getCurrentBuffer();

    // Switch to next message buffer.
    this->pBuffer->switchBuffer();

    // Generate Electrocardiogram Characteristic value to send.
    WB_RES::Characteristic charData;
    charData.bytes = wb::MakeArray<uint8_t>(currentBuffer, size);

    // Send ECG Voltage characteristics value.
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mCharHandle,
        charData
    );
    return true;
}

uint32_t ElectrocardiogramGATTSvcClient::getSampleRate()
{
    return (uint32_t)(1000 / this->mSampleRate);
}

void ElectrocardiogramGATTSvcClient::setSampleRate(uint16_t value)
{
    // Ensure that value is valid or fall back to `DEFAULT_ECG_MEASUREMENT_INTERVAL`.
    switch (value)
    {
        case  2: // 500 Hz
        case  4: // 250 Hz
        case  8: // 125 Hz
        case 10: // 100 Hz
            break;
        default:
            value = DEFAULT_ECG_MEASUREMENT_INTERVAL;
            break;
    }

    // Unsubscribe from current ECG subscription.
    this->unsubscribeFromEcgSamples();
    // Update measurement interval.
    this->mSampleRate = value;
    // Update ECG sample skip count.
    this->mSampleSkipCount = value / ECG_BASE_MEASUREMENT_INTERVAL;
    // Reset ecg sample counter.
    this->mSampleCounter = 0;
    // Set measurement interval to GATT Characteristics value.
    WB_RES::Characteristic sampleRateChar;
    sampleRateChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->mSampleRate, sizeof(uint16_t));
    this->asyncPut(
        WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
        AsyncRequestOptions::Empty,
        this->mSvcHandle,
        this->mSampleRateCharHandle,
        sampleRateChar
    );
    // Reset current ECG buffer and start over.
    this->pBuffer->resetCurrentBuffer();
    // Subscribe to new ECG subscription.
    this->subscribeToEcgSamples();
}

void ElectrocardiogramGATTSvcClient::subscribeToEcgSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Subscribe to ECG samples with the desired ECG sample rate.
    this->asyncSubscribe(
        WB_RES::LOCAL::MEAS_ECG_REQUIREDSAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void ElectrocardiogramGATTSvcClient::unsubscribeFromEcgSamples()
{
    // Compute desired sample rate.
    uint32_t sampleRate = this->getSampleRate();
    // Unsubscribe from ECG samples with desired ECG sample rate.
    this->asyncUnsubscribe(
        WB_RES::LOCAL::MEAS_ECG_REQUIREDSAMPLERATE(),
        AsyncRequestOptions::Empty,
        sampleRate
    );
}

void ElectrocardiogramGATTSvcClient::setBufferSize(uint16_t value)
{
    // Set new object size.
    this->mBufferSize = value;
    // Change object size in buffer.
    this->pBuffer->setLength((size_t)value);
    // Set object size to GATT Characteristics value.
    if (this->mBufferSizeCharHandle != 0) {
        WB_RES::Characteristic bufferSizeChar;
        bufferSizeChar.bytes = wb::MakeArray<uint8_t>((uint8_t*)&this->mBufferSize, sizeof(uint16_t));
        asyncPut(
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(),
            AsyncRequestOptions::Empty,
            this->mSvcHandle,
            this->mBufferSizeCharHandle,
            bufferSizeChar
        );
    }
}