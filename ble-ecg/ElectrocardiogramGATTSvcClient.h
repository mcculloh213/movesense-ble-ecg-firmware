#pragma once

#include <memory>
#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

#include "SeriesBuffer.h"
#include "config.h"


// Electrocardiogram GATT Service:

class ElectrocardiogramGATTSvcClient FINAL :
    private wb::ResourceClient,
    public wb::LaunchableModule
{
public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;

    /** Constructor for Electrocardiogram GATT Service. */
    ElectrocardiogramGATTSvcClient();
    /** Destructor for Electrocardiogram GATT Service. */
    virtual ~ElectrocardiogramGATTSvcClient();

private:
    
    // Module:

    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;


    // Subscriptions:

    /** @see whiteboard::ResourceClient::onGetResult */
    virtual void onGetResult(wb::RequestId requestId,
                             wb::ResourceId resourceId,
                             wb::Result resultCode,
                             const wb::Value& rResultData) OVERRIDE;
    /** @see whiteboard::ResourceClient::onPostResult */
    virtual void onPostResult(wb::RequestId requestId,
                              wb::ResourceId resourceId,
                              wb::Result resultCode,
                              const wb::Value& rResultData) OVERRIDE;
    /** @see whiteboard::ResourceClient::onNotify */
    virtual void onNotify(wb::ResourceId resourceId,
                          const wb::Value& rValue,
                          const wb::ParameterList& rParameters) OVERRIDE;

private:

    // GATT Service:

    /** The handle of the Accelerometer GATT Service. */
    int32_t mSvcHandle;

    /**
     * @brief Configures the GATT Service.
     */
    void configGattSvc();


    // GATT Characteristics:

    /** The handle of the Accelerometer GATT Characteristics. */
    int32_t mCharHandle;
    /** The resource ID of the Accelerometer GATT Characteristics. */
    wb::ResourceId mCharResource;

    /** The handle of the Sample Rate GATT Characteristics. */
    int32_t mSampleRateCharHandle;
    /** The resource ID of the Sample Rate GATT Characteristics. */
    wb::ResourceId mSampleRateCharResource;

    /** The handle of the Accelerometer Buffer Size GATT Characteristics. */
    int32_t mBufferSizeCharHandle;
    /** The resource ID of the Accelerometer Buffer Size GATT Characteristics. */
    wb::ResourceId mBufferSizeCharResource;


    // Buffers:

    /** Buffer to hold ECG samples. */
    SeriesBuffer<ecg_t>* pBuffer;

    // ECG Samples:

    uint16_t mSampleCounter;
    uint16_t mSampleSkipCount;

    /**
     * @brief Converts the sensor's ECG value to a BLE ECG value.
     * 
     * @param ecgValue Sensor's ECG value to convert.
     * @return ecg_t Converted BLE ECG value.
     */
    ecg_t convertEcgSample(int32 ecgValue);

    /**
     * @brief Sends the ECG buffer via BLE.
     * 
     * @return true Sending was successful.
     * @return false Sending was not successful since ECG buffer is not initialized.
     */
    bool sendEcgBuffer();


    // ECG Measurement Interval:

    /**
     * @brief Interval between two ECG measurements in milliseconds.
     * Must be one of the following: 
     *  -  2 ms =  500 Hz
     *  -  4 ms =  250 Hz
     *  -  5 ms =  200 Hz
     *  -  8 ms =  125 Hz
     *  - 10 ms =  100 Hz
     */
    uint16_t mSampleRate;
    /**
     * @brief Gets the desired ECG sampling rate.
     *  -  500 Hz =  2 ms
     *  -  250 Hz =  4 ms
     *  -  200 Hz =  5 ms
     *  -  125 Hz =  8 ms
     *  -  100 Hz = 10 ms
     * 
     * @return uint32_t Desired ECG sampling rate.
     */
    uint32_t getSampleRate();
    /**
     * @brief Converts a measurement interval to a sampling rate.
     *  -  500 Hz =  2 ms
     *  -  250 Hz =  4 ms
     *  -  200 Hz =  5 ms
     *  -  125 Hz =  8 ms
     *  -  100 Hz = 10 ms
     * 
     * @param interval measurement interval.
     * @return uint32_t Sampling rate.
     */
    uint32_t toSampleRate(uint16_t interval);
    /**
     * @brief Set the interval between two ECG measurements in milliseconds.
     * 
     * @param value ECG measurement interval in milliseconds.
     */
    void setMeasurementInterval(uint16_t value);

    /**
     * @brief Subscribes to ECG samples.
     */
    void subscribeToEcgSamples();
    /**
     * @brief Unsubscribes from ECG samples.
     */
    void unsubscribeFromEcgSamples();

    /** Number of objects per sample message. */
    uint16_t mObjectSize;
    /**
     * @brief Sets the number of objects per sample message.
     * 
     * @param value Number of objects per sample message.
     */
    void setObjectSize(uint16_t value);
};