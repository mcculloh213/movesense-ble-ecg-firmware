#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

#include "SeriesBuffer.h"
#include "config.h"


class GyroscopeGATTSvcClient FINAL :
    private wb::ResourceClient,
    public wb::LaunchableModule
{
public:
    /** Name of the class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;

    /** Constructor for Gyroscope GATT Service. */
    GyroscopeGATTSvcClient();
    /** Destructor for Gyroscope GATT Service. */
    ~GyroscopeGATTSvcClient();

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
                             const wb::Value& rResultData);
    /** @see whiteboard::ResourceClient::onPostResult */
    virtual void onPostResult(wb::RequestId requestId,
                              wb::ResourceId resourceId,
                              wb::Result resultCode,
                              const wb::Value& rResultData) OVERRIDE;
    /** @see whiteboard::ResourceClient::onNotify */
    virtual void onNotify(wb::ResourceId resourceId,
                          const wb::Value& rValue,
                          const wb::ParameterList& rParameters);

private:

    // GATT Service:

    /** The handle of the Gyroscope GATT Service. */
    int32_t mSvcHandle;

    /**
     * @brief Configures the GATT Service.
     */
    void configGattSvc();


    // GATT Characteristics:

    /** The handle of the Gyroscope GATT Characteristics. */
    int32_t mCharHandle;
    /** The resource ID of the Gyroscope GATT Characteristics. */
    wb::ResourceId mCharResource;

    /** The handle of the Sample Rate GATT Characteristics. */
    int32_t mSampleRateCharHandle;
    /** The resource ID of the Sample Rate GATT Characteristics. */
    wb::ResourceId mSampleRateCharResource;

    /** The handle of the Gyroscope Buffer Size GATT Characteristics. */
    int32_t mBufferSizeCharHandle;
    /** The resource ID of the Gyroscope Buffer Size GATT Characteristics. */
    wb::ResourceId mBufferSizeCharResource;


    // Buffers:

    /** Buffer to hold Gyroscope samples. */
    SeriesBuffer<gyr_vec4_t>* pBuffer;


    // Gyroscope Samples:

    /**
     * @brief Converts the sensor's Gyroscope vector to a BLE Gyroscope vector.
     * 
     * @param gyroVector Sensor's Gyroscope vector to convert.
     * @param timestamp Sensor's timestamp.
     * @return gyr_vec4_t Converted BLE Gyroscope vector with timestamp.
     */
    gyr_vec4_t convertGyroSample(whiteboard::FloatVector3D gyroVector, uint32_t timestamp);

    /**
     * @brief Sends the Gyroscope buffer via BLE.
     * 
     * @return true Sending was successful.
     * @return false Sending was not successful since Gyroscope buffer is not initialized.
     */
    bool sendGyroBuffer();


    // Movement Measurement Interval:

    /**
     * @brief Interval between two Movement measurements in milliseconds.
     * Must be one of the following: 
     *  -  5 ms = 208 Hz
     *  - 10 ms = 104 Hz
     *  - 20 ms =  52 Hz
     *  - 40 ms =  26 Hz
     */
    uint16_t mMeasurementInterval;
    /**
     * @brief Gets the desired sampling rate.
     *  - 208 Hz =   5 ms
     *  - 104 Hz =  10 ms
     *  -  52 Hz =  20 ms
     *  -  26 Hz =  40 ms
     * 
     * @return uint32_t Desired sampling rate.
     */
    uint32_t getSampleRate();
    /**
     * @brief Converts a measurement interval to a sampling rate.
     *  - 208 Hz =   5 ms
     *  - 104 Hz =  10 ms
     *  -  52 Hz =  20 ms
     *  -  26 Hz =  40 ms
     * 
     * @param interval measurement interval.
     * @return uint32_t Sampling rate.
     */
    uint32_t toSampleRate(uint16_t interval);
    /**
     * @brief Set the interval between two Movement measurements in milliseconds.
     * 
     * @param value Movement measurement interval in milliseconds.
     */
    void setMeasurementInterval(uint16_t value);

    /**
     * @brief Subscribes to Gyroscope samples.
     */
    void subscribeToGyroSamples();
    /**
     * @brief Unsubscribes from Gyroscope samples.
     */
    void unsubscribeFromGyroSamples();

    /** Number of objects per sample message. */
    uint16_t mBufferSize;
    /**
     * @brief Sets the number of objects per sample message.
     * 
     * @param value Number of objects per sample message.
     */
    void setBufferSize(uint16_t value);
};