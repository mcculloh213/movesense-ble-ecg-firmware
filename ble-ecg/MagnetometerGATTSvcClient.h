#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

#include "SeriesBuffer.h"
#include "config.h"


class MagnetometerGATTSvcClient FINAL :
    private wb::ResourceClient,
    public wb::LaunchableModule
{
public:
    /** Name of the class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;

    /** Constructor for Magnetometer GATT Service. */
    MagnetometerGATTSvcClient();
    /** Destructor for Magnetometer GATT Service. */
    ~MagnetometerGATTSvcClient();

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

    /** The handle of the Magnetometer GATT Service. */
    int32_t mSvcHandle;

    /**
     * @brief Configures the GATT Service.
     */
    void configGattSvc();


    // GATT Characteristics:

    /** The handle of the Magnetometer GATT Characteristics. */
    int32_t mCharHandle;
    /** The resource ID of the Magnetometer GATT Characteristics. */
    wb::ResourceId mCharResource;

    /** The handle of the Sample Rate GATT Characteristics. */
    int32_t mSampleRateCharHandle;
    /** The resource ID of the Sample Rate GATT Characteristics. */
    wb::ResourceId mSampleRateCharResource;

    /** The handle of the Magnetometer Buffer Size GATT Characteristics. */
    int32_t mBufferSizeCharHandle;
    /** The resource ID of the Magnetometer Buffer Size GATT Characteristics. */
    wb::ResourceId mBufferSizeCharResource;


    // Buffers:

    /** Buffer to hold Magnetometer samples. */
    SeriesBuffer<mag_vec4_t>* magnBuffer;


    // Magnetometer Samples:

    /**
     * @brief Converts the sensor's Magnetometer vector to a BLE Magnetometer vector.
     * 
     * @param magnVector Sensor's Magnetometer vector to convert.
     * @param timestamp Sensor's timestamp.
     * @return mag_vec4_t Converted BLE Magnetometer vector with timestamp.
     */
    mag_vec4_t convertMagnSample(whiteboard::FloatVector3D magnVector, uint32_t timestamp);

    /**
     * @brief Sends the Magnetometer buffer via BLE.
     * 
     * @return true Sending was successful.
     * @return false Sending was not successful since Magnetometer buffer is not initialized.
     */
    bool sendMagnBuffer();


    // Movement Measurement Interval:

    /**
     * @brief Interval between two Movement measurements in milliseconds.
     * Must be one of the following: 
     *  -  5 ms = 208 Hz
     *  - 10 ms = 104 Hz
     *  - 20 ms =  52 Hz
     *  - 40 ms =  26 Hz
     */
    uint16_t measurementInterval;
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
     * @brief Subscribes to Magnetometer samples.
     */
    void subscribeToMagnSamples();
    /**
     * @brief Unsubscribes from Magnetometer samples.
     */
    void unsubscribeFromMagnSamples();

    /** Number of objects per sample message. */
    uint16_t mObjectSize;
    /**
     * @brief Sets the number of objects per sample message.
     * 
     * @param value Number of objects per sample message.
     */
    void setObjectSize(uint16_t value);
};