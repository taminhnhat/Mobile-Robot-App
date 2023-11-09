#include <Arduino.h>

class CurrentSensor
{
private:
    uint32_t sig_calibrate = 0;  //
    uint32_t sig_ins = 0;        //
    uint32_t sig_pre = 0;        //
    uint32_t sig_sum = 0;        //
    uint32_t sig_cou = 0;        //
    uint32_t sig_ave = 0;        // average signal
    uint32_t sig_exp_filter = 0; // recursive filtered signal
    uint32_t sig_ave_filter = 0; // running average filtered signal
    double cur_ins = 0;          // instant current in Ampe
    double cur_ave = 0;          // average current in Ampe
    double cur_ave_filter = 0;   // average current in Ampe
    double cur_rec_filter = 0;   // average current in Ampe
    uint32_t CURRENT_SENSOR_PIN;
    double alpha = 0.5;
    uint32_t filterSize = 20;
    uint32_t filterBuffer[20];
    uint32_t filterIndex = 0;
    bool ifBufferReady = false;

public:
    CurrentSensor(uint32_t PIN)
    {
        this->CURRENT_SENSOR_PIN = PIN;
    }
    void tick()
    {
        this->sig_pre = this->sig_ins;
        this->sig_ins = analogRead(this->CURRENT_SENSOR_PIN);
        // averaging
        this->sig_sum += this->sig_ins;
        this->sig_cou += 1;
        if (this->sig_cou >= 20)
        {
            this->sig_ave = this->sig_sum / this->sig_cou;
            this->sig_sum = 0;
            this->sig_cou = 0;
        }
        // recursive filter
        this->sig_exp_filter = this->alpha * this->sig_ins + (1 - alpha) * this->sig_pre;
        // average filter
        filterBuffer[filterIndex] = sig_ins;
        filterIndex += 1;
        if (filterIndex >= filterSize)
        {
            filterIndex = 0;
            if (!ifBufferReady)
                ifBufferReady = true;
        }
        if (ifBufferReady)
        {
            uint32_t sum = 0;
            for (uint32_t v : filterBuffer)
            {
                sum += v;
            }
            this->sig_ave_filter = sum / filterSize;
        }
    }
    void calibrate()
    {
        uint32_t i = 0;
        uint32_t sum = 0;
        while (i < 100)
        {
            sum += analogRead(CURRENT_SENSOR_PIN);
            delay(10);
            i += 1;
        }
        sig_calibrate = sum / 100;
    }
    uint32_t getCalibValue()
    {
        return sig_calibrate;
    }
    uint32_t getRawValue()
    {
        return this->sig_ins;
    }
    uint32_t getAverageRawValue()
    {
        return this->sig_ave;
    }
    uint32_t getRecursiveFilterRawValue()
    {
        return this->sig_exp_filter;
    }
    uint32_t getAverageFilterRawValue()
    {
        return this->sig_ave_filter;
    }
    double getInstantCurrent()
    {
        int16_t del = sig_ins - sig_calibrate;
        cur_ins = 0.016 * del;
        return cur_ins;
    }
    double getAverageCurrent()
    {
        int16_t del = sig_ave - sig_calibrate;
        cur_ave = 0.016 * del;
        return cur_ave;
    }
    double getAverageFilterCurrent()
    {
        int16_t del = sig_ave_filter - sig_calibrate;
        cur_ave_filter = 0.016 * del;
        return cur_ave_filter;
    }
    double getRecursiveFilterCurrent()
    {
        int16_t del = sig_exp_filter - sig_calibrate;
        cur_rec_filter = 0.016 * del;
        return cur_rec_filter;
    }
};
