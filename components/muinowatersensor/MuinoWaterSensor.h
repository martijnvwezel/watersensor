#pragma once

#include "esphome.h"
using namespace esphome;

#define SMOOTHING_FACTOR 3 // * 2 - 10
#define ALPHA_COR 0.1      // * Value between 0-1
#define AC_STEPS 16        // * Given pi/3 coarse estimate of phase, calculate autocorrelation of signals within that pi/3 range
#define UPDATE_RATE_LITERS 10*1000

#define PI_3 1.0471975512
#define PI2_3 2.09439510239

#define SENS_A A0
#define SENS_B A1
#define SENS_C A2
#define LED D6
#define LIGHT_SEN_ENABLE D5

struct state_t {
    int8_t  phase  = 0;
    int8_t  fine   = 0;
    int32_t liters = 0;

    int a_min = 2500;
    int b_min = 2500;
    int c_min = 2500;

    int a_max = 0;
    int b_max = 0;
    int c_max = 0;
};

class MuinoWaterSensor : public PollingComponent, public Sensor {
public:
    Sensor* water_liter_sensor = new Sensor();



    MuinoWaterSensor() : PollingComponent(UPDATE_RATE_LITERS) {

        this->state.phase  = 0;
        this->state.fine   = 0;
        this->state.liters = 0;

        this->state.a_min = 0;
        this->state.b_min = 0;
        this->state.c_min = 0;

        this->state.a_max = 0;
        this->state.b_max = 0;
        this->state.c_max = 0;
    }

    float get_setup_priority() const override {
        return esphome::setup_priority::HARDWARE;
    }

    void setup();

    int  convert_adc_to_liters(int sen_a, int sen_b, int sen_c);

    void loop();

    void update() override {
        // * Send total after calculation
        this->water_liter_sensor->publish_state(this->liter);
    }

private:


    float liter = 0.0;
    state_t  state;


    // * sensor values
    int32_t sen_a = 0;
    int32_t sen_b = 0;
    int32_t sen_c = 0;



    float mini_average(float x, float y, float alpha_cor);
    float max_average(float x, float y, float alpha_cor);

    void magnitude_offset_iter(int a, int b, int c);

    void phase_coarse_iter(int a, int b, int c);
    int  phase_fine_iter(int a, int b, int c);
};
