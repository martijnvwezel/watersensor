#pragma once

// docker run   -v "${PWD}":/config  -it ghcr.io/esphome/esphome run muinowatersensor.yaml

// #include <Arduino.h>
#include "esphome.h"
using namespace esphome;
#define SMOOTHING_FACTOR 3 // 2 - 10
#define AC_STEPS 16        // given pi/3 coarse estimate of phase, calculate autocorrelation of signals within that pi/3 range
#define ALPHA_COR 0.1      // Value between 0-1

//* Some calculations help
#define PI_3 1.0471975512
#define PI2_3 2.09439510239
#define LOW 0
#define HIGH 1

// #define SENS_A A0
// #define SENS_B A1
// #define SENS_C A2
// #define LED D6
// #define LIGHT_SEN_ENABLE D5

static const uint8_t D5 = 6;
static const uint8_t D6 = 43;

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
    Sensor* watersensor_sensor = new Sensor();

    MuinoWaterSensor() : PollingComponent(15000) {
    }
    float get_setup_priority() const override {
        return esphome::setup_priority::HARDWARE;
    }

    void setup() override {
        // pinMode(SENS_A, INPUT); // ADC 0
        // pinMode(SENS_B, INPUT); // ADC 1
        // pinMode(SENS_C, INPUT); // ADC 2
        // pinMode(LED, OUTPUT);

        // pinMode(LIGHT_SEN_ENABLE, OUTPUT);
        // digitalWrite(LIGHT_SEN_ENABLE, HIGH);

        this->state.phase  = 0;
        this->state.fine   = 0;
        this->state.liters = 0;

        this->state.a_min = 0;
        this->state.b_min = 0;
        this->state.c_min = 0;

        this->state.a_max = 0;
        this->state.b_max = 0;
        this->state.c_max = 0;

        // subscribe("MuinoWaterSensor", &MuinoWaterSensor::on_message);
    }
    int  magic_code_box(int sen_a, int sen_b, int sen_c);
    void loop() override {
        digitalWrite(LED, HIGH);
        delay(5);
        int32_t sen_a = analogReadMilliVolts(SENS_A);
        int32_t sen_b = analogReadMilliVolts(SENS_B);
        int32_t sen_c = analogReadMilliVolts(SENS_C);

        digitalWrite(LED, LOW);
        delay(5);

        int32_t sen_a_zero = analogReadMilliVolts(SENS_A);
        int32_t sen_b_zero = analogReadMilliVolts(SENS_B);
        int32_t sen_c_zero = analogReadMilliVolts(SENS_C);

        sen_a = sen_a - sen_a_zero;
        sen_b = sen_b - sen_b_zero;
        sen_c = sen_c - sen_c_zero;

        bool send = magic_code_box(sen_a, sen_b, sen_c);

        // if (send && sender++ > 3000) {
        //     send_data_to_broker();
        ESP_LOGD("loop", "%d", (int)this->mili_liters_total);
        //     sender = 0;
        // }

        // // * Log values
        // ESP_LOGD("loop", "%d %d %d", sen_a, sen_b, sen_c);

        delay(5);
    }

    void update() override {
        // * Send total after calculation
        this->watersensor_sensor->publish_state(this->mili_liters_total);
    }

private:
    uint32_t       mili_liters_total = 0; // * since boot
    int            sender            = 0; // * Send only 1 update each 30 seconds
    bool           not_inited        = true;
    static state_t state;

    float mini_average(float x, float y, float alpha_cor);
    float max_average(float x, float y, float alpha_cor);

    void magnitude_offset_iter(int a, int b, int c);

    void phase_coarse_iter(int a, int b, int c);
    int  phase_fine_iter(int a, int b, int c);
};
