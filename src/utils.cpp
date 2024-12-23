#include <cmath>

#include <SimpleFOC.h>
#include "utils.h"

float float_rand(float min, float max)
{
    float scale = rand() / (float)RAND_MAX; /* [0, 1.0] */
    return min + scale * (max - min);       /* [min, max] */
};

float norm(float x, float y)
{
    return sqrtf(x * x + y * y);
}

float lerp(float p, float a, float b)
{
    return a + min(1.0f, max(0.0f, p)) * (b - a);
}

float inverse_lerp(float v, float a, float b)
{
    float p = (a - v) / (a - b);
    return _constrain(p, 0.f, 1.f);
}

float mean(float *arr, int num)
{
    float sum = 0;
    for (int i = 0; i < num; i++)
    {
        sum += arr[i];
    }
    return sum / num;
}

float stddev(float *arr, int num)
{
    float avg = mean(arr, num);
    float sum = 0;
    for (int i = 0; i < num; i++)
    {
        sum += powf(arr[i] - avg, 2);
    }
    return sqrtf(sum / num);
}

void estimate_resistance_and_inductance(float voltage_peak, BLDCDriver6PWM *driver, CurrentSense *currentSense)
{
    const int num_current_measurements = 200;
    const int num_inductance_measurements = 50;
    float center = driver->voltage_limit / 2;

    // #######
    // measure the current mean and stddev, without voltage applied.
    driver->setPwm(center, center, center);
    delay(10); // wait to stabilize

    float currents_a[num_current_measurements];
    float currents_b[num_current_measurements];
    float currents_c[num_current_measurements];
    for (int i = 0; i < num_current_measurements; i++)
    {
        delayMicroseconds(100);
        PhaseCurrent_s phasecurrents = currentSense->getPhaseCurrents();
        currents_a[i] = phasecurrents.a;
        currents_b[i] = phasecurrents.b;
        currents_c[i] = phasecurrents.c;
    }

    float a_mean = mean(currents_a, num_current_measurements);
    float a_std = stddev(currents_a, num_current_measurements);
    float b_mean = mean(currents_b, num_current_measurements);
    float b_std = stddev(currents_b, num_current_measurements);
    float c_mean = mean(currents_c, num_current_measurements);
    float c_std = stddev(currents_c, num_current_measurements);

    // #######
    // measure the current mean and stddev, with a voltage between a and b.
    driver->setPwm(center + voltage_peak / 2, center - voltage_peak / 2, center);
    delay(10); // wait to stabilize

    for (int i = 0; i < num_current_measurements; i++)
    {
        delayMicroseconds(100);
        PhaseCurrent_s phasecurrents = currentSense->getPhaseCurrents();
        currents_a[i] = phasecurrents.a - a_mean;
        currents_b[i] = phasecurrents.b - b_mean;
        currents_c[i] = phasecurrents.c - c_mean;
    }

    driver->setPwm(center, center, center);
    delay(100); // wait to stabilize

    float a_mean2 = mean(currents_a, num_current_measurements);
    float a_std2 = stddev(currents_a, num_current_measurements);
    float b_mean2 = mean(currents_b, num_current_measurements);
    float b_std2 = stddev(currents_b, num_current_measurements);
    float c_mean2 = mean(currents_c, num_current_measurements);
    float c_std2 = stddev(currents_c, num_current_measurements);

    float load_resistance = voltage_peak / ((a_mean2 - b_mean2) / 2);

    // #######
    // measure the inductance (method 1)
    float drops[num_inductance_measurements];

    for (int i = 0; i < num_inductance_measurements; i++)
    {
        driver->setPwm(center + voltage_peak / 2, center - voltage_peak / 2, center);
        delay(1);
        driver->setPwm(center, center, center);
        delayMicroseconds(20);

        float currents[10];
        int j = 0;
        while (j < 10)
        {
            PhaseCurrent_s phasecurrents = currentSense->getPhaseCurrents();
            float current = (phasecurrents.a - a_mean) - (phasecurrents.b - b_mean);
            if (j == 0 || current != currents[j - 1])
            {
                currents[j] = current;
                j++;
            }
            else
            {
                delayMicroseconds(5);
            }
        }
        // Serial.printf("%f %f %f %f %f\r\n", currents[0], currents[1], currents[2], currents[3], currents[4]);
        drops[i] = currents[1] / currents[0];
    }

    int interval = static_cast<int>(1000000ull / driver->pwm_frequency); // us
    float avg_drop = mean(drops, num_inductance_measurements);
    float inductance = load_resistance * interval / logf(1 / avg_drop);

    // #######
    // print stats
    Serial.printf("mean and std (idle):\r\n");
    Serial.printf("a: % f % f\r\n", a_mean, a_std);
    Serial.printf("b: % f % f\r\n", b_mean, b_std);
    Serial.printf("c: % f % f\r\n", c_mean, c_std);
    Serial.printf("mean and std (voltage applied between a, b):\r\n");
    Serial.printf("a: % f % f\r\n", a_mean2, a_std2);
    Serial.printf("b: % f % f\r\n", b_mean2, b_std2);
    Serial.printf("c: % f % f\r\n", c_mean2, c_std2);
    Serial.printf("estimated resistance:    % f\r\n", load_resistance);
    Serial.printf("estimated inductance:    % f uH\r\n", inductance);
    Serial.printf("estimated time constant: % f us\r\n", inductance / load_resistance);
}

float read_vbus(CurrentSense *currentSense)
{
    const float multiplier = (18 + 169) / 18.f;
    return _readADCVoltageLowSide(A_VBUS, currentSense->params) * multiplier;
}

static float Ntc2TempV(float ADCVoltage)
{
    // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
    const float ResistorBalance = 4700.0;
    const float Beta = 3425.0f;
    const float RoomTempI = 1.0F / 298.15f; //[K]
    const float Rt = ResistorBalance * ((3.3F / ADCVoltage) - 1);
    const float R25 = 10000.0F;

    float T = 1.0f / ((logf(Rt / R25) / Beta) + RoomTempI);
    T = T - 273.15f;

    return T;
}

float read_temperature(CurrentSense *currentSense)
{
    float adc_volts = _readADCVoltageLowSide(A_TEMPERATURE, currentSense->params);
    return Ntc2TempV(adc_volts);
}

float read_potentiometer(CurrentSense *currentSense)
{
    float adc_volts = _readADCVoltageLowSide(A_POTENTIOMETER, currentSense->params);
    return adc_volts;
}

void init_led_internal()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void write_led_internal(bool state)
{
    digitalWrite(LED_BUILTIN, state);
}
