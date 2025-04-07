// adc.h

// GPIO map: https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/]
// but we use the provided function adc_continuous_io_to_channel()
// to find the ADC unit and channel

#include "esp_adc/adc_continuous.h"

#define ADC_FRAME_LEN 1024 // 256

adc_continuous_handle_t continuous_adc_init(int SPS /* samples per sec */,
                                            int gpio_a, int gpio_b)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 16 * ADC_FRAME_LEN,
        .conv_frame_size = ADC_FRAME_LEN,
    };
    int rc = adc_continuous_new_handle(&adc_config, &handle);
    if (rc != ESP_OK) {
      Serial.printf("adc_continuous_new_handle() returned %d '%s'\r\n",
                    rc, esp_err_to_name(rc));
      return NULL;
    }

    // arm the ADC+DMA for two channels
    adc_unit_t unit;
    adc_channel_t ch0;
    adc_continuous_io_to_channel(gpio_a, &unit, &ch0);
    // adc_channel_t ch1;
    // adc_continuous_io_to_channel(gpio_b, &unit, &ch1);

    adc_digi_pattern_config_t adc_pattern[1];
    memset(&adc_pattern, 0, sizeof(adc_pattern));
    adc_pattern[0].atten = ADC_ATTEN_DB_0;
    adc_pattern[0].channel = ch0;
    adc_pattern[0].unit = unit;
    adc_pattern[0].bit_width = 12;

    // adc_pattern[1].atten = ADC_ATTEN_DB_0;
    // adc_pattern[1].channel = ch1;
    // adc_pattern[1].unit = unit;
    // adc_pattern[1].bit_width = 12;

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = SPS, // 2*SPS,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    rc = adc_continuous_config(handle, &dig_cfg);
    if (rc == ESP_OK)
        return handle;

    Serial.printf("adc_continuous_config() returned %d '%s'\r\n",
                  rc, esp_err_to_name(rc));
    return NULL;
}

// eof
