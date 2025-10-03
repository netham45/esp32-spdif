#ifndef SPDIF_H
#define SPDIF_H

#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/ringbuf.h"
#include "driver/rmt_rx.h"
#include <string.h>

// Configuration constants sourced from Kconfig
#define RMT_RESOLUTION_HZ CONFIG_SPDIF_IN_RMT_RESOLUTION_HZ
#define RMT_MEM_BLOCK_SYMBOLS CONFIG_SPDIF_IN_RMT_MEM_BLOCK_SYMBOLS
#define SYMBOL_BUFFER_SIZE CONFIG_SPDIF_IN_SYMBOL_BUFFER_SIZE
#define PCM_BUFFER_SIZE CONFIG_SPDIF_IN_PCM_BUFFER_SIZE
#define DECODER_TASK_STACK CONFIG_SPDIF_IN_DECODER_TASK_STACK
#define DECODER_TASK_PRIORITY CONFIG_SPDIF_IN_DECODER_TASK_PRIORITY
#define MIN_SAMPLES_FOR_ANALYSIS CONFIG_SPDIF_IN_MIN_SAMPLES_FOR_ANALYSIS

#ifdef __cplusplus
extern "C" {
#endif

extern RingbufHandle_t spdif_in_pcm_buffer;

typedef struct
{
    bool groups_identified;   // Three pulse groups found
    bool ratios_valid;        // Ratios match 1:2:3 within tolerance
    bool distribution_valid;  // Distribution matches expected percentages
    float ratio_error;        // Error from ideal 1:2:3 ratio
    float short_pulse_pct;    // Actual short pulse percentage
    float medium_pulse_pct;   // Actual medium pulse percentage
    float long_pulse_pct;     // Actual long pulse percentage
    float distribution_error; // Total distribution error
} timing_validation_t;

// Peak detection structure for histogram analysis
typedef struct
{
    uint32_t bin;
    uint32_t count;
    float center;
    uint32_t width;
} peak_t;


esp_err_t spdif_receiver_init(int input_pin, void (*init_done_cb)(void));
esp_err_t spdif_receiver_start(void);
esp_err_t spdif_receiver_stop(void);
void spdif_receiver_deinit(void);
uint32_t spdif_receiver_get_sample_rate(void);

inline static RingbufHandle_t spdif_in_get_ringbuf(){
    return spdif_in_pcm_buffer;
}

inline static int spdif_receiver_read(uint8_t *buffer, size_t size)
{
    if (!spdif_in_pcm_buffer)
    {
        return 0;
    }
    size_t received_size = 0;
    uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(
        spdif_in_pcm_buffer, &received_size, pdMS_TO_TICKS(10), size);
    if (data && received_size > 0)
    {
        memcpy(buffer, data, received_size);
        vRingbufferReturnItem(spdif_in_pcm_buffer, (void *)data);
        return received_size;
    }
    return 0;
}


#ifdef __cplusplus
}
#endif

#endif // SPDIF_H