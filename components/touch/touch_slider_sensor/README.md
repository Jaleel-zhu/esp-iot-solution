# Touch Slider Sensor

**Note:** This component is for developers testing only. It is not intended for production use.

A component providing enhanced touch slider detection functionality for ESP32 series chips.

## Features

- FSM-based touch detection with configurable thresholds
- Support for slider gesture detection with configurable position calculation window
- Automatic default value selection for optimal performance
- Callback-based event notification

## Dependencies

- [touch_sensor_fsm](https://components.espressif.com/components/espressif/touch_sensor_fsm)
- [touch_sensor_lowlevel](https://components.espressif.com/components/espressif/touch_sensor_lowlevel)

## Configuration Options

### Calculate Window (`calculate_window`)

The `calculate_window` parameter determines how many adjacent touch channels are used for position calculation. This affects the precision and noise immunity of the slider.

**Default Values (when set to 0):**
- **2 channels**: `calculate_window = 2` (uses all available channels)
- **3+ channels**: `calculate_window = 3` (optimal balance between precision and noise immunity)

**Manual Configuration:**
- You can explicitly set any value from 1 to `channel_num`
- Smaller values (1-2): Higher sensitivity but more susceptible to noise
- Larger values (3+): Better noise immunity but potentially lower resolution
- For high-precision applications: Recommended value is `min(3, channel_num)`

**Backward Compatibility:**
- Setting `calculate_window = 0` enables automatic default selection
- Existing code that doesn't initialize this field will use optimal defaults
- Explicitly set values continue to work as before

## Example

```c
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "touch_slider_sensor.h"

static uint32_t channel_list[] = {2, 4, 6, 12, 10, 8};
static float threshold[] = {0.005f, 0.005f, 0.005f, 0.005f, 0.005f, 0.005f};
static int channel_num = 6;
static int slider_range = 10000;

static void touch_slider_event_callback(touch_slider_handle_t handle, touch_slider_event_t event, int32_t data, void *cb_arg)
{
    if (event == TOUCH_SLIDER_EVENT_RIGHT_SWIPE) {
        printf("Right swipe (speed)\n");
    } else if (event == TOUCH_SLIDER_EVENT_LEFT_SWIPE) {
        printf("Left swipe (speed)\n");
    } else if (event == TOUCH_SLIDER_EVENT_RELEASE) {
        printf("Slide %" PRId32 "\n", data);
        if (data > slider_range / 10) {
            printf("Right swipe (displacement)\n");
        } else if (data < -slider_range / 10) {
            printf("Left swipe (displacement)\n");
        }
    } else if (event == TOUCH_SLIDER_EVENT_POSITION) {
        printf("pos,%" PRId32 "\n", data);
    }
}
/* Task function to handle touch slider events */
static void touch_slider_task(void *pvParameters)
{
    touch_slider_handle_t handle = (touch_slider_handle_t)pvParameters;
    while (1) {
        touch_slider_sensor_handle_events(handle);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    touch_slider_config_t config = {
        .channel_num = channel_num,
        .channel_list = channel_list,
        .channel_threshold = threshold,
        .filter_reset_times = 5,
        .position_range = 10000,
        .swipe_alpha = 0.9,
        .swipe_threshold = 50,
        .swipe_hysterisis = 40,
        .channel_gold_value = NULL,
        .debounce_times = 0,
        .calculate_window = 0,  // Use default value (auto-selected based on channel count)
        .skip_lowlevel_init = false
    };

    touch_slider_handle_t handle;
    ESP_ERROR_CHECK(touch_slider_sensor_create(&config, &handle, touch_slider_event_callback, NULL));

    // Create a task to handle touch slider events
    xTaskCreate(touch_slider_task, "touch_slider_sensor", 3072, handle, 2, NULL);

    // Main loop can do other things now
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task can do other work or just idle
    }
}
```