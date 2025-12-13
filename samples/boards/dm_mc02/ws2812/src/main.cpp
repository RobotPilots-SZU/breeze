#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>

LOG_MODULE_REGISTER(ws2812, LOG_LEVEL_INF);


/* default configuration */
#define SPI_FRAME_BITS 8
#define BITS_PER_COLOR_CHANNEL 8

// get ws2812 node
#define LED_WS2812_NODE DT_NODELABEL(rgb_led)

/* WS2812 device */
static const struct device *ws2812_dev;
static struct led_rgb *pixels;

struct k_thread led_thread_data;

/* Color definitions */
typedef enum
{
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_CYAN,
    LED_COLOR_MAGENTA,
    LED_COLOR_WHITE,
    LED_COLOR_PURPLE,
    LED_COLOR_ORANGE,
    LED_COLOR_PINK
} led_color_t;

static inline struct led_rgb led_color_to_rgb(led_color_t color)
{
    switch (color)
    {
    case LED_COLOR_RED:
        return {255, 0, 0};
    case LED_COLOR_GREEN:
        return {0, 255, 0};
    case LED_COLOR_BLUE:
        return {0, 0, 255};
    case LED_COLOR_YELLOW:
        return {255, 255, 0};
    case LED_COLOR_CYAN:
        return {0, 255, 255};
    case LED_COLOR_MAGENTA:
        return {255, 0, 255};
    case LED_COLOR_WHITE:
        return {255, 255, 255};
    case LED_COLOR_PURPLE:
        return {128, 0, 128};
    case LED_COLOR_ORANGE:
        return {255, 128, 0};
    case LED_COLOR_PINK:
        return {255, 192, 203};
    default:
        return {0, 0, 0};
    }
}

#define K_LED_TASK_TEST_STACK_SIZE 1024
#define K_LED_TASK_TEST_PRIORITY 5

K_THREAD_STACK_DEFINE(led_stack_area, K_LED_TASK_TEST_STACK_SIZE);

void led_task_test(void *arg1, void *arg2, void *arg3);

void Create_led_task_test_thread(void)
{
    k_tid_t led_tid = k_thread_create(
        &led_thread_data,                        // 线程控制块
        led_stack_area,                          // 栈空间
        K_THREAD_STACK_SIZEOF(led_stack_area),   // 栈大小
        led_task_test,                           // 入口函数
        NULL, NULL, NULL,                        // 参数
        K_LED_TASK_TEST_PRIORITY,                // 优先级（数字越小优先级越高）
        0,                                       // 线程选项
        K_NO_WAIT                                // 立即启动
    );
    k_thread_name_set(led_tid, "led_task_test");
    if (led_tid == NULL){
        LOG_ERR("Failed to create led_task_test thread");
    }
}

/**
 * @brief Initialize the WS2812 LED strip
 * @param None
 * @return None
 * 
 */
void Init_ws2812(void)
{
    ws2812_dev = DEVICE_DT_GET(LED_WS2812_NODE);
    __ASSERT(!device_is_ready(ws2812_dev), "Failed to get WS2812 device");

    auto num_leds = led_strip_length(ws2812_dev);
    __ASSERT(num_leds != 0, "WS2812 device has zero length");

    /* Allocate memory for LED pixels */
    pixels = new led_rgb[num_leds];
    __ASSERT(pixels != nullptr, "Failed to allocate memory for LED pixels");
}

/**
 * @brief Set the SpecificLed color object
 * 
 * @param ws2812_dev 
 * @param index 
 * @param red 
 * @param green 
 * @param blue 
 * @return int 
 */
int set_SpecificLed_color(const struct device *ws2812_dev, uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    auto num_leds = led_strip_length(ws2812_dev);
    int ret;

    if (index < num_leds) {
        pixels[index].r = red;
        pixels[index].g = green;
        pixels[index].b = blue;
    }
    else {
        LOG_ERR("LED index: %d out of range: %d", index, num_leds);
        return -EINVAL;
    }

    ret = led_strip_update_rgb(ws2812_dev, pixels, num_leds);
    if (ret < 0) {
        LOG_ERR("Failed to update LED color: %d", ret);
        return ret;
    }
    return 0;
}

/**
 * @brief Set the Led color object
 * 
 * @param ws2812_dev 
 * @param index 
 * @param color_t 
 * @return int 
 */
int set_Led_color(const struct device *ws2812_dev, uint8_t index, led_color_t color_t)
{
    auto num_leds = led_strip_length(ws2812_dev);
    auto color = led_color_to_rgb(color_t);
    return set_SpecificLed_color(ws2812_dev, index, color.r, color.g, color.b);
}

/**
 * @brief the entry function of led_task_test thread
 * 
 * @param arg1 
 * @param arg2 
 * @param arg3 
 */
void led_task_test(void *arg1, void *arg2, void *arg3)
{
    while (1)
    {
        set_Led_color(ws2812_dev, 0, LED_COLOR_YELLOW);
        k_sleep(K_MSEC(500));
        set_Led_color(ws2812_dev, 0, LED_COLOR_PURPLE);
        k_sleep(K_MSEC(500));
        set_Led_color(ws2812_dev, 0, LED_COLOR_ORANGE);
        k_sleep(K_MSEC(500));
    }
}

int main(void)
{
    Init_ws2812();
    Create_led_task_test_thread();

    while (1) {
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
