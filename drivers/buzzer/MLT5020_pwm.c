/* drivers/buzzer/MLT5020_pwm.c */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_pwm_buzzer

#include <drivers/buzzer.h>

#define LOG_LEVEL CONFIG_BUZZER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(buzzer_pwm);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#define PWM_MAX_VOLUME 100
#define BUZZER_QUEUE_CAP 512

struct buzzer_pwm_cfg{
    const struct pwm_dt_spec pwm;
    uint32_t freq;
    uint8_t volume;
};

struct buzzer_pwm_data {
    struct k_work_delayable stop_work;  
    const struct device *dev;
    atomic_t playing_status;         // 0:空闲 1:播放中
    atomic_t enqueue_enabled;        // 1:允许入栈 0:禁止入栈
    struct note_duration queue[BUZZER_QUEUE_CAP];
    volatile uint16_t q_head;
    volatile uint16_t q_tail;
    struct k_spinlock q_lock;
};

/*---------------------------------------队列工具 start------------------------------------------------*/
/*判断是否为空*/
static inline bool buzzer_q_empty(struct buzzer_pwm_data *date)
{
    return date->q_head == date->q_tail;
}

/*判断是否为满*/
static inline bool buzzer_q_full(struct buzzer_pwm_data *date)
{
    return ((uint16_t)(date->q_tail + 1U) % BUZZER_QUEUE_CAP) == date->q_head;
}

/*入队*/
static inline bool buzzer_q_push(struct buzzer_pwm_data *date, const struct note_duration *n)
{
    if (buzzer_q_full(date))
        return false;
    date->queue[date->q_tail] = *n;
    date->q_tail = (uint16_t)((date->q_tail + 1U) % BUZZER_QUEUE_CAP);
    return true;
}

/*出队*/
static inline bool buzzer_q_pop(struct buzzer_pwm_data *date, struct note_duration *out)
{
    if (buzzer_q_empty(date))
        return false;
    *out = date->queue[date->q_head];
    date->q_head = (uint16_t)((date->q_head + 1U) % BUZZER_QUEUE_CAP);
    return true;
}
/*---------------------------------------队列工具 end------------------------------------------------*/

static int buzzer_apply_note(const struct device *dev, const struct note_duration *note_cfg)
{
    const struct buzzer_pwm_cfg *cfg = dev->config;
    struct buzzer_pwm_data *data = dev->data;

    if (note_cfg->note == 0)
    {
        (void)pwm_set_pulse_dt(&cfg->pwm, 0);
        (void)k_work_reschedule(&data->stop_work, K_MSEC(note_cfg->duration));
        atomic_set(&data->playing_status, 1);
        return 0;
    }

    uint32_t period_ns = NSEC_PER_SEC / (uint32_t)note_cfg->note;
    uint32_t pulse_ns = (period_ns * cfg->volume) / PWM_MAX_VOLUME;

    (void)pwm_set_pulse_dt(&cfg->pwm, 0); // 先静音再切频
    int ret = pwm_set_dt(&cfg->pwm, period_ns, pulse_ns);
    if (ret < 0)
        return ret;

    (void)k_work_reschedule(&data->stop_work, K_MSEC(note_cfg->duration));
    atomic_set(&data->playing_status, 1);
    return 0;
}

// 到期静音回调
static void buzzer_stop_work_handler(struct k_work *work)
{
    struct buzzer_pwm_data *data = CONTAINER_OF(work, struct buzzer_pwm_data, stop_work.work);
    const struct buzzer_pwm_cfg *cfg = data->dev->config;

    // 到期静音当前音
    (void)pwm_set_pulse_dt(&cfg->pwm, 0);
    atomic_clear(&data->playing_status);

    // 取下一音并播放
    struct note_duration next;
    if (buzzer_q_pop(data, &next))
    {
        (void)buzzer_apply_note(data->dev, &next);
    }
}

/* 
 * 限幅函数：将请求频率限定在 100Hz~9kHz 范围内，并计算对应周期（纳秒）
*/
static inline uint32_t buzzer_clamp_freq_for_timer(uint32_t req_freq_hz, uint32_t *out_period_ns)
{
    if (req_freq_hz == 0)
    {
        if (out_period_ns)
            *out_period_ns = 0;
        LOG_WRN("Requested frequency is 0");
        return 0;
    }

    if (req_freq_hz > 9000)
    {
        LOG_WRN("Frequency %u Hz exceeds 9000 Hz", req_freq_hz);
    }
    else if (req_freq_hz < 100)
    {
        LOG_WRN("Frequency %u Hz is below 100 Hz", req_freq_hz);
    }

    if (out_period_ns)
    {
        *out_period_ns = (uint32_t)(NSEC_PER_SEC / req_freq_hz);
    }
    return req_freq_hz;
}

static int buzzer_pwm_play_note(const struct device *dev, const struct note_duration *note_cfg)
{
    const struct buzzer_pwm_cfg *cfg = dev->config;
    struct buzzer_pwm_data *data = dev->data;

    if (!pwm_is_ready_dt(&cfg->pwm))
        return -ENODEV;
    if (!note_cfg || note_cfg->duration <= 0)
        return -EINVAL;

    // 新增：禁止入栈时直接返回
    if (!atomic_get(&data->enqueue_enabled)) {
        return -EACCES; 
    }

    if (!buzzer_q_push(data, note_cfg))
    {
        LOG_WRN("Buzzer queue full, cannot enqueue note %d for %d ms",
                note_cfg->note, note_cfg->duration);
        return -ENOMEM;
    }

    if (!atomic_get(&data->playing_status))
    {
        struct note_duration first;
        if (buzzer_q_pop(data, &first))
        {
            return buzzer_apply_note(dev, &first);
        }
    }
    return 0;
}

// /* tempo 换算：从 BPM + 每拍单位 + 本音符分母 得到毫秒
//  * - bpm>0 时有效；beat_unit/div 负数表示附点（×1.5）
//  * - bpm==0 时不使用此函数（直接用毫秒）
//  */
// static inline uint32_t buzzer_ms_from_tempo(uint16_t bpm, int beat_unit, int div)
// {
//     if (bpm == 0 || div == 0) {
//         return 0U;
//     }

//     uint32_t beat_den = (uint32_t)((beat_unit >= 0) ? beat_unit : -beat_unit);
//     uint32_t note_den = (uint32_t)((div       >= 0) ? div       : -div);
//     if (beat_den == 0U) {
//         beat_den = 4U; /* 默认四分为一拍 */
//     }

//     /* 附点系数 3/2 */
//     uint32_t beat_num = (beat_unit < 0) ? 3U : 1U;
//     uint32_t beat_den_fac = (beat_unit < 0) ? 2U : 1U;
//     uint32_t note_num = (div < 0) ? 3U : 1U;
//     uint32_t note_den_fac = (div < 0) ? 2U : 1U;

//     /* ms = (60000/bpm) * (beat_den/note_den) * (note_factor/beat_factor) */
//     uint64_t numerator   = 60000ULL * beat_den * note_num * beat_den_fac;
//     uint64_t denominator = (uint64_t)bpm * note_den * note_den_fac * beat_num;

//     return (uint32_t)(numerator / denominator);
// }

static int buzzer_pwm_play_song(const struct device *dev, const struct song_config *song_cfg)
{
    if (!dev || !song_cfg || !song_cfg->notes)
        return -EINVAL;
    if (song_cfg->length == 0)
        return 0;

    for (size_t i = 0; i < song_cfg->length; ++i)
    {
        const struct note_duration *n = &song_cfg->notes[i];

        if (n->duration < 0)
        {
            LOG_WRN("Note %zu has invalid duration %d, skipping", i, n->duration);
            continue;
        }

        int ret = buzzer_pwm_play_note(dev, n);
        if (ret == -ENOMEM)
        {
            LOG_WRN("buzzer queue full at note %zu/%zu", i, song_cfg->length);
            return -ENOMEM;
        }
        if (ret < 0)
            return ret;
    }
    return 0;
}

// stop：禁止入栈，不清已入队，不打断当前播放
static int buzzer_pwm_stop(const struct device *dev)
{
    struct buzzer_pwm_data *data = dev->data;
    atomic_clear(&data->enqueue_enabled);
    return 0;
}

// start：允许入栈；若当前空闲且队列里有待播，则启动
static int buzzer_pwm_start(const struct device *dev)
{
    struct buzzer_pwm_data *data = dev->data;
    atomic_set(&data->enqueue_enabled, 1);

    if (!atomic_get(&data->playing_status)) {
        struct note_duration first;
        if (buzzer_q_pop(data, &first)) {
            return buzzer_apply_note(dev, &first);
        }
    }
    return 0;
}

static int buzzer_pwm_init(const struct device *dev)
{
    const struct buzzer_pwm_cfg *cfg = dev->config;
    const struct pwm_dt_spec *spec = &cfg->pwm;
    struct buzzer_pwm_data *data = dev->data;

    __ASSERT(device_is_ready(spec->dev), "PWM controller not ready");

    uint64_t cycles = 0;
    int err_code = pwm_get_cycles_per_sec(spec->dev, spec->channel, &cycles);
    __ASSERT((err_code >= 0) && (cycles != 0), "pwm_get_cycles_per_sec failed or cycles==0");

    uint16_t default_feq = cfg->freq;
    __ASSERT(default_feq > 100 && default_feq <= 9000,
             "Default frequency must be between 100 and 9000 Hz, Please modify the dts file!!!");

    uint32_t period_ns = (uint32_t)(NSEC_PER_SEC / default_feq);
    default_feq = buzzer_clamp_freq_for_timer(default_feq, &period_ns);

    // 上电静音：有效周期，脉冲=0
    err_code = pwm_set_dt(spec, period_ns, 0);
    if (err_code) return err_code;

    /* 初始化延迟工作 */
    data->dev = dev;
    k_work_init_delayable(&data->stop_work, buzzer_stop_work_handler);

    // 默认允许入栈
    atomic_set(&data->enqueue_enabled, 1);

    // 在 init 末尾加日志，确认初始化回调与工作项
    LOG_INF("init: stop_work initialized, dev = %p", dev);

    LOG_INF("buzzer init: default_freq = %u Hz, period_ns = %u, ch = %u",
            default_feq, period_ns, spec->channel);
    return 0;
}

static const struct buzzer_driver_api buzzer_pwm_api = {
    .buzzer_play_note  = buzzer_pwm_play_note,
    .buzzer_play_song  = buzzer_pwm_play_song,
    .buzzer_stop       = buzzer_pwm_stop,   // 现在表示“禁止入栈”
    .buzzer_start      = buzzer_pwm_start,  // 新增：恢复入栈
};

// 为设备添加 data（工作对象）
#define BUZZER_PWM_INIT(inst)                                          \
    BUILD_ASSERT(DT_INST_PROP(inst, default_volume) <= PWM_MAX_VOLUME, \
                 "Default volume exceeds maximum");                    \
    BUILD_ASSERT(DT_INST_PROP(inst, default_frequency) > 0,            \
                 "Default frequency must be > 0");                     \
    static struct buzzer_pwm_data buzzer_pwm_##inst##_data;            \
    static const struct buzzer_pwm_cfg buzzer_pwm_##inst##_cfg = {     \
        .pwm = PWM_DT_SPEC_INST_GET(inst),                             \
        .freq = DT_INST_PROP(inst, default_frequency),                 \
        .volume = DT_INST_PROP(inst, default_volume),                  \
    };                                                                 \
    DEVICE_DT_INST_DEFINE(inst,                                                   \
                          buzzer_pwm_init,                                        \
                          NULL,                                                   \
                          &buzzer_pwm_##inst##_data,                              \
                          &buzzer_pwm_##inst##_cfg,                               \
                          POST_KERNEL,                                            \
                          CONFIG_BUZZER_INIT_PRIORITY,                            \
                          &buzzer_pwm_api)

DT_INST_FOREACH_STATUS_OKAY(BUZZER_PWM_INIT);
