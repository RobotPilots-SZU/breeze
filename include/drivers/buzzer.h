/* drivers/buzzer/buzzer.h */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_BUZZER_BUZZER_H_
#define DRIVERS_BUZZER_BUZZER_H_

#include <zephyr/types.h>
#include <errno.h>

#include <zephyr/device.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    struct note_duration
    {
        int note;           // 频率，单位 Hz，0 表示休止符
        int duration;       
        int div;            
    };

    struct song_config
    {
        const struct note_duration *notes;
        uint32_t length;
    };


    /**
     * @typedef buzzer_api_play_note
     * @brief Callback API for play a note on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_note)(const struct device *dev,
                                        const struct note_duration *note_cfg);
    /**
     * @typedef buzzer_api_play_song
     * @brief Callback API for play a song on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_song)(const struct device *dev,
                                        const struct song_config *song_cfg);


    typedef int (*buzzer_api_stop)(const struct device *dev);

    typedef int (*buzzer_api_start)(const struct device *dev);

    struct buzzer_driver_api
    {
        buzzer_api_play_note buzzer_play_note;
        buzzer_api_play_song buzzer_play_song;
        buzzer_api_stop buzzer_stop;
        buzzer_api_start buzzer_start;
    };

    static inline int buzzer_play_note(const struct device *dev, const struct note_duration *note_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->buzzer_play_note == NULL) {
            return -ENOSYS;
        }
        return api->buzzer_play_note(dev, note_cfg);
    }

    static inline int buzzer_play_song(const struct device *dev, const struct song_config *song_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->buzzer_play_song == NULL) {
            return -ENOSYS;
        }
        return api->buzzer_play_song(dev, song_cfg);
    }

    static inline int buzzer_stop(const struct device *dev)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->buzzer_stop == NULL) {
            return -ENOSYS;
        }
        return api->buzzer_stop(dev);
    }

    static inline int buzzer_start(const struct device *dev)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->buzzer_start == NULL) {
            return -ENOSYS;
        }
        return api->buzzer_start(dev);
    }

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BUZZER_BUZZER_H_ */
