#ifndef SONG_LIB_H
#define SONG_LIB_H

#include "note_lib.h"
#include <drivers/buzzer.h>

/* -------------------------- 歌曲：《你》配置 --------------------------
 * 核心参数：Ab调、4/4拍、直接毫秒时长（不使用节拍制）
 * BPM约150时：四分音符=400ms, 八分音符=200ms, 二分音符=800ms, 附点四分=600ms
 */

// Ab调核心音高定义
#define NOTE_AB4 NOTE_GS4 // Ab4 = 415.30Hz
#define NOTE_BB4 NOTE_AS4 // Bb4 = 466.16Hz
#define NOTE_AB5 NOTE_GS5 // Ab5 = 830.61Hz
#define NOTE_BB5 NOTE_AS5 // Bb5 = 932.33Hz

#ifndef NOTE_REST
#define NOTE_REST 0U
#endif

#ifdef __cplusplus
extern "C" {
#endif

    static const struct note_duration song_note_YOU[] = {
        /* ============= 主歌1（歌词：我一直追寻着你...却总保持着距离） ============= */
        {NOTE_B4, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_D5, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 200, 0}, {NOTE_C5, 200, 0},
        {NOTE_B4, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_D5, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 200, 0}, {NOTE_C5, 200, 0},
        {NOTE_B4, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_D5, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 200, 0}, {NOTE_C5, 200, 0},

        /* ============= 主歌2（继续，保持相同节奏模式） ============= */
        {NOTE_B4, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_B4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_D5, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_C5, 200, 0}, {NOTE_G4, 200, 0}, {NOTE_D4, 400, 0},
        {NOTE_E4, 1200, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 800, 0},
        {NOTE_C5, 800, 0}, {NOTE_B4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 400, 0},
        {NOTE_E4, 1200, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_C5, 1600, 0},
        {NOTE_B4, 400, 0}, {NOTE_D5, 800, 0}, {NOTE_E4, 2000, 0},

        /* ============= 副歌1 ============= */
        {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_B4, 800, 0},
        {NOTE_C5, 800, 0}, {NOTE_B4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 400, 0},
        {NOTE_E4, 1200, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_B4, 800, 0},
        {NOTE_C5, 800, 0}, {NOTE_D5, 2000, 0},

        /* ============= 副歌2 ============= */
        {NOTE_REST, 800, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0},
        {NOTE_REST, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 800, 0}, {NOTE_C4, 1200, 0},
        {NOTE_REST, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0},
        {NOTE_REST, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0},
        {NOTE_REST, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_C5, 800, 0}, {NOTE_C4, 1200, 0},
        {NOTE_REST, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 200, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 400, 0}, {NOTE_B3, 400, 0}, {NOTE_A3, 200, 0}, {NOTE_G3, 1000, 0},

        /* ============= 桥段 ============= */
        {NOTE_REST, 200, 0}, {NOTE_G3, 200, 0}, {NOTE_REST, 0, 0}, {NOTE_G3, 200, 0}, {NOTE_REST, 0, 0}, {NOTE_G3, 200, 0}, {NOTE_G4, 800, 0}, {NOTE_E4, 600, 0}, {NOTE_D4, 200, 0}, {NOTE_C4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_C4, 800, 0},
        {NOTE_REST, 200, 0}, {NOTE_C4, 400, 0}, {NOTE_REST, 0, 0}, {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_A3, 1200, 0},
        {NOTE_REST, 400, 0}, {NOTE_A3, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_C4, 400, 0}, {NOTE_D4, 1200, 0}, {NOTE_REST, 400, 0},
        {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 1200, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_E4, 200, 0}, {NOTE_G4, 600, 0}, {NOTE_B4, 800, 0}, {NOTE_C5, 1200, 0},
        {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 800, 0}, {NOTE_A4, 800, 0}, {NOTE_G4, 200, 0}, {NOTE_A4, 600, 0}, {NOTE_G4, 200, 0}, {NOTE_REST, 20, 0}, {NOTE_G4, 600, 0}, {NOTE_REST, 20, 0}, {NOTE_G4, 800, 0}, {NOTE_D4, 1600, 0},
        {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 1200, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_E4, 200, 0}, {NOTE_G4, 600, 0}, {NOTE_B4, 800, 0}, {NOTE_C5, 1200, 0},
        {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_B4, 800, 0}, {NOTE_A4, 1200, 0}, {NOTE_REST, 0, 0}, {NOTE_A4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_A4, 600, 0}, {NOTE_C5, 800, 0}, {NOTE_D5, 1200, 0},
        {NOTE_REST, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_C5, 400, 0}, {NOTE_B4, 200, 0}, {NOTE_C5, 2400, 0},
        {NOTE_REST, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 1200, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_E4, 200, 0}, {NOTE_G4, 600, 0}, {NOTE_B4, 800, 0}, {NOTE_C5, 1200, 0},
        {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_C5, 800, 0}, {NOTE_A4, 1200, 0}, {NOTE_REST, 0, 0}, {NOTE_A4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_A4, 600, 0}, {NOTE_C5, 800, 0}, {NOTE_D5, 1200, 0},
        {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 1200, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_E4, 200, 0}, {NOTE_G4, 600, 0}, {NOTE_B4, 800, 0}, {NOTE_C5, 1200, 0},
        {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_G4, 800, 0}, {NOTE_A4, 1000, 0}, {NOTE_G4, 200, 0}, {NOTE_A4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_REST, 20, 0}, {NOTE_G4, 600, 0}, {NOTE_REST, 20, 0}, {NOTE_G4, 800, 0},
        {NOTE_D4, 1600, 0},
        {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_REST, 200, 0}, {NOTE_G4, 400, 0}, {NOTE_E4, 200, 0}, {NOTE_G4, 600, 0}, {NOTE_D5, 800, 0}, {NOTE_C5, 1200, 0},
        {NOTE_C4, 400, 0}, {NOTE_D4, 400, 0}, {NOTE_E4, 400, 0}, {NOTE_C5, 800, 0}, {NOTE_A4, 1000, 0},
        {NOTE_G4, 200, 0}, {NOTE_A4, 400, 0}, {NOTE_G4, 200, 0}, {NOTE_A4, 600, 0}, {NOTE_C5, 800, 0}, {NOTE_D5, 1200, 0},
        {NOTE_REST, 400, 0}, {NOTE_G4, 400, 0}, {NOTE_C5, 400, 0}, {NOTE_B4, 200, 0}, {NOTE_C5, 2400, 0},

        /* ============= 结尾 ============= */
        {NOTE_REST, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_F4, 800, 0},
        {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_G4, 800, 0},
        {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0},
        {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0},
        {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_C4, 800, 0}, {NOTE_D4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_G4, 800, 0}, {NOTE_F4, 800, 0}, {NOTE_E4, 800, 0}, {NOTE_D4, 800, 0}
    };

    // 简化配置：只需数组和长度
    static const struct song_config song_YOU = {
        song_note_YOU,
        ARRAY_SIZE(song_note_YOU),
    };

#ifdef __cplusplus
}
#endif

#endif // SONG_LIB_H
