#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

// Musical note frequencies (Hz) - Extended range
// Octave 2
#define NOTE_C2   65
#define NOTE_CS2  69   // C# / Db
#define NOTE_D2   73
#define NOTE_DS2  78   // D# / Eb
#define NOTE_E2   82
#define NOTE_F2   87
#define NOTE_FS2  92   // F# / Gb
#define NOTE_G2   98
#define NOTE_GS2  104  // G# / Ab
#define NOTE_A2   110
#define NOTE_AS2  117  // A# / Bb
#define NOTE_B2   123

// Octave 3
#define NOTE_C3   131
#define NOTE_CS3  139  // C# / Db
#define NOTE_D3   147
#define NOTE_DS3  156  // D# / Eb
#define NOTE_E3   165
#define NOTE_F3   175
#define NOTE_FS3  185  // F# / Gb
#define NOTE_G3   196
#define NOTE_GS3  208  // G# / Ab
#define NOTE_A3   220
#define NOTE_AS3  233  // A# / Bb
#define NOTE_B3   247

// Octave 4 (Middle C)
#define NOTE_C4   262
#define NOTE_CS4  277  // C# / Db
#define NOTE_D4   294
#define NOTE_DS4  311  // D# / Eb
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370  // F# / Gb
#define NOTE_G4   392
#define NOTE_GS4  415  // G# / Ab
#define NOTE_A4   440
#define NOTE_AS4  466  // A# / Bb
#define NOTE_B4   494

// Octave 5
#define NOTE_C5   523
#define NOTE_CS5  554  // C# / Db
#define NOTE_D5   587
#define NOTE_DS5  622  // D# / Eb
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740  // F# / Gb
#define NOTE_G5   784
#define NOTE_GS5  831  // G# / Ab
#define NOTE_A5   880
#define NOTE_AS5  932  // A# / Bb
#define NOTE_B5   988

// Octave 6
#define NOTE_C6   1047
#define NOTE_CS6  1109  // C# / Db
#define NOTE_D6   1175
#define NOTE_DS6  1245  // D# / Eb
#define NOTE_E6   1319
#define NOTE_F6   1397
#define NOTE_FS6  1480  // F# / Gb
#define NOTE_G6   1568
#define NOTE_GS6  1661  // G# / Ab
#define NOTE_A6   1760
#define NOTE_AS6  1865  // A# / Bb
#define NOTE_B6   1976

// Octave 7
#define NOTE_C7   2093
#define NOTE_CS7  2217  // C# / Db
#define NOTE_D7   2349
#define NOTE_DS7  2489  // D# / Eb
#define NOTE_E7   2637
#define NOTE_F7   2794
#define NOTE_FS7  2960  // F# / Gb
#define NOTE_G7   3136
#define NOTE_GS7  3322  // G# / Ab
#define NOTE_A7   3520
#define NOTE_AS7  3729  // A# / Bb
#define NOTE_B7   3951

#define NOTE_REST 0   // 休止符

// Note and duration structure
struct note_duration {
    int note;     /* hz */
    int duration; /* msec */
};

// Song enumeration
enum song_type {
    SONG_TWINKLE_STAR,
    SONG_HAPPY_BIRTHDAY,
    SONG_BEEP_TEST,
    SONG_GAME_OF_THRONES,
    SONG_SUPER_MARIO,
    SONG_MAX_VERSTAPPEN,
    SONG_PIRATES_CARIBBEAN,
    SONG_COUNT
};

// Get PWM device and channel from buzzer0 alias
#define BUZZER_NODE DT_ALIAS(buzzer0)
#define BUZZER_PWM_CTLR DT_PWMS_CTLR(BUZZER_NODE)
#define BUZZER_PWM_CHANNEL DT_PWMS_CHANNEL(BUZZER_NODE)

// Compile-time check for buzzer0 alias existence
#if !DT_NODE_EXISTS(BUZZER_NODE)
#error "buzzer0 alias is not defined in device tree"
#endif

// Global buzzer device
static const struct device *buzzer_dev;

// Song definitions
static const struct note_duration twinkle_star[] = {
    {NOTE_C4, 500}, {NOTE_C4, 500}, {NOTE_G4, 500}, {NOTE_G4, 500},
    {NOTE_A4, 500}, {NOTE_A4, 500}, {NOTE_G4, 1000},
    {NOTE_F4, 500}, {NOTE_F4, 500}, {NOTE_E4, 500}, {NOTE_E4, 500},
    {NOTE_D4, 500}, {NOTE_D4, 500}, {NOTE_C4, 1000},
};

static const struct note_duration happy_birthday[] = {
    {NOTE_C4, 250}, {NOTE_C4, 250}, {NOTE_D4, 500}, {NOTE_C4, 500},
    {NOTE_F4, 500}, {NOTE_E4, 1000},
    {NOTE_C4, 250}, {NOTE_C4, 250}, {NOTE_D4, 500}, {NOTE_C4, 500},
    {NOTE_G4, 500}, {NOTE_F4, 1000},
};

static const struct note_duration beep_test[] = {
    {NOTE_A4, 200}, {NOTE_REST, 100}, {NOTE_A4, 200}, {NOTE_REST, 100},
    {NOTE_A4, 200}, {NOTE_REST, 500},
};

// Game of Thrones theme song - Enhanced with extended note range
static const struct note_duration game_of_thrones[] = {
    // Opening melody - lower octave for dramatic effect
    {NOTE_G3, 500}, {NOTE_C3, 500}, {NOTE_DS3, 250}, {NOTE_F3, 250},
    {NOTE_G3, 500}, {NOTE_C3, 500}, {NOTE_DS3, 250}, {NOTE_F3, 250},
    {NOTE_G3, 500}, {NOTE_C3, 500}, {NOTE_DS3, 250}, {NOTE_F3, 250},
    {NOTE_G3, 500}, {NOTE_C3, 500}, {NOTE_E3, 250}, {NOTE_F3, 250},
    
    // Rising melody with octave jump
    {NOTE_G3, 500}, {NOTE_C4, 500}, {NOTE_DS4, 250}, {NOTE_F4, 250},
    {NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_DS4, 250}, {NOTE_F4, 250},
    
    // Climax with higher notes
    {NOTE_AS4, 250}, {NOTE_C5, 250}, {NOTE_D5, 500}, {NOTE_G4, 500},
    {NOTE_AS4, 250}, {NOTE_C5, 250}, {NOTE_D5, 750}, {NOTE_REST, 250},
    
    // Second theme - more complex harmony
    {NOTE_F4, 500}, {NOTE_AS3, 500}, {NOTE_C4, 500}, {NOTE_D4, 500},
    {NOTE_DS4, 500}, {NOTE_F4, 500}, {NOTE_G4, 1000},
    
    // Epic finale with wide range
    {NOTE_C3, 250}, {NOTE_G3, 250}, {NOTE_C4, 250}, {NOTE_G4, 250},
    {NOTE_C5, 500}, {NOTE_AS4, 500}, {NOTE_G4, 500}, {NOTE_F4, 500},
    {NOTE_DS4, 500}, {NOTE_D4, 500}, {NOTE_C4, 1000}, {NOTE_REST, 500},
    
    // Final dramatic notes
    {NOTE_G2, 1000}, {NOTE_C3, 1500}, {NOTE_REST, 1000},
};

// Super Mario Bros Theme Song
static const struct note_duration super_mario[] = {
    // Main theme opening
    {NOTE_E5, 150}, {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_E5, 150},
    {NOTE_REST, 150}, {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_REST, 150},
    {NOTE_G5, 150}, {NOTE_REST, 450}, {NOTE_G4, 150}, {NOTE_REST, 450},
    
    // First verse
    {NOTE_C5, 150}, {NOTE_REST, 300}, {NOTE_G4, 150}, {NOTE_REST, 300},
    {NOTE_E4, 150}, {NOTE_REST, 300}, {NOTE_A4, 150}, {NOTE_REST, 150},
    {NOTE_B4, 150}, {NOTE_REST, 150}, {NOTE_AS4, 150}, {NOTE_A4, 150},
    {NOTE_REST, 150},
    
    // Second part
    {NOTE_G4, 200}, {NOTE_E5, 200}, {NOTE_G5, 200}, {NOTE_A5, 150},
    {NOTE_REST, 150}, {NOTE_F5, 150}, {NOTE_G5, 150}, {NOTE_REST, 150},
    {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_C5, 150}, {NOTE_D5, 150},
    {NOTE_B4, 150}, {NOTE_REST, 300},
    
    // Repeat opening
    {NOTE_C5, 150}, {NOTE_REST, 300}, {NOTE_G4, 150}, {NOTE_REST, 300},
    {NOTE_E4, 150}, {NOTE_REST, 300}, {NOTE_A4, 150}, {NOTE_REST, 150},
    {NOTE_B4, 150}, {NOTE_REST, 150}, {NOTE_AS4, 150}, {NOTE_A4, 150},
    {NOTE_REST, 150},
    
    // Final section
    {NOTE_G4, 200}, {NOTE_E5, 200}, {NOTE_G5, 200}, {NOTE_A5, 150},
    {NOTE_REST, 150}, {NOTE_F5, 150}, {NOTE_G5, 150}, {NOTE_REST, 150},
    {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_C5, 150}, {NOTE_D5, 150},
    {NOTE_B4, 150}, {NOTE_REST, 300},
    
    // Ending
    {NOTE_G5, 150}, {NOTE_FS5, 150}, {NOTE_F5, 150}, {NOTE_DS5, 150},
    {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_GS4, 150}, {NOTE_A4, 150},
    {NOTE_C5, 150}, {NOTE_REST, 150}, {NOTE_A4, 150}, {NOTE_C5, 150},
    {NOTE_D5, 150}, {NOTE_REST, 300},
    
    // Classic Mario ending - matching the sheet music (1 1)
    {NOTE_C6, 300}, {NOTE_C6, 600},
};

// Max Verstappen Theme Song - "dudududu Max Verstappen"
static const struct note_duration max_verstappen[] = {
    // "du du du du du"
    {NOTE_G5, 200}, {NOTE_G5, 200}, {NOTE_G5, 200}, {NOTE_D6, 400},
    {NOTE_REST, 200},
    
    // "Max Ver-stap-pen"
    {NOTE_C6, 300}, {NOTE_C6, 300}, {NOTE_AS5, 300}, {NOTE_A5, 600},
    {NOTE_REST, 400},
    
    // "du du du du du"
    {NOTE_G5, 200}, {NOTE_G5, 200}, {NOTE_G5, 200}, {NOTE_D6, 400},
    {NOTE_REST, 200},
    
    // "Max Ver-stap-pen"
    {NOTE_C6, 300}, {NOTE_C6, 300}, {NOTE_AS5, 300}, {NOTE_A5, 600},
    {NOTE_REST, 400},
};

// Pirates of the Caribbean - "He's a Pirate" Theme
static const struct note_duration pirates_caribbean[] = {
    // Main theme opening
    {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_D5, 200}, {NOTE_D5, 200},
    {NOTE_D5, 200}, {NOTE_E5, 200}, {NOTE_F5, 200}, {NOTE_F5, 200},
    {NOTE_F5, 200}, {NOTE_G5, 200}, {NOTE_E5, 200}, {NOTE_E5, 200},
    {NOTE_D5, 200}, {NOTE_C5, 200}, {NOTE_D5, 400},
    
    // Second phrase
    {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_D5, 200}, {NOTE_D5, 200},
    {NOTE_D5, 200}, {NOTE_E5, 200}, {NOTE_F5, 200}, {NOTE_F5, 200},
    {NOTE_F5, 200}, {NOTE_G5, 200}, {NOTE_E5, 200}, {NOTE_E5, 200},
    {NOTE_D5, 200}, {NOTE_C5, 200}, {NOTE_D5, 400},
    
    // Bridge section
    {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_D5, 200}, {NOTE_F5, 200},
    {NOTE_G5, 200}, {NOTE_A5, 200}, {NOTE_D5, 400}, {NOTE_REST, 200},
    {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_D5, 200}, {NOTE_F5, 200},
    {NOTE_G5, 200}, {NOTE_A5, 200}, {NOTE_D5, 400},
    
    // Climactic section
    {NOTE_D5, 200}, {NOTE_E5, 200}, {NOTE_F5, 200}, {NOTE_G5, 200},
    {NOTE_A5, 200}, {NOTE_AS5, 200}, {NOTE_A5, 200}, {NOTE_G5, 200},
    {NOTE_F5, 200}, {NOTE_E5, 200}, {NOTE_D5, 200}, {NOTE_C5, 200},
    {NOTE_D5, 400}, {NOTE_REST, 200},
    
    // Return to main theme
    {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_D5, 200}, {NOTE_D5, 200},
    {NOTE_D5, 200}, {NOTE_E5, 200}, {NOTE_F5, 200}, {NOTE_F5, 200},
    {NOTE_F5, 200}, {NOTE_G5, 200}, {NOTE_E5, 200}, {NOTE_E5, 200},
    {NOTE_D5, 200}, {NOTE_C5, 200}, {NOTE_D5, 400},
    
    // Fast section
    {NOTE_F5, 150}, {NOTE_G5, 150}, {NOTE_A5, 150}, {NOTE_AS5, 150},
    {NOTE_A5, 150}, {NOTE_G5, 150}, {NOTE_F5, 150}, {NOTE_E5, 150},
    {NOTE_F5, 150}, {NOTE_G5, 150}, {NOTE_A5, 150}, {NOTE_G5, 150},
    {NOTE_F5, 150}, {NOTE_E5, 150}, {NOTE_D5, 300},
    
    // Adventure melody
    {NOTE_A5, 200}, {NOTE_G5, 200}, {NOTE_F5, 200}, {NOTE_E5, 200},
    {NOTE_D5, 200}, {NOTE_C5, 200}, {NOTE_AS4, 200}, {NOTE_A4, 200},
    {NOTE_D5, 400}, {NOTE_F5, 400}, {NOTE_A5, 600},
    
    // Final triumphant section
    {NOTE_D6, 200}, {NOTE_C6, 200}, {NOTE_AS5, 200}, {NOTE_A5, 200},
    {NOTE_G5, 200}, {NOTE_F5, 200}, {NOTE_E5, 200}, {NOTE_D5, 200},
    {NOTE_A5, 300}, {NOTE_F5, 300}, {NOTE_D5, 600},
    
    // Ending
    {NOTE_A4, 200}, {NOTE_D5, 200}, {NOTE_F5, 200}, {NOTE_A5, 400},
    {NOTE_D6, 800}, {NOTE_REST, 400},
};

// Song table
static const struct {
    const struct note_duration *notes;
    size_t count;
} songs[SONG_COUNT] = {
    [SONG_TWINKLE_STAR] = {twinkle_star, ARRAY_SIZE(twinkle_star)},
    [SONG_HAPPY_BIRTHDAY] = {happy_birthday, ARRAY_SIZE(happy_birthday)},
    [SONG_BEEP_TEST] = {beep_test, ARRAY_SIZE(beep_test)},
    [SONG_GAME_OF_THRONES] = {game_of_thrones, ARRAY_SIZE(game_of_thrones)},
    [SONG_SUPER_MARIO] = {super_mario, ARRAY_SIZE(super_mario)},
    [SONG_MAX_VERSTAPPEN] = {max_verstappen, ARRAY_SIZE(max_verstappen)},
    [SONG_PIRATES_CARIBBEAN] = {pirates_caribbean, ARRAY_SIZE(pirates_caribbean)},
};

// Play a single note
static void play_note(int frequency, int duration_ms)
{
    if (frequency == NOTE_REST) {
        // Rest - turn off buzzer
        pwm_set(buzzer_dev, BUZZER_PWM_CHANNEL, 0, 0, 0);
    } else {
        // Calculate period in microseconds
        uint32_t period_usec = 1000000 / frequency;
        // Set 50% duty cycle
        pwm_set(buzzer_dev, BUZZER_PWM_CHANNEL, PWM_USEC(period_usec), PWM_USEC(period_usec / 2), 0);
    }
    
    k_msleep(duration_ms);
    
    // Turn off buzzer after note
    pwm_set(buzzer_dev, BUZZER_PWM_CHANNEL, 0, 0, 0);
    k_msleep(50); // Small gap between notes
}

// Play song interface function
int play_song(enum song_type song, double speed_multiplier, double pitch_multiplier)
{
    if (song >= SONG_COUNT) {
        printk("Error: Invalid song type\n");
        return -1;
    }
    
    if (!device_is_ready(buzzer_dev)) {
        printk("Error: PWM device not ready\n");
        return -1;
    }
    
    if (speed_multiplier <= 0.0) {
        printk("Error: Invalid speed multiplier\n");
        return -1;
    }
    
    if (pitch_multiplier <= 0.0) {
        printk("Error: Invalid pitch multiplier\n");
        return -1;
    }
    
    printk("Playing song %d at %.2fx speed and %.2fx pitch...\n", song, speed_multiplier, pitch_multiplier);
    
    const struct note_duration *notes = songs[song].notes;
    size_t note_count = songs[song].count;
    
    for (size_t i = 0; i < note_count; i++) {
        int adjusted_duration = (int)(notes[i].duration / speed_multiplier);
        int adjusted_frequency = (notes[i].note == NOTE_REST) ? NOTE_REST : (int)(notes[i].note * pitch_multiplier);
        play_note(adjusted_frequency, adjusted_duration);
    }
    
    printk("Song finished\n");
    return 0;
}

int main(void)
{
    buzzer_dev = DEVICE_DT_GET(BUZZER_PWM_CTLR);

    if (!device_is_ready(buzzer_dev)) {
        printk("Error: PWM device not ready\n");
        return -1;
    }

    printk("Music player started (PWM channel %d)\n", BUZZER_PWM_CHANNEL);

    // Play Pirates of the Caribbean theme at 1.25x speed and 2.0x pitch (one octave higher)
    play_song(SONG_PIRATES_CARIBBEAN, 1.25, 2.0);

    return 0;
}
