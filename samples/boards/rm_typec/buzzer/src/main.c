#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

// Musical note frequencies (Hz)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
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

// Game of Thrones theme song
static const struct note_duration game_of_thrones[] = {
	{NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_D4, 250}, {NOTE_F4, 250},
	{NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_D4, 250}, {NOTE_F4, 250},
	{NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_D4, 250}, {NOTE_F4, 250},
	{NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_E4, 250}, {NOTE_F4, 250},
	{NOTE_G4, 750}, {NOTE_C4, 750}, {NOTE_D4, 250}, {NOTE_F4, 250},
	{NOTE_G4, 500}, {NOTE_C4, 500}, {NOTE_D4, 250}, {NOTE_F4, 250},
	{NOTE_D4, 500}, {NOTE_F4, 500}, {NOTE_A4, 250}, {NOTE_B4, 250},
	{NOTE_C5, 1000}, {NOTE_REST, 500},
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
int play_song(enum song_type song)
{
	if (song >= SONG_COUNT) {
		printk("Error: Invalid song type\n");
		return -1;
	}
	
	if (!device_is_ready(buzzer_dev)) {
		printk("Error: PWM device not ready\n");
		return -1;
	}
	
	printk("Playing song %d...\n", song);
	
	const struct note_duration *notes = songs[song].notes;
	size_t note_count = songs[song].count;
	
	for (size_t i = 0; i < note_count; i++) {
		play_note(notes[i].note, notes[i].duration);
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

	while (1) {
		// Play different songs in sequence
		play_song(SONG_BEEP_TEST);
		k_msleep(1000);
		
		play_song(SONG_TWINKLE_STAR);
		k_msleep(2000);
		
		play_song(SONG_HAPPY_BIRTHDAY);
		k_msleep(2000);
		
		play_song(SONG_GAME_OF_THRONES);
		k_msleep(3000);
	}

	return 0;
}
