#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "memory_protection.h"
#include <main.h>
#include "audio/audio_thread.h"
#include "audio/microphone.h"
#include "audio/play_sound_file.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "button.h"
#include "leds.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"

#include "arm_math.h"
#include "arm_const_structs.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static BSEMAPHORE_DECL(rec_buffer_filled, true);
static BSEMAPHORE_DECL(fft_computed, true);

static THD_WORKING_AREA(selector_thd_wa, 2048);
static THD_WORKING_AREA(record_thd_wa, 2048);

// Microphones data variables.
static int16_t *rec_data;
static uint16_t rec_num_samples;

// FFT variables and definitions.
#define FFT_ARRAY_LEN 2048 // The array length includes real and complex parts. The resolution of the FFT will be: 16000 (=> sampling frequency) / 1024 (=> FFT_ARRAY_LEN/2) = 15.625 Hz.
uint16_t mic0_data_complex_index = 0;
float32_t mic0_data_complex[FFT_ARRAY_LEN];
static float32_t fft_output[FFT_ARRAY_LEN/2];
uint32_t fftSize = FFT_ARRAY_LEN/2;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
uint32_t fft_index = 0;
float32_t fft_max_value;



static void mic_callback(int16_t *data, uint16_t num_samples) {
	rec_data = data;
	rec_num_samples = num_samples;
	chBSemSignal(&rec_buffer_filled);
	return;
}

static THD_FUNCTION(record_thd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	uint16_t i = 0;
	uint16_t j = 0;

	while(1) {
		chBSemWait(&rec_buffer_filled);

		//set_led(LED3, 1);

		for(i=0; i<rec_num_samples; i+=4) { // Consider only one microphone for FFT.
			mic0_data_complex[mic0_data_complex_index++] = rec_data[i];
			mic0_data_complex[mic0_data_complex_index++] = 0; // Set complex part to zero.

			if(mic0_data_complex_index == FFT_ARRAY_LEN) {
				mic0_data_complex_index = 0;

				//set_led(LED5, 1);

				// Process the data through the CFFT/CIFFT module.
				arm_cfft_f32(&arm_cfft_sR_f32_len1024, mic0_data_complex, ifftFlag, doBitReverse);

				// Process the data through the Complex Magnitude Module for calculating the magnitude at each bin.
				arm_cmplx_mag_f32(mic0_data_complex, fft_output, fftSize);

				// Set the magnitude to zero in the "fft_output" array starting from index 148 to not consider frequencies >= 16000/1024*148 = 2312.5 Hz.
				// This is done to avoid incorrectly detecting low frequencies emitted by the robot itself (e.g. when the robot emits about 800 Hz then
				// the FFT often detects around 3 KHz and the RGB colors continuously pass from blue to red and vice versa).
				// Basically this is only a workaround to let the demo behave correctly.
				for(j=148; j<(FFT_ARRAY_LEN/2); j++) {
					fft_output[j] = 0.0;
				}

				// Calculates maxValue and returns corresponding BIN value.
				arm_max_f32(fft_output, fftSize, &fft_max_value, &fft_index);

				chBSemSignal(&fft_computed);

				//set_led(LED5, 0);
			}
		}

		//set_led(LED3, 0);

	}
}

static THD_FUNCTION(selector_thd, arg) {
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

	float32_t curr_freq = 0.0;
	uint16_t rgb_from_freq = 0;
	uint16_t found_count = 0;
	uint16_t lost_count = 0;
	uint16_t refresh_count = 0;
	uint16_t temp_dist = 0;
	uint16_t play_freq = 0;
	uint8_t play_state = 0;
	uint16_t temp_dist_filt = 0;
	uint8_t play_rnd_wav = 1;
	uint16_t play_rnd_wav_count = 0;
	uint8_t rnd_index = 0;
	uint8_t play_sound_state = 0;
	uint8_t update_rate_count = 1;

	setSoundFileVolume(20);
	calibrate_ir();

    while(1) {
    	switch(get_selector()) {

			case 0: // Detect frequencies with FFT and playback tones based on the distance from ToF.
				// The buffer for the FFT will be filled every 64 ms:
				// - we get 160 samples (for each micro) every 10 ms
				// - the buffer is 1024 long => 1024/160 * 10 ms = 64 ms
				chBSemWait(&fft_computed);
				if(fft_index < FFT_ARRAY_LEN/4) { // Can only detect up to 8 KHz signals when sampling @ 16 KHz.
					curr_freq = (float32_t)fft_index*16000.0/(float32_t)fftSize; // 16000 / 1024 = 15.625 Hz for each BIN.
				}
				//if (SDU1.config->usbp->state == USB_ACTIVE) { // Skip printing if port not opened.
				//	chprintf((BaseSequentialStream *)&SDU1, "ind=%5d, val=%9.3f, freq=%7.2f\r\n", fft_index, fft_max_value, curr_freq);
				//}

				// Map the frequency detected to the RGB LEDs.
				rgb_from_freq = (uint16_t)(curr_freq);
				if(rgb_from_freq > 2176) { // Consider frequencies up to 2176 Hz.
					rgb_from_freq = 2176;
				}
				rgb_from_freq = rgb_from_freq>>7; // Range is 0..17
				if(rgb_from_freq < 2) { // 0..256 Hz
					set_rgb_led(LED2, 0, 0, 0);
					set_rgb_led(LED4, 0, 0, 0);
					set_rgb_led(LED6, 0, 0, 0);
					set_rgb_led(LED8, 0, 0, 0);
				} else if(rgb_from_freq <= 7) { // 256..896 Hz, blue LED "power" range is 1..6
					set_rgb_led(LED2, 0, 0, rgb_from_freq - 1);
					set_rgb_led(LED4, 0, 0, rgb_from_freq - 1);
					set_rgb_led(LED6, 0, 0, rgb_from_freq - 1);
					set_rgb_led(LED8, 0, 0, rgb_from_freq - 1);
				} else if(rgb_from_freq <= 12) { // 896..1536 Hz, green LED "power" range is 7..11
					set_rgb_led(LED2, 0, rgb_from_freq - 1, 0);
					set_rgb_led(LED4, 0, rgb_from_freq - 1, 0);
					set_rgb_led(LED6, 0, rgb_from_freq - 1, 0);
					set_rgb_led(LED8, 0, rgb_from_freq - 1, 0);
				} else { // 1536..2176 Hz, red LED "power" range is 12..16
					set_rgb_led(LED2, rgb_from_freq - 1, 0, 0);
					set_rgb_led(LED4, rgb_from_freq - 1, 0, 0);
					set_rgb_led(LED6, rgb_from_freq - 1, 0, 0);
					set_rgb_led(LED8, rgb_from_freq - 1, 0, 0);
				}

				//if (SDU1.config->usbp->state == USB_ACTIVE) { // Skip printing if port not opened.
				//chprintf((BaseSequentialStream *)&SDU1, "range=%d mm\r\n", VL53L0X_get_dist_mm());
				//}

				switch(play_state) {
					case 0: // Detect someone coming in front of the robot.
						temp_dist = VL53L0X_get_dist_mm();
						if(temp_dist <= 500) {
							found_count++;
							if(found_count >= 20) { // If someone (or something) is detected within 50 cm for about 64 * 20 = 1280 ms, then start the frequencies playback.
								refresh_count = 0;
								lost_count = 0;
								play_state = 1;
								playSoundFile("Nice_to_meet_you.wav", SF_FORCE_CHANGE, 16000);
								waitSoundFileHasFinished();
							}
						} else {
							found_count = 0;
							dac_stop();

							// If enabled with the button, then play a random wav effect (out of 10) every minute when there is nothing in front
							// of the distance sensor.
							if(play_rnd_wav == 1) {
								play_rnd_wav_count++;
								if(play_rnd_wav_count >= 1000) { // About 1 minute (64 sec).
									play_rnd_wav_count = 0;
									rnd_index = rec_data[0]%4;
									switch(rnd_index) {
										case 0: playSoundFile("0.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 1: playSoundFile("1.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 2: playSoundFile("2.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 3: playSoundFile("3.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 4: playSoundFile("4.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 5: playSoundFile("5.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 6: playSoundFile("6.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 7: playSoundFile("7.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 8: playSoundFile("8.wav", SF_FORCE_CHANGE, 16000);
											break;
										case 9: playSoundFile("9.wav", SF_FORCE_CHANGE, 16000);
											break;
									}
									waitSoundFileHasFinished();
								}
							} else {
								play_rnd_wav_count = 0;
							}
						}
						break;

					case 1: // Detect someone goes away from the robot.
						temp_dist = VL53L0X_get_dist_mm();
						if(temp_dist > 500) {
							refresh_count = 0;
							temp_dist_filt = 0;
							lost_count++;
							if(lost_count >= 20) { // If nobody (or nothing) is detected within 50 cm for about 64 * 20 = 1280 ms, then stop the frequencies playback.
								dac_stop();
								found_count = 0;
								play_state = 0;
							}
						} else {
							temp_dist_filt += temp_dist;
							lost_count = 0;
							refresh_count++;
							if(refresh_count >= update_rate_count) { // Update the frequency every 64 ms.
								refresh_count = 0;
								temp_dist_filt /= update_rate_count;
								//if (SDU1.config->usbp->state == USB_ACTIVE) { // Skip printing if port not opened.
								//chprintf((BaseSequentialStream *)&SDU1, "filt=%d\r\n", temp_dist_filt);
								//}
								// Map the distance detected from the ToF to a frequency emitted through the speaker.
								if(temp_dist_filt < 100) {
									temp_dist_filt = 100;
								} else if(temp_dist_filt > 400) {
									temp_dist_filt = 400;
								}
								play_freq = (500 - temp_dist_filt)/10; // Range is 10 (far) ... 40 (near).
								play_freq = (play_freq*play_freq) + (play_freq<<4); // Range is 260 Hz (far) ... 2240 Hz (near).
								//if (SDU1.config->usbp->state == USB_ACTIVE) { // Skip printing if port not opened.
								//chprintf((BaseSequentialStream *)&SDU1, "play=%d\r\n", play_freq);
								//}
								dac_play(play_freq);
								temp_dist_filt = 0;
							}
						}
						break;
				}

				// Enabled/disable the random play of wav files when nothing is detected; this is done through the press of the button.
				if(button_is_pressed()) {
					play_rnd_wav = 1 - play_rnd_wav;
					while(button_is_pressed());
				}
				// Body LED turned on => random play of wav files enabled.
				// Body LED turned off => random play of wav files disabled.
				if(play_rnd_wav == 1) {
					set_body_led(1);
				} else {
					set_body_led(0);
				}
				break;

			case 1: // Play wav files from the micro sd when a proximity sensor is "touched".
				switch(play_sound_state) {
					case 0:
						if(get_calibrated_prox(0) >= 3000) {
							play_sound_state = 1;
						}
						if(get_calibrated_prox(1) >= 3000) {
							play_sound_state = 3;
						}
						if(get_calibrated_prox(2) >= 3000) {
							play_sound_state = 5;
						}
						if(get_calibrated_prox(3) >= 3000) {
							play_sound_state = 7;
						}
						if(get_calibrated_prox(4) >= 3000) {
							play_sound_state = 9;
						}
						if(get_calibrated_prox(5) >= 3000) {
							play_sound_state = 11;
						}
						if(get_calibrated_prox(6) >= 3000) {
							play_sound_state = 13;
						}
						if(get_calibrated_prox(7) >= 3000) {
							play_sound_state = 15;
						}
						break;

					case 1:
						if(get_calibrated_prox(0) <= 300) {
							play_sound_state = 2;
						}
						break;
					case 2:
						playSoundFile("0.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 3:
						if(get_calibrated_prox(1) <= 300) {
							play_sound_state = 4;
						}
						break;
					case 4:
						playSoundFile("1.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 5:
						if(get_calibrated_prox(2) <= 300) {
							play_sound_state = 6;
						}
						break;
					case 6:
						playSoundFile("2.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 7:
						if(get_calibrated_prox(3) <= 300) {
							play_sound_state = 8;
						}
						break;
					case 8:
						playSoundFile("3.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 9:
						if(get_calibrated_prox(4) <= 300) {
							play_sound_state = 10;
						}
						break;
					case 10:
						playSoundFile("4.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 11:
						if(get_calibrated_prox(5) <= 300) {
							play_sound_state = 12;
						}
						break;
					case 12:
						playSoundFile("5.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 13:
						if(get_calibrated_prox(6) <= 300) {
							play_sound_state = 14;
						}
						break;
					case 14:
						playSoundFile("6.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

					case 15:
						if(get_calibrated_prox(7) <= 300) {
							play_sound_state = 16;
						}
						break;
					case 16:
						playSoundFile("7.wav", SF_FORCE_CHANGE, 16000);
						waitSoundFileHasFinished();
						play_sound_state = 0;
						break;

				}
				chThdSleepMilliseconds(20);
				break;
    	}
    }

}

int main(void) {

    halInit();
    chSysInit();
    mpu_init();

    // Inits the Inter Process Communication bus.
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	proximity_start();
    mic_start(mic_callback);
    usb_start();
    dac_start();
    spi_comm_start();
    VL53L0X_start();
    sdio_start();
    playSoundFileStart();

    chThdCreateStatic(selector_thd_wa, sizeof(selector_thd_wa), NORMALPRIO, selector_thd, NULL);
    chThdCreateStatic(record_thd_wa, sizeof(record_thd_wa), NORMALPRIO, record_thd, NULL);

    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
