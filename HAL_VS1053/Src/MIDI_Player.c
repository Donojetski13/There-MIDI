#include "MIDI_Player.h"
#include <stdio.h>

#define MIDI_ERROR ((uint8_t) 13U)
#define START_MSG ((uint8_t) 0U)

/* Initialize VS1053 & Open a file */
uint8_t midi_Init()
{
	/* Initialize VS1053 */
    uint8_t status = VS1053_Init();

    status |= midi_Treble_Bass(4, 8);
    status |= midi_SetChannelBank(0, BANK_MELODY);
    status |= midi_SetInstrument(0, MEL_VIOLIN);
    status |= midi_SetChannelVolume(0, 127);
    status |= midi_SetChannelReverb(0, 0);
    status |= midi_SetChannelReverbDecay(0, 80);
    status |= midi_SetChannelSustenuto(0, 127);

    return status;
}

uint8_t midi_Treble_Bass(int Treble, uint8_t bass)
{
	if ((Treble < -8 || Treble > 7) || bass > 15) return MIDI_ERROR;
	uint8_t t_lower_lim_freq = 0, b_lower_lim_freq = 0;
	uint16_t treble_bass_setting = ((Treble<<4)+t_lower_lim_freq)<<8;
	treble_bass_setting += ((bass<<4)+b_lower_lim_freq);

	uint8_t status = VS1053_SciWrite(VS1053_REG_BASS, treble_bass_setting);

	return status;
}
uint8_t midi_SetInstrument(uint8_t chan, uint8_t inst)
{
	if (chan > 15) return MIDI_ERROR;
	inst --; // page 32 has instruments starting with 1 not 0 :(
	if (inst > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_PROGRAM | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(inst);
    return status;
}

uint8_t midi_SetChannelVolume(uint8_t chan, uint8_t vol)
{
	if (chan > 15) return MIDI_ERROR;
	if (vol > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_MSG | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_VOLUME);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(vol);

	return status;
}

uint8_t midi_SetChannelBank(uint8_t chan, uint8_t bank)
{
	if (chan > 15) return MIDI_ERROR;
	if (bank > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_MSG | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_BANK);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(bank);

	return status;
}

uint8_t midi_SetChannelSustenuto(uint8_t chan, uint8_t length)
{
	if (chan > 15) return MIDI_ERROR;
	if (length> 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_MSG | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_SUSTENUTO);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(length);

	return status;
}
uint8_t midi_SetChannelReverb(uint8_t chan, uint8_t fx)
{
	if (chan > 15) return MIDI_ERROR;
	if (fx) fx = 127;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_MSG | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_REVERB);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(fx);

	return status;
}

uint8_t midi_SetChannelReverbDecay(uint8_t chan, uint8_t decay)
{
	if (chan > 15) return MIDI_ERROR;
	if (decay> 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_MSG | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_REVERB_DECAY);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(decay);

	return status;
}

uint8_t midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_NOTE_ON | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(n);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(vel);

	return status;
}

/* Send mp3 buffer to VS1053 */
uint8_t midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(MIDI_NOTE_OFF | chan);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(n);
	status |= VS1053_SdiWrite(START_MSG);
	status |= VS1053_SdiWrite(vel);

	return status;
}

void ToFSensor_sucess()
{
	printf("Playing Init success\n");
	midi_SetChannelVolume(0, 80);
	for (uint8_t i=60; i<69; i++) {
		midiNoteOn(0, i, 127);
		HAL_Delay(150);
		midiNoteOff(0, i, 127);
	}
	printf("Stopped Playing Init success\n");
}

void ToFSensor_failure()
{
	printf("Playing Init failed\n");
	uint8_t note = 70;
	midi_SetChannelVolume(0, 80);
	midi_Treble_Bass(7, 0);
	HAL_Delay(10);
	for (int i= 0; i< 2;i++){
		midiNoteOn(0, note, 127);
		HAL_Delay(300);
		midiNoteOff(0, note, 127);
		HAL_Delay(175);
	}
	midi_SetChannelReverb(0, 1);
	for (int i= 0; i< 1;i++){
	midiNoteOn(0, note, 127);
	HAL_Delay(300);
	midiNoteOff(0, note, 127);
	}
	printf("Stopped Playing Init failed\n");
}
