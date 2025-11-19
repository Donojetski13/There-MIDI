#include "MIDI_Player.h"
#include <stdio.h>

#define MIDI_ERROR ((uint8_t) 13U)

/* Initialize VS1053 & Open a file */
uint8_t midi_Init()
{
	/* Initialize VS1053 */
    uint8_t status = VS1053_Init();

    status |= midi_SetChannelBank(0, BANK_MELODY);
    status |= midi_SetInstrument(0, MEL_OCARINA);
    status |= midi_SetChannelVolume(0, 127);

    return status;
}

uint8_t midi_SetInstrument(uint8_t chan, uint8_t inst)
{
	if (chan > 15) return MIDI_ERROR;
	inst --; // page 32 has instruments starting with 1 not 0 :(
	if (inst > 127) return MIDI_ERROR;

//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_CHAN_PROGRAM);
	status |= VS1053_SdiWrite(inst);
    return status;
}

uint8_t midi_SetChannelVolume(uint8_t chan, uint8_t vol)
{
	if (chan > 15) return MIDI_ERROR;
	if (vol > 127) return MIDI_ERROR;

	//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_CHAN_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_VOLUME);
	status |= VS1053_SdiWrite(vol);

	return status;
}

uint8_t midi_SetChannelBank(uint8_t chan, uint8_t bank)
{
	if (chan > 15) return MIDI_ERROR;
	if (bank > 127) return MIDI_ERROR;

	//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_CHAN_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_BANK);
	status |= VS1053_SdiWrite(bank);

	return status;
}

uint8_t midi_SetChannelReverb(uint8_t chan, uint8_t fx)
{
	if (chan > 15) return MIDI_ERROR;
	if (fx) fx = 127;

	//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_CHAN_MSG);
	status |= VS1053_SdiWrite(MIDI_CHAN_REVERB);
	status |= VS1053_SdiWrite(fx);

	return status;
}

uint8_t midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_NOTE_ON);
	status |= VS1053_SdiWrite(n);
	status |= VS1053_SdiWrite(vel);

	return status;
}

/* Send mp3 buffer to VS1053 */
uint8_t midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	//	uint8_t status = VS1053_SdiWrite(0);
	uint8_t status = VS1053_SdiWrite(MIDI_NOTE_OFF);
	status |= VS1053_SdiWrite(n);
	status |= VS1053_SdiWrite(vel);

	return status;
}

void ToFSensor_sucess()
{
	printf("Playing Init success\n");
		for (uint8_t i=60; i<69; i++) {
			midiNoteOn(0, i, 127);
			HAL_Delay(100);
			midiNoteOff(0, i, 127);
		}
	printf("Stopped Playing Init success\n");
}

void ToFSensor_failure()
{
	printf("Playing Init failed\n");
	for (uint8_t i=69; i>=60; i--) {
		midiNoteOn(0, i, 127);
		HAL_Delay(100);
		midiNoteOff(0, i, 127);
	}
	midiNoteOn(0, 60, 127);
	HAL_Delay(500);
	midiNoteOff(0, 60, 127);
	printf("Stopped Playing Init failed\n");
}
