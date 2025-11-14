#include "MIDI_Player.h"

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

	uint8_t status = VS1053_SciWrite(VS1053_WRITE_CMD, (MIDI_CHAN_PROGRAM | chan));
	status |= VS1053_SciWrite(VS1053_WRITE_CMD,inst);

    return status;
}

uint8_t midi_SetChannelVolume(uint8_t chan, uint8_t vol)
{
	if (chan > 15) return MIDI_ERROR;
	if (vol > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_CHAN_MSG | chan);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_CHAN_VOLUME);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, vol);

	return status;
}

uint8_t midi_SetChannelBank(uint8_t chan, uint8_t bank)
{
	if (chan > 15) return MIDI_ERROR;
	if (bank > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_CHAN_MSG | chan);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_CHAN_BANK);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, bank);

	return status;
}

uint8_t midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_NOTE_ON | chan);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, n);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, vel);

	return status;
}

/* Send mp3 buffer to VS1053 */
uint8_t midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel)
{
	if (chan > 15) return MIDI_ERROR;
	if (n > 127) return MIDI_ERROR;
	if (vel > 127) return MIDI_ERROR;

	uint8_t status = VS1053_SciWrite(VS1053_WRITE_CMD, MIDI_NOTE_OFF | chan);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, n);
	status |= VS1053_SciWrite(VS1053_WRITE_CMD, vel);

	return status;
}

void ToFSensor_sucess()
{
	for (int i =0; i < 3; i++){
		for (uint8_t i=60; i<69; i++) {
			midiNoteOn(0, i, 127);
			HAL_Delay(100);
			midiNoteOff(0, i, 127);
		}
		HAL_Delay(1000);
	}
}

void ToFSensor_failure()
{
	for (int i =0; i < 3; i++){
		midiNoteOn(0, 63, 127);
		HAL_Delay(100);
		midiNoteOff(0, 63, 127);
		midiNoteOn(0, 60, 127);
		HAL_Delay(900);
		midiNoteOff(0, 60, 127);

		HAL_Delay(1000);
	}
}
