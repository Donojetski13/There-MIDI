#ifndef MIDI_PLAYER_H_
#define MIDI_PLAYER_H_

#include "VS1053.h"
// defines
// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define BANK_DEFAULT 0x00
#define BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define MEL_GRAND_PIANO 1
#define MEL_VIBRAPHONE 12
#define MEL_A_GUITAR 25
#define MEL_E_GUITAR 28
#define MEL_VIOLIN 41
#define MEL_TRUMPET 57
#define MEL_A_SAX 66
#define MEL_OCARINA 80
#define MEL_GOBLINS 102
#define MEL_SCIFI 104
#define MEL_Timpani 48
#define MEL_HELICOPTER 126
#define MEL_FLUTE 74
#define MEL_CHOIR 92
#define MEL_MARIMBA 13
#define MEL_TELEPHONE 125
#define MEL_GUNSHOTS 128

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_SUSTENUTO 0x42
#define MIDI_CHAN_REVERB 0x5b
#define MIDI_CHAN_REVERB_DECAY 0x0C
#define MIDI_CHAN_PROGRAM 0xC0


/* Functions */
uint8_t midi_Init();
uint8_t midi_Treble_Bass(int Treble, uint8_t bass);
uint8_t midi_SetInstrument(uint8_t chan, uint8_t inst);
uint8_t midi_SetChannelVolume(uint8_t chan, uint8_t vol);
uint8_t midi_SetChannelBank(uint8_t chan, uint8_t bank);
uint8_t midi_SetChannelReverb(uint8_t chan, uint8_t bank);
uint8_t midi_SetChannelReverbDecay(uint8_t chan, uint8_t decay);
uint8_t midi_SetChannelSustenuto(uint8_t chan, uint8_t length);
uint8_t midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel);
uint8_t midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel);
void ToFSensor_sucess();
void ToFSensor_failure();

#endif /* MP3_PLAYER_H_ */
