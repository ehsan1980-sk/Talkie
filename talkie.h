// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

#ifndef _Talkie_h_
#define _Talkie_h_

#include <Arduino.h>

enum TalkieDevice { // Can emulate two related but not-data-compatible devices
  TALKIE_TMS5220,   // "Classic" Talkie lib emulates TI 5220 (99/4A, etc.)
  TALKIE_TMS5100    // Or switch to TI 5100 mode (Speak & Spell, etc.)
};

class Talkie {
  public:
	Talkie(void);                                 // PWM
	Talkie(uint8_t cs, uint8_t clk, uint8_t dat); // DAC
	void say(const uint8_t *address, bool block=true);
	bool talking(void) const; // Poll this when block=false
	void mode(TalkieDevice mode = TALKIE_TMS5220);
};

#endif // end if _Talkie_h_
