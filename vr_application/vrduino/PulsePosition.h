/* PulsePosition Library for Teensy 3.x, LC, and 4.0

 * High resolution input and output of PPM encoded signals
 * http://www.pjrc.com/teensy/td_libs_PulsePosition.html
 * Copyright (c) 2019, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy
 * boards.  Please support PJRC's efforts to develop open source software by
 * purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Arduino.h>

#ifdef __AVR__
#error "Sorry, PulsePosition does not work on Teensy 2.0 and other AVR-based boards"
#elif defined(__IMXRT1062__)
#include "PulsePositionIMXRT.h"
#else

 // convert from microseconds to I/O clock ticks
#define CLOCKS_PER_MICROSECOND ((double)F_BUS / 1000000.0)
#endif
#define TX_MINIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MINIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MAXIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MAXIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_DEFAULT_SIGNAL_CLOCKS  (uint32_t)(TX_DEFAULT_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_SPACE_CLOCKS   (uint32_t)(TX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_FRAME_CLOCKS   (uint32_t)(TX_MINIMUM_FRAME * CLOCKS_PER_MICROSECOND)
#define TX_PULSE_WIDTH_CLOCKS     (uint32_t)(TX_PULSE_WIDTH * CLOCKS_PER_MICROSECOND)
#define RX_MINIMUM_SPACE_CLOCKS   (uint32_t)(RX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)


#define FTM0_SC_VALUE (FTM_SC_TOIE | FTM_SC_CLKS(1) | FTM_SC_PS(0))

#if defined(KINETISK)
#define CSC_CHANGE(reg, val)         ((reg)->csc = (val))
#define CSC_INTACK(reg, val)         ((reg)->csc = (val))
#define CSC_CHANGE_INTACK(reg, val)  ((reg)->csc = (val))
#define FRAME_PIN_SET()              *framePinReg = 1
#define FRAME_PIN_CLEAR()            *framePinReg = 0
#elif defined(KINETISL)
#define CSC_CHANGE(reg, val)         ({(reg)->csc = 0; while ((reg)->csc); (reg)->csc = (val);})
#define CSC_INTACK(reg, val)         ((reg)->csc = (val) | FTM_CSC_CHF)
#define CSC_CHANGE_INTACK(reg, val)  ({(reg)->csc = 0; while ((reg)->csc); (reg)->csc = (val) | FTM_CSC_CHF;})
#define FRAME_PIN_SET()              *(framePinReg + 4) = framePinMask
#define FRAME_PIN_CLEAR()            *(framePinReg + 8) = framePinMask

#define PULSEPOSITION_MAXCHANNELS 16

struct ftm_channel_struct {
  uint32_t csc;
  uint32_t cv;
};

class PulsePositionOutput
{
public:
  PulsePositionOutput(void);
  PulsePositionOutput(int polarity);
  bool begin(uint8_t txPin); // txPin can be 5,6,9,10,20,21,22,23
  bool begin(uint8_t txPin, uint8_t framePin);
  bool write(uint8_t channel, float microseconds);
  friend void ftm0_isr(void);
private:
  void isr(void);
  uint32_t pulse_width[PULSEPOSITION_MAXCHANNELS+1];
  uint32_t pulse_buffer[PULSEPOSITION_MAXCHANNELS+1];
  uint32_t pulse_remaining;
  volatile uint8_t *framePinReg;
  volatile uint8_t framePinMask;
  struct ftm_channel_struct *ftm;
  uint8_t state;
  uint8_t current_channel;
  uint8_t total_channels;
  uint8_t total_channels_buffer;
  uint8_t cscSet;
  uint8_t cscClear;
  static PulsePositionOutput *list[8];
  static uint8_t channelmask;
};


class PulsePositionInput
{
public:
  PulsePositionInput(void);
  PulsePositionInput(int polarity);
  bool begin(uint8_t rxPin); // rxPin can be 5,6,9,10,20,21,22,23
  int available(void);
  float read(uint8_t channel);
  friend void ftm0_isr(void);
private:
  void isr(void);
  struct ftm_channel_struct *ftm;
  uint32_t pulse_width[PULSEPOSITION_MAXCHANNELS];
  uint32_t pulse_buffer[PULSEPOSITION_MAXCHANNELS];
  uint32_t prev;
  uint8_t write_index;
  uint8_t total_channels;
  uint8_t cscEdge;
  bool available_flag;
  static bool overflow_inc;
  static uint16_t overflow_count;
  static PulsePositionInput *list[8];
  static uint8_t channelmask;
};

#endif
