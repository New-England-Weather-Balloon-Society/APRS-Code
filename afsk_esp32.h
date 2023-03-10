/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifdef ESP32

#ifndef __AFSK_ESP32_H__
#define __AFSK_ESP32_H__

#include <stdint.h>

#include "config.h"
#include <Arduino.h>

#define  PWM_CHANNEL 0
#define PWM_FREQ  62500       // Set it the same as the arduino default - 62500 hz
#define PWM_RESOLUTION  8     // 8 bit PWM

#define AFSK_ISR void IRAM_ATTR Esp32IRQ()

// Exported consts
extern const uint32_t MODEM_CLOCK_RATE;
extern const uint8_t REST_DUTY;
extern const uint16_t TABLE_SIZE;
extern const uint32_t PLAYBACK_RATE;

// Exported vars
extern const uint8_t afsk_sine_table[];

inline uint8_t afsk_read_sample(int phase)
{
  // ESP32 has tons of flash, so we just keep it in memory.  No special PROGMEM gyrations needed.
  return (afsk_sine_table[phase]);
}

inline void afsk_output_sample(uint8_t s)
{
  ledcWrite(PWM_CHANNEL, s);      // Change PWM frequency
}

inline void afsk_clear_interrupt_flag()
{
  // We don't need this on the ESP32.  We set up the timer to automatically reload after triggering.
}

#ifdef DEBUG_MODEM
inline uint16_t afsk_timer_counter()
{
   // Not sure what this was doing in AVR.  Just leaving blank.
}

inline int afsk_isr_overrun()
{
   // Not needed for ESP32.
}
#endif


// Exported functions
void afsk_setup();
void afsk_send(const uint8_t *buffer, int len);
void afsk_start();
bool afsk_flush();
void afsk_isr();
void afsk_timer_setup();
void afsk_timer_start();
void afsk_timer_stop();
void Esp32IRQ();
#ifdef DEBUG_MODEM
void afsk_debug();
#endif

#endif
#endif // ESP32
