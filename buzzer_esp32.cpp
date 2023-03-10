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

#include "config.h"
#include "buzzer.h"
#include "pin.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
#include <stdint.h>



// Module constants
static const unsigned long PWM_PERIOD = F_CPU / BUZZER_FREQ;
static const unsigned long ON_CYCLES = BUZZER_FREQ * BUZZER_ON_TIME;
static const unsigned long OFF_CYCLES = BUZZER_FREQ * BUZZER_OFF_TIME;
#if BUZZER_TYPE == 0  // active buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD;
#endif
#if BUZZER_TYPE == 1  // passive buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD / 2;
#endif

// Module variables
static volatile bool is_buzzer_on;
static volatile bool buzzing;
static volatile unsigned long myalarm;

// Exported functions
void buzzer_setup()
{
  Serial.println("In buzzer_setup()");
 pinMode(BUZZER_PIN, OUTPUT);
 pin_write(BUZZER_PIN, LOW);
  buzzing = false;
  is_buzzer_on = false;
  myalarm = 1;

  // Top is ICR1 (WGM1=14), p.135
  /* TODO ESP32
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // Set top to PWM_PERIOD
  ICR1 = PWM_PERIOD;

  // Enable interrupts on timer overflow
  TIMSK1 |= _BV(TOIE1);

  // Start the timer, no prescaler (CS1=1)
  TCCR1B |= _BV(CS10);
  */
  Serial.println("Leaving Buzzer_setup()");
}

void buzzer_on()
{
  is_buzzer_on = true;
}

void buzzer_off()
{
  is_buzzer_on = false;
}

// Interrupt Service Routine for TIMER1. This is used to switch between the
// buzzing and quiet periods when ON_CYCLES or OFF_CYCLES are reached.
/* TODO ESP32 
ISR (TIMER1_OVF_vect)
{
  interrupts();    // allow other interrupts (ie. modem)
  myalarm--;
  if (myalarm == 0) {
    buzzing = !buzzing;
    if (is_buzzer_on && buzzing) {
      switch(BUZZER_PIN) {
        case 9:
          // Non-inverting pin 9 (COM1A=2), p.135
          TCCR1A |= _BV(COM1A1);
          OCR1A = DUTY_CYCLE;
          break;
        case 10:
          // Non-inverting pin 10 (COM1B=2), p.135
          TCCR1A |= _BV(COM1B1);
          OCR1B = DUTY_CYCLE;
          break;
      }
      myalarm = ON_CYCLES;
    } else {
      switch(BUZZER_PIN) {
        // Disable PWM on pin 9/10
        case 9:  // TODO ESP32  TCCR1A &= ~_BV(COM1A1); break;
        case 10: //TODO ESP32 TCCR1A &= ~_BV(COM1B1); break;
      }
      pin_write(BUZZER_PIN, LOW);
      myalarm = OFF_CYCLES;
    }
  }
}
 TODO ESP32 */
#endif // #ifdef ESP32
