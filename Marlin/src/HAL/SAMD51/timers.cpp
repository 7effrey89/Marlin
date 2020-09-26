/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef __SAMD51__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../inc/MarlinConfig.h"
#include "ServoTimers.h" // for SERVO_TC

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 8

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

const tTimerConfig TimerConfig[NUM_HARDWARE_TIMERS+1] = {
  { {.pTc=TC0},  TC0_IRQn, TC_PRIORITY(0) },  // 0 - stepper (assigned priority 2)
  { {.pTc=TC1},  TC1_IRQn, TC_PRIORITY(1) },  // 1 - stepper (needed by 32 bit timers)
  { {.pTc=TC2},  TC2_IRQn, 5              },  // 2 - tone (reserved by framework and fixed assigned priority 5)
  { {.pTc=TC3},  TC3_IRQn, TC_PRIORITY(3) },  // 3 - servo (assigned priority 1)
  { {.pTc=TC4},  TC4_IRQn, TC_PRIORITY(4) },  // 4 - software serial (no interrupts used)
  { {.pTc=TC5},  TC5_IRQn, TC_PRIORITY(5) },
  { {.pTc=TC6},  TC6_IRQn, TC_PRIORITY(6) },
  { {.pTc=TC7},  TC7_IRQn, TC_PRIORITY(7) },
  { {.pRtc=RTC}, RTC_IRQn, TC_PRIORITY(8) }   // 8 - temperature (assigned priority 6)
};

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

FORCE_INLINE void Disable_Irq(IRQn_Type irq) {
  NVIC_DisableIRQ(irq);

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  Tc * const tc = TimerConfig[timer_num].pTimer;
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;

  // Disable interrupt, just in case it was already enabled
  Disable_Irq(irq);

  // Disable timer interrupt
  tc->COUNT32.INTENCLR.reg = TC_INTENCLR_OVF; // disable overflow interrupt

  // TCn clock setup
  const uint8_t clockID = GCLK_CLKCTRL_IDs[TCC_INST_NUM + timer_num];
  GCLK->PCHCTRL[clockID].bit.CHEN = false;
  SYNC(GCLK->PCHCTRL[clockID].bit.CHEN);
  GCLK->PCHCTRL[clockID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;   // 120MHz startup code programmed
  SYNC(!GCLK->PCHCTRL[clockID].bit.CHEN);

  // Stop timer, just in case, to be able to reconfigure it
  tc->COUNT32.CTRLA.bit.ENABLE = false;
  SYNC(tc->COUNT32.SYNCBUSY.bit.ENABLE);

  // Reset timer
  tc->COUNT32.CTRLA.bit.SWRST = true;
  SYNC(tc->COUNT32.SYNCBUSY.bit.SWRST);

  NVIC_SetPriority(irq, TimerConfig[timer_num].priority);

    // Wave mode, reset counter on compare match
    tc->COUNT32.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    tc->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1;
    tc->COUNT32.CTRLBCLR.reg = TC_CTRLBCLR_DIR;
    SYNC(tc->COUNT32.SYNCBUSY.bit.CTRLB);

    // Set compare value
    tc->COUNT32.CC[0].reg = (HAL_TIMER_RATE) / frequency;
    tc->COUNT32.COUNT.reg = 0;

  // And start timer
  tc->COUNT32.CTRLA.bit.ENABLE = true;
  SYNC(tc->COUNT32.SYNCBUSY.bit.ENABLE);

  // Enable interrupt on RC compare
  tc->COUNT32.INTENSET.reg = TC_INTENCLR_OVF; // enable overflow interrupt

  // Finally, enable IRQ
  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  NVIC_EnableIRQ(irq);
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  Disable_Irq(irq);
}

// missing from CMSIS: Check if interrupt is enabled or not
static bool NVIC_GetEnabledIRQ(IRQn_Type IRQn) {
  return (NVIC->ISER[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))) != 0;
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  return NVIC_GetEnabledIRQ(irq);
}

#endif // __SAMD51__
