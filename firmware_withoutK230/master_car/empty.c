/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define PWM_PERIOD_COUNTS (1000U)
#define BREATH_STEPS (250U)
#define BREATH_STEP_DELAY_CYCLES (CPUCLK_FREQ / 100U)

static void setLedDuty(uint32_t dutyCounts)
{
    uint32_t compareValue;

    if (dutyCounts >= PWM_PERIOD_COUNTS) {
        compareValue = 0U;
    } else {
        compareValue = PWM_PERIOD_COUNTS - dutyCounts - 1U;
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_1_INST, compareValue, DL_TIMER_CC_1_INDEX);
}

int main(void)
{
    SYSCFG_DL_init();
    DL_TimerG_startCounter(PWM_1_INST);

    while (1) {
        for (uint32_t step = 0U; step < BREATH_STEPS; step++) {
            setLedDuty((PWM_PERIOD_COUNTS * step) / (BREATH_STEPS - 1U));
            delay_cycles(BREATH_STEP_DELAY_CYCLES);
        }

        for (uint32_t step = BREATH_STEPS; step > 0U; step--) {
            setLedDuty((PWM_PERIOD_COUNTS * (step - 1U)) / (BREATH_STEPS - 1U));
            delay_cycles(BREATH_STEP_DELAY_CYCLES);
        }
    }
}
