/*!
 * @file      user_button.c
 *
 * @brief     Handle the user (blue) button
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>

#include "user_button.h"
#include "smtc_shield_pinout.h"
#include "smtc_hal_mcu_gpio.h"
#include "stm32l4xx_ll_gpio.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_mcu_gpio_stm32l4.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static struct
{
    smtc_hal_mcu_gpio_cfg_t       cfg;
    smtc_hal_mcu_gpio_input_cfg_t cfg_input;
    smtc_hal_mcu_gpio_inst_t      inst;
} user_button;

const struct smtc_hal_mcu_gpio_cfg_s blue_button_pin = { .port = GPIOC, .pin = LL_GPIO_PIN_13 };

static bool user_button_state = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief User callback for button EXTI
 *
 * @param context Define by the user at the init
 */
static void user_button_callback( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void user_button_init( void )
{
    user_button.cfg                 = ( const smtc_hal_mcu_gpio_cfg_t ) &blue_button_pin;
    user_button.cfg_input.pull_mode = SMTC_HAL_MCU_GPIO_PULL_MODE_NONE;
    user_button.cfg_input.irq_mode  = SMTC_HAL_MCU_GPIO_IRQ_MODE_FALLING;
    user_button.cfg_input.callback  = user_button_callback;
    user_button.cfg_input.context   = NULL;

    smtc_hal_mcu_gpio_init_input( user_button.cfg, &( user_button.cfg_input ), &( user_button.inst ) );
    smtc_hal_mcu_gpio_enable_irq( user_button.inst );
}

bool get_user_button_state( void )
{
    return user_button_state;
}

void set_user_button_state( bool state )
{
    user_button_state = state;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void user_button_callback( void* context )
{
    HAL_PERF_TEST_TRACE_PRINTF( "\r\nUser button pushed\r\n" );

    ( void ) context;  // Not used - avoid warning
    // Here rely on hardware for debounce.
    user_button_state = true;
}

/* --- EOF ------------------------------------------------------------------ */
