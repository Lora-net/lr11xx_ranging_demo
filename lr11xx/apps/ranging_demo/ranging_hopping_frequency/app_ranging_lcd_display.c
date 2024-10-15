/*!
 * @file      app_ranging_lcd_display.c
 *
 * @brief     Display some useful messages by using a LCD
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

#include "app_ranging_lcd_display.h"
#include "display_driver.h"
#include "user_button.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lcd_display_results( ranging_global_result_t* result )
{
    static uint16_t ranging_index         = 0;  // To record the times that ranging process has beed done.
    static float    base_ranging_distance = 0;  // To save a base distance for relative mode.
    static float    last_ranging_distance = 0;  // To save the last distance.
    static bool     relative_mode_flag    = false;

    ranging_index++;

    if( true == get_user_button_state( ) )  // Button has been pressed
    {
        set_user_button_state( false );  // Clear the pressed state
        base_ranging_distance = last_ranging_distance;
        relative_mode_flag    = true;
    }

    display_section_fill( 56, 32, 240, 96, DISPLAY_BACKGROUND );  // Clear the section that are showing values.
    if( result->cnt_packet_rx_ok != 0 )                           // Get ranging result.
    {
        if( relative_mode_flag != true )  // Absolute ranging mode
        {
            display_string_printf( 0, 32, DISPLAY_GREEN, "Range:  %0.1f m", result->rng_distance );
        }
        else  // Relative ranging mode
        {
            display_string_printf( 0, 32, DISPLAY_GREEN, "Range:  %0.1f m *",
                                   result->rng_distance - base_ranging_distance );
        }
        display_string_printf( 0, 80, DISPLAY_WHITE, "PER:    %d %%", result->rng_per );
    }
    else  // None of ranging process succeeded.
    {
        display_string_printf( 0, 32, DISPLAY_GREEN, "Range:  " );
        display_string_printf( 64, 32, DISPLAY_RED, "All channels failed" );

        display_string_printf( 0, 80, DISPLAY_WHITE, "PER:    " );
        display_string_printf( 64, 80, DISPLAY_RED, "100 %%" );
    }

    display_string_printf( 0, 48, DISPLAY_WHITE, "RSSI:   %d dBm", result->rssi_value );
    display_string_printf( 0, 64, DISPLAY_WHITE, "SNR:    %d dB", result->snr_value );
    display_section_fill( 168, 96, 240, 112, DISPLAY_BACKGROUND );
    display_string_printf( 0, 96, DISPLAY_WHITE, "Ranging transaction: %d", ranging_index );

    if( relative_mode_flag != true )  // Absolute ranging mode
    {
        display_section_fill( 0, 240, 240, 256, DISPLAY_BACKGROUND );  // Clear "* Relative Range" text.
        display_string_printf( 0, 272, DISPLAY_WHITE, "BLUE button: relative range   " );
        display_string_printf( 0, 288, DISPLAY_WHITE, "BLACK button: reset           " );
    }
    else  // Relative ranging mode
    {
        display_string_printf( 0, 240, DISPLAY_BLUE, " * Relative Range" );
        display_string_printf( 0, 272, DISPLAY_WHITE, "BLUE button: update relative  " );
        display_string_printf( 0, 288, DISPLAY_WHITE, "BLACK button: absolute range  " );
    }
    last_ranging_distance = result->rng_distance;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
