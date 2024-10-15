/*!
 * @file      main_ranging_demo.c
 *
 * @brief     Ranging and hopping frequency example for LR1110 or LR1120 chip
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

#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "display_driver.h"
#include "app_ranging_hopping.h"
#include "main_ranging_demo.h"
#include "app_ranging_result_output.h"
#include "app_ranging_lcd_display.h"
#include "user_button.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LORA_RANGING_PROCESS_IRQ_MASK                                                                                  \
    ( LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED | LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE | LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID | \
      LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT | LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE |                        \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR )

#if( LORA_PREAMBLE_LENGTH != 12 )
#error "Please set the preamble length, "LORA_PREAMBLE_LENGTH", as 12, because it is related to the timing of ranging process."
#endif

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

static lr11xx_hal_context_t* context;

#if defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_MANAGER )
static const bool is_manager = true;
#elif defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_SUBORDINATE )
static const bool is_manager = false;
#else
#error Application must define RANGING_DEVICE_MODE
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    uart_init( );

    HAL_PERF_TEST_TRACE_PRINTF( "===== LR11xx ranging and frequency hopping example =====\r\n" );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_print_ranging_configuration( );

    app_radio_ranging_params_init( );

#if defined( RANGING_DISPLAY_FOR_TEST )
    user_button_init( );
    display_init( ( void* ) context );
    if( is_manager == true )
        display_string_printf( 0, 0, DISPLAY_WHITE, "Ranging Demo - Manager" );
    else
        display_string_printf( 0, 0, DISPLAY_WHITE, "Ranging Demo - Subordinate" );
    display_string_printf( 0, 16, DISPLAY_WHITE, "==========================" );
    display_string_printf( 0, 48, DISPLAY_WHITE, "  Ranging ..." );
    display_string_printf( 0, 192, DISPLAY_WHITE, "SF: %s", lr11xx_radio_lora_sf_to_str( LORA_SPREADING_FACTOR ) );
    display_string_printf( 0, 208, DISPLAY_WHITE, "BW: %s", lr11xx_radio_lora_bw_to_str( LORA_BANDWIDTH ) );
#endif

    if( is_manager == true )
    {
        HAL_PERF_TEST_TRACE_PRINTF( "===== Running in ranging manager mode =====\r\n" );
        apps_common_lr11xx_ranging_tx_leds( );
    }
    else
    {
        HAL_PERF_TEST_TRACE_PRINTF( "===== Running in ranging subordinate mode =====\r\n" );
        apps_common_lr11xx_ranging_rx_leds( );
    }

    if( ( LORA_BANDWIDTH == LR11XX_RADIO_LORA_BW_200 ) || ( LORA_BANDWIDTH == LR11XX_RADIO_LORA_BW_400 ) ||
        ( LORA_BANDWIDTH == LR11XX_RADIO_LORA_BW_800 ) )
    {
        HAL_PERF_TEST_TRACE_PRINTF( "\r\nERROR: It doesn't support BW200/400/800 for ranging on sub-G or 2.4G.\r\n" );
        while( 1 )
            ;
    }

    if( LORA_IQ != LR11XX_RADIO_LORA_IQ_STANDARD )
    {
        HAL_PERF_TEST_TRACE_PRINTF(
            "\r\nERROR: Please set IQ value, LORA_IQ, as standard value, because all the calibration values are based "
            "on IQ standard.\r\n" );
    }

    while( 1 )
    {
        app_running_status_t demo_status;

        do
        {
            app_radio_ranging_setup( ( void* ) context );
            apps_common_lr11xx_irq_process( ( void* ) context, LORA_RANGING_PROCESS_IRQ_MASK );
            demo_status = app_radio_ranging_run( ( void* ) context, is_manager );
        } while( demo_status == APP_STATUS_RUNNING );

        if( is_manager == true )
        {
            if( demo_status == APP_STATUS_TERMINATED )
            {
                ranging_params_settings_t* ranging_settings = app_ranging_get_radio_settings( );
                ranging_global_result_t*   ranging_results  = app_ranging_get_result( );

                app_ranging_radio_settings_output( ranging_settings );
                app_ranging_results_output( ranging_results );

#if defined( RANGING_DISPLAY_FOR_TEST )
                lcd_display_results( ranging_results );
#endif
            }
            apps_common_lr11xx_ranging_tx_leds( );
        }
        else
        {
            apps_common_lr11xx_ranging_rx_leds( );
        }

        if( demo_status != APP_STATUS_TERMINATED )
        {
            app_ranging_params_reset( );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

void on_tx_done( void )
{
    set_ranging_process_state( APP_RADIO_TX );
}

void on_rx_done( void )
{
    set_ranging_process_state( APP_RADIO_RX );
}

void on_rx_timeout( void )
{
    set_ranging_process_state( APP_RADIO_TIMEOUT );
    HAL_PERF_TEST_TRACE_PRINTF( "WARN: LoRa rx timeout\r\n" );
}

void on_header_error( void )
{
    set_ranging_process_state( APP_RADIO_ERROR );
    HAL_PERF_TEST_TRACE_PRINTF( "ERROR: LoRa header error\r\n" );
}

void on_rx_crc_error( void )
{
    set_ranging_process_state( APP_RADIO_ERROR );
    HAL_PERF_TEST_TRACE_PRINTF( "ERROR: LoRa CRC error\r\n" );
}

void on_rttof_request_discarded( void )
{
    set_ranging_process_state( APP_RADIO_ERROR );
    HAL_PERF_TEST_TRACE_PRINTF( "WARN: Ranging request discarded\r\n" );
}

void on_rttof_response_done( void )
{
    set_ranging_process_state( APP_RADIO_RANGING_DONE );
}

void on_rttof_exchange_valid( void )
{
    set_ranging_process_state( APP_RADIO_RANGING_DONE );
}

void on_rttof_timeout( void )
{
    set_ranging_process_state( APP_RADIO_RANGING_TIMEOUT );
    HAL_PERF_TEST_TRACE_PRINTF( "WARN: Ranging timeout\r\n" );
}

/* --- EOF ------------------------------------------------------------------ */
