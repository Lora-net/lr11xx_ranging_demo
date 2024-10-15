/*!
 * @file      app_ranging_result_output.c
 *
 * @brief     Print the ranging results
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
#include <stdlib.h>

#include "app_ranging_result_output.h"
#include "smtc_hal_dbg_trace.h"
#include "lr11xx_radio_types_str.h"

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

void app_ranging_radio_settings_output( ranging_params_settings_t* settings )
{
    HAL_PERF_TEST_TRACE_PRINTF( "\r\n" );
    HAL_PERF_TEST_TRACE_PRINTF( "\r\n" );
    HAL_PERF_TEST_TRACE_PRINTF( "{\r\n" );
    HAL_PERF_TEST_TRACE_PRINTF( "\"SF\": \"%s\",\r\n", lr11xx_radio_lora_sf_to_str( settings->sf ) );
    HAL_PERF_TEST_TRACE_PRINTF( "\"BW\": \"%s\",\r\n", lr11xx_radio_lora_bw_to_str( settings->bw ) );
    HAL_PERF_TEST_TRACE_PRINTF( "\"Address\": \"0x%x\",\r\n", settings->rng_address );
}

void app_ranging_results_output( ranging_global_result_t* result )
{
    uint32_t freq;
    int      distance_int;
    int      distance_deci;

    HAL_PERF_TEST_TRACE_PRINTF( "\"LoRa RSSI\": \"%d dBm\",\r\n", result->rssi_value );
    HAL_PERF_TEST_TRACE_PRINTF( "\"LoRa SNR\": %d,\r\n", result->snr_value );

    if( result->cnt_packet_rx_ok == 0 )
    {
        HAL_PERF_TEST_TRACE_PRINTF( "\"RngResult\": {\r\n\t\"Num\": %d\r\n\t}\r\n},\r\n", result->cnt_packet_rx_ok );
    }
    else
    {
        HAL_PERF_TEST_TRACE_PRINTF( "\"RngResult\": {\r\n\t\"Num\": %d,\r\n\t\"Results\": [\r\n",
                                    result->cnt_packet_rx_ok );
        for( int i = 0; i < result->rng_result_index; i++ )
        {
            // Get the channel frequency according to index
            freq = get_ranging_hopping_channels( result->rng_freq_index[i] - 1 );

            HAL_PERF_TEST_TRACE_PRINTF( "\t\t{\"FreqIndex\": \"%d\", ", result->rng_freq_index[i] );
            HAL_PERF_TEST_TRACE_PRINTF( "\"Freq\": \"%d.%02d MHz\", ", freq / 1000000, ( freq / 10000 ) % 100 );
            HAL_PERF_TEST_TRACE_PRINTF( "\"RawDistance\": \"0x%08x\", ", result->raw_rng_results[i] );
            HAL_PERF_TEST_TRACE_PRINTF( "\"Distance\": \"%d m\", ", result->distance_rng_results[i] );

            if( i < result->rng_result_index - 1 )
            {
                HAL_PERF_TEST_TRACE_PRINTF( "\"RSSI\": \"%d dBm\" },\r\n", result->raw_rssi[i] );
            }
            else
            {
                HAL_PERF_TEST_TRACE_PRINTF( "\"RSSI\": \"%d dBm\" }\r\n", result->raw_rssi[i] );
            }
        }

        HAL_PERF_TEST_TRACE_PRINTF( "\t\t],\r\n" );
        distance_int  = ( int ) ( result->rng_distance );
        distance_deci = ( int ) ( ( result->rng_distance - distance_int ) * 100 );
        distance_deci = abs( distance_deci );
        HAL_PERF_TEST_TRACE_PRINTF( "\t\"DistanceRng\": \"%d.%01d m\",\r\n", distance_int, distance_deci );

        HAL_PERF_TEST_TRACE_PRINTF( "\t\"PER\": \"%d %%\"\r\n\t}\r\n", result->rng_per );
        HAL_PERF_TEST_TRACE_PRINTF( "},\r\n" );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
