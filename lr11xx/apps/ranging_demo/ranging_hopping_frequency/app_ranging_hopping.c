/*!
 * @file      app_ranging_hopping.c
 *
 * @brief     Ranging and frequency hopping for LR1110 or LR1120 chip
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
#include <string.h>

#include "main_ranging_demo.h"
#include "app_ranging_hopping.h"
#include "apps_common.h"
#include "app_ranging_timer.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_rttof.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_mcu_timer.h"
#include "smtc_hal_mcu_timer_stm32l4.h"
#include "smtc_shield_lr11xx.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*!
 * @brief Define preset ranging addresses
 */
#define RANGING_ADDR_1 0x32101222

/*!
 * @brief Total symbol numbers of a ranging process
 */
#define RANGING_ALL_SYMBOL 64.25
#define RANGING_DONE_PROCESSING_TIME 5  // ms

/*!
 * @brief Ranging related IRQs enabled on the Ranging manager device
 */
#define RANGING_MANAGER_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID | LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT )

/*!
 * @brief Ranging IRQs enabled on the Ranging subordinate device
 */
#define RANGING_SUBORDINATE_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED | LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE )

#define LORA_IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_HEADER_ERROR | \
      LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR )

/*!
 * @brief Number of ranging address bytes the subordinate has to check upon reception of a ranging request
 */
#define RANGING_SUBORDINATE_CHECK_LENGTH_BYTES ( 4 )

/*!
 * @brief Length in byte of the ranging result
 */
#define LR11XX_RANGING_RESULT_LENGTH ( 4 )

/*!
 * @brief Number of symbols in ranging response
 */
#ifndef RANGING_RESPONSE_SYMBOLS_COUNT
#define RANGING_RESPONSE_SYMBOLS_COUNT UINT8_C( 15 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Reference frequency hopping tables.
 *
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    863750000, 865100000, 864800000, 868400000, 865250000, 867500000, 865550000, 867650000, 866150000, 864050000,
    867800000, 863300000, 863450000, 867950000, 868550000, 868850000, 867200000, 867050000, 864650000, 863900000,
    864500000, 866450000, 865400000, 868700000, 863150000, 866750000, 866300000, 864950000, 864350000, 866000000,
    866900000, 868250000, 865850000, 865700000, 867350000, 868100000, 863600000, 866600000, 864200000,
  };
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    490810000, 508940000, 496690000, 507470000, 504040000, 508450000, 505020000, 497670000, 497180000, 500610000,
    494240000, 493260000, 495710000, 491300000, 504530000, 501100000, 502080000, 501590000, 499140000, 494730000,
    506980000, 492280000, 509430000, 495220000, 492770000, 507960000, 493750000, 499630000, 496200000, 498160000,
    505510000, 500120000, 503060000, 506000000, 506490000, 498650000, 491790000, 503550000, 502570000,
};
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    907850000, 902650000, 914350000, 906550000, 905900000, 924750000, 926700000, 918250000, 921500000, 909150000,
    907200000, 924100000, 903950000, 910450000, 917600000, 919550000, 923450000, 925400000, 909800000, 915000000,
    912400000, 904600000, 908500000, 911100000, 911750000, 916300000, 918900000, 905250000, 913700000, 927350000,
    926050000, 916950000, 913050000, 903300000, 920200000, 922800000, 915650000, 922150000, 920850000,
};
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    2450000000, 2402000000, 2476000000, 2436000000, 2430000000, 2468000000, 2458000000, 2416000000,
    2424000000, 2478000000, 2456000000, 2448000000, 2462000000, 2472000000, 2432000000, 2446000000,
    2422000000, 2442000000, 2460000000, 2474000000, 2414000000, 2464000000, 2454000000, 2444000000,
    2404000000, 2434000000, 2410000000, 2408000000, 2440000000, 2452000000, 2480000000, 2426000000,
    2428000000, 2466000000, 2418000000, 2412000000, 2406000000, 2470000000, 2438000000,
};
 */
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    907850000, 902650000, 914350000, 906550000, 905900000, 924750000, 926700000, 918250000, 921500000, 909150000,
    907200000, 924100000, 903950000, 910450000, 917600000, 919550000, 923450000, 925400000, 909800000, 915000000,
    912400000, 904600000, 908500000, 911100000, 911750000, 916300000, 918900000, 905250000, 913700000, 927350000,
    926050000, 916950000, 913050000, 903300000, 920200000, 922800000, 915650000, 922150000, 920850000,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Ranging result comprising raw distance, distance meter and RSSI.
 */
typedef struct ranging_result_s
{
    uint32_t raw_distance;  // Raw distance value(hexadecimal)
    int32_t  distance_m;    // Distance obtained from ranging [m]
    int8_t   rssi;          // RSSI corresponding to manager-side response reception [dBm]
} ranging_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Flag to indicate if the ranging demo is already running
 */
static bool ranging_running_flag = false;

/*!
 * @brief Set up a tmer for global ranging exchanging process
 */
static bool ranging_exch_timer_launch = false;

static lr11xx_radio_mod_params_lora_t mod_params;
static lr11xx_radio_pkt_params_lora_t pkt_params;

static ranging_params_settings_t ranging_settings = { 0 };
static ranging_global_result_t   ranging_results  = { 0 };

/*!
 * @brief Radio payload buffer
 */
static uint8_t radio_pl_buffer[PAYLOAD_LENGTH];

/*!
 * @brief Flag holding the current internal state of the ranging application
 */
static uint8_t ranging_internal_state;
static uint8_t ranging_next_start = false;

/*!
 * @brief Set up a mcu timer by using LPTIM1
 */
struct smtc_hal_mcu_timer_cfg_s ranging_mcu_timer_cfg;
smtc_hal_mcu_timer_cfg_app_t    ranging_mcu_timer_callback;
smtc_hal_mcu_timer_inst_t       ranging_mcu_timer_inst = { 0 };

/*!
 * @brief Current channel that is used for ranging
 */
static uint8_t current_channel;

/*!
 * @brief Count RANGING_HOPPING_CHANNELS_MAX that have been used
 */
static uint16_t measured_channels;

static uint32_t ranging_tx_start_ms;
static uint32_t ranging_tx_count_ms;

static uint32_t             app_timer_tick_timeout_ms = 0;
static app_running_status_t demo_status;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Callback of mcu timer
 */
static void ranging_send_next_packet( void );

/*!
 * @brief Read out and process a single ranging result from the ranging manager.
 *
 * @param [in] context  Pointer to the radio context
 * @param [in]  ranging_bw   Bandwidth used during ranging
 * @param [out] result     Ranging result
 *
 * @returns lr11xx_status_t Operation result
 */
static lr11xx_status_t get_ranging_result( const void* context, lr11xx_radio_lora_bw_t ranging_bw,
                                           ranging_result_t* result );

/*!
 * @brief  Handle the final ranging results.
 */
static void ranging_handle_distance_result( void );

/*!
 * @brief Calculate the single symbol time according to the given SF and BW
 *
 * @param [in]  bw   Bandwidth used during ranging
 * @param [in]  sf   Spreading factor used during ranging
 *
 * @returns Single symbol time, unit: ms
 */
static float get_single_symbol_time_ms( lr11xx_radio_lora_bw_t bw, lr11xx_radio_lora_sf_t sf );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void app_radio_ranging_params_init( void )
{
    ranging_settings.frequency = RF_FREQ_IN_HZ;  // Set frequency for LoRa mode on the initialization process
    ranging_settings.tx_power  = TX_OUTPUT_POWER_DBM;
    ranging_settings.sf        = LORA_SPREADING_FACTOR;  // just be used for output in the logs
    ranging_settings.bw        = LORA_BANDWIDTH;         // just be used for output in the logs

    ranging_settings.rng_req_count = RANGING_HOPPING_CHANNELS_MAX;
    ranging_settings.rng_address   = RANGING_ADDR_1;

    pkt_params.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    pkt_params.header_type          = LORA_PKT_LEN_MODE;
    pkt_params.iq                   = LORA_IQ;
    pkt_params.crc                  = LORA_CRC;

    mod_params.sf   = LORA_SPREADING_FACTOR;
    mod_params.bw   = LORA_BANDWIDTH;
    mod_params.cr   = LORA_CODING_RATE;
    mod_params.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );

    memset( &radio_pl_buffer, 0x00u, sizeof( radio_pl_buffer ) );

    ranging_results.cnt_packet_rx_ok = 0u;

    /* Initialize MCU timer  */
    ranging_mcu_timer_cfg.tim              = ( LPTIM_TypeDef* ) LPTIM1;
    ranging_mcu_timer_callback.expiry_func = ( void* ) ranging_send_next_packet;
    smtc_hal_mcu_timer_init( &ranging_mcu_timer_cfg, &ranging_mcu_timer_callback, &ranging_mcu_timer_inst );

    /* Initialize system tick handler */
    app_timer_tick_init( );

    ranging_running_flag = false;
}

void app_radio_ranging_setup( const void* context )
{
    demo_status = APP_STATUS_RUNNING;
    if( ranging_running_flag == false )
    {
        ranging_running_flag        = true;
        ranging_exch_timer_launch   = false;
        ranging_settings.rng_status = RANGING_STATUS_INIT;

        apps_common_lr11xx_radio_ranging_init( context, ranging_settings.frequency, ranging_settings.tx_power );

        ranging_settings.rng_req_delay =
            ( uint16_t )( get_single_symbol_time_ms( mod_params.bw, mod_params.sf ) * RANGING_ALL_SYMBOL ) +
            RANGING_DONE_PROCESSING_TIME;

        ranging_results.rng_distance = 0.0;
        ranging_internal_state       = APP_RADIO_RANGING_CONFIG;
    }

    /* Global ranging exchange timeout */
    if( ranging_exch_timer_launch == true )
    {
        if( app_timer_tick_has_expired( &app_timer_tick_timeout_ms ) == true )
        {
            ranging_exch_timer_launch = false;
            ranging_running_flag      = false;
            demo_status               = APP_STATUS_ERROR;
            smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
            ranging_internal_state = APP_RADIO_RANGING_CONFIG;
            lr11xx_system_set_standby( context, LR11XX_SYSTEM_STANDBY_CFG_RC );
            ranging_settings.rng_status = RANGING_STATUS_TIMEOUT;
            HAL_PERF_TEST_TRACE_PRINTF( "ERROR: Global Ranging Timeout\r\n" );
        }
    }
}

app_running_status_t app_radio_ranging_run( const void* context, const bool is_manager )
{
    switch( ranging_internal_state )
    {
    case APP_RADIO_RANGING_CONFIG:
        ranging_settings.rng_status  = RANGING_STATUS_INIT;
        ranging_settings.packet_type = LR11XX_RADIO_PKT_TYPE_LORA;
        pkt_params.pld_len_in_bytes  = PAYLOAD_LENGTH;

        lr11xx_radio_set_pkt_type( context, ranging_settings.packet_type );
        lr11xx_radio_set_rf_freq( context, ranging_settings.frequency );
        lr11xx_radio_set_lora_mod_params( context, &mod_params );
        lr11xx_radio_set_lora_pkt_params( context, &pkt_params );
        lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD );

        lr11xx_system_set_dio_irq_params( context, LORA_IRQ_MASK, 0 );
        lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );

        if( is_manager )  // manager
        {
            ranging_results.cnt_packet_rx_ok = 0;
            measured_channels                = 0u;
            current_channel                  = 0u;
            radio_pl_buffer[0]               = ( ranging_settings.rng_address >> 24u ) & 0xFFu;
            radio_pl_buffer[1]               = ( ranging_settings.rng_address >> 16u ) & 0xFFu;
            radio_pl_buffer[2]               = ( ranging_settings.rng_address >> 8u ) & 0xFFu;
            radio_pl_buffer[3]               = ( ranging_settings.rng_address & 0xFFu );
            radio_pl_buffer[4]               = current_channel;                 // set the first channel to use
            radio_pl_buffer[5]               = ranging_settings.rng_req_count;  // set the number of frequency hopping
            radio_pl_buffer[6]               = 0;

            // send lora packet
            lr11xx_regmem_write_buffer8( context, radio_pl_buffer, PAYLOAD_LENGTH );
            lr11xx_radio_set_tx( context, 0 );
        }
        else  // subordinate
        {
            lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, 0xFFFFFF );
        }

        ranging_internal_state = APP_RADIO_IDLE;
        break;

    case APP_RADIO_RANGING_START:
        if( ranging_next_start == true )
        {
            ranging_next_start = false;
            smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );  // stop the autoreload timer
            measured_channels++;
            if( measured_channels <= ranging_settings.rng_req_count )
            {
                lr11xx_radio_set_rf_freq( context, ranging_hopping_channels_array[current_channel] );
                current_channel++;
                if( current_channel >= RANGING_HOPPING_CHANNELS_MAX )
                {
                    current_channel -= RANGING_HOPPING_CHANNELS_MAX;
                }
                ranging_internal_state = APP_RADIO_IDLE;
                ranging_tx_start_ms = app_timer_tick_get_ms( );
                apps_common_lr11xx_ranging_toggle_tx_rx_leds( );

                if( is_manager )  // manager
                {
                    lr11xx_radio_set_tx( context, 0 );
                }
                else  // subordinate
                {
                    lr11xx_radio_set_rx( context,
                                         ranging_settings.rng_req_delay - ( RANGING_DONE_PROCESSING_TIME / 2 ) );
                }
            }
            else
            {
                smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );  // stop the autoreload timer
                demo_status            = APP_STATUS_TERMINATED;
                ranging_running_flag   = false;
                ranging_internal_state = APP_RADIO_RANGING_CONFIG;
                lr11xx_system_set_standby( context, LR11XX_SYSTEM_STANDBY_CFG_RC );
                ranging_settings.rng_status = RANGING_STATUS_VALID;
            }
        }
        break;

    case APP_RADIO_RANGING_DONE:
        smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );

        if( is_manager )  // manager
        {
            smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, RANGING_DONE_PROCESSING_TIME );

            // get ranging result
            ranging_result_t result = { 0 };
            get_ranging_result( context, mod_params.bw, &result );

            ranging_results.raw_rssi[ranging_results.rng_result_index]             = result.rssi;
            ranging_results.raw_rng_results[ranging_results.rng_result_index]      = result.raw_distance;
            ranging_results.distance_rng_results[ranging_results.rng_result_index] = result.distance_m;
            if( current_channel == 0 )
            {
                // save the last result's index
                ranging_results.rng_freq_index[ranging_results.rng_result_index] = RANGING_HOPPING_CHANNELS_MAX;
            }
            else
            {
                ranging_results.rng_freq_index[ranging_results.rng_result_index] = current_channel;
            }

            ranging_results.rng_result_index++;
        }
        else  // subordinate
        {
            // next ranging in RANGING_DONE_PROCESSING_TIME - 1 ms in order to start before the manager
            smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, RANGING_DONE_PROCESSING_TIME - 1 );
        }

        ranging_results.cnt_packet_rx_ok++;
        ranging_internal_state = APP_RADIO_RANGING_START;
        break;

    case APP_RADIO_RANGING_TIMEOUT:
        ranging_tx_count_ms = app_timer_tick_get_ms( ) - ranging_tx_start_ms;
        if( ranging_tx_count_ms < ranging_settings.rng_req_delay )
        {
            smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
            smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, ranging_settings.rng_req_delay - ranging_tx_count_ms );
        }
        else
        {
            smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );

            if( is_manager )  // manager
            {
                smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, RANGING_DONE_PROCESSING_TIME );
            }
            else  // subordinate
            {
                // next ranging in RANGING_DONE_PROCESSING_TIME - 1 ms in order to start before the manager
                smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, RANGING_DONE_PROCESSING_TIME - 1 );
            }
        }
        ranging_internal_state = APP_RADIO_RANGING_START;
        break;

    case APP_RADIO_RX:
        lr11xx_system_set_standby( context, LR11XX_SYSTEM_STANDBY_CFG_RC );
        if( ranging_settings.rng_status == RANGING_STATUS_INIT )
        {
            lr11xx_radio_rx_buffer_status_t rx_buffer_status;
            lr11xx_radio_pkt_status_lora_t  pkt_status_lora;

            lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
            lr11xx_regmem_read_buffer8( context, radio_pl_buffer, rx_buffer_status.buffer_start_pointer,
                                        rx_buffer_status.pld_len_in_bytes );

            if( ( rx_buffer_status.pld_len_in_bytes > 0 ) &&
                ( radio_pl_buffer[0] == ( ( ranging_settings.rng_address >> 24 ) & 0xFF ) ) &&
                ( radio_pl_buffer[1] == ( ( ranging_settings.rng_address >> 16 ) & 0xFF ) ) &&
                ( radio_pl_buffer[2] == ( ( ranging_settings.rng_address >> 8 ) & 0xFF ) ) &&
                ( radio_pl_buffer[3] == ( ranging_settings.rng_address & 0xFF ) ) )
            {
                lr11xx_radio_get_lora_pkt_status( context, &pkt_status_lora );
                ranging_results.rssi_value = pkt_status_lora.rssi_pkt_in_dbm;
                ranging_results.snr_value  = pkt_status_lora.snr_pkt_in_db;

                if( is_manager )  // manager
                {
                    ranging_settings.rng_status      = RANGING_STATUS_PROCESS;
                    ranging_results.slave_rssi_value = ( int8_t ) radio_pl_buffer[6];

                    ranging_settings.packet_type = LR11XX_RADIO_PKT_TYPE_RTTOF;
                    pkt_params.pld_len_in_bytes  = 10u;

                    lr11xx_radio_set_pkt_type( context, ranging_settings.packet_type );
                    lr11xx_radio_set_lora_mod_params( context, &mod_params );
                    lr11xx_radio_set_lora_pkt_params( context, &pkt_params );

                    uint32_t rttof_rx_tx_delay = 0u;
                    if( apps_common_lr11xx_get_ranging_rx_tx_delay( ranging_settings.frequency, mod_params.bw,
                                                                    mod_params.sf, &rttof_rx_tx_delay ) )
                    {
                        lr11xx_rttof_set_rx_tx_delay_indicator( context, rttof_rx_tx_delay );
                    }
                    else
                    {
                        HAL_PERF_TEST_TRACE_PRINTF( "ERROR: Failed to get RTToF delay indicator\r\n" );
                    }

                    lr11xx_rttof_set_parameters( context, RANGING_RESPONSE_SYMBOLS_COUNT );
                    lr11xx_rttof_set_request_address( context, ranging_settings.rng_address );

                    lr11xx_system_set_dio_irq_params( context, RANGING_MANAGER_IRQ_MASK, 0 );
                    lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );

                    measured_channels                = 0;
                    ranging_results.rng_result_index = 0;
                    ranging_internal_state           = APP_RADIO_RANGING_START;

                    // Launch global ranging timer
                    // Add 1, due to the 'ranging_mcu_timer_inst' timer
                    // +1000ms, more than real using time
                    ranging_settings.rng_exch_timeout =
                        ranging_settings.rng_req_delay * ( ranging_settings.rng_req_count + 1 ) + 1000;
                    app_timer_tick_set_ms( &app_timer_tick_timeout_ms, ranging_settings.rng_exch_timeout );
                    ranging_exch_timer_launch = true;
                    // Schedule next ranging
                    smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
                    smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, ranging_settings.rng_req_delay );
                }
                else  // subordinate
                {
                    current_channel                = radio_pl_buffer[4];
                    ranging_settings.rng_req_count = radio_pl_buffer[5];
                    radio_pl_buffer[6]             = ( uint8_t ) ranging_results.rssi_value;

                    lr11xx_regmem_write_buffer8( context, radio_pl_buffer, PAYLOAD_LENGTH );
                    lr11xx_radio_set_tx( context, 0 );
                    ranging_internal_state = APP_RADIO_IDLE;
                }
            }
            else
            {
                ranging_internal_state = APP_RADIO_RANGING_CONFIG;
            }
        }
        else
        {
            ranging_internal_state = APP_RADIO_RANGING_CONFIG;
        }
        break;

    case APP_RADIO_TX:
        if( ranging_settings.rng_status == RANGING_STATUS_INIT )
        {
            if( is_manager )  // manager
            {
                lr11xx_radio_set_rx( context,
                                     lr11xx_radio_get_lora_time_on_air_in_ms( &pkt_params, &mod_params ) + 10 );

                ranging_internal_state = APP_RADIO_IDLE;
            }
            else  // subordinate
            {
                ranging_settings.rng_status  = RANGING_STATUS_PROCESS;
                ranging_settings.packet_type = LR11XX_RADIO_PKT_TYPE_RTTOF;
                pkt_params.pld_len_in_bytes  = 10u;

                lr11xx_radio_set_pkt_type( context, ranging_settings.packet_type );
                lr11xx_radio_set_lora_mod_params( context, &mod_params );
                lr11xx_radio_set_lora_pkt_params( context, &pkt_params );

                uint32_t rttof_rx_tx_delay = 0u;
                if( apps_common_lr11xx_get_ranging_rx_tx_delay( ranging_settings.frequency, mod_params.bw,
                                                                mod_params.sf, &rttof_rx_tx_delay ) )
                {
                    lr11xx_rttof_set_rx_tx_delay_indicator( context, rttof_rx_tx_delay );
                }
                else
                {
                    HAL_PERF_TEST_TRACE_PRINTF( "ERROR: Failed to get RTToF delay indicator\r\n" );
                }

                lr11xx_rttof_set_parameters( context, RANGING_RESPONSE_SYMBOLS_COUNT );
                lr11xx_rttof_set_address( context, ranging_settings.rng_address,
                                          RANGING_SUBORDINATE_CHECK_LENGTH_BYTES );

                lr11xx_system_set_dio_irq_params( context, RANGING_SUBORDINATE_IRQ_MASK, 0 );
                lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );

                ranging_results.cnt_packet_rx_ok       = 0;
                measured_channels                      = 0;
                ranging_results.cnt_packet_rx_ko_slave = 0;

                // Launch global ranging timer
                // Add 1, due to the 'ranging_mcu_timer_inst' timer
                // +1000ms, more than real using time
                ranging_settings.rng_exch_timeout =
                    ranging_settings.rng_req_delay * ( ranging_settings.rng_req_count + 1 ) + 1000;
                app_timer_tick_set_ms( &app_timer_tick_timeout_ms, ranging_settings.rng_exch_timeout );
                ranging_exch_timer_launch = true;
                // Schedule next ranging
                smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
                smtc_hal_mcu_timer_start( ranging_mcu_timer_inst, ranging_settings.rng_req_delay );
                ranging_internal_state = APP_RADIO_RANGING_START;
            }
        }
        else
        {
            ranging_internal_state = APP_RADIO_RANGING_CONFIG;
        }
        break;

    case APP_RADIO_TIMEOUT:
        ranging_settings.rng_status = RANGING_STATUS_TIMEOUT;
        ranging_internal_state      = APP_RADIO_RANGING_CONFIG;
        break;

    case APP_RADIO_ERROR:
        ranging_internal_state = APP_RADIO_RANGING_CONFIG;
        break;

    case APP_RADIO_TX_TIMEOUT:
        if( ranging_settings.rng_status != RANGING_STATUS_PROCESS )
        {
            ranging_internal_state = APP_RADIO_RANGING_CONFIG;
        }
        else
        {
            ranging_internal_state = APP_RADIO_RANGING_START;
        }
        break;

    case APP_RADIO_IDLE:
        if( is_manager )  // manager
        {
            if( ( ranging_settings.rng_status == RANGING_STATUS_PROCESS ) && ( ranging_next_start == true ) )
            {
                ranging_results.cnt_packet_rx_ko_slave++;
                ranging_internal_state = APP_RADIO_RANGING_START;
            }
        }
        else  // subordinate
        {
            if( ranging_results.cnt_packet_rx_ko_slave > RANGING_HOPPING_CHANNELS_MAX )
            {
                ranging_results.cnt_packet_rx_ko_slave = 0;
                ranging_internal_state                 = APP_RADIO_RANGING_CONFIG;

                smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
            }
        }
        break;

    default:
        ranging_internal_state = APP_RADIO_RANGING_CONFIG;
        smtc_hal_mcu_timer_stop( ranging_mcu_timer_inst );
        break;
    }
    return demo_status;
}

ranging_global_result_t* app_ranging_get_result( void )
{
    ranging_results.rng_per =
        100 - ( ( uint8_t )( ( ( float ) ranging_results.cnt_packet_rx_ok / ( float ) ranging_settings.rng_req_count ) *
                             100 ) );
    ranging_handle_distance_result( );
    return &ranging_results;
}

ranging_params_settings_t* app_ranging_get_radio_settings( void )
{
    return &ranging_settings;
}

void app_ranging_params_reset( void )
{
    ranging_internal_state                 = APP_RADIO_IDLE;
    demo_status                            = APP_STATUS_NOT_CONFIGURED;
    ranging_results.cnt_packet_rx_ko_slave = 0;
    ranging_results.cnt_packet_rx_ok       = 0u;
    ranging_results.rng_distance           = 0;
    ranging_results.rssi_value             = 0;
    ranging_running_flag                   = false;
    measured_channels                      = 0;
    current_channel                        = 0;
}

uint32_t get_ranging_hopping_channels( uint8_t index )
{
    if( index >= RANGING_HOPPING_CHANNELS_MAX )  // overflow
    {
        return 0;
    }
    return ranging_hopping_channels_array[index];
}

void set_ranging_process_state( uint8_t state )
{
    ranging_internal_state = state;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void ranging_send_next_packet( void )
{
    ranging_next_start = true;
    if( ranging_settings.rng_status == RANGING_STATUS_PROCESS )
    {
        ranging_results.cnt_packet_rx_ko_slave++;
    }
}

static lr11xx_status_t get_ranging_result( const void* context, lr11xx_radio_lora_bw_t ranging_bw,
                                           ranging_result_t* result )
{
    lr11xx_status_t rc;
    uint8_t         buf[LR11XX_RANGING_RESULT_LENGTH];

    // Get raw distance value
    rc = lr11xx_rttof_get_raw_result( context, LR11XX_RTTOF_RESULT_TYPE_RAW, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->raw_distance = ( ( uint32_t ) buf[3] << 0 ) + ( ( uint32_t ) buf[2] << 8 ) + ( ( uint32_t ) buf[1] << 16 ) +
                           ( ( uint32_t ) buf[0] << 24 );
    result->distance_m = lr11xx_rttof_distance_raw_to_meter( ranging_bw, buf );

    // Get RSSI
    rc = lr11xx_rttof_get_raw_result( context, LR11XX_RTTOF_RESULT_TYPE_RSSI, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->rssi = lr11xx_rttof_rssi_raw_to_value( buf );

    return rc;
}

static void ranging_handle_distance_result( void )
{
    float   median;
    int32_t sort_distance_rng_results[RANGING_HOPPING_CHANNELS_MAX];

    // copy the distance results
    for( uint16_t k = 0; k < ranging_results.rng_result_index; k++ )
    {
        sort_distance_rng_results[k] = ranging_results.distance_rng_results[k];
    }

    if( ranging_results.rng_result_index > 0 )
    {
        // Sort ranging results
        for( uint16_t i = ranging_results.rng_result_index - 1; i > 0; --i )
        {
            for( uint16_t j = 0; j < i; ++j )
            {
                if( sort_distance_rng_results[j] > sort_distance_rng_results[j + 1] )
                {
                    int32_t temp                     = sort_distance_rng_results[j];
                    sort_distance_rng_results[j]     = sort_distance_rng_results[j + 1];
                    sort_distance_rng_results[j + 1] = temp;
                }
            }
        }

        // Get the median result as the final result
        if( ( ranging_results.rng_result_index % 2 ) == 0 )
        {
            median = ( float ) ( sort_distance_rng_results[ranging_results.rng_result_index / 2] +
                                 sort_distance_rng_results[( ranging_results.rng_result_index / 2 ) - 1] ) /
                     2.0;
        }
        else
        {
            median = sort_distance_rng_results[ranging_results.rng_result_index / 2];
        }

        if( ranging_results.rng_result_index < RANGING_HOPPING_CHANNELS_MIN )
        {
            ranging_settings.rng_status = RANGING_STATUS_PER_ERROR;
        }
        else
        {
            ranging_settings.rng_status = RANGING_STATUS_VALID;
        }

        ranging_results.rng_distance = median;
    }
}

static float get_single_symbol_time_ms( lr11xx_radio_lora_bw_t bw, lr11xx_radio_lora_sf_t sf )
{
    float bw_value;

    if( sf < LR11XX_RADIO_LORA_SF5 || sf > LR11XX_RADIO_LORA_SF12 )
    {
        HAL_PERF_TEST_TRACE_PRINTF( "ERROR: SF value is wrong! \r\n" );
        return 0;
    }

    switch( bw )
    {
    case LR11XX_RADIO_LORA_BW_125:
        bw_value = 125;
        break;
    case LR11XX_RADIO_LORA_BW_250:
        bw_value = 250;
        break;
    case LR11XX_RADIO_LORA_BW_500:
        bw_value = 500;
        break;
    default:
        HAL_PERF_TEST_TRACE_PRINTF( "ERROR: BW value is wrong! \r\n" );
        return 0;
    }

    return ( ( float ) ( 1 << sf ) / bw_value );
}

/* --- EOF ------------------------------------------------------------------ */
