/**
 * @file      app_ranging_hopping.h
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

#ifndef APP_RANGING_HOPPING_H
#define APP_RANGING_HOPPING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#include "lr11xx_hal_context.h"
#include "smtc_shield_lr11xx.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Define min and max ranging channels count
 */
#define RANGING_HOPPING_CHANNELS_MAX 39
#define RANGING_HOPPING_CHANNELS_MIN 10

typedef enum
{
    APP_STATUS_NOT_CONFIGURED = 0,
    APP_STATUS_RUNNING,
    APP_STATUS_TERMINATED,
    APP_STATUS_ERROR,
} app_running_status_t;

/*!
 * @brief Status of ranging
 */
enum ranging_running_status
{
    RANGING_STATUS_INIT = 0,
    RANGING_STATUS_PROCESS,
    RANGING_STATUS_VALID,
    RANGING_STATUS_TIMEOUT,
    RANGING_STATUS_PER_ERROR
};

/*!
 * @brief List of states for radio
 */
enum app_radio_internal_states
{
    APP_RADIO_IDLE = 0,  // nothing to do (or wait a radio interrupt)
    APP_RADIO_RANGING_DONE,
    APP_RADIO_RANGING_TIMEOUT,
    APP_RADIO_RANGING_CONFIG,
    APP_RADIO_RANGING_START,
    APP_RADIO_RX,          // Rx done
    APP_RADIO_TIMEOUT,     // Rx timeout
    APP_RADIO_ERROR,       // Rx error
    APP_RADIO_TX,          // Tx done
    APP_RADIO_TX_TIMEOUT,  // Tx error
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Ranging settings structure of structure
 */
typedef struct
{
    uint32_t frequency;         // Frequency
    int8_t   tx_power;          // TX power
    uint8_t  sf;                // Spreading factor
    uint8_t  bw;                // Bandwidth
    uint8_t  payload_len;       // Payload length
    uint8_t  packet_type;       // Packet type (LORA, Ranging)
    uint8_t  rng_req_count;     // Ranging Request Count
    uint16_t rng_req_delay;     // Time between ranging request
    uint32_t rng_address;       // Ranging Address
    uint8_t  rng_status;        // Status of ranging
    uint32_t rng_exch_timeout;  // Computed global timeout based time on air of the current ranging packet
} ranging_params_settings_t;

typedef struct
{
    uint16_t cnt_packet_rx_ok;        // Rx packet received OK
    uint16_t cnt_packet_rx_ko_slave;  // Rx packet received KO (slave side)
    int      rng_result_index;
    uint8_t  rng_freq_index[RANGING_HOPPING_CHANNELS_MAX];
    int32_t  distance_rng_results[RANGING_HOPPING_CHANNELS_MAX];
    uint32_t raw_rng_results[RANGING_HOPPING_CHANNELS_MAX];
    int8_t   raw_rssi[RANGING_HOPPING_CHANNELS_MAX];
    uint8_t  rng_per;           // Ranging PER
    float    rng_distance;      // Distance measured by ranging
    int8_t   rssi_value;        // RSSI Value
    int8_t   slave_rssi_value;  // Slave Rssi Value
    int8_t   snr_value;         // SNR Value (only for LORA mode type)
} ranging_global_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize some parameters for ranging
 */
void app_radio_ranging_params_init( void );

/*!
 * @brief Setup ranging and configure some parameters
 *
 * @param [in] context  Pointer to the radio context
 */
void app_radio_ranging_setup( const void* context );

/*!
 * @brief Run the ranging demo
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] is_manager  true: manager, false: subordinate
 *
 * @returns  Indicates the status of the demo.
 */
app_running_status_t app_radio_ranging_run( const void* context, const bool is_manager );

/*!
 * @brief Compute the result distance and get the ranging results.
 *
 * @returns Pointer on the results.
 */
ranging_global_result_t* app_ranging_get_result( void );

/*!
 * @brief Get the current radio settings for ranging.
 *
 * @returns The pointer on the radio settings.
 */
ranging_params_settings_t* app_ranging_get_radio_settings( void );

/*!
 * @brief Reset the ranging parameters.
 */
void app_ranging_params_reset( void );

/*!
 * @brief Get the channel value from the predefine list
 *
 * @param [in] index  Index of array
 *
 * @returns  Frequency value
 */
uint32_t get_ranging_hopping_channels( uint8_t index );

/*!
 * @brief Set the current state for ranging process
 *
 * @param [in] state  Current state
 */
void set_ranging_process_state( uint8_t state );

#ifdef __cplusplus
}
#endif

#endif  // APP_RANGING_HOPPING_H

/* --- EOF ------------------------------------------------------------------ */