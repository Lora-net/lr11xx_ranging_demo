/**
 * @file      app_software_timer.h
 *
 * @brief     Design software timer based on system tick
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef APP_SOFTWARE_TIMER_H
#define APP_SOFTWARE_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef void ( *app_soft_timer_callback )( void );

typedef struct
{
    uint32_t                start_time;
    uint32_t                duration;
    bool                    is_running;
    bool                    is_periodic;
    app_soft_timer_callback callback;
} app_soft_timer_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initiate the system tick.
 */
void app_system_ticks_init( void );

/*!
 * @brief Initiate a software timer.
 *
 * @param [in]  timer The point to create a new software timer.
 * @param [in]  callback Set a callback function for this timer.
 */
void app_soft_timer_init( app_soft_timer_t* timer, app_soft_timer_callback callback );

/*!
 * @brief Start a software timer.
 *
 * @param [in]  timer The point to set a new software timer.
 * @param [in]  duration_ms The duration in ms of the timer.
 * @param [in]  is_periodic Whether this timer is a periodic or single one. true: periodic; flase: single.
 */
void app_soft_timer_start( app_soft_timer_t* timer, uint32_t duration_ms, bool is_periodic );

/*!
 * @brief Stop a software timer.
 *
 * @param [in]  timer The point to a software timer.
 */
void app_soft_timer_stop( app_soft_timer_t* timer );

/*!
 * @brief Check if the timer is expired.
 *
 * @param [in]  timer The point to a software timer.
 *
 * @return true: expired; false: not expired.
 */
bool app_soft_timer_is_expired( app_soft_timer_t* timer );

/*!
 * @brief Get the system tick timestamp.
 *
 * @return The current the tick timestamp. Uint: ms.
 */
uint32_t app_system_tick_get_ms( void );

#ifdef __cplusplus
}
#endif

#endif  // APP_SOFTWARE_TIMER_H

/* --- EOF ------------------------------------------------------------------ */