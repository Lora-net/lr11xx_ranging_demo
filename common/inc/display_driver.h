/**
 * @file      display_driver.h
 *
 * @brief     Display driver
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

#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define DISPLAY_WHITE 0xFFFF
#define DISPLAY_BLACK 0x0000
#define DISPLAY_BLUE 0x001F
#define DISPLAY_RED 0xF800
#define DISPLAY_GREEN 0x07E0
#define DISPLAY_YELLOW 0xFFE0
#define DISPLAY_BROWN 0XBC40
#define DISPLAY_GRAY 0X8430

#define DISPLAY_BACKGROUND DISPLAY_BLACK

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize the DM-TFT28-105 display
 *
 * @param [in] context Radio implementation parameters
 */
void display_init( const void* context );

/*!
 * @brief  Display string with format and no fixed parameters
 *
 * @param [in] x Starting x coordinate
 * @param [in] y Starting y coordinate
 * @param [in] color String's color
 * @param [in] format The point to string's format
 */
void display_string_printf( uint16_t x, uint16_t y, uint16_t color, const char* format, ... );

/*!
 * @brief Fill the specified section on the display
 *
 * @param [in] x_start Starting x coordinate
 * @param [in] y_start Starting y coordinate
 * @param [in] x_end Ending x coordinate
 * @param [in] y_end Ending y coordinate
 * @param [in] color Fill the color of the section
 */
void display_section_fill( uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color );

#ifdef __cplusplus
}
#endif

#endif  // DISPLAY_DRIVER_H

/* --- EOF ------------------------------------------------------------------ */
