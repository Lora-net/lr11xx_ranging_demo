/*!
 * @file      display_driver.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

#include "display_driver.h"
#include "smtc_hal_mcu_gpio.h"
#include "smtc_hal_mcu_gpio_stm32l4.h"
#include "lr11xx_hal_context.h"
#include "stm32l4xx_ll_utils.h"
#include "smtc_shield_pinout_mapping.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/*!
 * @brief The font size is 8*16
 */
#define FONT_CHAR_WIDTH 8
#define FONT_CHAR_HEIGHT 16

/*!
 * @brief The size of the display is 240*320
 */
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 320

/*!
 * @brief To save memory, display up to two lines at a time
 */
#define DISPLAY_MAX_CHAR_ONCE ( 2 * ( DISPLAY_WIDTH / FONT_CHAR_WIDTH ) )

extern uint8_t font_1608[];
#define read_font_line( __char, __line ) font_1608[( ( uint16_t )( __char ) ) * FONT_CHAR_HEIGHT + ( __line )]

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} display_dc;

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} display_nss;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static const lr11xx_hal_context_t* lr11xx_context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void display_gpio_init( void );
static void display_send_command( const uint8_t command );
static void display_send_data_8bit( uint8_t data );
static void display_send_data_16bit( const uint16_t data );
static void display_set_address( uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end );

/*!
 * @brief Draw a char on the display
 *
 * @param [in] x Starting x coordinate
 * @param [in] y Starting y coordinate
 * @param [in] ch Character to display
 * @param [in] color Character's color
 */
static void display_draw_char( uint16_t x, uint16_t y, char ch, uint16_t color );

/*!
 * @brief Draw a string on the display
 *
 * @param [in] x Starting x coordinate
 * @param [in] y Starting y coordinate
 * @param [in] p The pointer to the string to display
 * @param [in] color String's color
 */
static void display_draw_string( uint16_t x, uint16_t y, const char* p, uint16_t color );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void display_init( const void* context )
{
    lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    display_gpio_init( );
    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );

    // ILI9341 init
    display_send_command( 0x11 );

    display_send_command( 0xCF );
    display_send_data_8bit( 0x00 );
    display_send_data_8bit( 0xc3 );
    display_send_data_8bit( 0X30 );

    display_send_command( 0xED );
    display_send_data_8bit( 0x64 );
    display_send_data_8bit( 0x03 );
    display_send_data_8bit( 0X12 );
    display_send_data_8bit( 0X81 );

    display_send_command( 0xE8 );
    display_send_data_8bit( 0x85 );
    display_send_data_8bit( 0x10 );
    display_send_data_8bit( 0x79 );

    display_send_command( 0xCB );
    display_send_data_8bit( 0x39 );
    display_send_data_8bit( 0x2C );
    display_send_data_8bit( 0x00 );
    display_send_data_8bit( 0x34 );
    display_send_data_8bit( 0x02 );

    display_send_command( 0xF7 );
    display_send_data_8bit( 0x20 );

    display_send_command( 0xEA );
    display_send_data_8bit( 0x00 );
    display_send_data_8bit( 0x00 );

    display_send_command( 0xC0 );    // Power control
    display_send_data_8bit( 0x22 );  // VRH[5:0]

    display_send_command( 0xC1 );    // Power control
    display_send_data_8bit( 0x11 );  // SAP[2:0];BT[3:0]

    display_send_command( 0xC5 );  // VCM control
    display_send_data_8bit( 0x3d );
    display_send_data_8bit( 0x20 );

    display_send_command( 0xC7 );    // VCM control2
    display_send_data_8bit( 0xAA );  // 0xB0

    display_send_command( 0x36 );    // Memory Access Control
    display_send_data_8bit( 0xC8 );  // display_send_data_8bit(0x08); if not inverted

    display_send_command( 0x3A );
    display_send_data_8bit( 0x55 );

    display_send_command( 0xB1 );
    display_send_data_8bit( 0x00 );
    display_send_data_8bit( 0x13 );

    display_send_command( 0xB6 );  // Display Function Control
    display_send_data_8bit( 0x0A );
    display_send_data_8bit( 0xA2 );

    display_send_command( 0xF6 );
    display_send_data_8bit( 0x01 );
    display_send_data_8bit( 0x30 );

    display_send_command( 0xF2 );  // 3Gamma Function Disable
    display_send_data_8bit( 0x00 );

    display_send_command( 0x26 );  // Gamma curve selected
    display_send_data_8bit( 0x01 );

    display_send_command( 0xE0 );  // Set Gamma
    display_send_data_8bit( 0x0F );
    display_send_data_8bit( 0x3F );
    display_send_data_8bit( 0x2F );
    display_send_data_8bit( 0x0C );
    display_send_data_8bit( 0x10 );
    display_send_data_8bit( 0x0A );
    display_send_data_8bit( 0x53 );
    display_send_data_8bit( 0XD5 );
    display_send_data_8bit( 0x40 );
    display_send_data_8bit( 0x0A );
    display_send_data_8bit( 0x13 );
    display_send_data_8bit( 0x03 );
    display_send_data_8bit( 0x08 );
    display_send_data_8bit( 0x03 );
    display_send_data_8bit( 0x00 );

    display_send_command( 0XE1 );  // Set Gamma
    display_send_data_8bit( 0x00 );
    display_send_data_8bit( 0x00 );

    display_send_data_8bit( 0x10 );
    display_send_data_8bit( 0x03 );
    display_send_data_8bit( 0x0F );
    display_send_data_8bit( 0x05 );
    display_send_data_8bit( 0x2C );
    display_send_data_8bit( 0xA2 );
    display_send_data_8bit( 0x3F );
    display_send_data_8bit( 0x05 );
    display_send_data_8bit( 0x0E );
    display_send_data_8bit( 0x0C );
    display_send_data_8bit( 0x37 );
    display_send_data_8bit( 0x3C );
    display_send_data_8bit( 0x0F );

    display_send_command( 0x11 );  // Exit Sleep

    display_send_command( 0x29 );  // Display on    
    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    // Set the background for all the display
    display_section_fill( 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_BACKGROUND );
    LL_mDelay( 5 );
}

void display_string_printf( uint16_t x, uint16_t y, uint16_t color, const char* format, ... )
{
    char    buffer[DISPLAY_MAX_CHAR_ONCE + 1];
    va_list p;

    va_start( p, format );
    vsnprintf( buffer, sizeof( buffer ), format, p );
    va_end( p );
    display_draw_string( x, y, buffer, color );
}

void display_section_fill( uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color )
{
    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );

    display_set_address( x_start, y_start, x_end - 1, y_end - 1 );
    for( uint16_t i = x_start; i < x_end; i++ )
    {
        for( uint16_t j = y_start; j < y_end; j++ )
        {
            display_send_data_16bit( color );
        }
    }
    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void display_gpio_init( void )
{
    display_nss.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_D10 );
    display_nss.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_HIGH;
    display_nss.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

    display_dc.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_D9 );
    display_dc.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_HIGH;
    display_dc.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

    smtc_hal_mcu_gpio_init_output( display_nss.cfg, &( display_nss.cfg_output ), &( display_nss.inst ) );
    smtc_hal_mcu_gpio_init_output( display_dc.cfg, &( display_dc.cfg_output ), &( display_dc.inst ) );
}

static void display_send_command( const uint8_t command )
{
    smtc_hal_mcu_gpio_set_state( display_dc.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &command, NULL, 1 );
    smtc_hal_mcu_gpio_set_state( display_dc.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
}

static void display_send_data_8bit( uint8_t data )
{
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &data, NULL, 1 );
}

static void display_send_data_16bit( const uint16_t data )
{
    uint8_t dh = data >> 8;
    uint8_t dl = data & 0xff;

    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &dh, NULL, 1 );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &dl, NULL, 1 );
}

static void display_set_address( uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end )
{
    display_send_command( 0x2A );
    display_send_data_16bit( x_start );
    display_send_data_16bit( x_end );

    display_send_command( 0x2B );
    display_send_data_16bit( y_start );
    display_send_data_16bit( y_end );

    display_send_command( 0x2C );
}

static void display_draw_char( uint16_t x, uint16_t y, char ch, uint16_t color )
{
    uint8_t temp;
    uint8_t pos, t;

    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );

    if( ( x > ( DISPLAY_WIDTH - FONT_CHAR_WIDTH ) ) || ( y > ( DISPLAY_HEIGHT - FONT_CHAR_HEIGHT ) ) )
    {
        return;
    }

    ch = ch - ' ';
    // Clear background
    display_set_address( x, y, x + FONT_CHAR_WIDTH - 1, y + FONT_CHAR_HEIGHT - 1 );
    for( pos = 0; pos < FONT_CHAR_HEIGHT; pos++ )
    {
        temp = read_font_line( ch, pos );
        for( t = 0; t < FONT_CHAR_WIDTH; t++ )
        {
            if( temp & 0x01 )
            {
                display_send_data_16bit( color );
            }
            else
            {
                display_send_data_16bit( DISPLAY_BACKGROUND );
            }
            temp >>= 1;
        }
        y++;
    }
    smtc_hal_mcu_gpio_set_state( display_nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
}

static void display_draw_string( uint16_t x, uint16_t y, const char* p, uint16_t color )
{
    while( *p != '\0' )
    {
        if( x > ( DISPLAY_WIDTH - FONT_CHAR_WIDTH ) )
        {
            x = 0;
            y += FONT_CHAR_HEIGHT;
        }
        if( y > ( DISPLAY_HEIGHT - FONT_CHAR_HEIGHT ) )
        {
            y = x = 0;
        }
        display_draw_char( x, y, *p, color );
        x += FONT_CHAR_WIDTH;
        p++;
    }
}

/* --- EOF ------------------------------------------------------------------ */
