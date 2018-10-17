/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
  @defgroup dtm_standalone main.c
  @{
  @ingroup ble_sdk_app_dtm_serial
  @brief Stand-alone DTM application for UART interface.

 */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "ble_dtm.h"
#include "boards.h"
#include "app_uart.h"

#include "ft6206.h"
#include "ugui.h"

#define GUI 1

UG_WINDOW window_1;
char m_summary_string1[64];
char m_summary_string2[64];

extern const nrf_lcd_t nrf_lcd_ili9341;
static const nrf_lcd_t * p_lcd = &nrf_lcd_ili9341;

#define DTM_SINGLE_CHAN_TEST_DEFAULT_FREQUENCY 1
#define DTM_SINGLE_CHAN_TEST_DEFAULT_LENGTH 10 /* bytes */

// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte.
 * As the time is only known when a byte is received, then the time between between stop bit 1st
 * byte and stop bit 2nd byte becomes:
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration):
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations.
 *
 * This is rounded down to 21.
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

// Error handler for UART
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief Function for UART initialization.
 */
static void uart_init(void)
{   
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          DTM_BITRATE
      };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       0,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
 *
 * @param[in]   command   The packed UART command.
 * @return      result status from dtmlib.
 */
static uint32_t dtm_cmd_put(uint16_t command)
{
    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;

    return dtm_cmd(command_code, freq, length, payload);
}

/*
static uint32_t dtm_tx_run_single_channel_test(dtm_cmd_t cmd, dtm_freq_t freq, uint32_t length, dtm_pkt_type_t payload)
{
    uint32_t ret_code;
    dtm_event_t result;
    bool ret;

    ret_code = dtm_cmd(LE_TEST_SETUP, 0, 0, DTM_PKT_PRBS9);
    if (ret_code != DTM_SUCCESS)
      return ret_code;
    ret = dtm_event_get(&result);
    ret_code = dtm_cmd(cmd, freq, length, payload);
    if (ret_code != DTM_SUCCESS)
      return ret_code;
    ret = dtm_event_get(&result);
    return DTM_SUCCESS;
}

static uint32_t dtm_rx_run_single_channel_test(dtm_cmd_t cmd, dtm_freq_t freq, uint32_t length, dtm_pkt_type_t payload)
{
    uint32_t ret_code;
    dtm_event_t result;
    bool ret;

    ret_code = dtm_cmd(LE_TEST_SETUP, 0, 0, DTM_PKT_PRBS9);
    if (ret_code != DTM_SUCCESS)
      return ret_code; 
    ret = dtm_event_get(&result);
    ret_code = dtm_cmd(cmd, freq, length, payload);
    if (ret_code != DTM_SUCCESS)
      return ret_code;
    ret = dtm_event_get(&result);
    return DTM_SUCCESS;
}
*/

static uint32_t dtm_send_command_to_fifo(dtm_cmd_t cmd, dtm_freq_t freq, uint32_t length, dtm_pkt_type_t payload)
{
    uint32_t data1 = 0;
    uint32_t data2 = 0;
    uint32_t command = 0;

    data1 = data1 + (cmd << 6);
    data2 = (data2 << 2) + payload;
    
    command = data1 + data2;
    app_uart_put((uint8_t)((command & 0x000000FF) >>  0));
    app_uart_put((uint8_t)((command & 0x0000FF00) >>  8));
}

void window_1_callback ( UG_MESSAGE* msg )
{
    static bool BTN_ID_0_tx = true;

    if ( msg->type == MSG_TYPE_OBJECT ) {
      if (msg->id == OBJ_TYPE_BUTTON) {
        if ((msg->event != OBJ_EVENT_PRERENDER) && (msg->event != OBJ_EVENT_POSTRENDER && (msg->event == OBJ_EVENT_RELEASED))) {
          switch( msg->sub_id ) {
            case BTN_ID_0 : {   
              BTN_ID_0_tx = !BTN_ID_0_tx;
              if (!BTN_ID_0_tx) {
                UG_ButtonSetText(&window_1, BTN_ID_0, "RX");                
                UG_ButtonSetText(&window_1, BTN_ID_1, "Start RX"); 
                UG_ButtonHide(&window_1, BTN_ID_2);
                UG_TextboxSetText(&window_1 , TXB_ID_1 , "Select RX Test:") ;
              }
              else {
                UG_ButtonSetText(&window_1, BTN_ID_0, "TX"); 
                UG_ButtonSetText(&window_1, BTN_ID_1, "TX 1-channel Test");                 
                UG_ButtonShow(&window_1, BTN_ID_2);
                UG_TextboxSetText(&window_1 , TXB_ID_1 , "Select TX Test:") ;
              }
              break ;
            }
            case BTN_ID_1 : {             
              if (BTN_ID_0_tx) {
                (void)dtm_send_command_to_fifo(LE_TRANSMITTER_TEST, DTM_SINGLE_CHAN_TEST_DEFAULT_FREQUENCY, DTM_SINGLE_CHAN_TEST_DEFAULT_LENGTH, DTM_PKT_PRBS9);                
              }
              else {
                (void)dtm_send_command_to_fifo(LE_RECEIVER_TEST, DTM_SINGLE_CHAN_TEST_DEFAULT_FREQUENCY, DTM_SINGLE_CHAN_TEST_DEFAULT_LENGTH, DTM_PKT_PRBS9);                
              }
              break;
            }
            default :
            {
                // . . .
                break ;
            }
         } 
       }
     }
    }
}


UG_GUI gui;
#define MAX_OBJECTS 10
UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];
UG_TEXTBOX textbox_select_role;
UG_TEXTBOX textbox_select_tx_test;
UG_BUTTON button_role;
UG_BUTTON button_tx_one_chan_test;
UG_BUTTON button_tx_many_chan_test;

/**@brief Function for application main entry.
 *
 * @details This function serves as an adaptation layer between a 2-wire UART interface and the
 *          dtmlib. After initialization, DTM commands submitted through the UART are forwarded to
 *          dtmlib and events (i.e. results from the command) is reported back through the UART.
 */
int main(void)
{
    uint32_t    current_time;
    uint32_t    dtm_error_code;
    uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    bool        is_msb_read       = false; // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;                   // Last byte read from UART.
    dtm_event_t result;                    // Result of a DTM operation.

    bsp_board_init(BSP_INIT_LEDS);

    uart_init();

    dtm_error_code = dtm_init();
    if (dtm_error_code != DTM_SUCCESS)
    {
        // If DTM cannot be correctly initialized, then we just return.
        return -1;
    }

    init_ft6206((void *)&UG_TouchUpdate, (void *)&UG_Update);

#define TXT_ID_0_X_LOCATION 15
#define TXT_ID_0_Y_LOCATION 10
#define TXT_ID_0_WIDTH 200
#define TXT_ID_0_HEIGHT 25

#define INTERWIDGET_SPACE 10

#define BTN_ID_0_X_LOCATION TXT_ID_0_X_LOCATION
#define BTN_ID_0_Y_LOCATION TXT_ID_0_Y_LOCATION+TXT_ID_0_HEIGHT+INTERWIDGET_SPACE-5
#define BTN_ID_0_WIDTH TXT_ID_0_WIDTH
#define BTN_ID_0_HEIGHT 30

#define TXT_ID_1_X_LOCATION TXT_ID_0_X_LOCATION
#define TXT_ID_1_Y_LOCATION BTN_ID_0_Y_LOCATION+BTN_ID_0_HEIGHT+INTERWIDGET_SPACE*2
#define TXT_ID_1_WIDTH TXT_ID_0_WIDTH
#define TXT_ID_1_HEIGHT TXT_ID_0_HEIGHT

#define BTN_ID_1_X_LOCATION TXT_ID_1_X_LOCATION
#define BTN_ID_1_Y_LOCATION TXT_ID_1_Y_LOCATION+TXT_ID_1_HEIGHT+INTERWIDGET_SPACE-5
#define BTN_ID_1_WIDTH TXT_ID_1_WIDTH
#define BTN_ID_1_HEIGHT BTN_ID_0_HEIGHT

#define BTN_ID_2_X_LOCATION BTN_ID_1_X_LOCATION
#define BTN_ID_2_Y_LOCATION BTN_ID_1_Y_LOCATION+BTN_ID_1_HEIGHT+INTERWIDGET_SPACE-5
#define BTN_ID_2_WIDTH BTN_ID_1_WIDTH
#define BTN_ID_2_HEIGHT BTN_ID_1_HEIGHT

    UG_Init(&gui, 240, 320, p_lcd);

    /* Create the window */
    UG_WindowCreate ( &window_1 , obj_buff_wnd_1 , MAX_OBJECTS, window_1_callback ) ;
    /* Modify the window t i t l e */
    UG_WindowSetTitleText (&window_1 , "Nordic DTM Test") ;
    UG_WindowSetTitleTextFont (&window_1 , &FONT_10X16) ;
    UG_WindowSetTitleTextAlignment(&window_1, ALIGN_CENTER);

    /* Create "Select role" textbox (TXB_ID_0) */
    UG_TextboxCreate(&window_1, &textbox_select_role, TXB_ID_0, TXT_ID_0_X_LOCATION, TXT_ID_0_Y_LOCATION, TXT_ID_0_X_LOCATION+TXT_ID_0_WIDTH, TXT_ID_0_Y_LOCATION+TXT_ID_0_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_0, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_0 , "Select role:") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_0 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_0 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_0 , ALIGN_CENTER );

    /* Create "Role" selection button (BTN_ID_0) */
    UG_ButtonCreate(&window_1, &button_role, BTN_ID_0, BTN_ID_0_X_LOCATION, BTN_ID_0_Y_LOCATION, BTN_ID_0_X_LOCATION+BTN_ID_0_WIDTH, BTN_ID_0_Y_LOCATION+BTN_ID_0_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_0, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_0, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_0, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_0, &FONT_10X16);
    UG_ButtonSetText(&window_1, BTN_ID_0, "TX");    

    /* Create "Select TX test" textbox (TXB_ID_1) */
    UG_TextboxCreate(&window_1, &textbox_select_tx_test, TXB_ID_1, TXT_ID_1_X_LOCATION, TXT_ID_1_Y_LOCATION, TXT_ID_1_X_LOCATION+TXT_ID_1_WIDTH, TXT_ID_1_Y_LOCATION+TXT_ID_1_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_1, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_1 , "Select TX Test:") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_1 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_1 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_1 , ALIGN_CENTER );

    /* Create "1 channel TX test" selection button (BTN_ID_1) */
    UG_ButtonCreate(&window_1, &button_tx_one_chan_test, BTN_ID_1, BTN_ID_1_X_LOCATION, BTN_ID_1_Y_LOCATION, BTN_ID_1_X_LOCATION+BTN_ID_1_WIDTH, BTN_ID_1_Y_LOCATION+BTN_ID_1_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_1, BTN_STYLE_2D|BTN_STYLE_USE_ALTERNATE_COLORS|BTN_STYLE_NO_BORDERS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_1, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_1, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_1, &FONT_8X12);
    UG_ButtonSetText(&window_1, BTN_ID_1, "TX 1-channel Test"); 
           
    /* Create "Many channels TX test" selection button (BTN_ID_2) */
    UG_ButtonCreate(&window_1, &button_tx_many_chan_test, BTN_ID_2, BTN_ID_2_X_LOCATION, BTN_ID_2_Y_LOCATION, BTN_ID_2_X_LOCATION+BTN_ID_2_WIDTH, BTN_ID_2_Y_LOCATION+BTN_ID_2_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_2, BTN_STYLE_2D|BTN_STYLE_USE_ALTERNATE_COLORS|BTN_STYLE_NO_BORDERS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_2, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_2, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_2, &FONT_8X12);
    UG_ButtonSetText(&window_1, BTN_ID_2, "TX n-channel Test"); 
    
     /* Finally , show the window */
    UG_WindowShow( &window_1 ) ;

    for (;;)
    {         
        // Will return every timeout, 625 us.
        current_time = dtm_wait();

        if (app_uart_get(&rx_byte) != NRF_SUCCESS)
        {
            // Nothing read from the UART.
            continue;
        }


        if (!is_msb_read)
        {
            // This is first byte of two-byte command.
            is_msb_read       = true;
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;

            // Go back and wait for 2nd byte of command word.
            continue;
        }

        // This is the second byte read; combine it with the first and process command
        if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        {
            // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
            // The variable is_msb_read will remains true.
            // Go back and wait for 2nd byte of the command word.
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;
            continue;
        }

        // 2-byte UART command received.
        is_msb_read        = false;
        dtm_cmd_from_uart |= (dtm_cmd_t)rx_byte;

        if (dtm_cmd_put(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here.
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result))
        {
/*
            // Report command status on the UART.
            // Transmit MSB of the result.
            while (app_uart_put((result >> 8) & 0xFF));
            // Transmit LSB of the result.
            while (app_uart_put(result & 0xFF));
*/
            if (result != DTM_SUCCESS)
              APP_ERROR_CHECK(2);
        }
    }
}

/// @}
