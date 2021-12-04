/*
  serial.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Serial driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of GrblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "project.h"
#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

static void uart_rx_interrupt_handler (void);

static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static int16_t serialGetC (void)
{
    int16_t data;
    uint32_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)   // If buffer empty
        return SERIAL_NO_DATA;  // return no data available

//    UARTIntDisable(UARTCH, UART_INT_RX|UART_INT_RT);
    data = rxbuffer.data[bptr++];               	// Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  	// and update pointer

//    UARTIntEnable(UARTCH, UART_INT_RX|UART_INT_RT);

//    if (rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)    // Clear RTS if
//        GPIOPinWrite(RTS_PORT, RTS_PIN, rts_state = 0);                             // buffer count is below low water mark

    return data;
}

static bool serialPutC (const char c)
{
    UART_PutChar(c);
    return true;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static inline uint16_t serialRxCount(void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree(void)
{
    return RX_BUFFER_SIZE - serialRxCount();
}

static void serialRxFlush(void)
{
    rxbuffer.tail = rxbuffer.head;
    UART_ClearRxBuffer(); // clear FIFO too
//    GPIOPinWrite(RTS_PORT, RTS_PIN, rts_state = 0);
}

static void serialRxCancel(void)
{
    rxbuffer.data[rxbuffer.head] = CMD_RESET;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
    UART_ClearRxBuffer(); // clear FIFO too

//    GPIOPinWrite(RTS_PORT, RTS_PIN, rts_state = 0);
}

static void serialWriteS(const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static void serialWrite(const char *data, uint16_t length)
{
    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);

}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .write_n = serialWrite,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };
    
    UART_Start();
    UART_RX_Interrupt_StartEx(uart_rx_interrupt_handler);

    return &stream;
}

static void uart_rx_interrupt_handler (void)
{
    uint8_t data;

    while((data = UART_GetChar())) { // UART_GetChar() returns 0 if no data
        if(!enqueue_realtime_command((char)data)) {

            uint32_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)data;      // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }
    } // loop until FIFO empty

//        if (!rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
//            GPIOPinWrite(RTS_PORT, RTS_PIN, rts_state = RTS_PIN);
}
