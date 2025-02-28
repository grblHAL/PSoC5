/*
  i2c.c - An embedded CNC Controller with rs274/ngc (g-code) support

  I2C driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "project.h"
#include "i2c.h"

static uint8 keycode;
keycode_callback_ptr on_keyclick;

static void keyclick_int_handler (void) {

    KeyPadIO_ClearInterrupt();

    on_keyclick(KeyPadIO_Read() != 0);
}

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(cap.started)
        return cap;
    
    I2CPort_Start();
    KeyPadInterrupt_StartEx(keyclick_int_handler); 
    
    cap.started = cap.tx_non_blocking = cap.tx_dma = On;

    return cap;  
}

bool i2c_probe (i2c_address_t i2cAddr)
{
    return true;
}

void I2C_ISR_ExitCallback(void)
{    
    if(I2CPort_mstrStatus & I2CPort_MSTAT_RD_CMPLT) {
        if(KeyPadIO_Read() == 0) // only add keycode when key is still pressed
            keyclick_int_handler();
    }
}

// get single byte - via interrupt
bool i2c_get_keycode (uint_fast16_t i2cAddr, keycode_callback_ptr callback)
{
    if(I2CPort_MasterStatus() != I2CPort_MSTAT_XFER_INP) { // ignore if busy
        keycode = 0;
        on_keyclick = callback;
        I2CPort_MasterReadBuf(i2cAddr, &keycode, 1, I2CPort_MODE_COMPLETE_XFER);
    }
    
    return true;
}