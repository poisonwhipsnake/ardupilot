/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include "WheelEncoder_AS5047P.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


void AP_WheelEncoder_AS5047P::update(void)
{


}

/*
bool AP_WheelEncoder_AS5047P::read_encoder_value(uint16_t &encoder_value)
{
    // read the encoder value
    uint8_t data[2];
    if (!read_registers(AS5047P_REG_ANGLE, data, 2)) {
        return false;
    }

    // convert to 16 bit value
    encoder_value = (data[0] << 8) | data[1];

    return true;
}

// Read registers method to read the position of an AS5047P encoder connected by SPI

bool AP_WheelEncoder_AS5047P::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    // send the register address
    uint8_t tx_data[2] = { (reg << 1) | 0x40, 0 };
    uint8_t rx_data[2] = { 0, 0 };
    if (!hal.spi_transfer(_spi, tx_data, rx_data, 2)) {
        return false;
    }

    // read the data
    tx_data[0] = 0;
    if (!hal.spi_transfer(_spi, tx_data, data, len)) {
        return false;
    }

    return true;
   
}
*/